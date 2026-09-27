"""sensors -- 16 point IR detectors and the demand logic built on them.

Spec: docs/Eight-Movement_Intersection_Study.html §4 (sensor model), §6 (D2
filter, passage time, extended initial), §7 (update_calls pseudocode), §13;
docs/ARCHITECTURE.md R1 (hybrid real/virtual sensors).
Must not: expose true queue counts or vehicle objects to controllers.

Layers
------
    DetectorSource  --raw on/off edges-->  SensorBank  --queries/snapshot-->  controllers

* DetectorSource (interface): the 16 raw detectors. It reports exact,
  timestamped on/off EDGES (integer ms). VirtualDetectors implements it from
  the VehicleModel; a Stage 9 ESP32 source will implement the same interface.
  Stage 8 contains NO serial / ESP32 code.
* SensorBank: the ONLY object controllers use. It turns edges plus the signal
  state into calls, demand levels, Q-hat, the D2 filter, the gap timer and the
  extended-initial flag. Virtual and real detectors share this logic, and
  nothing in it reveals which kind a detector is.

Raw detector (point sensor)
---------------------------
Detector at p metres before the stop line (D1 = 2.0, D2 = 32.0). A vehicle
occupies it from the instant its front reaches p until the instant its rear
passes p (front reaches p - 4.5), i.e. on the half-open interval
[front at p, front at p - 4.5). Queue slot 1 (0.5-5.0 m) covers D1 and slot 5
(30.5-35.0 m) covers D2; slots 2-4 and 6+ cover neither.

Demand lifecycle (per movement, all times integer ms)
-----------------------------------------------------
Signal state decides the mode: WAITING = RED, RED_YELLOW, YELLOW, ALL_RED;
SERVICE = GREEN, FLASHING_GREEN (one continuous service).

WAITING
  * Any occupancy of D1 or D2 latches a call (call_since = that instant) with
    LEVEL_1 (Q-hat 2). The call stays latched when the detectors clear.
  * LEVEL_2 (Q-hat 6) latches when D2 has been continuously occupied for
    >= 2000 ms, counted from the later of D2's rising edge and the start of
    waiting. The count restarts whenever D2 clears. LEVEL_2 never falls back.
GREEN onset (the actual RED_YELLOW -> GREEN instant)
  * extended_initial_required := (LEVEL_2 latched at that instant).
  * The call and level are cleared (the movement is being served).
SERVICE
  * No calls are placed. The gap timer counts time with BOTH detectors clear,
    from the later of the last clearing and GREEN onset; any occupancy resets
    it. gap_out := gap timer >= 2500 ms (not latched).
Service end (FLASHING_GREEN -> YELLOW)
  * Mode returns to WAITING; a detector occupied at that instant places a new
    call immediately. extended_initial_required returns to False.
Simultaneous events at one instant: thresholds reached at or before it are
latched first, then the signal change, then the detector edges.

Stepping
--------
Per simulation step: vehicles.advance(dt, bank); sensors.advance(dt, bank);
bank.advance(dt). SensorBank reads the signal state (and remaining()) at the
start of the step, like the vehicle model; RED_YELLOW -> GREEN and
FLASHING_GREEN -> YELLOW are applied at their exact instants inside the step.
"""
from dataclasses import dataclass
from enum import IntEnum
import heapq
import math
from typing import Dict, List, NamedTuple, Optional, Protocol, Sequence, Tuple

from smarttraffic.conflicts import MOVES, validate_movement
from smarttraffic.signals import SignalState
from smarttraffic.vehicles import VEHICLE_LENGTH, VehicleModel

__all__ = [
    "D1", "D2", "DETECTORS", "DETECTOR_POSITION", "SENSOR_IDS",
    "D2_FILTER_MS", "PASSAGE_TIME_MS", "EXTENDED_INITIAL_GREEN_S",
    "DemandLevel", "QHAT", "DetectorEdge", "DetectorSource", "VirtualDetectors",
    "SensorBank", "MovementSensors", "validate_detector",
]

# ----------------------------------------------------------- study values
D1, D2 = "D1", "D2"
DETECTORS = (D1, D2)
DETECTOR_POSITION = {D1: 2.0, D2: 32.0}          # metres before the stop line (§4)
SENSOR_IDS: Tuple[Tuple[str, str], ...] = tuple((m, d) for m in MOVES for d in DETECTORS)
D2_FILTER_MS = 2000                               # §4: continuous D2 occupancy for level 2
PASSAGE_TIME_MS = 2500                            # §6: passage (gap) time
EXTENDED_INITIAL_GREEN_S = 12.0                   # §6: minimum green when level 2 at onset
MS = 1000

_WAITING, _SERVICE = "WAITING", "SERVICE"
_SERVICE_STATES = (SignalState.GREEN, SignalState.FLASHING_GREEN)


class DemandLevel(IntEnum):
    NONE = 0
    LEVEL_1 = 1
    LEVEL_2 = 2


QHAT = {DemandLevel.NONE: 0, DemandLevel.LEVEL_1: 2, DemandLevel.LEVEL_2: 6}   # §4


def validate_detector(d: object) -> str:
    if d not in DETECTORS:
        raise ValueError(f"detector must be 'D1' or 'D2', got {d!r}")
    return d


class DetectorEdge(NamedTuple):
    """Raw detector change: from time_ms on, (movement, detector) is `occupied`."""
    time_ms: int
    movement: str
    detector: str
    occupied: bool


class DetectorSource(Protocol):
    """What SensorBank needs from any detector implementation (virtual or real).

    sensor_ids          -- the 16 (movement, detector) pairs, all of them.
    now_ms              -- time up to which edges have been delivered.
    initial_occupancy() -- {(m, d): bool} at the source's start time.
    poll(until_ms)      -- edges with now_ms < time_ms <= until_ms, sorted by
                           time, only real changes (on->off or off->on);
                           afterwards now_ms == until_ms.
    """
    sensor_ids: Tuple[Tuple[str, str], ...]

    @property
    def now_ms(self) -> int: ...

    def initial_occupancy(self) -> Dict[Tuple[str, str], bool]: ...

    def poll(self, until_ms: int) -> List[DetectorEdge]: ...


# ------------------------------------------------------ virtual detectors
class VirtualDetectors:
    """DetectorSource computed from a VehicleModel (Stage 8).

    Uses VehicleModel.watch_positions() so every on/off instant is exact to
    the millisecond whatever the step size. Must be created before the
    VehicleModel's first advance().
    """
    sensor_ids = SENSOR_IDS

    def __init__(self, vehicle_model: VehicleModel):
        if not isinstance(vehicle_model, VehicleModel):
            raise TypeError("VirtualDetectors needs a vehicles.VehicleModel")
        self._vm = vehicle_model
        # front reaching p switches the detector on; front reaching p - L switches it off
        self._role: Dict[float, Tuple[str, int]] = {}
        for d, p in DETECTOR_POSITION.items():
            self._role[float(p)] = (d, +1)
            self._role[float(p - VEHICLE_LENGTH)] = (d, -1)
        vehicle_model.watch_positions(*self._role)
        self._index = 0
        self._pending: List[Tuple[int, int, str, str, int]] = []   # heap
        self._count: Dict[Tuple[str, str], int] = {sid: 0 for sid in SENSOR_IDS}
        self._now = vehicle_model.now_ms

    @property
    def now_ms(self) -> int:
        return self._now

    def initial_occupancy(self) -> Dict[Tuple[str, str], bool]:
        return {sid: False for sid in SENSOR_IDS}      # cars enter 150 m upstream

    def poll(self, until_ms: int) -> List[DetectorEdge]:
        if until_ms < self._now:
            raise ValueError(f"cannot poll backwards ({until_ms} < {self._now})")
        if until_ms > self._vm.now_ms:
            raise ValueError(f"vehicle model is only at {self._vm.now_ms} ms; advance it first")
        new, self._index = self._vm.passages_since(self._index)
        for t, vid, m, p in new:
            if p in self._role:
                d, delta = self._role[p]
                heapq.heappush(self._pending, (t, MOVES.index(m), m, d, delta))
        edges: List[DetectorEdge] = []
        while self._pending and self._pending[0][0] <= until_ms:
            t = self._pending[0][0]
            before = {}
            while self._pending and self._pending[0][0] == t:     # all changes at one ms
                _, _, m, d, delta = heapq.heappop(self._pending)
                before.setdefault((m, d), self._count[(m, d)] > 0)
                self._count[(m, d)] += delta
                if self._count[(m, d)] < 0:
                    raise AssertionError(f"negative occupancy count at {(m, d)}")
            for (m, d), was in sorted(before.items(), key=lambda k: (MOVES.index(k[0][0]), k[0][1])):
                now_on = self._count[(m, d)] > 0
                if now_on != was:
                    edges.append(DetectorEdge(t, m, d, now_on))
        self._now = until_ms
        return edges


# ------------------------------------------------------------ demand logic
@dataclass(frozen=True)
class MovementSensors:
    """Snapshot of one movement's sensor information at `time_ms`."""
    movement: str
    time_ms: int
    d1: bool                         # raw occupancy now
    d2: bool
    d2_occupied_ms: int              # raw continuous D2 occupancy now (0 if free)
    d2_filter_ms: Optional[int]      # time counted toward LEVEL_2 (None unless waiting with D2 on)
    has_call: bool
    call_since_ms: Optional[int]
    demand_level: DemandLevel
    qhat: int
    level2_latched: bool
    level2_since_ms: Optional[int]
    in_service: bool                 # GREEN or FLASHING_GREEN
    gap_timer_ms: Optional[int]      # None outside service
    gap_out: bool
    extended_initial_required: bool  # current service began with LEVEL_2 latched
    green_onset_ms: Optional[int]    # GREEN onset of the current service


class _Movement:
    __slots__ = ("on", "on_since", "mode", "call", "call_ms", "level", "level2_ms",
                 "filter_start", "gap_ref", "ext", "green_ms")

    def __init__(self, occ: Dict[str, bool], t: int):
        self.on = dict(occ)
        self.on_since = {d: t for d in DETECTORS}
        self.mode: Optional[str] = None
        self.call = False
        self.call_ms: Optional[int] = None
        self.level = DemandLevel.NONE
        self.level2_ms: Optional[int] = None
        self.filter_start: Optional[int] = None
        self.gap_ref: Optional[int] = None
        self.ext = False
        self.green_ms: Optional[int] = None

    # --- time passes up to t (exclusive of events at t) ---
    def latch_until(self, t: int) -> None:
        if (self.mode == _WAITING and self.filter_start is not None
                and self.level < DemandLevel.LEVEL_2 and self.filter_start + D2_FILTER_MS <= t):
            self.level = DemandLevel.LEVEL_2
            self.level2_ms = self.filter_start + D2_FILTER_MS

    # --- mode changes ---
    def start_waiting(self, t: int) -> None:
        self.mode = _WAITING
        self.ext = False
        self.gap_ref = None
        self.filter_start = t if self.on[D2] else None
        self._call_if_occupied(t)

    def start_service(self, t: int) -> None:
        self.ext = self.level == DemandLevel.LEVEL_2
        self.mode = _SERVICE
        self.green_ms = t
        self.call, self.call_ms = False, None
        self.level, self.level2_ms = DemandLevel.NONE, None
        self.filter_start = None
        self.gap_ref = None if (self.on[D1] or self.on[D2]) else t

    # --- a raw edge ---
    def edge(self, t: int, d: str, occupied: bool) -> None:
        if self.on[d] == occupied:
            return
        self.on[d] = occupied
        self.on_since[d] = t
        if self.mode == _WAITING:
            if d == D2:
                self.filter_start = t if occupied else None
            self._call_if_occupied(t)
        elif self.mode == _SERVICE:
            self.gap_ref = None if (self.on[D1] or self.on[D2]) else t

    def _call_if_occupied(self, t: int) -> None:
        if (self.on[D1] or self.on[D2]) and not self.call:
            self.call, self.call_ms = True, t
            self.level = max(self.level, DemandLevel.LEVEL_1)


class SensorBank:
    """Unified sensor interface for controllers (virtual or real detectors)."""

    def __init__(self, source: DetectorSource):
        ids = tuple(getattr(source, "sensor_ids", ()))
        if set(ids) != set(SENSOR_IDS) or len(ids) != len(SENSOR_IDS):
            raise ValueError("detector source must provide exactly the 16 sensors "
                             "(D1, D2 for each of the 8 movements)")
        self._src = source
        self._now = source.now_ms
        occ = source.initial_occupancy()
        self._mv = {m: _Movement({d: bool(occ[(m, d)]) for d in DETECTORS}, self._now) for m in MOVES}
        self.edge_log: List[DetectorEdge] = []

    # ---------------------------------------------------------------- time
    @property
    def now(self) -> float:
        return self._now / MS

    @property
    def now_ms(self) -> int:
        return self._now

    def advance(self, dt: float, signals) -> None:
        """Process (now, now + dt]. `signals` gives each movement's state at the
        start of the step (state, since, and remaining for RED_YELLOW /
        FLASHING_GREEN), exactly as for VehicleModel.advance."""
        dt_ms = _to_ms(dt, "dt")
        if dt_ms < 0:
            raise ValueError(f"dt must not be negative, got {dt!r}")
        for attr in ("state", "since"):
            if not callable(getattr(signals, attr, None)):
                raise TypeError("signals must provide state(m) and since(m) (e.g. a SignalBank)")
        now_attr = getattr(signals, "now", None)
        if now_attr is not None and round(now_attr * MS) != self._now:
            raise ValueError(f"signals are at t={now_attr} s but sensors are at t={self.now} s; "
                             f"call sensors.advance() before the SignalBank's")
        t0, t1 = self._now, self._now + dt_ms
        per_m: Dict[str, List[Tuple[int, int, object]]] = {m: [] for m in MOVES}
        for m in MOVES:
            st = signals.state(m)
            if not isinstance(st, SignalState):
                raise TypeError(f"signals.state({m!r}) must return a SignalState, got {st!r}")
            mv = self._mv[m]
            mode0 = _SERVICE if st in _SERVICE_STATES else _WAITING
            if mv.mode != mode0:                         # first step, or an external jump
                if mode0 == _SERVICE:
                    mv.start_service(_to_ms(signals.since(m), "since") if st is SignalState.GREEN else t0)
                else:
                    mv.start_waiting(t0)
            if st in (SignalState.RED_YELLOW, SignalState.FLASHING_GREEN):
                if not callable(getattr(signals, "remaining", None)):
                    raise TypeError(f"signals must provide remaining(m) for {st.name}")
                tx = t0 + _to_ms(signals.remaining(m), "remaining")
                if tx <= t1:
                    kind = "service" if st is SignalState.RED_YELLOW else "waiting"
                    per_m[m].append((tx, 0, kind))      # rank 0: signal change before edges
        edges = self._src.poll(t1)
        for e in edges:
            if not (t0 < e.time_ms <= t1):
                raise ValueError(f"source returned an edge outside ({t0}, {t1}]: {e}")
            per_m[e.movement].append((e.time_ms, 1, e))
        self.edge_log.extend(edges)
        for m, events in per_m.items():
            mv = self._mv[m]
            for t, _, ev in sorted(events, key=lambda x: (x[0], x[1])):
                mv.latch_until(t)
                if ev == "service":
                    mv.start_service(t)
                elif ev == "waiting":
                    mv.start_waiting(t)
                else:
                    mv.edge(t, ev.detector, ev.occupied)
            mv.latch_until(t1)
        self._now = t1

    # ------------------------------------------------------------- queries
    def _m(self, m: str) -> _Movement:
        return self._mv[validate_movement(m)]

    def occupied(self, m: str, detector: str) -> bool:
        """Raw occupancy of detector 'D1' or 'D2' of movement m now."""
        return self._m(m).on[validate_detector(detector)]

    def d1(self, m: str) -> bool:
        return self.occupied(m, D1)

    def d2(self, m: str) -> bool:
        return self.occupied(m, D2)

    def d2_occupied_ms(self, m: str) -> int:
        """Raw continuous D2 occupancy up to now (0 if D2 is free)."""
        mv = self._m(m)
        return self._now - mv.on_since[D2] if mv.on[D2] else 0

    def d2_filter_ms(self, m: str) -> Optional[int]:
        """Continuous D2 occupancy counted toward LEVEL_2 (None unless waiting with D2 on)."""
        mv = self._m(m)
        return None if mv.filter_start is None else self._now - mv.filter_start

    def has_call(self, m: str) -> bool:
        return self._m(m).call

    def call_since_ms(self, m: str) -> Optional[int]:
        """When the current call was latched (the study's t_call)."""
        return self._m(m).call_ms

    def demand_level(self, m: str) -> DemandLevel:
        return self._m(m).level

    def qhat(self, m: str) -> int:
        """Estimated queue: 0 (no call), 2 (LEVEL_1) or 6 (LEVEL_2)."""
        return QHAT[self._m(m).level]

    def level2_latched(self, m: str) -> bool:
        return self._m(m).level == DemandLevel.LEVEL_2

    def in_service(self, m: str) -> bool:
        return self._m(m).mode == _SERVICE

    def gap_timer_ms(self, m: str) -> Optional[int]:
        """Time with both detectors clear during service (None outside service)."""
        mv = self._m(m)
        if mv.mode != _SERVICE:
            return None
        return 0 if mv.gap_ref is None else self._now - mv.gap_ref

    def gap_out(self, m: str) -> bool:
        g = self.gap_timer_ms(m)
        return g is not None and g >= PASSAGE_TIME_MS

    def extended_initial_required(self, m: str) -> bool:
        """True while serving a movement whose GREEN began with LEVEL_2 latched."""
        mv = self._m(m)
        return mv.mode == _SERVICE and mv.ext

    def green_onset_ms(self, m: str) -> Optional[int]:
        mv = self._m(m)
        return mv.green_ms if mv.mode == _SERVICE else None

    def status(self, m: str) -> MovementSensors:
        mv = self._m(m)
        return MovementSensors(
            movement=m, time_ms=self._now, d1=mv.on[D1], d2=mv.on[D2],
            d2_occupied_ms=self.d2_occupied_ms(m), d2_filter_ms=self.d2_filter_ms(m),
            has_call=mv.call, call_since_ms=mv.call_ms, demand_level=mv.level,
            qhat=QHAT[mv.level], level2_latched=mv.level == DemandLevel.LEVEL_2,
            level2_since_ms=mv.level2_ms, in_service=mv.mode == _SERVICE,
            gap_timer_ms=self.gap_timer_ms(m), gap_out=self.gap_out(m),
            extended_initial_required=self.extended_initial_required(m),
            green_onset_ms=self.green_onset_ms(m))

    def snapshot(self) -> Dict[str, MovementSensors]:
        """Status of all 8 movements at the current time."""
        return {m: self.status(m) for m in MOVES}


def _to_ms(seconds, what: str) -> int:
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds):
        raise ValueError(f"{what} must be a finite number of seconds, got {seconds!r}")
    ms = round(seconds * MS)
    if abs(seconds * MS - ms) > 1e-6:
        raise ValueError(f"{what} must be a whole number of milliseconds, got {seconds!r}")
    return ms
