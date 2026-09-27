"""vehicles -- vehicle and queue model for the eight approach lanes.

Spec: docs/Eight-Movement_Intersection_Study.html, §1 (geometry), §4 (queue
slot positions), §5 (vehicle model), §13; docs/ARCHITECTURE.md R3.
Responsibility: positions, queue slots, discharge headways, yellow rule,
free-flow crossing time for delay.
Must not: read controller internals, and must not decide signal timing.

Model
-----
A queue-server model with spatial positions (study §5). Each movement is one
lane, and the lanes are independent: they are coupled only through the signal
states supplied from outside.

* STOP-LINE CROSSING TIMES come only from the study's discharge rule:
      D = max(own arrival at stop line,  previous departure + h,  t_g + l1 + h)
  with l1 = 2.0 s and h = 1.9 s (straight) / 2.0 s (left). For a queue that is
  waiting at green onset t_g, this gives the n-th car (n = 1, 2, ...)
  t_g + 2.0 + n*h: 3.9 / 11.5 / 21.0 s for cars 1 / 5 / 10 (straight).
  "Own arrival at stop line" = actual road-entry time + 150 m / 50 km/h
  (= entry + 10.8 s). No travel time is added on top of D for a queued car:
  D already includes it.
* POSITIONS exist only so that sensors can be evaluated and cars drawn. A car
  is either stopped or moving at the free speed v = 50 km/h; no acceleration
  is modelled (study §5: statistics do not depend on it). It must never:
    - be closer than 7.5 m (front to front) to the car ahead in its lane,
    - pass the stop line before its crossing time D, and
    - go past the queue-slot-1 position (front bumper 0.5 m before the stop
      line, study §4) while it has no crossing time.
  A car that has a crossing time drives at v until it has to halt (behind a
  halted car, or at slot 1 if it is early). Once halted it stays where it is
  and starts at the last moment that still lets it reach the stop line at
  exactly D (so a queue starts from the front, one car at a time). A car
  WITHOUT a crossing time always closes up toward the stop line.
* PERMISSION to cross is read from the signal state at the start of each
  advance() step (commands only happen at step boundaries; timed changes are
  known in advance from signals.remaining()):
      RED_YELLOW        -> may not cross; GREEN (t_g) will begin at now + remaining
      GREEN since t_g   -> may cross at any time
      FLASHING_GREEN    -> may cross only if D < (yellow onset = now + remaining) + 2.0 s
      YELLOW since t_y  -> may cross only if D < t_y + 2.0 s (study §5, HCM e)
      ALL_RED, RED      -> may not cross
  t_g in the discharge rule is always the instant GREEN itself begins.
  Exact for any dt up to 3.9 s (no car can cross earlier than t_g + 3.9 s).
  As soon as the window end is known (at FLASHING_GREEN or YELLOW), cars whose
  D is at or beyond it lose their crossing time and stop.
* ENTRY (R3): a car appears with its front bumper 150 m upstream, but only if
  the car ahead has its front at <= 142.5 m (7.5 m spacing). Otherwise it
  waits outside the modelled road, first come first served, and enters at
  the first instant there is room. Its scheduled time is never changed; delay
  is measured from it.

Time: integer milliseconds (same clock as signals.py and traffic_gen.py). All
events (entries, crossings) are applied at their exact millisecond, however
large the step, so results do not depend on dt.
"""
from collections import deque
from enum import Enum
import math
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple, Union

from smarttraffic.conflicts import MOVES, validate_movement
from smarttraffic.signals import SignalState, movement_type
from smarttraffic.traffic_gen import Arrival, ArrivalSchedule

__all__ = [
    "VehicleModel", "Vehicle", "VehicleState", "ModelInvariantError",
    "FREE_SPEED", "VEHICLE_LENGTH", "STANDSTILL_GAP", "QUEUE_SPACING",
    "ENTRY_POSITION", "SLOT1_POSITION", "START_UP_LOST_TIME", "SATURATION_HEADWAY",
    "YELLOW_EXTENSION", "BOX_LENGTH", "BOX_SPEED", "FREE_TRAVEL_TIME", "slot_position",
]

# ------------------------------------------------------------ study values
FREE_SPEED = 50 / 3.6                    # m/s (study: 50 km/h, shown rounded as 13.9 m/s)
VEHICLE_LENGTH = 4.5                     # m
STANDSTILL_GAP = 3.0                     # m
QUEUE_SPACING = VEHICLE_LENGTH + STANDSTILL_GAP     # 7.5 m, front to front
ENTRY_POSITION = 150.0                   # m upstream of the stop line
SLOT1_POSITION = 0.5                     # front bumper of queued car #1 (§4: occupies 0.5-5.0 m)
START_UP_LOST_TIME = 2.0                 # l1, s
SATURATION_HEADWAY = {"straight": 1.9, "left": 2.0}  # h, s
YELLOW_EXTENSION = 2.0                   # e, s: crossing allowed in first 2.0 s of yellow
BOX_LENGTH = {"straight": 20.0, "left": 25.0}         # m, path through the junction (§1)
BOX_SPEED = 9.0                          # m/s, visual only (study: "about 8-10 m/s")

MS = 1000
_EPS = 1e-9
_L1_MS = 2000
_H_MS = {"straight": 1900, "left": 2000}
_YEXT_MS = 2000
FREE_TRAVEL_TIME = ENTRY_POSITION / FREE_SPEED       # 10.8 s
_FREE_TRAVEL_MS = round(FREE_TRAVEL_TIME * MS)
assert abs(FREE_TRAVEL_TIME * MS - _FREE_TRAVEL_MS) < 1e-6
_BOX_MS = {t: round(BOX_LENGTH[t] / BOX_SPEED * MS) for t in BOX_LENGTH}
_V_PER_MS = FREE_SPEED / MS                          # m per ms
_ENTRY_ROOM = ENTRY_POSITION - QUEUE_SPACING         # 142.5 m


def slot_position(k: int) -> float:
    """Front-bumper position of stopped queue slot k (k = 1 is at the stop line)."""
    if not isinstance(k, int) or isinstance(k, bool) or k < 1:
        raise ValueError(f"slot must be an int >= 1, got {k!r}")
    return SLOT1_POSITION + QUEUE_SPACING * (k - 1)


class ModelInvariantError(AssertionError):
    """An internal physical invariant was violated (a model bug, never expected)."""


class VehicleState(Enum):
    SCHEDULED = "SCHEDULED"              # arrival time not reached yet
    WAITING_OUTSIDE = "WAITING_OUTSIDE"  # R3: scheduled, but no room at the entry yet
    MOVING = "MOVING"                    # on the approach, moving at free speed
    STOPPED = "STOPPED"                  # on the approach, stationary
    IN_INTERSECTION = "IN_INTERSECTION"  # crossed the stop line, inside the box
    EXITED = "EXITED"                    # left the box


class Vehicle:
    """One vehicle. Read-only to everything except VehicleModel.

    Times are exposed both in ms (int, exact) and in seconds (float).
    position: metres of the FRONT bumper upstream of the stop line (> 0 on the
    approach); inside the box it is negative (metres travelled into the box);
    None before entry. The body occupies [position, position + 4.5].
    """
    __slots__ = ("_arr", "_state", "_x", "_entry_ms", "_cross_plan_ms", "_released_ms",
                 "_cross_ms", "_exit_ms", "_queue_join_ms", "_stopped_s", "_pinned", "_passages")

    def __init__(self, arrival: Arrival):
        self._arr = arrival
        self._state = VehicleState.SCHEDULED
        self._x: Optional[float] = None
        self._entry_ms: Optional[int] = None
        self._cross_plan_ms: Optional[int] = None
        self._released_ms: Optional[int] = None
        self._cross_ms: Optional[int] = None
        self._exit_ms: Optional[int] = None
        self._queue_join_ms: Optional[int] = None
        self._stopped_s = 0.0
        self._pinned = False          # released and halted: waits for its latest start
        self._passages: Dict[float, int] = {}   # watched position -> first ms front <= it

    # identity (from traffic_gen, immutable)
    id = property(lambda s: s._arr.id)
    movement = property(lambda s: s._arr.movement)
    seq = property(lambda s: s._arr.seq)
    source = property(lambda s: s._arr.source)
    arrival = property(lambda s: s._arr)
    scheduled_ms = property(lambda s: s._arr.time_ms)
    scheduled_time = property(lambda s: s._arr.time_ms / MS)
    # dynamic
    state = property(lambda s: s._state)
    position = property(lambda s: s._x)
    entry_ms = property(lambda s: s._entry_ms)
    entry_time = property(lambda s: None if s._entry_ms is None else s._entry_ms / MS)
    planned_cross_ms = property(lambda s: s._cross_plan_ms)
    released_ms = property(lambda s: s._released_ms)
    cross_ms = property(lambda s: s._cross_ms)
    cross_time = property(lambda s: None if s._cross_ms is None else s._cross_ms / MS)
    exit_ms = property(lambda s: s._exit_ms)
    exit_time = property(lambda s: None if s._exit_ms is None else s._exit_ms / MS)
    queue_join_ms = property(lambda s: s._queue_join_ms)

    def passage_ms(self, position: float) -> Optional[int]:
        """First ms at which the FRONT bumper was at or past `position` (a watched
        position, see VehicleModel.watch_positions); None if not reached yet."""
        if isinstance(position, bool) or not isinstance(position, (int, float)):
            raise TypeError(f"position must be a number, got {position!r}")
        return self._passages.get(float(position))
    stopped_time = property(lambda s: s._stopped_s)

    @property
    def free_flow_cross_ms(self) -> int:
        """When it would have crossed at free speed with no signal and no entry wait
        (scheduled arrival + 10.8 s). Control delay = cross_ms - this."""
        return self._arr.time_ms + _FREE_TRAVEL_MS

    @property
    def entry_wait_ms(self) -> Optional[int]:
        return None if self._entry_ms is None else self._entry_ms - self._arr.time_ms

    def __repr__(self):
        return (f"Vehicle(id={self.id}, {self.movement}#{self.seq}, {self._state.name}, "
                f"x={self._x}, sched={self.scheduled_time}, entry={self.entry_time}, "
                f"cross={self.cross_time})")


# ------------------------------------------------------------------ lane
class _Lane:
    def __init__(self, m: str, arrivals: List[Arrival]):
        self.m = m
        self.kind = movement_type(m)
        self.h_ms = _H_MS[self.kind]
        self.box_ms = _BOX_MS[self.kind]
        self.box_len = BOX_LENGTH[self.kind]
        self.pending: Deque[Vehicle] = deque(Vehicle(a) for a in arrivals)
        self.waiting: Deque[Vehicle] = deque()
        self.road: List[Vehicle] = []          # front (nearest stop line) first
        self.box: List[Vehicle] = []
        self.done: List[Vehicle] = []
        self.last_cross_ms: Optional[int] = None
        self.green_start_ms: Optional[int] = None
        self.perm: Optional[Tuple[Optional[int], float]] = None   # (t_g, window end) or None
        self.watch_road: Tuple[float, ...] = ()     # watched positions > 0 (on the approach)
        self.watch_box: Tuple[float, ...] = ()      # watched positions <= 0 (inside the box)
        self.passage_log: Optional[list] = None     # shared model log, set by VehicleModel

    # ---- permission -----------------------------------------------------
    def set_permission(self, state: SignalState, since_ms: int, now: int,
                       remaining_ms: Optional[int] = None) -> None:
        if state is SignalState.GREEN:
            self.green_start_ms = since_ms
            self.perm = (since_ms, math.inf)
        elif state is SignalState.FLASHING_GREEN:
            # permissive; yellow starts when the flashing timer ends, then the 2.0 s rule
            self.perm = (self.green_start_ms, now + remaining_ms + _YEXT_MS)
        elif state is SignalState.YELLOW:
            self.perm = (self.green_start_ms, since_ms + _YEXT_MS)
        elif state is SignalState.RED_YELLOW:
            # not permissive; remember when GREEN will begin (t_g of the discharge rule)
            self.green_start_ms = now + remaining_ms
            self.perm = None
        else:                                   # RED, ALL_RED
            self.perm = None
        # revoke crossing times outside the window (they are increasing along the lane)
        end = -math.inf if self.perm is None else self.perm[1]
        for v in self.road:
            if v._cross_plan_ms is not None and v._cross_plan_ms >= end:
                v._cross_plan_ms = None
                v._pinned = False
        self._release_in_order(now)

    def _release_in_order(self, now: int) -> None:
        if self.perm is None:
            return
        prev = self.last_cross_ms
        for v in self.road:
            if v._cross_plan_ms is None and not self._try_release(v, prev, now):
                return
            prev = v._cross_plan_ms

    def _try_release(self, v: Vehicle, prev_ms: Optional[int], now: int) -> bool:
        t_g, end = self.perm
        d = v._entry_ms + _FREE_TRAVEL_MS                   # own arrival at stop line
        if prev_ms is not None:
            d = max(d, prev_ms + self.h_ms)                  # previous departure + h
        if t_g is not None:
            d = max(d, t_g + _L1_MS + self.h_ms)            # t_g + l1 + h
        # physical guard: cannot reach the line faster than at v (never binding in
        # the study's cases; only matters if the model starts in mid-yellow)
        d = max(d, now + math.ceil(v._x / _V_PER_MS - 1e-6))
        if d < end:
            v._cross_plan_ms = d
            v._released_ms = now
            v._pinned = v._state is VehicleState.STOPPED
            return True
        return False

    # ---- kinematics -----------------------------------------------------
    @staticmethod
    def positions_at(base: Sequence[Tuple[Vehicle, float]], ta: int, t: int) -> List[float]:
        """Exact positions at time t in [ta, tb] of an event-free interval.

        Speeds are 0 or v, every constraint below is continuous and
        non-increasing with slope >= -v inside such an interval, so
        x(t) = min(x0, max(x0 - v(t - ta), constraint(t))). Constraints:
          no crossing time : max(behind leader, slot 1)        -- close up
          released, moving : max(behind leader, min(slot 1, B)) -- drive until halted
          released, halted : max(behind leader, B)              -- wait for latest start
        with B(t) = v * (D - t), the latest-start curve that reaches the line at D.
        """
        out: List[float] = []
        lead = None
        dt_m = _V_PER_MS * (t - ta)
        for v, x0 in base:
            behind = -math.inf if lead is None else lead + QUEUE_SPACING
            if v._cross_plan_ms is None:
                c = max(behind, SLOT1_POSITION)
            else:
                b = _V_PER_MS * (v._cross_plan_ms - t)
                c = max(behind, b) if v._pinned else max(behind, min(SLOT1_POSITION, b))
            x = min(x0, max(x0 - dt_m, c))
            out.append(x)
            lead = x
        return out

    def advance(self, t0: int, t1: int) -> None:
        self._events_at(t0)
        t = t0
        last_base = None
        while t < t1:
            base = [(v, v._x) for v in self.road]
            t_next = t1
            if self.road and self.road[0]._cross_plan_ms is not None:
                t_next = min(t_next, self.road[0]._cross_plan_ms)
            if self.pending:
                t_next = min(t_next, self.pending[0].scheduled_ms)
            if self.waiting and base:
                ta = self._admission_time(base, t, t_next)
                if ta is not None:
                    t_next = ta
            th = self._halt_time(base, t, t_next)
            if th is not None:
                t_next = th
            if t_next <= t:
                raise ModelInvariantError(f"{self.m}: event loop did not advance at {t} ms")
            self._move(base, t, t_next)
            last_base = (base, t, t_next)
            t = t_next
            self._events_at(t)
        if last_base is not None:
            self._update_motion_flags(*last_base)
        for v in self.box:
            v._x = -min(self.box_len, BOX_SPEED * (t1 - v._cross_ms) / MS)

    def _admission_time(self, base, t: int, t_end: int) -> Optional[int]:
        """Earliest ms in (t, t_end] at which the last car's front is <= 142.5 m."""
        ok = lambda tau: self.positions_at(base, t, tau)[-1] <= _ENTRY_ROOM + _EPS
        if not ok(t_end):
            return None
        lo, hi = t, t_end          # ok(lo) is False (else admitted already), ok(hi) True
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ok(mid):
                hi = mid
            else:
                lo = mid
        return hi

    @staticmethod
    def _deviated(base, idx, ta: int, xs, tau: int) -> List[int]:
        return [i for i in idx if xs[i] > base[i][1] - _V_PER_MS * (tau - ta) + 1e-6]

    def _halt_time(self, base, t: int, t_end: int) -> Optional[int]:
        """Earliest ms in (t, t_end] at which a released, still-moving car halts."""
        idx = [i for i, (v, _) in enumerate(base) if v._cross_plan_ms is not None and not v._pinned]
        if not idx or not self._deviated(base, idx, t, self.positions_at(base, t, t_end), t_end):
            return None
        lo, hi = t, t_end
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._deviated(base, idx, t, self.positions_at(base, t, mid), mid):
                hi = mid
            else:
                lo = mid
        return hi

    def _move(self, base, t: int, t_next: int) -> None:
        if t_next == t:
            return
        xs = self.positions_at(base, t, t_next)
        idx = [i for i, (v, _) in enumerate(base) if v._cross_plan_ms is not None and not v._pinned]
        halted = self._deviated(base, idx, t, xs, t_next)    # applied AFTER the searches below
        span_s = (t_next - t) / MS
        for i, ((v, x0), x1) in enumerate(zip(base, xs)):
            moved_s = (x0 - x1) / FREE_SPEED
            v._stopped_s += max(0.0, span_s - moved_s)
            if v._queue_join_ms is None:
                free = lambda tau, v=v: ENTRY_POSITION - _V_PER_MS * (tau - v._entry_ms)
                if x1 > free(t_next) + 1e-6:          # left the free-flow path: it stopped
                    lo, hi = t, t_next
                    while hi - lo > 1:
                        mid = (lo + hi) // 2
                        if self.positions_at(base, t, mid)[i] > free(mid) + 1e-6:
                            hi = mid
                        else:
                            lo = mid
                    v._queue_join_ms = lo
            for p in self.watch_road:
                if p not in v._passages and x0 > p + _EPS and x1 <= p + _EPS:
                    lo, hi = t, t_next                       # first ms with front <= p
                    while hi - lo > 1:
                        mid = (lo + hi) // 2
                        if self.positions_at(base, t, mid)[i] <= p + _EPS:
                            hi = mid
                        else:
                            lo = mid
                    self._record(v, p, hi)
            v._x = x1
        for i in halted:                                     # mode switch takes effect from t_next
            base[i][0]._pinned = True

    def _update_motion_flags(self, base, ta: int, tb: int) -> None:
        now_x = self.positions_at(base, ta, tb)
        before = self.positions_at(base, ta, tb - 1)
        for (v, _), xb, xa in zip(base, before, now_x):
            if v._state in (VehicleState.MOVING, VehicleState.STOPPED):
                v._state = VehicleState.MOVING if xb > xa + _EPS else VehicleState.STOPPED

    def _record(self, v: Vehicle, p: float, t: int) -> None:
        v._passages[p] = t
        if self.passage_log is not None:
            self.passage_log.append((t, v.id, self.m, p))

    def _events_at(self, t: int) -> None:
        # 1. stop-line crossings due now
        while self.road and self.road[0]._cross_plan_ms == t:
            v = self.road.pop(0)
            if abs(v._x) > 1e-6:
                raise ModelInvariantError(f"{v!r} scheduled to cross at {t} ms but is at {v._x} m")
            t_g, end = self.perm if self.perm is not None else (None, -math.inf)
            if not t < end:
                raise ModelInvariantError(f"{v!r} crossing at {t} ms outside its permission window")
            v._cross_ms = t
            v._exit_ms = t + self.box_ms
            v._x = 0.0
            v._state = VehicleState.IN_INTERSECTION
            self.last_cross_ms = t
            self.box.append(v)
            for p in self.watch_box:                        # box motion: x = -BOX_SPEED * (t - cross)
                d = -p
                tau = t + (math.ceil(d / BOX_SPEED * MS - 1e-6) if d <= self.box_len else self.box_ms)
                self._record(v, p, tau)
        # 2. scheduled arrivals join the outside queue (FIFO by scheduled time, id)
        while self.pending and self.pending[0].scheduled_ms <= t:
            v = self.pending.popleft()
            v._state = VehicleState.WAITING_OUTSIDE
            self.waiting.append(v)
        # 3. admit while there is room at the entry
        while self.waiting and (not self.road or self.road[-1]._x <= _ENTRY_ROOM + _EPS):
            v = self.waiting.popleft()
            v._entry_ms = t
            v._x = ENTRY_POSITION
            v._state = VehicleState.MOVING
            ahead = self.road[-1] if self.road else None
            self.road.append(v)
            if self.perm is not None and (ahead is None or ahead._cross_plan_ms is not None):
                prev = self.last_cross_ms if ahead is None else ahead._cross_plan_ms
                self._try_release(v, prev, t)
        # 4. leave the box
        while self.box and self.box[0]._exit_ms <= t:
            v = self.box.pop(0)
            v._state = VehicleState.EXITED
            v._x = -self.box_len
            self.done.append(v)


# ------------------------------------------------------------------ model
class VehicleModel:
    """All eight lanes. Drive it with advance(dt, signals).

    `signals` is anything with state(m) -> SignalState and since(m) -> seconds,
    plus remaining(m) -> seconds whenever it reports RED_YELLOW or
    FLASHING_GREEN (a signals.SignalBank works directly). It must describe the signal states
    in force from now until now + dt; call advance() BEFORE advancing the
    SignalBank for the same step. If it has a `now` attribute it must equal
    this model's time.
    """

    def __init__(self, arrivals: Union[ArrivalSchedule, Iterable[Arrival]],
                 start: Optional[float] = None):
        if isinstance(arrivals, ArrivalSchedule):
            items = list(arrivals.arrivals)
            start_ms = arrivals.meta.start_ms if start is None else _to_ms(start, "start")
        else:
            items = list(arrivals)
            if start is None:
                raise ValueError("start is required when arrivals is not an ArrivalSchedule")
            start_ms = _to_ms(start, "start")
        ids = set()
        per: Dict[str, List[Arrival]] = {m: [] for m in MOVES}
        for a in items:
            if not isinstance(a, Arrival):
                raise TypeError(f"arrivals must be traffic_gen.Arrival records, got {a!r}")
            validate_movement(a.movement)
            if a.id in ids:
                raise ValueError(f"duplicate arrival id {a.id}")
            if a.time_ms < start_ms:
                raise ValueError(f"arrival {a.id} at {a.time_ms / MS} s is before model start {start_ms / MS} s")
            ids.add(a.id)
            per[a.movement].append(a)
        self._now = start_ms
        self._lanes = {m: _Lane(m, sorted(per[m], key=lambda a: (a.time_ms, a.seq, a.id)))
                       for m in MOVES}
        self._by_id = {v.id: v for lane in self._lanes.values() for v in lane.pending}
        self._passage_log: List[Tuple[int, int, str, float]] = []
        self._watched: Tuple[float, ...] = ()
        self._advanced = False
        for lane in self._lanes.values():
            lane.passage_log = self._passage_log
            lane._events_at(self._now)

    # ----------------------------------------------------- observation hook
    def watch_positions(self, *positions: float) -> None:
        """Record, for every vehicle, the exact ms at which its FRONT bumper first
        reaches each position (metres before the stop line; <= 0 is inside the box).

        Passive and sensor-agnostic: it changes no vehicle behaviour. Must be
        called before the first advance(). Results: Vehicle.passage_ms(p) and
        passages_since(index).
        """
        if self._advanced:
            raise RuntimeError("watch_positions() must be called before the first advance()")
        ps = []
        for p in positions:
            if isinstance(p, bool) or not isinstance(p, (int, float)) or not math.isfinite(p):
                raise ValueError(f"watched position must be a finite number, got {p!r}")
            if p >= ENTRY_POSITION:
                raise ValueError(f"watched position must be < {ENTRY_POSITION} m, got {p!r}")
            ps.append(float(p))
        self._watched = tuple(sorted(set(self._watched) | set(ps), reverse=True))
        for lane in self._lanes.values():
            lane.watch_road = tuple(p for p in self._watched if p > 0)
            lane.watch_box = tuple(p for p in self._watched if p <= 0)

    @property
    def watched_positions(self) -> Tuple[float, ...]:
        return self._watched

    def passages_since(self, index: int) -> Tuple[List[Tuple[int, int, str, float]], int]:
        """Passage records (time_ms, vehicle id, movement, position) appended since
        `index`, and the new index. Box passages are recorded at the crossing and
        may be dated after now."""
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index <= len(self._passage_log):
            raise ValueError(f"bad passage index {index!r}")
        return list(self._passage_log[index:]), len(self._passage_log)

    # ---------------------------------------------------------------- time
    @property
    def now(self) -> float:
        return self._now / MS

    @property
    def now_ms(self) -> int:
        return self._now

    def advance(self, dt: float, signals) -> None:
        dt_ms = _to_ms(dt, "dt")
        if dt_ms < 0:
            raise ValueError(f"dt must not be negative, got {dt!r}")
        for attr in ("state", "since"):
            if not callable(getattr(signals, attr, None)):
                raise TypeError("signals must provide state(m) and since(m) (e.g. a SignalBank)")
        now_attr = getattr(signals, "now", None)
        if now_attr is not None and round(now_attr * MS) != self._now:
            raise ValueError(f"signals are at t={now_attr} s but the vehicle model is at "
                             f"t={self.now} s; call advance() before the SignalBank's")
        t0, t1 = self._now, self._now + dt_ms
        self._advanced = True
        for m, lane in self._lanes.items():
            st = signals.state(m)
            if not isinstance(st, SignalState):
                raise TypeError(f"signals.state({m!r}) must return a SignalState, got {st!r}")
            rem = None
            if st in (SignalState.RED_YELLOW, SignalState.FLASHING_GREEN):
                if not callable(getattr(signals, "remaining", None)):
                    raise TypeError(f"signals must provide remaining(m) for {st.name}")
                rem = _to_ms(signals.remaining(m), "remaining")
            lane.set_permission(st, _to_ms(signals.since(m), "since"), t0, rem)
            lane.advance(t0, t1)
        self._now = t1
        self.check_invariants()

    # ------------------------------------------------------------- queries
    def _lane(self, m: str) -> _Lane:
        return self._lanes[validate_movement(m)]

    def vehicles(self, m: str) -> Tuple[Vehicle, ...]:
        """Vehicles on the approach road, front (nearest the stop line) first."""
        return tuple(self._lane(m).road)

    def lead_vehicle(self, m: str) -> Optional[Vehicle]:
        road = self._lane(m).road
        return road[0] if road else None

    def queue_length(self, m: str) -> int:
        """Stopped vehicles on the approach (study §11 queue definition)."""
        return sum(v._state is VehicleState.STOPPED for v in self._lane(m).road)

    def stopped_positions(self, m: str) -> Tuple[float, ...]:
        return tuple(v._x for v in self._lane(m).road if v._state is VehicleState.STOPPED)

    def waiting_outside(self, m: str) -> Tuple[Vehicle, ...]:
        """R3: scheduled vehicles that have not yet found room to enter."""
        return tuple(self._lane(m).waiting)

    def in_intersection(self, m: str) -> Tuple[Vehicle, ...]:
        return tuple(self._lane(m).box)

    def crossed(self, m: str) -> Tuple[Vehicle, ...]:
        """All vehicles of m that have crossed the stop line, in crossing order."""
        lane = self._lane(m)
        return tuple(lane.done) + tuple(lane.box)

    def completed(self, m: Optional[str] = None) -> Tuple[Vehicle, ...]:
        """Vehicles that have left the intersection (all movements if m is None)."""
        if m is not None:
            return tuple(self._lane(m).done)
        return tuple(sorted((v for l in self._lanes.values() for v in l.done),
                            key=lambda v: (v._exit_ms, v.id)))

    def vehicle(self, vid: int) -> Vehicle:
        if vid not in self._by_id:
            raise KeyError(f"no vehicle with id {vid!r}")
        return self._by_id[vid]

    def all_vehicles(self) -> Tuple[Vehicle, ...]:
        return tuple(self._by_id[k] for k in sorted(self._by_id))

    # ---------------------------------------------------------- invariants
    def check_invariants(self) -> None:
        """No overlap, FIFO order, nobody past the stop line on the approach."""
        for m, lane in self._lanes.items():
            prev = None
            for v in lane.road:
                if not (v._x >= -1e-6 and v._x <= ENTRY_POSITION + _EPS):
                    raise ModelInvariantError(f"{m}: {v!r} outside the approach")
                if v._cross_plan_ms is None and v._x < SLOT1_POSITION - 1e-6:
                    raise ModelInvariantError(f"{m}: {v!r} past slot 1 without permission")
                if prev is not None:
                    if v._x - prev._x < QUEUE_SPACING - 1e-6:
                        raise ModelInvariantError(f"{m}: overlap between {prev!r} and {v!r}")
                    if (v.scheduled_ms, v.seq) < (prev.scheduled_ms, prev.seq):
                        raise ModelInvariantError(f"{m}: FIFO order broken at {v!r}")
                prev = v


def _to_ms(seconds, what: str) -> int:
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds):
        raise ValueError(f"{what} must be a finite number of seconds, got {seconds!r}")
    ms = round(seconds * MS)
    if abs(seconds * MS - ms) > 1e-6:
        raise ValueError(f"{what} must be a whole number of milliseconds, got {seconds!r}")
    return ms
