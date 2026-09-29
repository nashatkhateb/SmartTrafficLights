"""manual -- Custom / Demo only: manual vehicle injection ("exact count" mode).

The user adds an exact number of vehicles to one movement while a Custom / Demo run is going
(+1, +5, or a typed count). Official scenarios never use this module: runner.simulate refuses an
injector outside mode "custom", and VizSession creates one only for Custom / Demo runs.

Schedule (deterministic, documented)
------------------------------------
A request (movement m, count n) is applied at the next simulation step boundary T (with 100 ms
steps: at most 100 ms of simulation time after the click; a request made while paused is applied
at the paused instant). It becomes n ordinary arrivals of m at the 150 m entry point:

    t_1 = max(T, previous manual arrival of m + 1.5 s),   t_k = t_1 + (k - 1) * 1.5 s

1.5 s is the arrival model's minimum headway (traffic_gen.MIN_HEADWAY_S). The vehicles then go
through the unchanged VehicleModel: R3 entry spacing (a car enters only when the car ahead is
>= 7.5 m in, otherwise it waits outside), queue spacing, red / yellow / green, Stop on Yellow,
immediate first vehicle at green, 1.9 s / 2.0 s discharge headway. They reach the detectors
like any other car, so a controller only learns about them through SensorBank - nothing forces
a green. Their Arrival.source is "manual" (visible in the per-vehicle results).

A batch is applied completely or not at all. It is rejected (and recorded) when
  * m is a real-demand-only movement of the hardware session (NS with the demo hardware),
  * any of its arrivals would fall at or after the end of the measurement window
    (the run's arrival window; after it the run only flushes), or
  * n is not an integer in 1..MAX_BATCH.

Replay: every applied request is recorded as (T, m, n); ManualInjector(schedule=...) applies the
same requests at the same boundaries, so a run with the same schedule is identical
(run metadata "manual_injections").
"""
import threading
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from smarttraffic.conflicts import MOVES, validate_movement
from smarttraffic.traffic_gen import Arrival

__all__ = ["ManualInjector", "MANUAL_SPACING_MS", "MAX_BATCH", "MANUAL_SOURCE", "InjectionEvent",
           "replay_schedule"]

MANUAL_SPACING_MS = 1500        # = traffic_gen minimum headway 1.5 s
MAX_BATCH = 50
MANUAL_SOURCE = "manual"


class InjectionEvent(tuple):
    """(t_ms, movement, count, first_arrival_ms, last_arrival_ms) of an applied request."""
    __slots__ = ()

    def __new__(cls, t_ms, movement, count, first_ms, last_ms):
        return super().__new__(cls, (t_ms, movement, count, first_ms, last_ms))

    t_ms = property(lambda s: s[0])
    movement = property(lambda s: s[1])
    count = property(lambda s: s[2])
    first_ms = property(lambda s: s[3])
    last_ms = property(lambda s: s[4])


def _check_count(n) -> int:
    if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= MAX_BATCH:
        raise ValueError(f"vehicle count must be an integer 1..{MAX_BATCH}, got {n!r}")
    return n


class ManualInjector:
    """Thread-safe queue of manual requests, applied by the runner at step boundaries.

    request(m, n) may be called from any thread (the UI); the simulation thread calls apply().
    schedule: replay requests [(t_ms, m, n), ...] applied at the first boundary >= t_ms."""

    def __init__(self, schedule: Iterable[Sequence] = (), blocked: Iterable[str] = ()):
        self._lock = threading.Lock()
        self._live: List[Tuple[str, int]] = []
        self._scheduled: List[Tuple[int, int, str, int]] = []
        for i, (t, m, n, *_) in enumerate(schedule):
            self._scheduled.append((int(t), i, validate_movement(m), _check_count(n)))
        self._scheduled.sort()
        self.blocked = frozenset(blocked)
        self.events: List[InjectionEvent] = []
        self.rejected: List[Tuple[int, str, int, str]] = []
        self._last: Dict[str, int] = {}
        self.closed = False                 # set when the run's arrival window has ended

    # ------------------------------------------------------------ UI side
    def request(self, m: str, n: int) -> Optional[str]:
        """Queue n vehicles for m. Returns None if queued, else the reason it was refused."""
        m = validate_movement(m)
        n = _check_count(n)
        if m in self.blocked:
            return f"{m} is real-demand-only in hardware mode (physical sensors only)"
        if self.closed:
            return "the measurement window has ended: no new arrivals"
        with self._lock:
            self._live.append((m, n))
        return None

    def pending(self) -> int:
        with self._lock:
            return sum(n for _, n in self._live)

    def counts(self) -> Dict[str, int]:
        """Vehicles actually injected so far, per movement."""
        out = {m: 0 for m in MOVES}
        for e in list(self.events):
            out[e.movement] += e.count
        return out

    def block(self, movements: Iterable[str]) -> None:
        self.blocked = frozenset(self.blocked) | frozenset(movements)

    # ------------------------------------------------------------ simulation side
    def apply(self, now_ms: int, vm, measure_end_ms: int) -> List[Arrival]:
        """Build and inject the arrivals of every request due at boundary now_ms.
        Returns the new arrivals (already injected into vm)."""
        with self._lock:
            live, self._live = self._live, []
        due = []
        while self._scheduled and self._scheduled[0][0] <= now_ms:
            _, _, m, n = self._scheduled.pop(0)
            due.append((m, n))
        due.extend(live)
        if now_ms >= measure_end_ms:
            self.closed = True
        out: List[Arrival] = []
        for m, n in due:
            if m in self.blocked:
                self.rejected.append((now_ms, m, n, "real-demand-only movement"))
                continue
            first = max(now_ms, self._last.get(m, -10**12) + MANUAL_SPACING_MS)
            times = [first + k * MANUAL_SPACING_MS for k in range(n)]
            if times[-1] >= measure_end_ms:
                self.rejected.append((now_ms, m, n, "after the measurement window"))
                continue
            next_id = vm.next_vehicle_id()
            seq0 = vm.next_seq(m)
            arr = [Arrival(next_id + k, t, m, seq0 + k, MANUAL_SOURCE) for k, t in enumerate(times)]
            vm.inject(arr)
            self._last[m] = times[-1]
            self.events.append(InjectionEvent(now_ms, m, n, times[0], times[-1]))
            out.extend(arr)
        return out

    def metadata(self) -> Dict[str, object]:
        """Run-metadata keys; empty when nothing was requested, so a Custom run without manual
        vehicles is exactly the same as before this feature."""
        if not self.events and not self.rejected:
            return {}
        return {
            "manual_injections": [list(e) for e in self.events],
            "manual_injection_format": "t_ms, movement, count, first_arrival_ms, last_arrival_ms",
            "manual_vehicles": sum(e.count for e in self.events),
            "manual_rejected": [list(r) for r in self.rejected],
            "manual_spacing_ms": MANUAL_SPACING_MS,
        }


def replay_schedule(metadata: Dict[str, object]) -> List[Tuple[int, str, int]]:
    """The (t_ms, m, n) requests recorded in a run's metadata, for ManualInjector(schedule=...)."""
    return [(int(e[0]), e[1], int(e[2])) for e in metadata.get("manual_injections", [])]
