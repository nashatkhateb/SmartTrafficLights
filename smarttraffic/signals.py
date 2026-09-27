"""signals -- per-movement signal-head state machine and lamp outputs.

Spec: docs/Eight-Movement_Intersection_Study.html, §6 (timing), §7 (states,
invariant), §13 (module table), §16 step 2; visible sequence revision
(docs/ARCHITECTURE.md, "Signal sequence").
Responsibility: per-head state machine with fixed timed intervals; refuses
illegal transitions; says which lamps are lit.
Must not: decide when to change (that is the controller's job).

Visible sequence (per movement)
-------------------------------
    RED --start_green()--> RED_YELLOW --(timer)--> GREEN
        --end_green()--> FLASHING_GREEN --(timer)--> YELLOW --(timer)--> ALL_RED --(timer)--> RED

* A controller issues only two commands:
    start_green(m): starts the sequence that RESULTS in green (RED -> RED_YELLOW;
                    GREEN follows automatically). It is not an instant green.
    end_green(m):   starts the termination sequence (GREEN -> FLASHING_GREEN;
                    YELLOW, ALL_RED and RED follow automatically).
* Every other change happens only when a fixed timer expires inside advance().
  Nothing can be skipped, shortened or reversed.

Safety semantics
----------------
* CONFLICT-BLOCKING states: RED_YELLOW, GREEN, FLASHING_GREEN, YELLOW, ALL_RED.
  Only RED is non-blocking. A movement may begin start_green() only if it is RED
  and every conflicting movement is RED (conflicts.compatible decides "conflicting").
* ALL_RED lights the same lamp as RED, but it is the terminating movement's
  own red clearance: vehicles may still be in the junction, so it blocks
  conflicting movements until its timer ends. It is per movement, never a
  global all-red: compatible movements keep running and may even start.
* RED_YELLOW is blocking (the movement is committed to going green) but NOT
  permissive: no vehicle may cross. GREEN and FLASHING_GREEN are permissive.
  The first 2.0 s of YELLOW rule lives in vehicles.py (driver behaviour).

Green-time convention (for the controllers)
-------------------------------------------
Green time G = GREEN + FLASHING_GREEN, measured from GREEN onset (both are
permissive). Minimum green, maximum green and fixed-plan greens refer to G;
to end service at G, a controller calls end_green at G - FLASHING_GREEN_S.
RED_YELLOW is additional lost time before GREEN.

Lamps
-----
lamp_state(m) / lamps_for(state, elapsed_ms) return an immutable
Lamps(red, yellow, green). FLASHING_GREEN blinks with period FLASH_PERIOD_S,
lamp ON for the first FLASH_ON_S of each period, counted from the onset of
FLASHING_GREEN in simulation time only.

Time: integer milliseconds internally; timer expiries inside a large step are
applied at their exact times, in chronological order.
"""
from enum import Enum
import math
from typing import Dict, List, NamedTuple, Optional, Tuple

from smarttraffic import conflicts
from smarttraffic.conflicts import MOVES, validate_movement

__all__ = [
    "SignalState", "RED", "RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED",
    "ACTIVE_STATES", "CONFLICT_BLOCKING_STATES", "PERMISSIVE_STATES", "DISPLAY",
    "YELLOW_S", "ALL_RED_S", "RED_YELLOW_S", "FLASHING_GREEN_S", "FLASH_PERIOD_S", "FLASH_ON_S",
    "Lamps", "lamps_for", "SignalBank", "TransitionEvent", "movement_type",
    "IllegalTransition", "InvalidStateError",
]

MS_PER_S = 1000

# ------------------------------------------------ study §6 / §14 timing (frozen)
YELLOW_S = {"straight": 3.5, "left": 3.0}
ALL_RED_S = {"straight": 2.0, "left": 3.0}

# ------------------------------------------------ project assumptions (configurable)
RED_YELLOW_S = 1.0        # pre-green red+yellow; additional lost time per start
FLASHING_GREEN_S = 3.0    # permissive; counts as part of green time G
FLASH_PERIOD_S = 1.0      # one ON+OFF blink
FLASH_ON_S = 0.5          # green lamp ON for the first 0.5 s of each period


class SignalState(Enum):
    RED = "RED"
    RED_YELLOW = "RED_YELLOW"
    GREEN = "GREEN"
    FLASHING_GREEN = "FLASHING_GREEN"
    YELLOW = "YELLOW"
    ALL_RED = "ALL_RED"


# Canonical representation is the SignalState enum. These module-level names
# are convenience aliases for the SAME enum members (identity, not copies).
RED = SignalState.RED
RED_YELLOW = SignalState.RED_YELLOW
GREEN = SignalState.GREEN
FLASHING_GREEN = SignalState.FLASHING_GREEN
YELLOW = SignalState.YELLOW
ALL_RED = SignalState.ALL_RED

# Every state except RED blocks conflicting movements (study §7 "non-RED").
CONFLICT_BLOCKING_STATES = frozenset({RED_YELLOW, GREEN, FLASHING_GREEN, YELLOW, ALL_RED})
ACTIVE_STATES = CONFLICT_BLOCKING_STATES          # name kept from the first version
# States in which the signal itself permits crossing (YELLOW's 2.0 s rule is in vehicles.py).
PERMISSIVE_STATES = frozenset({GREEN, FLASHING_GREEN})
# Compact code for logs/timelines (G8). ALL_RED shows "R" like RED.
DISPLAY = {RED: "R", RED_YELLOW: "RY", GREEN: "G", FLASHING_GREEN: "FG", YELLOW: "Y", ALL_RED: "R"}


class IllegalTransition(Exception):
    """A commanded transition is not allowed (wrong state, or unsafe)."""


class InvalidStateError(TypeError):
    """A value that is not a SignalState was given where one is required."""


class TransitionEvent(NamedTuple):
    time: float          # seconds, exact time of the change
    movement: str
    old: SignalState
    new: SignalState


class Lamps(NamedTuple):
    """Physical lamp outputs of one signal head at one instant."""
    red: bool
    yellow: bool
    green: bool


def movement_type(m: str) -> str:
    """'straight' for NS/SS/ES/WS, 'left' for NL/SL/EL/WL."""
    validate_movement(m)
    return "straight" if m[1] == "S" else "left"


def _to_ms(seconds: float, what: str) -> int:
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
        raise TypeError(f"{what} must be a number of seconds, got {seconds!r}")
    if not math.isfinite(seconds):
        raise ValueError(f"{what} must be finite, got {seconds!r}")
    ms = round(seconds * MS_PER_S)
    if abs(seconds * MS_PER_S - ms) > 1e-6:
        raise ValueError(f"{what} must be a whole number of milliseconds, got {seconds!r}")
    return ms


def _validate_state(s: object) -> SignalState:
    if not isinstance(s, SignalState):
        raise InvalidStateError(
            f"state must be a SignalState ({', '.join(x.name for x in SignalState)}), got {s!r}")
    return s


YELLOW_MS = {t: _to_ms(v, "yellow") for t, v in YELLOW_S.items()}
ALL_RED_MS = {t: _to_ms(v, "all-red") for t, v in ALL_RED_S.items()}
RED_YELLOW_MS = _to_ms(RED_YELLOW_S, "red-yellow")
FLASHING_GREEN_MS = _to_ms(FLASHING_GREEN_S, "flashing green")
FLASH_PERIOD_MS = _to_ms(FLASH_PERIOD_S, "flash period")
FLASH_ON_MS = _to_ms(FLASH_ON_S, "flash on")
assert 0 < FLASH_ON_MS < FLASH_PERIOD_MS

# next state when a timed state expires
_AUTO_NEXT = {RED_YELLOW: GREEN, FLASHING_GREEN: YELLOW, YELLOW: ALL_RED, ALL_RED: RED}

_LAMPS = {
    RED: Lamps(True, False, False),
    RED_YELLOW: Lamps(True, True, False),
    GREEN: Lamps(False, False, True),
    YELLOW: Lamps(False, True, False),
    ALL_RED: Lamps(True, False, False),
}


def lamps_for(state: SignalState, elapsed_ms: int) -> Lamps:
    """Lamp outputs for a head that has been in `state` for `elapsed_ms` (pure function).

    FLASHING_GREEN: green ON for the first FLASH_ON_S of every FLASH_PERIOD_S,
    counted from the onset of FLASHING_GREEN. Deterministic; no clock is read.
    """
    _validate_state(state)
    if isinstance(elapsed_ms, bool) or not isinstance(elapsed_ms, int) or elapsed_ms < 0:
        raise ValueError(f"elapsed_ms must be a non-negative int, got {elapsed_ms!r}")
    if state is FLASHING_GREEN:
        return Lamps(False, False, elapsed_ms % FLASH_PERIOD_MS < FLASH_ON_MS)
    return _LAMPS[state]


class SignalBank:
    """The 8 signal heads of the intersection. All start RED at time 0 (or `start_time`)."""

    def __init__(self, start_time: float = 0.0):
        self._now_ms = _to_ms(start_time, "start_time")
        self._state: Dict[str, SignalState] = {m: RED for m in MOVES}
        self._since_ms: Dict[str, int] = {m: self._now_ms for m in MOVES}
        self.log: List[TransitionEvent] = []

    # ------------------------------------------------------------ queries
    @property
    def now(self) -> float:
        return self._now_ms / MS_PER_S

    def state(self, m: str) -> SignalState:
        return self._state[validate_movement(m)]

    def states(self) -> Dict[str, SignalState]:
        return dict(self._state)

    def display(self, m: str) -> str:
        """Compact code: 'R', 'RY', 'G', 'FG', 'Y' (ALL_RED shows 'R')."""
        return DISPLAY[self.state(m)]

    def lamp_state(self, m: str) -> Lamps:
        """Which lamps are physically ON for m at the current simulation time."""
        validate_movement(m)
        return lamps_for(self._state[m], self._now_ms - self._since_ms[m])

    def lamp_states(self) -> Dict[str, Lamps]:
        return {m: self.lamp_state(m) for m in MOVES}

    def since(self, m: str) -> float:
        """Time at which m entered its current state."""
        return self._since_ms[validate_movement(m)] / MS_PER_S

    def elapsed(self, m: str) -> float:
        """Seconds m has been in its current state."""
        return (self._now_ms - self._since_ms[validate_movement(m)]) / MS_PER_S

    def remaining(self, m: str) -> Optional[float]:
        """Seconds left in a timed state (RED_YELLOW, FLASHING_GREEN, YELLOW, ALL_RED);
        None for GREEN and RED, which last until commanded."""
        end = self._timed_end_ms(validate_movement(m))
        return None if end is None else (end - self._now_ms) / MS_PER_S

    def is_permissive(self, m: str) -> bool:
        """True in GREEN and FLASHING_GREEN (YELLOW's 2 s rule is the vehicle model's)."""
        return self.state(m) in PERMISSIVE_STATES

    def active(self) -> Tuple[str, ...]:
        """Movements in a conflict-blocking state, in MOVES order."""
        return tuple(m for m in MOVES if self._state[m] in CONFLICT_BLOCKING_STATES)

    def in_state(self, s: SignalState) -> Tuple[str, ...]:
        _validate_state(s)
        return tuple(m for m in MOVES if self._state[m] is s)

    def blockers(self, m: str) -> Tuple[str, ...]:
        """Conflict-blocking movements that conflict with m (m cannot start while non-empty)."""
        validate_movement(m)
        return tuple(x for x in self.active() if not conflicts.compatible(m, x))

    def can_start_green(self, m: str) -> bool:
        return self.state(m) is RED and not self.blockers(m)

    # ----------------------------------------------------------- commands
    def start_green(self, m: str) -> None:
        """Start the green SEQUENCE: RED -> RED_YELLOW now, GREEN after RED_YELLOW_S.

        Rejected unless m is RED and every conflicting movement is RED.
        """
        cur = self.state(m)
        if cur is not RED:
            raise IllegalTransition(f"{m}: cannot start the green sequence from {cur.name} (must be RED)")
        blk = self.blockers(m)
        if blk:
            detail = ", ".join(f"{b} is {self._state[b].name}" for b in blk)
            raise IllegalTransition(f"{m}: cannot start the green sequence while conflicting {detail}")
        self._set(m, RED_YELLOW, self._now_ms)
        self.check_invariant()

    def end_green(self, m: str) -> None:
        """Start the termination SEQUENCE: GREEN -> FLASHING_GREEN now; YELLOW,
        ALL_RED and RED follow on fixed timers."""
        cur = self.state(m)
        if cur is not GREEN:
            raise IllegalTransition(f"{m}: cannot end green from {cur.name} (must be GREEN)")
        self._set(m, FLASHING_GREEN, self._now_ms)

    # --------------------------------------------------------------- time
    def advance(self, dt: float) -> None:
        """Advance simulated time by dt seconds, expiring timed states.

        Expiries are applied at their exact times, in chronological order
        (ties in MOVES order), even if dt spans several of them.
        """
        dt_ms = _to_ms(dt, "dt")
        if dt_ms < 0:
            raise ValueError(f"dt must not be negative, got {dt!r}")
        target = self._now_ms + dt_ms
        while True:
            due = [(end, MOVES.index(m), m) for m in MOVES
                   for end in [self._timed_end_ms(m)] if end is not None and end <= target]
            if not due:
                break
            end, _, m = min(due)
            self._now_ms = end
            self._set(m, _AUTO_NEXT[self._state[m]], end)
        self._now_ms = target
        self.check_invariant()

    # ------------------------------------------------------------- safety
    def check_invariant(self) -> None:
        """Study §7: no two conflict-blocking movements may conflict.

        Raises conflicts.SafetyViolation. Called after every command and
        every advance(); the simulator should also call it every step.
        """
        conflicts.check_invariant(self.active())

    # ----------------------------------------------------------- internal
    def _timed_end_ms(self, m: str) -> Optional[int]:
        s, t = self._state[m], movement_type(m)
        dur = {RED_YELLOW: RED_YELLOW_MS, FLASHING_GREEN: FLASHING_GREEN_MS,
               YELLOW: YELLOW_MS[t], ALL_RED: ALL_RED_MS[t]}.get(s)
        return None if dur is None else self._since_ms[m] + dur

    def _set(self, m: str, new: SignalState, at_ms: int) -> None:
        old = self._state[m]
        self._state[m] = new
        self._since_ms[m] = at_ms
        self.log.append(TransitionEvent(at_ms / MS_PER_S, m, old, new))
