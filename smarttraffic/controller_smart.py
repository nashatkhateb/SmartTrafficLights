"""controller_smart -- movement-based, fully actuated controller (Stage 11).

Spec: docs/Eight-Movement_Intersection_Study.html §6 (timing), §7 (algorithm and
pseudocode), §9 (fairness), §13; docs/ARCHITECTURE.md ("Smart controller (Stage 11)").
Input: SensorBank queries and SignalBank state only. Output: start_green / end_green.
Must not: import vehicles, detector sources, hardware or serial code; must not know
whether a detector is real or virtual.

Summary (study §7, under the Signals V2 sequence)
-------------------------------------------------
* Demand. A movement is WAITING when it is RED and has a call (SensorBank.has_call,
  or a fail-safe recall while one of its detectors is faulted).
  P(m) = Qhat(m) + wait(m) / tau, wait = now - call_since. Integer form:
  P * tau_ms = Qhat * tau_ms + wait_ms.
* Target. Every compatible group G (conflicts.GROUPS, fixed order) is scored
  S(G) = sum of P over G's waiting members + green_bonus_qhat for each member that is
  serving (RED_YELLOW or GREEN) and not gapped out. Exact ties: more serving members,
  then the longest single wait among G's waiting members, then GROUPS order.
  With a starvation hold, only groups containing the hold are candidates.
* Add compatible demand. A waiting movement is started when SignalBank allows it and it
  is compatible with BOTH members of the target (target protection, literal study rule).
  Nothing is stopped for this.
* Switch. to_start = the target's waiting members. Only GREEN movements that conflict
  with a to_start member are ended, and only when every one of them is releasable:
  minimum green met and (gap-out, max-out, or conflict with the hold). Then the
  transition is committed: the targets are started as soon as SignalBank allows and
  nothing else is decided until they are all GREEN.
* Rest in green. Without a conflicting target nothing is ended, even after gap-out.
* Green time G = GREEN + FLASHING_GREEN from GREEN onset (ARCHITECTURE.md): end_green
  for a minimum green MIN is allowed from onset + MIN - FLASHING_GREEN; max-out issues
  end_green at max_start + MAX - FLASHING_GREEN (max_start = first update, while GREEN,
  at which a conflicting movement has a call).
* Starvation. A waiting call with wait >= W_MAX becomes the hold (longest wait, then MOVES
  order). It is only a selection rule: SignalBank intervals are never shortened.
* Faulted detector (fail-safe recall). While any detector of m is faulted (SensorBank's
  faulted_detectors(), however the fault arises), m has demand with Qhat >= fault_qhat,
  its call time is the later of the fault onset and m's last service end, and its
  gap-out is ignored (it ends by max-out or a starvation hold).

Time: integer milliseconds. update(bank) is called once per simulation step, after
vehicles, sensors and SignalBank have advanced to the same time; decisions happen only
at those step boundaries.
"""
from dataclasses import dataclass
import math
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

from smarttraffic.conflicts import GROUPS, MOVES, compatible
from smarttraffic.signals import (ALL_RED, FLASHING_GREEN, FLASHING_GREEN_MS, GREEN, MS_PER_S, RED,
                                  RED_YELLOW, YELLOW, movement_type)

__all__ = ["SmartParams", "SmartController", "Command", "Decision", "GROUP_ORDER",
           "START_GREEN", "END_GREEN"]

START_GREEN = "start_green"
END_GREEN = "end_green"

# Compatible groups in the conflict model's fixed order, members in MOVES order.
# (GROUPS holds frozensets; their iteration order must never be used.)
GROUP_ORDER: Tuple[Tuple[str, ...], ...] = tuple(tuple(sorted(g, key=MOVES.index)) for g in GROUPS)

_SERVING = (RED_YELLOW, GREEN)
_IN_SERVICE = (GREEN, FLASHING_GREEN)


def _ms(seconds: object, what: str) -> int:
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds):
        raise ValueError(f"{what} must be a finite number of seconds, got {seconds!r}")
    ms = round(seconds * MS_PER_S)
    if abs(seconds * MS_PER_S - ms) > 1e-6:
        raise ValueError(f"{what} must be a whole number of milliseconds, got {seconds!r}")
    return ms


@dataclass(frozen=True)
class SmartParams:
    """Study §6/§7/§9/§14 values. The three §9 switches exist for later ablation runs."""
    min_green_straight_s: float = 7.0
    min_green_left_s: float = 5.0
    extended_initial_s: float = 12.0      # = sensors.EXTENDED_INITIAL_GREEN_S (checked by tests)
    max_green_straight_s: float = 45.0
    max_green_left_s: float = 30.0
    tau_s: float = 10.0
    w_max_s: float = 90.0
    green_bonus_qhat: int = 2             # study: Qhat_green = 2
    fault_qhat: int = 2                   # fail-safe recall demand (= LEVEL_1 Qhat)
    use_max_green: bool = True
    use_aging: bool = True
    use_w_max: bool = True

    def __post_init__(self):
        for name in ("min_green_straight_s", "min_green_left_s", "extended_initial_s",
                     "max_green_straight_s", "max_green_left_s", "tau_s", "w_max_s"):
            v = _ms(getattr(self, name), name)
            if v <= 0:
                raise ValueError(f"{name} must be positive")
        for t in ("straight", "left"):
            if self.min_green_ms(t, False) < FLASHING_GREEN_MS:
                raise ValueError(f"minimum green ({t}) must be at least FLASHING_GREEN (it is part of G)")
            if self.max_green_ms(t) < max(self.min_green_ms(t, False), self.min_green_ms(t, True)):
                raise ValueError(f"maximum green ({t}) must not be below the minimum greens")
        for name in ("green_bonus_qhat", "fault_qhat"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < 0:
                raise ValueError(f"{name} must be a non-negative int")

    def min_green_ms(self, kind: str, extended: bool) -> int:
        if extended:
            return _ms(self.extended_initial_s, "extended_initial_s")
        return _ms(self.min_green_straight_s if kind == "straight" else self.min_green_left_s, "min green")

    def max_green_ms(self, kind: str) -> int:
        return _ms(self.max_green_straight_s if kind == "straight" else self.max_green_left_s, "max green")

    @property
    def tau_ms(self) -> int:
        return _ms(self.tau_s, "tau_s")

    @property
    def w_max_ms(self) -> int:
        return _ms(self.w_max_s, "w_max_s")


class Command(NamedTuple):
    time_ms: int
    kind: str            # START_GREEN or END_GREEN
    movement: str


class Decision(NamedTuple):
    time_ms: int
    target: Tuple[str, ...]
    hold: Optional[str]
    stopped: Tuple[Tuple[str, str], ...]     # (movement, reason: "gap" | "max" | "hold")
    to_start: Tuple[str, ...]


class SmartController:
    """Fully actuated controller. Reads a SensorBank; commands a SignalBank."""
    uses_sensors = True

    def __init__(self, sensors, params: SmartParams = SmartParams()):
        if not isinstance(params, SmartParams):
            raise TypeError("params must be a SmartParams")
        self.sensors = sensors
        self.params = params
        self._onset: Dict[str, Optional[int]] = {m: None for m in MOVES}
        self._min: Dict[str, Optional[int]] = {m: None for m in MOVES}
        self._max_start: Dict[str, Optional[int]] = {m: None for m in MOVES}
        self._service_end: Dict[str, Optional[int]] = {m: None for m in MOVES}
        self._fault_since: Dict[str, Optional[int]] = {m: None for m in MOVES}
        self._commit: Optional[Tuple[Tuple[str, ...], Tuple[str, ...]]] = None
        self._last: Optional[int] = None
        self.command_log: List[Command] = []
        self.decision_log: List[Decision] = []
        self.last_target: Optional[Tuple[str, ...]] = None
        self.last_hold: Optional[str] = None

    # ================================================================ queries
    def committed(self) -> Optional[Tuple[Tuple[str, ...], Tuple[str, ...]]]:
        """(stopped, to_start) of the transition in progress, or None."""
        return self._commit

    def green_onset_ms(self, m: str) -> Optional[int]:
        return self._onset[m]

    def min_green_of(self, m: str) -> Optional[int]:
        """Minimum green G (ms) chosen at m's current GREEN onset."""
        return self._min[m]

    def max_start_ms(self, m: str) -> Optional[int]:
        return self._max_start[m]

    def faulted_movements(self) -> Tuple[str, ...]:
        return tuple(m for m in MOVES if self._fault_since[m] is not None)

    # ================================================================ update
    def update(self, bank) -> List[Command]:
        now = round(bank.now * MS_PER_S)
        if self.sensors.now_ms != now:
            raise ValueError(f"sensors are at {self.sensors.now_ms} ms but signals at {now} ms: "
                             f"update the controller after sensors and SignalBank have advanced")
        if self._last is not None and now < self._last:
            raise ValueError(f"time went backwards ({now} < {self._last} ms)")
        self._last = now
        issued: List[Command] = []
        st = {m: bank.state(m) for m in MOVES}
        self._track(bank, st, now)
        if self._commit is not None:
            self._finish_commit(bank, now, issued)
            return issued

        waiting = [m for m in MOVES if st[m] is RED and self._has_call(m, st)]
        if not waiting:
            self.last_target, self.last_hold = None, None
            return issued                                   # rest in the current greens
        wait = {m: now - self._call_ms(m, st) for m in waiting}
        hold = self._hold(waiting, wait)
        target = self._target(waiting, wait, hold, st)
        self.last_target, self.last_hold = target, hold

        # (a) add compatible demand without stopping anything (target protection: both members)
        for m in waiting:
            if bank.can_start_green(m) and all(compatible(m, t) for t in target):
                self._start(bank, m, now, issued)
        # (b) switch toward the target: stop only conflicting GREEN movements, all releasable
        st = {m: bank.state(m) for m in MOVES}
        to_start = tuple(m for m in target if m in wait and st[m] is RED)
        if not to_start:
            return issued
        reasons = self._switch_reasons(to_start, hold, st, now)
        if reasons is None:
            return issued
        for g, _ in reasons:
            bank.end_green(g)
            self._service_end[g] = now + FLASHING_GREEN_MS
            self._log(now, END_GREEN, g, issued)
        self.decision_log.append(Decision(now, target, hold, tuple(reasons), to_start))
        self._commit = (tuple(g for g, _ in reasons), to_start)
        self._finish_commit(bank, now, issued)
        return issued

    # ============================================================== internal
    def _track(self, bank, st, now: int) -> None:
        faulted = {m for m, _ in self.sensors.faulted_detectors()}
        for m in MOVES:
            if m in faulted:
                if self._fault_since[m] is None:
                    self._fault_since[m] = now
            else:
                self._fault_since[m] = None
        for m in MOVES:
            if st[m] is GREEN:
                onset = round(bank.since(m) * MS_PER_S)
                if self._onset[m] != onset:                  # a new service began
                    self._onset[m] = onset
                    self._min[m] = self.params.min_green_ms(
                        movement_type(m), self.sensors.extended_initial_required(m))
                    self._max_start[m] = None
            elif st[m] is not FLASHING_GREEN:
                self._onset[m], self._min[m], self._max_start[m] = None, None, None
        for m in MOVES:
            if st[m] is GREEN and self._max_start[m] is None and any(
                    self._has_call(c, st) for c in MOVES if not compatible(m, c)):
                self._max_start[m] = now

    def _has_call(self, m: str, st) -> bool:
        if st[m] in _IN_SERVICE:
            return False
        return self.sensors.has_call(m) or self._fault_since[m] is not None

    def _call_ms(self, m: str, st) -> int:
        times = []
        if self.sensors.has_call(m):
            times.append(self.sensors.call_since_ms(m))
        if self._fault_since[m] is not None:
            end = self._service_end[m]
            times.append(self._fault_since[m] if end is None else max(self._fault_since[m], end))
        return min(times)

    def _qhat(self, m: str) -> int:
        q = self.sensors.qhat(m) if self.sensors.has_call(m) else 0
        if self._fault_since[m] is not None:
            q = max(q, self.params.fault_qhat)
        return q

    def _hold(self, waiting: Sequence[str], wait: Dict[str, int]) -> Optional[str]:
        if not self.params.use_w_max:
            return None
        over = [m for m in waiting if wait[m] >= self.params.w_max_ms]
        if not over:
            return None
        return max(over, key=lambda m: (wait[m], -MOVES.index(m)))

    def _gapped(self, m: str) -> bool:
        return self._fault_since[m] is None and self.sensors.gap_out(m)

    def _priority(self, m: str, wait: Dict[str, int]) -> int:
        """P(m) * tau_ms, exact integer."""
        return self._qhat(m) * self.params.tau_ms + (wait[m] if self.params.use_aging else 0)

    def _target(self, waiting, wait, hold, st) -> Tuple[str, ...]:
        best, best_key = None, None
        bonus = self.params.green_bonus_qhat * self.params.tau_ms
        for idx, g in enumerate(GROUP_ORDER):
            if hold is not None and hold not in g:
                continue
            members_waiting = [m for m in g if m in wait]
            serving = [m for m in g if st[m] in _SERVING]
            score = sum(self._priority(m, wait) for m in members_waiting)
            score += bonus * sum(1 for m in serving if not (st[m] is GREEN and self._gapped(m)))
            key = (score, len(serving), max((wait[m] for m in members_waiting), default=-1), -idx)
            if best_key is None or key > best_key:
                best, best_key = g, key
        return best

    def _switch_reasons(self, to_start, hold, st, now: int):
        """[(green to stop, reason)] if every serving movement that conflicts with to_start is
        GREEN and releasable now; None otherwise."""
        blockers = [x for x in MOVES if st[x] in _SERVING
                    and any(not compatible(x, s) for s in to_start)]
        if any(st[x] is RED_YELLOW for x in blockers):
            return None                                     # wait until it is GREEN and releasable
        reasons = [(g, self._release_reason(g, hold, now)) for g in blockers]
        if any(r is None for _, r in reasons):
            return None
        return reasons

    def _release_reason(self, g: str, hold: Optional[str], now: int) -> Optional[str]:
        onset, min_g = self._onset[g], self._min[g]
        if onset is None or now < onset + min_g - FLASHING_GREEN_MS:
            return None
        if hold is not None and not compatible(g, hold):
            return "hold"
        ms = self._max_start[g]
        if self.params.use_max_green and ms is not None and \
                now >= ms + self.params.max_green_ms(movement_type(g)) - FLASHING_GREEN_MS:
            return "max"
        if self._gapped(g):
            return "gap"
        return None

    def _finish_commit(self, bank, now: int, issued: List[Command]) -> None:
        stopped, to_start = self._commit
        for m in to_start:
            if bank.state(m) is RED and bank.can_start_green(m):
                self._start(bank, m, now, issued)
        if all(bank.state(m) not in (RED, RED_YELLOW) for m in to_start):
            self._commit = None

    def _start(self, bank, m: str, now: int, issued: List[Command]) -> None:
        bank.start_green(m)
        self._log(now, START_GREEN, m, issued)

    def _log(self, now: int, kind: str, m: str, issued: List[Command]) -> None:
        c = Command(now, kind, m)
        self.command_log.append(c)
        issued.append(c)
