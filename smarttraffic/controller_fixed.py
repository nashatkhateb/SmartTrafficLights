"""controller_fixed -- fixed-time baseline controller (FT-A / FT-B / time-of-day).

Spec: docs/Eight-Movement_Intersection_Study.html, §8 (baseline), §6 (intervals),
§13 (module table); docs/ARCHITECTURE.md ("Green-time convention", "Stage 10").
Responsibility: follow a timing plan; issue start_green / end_green on time.
Must not: read sensors (SensorBank, detectors, hardware), vehicles or queues.

What "fixed-time" means here
----------------------------
The schedule is a pure function of (plan schedule, start time). Phases run in a
fixed order and are never skipped, shortened or extended:

    φA NL+SL -> φB NS+SS -> φC EL+WL -> φD ES+WS -> φA ...

For phase k starting at s (both movements RED, all conflicting movements RED):

    s                    start_green(m) for each movement of the phase  (RED -> RED_YELLOW)
    s + RY               GREEN (SignalBank timer)
    s + RY + G - FG      end_green(m)                                   (GREEN -> FLASHING_GREEN)
    s + RY + G           YELLOW (SignalBank timer)
    s + RY + G + Y + AR  every movement of the phase is RED -> next phase starts

G is the plan's green time (GREEN + FLASHING_GREEN, docs/ARCHITECTURE.md); RY, FG,
Y and AR are SignalBank's own constants (read from signals.py, never copied).
Where a phase mixes movement types, the next phase waits for the longest Y + AR.

SignalBank stays authoritative: the controller only calls start_green and
end_green, and SignalBank still refuses any unsafe or out-of-order command.

Time: integer milliseconds. A command must be issued exactly at its time, so a
runner must not advance SignalBank past next_command_ms(); update() raises
ScheduleOvershootError if it did (a late command cannot be caught up without
skipping safety intervals). advance(bank, dt) splits any dt at command times.

Plans live in config/fixed_plans.json. A PlanSchedule may switch plans
(S6 time-of-day): the plan in force when a phase STARTS gives that phase's green,
so the phase order is never broken and no phase is cut.

This module has no dependency on sensors, vehicles, hardware, pygame or the wall clock.
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, List, Mapping, NamedTuple, Optional, Sequence, Tuple, Union

from smarttraffic import conflicts
from smarttraffic.conflicts import MOVES, validate_movement
from smarttraffic.signals import (ALL_RED_MS, FLASHING_GREEN_MS, MS_PER_S, RED, RED_YELLOW_MS,
                                  YELLOW_MS, movement_type)

__all__ = [
    "FixedPlanError", "ScheduleOvershootError", "Phase", "FixedPlan", "PlanSchedule",
    "Command", "PhaseRecord", "FixedTimeController", "FixedPlanBook",
    "load_fixed_plans", "DEFAULT_PLANS_FILE", "START_GREEN", "END_GREEN",
]

DEFAULT_PLANS_FILE = Path(__file__).resolve().parent.parent / "config" / "fixed_plans.json"
START_GREEN = "start_green"
END_GREEN = "end_green"


class FixedPlanError(ValueError):
    """A phase list, plan, schedule or plan file is invalid."""


class ScheduleOvershootError(RuntimeError):
    """SignalBank was advanced past a command time; the fixed schedule would drift."""


def _ms(seconds: object, what: str) -> int:
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
        raise FixedPlanError(f"{what} must be a number of seconds, got {seconds!r}")
    if not math.isfinite(seconds):
        raise FixedPlanError(f"{what} must be finite, got {seconds!r}")
    ms = round(seconds * MS_PER_S)
    if abs(seconds * MS_PER_S - ms) > 1e-6:
        raise FixedPlanError(f"{what} must be a whole number of milliseconds, got {seconds!r}")
    return ms


def _bank_ms(bank) -> int:
    return round(bank.now * MS_PER_S)


# ================================================================ plans
@dataclass(frozen=True)
class Phase:
    name: str
    movements: Tuple[str, ...]

    @property
    def clearance_ms(self) -> int:
        """Longest YELLOW + ALL_RED of the phase's movements (SignalBank constants)."""
        return max(YELLOW_MS[movement_type(m)] + ALL_RED_MS[movement_type(m)] for m in self.movements)


def validate_phases(phases: Sequence[Phase]) -> Tuple[Phase, ...]:
    """Every phase compatible (conflicts API), every movement served exactly once per cycle."""
    phases = tuple(phases)
    if not phases:
        raise FixedPlanError("a plan needs at least one phase")
    names = [p.name for p in phases]
    if len(set(names)) != len(names) or not all(isinstance(n, str) and n for n in names):
        raise FixedPlanError(f"phase names must be unique non-empty strings, got {names}")
    seen: Dict[str, str] = {}
    for p in phases:
        if not p.movements:
            raise FixedPlanError(f"phase {p.name} has no movements")
        try:
            for m in p.movements:
                validate_movement(m)
        except conflicts.InvalidMovementError as e:
            raise FixedPlanError(f"phase {p.name}: {e}") from None
        if len(set(p.movements)) != len(p.movements):
            raise FixedPlanError(f"phase {p.name} lists a movement twice: {p.movements}")
        bad = conflicts.conflicting_pairs(p.movements)
        if bad:
            raise FixedPlanError(f"phase {p.name} has conflicting movements: "
                                 + ", ".join(f"{a}-{b}" for a, b in bad))
        for m in p.movements:
            if m in seen:
                raise FixedPlanError(f"{m} is in phase {seen[m]} and phase {p.name}")
            seen[m] = p.name
    missing = [m for m in MOVES if m not in seen]
    if missing:
        raise FixedPlanError(f"movements never served by the plan: {', '.join(missing)}")
    return phases


@dataclass(frozen=True)
class FixedPlan:
    """One timing plan: fixed phase order and a green time G (ms) per phase."""
    name: str
    phases: Tuple[Phase, ...]
    greens_ms: Tuple[int, ...]

    def __post_init__(self):
        validate_phases(self.phases)
        if len(self.greens_ms) != len(self.phases):
            raise FixedPlanError(f"plan {self.name}: {len(self.greens_ms)} greens for {len(self.phases)} phases")
        for p, g in zip(self.phases, self.greens_ms):
            if isinstance(g, bool) or not isinstance(g, int):
                raise FixedPlanError(f"plan {self.name}: green of {p.name} must be int ms, got {g!r}")
            if g < FLASHING_GREEN_MS:
                raise FixedPlanError(f"plan {self.name}: green of {p.name} ({g} ms) is shorter than "
                                     f"FLASHING_GREEN ({FLASHING_GREEN_MS} ms), which is part of G")

    @classmethod
    def from_seconds(cls, name: str, phases: Sequence[Phase], greens_s: Mapping[str, float],
                     min_green_s: Optional[Mapping[str, float]] = None) -> "FixedPlan":
        phases = validate_phases(phases)
        if set(greens_s) != {p.name for p in phases}:
            raise FixedPlanError(f"plan {name}: greens must be given for exactly the phases "
                                 f"{[p.name for p in phases]}, got {sorted(greens_s)}")
        greens = tuple(_ms(greens_s[p.name], f"plan {name} green {p.name}") for p in phases)
        if min_green_s is not None:
            for p, g in zip(phases, greens):
                need = max(_ms(min_green_s[movement_type(m)], "min green") for m in p.movements)
                if g < need:
                    raise FixedPlanError(f"plan {name}: green of {p.name} ({g} ms) is below the "
                                         f"minimum green {need} ms")
        return cls(name, phases, greens)

    def green_ms(self, phase_name: str) -> int:
        for p, g in zip(self.phases, self.greens_ms):
            if p.name == phase_name:
                return g
        raise FixedPlanError(f"plan {self.name} has no phase {phase_name!r}")

    def phase_duration_ms(self, i: int) -> int:
        """Phase start to next phase start: RY + G + longest (Y + AR)."""
        return RED_YELLOW_MS + self.greens_ms[i] + self.phases[i].clearance_ms

    @property
    def cycle_ms(self) -> int:
        """Actual cycle, including the RED_YELLOW before every phase."""
        return sum(self.phase_duration_ms(i) for i in range(len(self.phases)))

    @property
    def lost_time_ms(self) -> int:
        """Study lost time L = sum of the phases' Y + AR (23 s for the §8 plan)."""
        return sum(p.clearance_ms for p in self.phases)

    @property
    def study_cycle_ms(self) -> int:
        """The study's cycle C = sum(G) + L (without RED_YELLOW)."""
        return sum(self.greens_ms) + self.lost_time_ms


class PlanSchedule:
    """Plans over time. entries = [(from_ms or None, plan), ...]; the first entry has None.

    A phase uses the plan in force at the instant the phase starts."""

    def __init__(self, name: str, entries: Sequence[Tuple[Optional[int], FixedPlan]]):
        entries = tuple(entries)
        if not entries:
            raise FixedPlanError(f"schedule {name} is empty")
        if entries[0][0] is not None:
            raise FixedPlanError(f"schedule {name}: the first entry must apply from the start (from = None)")
        prev = None
        for t, plan in entries[1:]:
            if isinstance(t, bool) or not isinstance(t, int):
                raise FixedPlanError(f"schedule {name}: switch times must be int ms, got {t!r}")
            if prev is not None and t <= prev:
                raise FixedPlanError(f"schedule {name}: switch times must increase")
            prev = t
        for _, plan in entries:
            if not isinstance(plan, FixedPlan):
                raise FixedPlanError(f"schedule {name}: {plan!r} is not a FixedPlan")
            if plan.phases != entries[0][1].phases:
                raise FixedPlanError(f"schedule {name}: all plans must use the same phases and order")
        self.name = name
        self.entries = entries

    @classmethod
    def single(cls, plan: FixedPlan) -> "PlanSchedule":
        return cls(plan.name, [(None, plan)])

    @property
    def phases(self) -> Tuple[Phase, ...]:
        return self.entries[0][1].phases

    def plan_at(self, t_ms: int) -> FixedPlan:
        plan = self.entries[0][1]
        for t, p in self.entries[1:]:
            if t <= t_ms:
                plan = p
        return plan

    def __repr__(self):
        return f"PlanSchedule({self.name!r}, {[(t, p.name) for t, p in self.entries]})"


# =========================================================== timetable
class Command(NamedTuple):
    time_ms: int
    kind: str            # START_GREEN or END_GREEN
    movement: str


class PhaseRecord(NamedTuple):
    index: int           # 0, 1, 2, ... since the controller started
    phase: str
    movements: Tuple[str, ...]
    plan: str
    start_ms: int        # start_green (RED -> RED_YELLOW)
    green_ms: int        # RED_YELLOW -> GREEN
    end_green_ms: int    # end_green (GREEN -> FLASHING_GREEN)
    yellow_ms: int       # FLASHING_GREEN -> YELLOW (= green_ms + G)
    next_start_ms: int   # every movement of the phase is RED again


def _phase_records(schedule: PlanSchedule, start_ms: int) -> Iterator[PhaseRecord]:
    k, s = 0, start_ms
    n = len(schedule.phases)
    while True:
        plan = schedule.plan_at(s)
        i = k % n
        ph, g = plan.phases[i], plan.greens_ms[i]
        green = s + RED_YELLOW_MS
        rec = PhaseRecord(k, ph.name, ph.movements, plan.name, s, green,
                          green + g - FLASHING_GREEN_MS, green + g, s + plan.phase_duration_ms(i))
        yield rec
        k, s = k + 1, rec.next_start_ms


def _commands(rec: PhaseRecord) -> List[Command]:
    return ([Command(rec.start_ms, START_GREEN, m) for m in rec.movements]
            + [Command(rec.end_green_ms, END_GREEN, m) for m in rec.movements])


# ========================================================== controller
class FixedTimeController:
    """Fixed-time controller. Takes no sensor input by construction.

    Per simulation step, call update(bank) at the controller point of the step
    (after vehicles and sensors, before bank.advance). Never advance the bank
    past next_command_ms(); or use advance(bank, dt) to do that automatically.
    One controller per run: build a new one to restart.
    """
    uses_sensors = False

    def __init__(self, schedule: Union[PlanSchedule, FixedPlan], start_time: float = 0.0):
        if isinstance(schedule, FixedPlan):
            schedule = PlanSchedule.single(schedule)
        if not isinstance(schedule, PlanSchedule):
            raise FixedPlanError(f"expected a PlanSchedule or FixedPlan, got {schedule!r}")
        self.schedule = schedule
        self.name = schedule.name
        self.start_ms = _ms(start_time, "start_time")
        self._records = _phase_records(schedule, self.start_ms)
        self._pending: List[Command] = []
        self._current: Optional[PhaseRecord] = None
        self._next: PhaseRecord = next(self._records)
        self._last_update: Optional[int] = None
        self.phase_log: List[PhaseRecord] = []
        self.command_log: List[Command] = []

    # ---------------------------------------------------------- queries
    def next_command_ms(self) -> int:
        """Time of the next command still to be issued."""
        return self._pending[0].time_ms if self._pending else self._next.start_ms

    def current_phase(self) -> Optional[PhaseRecord]:
        """The phase started most recently (None before the first start)."""
        return self._current

    @staticmethod
    def timetable(schedule: Union[PlanSchedule, FixedPlan], start_time: float,
                  until_time: float) -> Tuple[List[PhaseRecord], List[Command]]:
        """Phases starting before until_time and their commands (pure, no SignalBank)."""
        if isinstance(schedule, FixedPlan):
            schedule = PlanSchedule.single(schedule)
        until = _ms(until_time, "until_time")
        recs, cmds = [], []
        for r in _phase_records(schedule, _ms(start_time, "start_time")):
            if r.start_ms >= until:
                break
            recs.append(r)
            cmds += [c for c in _commands(r) if c.time_ms < until]
        return recs, cmds

    # --------------------------------------------------------- commands
    def update(self, bank) -> List[Command]:
        """Issue every command due at bank.now. Returns the commands issued."""
        now = _bank_ms(bank)
        if self._last_update is not None and now < self._last_update:
            raise ValueError(f"time went backwards ({now} < {self._last_update} ms)")
        self._last_update = now
        issued: List[Command] = []
        while True:
            if not self._pending:
                if self._next.start_ms > now:
                    break
                self._begin_phase(bank, now)
            c = self._pending[0]
            if c.time_ms > now:
                break
            if c.time_ms < now:
                raise ScheduleOvershootError(
                    f"{self.name}: {c.kind}({c.movement}) was due at {c.time_ms} ms but SignalBank is "
                    f"already at {now} ms; do not advance past next_command_ms()")
            if c.kind == START_GREEN:
                bank.start_green(c.movement)
            else:
                bank.end_green(c.movement)
            self._pending.pop(0)
            self.command_log.append(c)
            issued.append(c)
        return issued

    def advance(self, bank, dt: float) -> List[Command]:
        """Advance SignalBank by dt, issuing every command at its exact time
        (for controller + SignalBank only; a full simulation step does the same
        by capping its dt at next_command_ms())."""
        dt_ms = _ms(dt, "dt")
        if dt_ms < 0:
            raise ValueError(f"dt must not be negative, got {dt!r}")
        target = _bank_ms(bank) + dt_ms
        issued = self.update(bank)
        while _bank_ms(bank) < target:
            step = min(target, self.next_command_ms()) - _bank_ms(bank)
            bank.advance(step / MS_PER_S)
            issued += self.update(bank)
        return issued

    # --------------------------------------------------------- internal
    def _begin_phase(self, bank, now: int) -> None:
        rec = self._next
        if rec.start_ms < now:
            raise ScheduleOvershootError(
                f"{self.name}: phase {rec.phase} was due at {rec.start_ms} ms but SignalBank is "
                f"already at {now} ms; do not advance past next_command_ms()")
        if rec.index == 0:
            not_red = [m for m in MOVES if bank.state(m) is not RED]
            if not_red:
                raise FixedPlanError(f"{self.name} must start from an all-RED intersection; "
                                     f"not RED: {', '.join(not_red)}")
        self._pending = _commands(rec)
        self._current = rec
        self.phase_log.append(rec)
        self._next = next(self._records)


# ============================================================ plan file
def _no_duplicate_keys(pairs):
    out = {}
    for k, v in pairs:
        if k in out:
            raise FixedPlanError(f"duplicate key {k!r} in fixed-plan file")
        out[k] = v
    return out


class FixedPlanBook:
    """Validated contents of config/fixed_plans.json."""

    def __init__(self, raw: Mapping):
        if not isinstance(raw, Mapping):
            raise FixedPlanError("fixed-plan data must be a JSON object")
        if raw.get("version") != 1:
            raise FixedPlanError(f"unsupported fixed-plan version {raw.get('version')!r}")
        try:
            phases = tuple(Phase(p["name"], tuple(p["movements"])) for p in raw["phases"])
            min_green = dict(raw["min_green_s"])
            plans_raw, base_raw = raw["plans"], raw["baselines"]
        except (KeyError, TypeError) as e:
            raise FixedPlanError(f"fixed-plan file is missing or has a malformed field: {e}") from None
        self.phases = validate_phases(phases)
        if set(min_green) != {"straight", "left"}:
            raise FixedPlanError("min_green_s needs exactly 'straight' and 'left'")
        self.plans: Dict[str, FixedPlan] = {}
        for name, p in plans_raw.items():
            if not isinstance(p, Mapping) or "greens_s" not in p or "study_cycle_s" not in p:
                raise FixedPlanError(f"plan {name} needs 'greens_s' and 'study_cycle_s'")
            plan = FixedPlan.from_seconds(name, self.phases, p["greens_s"], min_green)
            study = _ms(p["study_cycle_s"], f"plan {name} study_cycle_s")
            if plan.study_cycle_ms != study:
                raise FixedPlanError(f"plan {name}: sum of greens + L = {plan.study_cycle_ms} ms "
                                     f"but study_cycle_s says {study} ms")
            self.plans[name] = plan
        self._baselines: Dict[str, Dict[str, PlanSchedule]] = {}
        for base, by_scn in base_raw.items():
            self._baselines[base] = {}
            for scn, entries in by_scn.items():
                sched = []
                for e in entries:
                    if e.get("plan") not in self.plans:
                        raise FixedPlanError(f"{base}/{scn}: unknown plan {e.get('plan')!r}")
                    t = None if e.get("from_s") is None else _ms(e["from_s"], f"{base}/{scn} from_s")
                    sched.append((t, self.plans[e["plan"]]))
                label = base if scn == "*" else f"{base}/{scn}"
                self._baselines[base][scn] = PlanSchedule(label, sched)

    @property
    def baselines(self) -> Tuple[str, ...]:
        return tuple(self._baselines)

    def scenarios(self, baseline: str) -> Tuple[str, ...]:
        return tuple(self._baselines[baseline])

    def schedule(self, baseline: str, scenario: str) -> PlanSchedule:
        """The plan schedule of a baseline for a scenario ("*" entries apply to every scenario)."""
        if baseline not in self._baselines:
            raise FixedPlanError(f"unknown baseline {baseline!r}; known: {', '.join(self._baselines)}")
        by = self._baselines[baseline]
        if scenario in by:
            return by[scenario]
        if "*" in by:
            return by["*"]
        raise FixedPlanError(f"baseline {baseline} has no plan for scenario {scenario!r}")

    def controller(self, baseline: str, scenario: str, start_time: float = 0.0) -> FixedTimeController:
        return FixedTimeController(self.schedule(baseline, scenario), start_time)


def load_fixed_plans(path: Union[str, Path] = DEFAULT_PLANS_FILE) -> FixedPlanBook:
    with open(path, encoding="utf-8") as f:
        try:
            raw = json.load(f, object_pairs_hook=_no_duplicate_keys)
        except json.JSONDecodeError as e:
            raise FixedPlanError(f"{path}: invalid JSON: {e}") from None
    return FixedPlanBook(raw)
