"""Tests for smarttraffic.controller_fixed (Stage 10: fixed-time baseline).

The controller is checked against SignalBank (the authority on intervals and
safety) and against full simulations with detector activity, which must not
change the schedule at all.
"""
import ast
import copy
import inspect
import json
import random
from pathlib import Path

import pytest

from smarttraffic import conflicts
from smarttraffic import controller_fixed as cf
from smarttraffic import signals as sg
from smarttraffic.conflicts import MOVES
from smarttraffic.controller_fixed import (END_GREEN, START_GREEN, Command, FixedPlan, FixedPlanError,
                                           FixedTimeController, Phase, PlanSchedule,
                                           ScheduleOvershootError, load_fixed_plans)
from smarttraffic.signals import (ALL_RED, FLASHING_GREEN, GREEN, RED, RED_YELLOW, YELLOW,
                                  IllegalTransition, SignalBank)

ROOT = Path(__file__).resolve().parent.parent
PLANS_FILE = ROOT / "config" / "fixed_plans.json"
STUDY_PHASES = [("A", ("NL", "SL")), ("B", ("NS", "SS")), ("C", ("EL", "WL")), ("D", ("ES", "WS"))]

# study §8 table: plan -> (A, B, C, D, study cycle)
STUDY_TABLE = {
    "FT-A": (8, 25, 8, 25, 89),
    "FT-B/S1": (8, 25, 8, 25, 89), "FT-B/S2": (14, 47, 9, 27, 120), "FT-B/S3": (8, 25, 8, 25, 89),
    "FT-B/S4": (5, 14, 5, 14, 61), "FT-B/S5": (8, 25, 8, 25, 89), "FT-B/S6": (6, 22, 6, 22, 79),
    "FT-B/S7a": (11, 37, 11, 37, 119), "FT-B/S7b": (12, 36, 12, 36, 119),
    "FT-B/S8": (24, 17, 24, 17, 105), "FT-B/S9": (19, 32, 17, 30, 121),
    "FT-B/S10": (11, 37, 11, 37, 119),
    "TOD/S6/first": (9, 35, 5, 9, 81), "TOD/S6/second": (5, 9, 9, 35, 81),
}


@pytest.fixture(scope="module")
def book():
    return load_fixed_plans()


@pytest.fixture(scope="module")
def fta(book):
    return book.plans["FT-A"]


def raw_plans():
    return json.loads(PLANS_FILE.read_text(encoding="utf-8"))


def study_phases():
    return tuple(Phase(n, m) for n, m in STUDY_PHASES)


def run_steps(ctrl, bank, seconds, dt_ms=100, each=None):
    """The simulation step pattern: controller at t, then SignalBank advances dt."""
    t_end = round(bank.now * 1000) + seconds * 1000
    while round(bank.now * 1000) < t_end:
        ctrl.update(bank)
        if each:
            each(bank)
        bank.advance(dt_ms / 1000)
    ctrl.update(bank)


def transitions(bank):
    return [(round(e.time * 1000), e.movement, e.old, e.new) for e in bank.log]


# ======================================================== plan file / study values
def test_plan_file_phases_are_the_study_phases(book):
    assert [(p.name, p.movements) for p in book.phases] == STUDY_PHASES


def test_every_phase_pair_is_compatible_in_the_conflict_model(book):
    for p in book.phases:
        assert conflicts.is_allowed_set(p.movements)
        assert frozenset(p.movements) in conflicts.GROUPS


def test_every_movement_is_served_exactly_once_per_cycle(book):
    served = [m for p in book.phases for m in p.movements]
    assert sorted(served) == sorted(MOVES) and len(served) == 8


def test_consecutive_phases_really_conflict(book):
    """Each phase must wait for the previous one's clearance (why phases are sequential)."""
    ph = book.phases
    for i, p in enumerate(ph):
        nxt = ph[(i + 1) % len(ph)]
        for m in nxt.movements:
            assert any(not conflicts.compatible(m, x) for x in p.movements)


@pytest.mark.parametrize("name", sorted(STUDY_TABLE))
def test_plan_greens_match_study_table(book, name):
    a, b, c, d, cycle = STUDY_TABLE[name]
    plan = book.plans[name]
    assert plan.greens_ms == (a * 1000, b * 1000, c * 1000, d * 1000)
    assert plan.study_cycle_ms == cycle * 1000
    assert plan.cycle_ms == cycle * 1000 + 4 * sg.RED_YELLOW_MS


def test_no_unexpected_plans(book):
    assert set(book.plans) == set(STUDY_TABLE)


def test_fta_timing_summary(fta):
    assert fta.lost_time_ms == 23000
    assert fta.study_cycle_ms == 89000
    assert fta.cycle_ms == 93000
    assert [fta.phase_duration_ms(i) for i in range(4)] == [15000, 31500, 15000, 31500]


def test_phase_timing_comes_from_signalbank_constants(fta):
    for i, p in enumerate(fta.phases):
        t = sg.movement_type(p.movements[0])
        assert fta.phase_duration_ms(i) == (sg.RED_YELLOW_MS + fta.greens_ms[i]
                                            + sg.YELLOW_MS[t] + sg.ALL_RED_MS[t])


def test_controller_module_copies_no_interval_values():
    src = inspect.getsource(cf)
    tree = ast.parse(src)
    numbers = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)
               and isinstance(n.value, (int, float)) and not isinstance(n.value, bool)}
    assert not numbers & {3.5, 3500, 2.0, 2000, 3000, 3.0}, numbers


def test_ftb_covers_every_scenario_and_fta_every_scenario(book):
    scenarios = json.loads((ROOT / "config" / "scenarios.json").read_text(encoding="utf-8"))["scenarios"]
    assert set(book.scenarios("FT-B")) == set(scenarios)
    for s in scenarios:
        assert book.schedule("FT-A", s).plan_at(0).name == "FT-A"
        assert book.schedule("FT-B", s).plan_at(0).name == f"FT-B/{s}"
    assert set(book.baselines) == {"FT-A", "FT-B", "FT-TOD"}


def test_tod_schedule_is_s6_only(book):
    sched = book.schedule("FT-TOD", "S6")
    assert [(t, p.name) for t, p in sched.entries] == [(None, "TOD/S6/first"), (1800000, "TOD/S6/second")]
    with pytest.raises(FixedPlanError):
        book.schedule("FT-TOD", "S1")
    with pytest.raises(FixedPlanError):
        book.schedule("FT-X", "S1")


# ============================================================== validation
def _book_from(raw):
    return cf.FixedPlanBook(raw)


@pytest.mark.parametrize("phases, msg", [
    ([["NL", "SS"], ["NS", "SL"], ["EL", "WL"], ["ES", "WS"]], "conflicting"),
    ([["NL", "SL"], ["NS", "SS"], ["EL", "WL"], ["ES"]], "never served"),
    ([["NL", "SL"], ["NS", "NL"], ["EL", "WL"], ["ES", "WS"]], "phase A and phase B"),
    ([["NL", "SL"], ["NS", "NS"], ["EL", "WL"], ["ES", "WS", "SS"]], "twice"),
    ([["NL", "SL"], ["NS", "SS"], ["EL", "WL"], ["ES", "XX"]], "unknown movement"),
    ([["NL", "SL"], [], ["EL", "WL"], ["ES", "WS", "NS", "SS"]], "no movements"),
])
def test_invalid_phases_rejected(phases, msg):
    raw = raw_plans()
    for p, mv in zip(raw["phases"], phases):
        p["movements"] = mv
    with pytest.raises(FixedPlanError, match=msg):
        _book_from(raw)


def test_real_plan_file_has_no_conflicting_phase_even_if_rewritten():
    """Every compatible rearrangement is accepted; every conflicting one refused."""
    for a, b in [("NL", "NS"), ("SS", "SL")]:
        assert conflicts.compatible(a, b)
    Phase("A", ("NL", "NS"))
    with pytest.raises(FixedPlanError, match="conflicting"):
        FixedPlan.from_seconds("x", [Phase("A", ("NL", "SS")), Phase("B", ("NS", "SL")),
                                     Phase("C", ("EL", "WL")), Phase("D", ("ES", "WS"))],
                               {"A": 8, "B": 25, "C": 8, "D": 25})


@pytest.mark.parametrize("green, msg", [(2.999, "minimum green"), (4, "minimum green"),
                                         (8.0005, "whole number"), ("8", "number"), (True, "number")])
def test_invalid_greens_rejected(green, msg):
    raw = raw_plans()
    raw["plans"]["FT-A"]["greens_s"]["A"] = green
    with pytest.raises(FixedPlanError, match=msg):
        _book_from(raw)


def test_green_shorter_than_flashing_green_rejected():
    with pytest.raises(FixedPlanError, match="FLASHING_GREEN"):
        FixedPlan("x", study_phases(), (2999, 25000, 8000, 25000))
    with pytest.raises(FixedPlanError, match="FLASHING_GREEN"):
        FixedPlan.from_seconds("x", study_phases(), {"A": 2.9, "B": 25, "C": 8, "D": 25})


def test_straight_min_green_is_enforced():
    raw = raw_plans()
    raw["plans"]["FT-A"]["greens_s"]["B"] = 6
    raw["plans"]["FT-A"]["study_cycle_s"] = 70
    with pytest.raises(FixedPlanError, match="minimum green 7000"):
        _book_from(raw)


def test_study_cycle_mismatch_rejected():
    raw = raw_plans()
    raw["plans"]["FT-B/S9"]["greens_s"]["C"] = 18
    with pytest.raises(FixedPlanError, match="study_cycle_s"):
        _book_from(raw)


@pytest.mark.parametrize("mutate, msg", [
    (lambda r: r["plans"]["FT-A"]["greens_s"].pop("D"), "exactly the phases"),
    (lambda r: r["plans"]["FT-A"]["greens_s"].update(E=5), "exactly the phases"),
    (lambda r: r["baselines"]["FT-A"]["*"][0].update(plan="nope"), "unknown plan"),
    (lambda r: r["baselines"]["FT-TOD"]["S6"][0].update(from_s=0), "first entry"),
    (lambda r: r["baselines"]["FT-TOD"]["S6"].append({"from_s": 900, "plan": "FT-A"}), "increase"),
    (lambda r: r.update(version=2), "version"),
    (lambda r: r.pop("phases"), "missing"),
    (lambda r: r["plans"]["FT-A"].pop("study_cycle_s"), "study_cycle_s"),
    (lambda r: r.update(min_green_s={"straight": 7}), "min_green_s"),
])
def test_malformed_plan_data_rejected(mutate, msg):
    raw = raw_plans()
    mutate(raw)
    with pytest.raises(FixedPlanError, match=msg):
        _book_from(raw)


def test_duplicate_json_keys_rejected(tmp_path):
    text = PLANS_FILE.read_text(encoding="utf-8").replace('"version": 1,', '"version": 1, "version": 1,')
    p = tmp_path / "plans.json"
    p.write_text(text, encoding="utf-8")
    with pytest.raises(FixedPlanError, match="duplicate"):
        load_fixed_plans(p)


def test_schedule_plans_must_share_phase_order(fta):
    other = FixedPlan.from_seconds("rev", tuple(reversed(fta.phases)), {"A": 8, "B": 25, "C": 8, "D": 25})
    with pytest.raises(FixedPlanError, match="same phases"):
        PlanSchedule("x", [(None, fta), (1000, other)])


def test_plan_objects_are_immutable(fta):
    with pytest.raises(Exception):
        fta.greens_ms = (1, 2, 3, 4)


# ============================================================ exact cycle
def expected_fta_cycle(t0=0):
    """Every SignalBank transition of one FT-A cycle, hand-written from §6/§8."""
    ev = []
    for start, moves, g, typ in [(0, ("NL", "SL"), 8000, "left"), (15000, ("NS", "SS"), 25000, "straight"),
                                 (46500, ("EL", "WL"), 8000, "left"), (61500, ("ES", "WS"), 25000, "straight")]:
        y, ar = {"left": (3000, 3000), "straight": (3500, 2000)}[typ]
        s = t0 + start
        for new, old, at in [(RED_YELLOW, RED, s), (GREEN, RED_YELLOW, s + 1000),
                             (FLASHING_GREEN, GREEN, s + 1000 + g - 3000), (YELLOW, FLASHING_GREEN, s + 1000 + g),
                             (ALL_RED, YELLOW, s + 1000 + g + y), (RED, ALL_RED, s + 1000 + g + y + ar)]:
            for m in moves:
                ev.append((at, m, old, new))
    return sorted(ev, key=lambda e: (e[0], [RED, ALL_RED, YELLOW, FLASHING_GREEN, GREEN, RED_YELLOW].index(e[3]),
                                     MOVES.index(e[1])))


def test_one_full_fta_cycle_exact_transitions(fta):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    run_steps(ctrl, bank, 93)
    got = transitions(bank)
    exp = expected_fta_cycle()
    # at t = 93 s phase A of cycle 2 has started too
    assert got[:len(exp)] == exp
    assert got[len(exp):] == [(93000, "NL", RED, RED_YELLOW), (93000, "SL", RED, RED_YELLOW)]


def test_phase_order_over_many_cycles(fta):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    run_steps(ctrl, bank, 5 * 93)
    names = [r.phase for r in ctrl.phase_log]
    assert names == list("ABCD" * 5) + ["A"]
    for r in ctrl.phase_log:
        assert r.movements == dict(STUDY_PHASES)[r.phase]
        assert r.plan == "FT-A"
    starts = [r.start_ms for r in ctrl.phase_log]
    assert starts[::4] == [k * 93000 for k in range(6)]


def test_commands_are_exactly_start_and_end_green_in_pairs(fta):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    run_steps(ctrl, bank, 2 * 93 - 1)
    log = ctrl.command_log
    assert {c.kind for c in log} == {START_GREEN, END_GREEN}
    for i in range(0, len(log), 2):
        a, b = log[i], log[i + 1]
        assert a.time_ms == b.time_ms and a.kind == b.kind
        assert frozenset((a.movement, b.movement)) in conflicts.GROUPS
    per_move = {m: [c.kind for c in log if c.movement == m] for m in MOVES}
    for m, kinds in per_move.items():
        assert kinds == [START_GREEN, END_GREEN] * 2, (m, kinds)


@pytest.mark.parametrize("t_ms, m, state", [
    (0, "NL", RED_YELLOW), (999, "SL", RED_YELLOW), (1000, "SL", GREEN), (5999, "NL", GREEN),
    (6000, "NL", FLASHING_GREEN), (8999, "SL", FLASHING_GREEN), (9000, "SL", YELLOW),
    (11999, "NL", YELLOW), (12000, "NL", ALL_RED), (14999, "SL", ALL_RED), (15000, "SL", RED),
    (14999, "NS", RED), (15000, "NS", RED_YELLOW), (16000, "SS", GREEN), (37999, "NS", GREEN),
    (38000, "NS", FLASHING_GREEN), (41000, "SS", YELLOW), (44499, "SS", YELLOW), (44500, "SS", ALL_RED),
    (46499, "NS", ALL_RED), (46499, "EL", RED), (46500, "NS", RED), (46500, "WL", RED_YELLOW),
    (61499, "ES", RED), (61500, "WS", RED_YELLOW), (84499, "ES", GREEN), (84500, "ES", FLASHING_GREEN),
    (92999, "WS", ALL_RED), (92999, "NL", RED), (93000, "NL", RED_YELLOW), (93000, "WS", RED),
])
def test_exact_millisecond_boundaries(fta, t_ms, m, state):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    ctrl.advance(bank, t_ms / 1000)
    assert bank.state(m) is state


def test_state_at_every_100ms_matches_timetable(fta):
    """Walk 2 cycles at dt = 0.1 s and compare with the pure timetable each step."""
    recs, _ = FixedTimeController.timetable(fta, 0, 2 * 93)

    def expected(m, t):
        for r in recs:
            if m in r.movements and r.start_ms <= t < r.next_start_ms:
                typ = sg.movement_type(m)
                if t < r.green_ms:
                    return RED_YELLOW
                if t < r.end_green_ms:
                    return GREEN
                if t < r.yellow_ms:
                    return FLASHING_GREEN
                if t < r.yellow_ms + sg.YELLOW_MS[typ]:
                    return YELLOW
                if t < r.yellow_ms + sg.YELLOW_MS[typ] + sg.ALL_RED_MS[typ]:
                    return ALL_RED
        return RED

    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    for t in range(0, 2 * 93000, 100):
        ctrl.update(bank)
        for m in MOVES:
            assert bank.state(m) is expected(m, t), (t, m)
        bank.advance(0.1)


def test_no_conflicting_movements_are_ever_active(book):
    for name in ("FT-A", "FT-B/S2", "FT-B/S9", "TOD/S6/first"):
        bank = SignalBank()
        ctrl = FixedTimeController(book.plans[name])
        seen = set()

        def check(b):
            act = b.active()
            assert conflicts.is_allowed_set(act), (b.now, act)
            seen.add(frozenset(act))

        run_steps(ctrl, bank, 3 * 130, each=check)
        # only the four phase pairs are ever active together
        assert {s for s in seen if s} <= {frozenset(m) for _, m in STUDY_PHASES}


def test_interval_durations_are_signalbanks(book):
    bank = SignalBank()
    ctrl = FixedTimeController(book.plans["FT-B/S8"])
    run_steps(ctrl, bank, 3 * 110)
    for m in MOVES:
        ev = [e for e in bank.log if e.movement == m]
        typ = sg.movement_type(m)
        for a, b in zip(ev, ev[1:]):
            dur = round((b.time - a.time) * 1000)
            want = {RED_YELLOW: sg.RED_YELLOW_MS, FLASHING_GREEN: sg.FLASHING_GREEN_MS,
                    YELLOW: sg.YELLOW_MS[typ], ALL_RED: sg.ALL_RED_MS[typ]}.get(a.new)
            if want is not None:
                assert dur == want, (m, a.new, dur)


class OnlyCommands:
    """SignalBank proxy: the controller may read time/state and call the two commands only."""
    ALLOWED = {"now", "state", "start_green", "end_green"}

    def __init__(self, bank):
        self._bank = bank
        self.calls = []

    def __getattr__(self, name):
        if name not in self.ALLOWED:
            raise AssertionError(f"controller used SignalBank.{name}")
        attr = getattr(self._bank, name)
        if callable(attr):
            def wrapped(*a):
                self.calls.append((name,) + a)
                return attr(*a)
            return wrapped
        return attr


def test_controller_uses_only_start_green_and_end_green(fta):
    bank = SignalBank()
    proxy = OnlyCommands(bank)
    ctrl = FixedTimeController(fta)
    for _ in range(2 * 930):
        ctrl.update(proxy)
        bank.advance(0.1)
    kinds = {c[0] for c in proxy.calls}
    assert kinds <= {"state", "start_green", "end_green"}
    assert sum(c[0] == "start_green" for c in proxy.calls) == 16


def test_signalbank_still_refuses_unsafe_start(fta):
    """If something else holds a conflicting head, SignalBank refuses; the controller does not bypass it."""
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    ctrl.advance(bank, 14.9)
    bank.advance(0.1)                         # t = 15.0: NL/SL just turned RED, controller not run yet
    assert bank.can_start_green("ES")
    bank.start_green("ES")                    # interference: ES conflicts with NS and SS
    with pytest.raises(IllegalTransition):
        ctrl.update(bank)
    assert bank.state("NS") is RED and bank.state("SS") is RED


def test_signalbank_refuses_end_green_when_state_was_disturbed(fta):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    ctrl.advance(bank, 3.0)
    bank.end_green("NL")                      # someone ended NL early
    with pytest.raises(IllegalTransition):
        ctrl.advance(bank, 3.0)


# ============================================================ initialization
def test_must_start_from_all_red(fta):
    bank = SignalBank()
    bank.start_green("ES")
    with pytest.raises(FixedPlanError, match="all-RED"):
        FixedTimeController(fta).update(bank)


def test_first_command_is_at_start_time(fta):
    ctrl = FixedTimeController(fta, start_time=-300)
    assert ctrl.next_command_ms() == -300000
    assert ctrl.current_phase() is None
    bank = SignalBank(start_time=-310)
    assert ctrl.update(bank) == []
    ctrl.advance(bank, 9.9)
    assert all(s is RED for s in bank.states().values())
    issued = ctrl.advance(bank, 0.1)
    assert [c.movement for c in issued] == ["NL", "SL"]
    assert ctrl.current_phase().phase == "A" and ctrl.current_phase().start_ms == -300000


def test_start_offset_shifts_whole_schedule(fta):
    r0, c0 = FixedTimeController.timetable(fta, 0, 400)
    r1, c1 = FixedTimeController.timetable(fta, -300, 100)
    assert [(c.time_ms + 300000, c.kind, c.movement) for c in c1] == [tuple(c) for c in c0]


def test_update_before_start_and_repeated_update_are_harmless(fta):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    assert len(ctrl.update(bank)) == 2
    assert ctrl.update(bank) == []
    assert ctrl.update(bank) == []
    assert bank.state("NL") is RED_YELLOW


def test_time_going_backwards_is_rejected(fta):
    ctrl2 = FixedTimeController(fta, start_time=10)
    ctrl2.update(SignalBank(start_time=5))
    with pytest.raises(ValueError, match="backwards"):
        ctrl2.update(SignalBank(start_time=4))


def test_bad_constructor_arguments():
    with pytest.raises(FixedPlanError):
        FixedTimeController("FT-A")
    with pytest.raises(FixedPlanError):
        FixedTimeController(load_fixed_plans().plans["FT-A"], start_time=0.0005)


# ================================================== large steps / time jumps
def test_one_huge_advance_equals_fine_steps(fta):
    fine_bank, fine = SignalBank(), FixedTimeController(fta)
    run_steps(fine, fine_bank, 3600)
    big_bank, big = SignalBank(), FixedTimeController(fta)
    big.advance(big_bank, 3600)
    assert transitions(big_bank) == transitions(fine_bank)
    assert big.command_log == fine.command_log
    assert big.phase_log == fine.phase_log


def test_random_step_sizes_equal_fine_steps(book):
    plan = book.plans["FT-B/S9"]
    ref_bank, ref = SignalBank(), FixedTimeController(plan)
    run_steps(ref, ref_bank, 1000)
    rng = random.Random(7)
    bank, ctrl = SignalBank(), FixedTimeController(plan)
    while round(bank.now * 1000) < 1000000:
        dt = min(rng.choice([1, 7, 100, 999, 2500, 30000, 123457]), 1000000 - round(bank.now * 1000))
        ctrl.advance(bank, dt / 1000)
    assert transitions(bank) == transitions(ref_bank)


def test_next_command_capping_like_a_simulation_runner(fta):
    """A runner with dt = 2.3 s that caps each step at next_command_ms() gets the exact schedule."""
    ref_bank, ref = SignalBank(), FixedTimeController(fta)
    ref.advance(ref_bank, 400)
    bank, ctrl = SignalBank(), FixedTimeController(fta)
    while round(bank.now * 1000) < 400000:
        ctrl.update(bank)
        now = round(bank.now * 1000)
        step = min(2300, ctrl.next_command_ms() - now, 400000 - now)
        bank.advance(step / 1000)
    ctrl.update(bank)
    assert transitions(bank) == transitions(ref_bank)


def test_overshooting_a_command_raises_instead_of_drifting(fta):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    ctrl.update(bank)
    assert ctrl.next_command_ms() == 6000
    bank.advance(6.1)
    with pytest.raises(ScheduleOvershootError, match="6000"):
        ctrl.update(bank)


def test_overshooting_a_phase_start_raises(fta):
    bank = SignalBank()
    ctrl = FixedTimeController(fta)
    ctrl.advance(bank, 6.0)
    assert ctrl.next_command_ms() == 15000
    bank.advance(9.1)
    with pytest.raises(ScheduleOvershootError, match="phase B"):
        ctrl.update(bank)


def test_a_full_hour_of_every_plan_runs_cleanly(book):
    for name, plan in book.plans.items():
        bank = SignalBank(start_time=-300)
        ctrl = FixedTimeController(plan, start_time=-300)
        ctrl.advance(bank, 3900)
        cycles = sum(1 for r in ctrl.phase_log if r.phase == "A")
        assert abs(cycles - 3900000 / plan.cycle_ms) <= 1, name


# ============================================================ configurable plans
def test_configurable_greens_change_only_green_lengths(fta):
    phases = study_phases()
    custom = FixedPlan.from_seconds("custom", phases, {"A": 6, "B": 10, "C": 12, "D": 30})
    recs, _ = FixedTimeController.timetable(custom, 0, 200)
    got = [(r.phase, r.end_green_ms - r.green_ms + sg.FLASHING_GREEN_MS) for r in recs[:4]]
    assert got == [("A", 6000), ("B", 10000), ("C", 12000), ("D", 30000)]
    assert custom.cycle_ms == 6000 + 10000 + 12000 + 30000 + 23000 + 4000
    bank = SignalBank()
    FixedTimeController(custom).advance(bank, custom.cycle_ms / 1000)
    assert transitions(bank)[:2] == [(0, "NL", RED, RED_YELLOW), (0, "SL", RED, RED_YELLOW)]


def test_green_equal_to_flashing_green_is_allowed(fta):
    plan = FixedPlan("short", study_phases(), (3000, 3000, 3000, 3000))
    bank = SignalBank()
    ctrl = FixedTimeController(plan)
    ctrl.advance(bank, 1.0)
    assert bank.state("NL") is FLASHING_GREEN    # GREEN lasted 0 ms, G = FG
    ctrl.advance(bank, plan.cycle_ms / 1000 * 2)


def test_other_phase_layout_uses_conflict_model_not_hardcoded_pairs():
    """A split-phase plan (NS+NL, SS+SL, ES+EL, WS+WL) is also valid and runs safely."""
    phases = (Phase("N", ("NS", "NL")), Phase("S", ("SS", "SL")), Phase("E", ("ES", "EL")),
              Phase("W", ("WS", "WL")))
    plan = FixedPlan.from_seconds("split", phases, {"N": 10, "S": 10, "E": 10, "W": 10}, {"straight": 7, "left": 5})
    # mixed phase: next start waits for the longer of the two clearances (left: 6.0 s)
    assert plan.phase_duration_ms(0) == 1000 + 10000 + 6000
    bank = SignalBank()
    FixedTimeController(plan).advance(bank, 600)


# ============================================================ time-of-day (S6)
def test_tod_switches_at_first_phase_starting_at_or_after_1800(book):
    sched = book.schedule("FT-TOD", "S6")
    recs, _ = FixedTimeController.timetable(sched, -300, 3600)
    before = [r for r in recs if r.start_ms < 1800000]
    after = [r for r in recs if r.start_ms >= 1800000]
    assert all(r.plan == "TOD/S6/first" for r in before)
    assert all(r.plan == "TOD/S6/second" for r in after)
    phases = [r.phase for r in recs]
    assert phases == [["A", "B", "C", "D"][k % 4] for k in range(len(phases))]  # nothing skipped
    for r in recs:
        g = r.yellow_ms - r.green_ms
        assert g == book.plans[r.plan].green_ms(r.phase)


def test_switch_exactly_at_a_phase_start_uses_new_plan(fta):
    other = FixedPlan.from_seconds("other", fta.phases, {"A": 5, "B": 7, "C": 5, "D": 7})
    sched = PlanSchedule("s", [(None, fta), (46500, other)])   # 46.5 s = start of phase C
    recs, _ = FixedTimeController.timetable(sched, 0, 100)
    assert [(r.phase, r.plan) for r in recs[:5]] == [("A", "FT-A"), ("B", "FT-A"), ("C", "other"),
                                                   ("D", "other"), ("A", "other")]
    sched2 = PlanSchedule("s", [(None, fta), (46501, other)])
    recs2, _ = FixedTimeController.timetable(sched2, 0, 100)
    assert recs2[2].plan == "FT-A" and recs2[3].plan == "other"


def test_tod_run_is_safe_through_the_switch(book):
    bank = SignalBank(start_time=1700)
    ctrl = book.controller("FT-TOD", "S6", start_time=1700)
    run_steps(ctrl, bank, 400, each=lambda b: conflicts.check_invariant(b.active()))


# ================================================== determinism / reproducibility
def test_deterministic_and_matches_pure_timetable(book):
    sched = book.schedule("FT-TOD", "S6")
    logs = []
    for _ in range(2):
        bank = SignalBank(start_time=-300)
        ctrl = FixedTimeController(sched, start_time=-300)
        run_steps(ctrl, bank, 3900)
        logs.append((transitions(bank), list(ctrl.command_log)))
    assert logs[0] == logs[1]
    _, cmds = FixedTimeController.timetable(sched, -300, 3600.001)
    assert logs[0][1] == cmds


# ================================================== independence from sensors
FORBIDDEN_IMPORTS = {"smarttraffic.sensors", "smarttraffic.vehicles", "smarttraffic.hardware",
                     "smarttraffic.hybrid_sensors", "smarttraffic.esp_protocol", "smarttraffic.traffic_gen",
                     "smarttraffic.controller_smart", "smarttraffic.viz", "smarttraffic.stats",
                     "serial", "pygame", "time", "datetime", "random", "numpy"}


def test_controller_module_imports_no_sensor_vehicle_hardware_or_clock():
    tree = ast.parse(inspect.getsource(cf))
    imported = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            imported |= {a.name for a in n.names}
        elif isinstance(n, ast.ImportFrom):
            imported.add(n.module)
            imported |= {f"{n.module}.{a.name}" for a in n.names}
    assert not imported & FORBIDDEN_IMPORTS, imported & FORBIDDEN_IMPORTS
    assert {i for i in imported if i.startswith("smarttraffic")} <= {
        "smarttraffic", "smarttraffic.conflicts", "smarttraffic.signals",
        "smarttraffic.conflicts.MOVES", "smarttraffic.conflicts.validate_movement",
        *{f"smarttraffic.signals.{x}" for x in ("ALL_RED_MS", "FLASHING_GREEN_MS", "MS_PER_S", "RED",
                                                  "RED_YELLOW_MS", "YELLOW_MS", "movement_type")},
        "smarttraffic.conflicts"}
    assert "sensor" not in inspect.getsource(cf.FixedTimeController.update).lower()


def test_controller_takes_no_sensor_input():
    params = list(inspect.signature(FixedTimeController.__init__).parameters)
    assert params == ["self", "schedule", "start_time"]
    assert list(inspect.signature(FixedTimeController.update).parameters) == ["self", "bank"]
    assert FixedTimeController.uses_sensors is False


class TogglingSource:
    """DetectorSource whose 16 detectors flip on/off every period_ms (or stay fixed)."""
    def __init__(self, mode, period_ms=700, start_ms=0):
        from smarttraffic.sensors import SENSOR_IDS
        self.sensor_ids = SENSOR_IDS
        self.mode, self.period, self._now = mode, period_ms, start_ms

    @property
    def now_ms(self):
        return self._now

    def initial_occupancy(self):
        return {s: self.mode == "occupied" for s in self.sensor_ids}

    def poll(self, until_ms):
        from smarttraffic.sensors import DetectorEdge
        edges = []
        if self.mode == "toggle":
            k = self._now // self.period + 1
            while k * self.period <= until_ms:
                for m, d in self.sensor_ids:
                    edges.append(DetectorEdge(k * self.period, m, d, k % 2 == 1))
                k += 1
        self._now = until_ms
        return edges


def _full_step_run(plan, source_factory, seconds=600, arrivals=()):
    """The real step order: vehicles -> sensors -> controller -> signals, all at dt = 0.1 s."""
    from smarttraffic.sensors import SensorBank, VirtualDetectors
    from smarttraffic.vehicles import VehicleModel
    vm = VehicleModel(list(arrivals), start=0.0)
    src = source_factory(vm)
    sb = SensorBank(src)
    bank = SignalBank()
    ctrl = FixedTimeController(plan)
    calls = []
    for _ in range(seconds * 10):
        vm.advance(0.1, bank)
        sb.advance(0.1, bank)
        ctrl.update(bank)
        bank.advance(0.1)
        calls.append(sum(sb.has_call(m) for m in MOVES))
    return transitions(bank), ctrl.command_log, calls


def test_detector_activity_cannot_alter_the_schedule(fta):
    from smarttraffic.sensors import VirtualDetectors
    ref = _full_step_run(fta, lambda vm: VirtualDetectors(vm))
    for mode in ("occupied", "clear", "toggle"):
        got = _full_step_run(fta, lambda vm, mode=mode: TogglingSource(mode))
        assert got[0] == ref[0] and got[1] == ref[1], mode
    occupied = _full_step_run(fta, lambda vm: TogglingSource("occupied"))
    assert max(occupied[2]) > 0          # the sensors really did see demand


def test_heavy_traffic_and_empty_roads_give_identical_signals(fta):
    from smarttraffic.sensors import VirtualDetectors
    from smarttraffic.traffic_gen import generate_arrivals
    heavy = generate_arrivals("S7b", 1, 0, 600).arrivals
    empty = _full_step_run(fta, lambda vm: VirtualDetectors(vm))
    busy = _full_step_run(fta, lambda vm: VirtualDetectors(vm), arrivals=heavy)
    assert busy[0] == empty[0] and busy[1] == empty[1]
    assert max(busy[2]) > 0 and max(empty[2]) == 0


def test_real_hardware_style_detector_cannot_alter_schedule(fta):
    """NS:D1 from a 'real' source (hybrid assignment, as in Stage 9) toggling constantly."""
    from smarttraffic.hybrid_sensors import HybridDetectorSource
    from smarttraffic.sensors import VirtualDetectors
    ref = _full_step_run(fta, lambda vm: VirtualDetectors(vm))

    def hybrid(vm):
        real = TogglingSource("toggle", period_ms=450)
        real.sensor_ids = (("NS", "D1"),)
        return HybridDetectorSource.with_overrides(VirtualDetectors(vm), [(("NS", "D1"), real)])

    got = _full_step_run(fta, hybrid)
    assert got[0] == ref[0] and got[1] == ref[1]


def test_step_order_run_equals_controller_only_run(fta):
    """Inside the full step loop the controller produces exactly its pure timetable."""
    from smarttraffic.sensors import VirtualDetectors
    trans, cmds, _ = _full_step_run(fta, lambda vm: VirtualDetectors(vm), seconds=300)
    _, want = FixedTimeController.timetable(fta, 0, 299.95)
    assert cmds == want


# ================================================== capacity with the vehicle model
def _fta_left_crossings(fta, policy):
    from smarttraffic.traffic_gen import Arrival
    from smarttraffic.vehicles import VehicleModel
    arrivals = [Arrival(i, -100000 + 2000 * i, "NL", i, "random") for i in range(10)]
    vm = VehicleModel(arrivals, start=-120.0, policy=policy)
    bank = SignalBank(start_time=-120)
    ctrl = FixedTimeController(fta, start_time=0)
    for _ in range(round((120 + 2 * 93) * 10)):
        vm.advance(0.1, bank)
        ctrl.update(bank)
        bank.advance(0.1)
    return sorted(v.cross_ms for v in vm.crossed("NL"))


def test_fta_left_phase_discharges_four_queued_cars_per_8s_green(fta):
    """Study §8: FT-A left capacity 162 veh/h = 4 cars per 8 s green. Standard policy: the
    first car leaves at green onset (crosses 36 ms later), then one car per h = 2.0 s; the 5th
    would be due at yellow onset + 36 ms and stops (Stop on Yellow)."""
    crossings = _fta_left_crossings(fta, None)
    # phase A: GREEN at 1.0 s, YELLOW at 9.0 s -> 1.036, 3.036, 5.036, 7.036 (9.036 >= 9.0 stops)
    assert crossings[:4] == [1036, 3036, 5036, 7036]
    # next cycle: GREEN at 94.0 s
    assert crossings[4:8] == [94036, 96036, 98036, 100036]
    assert len(crossings) == 8


def test_legacy_fta_left_phase_discharges_four_queued_cars_per_8s_green(fta):
    """legacy-study policy (effective green = displayed green, l1 = e = 2.0 s). Car 4 is due
    exactly at yellow onset + 2.0 s and must cross (inclusive yellow window)."""
    crossings = _fta_left_crossings(fta, "legacy-study")
    # phase A: GREEN at 1.0 s, YELLOW at 9.0 s -> window ends at 11.0 s (inclusive)
    assert crossings[:4] == [5000, 7000, 9000, 11000]
    # next cycle: GREEN at 94.0 s -> 98, 100, 102, 104 s
    assert crossings[4:8] == [98000, 100000, 102000, 104000]
    assert len(crossings) == 8
