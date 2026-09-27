"""Tests for smarttraffic.signals (study §6, §7, §16 step 2; visible-sequence revision).

Time is always advanced explicitly; nothing here sleeps or reads a clock.

Revision note: the first version went RED -> GREEN instantly on start_green()
and GREEN -> YELLOW on start_yellow(). The approved visible sequence is now
RED -> RED_YELLOW -> GREEN and GREEN -> FLASHING_GREEN -> YELLOW -> ALL_RED -> RED,
with start_yellow() renamed end_green(). Tests that relied on the old instant
transitions were updated explicitly (each says what changed); the study's
yellow and all-red durations are still asserted unchanged.
"""
import inspect
import time

import pytest

from smarttraffic import conflicts, signals
from smarttraffic.conflicts import MOVES, InvalidMovementError, SafetyViolation
from smarttraffic.signals import (ALL_RED, FLASHING_GREEN, GREEN, RED, RED_YELLOW, YELLOW,
                                  IllegalTransition, InvalidStateError, Lamps, SignalBank,
                                  SignalState, lamps_for, movement_type)

DT = 0.1
STRAIGHTS = ["NS", "SS", "ES", "WS"]
LEFTS = ["NL", "SL", "EL", "WL"]
TIMING = {"straight": (3.5, 2.0), "left": (3.0, 3.0)}   # (yellow, all-red), study §14
RY = signals.RED_YELLOW_S                                # 1.0 s (project assumption)
FG = signals.FLASHING_GREEN_S                            # 3.0 s (project assumption)


def run(bank, seconds, dt=DT):
    """Advance in dt steps; step count is rounded so it is exact."""
    for _ in range(round(seconds / dt)):
        bank.advance(dt)


def to_green(bank, *ms):
    """Start the green sequence for each movement and run until they are GREEN."""
    for m in ms:
        bank.start_green(m)
    run(bank, RY)
    assert all(bank.state(m) is GREEN for m in ms)


# ------------------------------------------------------------- basics
def test_all_start_red_at_zero():
    b = SignalBank()
    assert b.now == 0.0
    assert all(b.state(m) is RED for m in MOVES)
    assert b.active() == ()


def test_six_states_exist():
    # updated: the revision adds RED_YELLOW and FLASHING_GREEN to the four original states
    assert {s.name for s in SignalState} == {
        "RED", "RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED"}


def test_state_constants_are_the_enum_members():
    # One canonical representation: the aliases ARE the enum members.
    assert RED is SignalState.RED and GREEN is SignalState.GREEN
    assert YELLOW is SignalState.YELLOW and ALL_RED is SignalState.ALL_RED
    assert RED_YELLOW is SignalState.RED_YELLOW and FLASHING_GREEN is SignalState.FLASHING_GREEN


def test_public_api_exports_exist():
    for name in signals.__all__:
        assert hasattr(signals, name), name
    assert {"SignalState", "RED", "RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED",
            "SignalBank", "Lamps", "lamps_for"} <= set(signals.__all__)


def test_old_misleading_commands_are_gone():
    # start_yellow() would now start FLASHING_GREEN, so it was renamed end_green();
    # the generic request() was removed rather than hide the new sequence.
    b = SignalBank()
    assert not hasattr(b, "start_yellow") and not hasattr(b, "request")
    assert callable(b.start_green) and callable(b.end_green)


def test_blocking_and_permissive_sets():
    assert signals.CONFLICT_BLOCKING_STATES == {RED_YELLOW, GREEN, FLASHING_GREEN, YELLOW, ALL_RED}
    assert signals.ACTIVE_STATES == signals.CONFLICT_BLOCKING_STATES
    assert signals.PERMISSIVE_STATES == {GREEN, FLASHING_GREEN}


@pytest.mark.parametrize("m", MOVES)
def test_movement_type(m):
    assert movement_type(m) == ("straight" if m in STRAIGHTS else "left")


def test_study_timing_values_unchanged():
    assert signals.YELLOW_S == {"straight": 3.5, "left": 3.0}
    assert signals.ALL_RED_S == {"straight": 2.0, "left": 3.0}


def test_project_timing_assumptions():
    assert signals.RED_YELLOW_S == 1.0 and signals.FLASHING_GREEN_S == 3.0
    assert signals.FLASH_PERIOD_S == 1.0 and signals.FLASH_ON_S == 0.5


# --------------------------------------------- legal sequence + timing
@pytest.mark.parametrize("m", MOVES)
def test_full_sequence_and_timing(m):
    # updated: was RED->GREEN->YELLOW->ALL_RED->RED
    y, ar = TIMING[movement_type(m)]
    b = SignalBank()
    b.start_green(m)
    assert b.state(m) is RED_YELLOW
    run(b, RY - DT)
    assert b.state(m) is RED_YELLOW
    run(b, DT)
    assert b.state(m) is GREEN and b.since(m) == RY
    run(b, 10.0)
    assert b.state(m) is GREEN and b.elapsed(m) == 10.0   # GREEN has no timer
    t_end = RY + 10.0
    b.end_green(m)
    run(b, FG - DT)
    assert b.state(m) is FLASHING_GREEN
    run(b, DT)
    assert b.state(m) is YELLOW and b.since(m) == t_end + FG
    run(b, y - DT)
    assert b.state(m) is YELLOW
    run(b, DT)
    assert b.state(m) is ALL_RED and b.since(m) == t_end + FG + y
    run(b, ar - DT)
    assert b.state(m) is ALL_RED
    run(b, DT)
    assert b.state(m) is RED and b.since(m) == t_end + FG + y + ar
    assert [(e.old, e.new) for e in b.log] == [
        (RED, RED_YELLOW), (RED_YELLOW, GREEN), (GREEN, FLASHING_GREEN),
        (FLASHING_GREEN, YELLOW), (YELLOW, ALL_RED), (ALL_RED, RED)]
    assert [e.time for e in b.log] == [0.0, RY, t_end, t_end + FG, t_end + FG + y,
                                       t_end + FG + y + ar]


def test_change_period_totals():
    # study §6: yellow + all-red = 5.5 s straights, 6.0 s lefts (unchanged);
    # the full termination after end_green() adds the 3.0 s flashing green.
    for m, total in (("NS", 5.5), ("NL", 6.0)):
        b = SignalBank()
        to_green(b, m)
        b.end_green(m)
        run(b, FG)
        assert b.state(m) is YELLOW
        run(b, total - DT)
        assert b.state(m) is ALL_RED
        run(b, DT)
        assert b.state(m) is RED and b.since(m) == RY + FG + total


def test_display_codes():
    # updated: adds RY and FG codes; ALL_RED still displays "R"
    b = SignalBank()
    b.start_green("SL")
    assert b.display("SL") == "RY"
    run(b, RY)
    assert b.display("SL") == "G"
    b.end_green("SL")
    assert b.display("SL") == "FG"
    run(b, FG)
    assert b.display("SL") == "Y"
    run(b, 3.0)
    assert b.state("SL") is ALL_RED and b.display("SL") == "R"
    assert "SL" in b.active()          # displays red, but still clearing
    run(b, 3.0)
    assert b.display("SL") == "R" and "SL" not in b.active()


def test_remaining():
    b = SignalBank()
    b.start_green("NS")
    assert b.remaining("NS") == RY
    run(b, RY)
    assert b.remaining("NS") is None          # GREEN: until commanded
    b.end_green("NS")
    run(b, 1.0)
    assert b.remaining("NS") == FG - 1.0
    run(b, FG - 1.0 + 1.0)                    # 1.0 s into yellow
    assert b.remaining("NS") == 2.5
    run(b, 3.0)                               # 0.5 s into the 2.0 s ALL_RED
    assert b.remaining("NS") == 1.5
    run(b, 1.5)
    assert b.remaining("NS") is None


# --------------------------------------------- exact boundary tests
@pytest.mark.parametrize("m", ["NS", "NL"])
def test_exact_boundaries_ms(m):
    y, ar = TIMING[movement_type(m)]
    b = SignalBank()
    b.start_green(m)
    for dur, before, after in ((RY, RED_YELLOW, GREEN),):
        b.advance(dur - 0.001)
        assert b.state(m) is before
        b.advance(0.001)
        assert b.state(m) is after and b.elapsed(m) == 0.0
    b.end_green(m)
    for dur, before, after in ((FG, FLASHING_GREEN, YELLOW), (y, YELLOW, ALL_RED), (ar, ALL_RED, RED)):
        b.advance(dur - 0.001)
        assert b.state(m) is before
        b.advance(0.001)
        assert b.state(m) is after and b.elapsed(m) == 0.0


def test_no_float_drift_over_many_steps():
    b = SignalBank()
    run(b, 3600.0)                      # 36 000 steps of 0.1
    assert b.now == 3600.0
    to_green(b, "NS")
    b.end_green("NS")
    run(b, FG + 3.5)
    assert b.state("NS") is ALL_RED and b.since("NS") == 3600.0 + RY + FG + 3.5


# ------------------------------------------- large dt / overshoot
def test_large_dt_cascades_to_red_at_exact_times():
    b = SignalBank()
    b.start_green("NS")
    b.advance(RY)
    b.end_green("NS")
    b.advance(100.0)
    assert b.state("NS") is RED and b.now == 100.0 + RY
    assert [(e.time, e.new) for e in b.log[-3:]] == [
        (RY + FG, YELLOW), (RY + FG + 3.5, ALL_RED), (RY + FG + 5.5, RED)]


def test_large_dt_through_red_yellow():
    b = SignalBank()
    b.start_green("EL")
    b.advance(7.0)
    assert b.state("EL") is GREEN and b.since("EL") == RY and b.elapsed("EL") == 7.0 - RY


def test_large_dt_stops_in_all_red_when_appropriate():
    b = SignalBank()
    to_green(b, "NL")
    b.end_green("NL")
    b.advance(FG + 4.0)                 # 3.0 yellow + 1.0 into all-red
    assert b.state("NL") is ALL_RED and b.elapsed("NL") == 1.0 and b.remaining("NL") == 2.0


def test_large_dt_multiple_movements_chronological_log():
    b = SignalBank()
    to_green(b, "NS", "NL")             # GREEN at 1.0
    b.end_green("NS")                   # FG 1-4, Y 4-7.5, AR 7.5-9.5
    b.end_green("NL")                   # FG 1-4, Y 4-7,   AR 7-10
    b.advance(50.0)
    tail = [(e.time, e.movement, e.new) for e in b.log[-6:]]
    assert tail == [(4.0, "NS", YELLOW), (4.0, "NL", YELLOW), (7.0, "NL", ALL_RED),
                    (7.5, "NS", ALL_RED), (9.5, "NS", RED), (10.0, "NL", RED)]


def test_zero_dt_is_noop():
    b = SignalBank()
    b.start_green("NS")
    b.advance(0)
    assert b.now == 0.0 and b.state("NS") is RED_YELLOW


@pytest.mark.parametrize("bad", [-0.1, float("nan"), float("inf"), 0.00005, "0.1", None, True])
def test_bad_dt_rejected(bad):
    with pytest.raises((ValueError, TypeError)):
        SignalBank().advance(bad)


# --------------------------------------- illegal / skipped transitions
def test_end_green_only_from_green():
    b = SignalBank()
    with pytest.raises(IllegalTransition, match="from RED"):
        b.end_green("NS")
    b.start_green("NS")
    with pytest.raises(IllegalTransition, match="from RED_YELLOW"):     # committed to green
        b.end_green("NS")
    run(b, RY)
    b.end_green("NS")
    with pytest.raises(IllegalTransition, match="from FLASHING_GREEN"):
        b.end_green("NS")
    run(b, FG)
    with pytest.raises(IllegalTransition, match="from YELLOW"):
        b.end_green("NS")
    run(b, 3.5)
    with pytest.raises(IllegalTransition, match="from ALL_RED"):
        b.end_green("NS")


def test_start_green_only_from_red():
    # the old "yellow never goes back to green", extended to every non-RED state
    b = SignalBank()
    b.start_green("NS")
    for dur in (RY, 5.0):                           # in RED_YELLOW, then GREEN
        with pytest.raises(IllegalTransition, match="must be RED"):
            b.start_green("NS")
        run(b, dur)
    b.end_green("NS")
    for dur in (FG, 3.5, 2.0):                      # FLASHING_GREEN, YELLOW, ALL_RED
        with pytest.raises(IllegalTransition, match="must be RED"):
            b.start_green("NS")
        run(b, dur)
    b.start_green("NS")                             # fine once RED
    assert b.state("NS") is RED_YELLOW


def test_no_command_can_skip_timed_states():
    # replaces the old request(RED/ALL_RED) test: there is no API to command a
    # timed state, and the commands that exist refuse every state they do not start from
    b = SignalBank()
    to_green(b, "NS")
    b.end_green("NS")
    seen = []
    for _ in range(round((FG + 5.5) / DT)):
        seen.append(b.state("NS"))
        for cmd in (b.start_green, b.end_green):
            with pytest.raises(IllegalTransition):
                cmd("NS")
        b.advance(DT)
    order = [s for i, s in enumerate(seen) if i == 0 or s is not seen[i - 1]]
    assert order == [FLASHING_GREEN, YELLOW, ALL_RED]


def test_rejected_command_changes_nothing():
    b = SignalBank()
    to_green(b, "NS")
    before = (b.states(), len(b.log))
    with pytest.raises(IllegalTransition):
        b.start_green("SL")
    assert (b.states(), len(b.log)) == before


# ------------------------------- conflicting start during any blocking state
@pytest.mark.parametrize("m", MOVES)
def test_conflicting_start_rejected_in_every_blocking_state(m):
    y, ar = TIMING[movement_type(m)]
    for c in conflicts.conflicts_with(m):
        b = SignalBank()
        b.start_green(m)
        with pytest.raises(IllegalTransition, match=f"{m} is RED_YELLOW"):
            b.start_green(c)
        run(b, RY)
        with pytest.raises(IllegalTransition, match=f"{m} is GREEN"):
            b.start_green(c)
        b.end_green(m)
        for dur, st in ((FG, FLASHING_GREEN), (y, YELLOW), (ar - DT, ALL_RED)):
            assert b.state(m) is st
            with pytest.raises(IllegalTransition, match=f"conflicting {m} is {st.name}"):
                b.start_green(c)
            assert not b.can_start_green(c)
            run(b, dur)
        assert b.state(m) is ALL_RED
        run(b, DT)                      # clearance complete
        assert b.state(m) is RED and b.can_start_green(c)
        b.start_green(c)                # accepted at exactly the end of clearance
        assert b.since(c) == RY + FG + y + ar
        assert b.state(c) is RED_YELLOW


@pytest.mark.parametrize("m", MOVES)
def test_red_never_blocks_compatible_or_conflicting_starts(m):
    b = SignalBank()
    for c in MOVES:
        if c != m:
            assert m not in b.blockers(c)        # m is RED: blocks nobody


def test_compatible_start_allowed_during_every_state_of_partner():
    # NL is compatible with SL: it may start while SL is in any of its five blocking states
    def sl_in(state):
        b = SignalBank()
        b.start_green("SL")
        if state is RED_YELLOW:
            return b
        run(b, RY + 1.0)
        if state is GREEN:
            return b
        b.end_green("SL")
        run(b, {FLASHING_GREEN: 0.0, YELLOW: FG, ALL_RED: FG + 3.0}[state])
        return b
    for st in (RED_YELLOW, GREEN, FLASHING_GREEN, YELLOW, ALL_RED):
        b = sl_in(st)
        assert b.state("SL") is st
        b.start_green("NL")
        assert b.state("NL") is RED_YELLOW and b.state("SL") is st


def test_compatible_green_may_start_during_all_red():
    b = SignalBank()
    to_green(b, "SL")
    b.end_green("SL")
    run(b, FG + 3.0)
    assert b.state("SL") is ALL_RED
    b.start_green("NL")
    run(b, RY)
    assert b.state("NL") is GREEN


def test_two_conflicting_red_yellows_impossible():
    b = SignalBank()
    b.start_green("NS")
    with pytest.raises(IllegalTransition):
        b.start_green("ES")
    assert b.state("ES") is RED


def test_waits_for_the_longest_of_several_clearances():
    # ES conflicts with NS (C) and SL (M).
    b = SignalBank()
    to_green(b, "NS")                   # GREEN at 1.0
    b.end_green("NS")                   # FG 1-4, Y 4-7.5, AR 7.5-9.5
    b.advance(0.5)
    to_green(b, "NL")                   # compatible with NS; GREEN at 2.5
    b.end_green("NL")                   # FG 2.5-5.5, Y 5.5-8.5, AR 8.5-11.5
    b.advance(9.5 - b.now)
    assert b.state("NS") is RED and b.state("NL") is ALL_RED
    assert b.blockers("ES") == ("NL",)
    with pytest.raises(IllegalTransition):
        b.start_green("ES")
    b.advance(11.5 - b.now)
    b.start_green("ES")


# -------------------------------------------- partial transitions
def test_ss_sl_to_ss_ns_timeline_with_new_sequence():
    """Study §7 example under the approved sequence.

    Updated: in the study, SL's gap-out at t = 11 starts YELLOW and NS turns
    GREEN at 17. Now the gap-out starts FLASHING_GREEN (3 s, still serving SL),
    so SL's conflict-blocking clearance ends at 11 + 3 + 3 + 3 = 20; NS may begin
    its sequence at exactly 20 and is GREEN at 21. SS is never interrupted.
    """
    b = SignalBank(start_time=-RY)
    b.start_green("SS")
    b.start_green("SL")
    run(b, RY)                                  # both GREEN at t = 0
    run(b, 11.0)
    b.end_green("SL")
    first_ns = None
    ss_states = set()
    while b.now < 30.0 - 1e-9:
        b.advance(DT)
        ss_states.add(b.state("SS"))
        if first_ns is None and b.can_start_green("NS"):
            b.start_green("NS")
            first_ns = b.now
        elif first_ns is None:
            with pytest.raises(IllegalTransition):
                b.start_green("NS")
        b.check_invariant()
    assert first_ns == 20.0
    assert ss_states == {GREEN}                 # never interrupted
    assert b.since("SS") == 0.0                 # original green start kept
    assert b.elapsed("SS") == 30.0
    assert [(e.time, e.movement, e.new) for e in b.log] == [
        (-1.0, "SS", RED_YELLOW), (-1.0, "SL", RED_YELLOW), (0.0, "SS", GREEN), (0.0, "SL", GREEN),
        (11.0, "SL", FLASHING_GREEN), (14.0, "SL", YELLOW), (17.0, "SL", ALL_RED), (20.0, "SL", RED),
        (20.0, "NS", RED_YELLOW), (21.0, "NS", GREEN)]
    assert b.active() == ("NS", "SS")


def test_ns_rejected_1ms_before_clearance_end_accepted_at_it():
    b = SignalBank(start_time=-RY)
    b.start_green("SS")
    b.start_green("SL")
    b.advance(RY + 11.0)
    b.end_green("SL")
    b.advance(8.999)
    with pytest.raises(IllegalTransition, match="SL is ALL_RED"):
        b.start_green("NS")
    b.advance(0.001)
    b.start_green("NS")
    assert b.states()["SS"] is GREEN and b.since("SS") == 0.0


@pytest.mark.parametrize("stay,end,new", [
    ("SS", "SL", "NS"), ("NS", "NL", "SS"), ("SL", "SS", "NL"), ("NL", "NS", "SL"),
    ("ES", "EL", "WS"), ("WS", "WL", "ES"), ("EL", "ES", "WL"), ("WL", "WS", "EL"),
])
def test_every_partial_transition_keeps_partner_green(stay, end, new):
    assert conflicts.compatible(stay, end) and conflicts.compatible(stay, new)
    assert not conflicts.compatible(end, new)
    b = SignalBank()
    to_green(b, stay, end)
    run(b, 7.0)
    b.end_green(end)
    y, ar = TIMING[movement_type(end)]
    run(b, FG + y + ar)
    b.start_green(new)
    run(b, RY)
    assert b.state(new) is GREEN
    assert b.state(stay) is GREEN and b.since(stay) == RY
    assert b.since(new) == RY + 7.0 + FG + y + ar + RY
    assert [e.new for e in b.log if e.movement == stay] == [RED_YELLOW, GREEN]


# ------------------------------------------- independent timers
def test_independent_timers():
    b = SignalBank()
    to_green(b, "NS")                   # GREEN at 1.0
    b.start_green("NL")
    b.advance(RY)                       # NL GREEN at 2.0
    b.end_green("NS")                   # NS: FG 2-5, Y 5-8.5, AR 8.5-10.5
    b.advance(1.5)
    b.end_green("NL")                   # NL: FG 3.5-6.5, Y 6.5-9.5, AR 9.5-12.5
    b.advance(8.5 - b.now)
    assert (b.state("NS"), b.state("NL")) == (ALL_RED, YELLOW)
    assert (b.elapsed("NS"), b.elapsed("NL")) == (0.0, 2.0)
    b.advance(2.0)                      # t = 10.5
    assert (b.state("NS"), b.state("NL")) == (RED, ALL_RED)
    assert b.remaining("NL") == 2.0
    b.advance(2.0)
    assert b.state("NL") is RED and b.since("NL") == 12.5


def test_untouched_movements_keep_their_times():
    b = SignalBank()
    to_green(b, "ES", "WS")
    run(b, 20.0)
    b.end_green("WS")
    run(b, 10.0)
    assert b.since("ES") == RY and b.elapsed("ES") == 30.0
    assert all(b.since(m) == 0.0 for m in ["NS", "NL", "SS", "SL", "EL", "WL"])


# ------------------------------------------------ lamps
@pytest.mark.parametrize("state,lamps", [
    (RED, (True, False, False)), (RED_YELLOW, (True, True, False)),
    (GREEN, (False, False, True)), (YELLOW, (False, True, False)),
    (ALL_RED, (True, False, False)),
])
def test_steady_lamp_outputs(state, lamps):
    for elapsed in (0, 1, 499, 500, 1234, 10_000):
        assert lamps_for(state, elapsed) == Lamps(*lamps)


def test_red_and_all_red_look_identical_but_differ_in_safety():
    assert lamps_for(RED, 0) == lamps_for(ALL_RED, 0)
    assert RED not in signals.CONFLICT_BLOCKING_STATES and ALL_RED in signals.CONFLICT_BLOCKING_STATES


@pytest.mark.parametrize("elapsed,on", [
    (0, True), (1, True), (499, True), (500, False), (999, False), (1000, True),
    (1499, True), (1500, False), (2000, True), (2499, True), (2500, False), (2999, False),
])
def test_flashing_green_boundaries(elapsed, on):
    assert lamps_for(FLASHING_GREEN, elapsed) == Lamps(False, False, on)


def test_flashing_green_gives_three_blinks():
    pattern = [lamps_for(FLASHING_GREEN, t).green for t in range(signals.FLASHING_GREEN_MS)]
    ons = sum(1 for i in range(len(pattern)) if pattern[i] and (i == 0 or not pattern[i - 1]))
    assert ons == 3 and sum(pattern) == 1500


@pytest.mark.parametrize("m", MOVES)
def test_lamp_state_through_full_sequence(m):
    y, ar = TIMING[movement_type(m)]
    b = SignalBank(start_time=5.0)
    assert b.lamp_state(m) == Lamps(True, False, False)
    b.start_green(m)
    assert b.lamp_state(m) == Lamps(True, True, False)
    b.advance(RY)
    assert b.lamp_state(m) == Lamps(False, False, True)
    b.advance(2.0)
    b.end_green(m)
    seq = []
    for _ in range(round(FG * 10)):
        seq.append(b.lamp_state(m).green)
        b.advance(DT)
    assert seq == [True] * 5 + [False] * 5 + [True] * 5 + [False] * 5 + [True] * 5 + [False] * 5
    assert b.lamp_state(m) == Lamps(False, True, False)
    b.advance(y)
    assert b.lamp_state(m) == Lamps(True, False, False) and b.state(m) is ALL_RED
    b.advance(ar)
    assert b.lamp_state(m) == Lamps(True, False, False) and b.state(m) is RED
    assert set(b.lamp_states()) == set(MOVES)


def test_flashing_is_anchored_to_flash_onset_not_absolute_time():
    for start in (0.0, 0.3, 17.7):
        b = SignalBank(start_time=start)
        to_green(b, "WL")
        b.advance(0.2)
        b.end_green("WL")
        got = []
        for _ in range(30):
            got.append(b.lamp_state("WL").green)
            b.advance(DT)
        assert got == [True] * 5 + [False] * 5 + [True] * 5 + [False] * 5 + [True] * 5 + [False] * 5


def test_flashing_deterministic_across_dt():
    def trace(dt):
        b = SignalBank()
        to_green(b, "ES")
        b.end_green("ES")
        out = {}
        while b.now < RY + FG - 1e-9:
            out[round(b.now * 1000)] = b.lamp_state("ES")
            b.advance(dt)
        return out
    fine, coarse = trace(0.001), trace(0.25)
    assert all(fine[t] == v for t, v in coarse.items())


@pytest.mark.parametrize("bad", [-1, 1.5, "0", None, True])
def test_lamps_for_bad_elapsed(bad):
    with pytest.raises(ValueError):
        lamps_for(GREEN, bad)


# ------------------------------------------------ permissive / blocking
def test_permissive_only_in_green_and_flashing_green():
    b = SignalBank()
    assert not b.is_permissive("NS")
    b.start_green("NS")
    assert not b.is_permissive("NS")                # RED_YELLOW: no crossing
    run(b, RY)
    assert b.is_permissive("NS")
    b.end_green("NS")
    assert b.is_permissive("NS")                    # FLASHING_GREEN: still serving
    run(b, FG)
    assert not b.is_permissive("NS")                # YELLOW: vehicle model's 2.0 s rule
    run(b, 3.5)
    assert not b.is_permissive("NS")


# ------------------------------------------------ safety invariant
def test_invariant_holds_through_legal_operation():
    b = SignalBank()
    to_green(b, "NS", "SS")
    for _ in range(3):
        b.check_invariant()
        b.advance(DT)


def test_invariant_detects_corrupted_state():
    b = SignalBank()
    to_green(b, "NS")
    b._state["SL"] = GREEN              # simulate a bug bypassing the API
    with pytest.raises(SafetyViolation, match="NS-SL"):
        b.check_invariant()
    with pytest.raises(SafetyViolation):
        b.advance(DT)                   # advance() also checks


@pytest.mark.parametrize("bad_state", [RED_YELLOW, GREEN, FLASHING_GREEN, YELLOW, ALL_RED])
def test_invariant_counts_every_blocking_state(bad_state):
    b = SignalBank()
    to_green(b, "ES")
    b._state["NS"] = bad_state
    with pytest.raises(SafetyViolation, match="NS-ES"):
        b.check_invariant()


def test_red_never_violates():
    b = SignalBank()
    to_green(b, "ES")
    for m in MOVES:
        if m != "ES":
            assert b.state(m) is RED
    b.check_invariant()


# ------------------------------------------------- invalid inputs
BAD_MOVES = ["XX", "ns", "", None, 3]


@pytest.mark.parametrize("bad", BAD_MOVES, ids=repr)
def test_invalid_movement_rejected(bad):
    b = SignalBank()
    for call in (b.start_green, b.end_green, b.state, b.elapsed, b.since, b.remaining,
                 b.blockers, b.can_start_green, b.display, b.lamp_state, b.is_permissive,
                 movement_type):
        with pytest.raises(InvalidMovementError):
            call(bad)


@pytest.mark.parametrize("bad", ["GREEN", "green", 1, None, "G"], ids=repr)
def test_invalid_state_rejected(bad):
    b = SignalBank()
    with pytest.raises(InvalidStateError):
        b.in_state(bad)
    with pytest.raises(InvalidStateError):
        lamps_for(bad, 0)
    assert b.state("NS") is RED


# ------------------------------------------------ no wall clock
def test_no_wall_clock_dependency(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("wall clock used")
    for name in ("time", "monotonic", "perf_counter", "sleep", "time_ns", "monotonic_ns"):
        monkeypatch.setattr(time, name, boom)
    b = SignalBank(start_time=-RY)
    b.start_green("SS")
    b.start_green("SL")
    b.advance(RY + 11.0)
    b.end_green("SL")
    lamps = [b.lamp_state("SL")]
    b.advance(9.0)
    b.start_green("NS")
    b.advance(RY)
    assert b.now == 21.0 and b.state("NS") is GREEN and lamps[0].green


def test_module_does_not_import_time_hardware_or_duplicate_matrix():
    src = inspect.getsource(signals)
    for forbidden in ("import time", "datetime", "pygame", "serial", "machine", "controller"):
        assert forbidden not in src.split('"""', 2)[-1], forbidden
    assert "_MATRIX_ROWS" not in src and "GROUPS" not in src
    assert "conflicts.compatible" in src and "conflicts.check_invariant" in src
