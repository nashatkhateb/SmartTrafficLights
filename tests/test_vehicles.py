"""Tests for smarttraffic.vehicles (study §4, §5; ARCHITECTURE R3)."""
import ast
import random
import time
from pathlib import Path

import numpy as np
import pytest

from smarttraffic import vehicles as vh
from smarttraffic.conflicts import MOVES, InvalidMovementError
from smarttraffic import signals as sg
from smarttraffic.signals import SignalBank, SignalState
from smarttraffic.traffic_gen import Arrival, generate_arrivals
from smarttraffic.vehicles import (ModelInvariantError, VehicleModel, VehicleState,
                                   slot_position)

DT = 0.1
STRAIGHTS = ["NS", "SS", "ES", "WS"]
LEFTS = ["NL", "SL", "EL", "WL"]
H = {"straight": 1.9, "left": 2.0}


def arrs(spec, id0=0):
    """spec: list of (movement, time_s). Returns Arrival records with per-movement seq."""
    seqs = {}
    out = []
    for i, (m, t) in enumerate(sorted(spec, key=lambda x: (x[1], MOVES.index(x[0])))):
        out.append(Arrival(id0 + i, round(t * 1000), m, seqs.get(m, 0), "random"))
        seqs[m] = seqs.get(m, 0) + 1
    return out


def queue_spec(m, n, start=0.0, gap=2.0):
    return [(m, start + gap * i) for i in range(n)]


class Sim:
    """Minimal driver: commands at exact times, then vehicles, then signals (study §7 order).

    Signal-revision note: at(t, m, "G") still means GREEN BEGINS at t, so t_g in
    the discharge rule is unchanged; the driver issues start_green RED_YELLOW_S
    earlier. at(t, m, "Y") still means YELLOW BEGINS at t; the driver issues
    end_green FLASHING_GREEN_S earlier (the flashing green is permissive, so the
    service window is the same as before). "start"/"end" issue raw commands at t.
    Default start is -5 s so that a green at t = 0 can be prepared.
    """
    def __init__(self, arrivals, start=-5.0, dt=DT):
        self.vm = VehicleModel(arrivals, start=start)
        self.bank = SignalBank(start_time=start)
        self.dt = dt
        self.cmds = {}
        self.snap = []                      # per-step checks

    def at(self, t, m, what):
        lead = {"G": sg.RED_YELLOW_S, "Y": sg.FLASHING_GREEN_S, "start": 0.0, "end": 0.0}[what]
        cmd = "start" if what in ("G", "start") else "end"
        t_ms = round((t - lead) * 1000)
        if t_ms < self.vm.now_ms:
            raise ValueError(f"command for {m} at {t_ms / 1000} s is in the past (now {self.vm.now} s)")
        self.cmds.setdefault(t_ms, []).append((m, cmd))
        return self

    def run(self, until, check=None):
        while self.vm.now_ms < round(until * 1000):
            for m, what in self.cmds.pop(self.vm.now_ms, []):
                (self.bank.start_green if what == "start" else self.bank.end_green)(m)
            self.vm.advance(self.dt, self.bank)
            self.bank.advance(self.dt)
            if check:
                check(self)
        return self


def rel_cross(vm, m, t0):
    return [round(v.cross_time - t0, 3) for v in vm.crossed(m)]


def assert_no_overlap(vm):
    for m in MOVES:
        xs = [v.position for v in vm.vehicles(m)]
        assert all(b - a >= 7.5 - 1e-6 for a, b in zip(xs, xs[1:])), (m, xs)
        assert all(0 - 1e-6 <= x <= 150 + 1e-9 for x in xs)


# ------------------------------------------------------------- constants
def test_study_constants():
    assert vh.FREE_SPEED == pytest.approx(13.8889, abs=1e-4)          # 50 km/h
    assert vh.VEHICLE_LENGTH == 4.5 and vh.STANDSTILL_GAP == 3.0 and vh.QUEUE_SPACING == 7.5
    assert vh.ENTRY_POSITION == 150.0
    assert vh.START_UP_LOST_TIME == 2.0 and vh.SATURATION_HEADWAY == {"straight": 1.9, "left": 2.0}
    assert vh.YELLOW_EXTENSION == 2.0
    assert vh.FREE_TRAVEL_TIME == pytest.approx(10.8)


def test_slot_positions_match_sensor_table():
    assert slot_position(1) == 0.5                       # occupies 0.5-5.0 m, covers D1 at 2 m
    assert slot_position(5) == 30.5                      # occupies 30.5-35.0 m, covers D2 at 32 m
    assert slot_position(20) == 143.0
    for bad in (0, -1, 1.0, True, "1"):
        with pytest.raises(ValueError):
            slot_position(bad)


# ------------------------------------------------------------- entry / free flow
@pytest.mark.parametrize("m", MOVES)
def test_appears_at_150m_and_free_flow_through_green(m):
    s = Sim(arrs([(m, 5.0)])).at(0, m, "G")
    seen = []
    s.run(5.1, lambda s: seen.append(s.vm.vehicles(m)[0].position) if s.vm.vehicles(m) else None)
    v = s.vm.vehicle(0)
    assert v.entry_ms == 5000 and v.scheduled_ms == 5000
    assert seen == pytest.approx([150.0, 150 - vh.FREE_SPEED * 0.1])   # at t = 5.0 and 5.1
    s.run(30)
    assert v.cross_ms == 5000 + 10800                     # 150 m at 50 km/h, no delay
    assert v.queue_join_ms is None and v.stopped_time == pytest.approx(0.0, abs=1e-9)
    assert v.cross_ms == v.free_flow_cross_ms
    assert v.state is VehicleState.EXITED
    assert v.exit_ms == v.cross_ms + round(vh.BOX_LENGTH[vh.movement_type(m)] / vh.BOX_SPEED * 1000)


def test_position_decreases_at_free_speed():
    s = Sim(arrs([("NS", 0.0)])).at(0, "NS", "G")
    s.run(1.0)
    assert s.vm.vehicle(0).position == pytest.approx(150 - vh.FREE_SPEED * 1.0)
    s.run(10.0)
    assert s.vm.vehicle(0).position == pytest.approx(150 - vh.FREE_SPEED * 10.0)


# ------------------------------------------------------------- queue formation
@pytest.mark.parametrize("m", MOVES)
def test_red_forms_queue_at_7_5m_slots(m):
    s = Sim(arrs(queue_spec(m, 12))).run(80)
    assert s.vm.stopped_positions(m) == tuple(slot_position(k) for k in range(1, 13))
    assert s.vm.queue_length(m) == 12
    assert all(v.state is VehicleState.STOPPED for v in s.vm.vehicles(m))
    assert s.vm.crossed(m) == ()
    assert s.vm.lead_vehicle(m).seq == 0


def test_queue_join_time_is_exact():
    s = Sim(arrs([("NS", 0.0), ("NS", 2.0)])).run(40)
    a, b = s.vm.vehicle(0), s.vm.vehicle(1)
    assert a.queue_join_ms == round((150 - 0.5) / vh.FREE_SPEED * 1000)   # 10764 ms
    assert b.queue_join_ms == 2000 + round((150 - 8.0) / vh.FREE_SPEED * 1000)


def test_queue_join_exact_when_halting_mid_step():
    # halts behind a stopped car at a non-grid instant; must not depend on dt
    joins = []
    for dt in (0.1, 1.0, 0.001):
        s = Sim(arrs([("SS", 0.0), ("SS", 3.3)]), dt=dt).run(40)
        joins.append(s.vm.vehicle(1).queue_join_ms)
    expect = 3300 + round((150 - 8.0) / vh.FREE_SPEED * 1000)
    assert joins == [expect] * 3


def test_stopped_time_accumulates():
    s = Sim(arrs([("NS", 0.0)])).run(50)
    v = s.vm.vehicle(0)
    assert v.stopped_time == pytest.approx(50 - 149.5 / vh.FREE_SPEED, abs=1e-6)


def test_no_crossing_on_red_ever():
    s = Sim(arrs(queue_spec("EL", 5))).run(600)
    assert s.vm.crossed("EL") == () and s.vm.lead_vehicle("EL").position == 0.5


# ------------------------------------------------------------- discharge
@pytest.mark.parametrize("m", MOVES)
def test_discharge_timing_first_fifth_tenth(m):
    h = H[vh.movement_type(m)]
    s = Sim(arrs(queue_spec(m, 10))).at(100, m, "G").run(140)
    got = rel_cross(s.vm, m, 100)
    assert got == [round(2.0 + n * h, 3) for n in range(1, 11)]
    assert got[0] == (3.9 if h == 1.9 else 4.0)
    assert got[4] == (11.5 if h == 1.9 else 12.0)
    assert got[9] == (21.0 if h == 1.9 else 22.0)


def test_regression_10_car_straight_queue_clears_in_21_s():
    """Study §5/§16 validation target: a 10-car queue clears in 21.0 s."""
    s = Sim(arrs(queue_spec("NS", 10))).at(100, "NS", "G").run(130)
    assert s.vm.crossed("NS")[-1].cross_time - 100 == pytest.approx(21.0, abs=1e-9)


def test_20_car_queue_matches_study_table():
    s = Sim(arrs(queue_spec("SS", 20, gap=1.0))).at(100, "SS", "G").run(150)
    got = rel_cross(s.vm, "SS", 100)
    assert (got[0], got[2], got[4], got[9], got[19]) == (3.9, 7.7, 11.5, 21.0, 40.0)


def test_queue_starts_from_the_front_one_car_at_a_time():
    s = Sim(arrs(queue_spec("NS", 5))).at(100, "NS", "G")
    s.run(103.8)
    assert s.vm.stopped_positions("NS") == (0.5, 8.0, 15.5, 23.0, 30.5)
    s.run(104.0)                            # car 1 crossed at 103.9; others still in their slots
    assert s.vm.stopped_positions("NS") == (8.0, 15.5, 23.0, 30.5)
    # car 2 (slot 2, 8.0 m) starts at 105.8 - 8.0/v and reaches the line at 105.8
    start2 = 105.8 - 8.0 / vh.FREE_SPEED
    s.run(round(start2 - 0.05, 1))
    assert s.vm.vehicle(1).state is VehicleState.STOPPED
    s.run(105.8)
    assert s.vm.vehicle(1).cross_ms == 105800


def test_queue_length_never_flickers_during_discharge():
    # stopped count must fall monotonically: cars leave their slots one at a time
    s = Sim(arrs(queue_spec("NS", 10))).at(100, "NS", "G")
    q = []
    s.run(125, lambda s: q.append(s.vm.queue_length("NS")) if s.vm.now >= 100 else None)
    assert q[0] == 10 and q[-1] == 0
    assert all(b <= a for a, b in zip(q, q[1:]))
    # car k (slot k) is still stopped until its own latest start D_k - x_k / v
    for k in range(1, 11):
        start = 100 + 2.0 + 1.9 * k - slot_position(k) / vh.FREE_SPEED
        assert q[int((start - 100) * 10) - 1] >= 11 - k


def test_green_with_no_queue_no_delay():
    s = Sim(arrs([("WS", 30.0), ("WS", 40.0)])).at(0, "WS", "G").run(70)
    for v in s.vm.crossed("WS"):
        assert v.cross_ms == v.scheduled_ms + 10800 and v.queue_join_ms is None


def test_green_start_before_car_reaches_line_applies_start_up_rule():
    # study rule: D >= t_g + l1 + h, even for a car that was still moving
    s = Sim(arrs([("NS", 0.0)])).at(8.0, "NS", "G").run(30)
    assert s.vm.vehicle(0).cross_ms == 8000 + 3900       # not 10800


def test_car_joining_moving_queue_uses_prev_plus_h():
    spec = queue_spec("NS", 3) + [("NS", 95.0)]
    s = Sim(arrs(spec)).at(100, "NS", "G").run(130)
    got = rel_cross(s.vm, "NS", 100)
    assert got[:3] == [3.9, 5.8, 7.7]
    assert got[3] == pytest.approx(max(95 + 10.8 - 100, 7.7 + 1.9))


def test_arrivals_during_long_green_keep_headway():
    spec = [("ES", t) for t in (0.0, 0.6, 1.2, 1.8, 2.4)]      # denser than saturation
    s = Sim(arrs(spec)).at(0, "ES", "G").run(40)
    c = [v.cross_ms for v in s.vm.crossed("ES")]
    assert all(b - a >= 1900 for a, b in zip(c, c[1:]))
    assert c[0] == 10800


# ------------------------------------------------------------- yellow rule
def _yellow_case(t_y, n=10, m="NS"):
    return Sim(arrs(queue_spec(m, n))).at(100, m, "G").at(t_y, m, "Y")


def test_yellow_first_2s_crossing():
    s = _yellow_case(110.0).run(140)
    got = rel_cross(s.vm, "NS", 100)
    assert got == [3.9, 5.8, 7.7, 9.6, 11.5]                  # 111.5 < 112.0
    assert s.vm.lead_vehicle("NS").position == 0.5
    assert s.vm.queue_length("NS") == 5


@pytest.mark.parametrize("t_y, after_yellow_ms, crosses", [
    (109.501, 1999, True),     # D = t_y + 1999 ms: allowed
    (109.500, 2000, True),     # D = t_y + 2000 ms exactly: allowed (window end is inclusive)
    (109.499, 2001, False),    # D = t_y + 2001 ms: blocked
])
def test_yellow_boundary_exact(t_y, after_yellow_ms, crosses):
    # car 5 is due at D = 111.5 s; yellow begins at t_y
    assert round((111.5 - t_y) * 1000) == after_yellow_ms
    s = Sim(arrs(queue_spec("NS", 10)), dt=0.001).at(100, "NS", "G").at(t_y, "NS", "Y")
    s.run(115)
    assert len(s.vm.crossed("NS")) == (5 if crosses else 4)
    car5 = s.vm.vehicle(4)
    assert car5.cross_ms == (111500 if crosses else None)
    if not crosses:
        assert s.vm.lead_vehicle("NS") is car5 and car5.position == 0.5


def test_no_crossing_during_all_red_or_red():
    s = _yellow_case(110.0)
    crossings = []
    def chk(s):
        for v in s.vm.crossed("NS"):
            if v.cross_ms not in crossings:
                crossings.append(v.cross_ms)
                assert s.bank.state("NS") in (SignalState.GREEN, SignalState.FLASHING_GREEN,
                                              SignalState.YELLOW)
    s.run(200, chk)
    assert max(crossings) < 112000


def test_car_arriving_in_yellow_crosses_only_within_window():
    early = Sim(arrs([("SS", 0.0)])).at(0, "SS", "G").at(9.0, "SS", "Y").run(30)
    assert early.vm.vehicle(0).cross_ms == 10800             # 10.8 < 9.0 + 2.0
    edge = Sim(arrs([("SS", 0.0)])).at(0, "SS", "G").at(8.8, "SS", "Y").run(30)
    assert edge.vm.vehicle(0).cross_ms == 10800              # 10.8 <= 8.8 + 2.0 (inclusive)
    late = Sim(arrs([("SS", 0.0)]), dt=0.001).at(0, "SS", "G").at(8.799, "SS", "Y").run(30)
    assert late.vm.vehicle(0).cross_ms is None               # 10.8 > 8.799 + 2.0
    assert late.vm.vehicle(0).position == 0.5
    # crossing time first granted when the window end is already known: GREEN at 100 is
    # ended at once (FLASHING_GREEN 100-103, YELLOW 103, window end 105.0); the car arrives
    # at the line at scheduled + 10.8 s
    for sched, crosses in [(94.199, True), (94.200, True), (94.201, False)]:
        g0 = Sim(arrs([("SS", sched)])).at(100, "SS", "G").at(100, "SS", "end").run(130)
        v = g0.vm.vehicle(0)
        assert v.cross_ms == (round(sched * 1000) + 10800 if crosses else None), sched


def test_stopped_by_yellow_then_served_next_green():
    s = _yellow_case(110.0).at(150, "NS", "G").run(200)
    got = rel_cross(s.vm, "NS", 150)
    assert got[5:] == [3.9, 5.8, 7.7, 9.6, 11.5]             # new queue of 5 restarts the formula


def test_sim_driver_rejects_commands_in_the_past():
    s = Sim(arrs([("NS", 0.0)])).run(10)
    with pytest.raises(ValueError, match="in the past"):
        s.at(10.5, "NS", "G")                       # would need start_green at 9.5


# ------------------------------------------------------------- new signal states
def test_red_yellow_is_not_permissive_and_t_g_is_green_onset():
    s = Sim(arrs(queue_spec("NS", 3))).at(100, "NS", "start")       # RED_YELLOW 100-101
    crossings = []
    def chk(s):
        for v in s.vm.crossed("NS"):
            if v.cross_ms not in crossings:
                crossings.append(v.cross_ms)
                assert s.bank.state("NS") in sg.PERMISSIVE_STATES
    s.run(99.9)
    s.run(101.0, chk)
    assert s.bank.state("NS") is SignalState.GREEN and s.bank.since("NS") == 101.0
    assert s.vm.crossed("NS") == () and s.vm.stopped_positions("NS") == (0.5, 8.0, 15.5)
    s.run(130, chk)
    assert rel_cross(s.vm, "NS", 101.0) == [3.9, 5.8, 7.7]            # measured from GREEN onset


def test_car_arriving_during_red_yellow_waits_for_green_rule():
    # free arrival at the line is 10.8 s; RED_YELLOW 10.0-11.0, GREEN at 11.0
    s = Sim(arrs([("SL", 0.0)])).at(10.0, "SL", "start").run(30)
    v = s.vm.vehicle(0)
    assert v.cross_ms == 11000 + 2000 + 2000                         # t_g + l1 + h (left)
    assert v.queue_join_ms is not None


def test_flashing_green_is_permissive():
    # GREEN at 100; end_green at 105 -> FLASHING_GREEN 105-108, YELLOW from 108 (window to 110)
    s = Sim(arrs(queue_spec("NS", 10))).at(100, "NS", "G").at(105, "NS", "end")
    s.run(140)
    assert rel_cross(s.vm, "NS", 100) == [3.9, 5.8, 7.7, 9.6]          # 105.8, 107.7 during FG
    assert s.vm.lead_vehicle("NS").position == 0.5


def test_crossings_happen_in_flashing_green_state():
    s = Sim(arrs(queue_spec("NS", 10))).at(100, "NS", "G").at(105, "NS", "end")
    seen = {}
    def chk(s):
        for v in s.vm.crossed("NS"):
            seen.setdefault(v.cross_ms, s.bank.state("NS"))
    s.run(112, chk)
    # the step that ends at a crossing time is driven by the state at its start
    assert seen[105800] is SignalState.FLASHING_GREEN
    assert seen[107700] is SignalState.FLASHING_GREEN


def test_yellow_2s_rule_unchanged_after_flashing_green():
    # same as the old yellow test: YELLOW at 110 -> cars with D < 112 cross, 111.5 does, 113.4 not
    s = Sim(arrs(queue_spec("NS", 10))).at(100, "NS", "G").at(110, "NS", "Y").run(140)
    assert rel_cross(s.vm, "NS", 100) == [3.9, 5.8, 7.7, 9.6, 11.5]


def test_red_yellow_requires_remaining_from_signal_view():
    class NoRemaining:
        def state(self, m):
            return SignalState.RED_YELLOW if m == "NS" else SignalState.RED
        def since(self, m):
            return 0.0
    with pytest.raises(TypeError, match="remaining"):
        VehicleModel(arrs([("NS", 0.0)]), start=0).advance(0.1, NoRemaining())


def test_flashing_window_taken_from_signal_timer():
    # a view reporting FLASHING_GREEN with 0.5 s left: window ends at now + 0.5 + 2.0
    class View:
        def __init__(self): self.t = 0.0
        def state(self, m):
            return SignalState.FLASHING_GREEN if m == "ES" else SignalState.RED
        def since(self, m): return 0.0
        def remaining(self, m): return 0.5
    vm = VehicleModel(arrs([("ES", 0.0)]), start=0)
    vm._lanes["ES"].green_start_ms = -100_000
    vm.advance(0.0, View())
    lane = vm._lanes["ES"]
    assert lane.perm == (-100_000, 0 + 500 + 2000)


def test_t_g_known_even_if_green_shorter_than_a_step():
    # a view that jumps RED_YELLOW -> FLASHING_GREEN between two 1 s steps (GREEN never seen):
    # t_g must still be the GREEN onset announced during RED_YELLOW (t = 11.0)
    class View:
        def __init__(self): self.phase = "RED"
        def state(self, m):
            return getattr(SignalState, self.phase) if m == "NS" else SignalState.RED
        def since(self, m): return {"RED": 0.0, "RED_YELLOW": 10.0, "FLASHING_GREEN": 11.0}[self.phase]
        def remaining(self, m): return {"RED_YELLOW": 1.0, "FLASHING_GREEN": 3.0}[self.phase]
    vm = VehicleModel(arrs([("NS", 0.0)]), start=0)
    v = View()
    for _ in range(10):
        vm.advance(1.0, v)                       # car stops at slot 1 on red
    v.phase = "RED_YELLOW"
    vm.advance(1.0, v)                           # t = 10 -> 11
    v.phase = "FLASHING_GREEN"
    for _ in range(10):
        vm.advance(1.0, v)
    assert vm.vehicle(0).cross_ms == 11000 + 3900   # t_g + l1 + h from the true GREEN onset


# ------------------------------------------------------------- R3 entry blocking
def test_too_close_arrivals_wait_outside_fifo():
    spec = [("ES", 10.0), ("ES", 10.018), ("ES", 10.1), ("ES", 10.2)]
    s = Sim(arrs(spec)).run(10.3)
    w = s.vm.waiting_outside("ES")
    assert [v.seq for v in w] == [1, 2, 3]
    assert all(v.state is VehicleState.WAITING_OUTSIDE and v.entry_ms is None for v in w)
    s.run(20)
    e = [s.vm.vehicle(i).entry_ms for i in range(4)]
    assert e == [10000, 10540, 11080, 11620]                 # 7.5 m / v = 0.54 s apart
    assert [s.vm.vehicle(i).scheduled_ms for i in range(4)] == [10000, 10018, 10100, 10200]
    assert s.vm.vehicle(1).entry_wait_ms == 522


def test_scheduled_time_preserved_for_delay():
    spec = [("ES", 10.0), ("ES", 10.018)]
    s = Sim(arrs(spec)).at(0, "ES", "G").run(40)
    v = s.vm.vehicle(1)
    assert v.free_flow_cross_ms == 10018 + 10800            # delay reference = schedule
    assert v.cross_ms == max(10540 + 10800, (10000 + 10800) + 1900)   # prev crossed at 20.8 s
    assert v.cross_ms - v.free_flow_cross_ms > 0


def test_spillback_to_entry_blocks_and_releases_in_order():
    spec = queue_spec("WL", 25, gap=1.0)
    s = Sim(arrs(spec)).run(60, lambda s: assert_no_overlap(s.vm))
    assert len(s.vm.vehicles("WL")) == 20                    # 20 slots fit in 150 m
    assert [v.seq for v in s.vm.waiting_outside("WL")] == [20, 21, 22, 23, 24]
    s.at(61, "WL", "G").run(140, lambda s: assert_no_overlap(s.vm))   # sequence starts at 60
    entries = [s.vm.vehicle(i).entry_ms for i in range(25)]
    assert entries == sorted(entries) and all(e is not None for e in entries)
    assert [v.seq for v in s.vm.crossed("WL")] == list(range(25))


def test_S5_burst_overlap_never_overlaps(tmp_path):
    sched = generate_arrivals("S5", 1, -300, 1300)
    es = [a for a in sched.arrivals if a.movement == "ES"]
    s = Sim(es, start=-300, dt=DT).at(-299, "ES", "G")
    s.run(1300, lambda s: assert_no_overlap(s.vm))
    burst = [s.vm.vehicle(a.id) for a in es if a.source == "event"]
    assert len(burst) == 20
    # some burst/random pairs were closer than physical spacing and had to wait
    waited = [v for v in (s.vm.vehicle(a.id) for a in es) if v.entry_wait_ms]
    assert waited
    ent = [s.vm.vehicle(a.id) for a in sorted(es, key=lambda a: (a.time_ms, a.seq))]
    assert [v.entry_ms for v in ent if v.entry_ms is not None] == \
           sorted(v.entry_ms for v in ent if v.entry_ms is not None)
    for a in es:
        assert s.vm.vehicle(a.id).scheduled_ms == a.time_ms   # never modified


# ------------------------------------------------------------- multiple movements
def test_movements_independent():
    spec_ns = queue_spec("NS", 6)
    alone = Sim(arrs(spec_ns)).at(50, "NS", "G").run(100)
    spec_all = spec_ns + queue_spec("EL", 8, start=0.3) + queue_spec("SL", 3, start=0.7)
    both = Sim(arrs(spec_all)).at(50, "NS", "G").run(100)
    assert [v.cross_ms for v in alone.vm.crossed("NS")] == [v.cross_ms for v in both.vm.crossed("NS")]
    assert both.vm.crossed("EL") == () and both.vm.queue_length("EL") == 8
    assert [v.seq for v in both.vm.vehicles("EL")] == list(range(8))


def test_compatible_pair_discharges_in_parallel():
    spec = queue_spec("SS", 5) + queue_spec("SL", 5, start=0.5)
    s = Sim(arrs(spec)).at(100, "SS", "G").at(100, "SL", "G").run(130)
    assert rel_cross(s.vm, "SS", 100) == [3.9, 5.8, 7.7, 9.6, 11.5]
    assert rel_cross(s.vm, "SL", 100) == [4.0, 6.0, 8.0, 10.0, 12.0]


# ------------------------------------------------------------- determinism / dt
def _full_run(dt):
    sched = generate_arrivals("S1", 3, -300, 300)
    s = Sim(sched.arrivals, start=-300, dt=dt)
    for k in range(-299, 300, 60):                    # simple scripted plan, 1 s aligned
        s.at(k, "NS", "G").at(k, "SS", "G").at(k + 25, "NS", "Y").at(k + 25, "SS", "Y")
        s.at(k + 32, "EL", "G").at(k + 32, "WL", "G").at(k + 46, "EL", "Y").at(k + 46, "WL", "Y")
    s.run(360)
    return s.vm


def test_reproducible_same_dt():
    a, b = _full_run(0.1), _full_run(0.1)
    key = lambda vm: [(v.id, v.entry_ms, v.cross_ms, v.queue_join_ms, v.exit_ms) for v in vm.all_vehicles()]
    assert key(a) == key(b)


@pytest.mark.parametrize("dt", [1.0, 0.5])
def test_large_dt_gives_identical_events(dt):
    a, b = _full_run(0.1), _full_run(dt)
    key = lambda vm: [(v.id, v.entry_ms, v.cross_ms, v.queue_join_ms, v.exit_ms) for v in vm.all_vehicles()]
    assert key(a) == key(b)
    for va, vb in zip(a.all_vehicles(), b.all_vehicles()):
        assert va.stopped_time == pytest.approx(vb.stopped_time, abs=1e-6)
        if va.position is not None:
            assert va.position == pytest.approx(vb.position, abs=1e-6)


def test_invariants_hold_through_mixed_run():
    sched = generate_arrivals("S7b", 2, -300, 200)
    s = Sim(sched.arrivals, start=-300)
    for k in range(-299, 200, 40):
        s.at(k, "ES", "G").at(k, "WS", "G").at(k + 20, "ES", "Y").at(k + 20, "WS", "Y")
    s.run(200, lambda s: assert_no_overlap(s.vm))


# ------------------------------------------------------------- interface / validation
def test_signal_clock_must_match():
    vm = VehicleModel(arrs([("NS", 1.0)]), start=0)
    bank = SignalBank()
    bank.advance(0.1)
    with pytest.raises(ValueError, match="call advance"):
        vm.advance(0.1, bank)


class FakeSignals:
    def __init__(self, states):
        self.states = states
    def state(self, m):
        return self.states.get(m, SignalState.RED)
    def since(self, m):
        return 0.0


def test_works_with_any_signal_view():
    vm = VehicleModel(arrs([("WL", 0.0)]), start=0)
    for _ in range(200):
        vm.advance(0.1, FakeSignals({"WL": SignalState.GREEN}))
    assert vm.vehicle(0).cross_ms == 10800


@pytest.mark.parametrize("bad", [object(), None, 5])
def test_bad_signal_object_rejected(bad):
    with pytest.raises(TypeError):
        VehicleModel(arrs([("NS", 0.0)]), start=0).advance(0.1, bad)


def test_bad_signal_state_rejected():
    class Bad(FakeSignals):
        def state(self, m):
            return "GREEN"
    with pytest.raises(TypeError, match="SignalState"):
        VehicleModel(arrs([("NS", 0.0)]), start=0).advance(0.1, Bad({}))


@pytest.mark.parametrize("dt", [-0.1, float("nan"), float("inf"), 0.00005, "0.1", None, True])
def test_bad_dt_rejected(dt):
    with pytest.raises(ValueError):
        VehicleModel(arrs([("NS", 0.0)]), start=0).advance(dt, SignalBank())


def test_zero_dt_noop():
    vm = VehicleModel(arrs([("NS", 0.0)]), start=0)
    vm.advance(0, SignalBank())
    assert vm.now == 0.0


@pytest.mark.parametrize("bad", ["XX", "ns", "", None, 3])
def test_invalid_movement_queries(bad):
    vm = VehicleModel(arrs([("NS", 0.0)]), start=0)
    qs = [vm.vehicles, vm.queue_length, vm.lead_vehicle, vm.stopped_positions,
          vm.waiting_outside, vm.in_intersection, vm.crossed]
    if bad is not None:                      # completed(None) legitimately means "all movements"
        qs.append(vm.completed)
    for q in qs:
        with pytest.raises(InvalidMovementError):
            q(bad)


def test_invalid_arrivals_rejected():
    with pytest.raises(ValueError, match="duplicate"):
        VehicleModel([Arrival(0, 0, "NS", 0, "random"), Arrival(0, 5, "SS", 0, "random")], start=0)
    with pytest.raises(ValueError, match="before model start"):
        VehicleModel([Arrival(0, -1, "NS", 0, "random")], start=0)
    with pytest.raises(TypeError):
        VehicleModel([("NS", 0.0)], start=0)
    with pytest.raises(InvalidMovementError):
        VehicleModel([Arrival(0, 0, "XX", 0, "random")], start=0)
    with pytest.raises(ValueError, match="start is required"):
        VehicleModel([Arrival(0, 0, "NS", 0, "random")])
    with pytest.raises(KeyError):
        VehicleModel([], start=0).vehicle(99)


def test_schedule_object_accepted_and_ids_preserved():
    sched = generate_arrivals("S4", 1, -300, 0)
    vm = VehicleModel(sched)
    assert vm.now == -300.0
    assert [v.id for v in vm.all_vehicles()] == [a.id for a in sched.arrivals]
    assert all(v.arrival is a for v, a in zip(vm.all_vehicles(), sched.arrivals))


def test_vehicle_is_read_only():
    v = VehicleModel(arrs([("NS", 0.0)]), start=0).vehicle(0)
    with pytest.raises(AttributeError):
        v.position = 3.0
    with pytest.raises(AttributeError):
        v.cross_ms = 1


def test_invariant_checker_detects_overlap():
    s = Sim(arrs(queue_spec("NS", 3))).run(30)
    s.vm._lanes["NS"].road[1]._x = 3.0
    with pytest.raises(ModelInvariantError, match="overlap"):
        s.vm.check_invariants()


# ------------------------------------------------------------- independence
def test_no_wall_clock_or_global_random(monkeypatch):
    ref = [(v.id, v.cross_ms) for v in _full_run(0.5).all_vehicles()]
    def boom(*a, **k):
        raise AssertionError("forbidden call")
    for name in ("time", "monotonic", "perf_counter", "sleep", "time_ns"):
        monkeypatch.setattr(time, name, boom)
    for name in ("random", "uniform", "choice", "shuffle"):
        monkeypatch.setattr(random, name, boom)
    for name in ("random", "rand", "exponential", "seed"):
        monkeypatch.setattr(np.random, name, boom)
    assert [(v.id, v.cross_ms) for v in _full_run(0.5).all_vehicles()] == ref


def test_imports_no_controller_and_no_conflict_matrix():
    src = Path(vh.__file__).read_text(encoding="utf-8")
    tree = ast.parse(src)
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert not any("controller" in (m or "") for m in mods)
    assert not mods & {"random", "time", "datetime", "numpy"}
    assert "sensors" not in src.split('"""')[-1]            # no sensor code
    assert "_MATRIX_ROWS" not in src and "GROUPS" not in src and "compatible(" not in src
