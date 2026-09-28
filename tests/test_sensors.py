"""Tests for smarttraffic.sensors (Stage 8: virtual IR detectors + demand logic).

Two kinds of test:
* with the real VehicleModel + SignalBank (geometry, dynamics, all 8 movements, dt);
* with a ScriptedSource that feeds exact edges through the DetectorSource
  interface (exact millisecond boundaries of the D2 filter and the gap timer).
"""
import ast
import time
import random
from pathlib import Path

import numpy as np
import pytest

from smarttraffic import sensors as sn
from smarttraffic import signals as sg
from smarttraffic import vehicles as vh
from smarttraffic.conflicts import MOVES, InvalidMovementError
from smarttraffic.sensors import (D1, D2, DemandLevel, DetectorEdge, SensorBank,
                                  VirtualDetectors)
from smarttraffic.signals import SignalBank, SignalState
from smarttraffic.traffic_gen import Arrival, generate_arrivals
from smarttraffic.vehicles import VehicleModel, slot_position

V = vh.FREE_SPEED
RY_MS, FG_MS = sg.RED_YELLOW_MS, sg.FLASHING_GREEN_MS


def arrs(spec, id0=0):
    seqs, out = {}, []
    for i, (m, t) in enumerate(sorted(spec, key=lambda x: (x[1], MOVES.index(x[0])))):
        out.append(Arrival(id0 + i, round(t * 1000), m, seqs.get(m, 0), "random"))
        seqs[m] = seqs.get(m, 0) + 1
    return out


def queue_spec(m, n, start=0.0, gap=2.0):
    return [(m, start + gap * i) for i in range(n)]


class World:
    """vehicles.advance -> sensors.advance -> signals.advance, commands at exact ms."""
    def __init__(self, arrivals, start=-5.0, dt=0.1):
        self.vm = VehicleModel(arrivals, start=start)
        self.det = VirtualDetectors(self.vm)
        self.sb = SensorBank(self.det)
        self.bank = SignalBank(start_time=start)
        self.dt = dt
        self.cmds = {}

    def at(self, t, m, what):
        """'start'/'end' raw commands at t; 'G' = GREEN begins at t; 'Y' = YELLOW begins at t."""
        lead = {"G": sg.RED_YELLOW_S, "Y": sg.FLASHING_GREEN_S, "start": 0.0, "end": 0.0}[what]
        t_ms = round((t - lead) * 1000)
        assert t_ms >= self.vm.now_ms, "command in the past"
        self.cmds.setdefault(t_ms, []).append((m, "start" if what in ("G", "start") else "end"))
        return self

    def step(self):
        if self.cmds and min(self.cmds) < self.vm.now_ms:
            raise AssertionError(f"command at {min(self.cmds)} ms is not on the dt grid (skipped)")
        for m, c in self.cmds.pop(self.vm.now_ms, []):
            (self.bank.start_green if c == "start" else self.bank.end_green)(m)
        self.vm.advance(self.dt, self.bank)
        self.sb.advance(self.dt, self.bank)
        self.bank.advance(self.dt)

    def run(self, until, check=None):
        while self.vm.now_ms < round(until * 1000):
            self.step()
            if check:
                check(self)
        return self

    def edges(self, m, d=None):
        return [(e.time_ms, e.detector, e.occupied) for e in self.sb.edge_log
                if e.movement == m and (d is None or e.detector == d)]


class ScriptedSource:
    """Test double implementing DetectorSource from a fixed list of edges (not hardware)."""
    sensor_ids = sn.SENSOR_IDS

    def __init__(self, edges, start_ms=0, initial=None):
        self._edges = sorted(edges, key=lambda e: e.time_ms)
        self._now = start_ms
        self._initial = initial or {sid: False for sid in sn.SENSOR_IDS}

    @property
    def now_ms(self):
        return self._now

    def initial_occupancy(self):
        return dict(self._initial)

    def poll(self, until_ms):
        out = [e for e in self._edges if self._now < e.time_ms <= until_ms]
        self._now = until_ms
        return out


def scripted(m, pulses, start_ms=0):
    """pulses: list of (detector, on_ms, off_ms or None)."""
    edges = []
    for d, on, off in pulses:
        edges.append(DetectorEdge(on, m, d, True))
        if off is not None:
            edges.append(DetectorEdge(off, m, d, False))
    return ScriptedSource(edges, start_ms)


def drive(sb, bank, until_ms, dt_ms=1, cmds=None):
    """Advance SensorBank + SignalBank together; cmds: {ms: [(m, 'start'|'end')]}."""
    cmds = dict(cmds or {})
    while sb.now_ms < until_ms:
        for m, c in cmds.pop(sb.now_ms, []):
            (bank.start_green if c == "start" else bank.end_green)(m)
        step = min(dt_ms, until_ms - sb.now_ms)
        sb.advance(step / 1000, bank)
        bank.advance(step / 1000)


# =========================================================== layout / API
def test_sixteen_sensors():
    assert len(sn.SENSOR_IDS) == 16
    assert set(sn.SENSOR_IDS) == {(m, d) for m in MOVES for d in (D1, D2)}
    w = World(arrs([("NS", 0.0)]))
    for m in MOVES:
        for d in (D1, D2):
            assert w.sb.occupied(m, d) is False
    assert set(w.sb.snapshot()) == set(MOVES)


def test_study_constants():
    assert sn.DETECTOR_POSITION == {D1: 2.0, D2: 32.0}
    assert sn.D2_FILTER_MS == 2000 and sn.PASSAGE_TIME_MS == 2500
    assert sn.EXTENDED_INITIAL_GREEN_S == 12.0
    assert sn.QHAT == {DemandLevel.NONE: 0, DemandLevel.LEVEL_1: 2, DemandLevel.LEVEL_2: 6}


def test_sensor_bank_rejects_incomplete_source():
    class Partial(ScriptedSource):
        sensor_ids = sn.SENSOR_IDS[:-1]
    with pytest.raises(ValueError, match="16 sensors"):
        SensorBank(Partial([]))


# =========================================================== geometry
@pytest.mark.parametrize("m", MOVES)
@pytest.mark.parametrize("n,d1,d2", [(1, True, False), (2, True, False), (4, True, False),
                                     (5, True, True), (6, True, True), (12, True, True)])
def test_stopped_queue_geometry(m, n, d1, d2):
    w = World(arrs(queue_spec(m, n))).run(n * 2.0 + 20)
    assert w.vm.stopped_positions(m) == tuple(slot_position(k) for k in range(1, n + 1))
    assert (w.sb.d1(m), w.sb.d2(m)) == (d1, d2)


def test_slot_extents_match_detector_points():
    assert slot_position(1) <= 2.0 < slot_position(1) + vh.VEHICLE_LENGTH       # slot 1 covers D1
    assert slot_position(5) <= 32.0 < slot_position(5) + vh.VEHICLE_LENGTH      # slot 5 covers D2
    for k in (2, 3, 4, 6, 7):
        for p in (2.0, 32.0):
            assert not (slot_position(k) <= p < slot_position(k) + vh.VEHICLE_LENGTH)


def test_other_movements_unaffected():
    w = World(arrs(queue_spec("EL", 6))).run(40)
    assert w.sb.d1("EL") and w.sb.d2("EL")
    assert not any(w.sb.d1(m) or w.sb.d2(m) for m in MOVES if m != "EL")


# =========================================================== moving vehicles
@pytest.mark.parametrize("m", MOVES)
def test_moving_car_exact_edges_on_green(m):
    w = World(arrs([(m, 10.0)])).at(0, m, "G").run(40)
    e = 10000
    d2_on, d2_off = e + round(118 / V * 1000), e + round(122.5 / V * 1000)     # front at 32 / 27.5
    d1_on = e + round(148 / V * 1000)                                          # front at 2.0
    cross = e + 10800
    d1_off = cross + 278                                                       # rear at 2.0: -2.5 m in box at 9 m/s
    assert w.edges(m) == [(d2_on, D2, True), (d2_off, D2, False), (d1_on, D1, True), (d1_off, D1, False)]
    assert d2_off - d2_on == 324                                               # 4.5 m at 50 km/h
    assert w.vm.vehicle(0).passage_ms(32.0) == d2_on


def test_car_passing_d2_on_red_then_stopping_on_d1():
    w = World(arrs([("SS", 0.0)])).run(30)
    assert w.edges("SS") == [(8496, D2, True), (8820, D2, False), (10656, D1, True)]
    assert w.sb.d1("SS") and not w.sb.d2("SS")


def test_queue_discharge_edges_are_ordered():
    w = World(arrs(queue_spec("NS", 6))).at(100, "NS", "G").run(130)
    d1 = w.edges("NS", D1)
    assert all(a[0] < b[0] for a, b in zip(d1, d1[1:]))
    assert d1[-1] == (100000 + 36 + 5 * 1900 + 278, D1, False)            # last car's rear leaves
    assert [x[2] for x in d1[:3]] == [True, False, True]


# =========================================================== calls / levels (real vehicles)
def test_short_d2_pass_is_level_1_not_level_2():
    w = World(arrs([("NS", 0.0)])).run(9.0)
    assert w.sb.has_call("NS") and w.sb.demand_level("NS") == DemandLevel.LEVEL_1
    assert w.sb.call_since_ms("NS") == 8496                               # front reaches D2
    w.run(60)
    assert w.sb.demand_level("NS") == DemandLevel.LEVEL_1 and w.sb.qhat("NS") == 2


@pytest.mark.parametrize("m", MOVES)
def test_standing_queue_reaches_level_2_exactly(m):
    w = World(arrs(queue_spec(m, 5))).run(40)
    car5 = w.vm.vehicle(4)
    t_on = car5.passage_ms(32.0)
    s = w.sb.status(m)
    assert s.level2_latched and s.level2_since_ms == t_on + 2000 and s.qhat == 6
    assert s.d2_occupied_ms == w.vm.now_ms - t_on


def test_call_latched_after_pulse_clears():
    w = World(arrs([("WL", 0.0)])).run(8.9)
    assert not w.sb.d1("WL") and not w.sb.d2("WL") and w.sb.has_call("WL")   # D2 already cleared


# =========================================================== service / gap (real vehicles)
def test_extended_initial_and_gap_out_through_discharge():
    w = World(arrs(queue_spec("NS", 6))).at(100, "NS", "G")
    w.run(99.0)
    assert w.sb.level2_latched("NS") and not w.sb.in_service("NS")
    w.run(100.0)                                                   # GREEN begins at exactly 100.0
    s = w.sb.status("NS")
    assert s.in_service and s.extended_initial_required and s.green_onset_ms == 100000
    assert not s.has_call and s.demand_level == DemandLevel.NONE and s.gap_timer_ms == 0
    w.run(130)
    last_off = w.edges("NS", D1)[-1][0]
    gaps = []
    w2 = World(arrs(queue_spec("NS", 6))).at(100, "NS", "G")
    w2.run(100.0)
    w2.run(130, lambda w: gaps.append((w.vm.now_ms, w.sb.gap_out("NS"))))
    first_gap = next(t for t, g in gaps if g)
    assert first_gap == ((last_off + 2500 + 99) // 100) * 100             # first step boundary
    assert w.sb.gap_timer_ms("NS") == 130000 - last_off


def test_red_yellow_is_not_service_and_can_still_escalate():
    # queue of 5: car 5 reaches D2 at 16.496, LEVEL_2 at 18.496; RED_YELLOW 17.0-18.0; GREEN 18.0
    w = World(arrs(queue_spec("NS", 5))).at(17.0, "NS", "start")
    w.run(17.5)
    s = w.sb.status("NS")
    assert w.bank.state("NS") is SignalState.RED_YELLOW
    assert not s.in_service and s.gap_timer_ms is None and s.has_call
    w.run(18.0)
    s = w.sb.status("NS")
    assert s.in_service and s.green_onset_ms == 18000
    assert not s.extended_initial_required                         # LEVEL_2 would come at 18.496


def test_new_call_after_service_ends():
    # GREEN 100; YELLOW at 110: cars with D >= 112 are held; car on D1 at yellow -> call
    w = World(arrs(queue_spec("NS", 8))).at(100, "NS", "G").at(110, "NS", "Y")
    w.run(109.9)
    assert w.sb.in_service("NS") and not w.sb.has_call("NS")
    w.run(125)
    s = w.sb.status("NS")
    assert not s.in_service and s.has_call and s.demand_level >= DemandLevel.LEVEL_1
    assert s.call_since_ms >= 110000


def test_flashing_green_is_service_for_gap_logic():
    # GREEN at 0 for a lone car; end_green at 5 -> FLASHING_GREEN 5-8, YELLOW 8
    w = World(arrs([("ES", 20.0)])).at(0, "ES", "G").at(5.0, "ES", "end")
    w.run(5.0)
    assert w.sb.gap_out("ES") and w.sb.gap_timer_ms("ES") == 5000
    w.run(7.9)
    assert w.bank.state("ES") is SignalState.FLASHING_GREEN
    assert w.sb.in_service("ES") and w.sb.gap_timer_ms("ES") == 7900 and w.sb.green_onset_ms("ES") == 0
    w.run(8.0)
    assert not w.sb.in_service("ES") and w.sb.gap_timer_ms("ES") is None


# =========================================================== exact boundaries (scripted)
def _level_after_d2(duration_ms, dt_ms=1):
    src = scripted("NS", [(D2, 1000, 1000 + duration_ms)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 1000 + duration_ms + 500, dt_ms)
    return sb


@pytest.mark.parametrize("dt_ms", [1, 100, 700, 5000])
def test_d2_filter_1999_vs_2000(dt_ms):
    assert _level_after_d2(1999, dt_ms).demand_level("NS") == DemandLevel.LEVEL_1
    sb = _level_after_d2(2000, dt_ms)
    assert sb.demand_level("NS") == DemandLevel.LEVEL_2 and sb.status("NS").level2_since_ms == 3000


def test_d2_filter_live_boundary():
    src = scripted("SL", [(D2, 1000, None)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 2999)
    assert sb.d2_filter_ms("SL") == 1999 and sb.demand_level("SL") == DemandLevel.LEVEL_1
    drive(sb, bank, 3000)
    assert sb.d2_filter_ms("SL") == 2000 and sb.level2_latched("SL")


def test_d2_timer_resets_when_free():
    src = scripted("WS", [(D2, 1000, 2900), (D2, 3000, 4999)])        # 1900 + 1999 ms
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 6000, 100)
    assert sb.demand_level("WS") == DemandLevel.LEVEL_1
    src = scripted("WS", [(D2, 1000, 2900), (D2, 3000, 5000)])        # second pulse 2000 ms
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 6000, 100)
    assert sb.level2_latched("WS") and sb.status("WS").level2_since_ms == 5000


def test_level_2_stays_latched_until_green_onset():
    src = scripted("EL", [(D2, 1000, 3500), (D1, 500, None)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 10000, 100)
    assert not sb.d2("EL") and sb.level2_latched("EL") and sb.qhat("EL") == 6
    drive(sb, bank, 12000, 100, {10000: [("EL", "start")]})          # RED_YELLOW 10-11, GREEN 11
    assert sb.in_service("EL") and sb.extended_initial_required("EL")
    assert sb.demand_level("EL") == DemandLevel.NONE and not sb.has_call("EL")


def test_level_1_gives_qhat_2_and_d1_pulse_latches_call():
    src = scripted("NL", [(D1, 1000, 1300)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 5000, 100)
    s = sb.status("NL")
    assert not s.d1 and s.has_call and s.call_since_ms == 1000
    assert s.demand_level == DemandLevel.LEVEL_1 and s.qhat == 2


def test_no_call_without_activity():
    sb, bank = SensorBank(ScriptedSource([])), SignalBank()
    drive(sb, bank, 5000, 1000)
    assert all(not st.has_call and st.qhat == 0 for st in sb.snapshot().values())


@pytest.mark.parametrize("dt_ms", [1, 100, 1000])
def test_gap_timer_2499_vs_2500(dt_ms):
    # GREEN at 1000 (start at 0); D1 pulse 2000-3000; free from 3000
    src = scripted("NS", [(D1, 2000, 3000)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 5499, dt_ms, {0: [("NS", "start")]})
    if sb.now_ms == 5499:
        assert sb.gap_timer_ms("NS") == 2499 and not sb.gap_out("NS")
    drive(sb, bank, 5500, dt_ms)
    assert sb.gap_timer_ms("NS") == 2500 and sb.gap_out("NS")


def test_gap_timer_counts_from_green_onset_when_clear():
    sb, bank = SensorBank(ScriptedSource([])), SignalBank()
    drive(sb, bank, 3499, 1, {0: [("SS", "start")]})                 # GREEN at 1000
    assert sb.gap_timer_ms("SS") == 2499 and not sb.gap_out("SS")
    drive(sb, bank, 3500, 1)
    assert sb.gap_out("SS")


def test_activity_resets_gap_timer_and_clears_gap_out():
    src = scripted("NS", [(D2, 5000, 5300), (D1, 9000, None)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 4000, 100, {0: [("NS", "start")]})              # GREEN at 1000
    assert sb.gap_out("NS")
    drive(sb, bank, 5100, 100)
    assert sb.gap_timer_ms("NS") == 0 and not sb.gap_out("NS")      # D2 occupied
    drive(sb, bank, 7700, 100)
    assert sb.gap_timer_ms("NS") == 2400 and not sb.gap_out("NS")   # free since 5300
    drive(sb, bank, 7800, 100)
    assert sb.gap_out("NS")
    drive(sb, bank, 12000, 100)
    assert sb.gap_timer_ms("NS") == 0                                # D1 held on


def test_no_calls_during_service():
    src = scripted("WL", [(D1, 3000, 3200), (D2, 4000, 9000)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 10000, 100, {0: [("WL", "start")]})
    assert sb.in_service("WL") and not sb.has_call("WL") and sb.demand_level("WL") == DemandLevel.NONE


def test_detector_on_at_service_end_places_call_immediately():
    # GREEN 1000, end_green 2000 -> FG 2000-5000, YELLOW 5000; D1 on from 4000 (not cleared)
    src = scripted("NS", [(D1, 4000, None)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 6000, 100, {0: [("NS", "start")], 2000: [("NS", "end")]})
    assert sb.has_call("NS") and sb.call_since_ms("NS") == 5000


def test_d2_filter_starts_at_waiting_start_if_already_on():
    # D2 on from 4000 during service; waiting from 5000 (yellow) -> LEVEL_2 at 7000 not 6000
    src = scripted("NS", [(D2, 4000, None)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 6999, 1, {0: [("NS", "start")], 2000: [("NS", "end")]})
    assert sb.demand_level("NS") == DemandLevel.LEVEL_1 and sb.d2_occupied_ms("NS") == 2999
    drive(sb, bank, 7000, 1)
    assert sb.level2_latched("NS")


# =========================================================== extended initial at GREEN onset
def _ext_at_onset(level2_at_ms):
    # D2 on so that LEVEL_2 latches at level2_at_ms; RED_YELLOW at 9000, GREEN at 10000
    src = scripted("SL", [(D2, level2_at_ms - 2000, None)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 10500, 1, {9000: [("SL", "start")]})
    return sb.extended_initial_required("SL")


def test_extended_initial_captured_at_green_onset_not_red_yellow():
    assert _ext_at_onset(9500) is True        # latched during RED_YELLOW: counts
    assert _ext_at_onset(10000) is True       # latched exactly at GREEN onset: counts
    assert _ext_at_onset(10001) is False      # 1 ms after GREEN onset: too late
    assert _ext_at_onset(8999) is True


def test_extended_initial_false_outside_service():
    src = scripted("SL", [(D2, 1000, None)])
    sb, bank = SensorBank(src), SignalBank()
    drive(sb, bank, 12000, 100, {5000: [("SL", "start")], 8000: [("SL", "end")]})
    assert not sb.in_service("SL") and not sb.extended_initial_required("SL")


# =========================================================== all 8 movements lifecycle
@pytest.mark.parametrize("m", MOVES)
def test_full_lifecycle_each_movement(m):
    w = World(arrs(queue_spec(m, 6))).at(60, m, "G").at(80, m, "Y")
    w.run(59.9)
    s = w.sb.status(m)
    assert s.has_call and s.level2_latched and not s.in_service
    w.run(60.0)
    s = w.sb.status(m)
    assert s.in_service and s.extended_initial_required and not s.has_call
    w.run(79.9)
    assert w.sb.in_service(m) and w.sb.gap_out(m)                      # 6 cars cleared long before
    w.run(90)
    s = w.sb.status(m)
    assert not s.in_service and not s.has_call and s.qhat == 0


# =========================================================== dt independence / determinism
def _full(dt):
    sched = generate_arrivals("S1", 4, -300, 200)
    w = World(sched.arrivals, start=-300, dt=dt)
    for k in range(-299, 200, 60):              # every command on the 2 s grid (k odd)
        w.at(k, "NS", "G").at(k, "SS", "G").at(k + 24, "NS", "Y").at(k + 24, "SS", "Y")
        w.at(k + 32, "EL", "G").at(k + 32, "WL", "G").at(k + 46, "EL", "Y").at(k + 46, "WL", "Y")
    snaps = {}
    def chk(w):
        if w.vm.now_ms % 2000 == 0:
            snaps[w.vm.now_ms] = w.sb.snapshot()
    w.run(240, chk)
    return w, snaps


@pytest.mark.parametrize("dt", [0.5, 1.0, 2.0])
def test_results_independent_of_dt(dt):
    ref, ref_snaps = _full(0.1)
    other, snaps = _full(dt)
    assert other.sb.edge_log == ref.sb.edge_log
    for t, snap in snaps.items():
        assert snap == ref_snaps[t], t


def test_deterministic_repeat():
    a, sa = _full(0.1)
    b, sb_ = _full(0.1)
    assert a.sb.edge_log == b.sb.edge_log and sa == sb_


def test_large_dt_does_not_miss_short_pulses():
    w = World(arrs([("NS", 0.0)]), dt=2.0).run(12.0)
    assert (8496, D2, True) in w.edges("NS") and (8820, D2, False) in w.edges("NS")
    assert w.sb.call_since_ms("NS") == 8496


# =========================================================== interface / validation
def test_signal_clock_must_match():
    sb, bank = SensorBank(ScriptedSource([])), SignalBank()
    bank.advance(0.1)
    with pytest.raises(ValueError, match="sensors.advance"):
        sb.advance(0.1, bank)


def test_virtual_source_requires_vehicle_model_ahead():
    vm = VehicleModel(arrs([("NS", 0.0)]), start=0)
    det = VirtualDetectors(vm)
    with pytest.raises(ValueError, match="advance it first"):
        det.poll(100)


def test_virtual_detectors_must_be_created_before_first_advance():
    vm = VehicleModel(arrs([("NS", 0.0)]), start=0)
    vm.advance(0.1, SignalBank())
    with pytest.raises(RuntimeError, match="before the first advance"):
        VirtualDetectors(vm)


def test_source_edge_outside_step_rejected():
    class Bad(ScriptedSource):
        def poll(self, until_ms):
            self._now = until_ms
            return [DetectorEdge(until_ms + 5, "NS", D1, True)]
    sb = SensorBank(Bad([]))
    with pytest.raises(ValueError, match="outside"):
        sb.advance(0.1, SignalBank())


@pytest.mark.parametrize("bad", ["XX", "ns", None, 3])
def test_invalid_movement(bad):
    sb = SensorBank(ScriptedSource([]))
    for q in (sb.d1, sb.d2, sb.has_call, sb.demand_level, sb.qhat, sb.gap_out, sb.gap_timer_ms,
              sb.level2_latched, sb.extended_initial_required, sb.status, sb.d2_occupied_ms):
        with pytest.raises(InvalidMovementError):
            q(bad)


@pytest.mark.parametrize("bad", ["D3", "d1", None, 1])
def test_invalid_detector(bad):
    with pytest.raises(ValueError, match="detector"):
        SensorBank(ScriptedSource([])).occupied("NS", bad)


@pytest.mark.parametrize("dt", [-0.1, float("nan"), "0.1", None, True, 0.0001])
def test_bad_dt(dt):
    with pytest.raises(ValueError):
        SensorBank(ScriptedSource([])).advance(dt, SignalBank())


def test_snapshot_is_immutable_and_has_no_vehicle_objects():
    w = World(arrs(queue_spec("NS", 3))).run(20)
    s = w.sb.status("NS")
    with pytest.raises(Exception):
        s.has_call = False
    assert all(not isinstance(getattr(s, f), vh.Vehicle) for f in s.__dataclass_fields__)


def test_watch_positions_is_passive():
    spec = queue_spec("NS", 6)
    plain = VehicleModel(arrs(spec), start=-5)
    watched = VehicleModel(arrs(spec), start=-5)
    watched.watch_positions(2.0, -2.5, 32.0, 27.5)
    b1, b2 = SignalBank(start_time=-5), SignalBank(start_time=-5)
    for _ in range(600):
        plain.advance(0.1, b1); watched.advance(0.1, b2); b1.advance(0.1); b2.advance(0.1)
    key = lambda vm: [(v.id, v.position, v.cross_ms, v.queue_join_ms) for v in vm.all_vehicles()]
    assert key(plain) == key(watched)


@pytest.mark.parametrize("bad", [150.0, 200, float("nan"), "2", True])
def test_watch_positions_validation(bad):
    with pytest.raises(ValueError):
        VehicleModel(arrs([("NS", 0.0)]), start=0).watch_positions(bad)


# =========================================================== independence
def test_no_wall_clock_or_global_random(monkeypatch):
    ref = _full(1.0)[0].sb.edge_log
    def boom(*a, **k):
        raise AssertionError("forbidden")
    for name in ("time", "monotonic", "perf_counter", "sleep", "time_ns"):
        monkeypatch.setattr(time, name, boom)
    for name in ("random", "uniform", "choice"):
        monkeypatch.setattr(random, name, boom)
    for name in ("random", "rand", "seed"):
        monkeypatch.setattr(np.random, name, boom)
    assert _full(1.0)[0].sb.edge_log == ref


def test_forbidden_imports():
    tree = ast.parse(Path(sn.__file__).read_text(encoding="utf-8"))
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    for bad in ("serial", "pygame", "controller", "machine", "esp", "random", "time", "numpy"):
        assert not any(bad in (m or "").lower() for m in mods), bad
    assert not any(isinstance(n, ast.Global) for n in ast.walk(tree))
