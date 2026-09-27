"""Tests for smarttraffic.controller_smart (Stage 11: movement-based actuated controller).

Most tests drive a real SensorBank from scripted detector edges (exact ms control) and a
real SignalBank; integration tests use the full stack (VehicleModel -> VirtualDetectors
or HybridDetectorSource -> SensorBank -> SmartController -> SignalBank).
Step order everywhere: [vehicles] -> sensors -> SignalBank -> controller.
"""
import ast
import hashlib
import inspect
import json
import os
import random
import subprocess
import sys
from pathlib import Path

import pytest

from smarttraffic import conflicts
from smarttraffic import controller_smart as cs
from smarttraffic import signals as sg
from smarttraffic.conflicts import MOVES, compatible
from smarttraffic.controller_smart import (END_GREEN, GROUP_ORDER, START_GREEN, Command, SmartController,
                                           SmartParams)
from smarttraffic.sensors import (EXTENDED_INITIAL_GREEN_S, QHAT, SENSOR_IDS, DemandLevel, DetectorEdge,
                                  SensorBank, VirtualDetectors)
from smarttraffic.signals import (ALL_RED, FLASHING_GREEN, GREEN, RED, RED_YELLOW, YELLOW, SignalBank)

ROOT = Path(__file__).resolve().parent.parent
NO_MAX = SmartParams(use_max_green=False)


# =============================================================== scripted rig
class Src:
    """DetectorSource from a fixed edge list; optional fault windows [(sid, from_ms, until_ms)]."""
    sensor_ids = SENSOR_IDS

    def __init__(self, edges=(), faults=(), start_ms=0):
        self.edges = sorted(edges, key=lambda e: e.time_ms)
        self.fault_windows = list(faults)
        self._now = start_ms

    @property
    def now_ms(self):
        return self._now

    def initial_occupancy(self):
        return {s: False for s in SENSOR_IDS}

    def poll(self, until):
        out = [e for e in self.edges if self._now < e.time_ms <= until]
        self._now = until
        return out

    def faults(self):
        return tuple(sid for sid, a, b in self.fault_windows if a <= self._now and (b is None or self._now < b))


def on(m, d, t_on, t_off=None):
    out = [DetectorEdge(t_on, m, d, True)]
    if t_off is not None:
        out.append(DetectorEdge(t_off, m, d, False))
    return out


class Rig:
    def __init__(self, edges=(), params=SmartParams(), faults=()):
        self.src = Src(list(edges), faults)
        self.sb = SensorBank(self.src)
        self.bank = SignalBank()
        self.ctrl = SmartController(self.sb, params)
        self.snapshots = {}
        self.ctrl.update(self.bank)

    @property
    def now(self):
        return self.sb.now_ms

    def run(self, until_ms, dt_ms=100, each=None, watch=()):
        while self.now < until_ms:
            step = min(dt_ms, until_ms - self.now)
            self.sb.advance(step / 1000, self.bank)
            self.bank.advance(step / 1000)
            self.ctrl.update(self.bank)
            conflicts.check_invariant(self.bank.active())
            if self.now in watch:
                self.snapshots[self.now] = (self.ctrl.last_target, self.ctrl.last_hold)
            if each:
                each(self)
        return self

    def tl(self, *moves):
        return [(round(e.time * 1000), e.movement, e.new.name) for e in self.bank.log
                if not moves or e.movement in moves]

    def t_of(self, m, state, nth=0):
        ts = [round(e.time * 1000) for e in self.bank.log if e.movement == m and e.new is state]
        return ts[nth] if len(ts) > nth else None


# ================================================================ parameters / API
def test_study_parameter_values():
    p = SmartParams()
    assert (p.min_green_ms("straight", False), p.min_green_ms("left", False)) == (7000, 5000)
    assert p.min_green_ms("straight", True) == p.min_green_ms("left", True) == 12000
    assert (p.max_green_ms("straight"), p.max_green_ms("left")) == (45000, 30000)
    assert p.tau_ms == 10000 and p.w_max_ms == 90000
    assert p.green_bonus_qhat == 2 and p.fault_qhat == 2
    assert p.use_max_green and p.use_aging and p.use_w_max


def test_parameters_agree_with_sensor_layer():
    assert SmartParams().extended_initial_s == EXTENDED_INITIAL_GREEN_S
    assert SmartParams().fault_qhat == QHAT[DemandLevel.LEVEL_1]


@pytest.mark.parametrize("kw", [dict(min_green_left_s=2.9), dict(max_green_left_s=11.0),
                                dict(tau_s=0), dict(w_max_s=1e-4), dict(fault_qhat=-1),
                                dict(green_bonus_qhat=1.5), dict(min_green_straight_s=float("nan"))])
def test_invalid_parameters_rejected(kw):
    with pytest.raises(ValueError):
        SmartParams(**kw)


def test_params_are_immutable_and_typed():
    with pytest.raises(Exception):
        SmartParams().tau_s = 5.0
    with pytest.raises(TypeError):
        SmartController(SensorBank(Src()), params={"tau_s": 10})


def test_group_order_is_the_conflict_models_order_and_never_set_order():
    assert GROUP_ORDER == (("NS", "SS"), ("NL", "SL"), ("NS", "NL"), ("SS", "SL"),
                           ("ES", "WS"), ("EL", "WL"), ("ES", "EL"), ("WS", "WL"))
    assert [frozenset(g) for g in GROUP_ORDER] == list(conflicts.GROUPS)


def test_update_requires_sensors_and_signals_at_the_same_time():
    r = Rig()
    r.sb.advance(0.1, r.bank)                       # sensors ahead of the SignalBank
    with pytest.raises(ValueError, match="after sensors and SignalBank"):
        r.ctrl.update(r.bank)


def test_time_going_backwards_rejected():
    sb = SensorBank(Src(start_ms=5000))
    c = SmartController(sb)
    c.update(SignalBank(start_time=5.0))
    sb2 = SensorBank(Src(start_ms=4000))
    c.sensors = sb2
    with pytest.raises(ValueError, match="backwards"):
        c.update(SignalBank(start_time=4.0))


# ================================================================ 1-2. no demand / one demand
def test_no_demand_no_commands():
    r = Rig().run(300000)
    assert r.ctrl.command_log == [] and r.bank.log == []
    assert r.ctrl.last_target is None


def test_timeline_A_single_demand_served_and_rests_in_green():
    r = Rig(on("NS", "D1", 1000, 1500)).run(300000)
    assert r.tl() == [(1000, "NS", "RED_YELLOW"), (2000, "NS", "GREEN")]
    assert r.ctrl.command_log == [Command(1000, START_GREEN, "NS")]
    assert r.bank.state("NS") is GREEN                 # rest in green: no conflicting call


def test_single_left_call_runs_alone():
    r = Rig(on("WL", "D2", 700, 1000)).run(20000)
    assert r.tl() == [(700, "WL", "RED_YELLOW"), (1700, "WL", "GREEN")]
    assert r.bank.active() == ("WL",)


# ================================================================ 3-4. pairs
def test_compatible_pair_called_together_starts_together():
    r = Rig(on("NS", "D1", 1000, 1200) + on("SS", "D1", 1000, 1200)).run(5000)
    assert r.tl() == [(1000, "NS", "RED_YELLOW"), (1000, "SS", "RED_YELLOW"),
                      (2000, "NS", "GREEN"), (2000, "SS", "GREEN")]


def test_incompatible_pair_never_together_and_both_served():
    r = Rig(on("NS", "D1", 1000, 1200) + on("ES", "D1", 1000, 1200)).run(60000)
    assert r.t_of("NS", GREEN) == 2000                  # tie -> fixed GROUPS order: {NS,SS} first
    # NS gapped out at 4500, minimum green 7 s -> end_green at 2000 + 4000
    assert r.t_of("NS", FLASHING_GREEN) == 6000
    assert r.t_of("NS", RED) == 14500
    assert r.t_of("ES", RED_YELLOW) == 14500 and r.t_of("ES", GREEN) == 15500


# ================================================================ 5-7. priority and ties
def test_exact_tie_uses_fixed_groups_order():
    # NS, NL, ES, EL all called at the same instant: every group scores 2 + 2 = 4 or 2
    r = Rig([e for m in ("EL", "ES", "NL", "NS") for e in on(m, "D1", 1000, 1200)]).run(3000)
    # {NS,SS}=2, {NL,SL}=2, {NS,NL}=4, {ES,EL}=4 -> {NS,NL} (earlier in GROUPS order)
    assert r.tl()[:2] == [(1000, "NS", "RED_YELLOW"), (1000, "NL", "RED_YELLOW")]
    assert r.bank.state("ES") is RED and r.bank.state("EL") is RED


def test_tie_prefers_group_keeping_more_serving_movements():
    # study §7 example at t = 4.0 s: {NS,SS} and {SS,SL} both score 4 -> keep {SS,SL}
    r = Rig(_study_example_edges()).run(4100, watch=(4000, 4100))
    assert r.snapshots[4000][0] == ("SS", "SL")
    assert r.snapshots[4100][0] == ("NS", "SS")         # NS has waited 0.1 s longer


def _qhat_rig(sl_call):
    # ES green with continuous detection; NS called at 2.0 s (LEVEL_1); SL called later and
    # latches LEVEL_2 2 s after its call. ES max-out: max timer from 2.0 s -> end_green 44.0 s.
    edges = on("ES", "D1", 100) + on("NS", "D1", 2000, 2300) + on("SL", "D2", sl_call, sl_call + 4000)
    return Rig(edges).run(60000)


def test_higher_qhat_wins_at_equal_wait():
    edges = on("ES", "D1", 100, 3000) + on("NS", "D1", 2000, 2300) + on("SL", "D2", 2000, 4500)
    r = Rig(edges).run(30000)
    # ES: onset 1100, gap-out 5500, minimum met 1100 + 4000 -> end_green 5500
    assert r.t_of("ES", FLASHING_GREEN) == 5500
    first = r.ctrl.decision_log[0]
    assert first.time_ms == 5500 and first.target == ("NL", "SL")     # SL: Qhat 6 beats NS: Qhat 2


@pytest.mark.parametrize("sl_call, winner", [(41900, "SL"), (42000, "NS"), (42100, "NS")])
def test_waiting_time_ages_priority_40s_equals_4_cars(sl_call, winner):
    r = _qhat_rig(sl_call)
    d = r.ctrl.decision_log[0]
    assert d.time_ms == 44000 and d.stopped == (("ES", "max"),)
    # at 44.0 s: P(NS) = 2 + 42.0/10 = 6.2;  P(SL) = 6 + (44.0 - call)/10
    assert winner in d.target
    # exact tie at 42.0 s: equal score and serving count -> longest single wait (NS)


def test_exact_tie_goes_to_longest_waiting_call_even_if_later_in_groups_order():
    # mirror of the 42.0 s case: old LEVEL_1 call on SL (group index 1), new LEVEL_2 call on
    # NS (index 0) latched exactly so that P(SL) = P(NS) = 6.2 at the 44.0 s decision
    edges = on("ES", "D1", 100) + on("SL", "D1", 2000, 2300) + on("NS", "D2", 42000, 46000)
    d = Rig(edges).run(60000).ctrl.decision_log[0]
    assert d.time_ms == 44000 and d.stopped == (("ES", "max"),)
    assert "SL" in d.target and "NS" not in d.target


def test_no_aging_ablation_ignores_waiting_time():
    r_aging = Rig(on("ES", "D1", 100) + on("NS", "D1", 2000, 2300) + on("SL", "D2", 43000, 46000)).run(60000)
    r_flat = Rig(on("ES", "D1", 100) + on("NS", "D1", 2000, 2300) + on("SL", "D2", 43000, 46000),
                 SmartParams(use_aging=False)).run(60000)
    assert "NS" in r_aging.ctrl.decision_log[0].target        # waited 42 s: 6.2 vs SL 2.1
    assert r_aging.ctrl.decision_log[0].time_ms == 44000
    # without aging NS scores 2 = the running ES group's continuity bonus: the tie keeps the
    # serving group, so nothing switches until SL latches LEVEL_2 (Qhat 6) at 45.0 s
    d = r_flat.ctrl.decision_log[0]
    assert (d.time_ms, d.target) == (45000, ("NL", "SL"))


# ================================================================ 8-11. minimum green, LEVEL_2, gap-out
def _min_rig(ns_d2=None, es_again=16000):
    """ES served first; NS waits with D1 call (and optional D2 window) until ES clears at 13.5 s;
    NS GREEN at 14.5 s; ES calls again at es_again so NS must be ended."""
    edges = on("ES", "D1", 100, 400) + on("NS", "D1", 500, 800) + on("ES", "D1", es_again, es_again + 300)
    if ns_d2:
        edges += on("NS", "D2", *ns_d2)
    return Rig(edges).run(60000)


def test_normal_minimum_green_straight():
    r = _min_rig()
    # ES: GREEN 1.1 s, gap-out 3.6 s, minimum met 1.1 + 4.0 = 5.1 s -> clears, NS RY 13.6 s
    assert r.t_of("ES", FLASHING_GREEN) == 5100
    assert r.t_of("NS", GREEN) == 14600
    # NS gapped out at 17.1 s (clear since onset), min 7 s met at 14.6 + 4.0 = 18.6 s
    assert r.t_of("NS", FLASHING_GREEN) == 18600
    assert r.t_of("NS", YELLOW) - r.t_of("NS", GREEN) == 7000          # G = 7 s


def test_level2_at_onset_gives_12s_minimum_green():
    r = _min_rig(ns_d2=(600, 17000))                  # D2 continuously occupied: LEVEL_2 at 2.6 s
    assert r.t_of("NS", GREEN) == 14600
    # D2 clears at 17.0 -> gap-out 19.5; min 12 s met at 14.6 + 9.0 = 23.6
    assert r.t_of("NS", FLASHING_GREEN) == 23600
    assert r.t_of("NS", YELLOW) - r.t_of("NS", GREEN) == 12000


@pytest.mark.parametrize("d2_on, extended", [(12600, True), (12601, False)])
def test_level2_latched_during_red_yellow_counts_exactly(d2_on, extended):
    # NS RED_YELLOW 13.6-14.6 s; D2 must be continuously on for 2000 ms by the GREEN onset 14.6 s
    r = _min_rig(ns_d2=(d2_on, 15000))
    assert r.t_of("NS", GREEN) == 14600
    g = r.t_of("NS", YELLOW) - r.t_of("NS", GREEN)
    # D2 off at 15.0 -> gap-out 17.5; min 12 s -> end 23.6 (G 12), min 7 s -> end 18.6 (G 7)
    assert g == (12000 if extended else 7000)


@pytest.mark.parametrize("m, d2, expect_end", [("NS", None, 18600), ("NS", (600, 14000), 23600)])
def test_minimum_green_boundary_at_1ms_resolution(m, d2, expect_end):
    # GREEN at 14.6 s; the movement gaps out before its minimum green, so it ends exactly at
    # onset + MIN - 3.0 s: straight 7 s, LEVEL_2 12 s. Stepped at 1 ms. (A left's 5 s minimum
    # always ends before its earliest gap-out; see the hold test below for the left boundary.)
    edges = on("ES", "D1", 100, 400) + on(m, "D1", 500, 800) + on("ES", "D1", 15000, 15300)
    if d2:
        edges += on(m, "D2", *d2)
    r = Rig(edges)
    r.run(14000)
    r.run(30000, dt_ms=1)
    assert r.t_of(m, GREEN) == 14600
    assert r.t_of(m, FLASHING_GREEN) == expect_end
    assert r.t_of(m, YELLOW) - r.t_of(m, GREEN) == expect_end + 3000 - 14600


def test_normal_minimum_green_left():
    edges = on("ES", "D1", 100, 400) + on("NL", "D1", 500, 800) + on("ES", "D1", 16000, 16300)
    r = Rig(edges).run(60000)
    assert r.t_of("NL", GREEN) == 14600
    # NL gapped out at 17.1; left min 5 s met at 14.6 + 2.0 = 16.6 -> end_green 17.1
    assert r.t_of("NL", FLASHING_GREEN) == 17100


def test_gap_out_ends_green_exactly_2_5s_after_detectors_clear():
    edges = on("NS", "D1", 1000, 20000) + on("ES", "D1", 5000, 5300)
    r = Rig(edges).run(60000)
    assert r.t_of("NS", FLASHING_GREEN) == 22500
    assert r.ctrl.decision_log[0].stopped == (("NS", "gap"),)


def test_short_gap_does_not_end_green():
    # D1 clear for 2.4 s (19.0-21.4) then occupied again until 30.0 -> no gap-out until 32.5
    edges = on("NS", "D1", 1000, 19000) + on("NS", "D1", 21400, 30000) + on("ES", "D1", 5000, 5300)
    r = Rig(edges).run(60000)
    assert r.t_of("NS", FLASHING_GREEN) == 32500


def test_no_gap_out_release_before_minimum_green():
    r = _min_rig(es_again=15000)                       # NS gapped out at 17.1 < min at 18.6
    assert r.t_of("NS", FLASHING_GREEN) == 18600


# ================================================================ 12-14. transitions
def _study_example_edges():
    # SS and SL called at 0.1 s; SL serves cars until 8.5 s; SS keeps serving; NS arrives at 4.0 s
    return (on("SS", "D1", 100, 200) + on("SL", "D1", 100, 200) + on("SL", "D1", 1500, 8500)
            + on("SS", "D1", 1500, 60000) + on("NS", "D1", 4000))


def test_timeline_C_study_example_partial_transition():
    r = Rig(_study_example_edges()).run(40000)
    assert r.tl() == [(100, "SS", "RED_YELLOW"), (100, "SL", "RED_YELLOW"),
                      (1100, "SS", "GREEN"), (1100, "SL", "GREEN"),
                      (11000, "SL", "FLASHING_GREEN"), (14000, "SL", "YELLOW"),
                      (17000, "SL", "ALL_RED"), (20000, "SL", "RED"),
                      (20000, "NS", "RED_YELLOW"), (21000, "NS", "GREEN")]
    d = r.ctrl.decision_log
    assert len(d) == 1 and d[0].stopped == (("SL", "gap"),) and d[0].to_start == ("NS",)
    assert r.bank.state("SS") is GREEN and r.bank.since("SS") == 1.1    # never interrupted


def test_timers_survive_partial_transition():
    r = Rig(_study_example_edges()).run(30000)
    assert r.ctrl.green_onset_ms("SS") == 1100                          # original onset kept
    assert r.ctrl.min_green_of("SS") == 7000
    assert r.ctrl.max_start_ms("SS") is None     # NS is compatible with SS: no max timer for SS
    r2 = Rig(_study_example_edges()).run(10000)
    assert r2.ctrl.max_start_ms("SL") == 4000    # NS conflicts with SL: SL's max timer started


def test_commitment_locks_target_until_green():
    # a strong EL call during the SL -> NS transition does not change the committed target
    edges = _study_example_edges() + on("EL", "D2", 12000, 16000)
    r = Rig(edges).run(22000)
    assert r.t_of("NS", GREEN) == 21000
    assert r.bank.state("EL") is RED
    assert r.ctrl.committed() is None                                    # released at NS GREEN


def test_timeline_B_compatible_demand_added_without_stopping():
    edges = on("NS", "D1", 1000, 1500) + on("SS", "D1", 5000, 5300)
    r = Rig(edges).run(30000)
    assert r.tl() == [(1000, "NS", "RED_YELLOW"), (2000, "NS", "GREEN"),
                      (5000, "SS", "RED_YELLOW"), (6000, "SS", "GREEN")]
    assert r.ctrl.decision_log == []                    # nothing was stopped
    assert r.ctrl.max_start_ms("NS") is None            # a compatible call starts no max timer


def test_only_conflicting_green_is_stopped():
    # NS + NL green (resting); SS called: SS conflicts NL only -> only NL stops, NS stays
    edges = (on("NS", "D1", 1000, 1300) + on("NL", "D1", 1000, 1300) + on("SS", "D1", 10000, 10300))
    r = Rig(edges).run(40000)
    assert r.ctrl.decision_log[0].stopped == (("NL", "gap"),)
    assert r.t_of("NS", FLASHING_GREEN) is None and r.bank.state("NS") is GREEN
    assert r.t_of("SS", GREEN) == r.t_of("NL", RED) + 1000


def test_conflicting_demand_causes_full_safe_sequence():
    r = Rig(on("NS", "D1", 1000, 1500) + on("EL", "D1", 3000, 3300)).run(40000)
    seq = [s for t, m, s in r.tl("NS")]
    assert seq == ["RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED", "RED"]
    assert r.t_of("EL", RED_YELLOW) == r.t_of("NS", RED)


# ================================================================ target protection
def test_target_protection_blocks_compatible_addition_that_conflicts_with_target():
    # NS green; EL waits long with LEVEL_2; NL (compatible with NS, conflicts EL) calls later.
    edges = (on("NS", "D1", 1000, 30000) + on("EL", "D2", 2000, 60000) + on("NL", "D1", 5000, 5300))
    r = Rig(edges).run(60000)
    el_green = r.t_of("EL", GREEN)
    nl_start = r.t_of("NL", RED_YELLOW)
    assert el_green is not None
    assert nl_start is None or nl_start >= el_green     # NL never jumped ahead of the target
    assert r.bank.log[0].movement == "NS"


def test_stream_of_compatible_calls_cannot_delay_higher_priority_target():
    # NS green with a stream of NL calls every 4 s; the EL target is still served when NS ends
    edges = on("NS", "D1", 1000, 25000) + on("EL", "D2", 2000, 9000)
    for t in range(4000, 60000, 4000):
        edges += on("NL", "D1", t, t + 300)
    r = Rig(edges).run(90000)
    # NS gap-out at 27.5 s (min met) -> EL served; NL not started before EL
    assert r.t_of("NS", FLASHING_GREEN) == 27500
    assert r.t_of("EL", GREEN) is not None
    nl = r.t_of("NL", RED_YELLOW)
    assert nl is None or nl >= r.t_of("EL", GREEN)


# ================================================================ rest in green / max green
def test_rest_in_green_after_gap_out_without_conflicting_demand():
    r = Rig(on("NS", "D1", 1000, 1500) + on("SS", "D1", 1000, 1500)).run(600000)
    assert r.bank.state("NS") is GREEN and r.bank.state("SS") is GREEN
    assert r.sb.gap_out("NS") and r.ctrl.command_log == [Command(1000, START_GREEN, "NS"),
                                                         Command(1000, START_GREEN, "SS")]


def test_timeline_max_green_straight_45s_from_conflicting_call():
    edges = on("NS", "D1", 1000) + on("ES", "D1", 10000, 10300)       # NS never gaps out
    r = Rig(edges).run(80000)
    assert r.ctrl.decision_log[0].stopped == (("NS", "max"),)
    assert r.t_of("NS", FLASHING_GREEN) == 10000 + 45000 - 3000       # 52.0 s
    assert r.t_of("NS", YELLOW) == 55000                               # service ends 45 s after call
    assert r.tl()[2:8] == [(52000, "NS", "FLASHING_GREEN"), (55000, "NS", "YELLOW"),
                          (58500, "NS", "ALL_RED"), (60500, "NS", "RED"),
                          (60500, "ES", "RED_YELLOW"), (61500, "ES", "GREEN")]


def test_max_green_left_30s():
    edges = on("NL", "D1", 1000) + on("EL", "D1", 10000, 10300)
    r = Rig(edges).run(80000)
    assert r.t_of("NL", FLASHING_GREEN) == 10000 + 30000 - 3000


def test_max_timer_starts_at_onset_when_conflicting_call_already_waits():
    edges = on("ES", "D1", 100) + on("NS", "D1", 500)               # both stuck on
    r = Rig(edges).run(60000)
    # ES GREEN at 1.1 s with NS already calling -> max from 1.1 s -> end_green 43.1 s
    assert r.ctrl.decision_log[0].stopped == (("ES", "max"),)
    assert r.t_of("ES", FLASHING_GREEN) == 1100 + 42000


def test_max_green_not_started_without_conflicting_call():
    edges = on("NS", "D1", 1000) + on("ES", "D1", 150000, 150300)
    r = Rig(edges).run(240000)
    assert r.t_of("NS", FLASHING_GREEN) == 150000 + 42000


def test_max_green_ablation_switch():
    edges = on("NS", "D1", 1000) + on("ES", "D1", 10000, 10300)
    r = Rig(edges, NO_MAX).run(80000)
    assert r.t_of("NS", FLASHING_GREEN) is None


# ================================================================ W_MAX starvation hold
@pytest.mark.parametrize("call_ms, end_ms", [(1000, 91000), (1001, 91100)])
def test_hold_begins_exactly_at_90000_ms(call_ms, end_ms):
    # NS stuck green (no max green in this isolated test); EL can only be served by the hold
    r = Rig(on("NS", "D1", 1000) + on("EL", "D1", call_ms, call_ms + 300), NO_MAX)
    r.run(end_ms - 100, watch=())
    assert r.ctrl.last_hold is None and r.bank.state("NS") is GREEN
    r.run(end_ms)
    assert r.ctrl.last_hold == "EL"
    assert r.ctrl.decision_log[-1].stopped == (("NS", "hold"),)
    assert r.t_of("NS", FLASHING_GREEN) == end_ms   # 89 999 ms of waiting is not yet a hold


def test_timeline_starvation_hold():
    r = Rig(on("NS", "D1", 1000) + on("EL", "D1", 1000, 1300), NO_MAX).run(110000)
    assert r.tl()[:8] == [(1000, "NS", "RED_YELLOW"), (2000, "NS", "GREEN"),
                      (91000, "NS", "FLASHING_GREEN"), (94000, "NS", "YELLOW"),
                      (97500, "NS", "ALL_RED"), (99500, "NS", "RED"),
                      (99500, "EL", "RED_YELLOW"), (100500, "EL", "GREEN")]
    # hold at 90 s; GREEN 99.5 s after the call (single hold, straight clearance)


def test_multiple_simultaneous_holds_documented_behaviour():
    # NS stuck green; EL and WS (conflicting with each other and with NS) both called at 1.0 s.
    r = Rig(on("NS", "D1", 1000) + on("EL", "D1", 1000, 1300) + on("WS", "D1", 1000, 1300),
            NO_MAX).run(140000)
    # both reach 90 s at 91.0 s; hold = longest wait, tie -> MOVES order -> EL
    assert r.t_of("NS", FLASHING_GREEN) == 91000
    assert r.t_of("EL", GREEN) == 100500
    # WS is now the hold; EL is released by the hold only once ITS minimum green is met
    # (left: onset 100.5 + 5 - 3 = 102.5 s), not earlier
    assert r.t_of("EL", FLASHING_GREEN) == 102500
    assert r.ctrl.decision_log[1].stopped == (("EL", "hold"),)
    assert r.t_of("WS", RED_YELLOW) == 111500 and r.t_of("WS", GREEN) == 112500
    # measured: the second hold reaches GREEN 111.5 s after its call


def test_left_minimum_green_boundary_under_hold_at_1ms_resolution():
    r = Rig(on("NS", "D1", 1000) + on("EL", "D1", 1000, 1300) + on("WS", "D1", 1000, 1300), NO_MAX)
    r.run(100000)
    r.run(104000, dt_ms=1)
    assert r.t_of("EL", GREEN) == 100500
    assert r.t_of("EL", FLASHING_GREEN) == 100500 + 5000 - 3000      # not 1 ms earlier


def test_hold_waits_for_partner_blocker_study_policy():
    """Study policy, literally: hold NL's best group is {NL,SL}; NS (compatible with NL, but
    conflicting with SL) must gap out or max out first. With max green disabled and NS stuck
    on, NL keeps waiting - this is the documented limitation, not a bug."""
    edges = (on("NS", "D1", 1000, 150000) + on("SS", "D1", 1000, 150000)
             + on("NL", "D1", 1100, 1400) + on("SL", "D2", 1100, 4000))
    r = Rig(edges, NO_MAX).run(170000)
    d = r.ctrl.decision_log[0]
    # NL is the hold from 91.1 s, but the switch needs ALL blockers releasable at once:
    # SS (conflicts NL) is releasable by the hold, NS (conflicts SL only) must gap out (152.5 s)
    assert d.time_ms == 152500 and d.hold == "NL" and d.target == ("NL", "SL")
    assert d.stopped == (("NS", "gap"), ("SS", "hold"))
    assert r.t_of("NL", RED_YELLOW) == 161000 and r.t_of("SL", RED_YELLOW) == 161000
    assert r.t_of("NL", RED_YELLOW) - 1100 == 159900          # measured: 159.9 s at RED


def test_w_max_ablation_switch():
    r = Rig(on("NS", "D1", 1000) + on("EL", "D1", 1000, 1300), SmartParams(use_max_green=False,
                                                                           use_w_max=False)).run(200000)
    assert r.t_of("NS", FLASHING_GREEN) is None


# ================================================================ fault recall
def test_faulted_detector_places_recall_demand():
    r = Rig(faults=[(("NS", "D1"), 0, None)]).run(5000)
    assert r.ctrl.faulted_movements() == ("NS",)
    assert r.tl() == [(0, "NS", "RED_YELLOW"), (1000, "NS", "GREEN")]


def test_faulted_movement_ignores_gap_out_and_maxes_out():
    r = Rig(on("ES", "D1", 5000, 5300), faults=[(("NS", "D1"), 0, None)]).run(20000)
    assert r.sb.gap_out("NS") and r.bank.state("NS") is GREEN      # gapped out, still served
    r.run(80000)
    assert r.ctrl.decision_log[0].stopped == (("NS", "max"),)
    assert r.t_of("NS", FLASHING_GREEN) == 5000 + 42000


def test_same_scenario_without_fault_has_no_ns_demand():
    r = Rig(on("ES", "D1", 5000, 5300)).run(80000)
    assert r.t_of("NS", RED_YELLOW) is None and r.t_of("ES", GREEN) == 6000


def test_fault_call_timing_after_service_end():
    # NS on recall; after its max-out, its synthetic call restarts at service end (FG -> YELLOW)
    r = Rig(on("ES", "D1", 5000, 5300), faults=[(("NS", "D1"), 0, None)]).run(80000)
    service_end = r.t_of("NS", YELLOW)
    assert service_end == 50000
    assert r.ctrl._call_ms("NS", {m: r.bank.state(m) for m in MOVES}) == service_end
    # NS is served again after ES (recall), without any detector activity
    assert r.t_of("NS", GREEN, nth=1) is not None


def test_fault_uses_qhat_2_but_keeps_real_level2():
    r = Rig(faults=[(("ES", "D2"), 0, None)])
    assert r.ctrl._qhat("ES") == 2
    r2 = Rig(on("NS", "D1", 100) + on("SL", "D2", 200, 5000), faults=[(("SL", "D2"), 3000, None)])
    r2.run(3000)
    assert r2.sb.demand_level("SL") is DemandLevel.LEVEL_2 and r2.ctrl._qhat("SL") == 6


def test_fault_clearing_restores_normal_behaviour():
    r = Rig(on("ES", "D1", 20000, 20300), faults=[(("NS", "D1"), 0, 10000)]).run(60000)
    assert r.ctrl.faulted_movements() == ()
    # NS served on recall from 0; fault gone at 10 s -> gap-out counts again -> released for ES
    assert r.ctrl.decision_log[0].stopped == (("NS", "gap"),)
    assert r.ctrl.decision_log[0].time_ms == 20000


def test_fault_only_branches_on_fault_not_on_detector_kind():
    src = inspect.getsource(cs)
    tree = ast.parse(src)
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | \
            {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    for bad in ("real", "virtual", "hardware", "esp", "Hybrid", "HybridDetectorSource",
                "HardwareDetectorSource", "VirtualDetectors", "occupied", "d1", "d2", "_src"):
        assert bad not in names, bad


# ================================================================ imports / isolation
def test_controller_imports_only_conflicts_and_signals():
    tree = ast.parse(inspect.getsource(cs))
    mods = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            mods |= {a.name for a in n.names}
        elif isinstance(n, ast.ImportFrom):
            mods.add(n.module)
    project = {m for m in mods if m.startswith("smarttraffic")}
    assert project == {"smarttraffic.conflicts", "smarttraffic.signals"}
    assert not mods & {"serial", "pygame", "time", "random", "numpy", "datetime"}


def test_controller_uses_only_public_sensorbank_queries():
    src = inspect.getsource(cs)
    used = set()
    for n in ast.walk(ast.parse(src)):
        if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Attribute) and \
                n.value.attr == "sensors":
            used.add(n.attr)
    assert used == {"now_ms", "has_call", "call_since_ms", "qhat", "gap_out",
                    "extended_initial_required", "faulted_detectors"}


def test_fixed_controller_and_firmware_untouched():
    # SHA-256 of the Stage 10 checkpoint versions (commit 7e4ae52)
    want = {"smarttraffic/controller_fixed.py": "aefddb0b55492cd8b446fb3eb9de8aa4cb46499a528b82ed2f54d26e348b4714",
            "hardware/esp32_smart_traffic/esp32_smart_traffic.ino":
                "81c6f85e8620d2425fad4d1412c99fc2c583e2aeadcd73551974dc619d3e8b64"}
    for f, h in want.items():
        data = (ROOT / f).read_bytes().replace(b"\r\n", b"\n")
        assert hashlib.sha256(data).hexdigest() == h, f


# ================================================================ full stack
def _full(scn="S1", seed=1, seconds=600, params=SmartParams(), source=None, dt_ms=100, each=None):
    from smarttraffic.traffic_gen import generate_arrivals
    from smarttraffic.vehicles import VehicleModel
    vm = VehicleModel(list(generate_arrivals(scn, seed, 0, seconds).arrivals), start=0.0)
    src = source(vm) if source else VirtualDetectors(vm)
    sb = SensorBank(src)
    bank = SignalBank()
    c = SmartController(sb, params)
    c.update(bank)
    rng = dt_ms if callable(dt_ms) else (lambda: dt_ms)
    while sb.now_ms < seconds * 1000:
        step = min(rng(), seconds * 1000 - sb.now_ms)
        vm.advance(step / 1000, bank)
        sb.advance(step / 1000, bank)
        bank.advance(step / 1000)
        c.update(bank)
        conflicts.check_invariant(bank.active())
        if each:
            each(vm, sb, bank, c)
    return vm, sb, bank, c


@pytest.mark.parametrize("scn", ["S1", "S7b", "S9", "S5"])
def test_long_runs_are_safe(scn):
    seen_green = set()

    def check(vm, sb, bank, c):
        act = bank.active()
        assert conflicts.is_allowed_set(act)
        serving = [m for m in MOVES if bank.state(m) in (RED_YELLOW, GREEN)]
        assert conflicts.is_allowed_set(serving)
        seen_green.update(m for m in MOVES if bank.state(m) is GREEN)

    vm, sb, bank, c = _full(scn, 1, 900, each=check)
    assert seen_green == set(MOVES)
    # every command was a legal SignalBank command (it would have raised otherwise)
    assert {cmd.kind for cmd in c.command_log} == {START_GREEN, END_GREEN}


def _service_lengths(bank):
    out, onset = [], {}
    for e in bank.log:
        t = round(e.time * 1000)
        if e.new is GREEN:
            onset[e.movement] = t
        elif e.new is YELLOW and e.movement in onset:
            out.append((e.movement, t - onset.pop(e.movement)))
    return out


def test_minimum_green_holds_in_full_runs():
    vm, sb, bank, c = _full("S7a", 2, 900)
    lens = _service_lengths(bank)
    assert lens
    for m, g in lens:
        kind = sg.movement_type(m)
        assert g >= (7000 if kind == "straight" else 5000)
    for d in c.decision_log:
        for m, why in d.stopped:
            assert why in ("gap", "max", "hold")


def test_irregular_steps_are_safe_and_respect_minimum_green():
    rnd = random.Random(11)
    vm, sb, bank, c = _full("S1", 3, 900, dt_ms=lambda: rnd.choice([1, 7, 100, 250, 1000, 2600]))
    for m, g in _service_lengths(bank):
        assert g >= (7000 if sg.movement_type(m) == "straight" else 5000)
    assert len(c.command_log) > 20


def test_deterministic_replay():
    a = _full("S9", 4, 600)[3]
    b = _full("S9", 4, 600)[3]
    assert a.command_log == b.command_log and a.decision_log == b.decision_log


def test_deterministic_across_hash_seeds():
    code = ("import sys; sys.path.insert(0, %r); from tests.test_controller_smart import _full; "
            "c = _full('S1', 5, 300)[3]; import hashlib; "
            "print(hashlib.sha256(repr(c.command_log).encode()).hexdigest())") % str(ROOT)
    outs = set()
    for seed in ("0", "1", "12345"):
        env = dict(os.environ, PYTHONHASHSEED=seed)
        outs.add(subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True,
                                text=True, check=True).stdout.strip())
    assert len(outs) == 1


def test_starvation_scenario_s10_all_calls_served():
    waits = []

    def check(vm, sb, bank, c):
        for m in MOVES:
            if bank.state(m) is RED and sb.has_call(m):
                waits.append(sb.now_ms - sb.call_since_ms(m))

    vm, sb, bank, c = _full("S10", 1, 900, each=check)
    wl = [v for v in vm.crossed("WL")]
    assert len(wl) >= 4                                   # the deterministic WL cars are served
    assert max(waits) < 200000                            # measured maxima are documented (~150 s)


# ================================================================ virtual vs hybrid NS:D1
class Replay:
    """A real-style source for one detector: replays given edges (as HardwareDetectorSource would)."""
    def __init__(self, sid, edges, start_ms=0):
        self.sensor_ids = (sid,)
        self.edges = list(edges)
        self._now = start_ms

    @property
    def now_ms(self):
        return self._now

    def initial_occupancy(self):
        return {self.sensor_ids[0]: False}

    def poll(self, until):
        out = [e for e in self.edges if self._now < e.time_ms <= until]
        self._now = until
        return out

    def faults(self):
        return ()


def test_virtual_and_hybrid_real_style_ns_d1_give_identical_decisions():
    from smarttraffic.hybrid_sensors import HybridDetectorSource
    vm0, sb0, bank0, c0 = _full("S2", 1, 900)
    ns_d1 = [e for e in sb0.edge_log if (e.movement, e.detector) == ("NS", "D1")]
    assert len(ns_d1) > 50
    holder = {}

    def hybrid(vm):
        h = HybridDetectorSource.with_overrides(VirtualDetectors(vm), [(("NS", "D1"), Replay(("NS", "D1"), ns_d1))])
        holder["h"] = h
        return h

    vm1, sb1, bank1, c1 = _full("S2", 1, 900, source=hybrid)
    assert c1.command_log == c0.command_log
    assert c1.decision_log == c0.decision_log
    assert [(e.time, e.movement, e.new) for e in bank1.log] == [(e.time, e.movement, e.new) for e in bank0.log]
    h = holder["h"]
    assert len(h.sensor_ids) == 16 == len(set(h.sensor_ids)) == len(SENSOR_IDS)
    assert h.dropped_overridden > 0                      # the virtual NS:D1 copy was dropped
    assert h.source_of(("NS", "D1")) is not h.source_of(("NS", "D2"))


class _Serial:
    """Minimal fake serial transport speaking the Stage 9 protocol."""
    def __init__(self):
        self.is_open, self.inbox, self.sent = False, bytearray(), []

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def read_available(self):
        data, self.inbox = bytes(self.inbox), bytearray()
        return data

    def write(self, data):
        self.sent.append(data)


def test_real_esp32_sensor_report_is_served_like_any_call():
    from smarttraffic import esp_protocol as p
    from smarttraffic.hardware import build_hardware, load_hardware_config
    from smarttraffic.vehicles import VehicleModel
    cfg = load_hardware_config()
    ports = {}

    def factory(dev):
        ports[dev.device_id] = _Serial()
        return ports[dev.device_id]

    vm = VehicleModel([], start=0.0)
    hw = build_hardware(cfg, vm, factory)
    sb = SensorBank(hw.detectors)
    bank = SignalBank()
    c = SmartController(sb)
    assert len(hw.detectors.sensor_ids) == 16
    seq = iter(range(10 ** 6))
    t = 0
    c.update(bank)
    while t < 20000:
        if t == 100:
            ports["esp32_a"].inbox += p.encode_hello(next(seq), t + 5000, "esp32_a", "0000BEEF", 1, 1)
            ports["esp32_a"].inbox += p.encode_sensor(next(seq), t + 5000, 0, False)
        elif t > 100 and t % 200 == 0:
            ports["esp32_a"].inbox += p.encode_heartbeat(next(seq), t + 5000)
        if t == 3000:
            ports["esp32_a"].inbox += p.encode_sensor(next(seq), t + 5000, 0, True)
        if t == 3500:
            ports["esp32_a"].inbox += p.encode_sensor(next(seq), t + 5000, 0, False)
        t += 100
        vm.advance(0.1, bank)
        sb.advance(0.1, bank)
        bank.advance(0.1)
        c.update(bank)
    starts = [cmd for cmd in c.command_log if cmd.kind == START_GREEN]
    # before the ESP32 reported NS:D1 it was faulted -> recall; afterwards the real call counts
    assert starts[0].movement == "NS"
    assert bank.state("NS") in (GREEN, RED_YELLOW, FLASHING_GREEN)
    hw.close()


def test_disconnected_real_detector_puts_its_movement_on_recall():
    from smarttraffic.hardware import build_hardware, load_hardware_config
    from smarttraffic.vehicles import VehicleModel

    class Dead(_Serial):
        def open(self):
            raise OSError("no such port")

    vm = VehicleModel([], start=0.0)
    hw = build_hardware(load_hardware_config(), vm, lambda d: Dead())
    sb = SensorBank(hw.detectors)
    bank, c = SignalBank(), SmartController(sb)
    c.update(bank)
    for _ in range(50):
        vm.advance(0.1, bank)
        sb.advance(0.1, bank)
        bank.advance(0.1)
        c.update(bank)
    assert sb.faulted_detectors() == (("NS", "D1"),)
    assert c.faulted_movements() == ("NS",)
    assert bank.state("NS") is GREEN                      # recall: served without any detection
