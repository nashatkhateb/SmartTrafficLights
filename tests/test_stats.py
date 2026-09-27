"""Tests for smarttraffic.stats (Stage 12): exact metric definitions on controlled inputs.

A stub world (vehicles / signals / sensors with fixed values) drives the StatsCollector so
every boundary can be set to the millisecond, independently of the traffic dynamics.
"""
import ast
import inspect
import json
import math
import random
from types import SimpleNamespace

import numpy as np
import pytest
from scipy import stats as st

from smarttraffic import stats as S
from smarttraffic.conflicts import MOVES
from smarttraffic.signals import SignalState, TransitionEvent
from smarttraffic.stats import (DELAY_OVER_90S_MS, SAMPLE_MS, StatsCollector, Window, mean_ci95,
                                paired_comparison, percentile_linear)

FREE = 10_800


# ================================================================ stub world
class V:
    def __init__(self, vid, m, sched, cross=None, entry="same", stopped_s=0.0, source="random"):
        self.id, self.movement, self.source = vid, m, source
        self.scheduled_ms = sched
        self.entry_ms = sched if entry == "same" else entry
        self.cross_ms = cross
        self.exit_ms = None if cross is None else cross + 2000
        self.stopped_time = stopped_s


class World:
    def __init__(self, vehicles=(), window=Window(0, 10_000, 5_000), log=(), sensors=None, setter=None):
        self.now_ms = -window.warmup_ms
        self.setter = setter
        self.vehicles = list(vehicles)
        self.queue = {m: 0 for m in MOVES}
        self.outside = {m: 0 for m in MOVES}
        self.log = list(log)
        self.window = window
        self.sensors = sensors if sensors is not None else SimpleNamespace()
        self.c = StatsCollector(self, self.sensors, self, SimpleNamespace(), window)
        if setter:
            setter(self)
        self.c.observe()                       # the runner observes the start instant too

    # vehicle-model API used by the collector
    def queue_length(self, m):
        return self.queue[m]

    def waiting_outside(self, m):
        return [None] * self.outside[m]

    def all_vehicles(self):
        return tuple(self.vehicles)

    # signal API used by the collector
    def active(self):
        return ()

    def state(self, m):
        return SignalState.RED

    def run(self, until, step=500):
        setter = self.setter
        while self.now_ms < until:
            self.now_ms = min(self.now_ms + step, until)
            if setter:
                setter(self)
            self.c.observe()
        return self

    def result(self, end_ms=None, early=False):
        return self.c.finalize(self.now_ms if end_ms is None else end_ms, {"scenario": "T", "seed": 1,
                                                                          "controller": "X"}, early)


def ev(t_s, m, new):
    return TransitionEvent(t_s, m, SignalState.RED, SignalState[new])


# ============================================================ sets / windows
@pytest.mark.parametrize("sched, measured", [(-1, False), (0, True), (9_999, True), (10_000, False)])
def test_measured_set_boundaries(sched, measured):
    r = World([V(0, "NS", sched, cross=sched + 20_000)]).run(10_000).result()
    assert r.vehicles[0].measured is measured
    assert r.intersection.arrivals == (1 if measured else 0)


@pytest.mark.parametrize("cross, counted", [(-1, False), (0, True), (9_999, True), (10_000, False)])
def test_throughput_window_boundaries(cross, counted):
    r = World([V(0, "NS", -20_000, cross=cross)], window=Window(20_000, 10_000, 5_000)).run(10_000).result()
    assert r.movement("NS").throughput_veh_h == (360.0 if counted else 0.0)    # 1 per 10 s -> veh/h
    assert r.intersection.arrivals == 0                                          # warm-up car not in V


def test_measured_vehicle_crossing_in_flush_counts_delay_not_throughput():
    r = World([V(0, "NS", 1_000, cross=12_000)]).run(12_000).result()
    ms = r.movement("NS")
    assert ms.crossed == 1 and ms.delay_mean_s == (12_000 - 1_000 - FREE) / 1000
    assert ms.throughput_veh_h == 0.0 and ms.served_in_hour == 0 and ms.pct_served == 0.0


def test_served_in_hour_and_pct():
    vs = [V(0, "NS", 0, cross=9_999), V(1, "NS", 100, cross=10_000), V(2, "SS", 200, cross=None)]
    i = World(vs).run(15_000).result().intersection
    assert (i.arrivals, i.crossed, i.unfinished, i.served_in_hour) == (3, 2, 1, 1)
    assert i.pct_served == pytest.approx(100 / 3)


# ================================================================ delay
def test_control_delay_definition_and_units():
    r = World([V(0, "WL", 2_000, cross=2_000 + FREE + 7_250, entry=2_500)]).run(20_000).result()
    v = r.vehicles[0]
    assert v.control_delay_ms == 7_250                    # from SCHEDULED arrival, not entry
    assert r.intersection.delay_mean_s == 7.25
    assert r.intersection.delay_total_veh_h == 7_250 / 3_600_000


def test_delay_mean_p95_max_match_numpy():
    rnd = random.Random(3)
    vs = [V(i, MOVES[i % 8], rnd.randrange(0, 9_000), None) for i in range(200)]
    for v in vs:
        v.cross_ms = v.scheduled_ms + FREE + rnd.randrange(0, 200_000)
    r = World(vs).run(10_000).result(end_ms=300_000)
    d = np.array([v.cross_ms - v.scheduled_ms - FREE for v in vs])
    i = r.intersection
    assert i.delay_mean_s == pytest.approx(d.mean() / 1000)
    assert i.delay_p95_s == pytest.approx(np.percentile(d, 95) / 1000)
    assert i.delay_max_s == d.max() / 1000
    for m in MOVES:
        dm = np.array([v.cross_ms - v.scheduled_ms - FREE for v in vs if v.movement == m])
        assert r.movement(m).delay_p95_s == pytest.approx(np.percentile(dm, 95) / 1000)


def test_percentile_linear_edge_cases():
    assert percentile_linear([], 95) is None
    assert percentile_linear([7], 95) == 7
    assert percentile_linear([0, 100], 95) == 95
    for n in range(1, 40):
        xs = [random.Random(n).random() for _ in range(n)]
        assert percentile_linear(xs, 95) == pytest.approx(np.percentile(xs, 95))
    with pytest.raises(ValueError):
        percentile_linear([1], 101)


@pytest.mark.parametrize("delay, counted", [(DELAY_OVER_90S_MS, 0), (DELAY_OVER_90S_MS + 1, 1)])
def test_vehicles_control_delay_over_90s_threshold(delay, counted):
    r = World([V(0, "NS", 0, cross=FREE + delay)]).run(10_000).result(end_ms=200_000)
    assert r.intersection.vehicles_control_delay_over_90s == counted


def test_unfinished_vehicle_excluded_but_counted_and_in_lower_bound():
    vs = [V(0, "NS", 1_000, cross=1_000 + FREE + 4_000), V(1, "NS", 2_000, cross=None, entry=None)]
    r = World(vs).run(15_000).result(end_ms=15_000)
    ms = r.movement("NS")
    assert (ms.arrivals, ms.crossed, ms.unfinished) == (2, 1, 1)
    assert ms.delay_mean_s == 4.0 and ms.delay_max_s == 4.0
    censored = 15_000 - (2_000 + FREE)
    assert ms.delay_mean_lower_bound_s == pytest.approx((4_000 + censored) / 2 / 1000)
    assert r.vehicles[1].control_delay_ms is None
    assert r.vehicles[1].waited_outside_ms == 13_000        # still held outside at the end


def test_waiting_time_is_stopped_on_road_plus_outside():
    vs = [V(0, "EL", 1_000, cross=40_000, entry=3_500, stopped_s=12.3456)]
    r = World(vs).run(10_000).result(end_ms=50_000)
    v = r.vehicles[0]
    assert (v.stopped_on_road_ms, v.waited_outside_ms, v.waiting_ms) == (12_346, 2_500, 14_846)
    assert r.intersection.waiting_mean_s == 14.846


# ================================================================ queue
def _q(world):
    t = world.now_ms
    world.queue["NS"] = 5 if t % 1000 else 1          # half-second states are never sampled
    world.outside["NS"] = 2 if t % 1000 == 0 and t >= 5_000 else 0
    world.queue["EL"] = t // 1000                       # 0 .. 9 at the sample instants
    if t >= 10_000:
        world.queue["NS"], world.queue["EL"] = 99, 99  # flush: never sampled


def test_queue_uses_exact_1s_samples_only():
    w = World(setter=_q).run(12_000, step=500)
    r = w.result()
    ts = [t for t, _, _ in r.queue_samples]
    assert ts == list(range(0, 10_000, 1000))          # t = 0 exists, last is measure_end - 1000
    ns = r.movement("NS")
    assert ns.queue_road_mean_veh == 1.0                # the 5s at .5 s instants are ignored
    assert ns.queue_outside_mean_veh == 1.0             # 2 at t = 5..9 s -> 10 / 10
    assert ns.queue_mean_veh == 2.0 and ns.queue_max_veh == 3
    el = r.movement("EL")
    assert el.queue_mean_veh == 4.5 and el.queue_max_veh == 9
    i = r.intersection
    assert i.queue_mean_per_movement_veh == pytest.approx((2.0 + 4.5) / 8)
    assert i.queue_max_movement_veh == 9
    assert i.queue_total_max_veh == max(1 + (2 if t >= 5 else 0) + t for t in range(10))
    assert i.queue_total_mean_veh == pytest.approx(sum(1 + (2 if t >= 5 else 0) + t for t in range(10)) / 10)


def test_flush_never_changes_queue_statistics():
    a = World(setter=_q).run(10_000).result()
    b = World(setter=_q).run(15_000).result()
    assert [(m.queue_mean_veh, m.queue_max_veh) for m in a.movements] == \
           [(m.queue_mean_veh, m.queue_max_veh) for m in b.movements]


def test_skipped_sample_instant_is_an_error():
    w = World()
    w.now_ms = 1_500
    with pytest.raises(ValueError, match="skipped"):
        w.c.observe()


def test_finalize_requires_all_samples():
    w = World().run(5_000)
    with pytest.raises(ValueError, match="queue samples"):
        w.result()


# ================================================================ totals
def test_movement_totals_sum_to_intersection():
    rnd = random.Random(9)
    vs = []
    for i in range(300):
        m = MOVES[rnd.randrange(8)]
        s = rnd.randrange(-5_000, 10_000)
        c = None if rnd.random() < 0.1 else s + FREE + rnd.randrange(0, 50_000)
        vs.append(V(i, m, s, cross=c))
    r = World(vs, window=Window(5_000, 10_000, 5_000)).run(10_000).result(end_ms=80_000)
    i = r.intersection
    for f in ("arrivals", "crossed", "unfinished", "served_in_hour", "vehicles_control_delay_over_90s"):
        assert sum(getattr(ms, f) for ms in r.movements) == getattr(i, f), f
    assert sum(ms.throughput_veh_h for ms in r.movements) == pytest.approx(i.throughput_veh_h)
    assert sum(ms.delay_total_veh_h for ms in r.movements) == pytest.approx(i.delay_total_veh_h)
    weighted = sum(ms.delay_mean_s * ms.crossed for ms in r.movements if ms.crossed) / i.crossed
    assert i.delay_mean_s == pytest.approx(weighted)


def test_movement_attribution():
    r = World([V(0, "SL", 0, cross=FREE + 1_000), V(1, "WS", 0, cross=FREE + 3_000)]).run(10_000).result()
    assert r.movement("SL").delay_mean_s == 1.0 and r.movement("WS").delay_mean_s == 3.0
    assert all(r.movement(m).arrivals == 0 for m in MOVES if m not in ("SL", "WS"))


# ================================================================ signals
def test_green_seconds_onsets_utilization_and_changes():
    log = [ev(-2.0, "NS", "RED_YELLOW"), ev(-1.0, "NS", "GREEN"), ev(3.0, "NS", "FLASHING_GREEN"),
           ev(6.0, "NS", "YELLOW"), ev(8.0, "NS", "RED_YELLOW"), ev(9.0, "NS", "GREEN")]
    vs = [V(i, "NS", -20_000, cross=c) for i, c in enumerate((0, 2_500, 6_900, 9_500, 11_000))]
    r = World(vs, window=Window(20_000, 10_000, 5_000), log=log).run(12_000).result()
    ns = r.movement("NS")
    # service [-1.0, 6.0) and [9.0, end) clipped to [0, 10): 6.0 + 1.0 s
    assert ns.green_s == 7.0
    assert ns.green_onsets == 1                         # the -1.0 s onset is before the window
    # n = crossings in [0, 10 s) = 4 (6.9 s is during YELLOW: counted), h = 1.9 s
    assert ns.green_utilization == pytest.approx(4 * 1900 / 7000)
    assert r.intersection.signal_changes_per_hour == 1 * 3_600_000 / 10_000
    assert r.movement("SS").green_utilization is None   # no green at all


def test_jain_index_cases():
    empty = World().run(10_000).result().intersection
    assert empty.jain_fairness is None and empty.arrivals == 0 and empty.delay_mean_s is None
    zero = World([V(0, "NS", 0, cross=FREE), V(1, "EL", 0, cross=FREE)]).run(10_000).result()
    assert zero.intersection.jain_fairness == 1.0
    two = World([V(0, "NS", 0, cross=FREE + 10_000), V(1, "EL", 0, cross=FREE + 20_000),
                 V(2, "WL", 0, cross=None)]).run(10_000).result(end_ms=40_000)
    assert two.intersection.jain_fairness == pytest.approx(30 ** 2 / (2 * (10 ** 2 + 20 ** 2)))


def test_zero_vehicle_result_is_well_defined():
    i = World().run(10_000).result().intersection
    assert (i.arrivals, i.crossed, i.throughput_veh_h, i.delay_total_veh_h) == (0, 0, 0.0, 0.0)
    assert i.pct_served is None and i.waiting_mean_s is None and i.delay_mean_lower_bound_s is None
    assert i.queue_mean_per_movement_veh == 0.0 and i.safety_violations == 0


# ================================================================ diagnostics
def test_missing_diagnostics_do_not_invalidate_result():
    r = World([V(0, "NS", 0, cross=FREE + 5_000)]).run(10_000).result()
    d = r.diagnostics
    assert d["detector_call_wait_max_s"] is None and d["qhat_saturation_s"] is None
    assert d["faulted_s"] is None and d["controller_terminations"] is None
    assert r.intersection.delay_mean_s == 5.0


def test_diagnostics_are_separate_from_official_metrics():
    fields = {f for cls in (S.MovementStats, S.IntersectionStats) for f in cls.__dataclass_fields__}
    for bad in ("detector", "call", "qhat", "fault", "hold", "gap", "max_out", "wait_max"):
        assert not any(bad in f for f in fields), bad


def test_detector_call_wait_diagnostic():
    sensors = SimpleNamespace(has_call=lambda m: m == "EL", call_since_ms=lambda m: 1_000,
                              level2_latched=lambda m: False, faulted_detectors=lambda: (("NS", "D1"),))
    w = World(window=Window(0, 200_000, 0), sensors=sensors).run(200_000, step=1000)
    d = w.result().diagnostics
    assert d["detector_call_wait_max_s"]["EL"] == 198.0      # observed up to 199 s
    assert d["detector_calls_over_120s"]["EL"] == 1 and d["detector_calls_over_120s"]["NS"] == 0
    assert d["faulted_s"]["NS"] == 200 and d["green_while_faulted_s"]["NS"] == 0


# ================================================================ exports
def test_exports_are_deterministic_and_complete():
    vs = [V(i, MOVES[i % 8], i * 100, cross=i * 100 + FREE + i) for i in range(20)]
    a = World(vs, setter=_q).run(10_000).result()
    b = World(vs, setter=_q).run(10_000).result()
    assert a.to_json() == b.to_json()
    d = json.loads(a.to_json())
    assert d["intersection"]["arrivals"] == 20 and len(d["movements"]) == 8
    assert len(a.movement_rows()) == 8 and len(a.vehicle_rows()) == 20
    assert len(a.queue_rows()) == 8 * 10
    row = a.summary_row()
    assert row["cfg_scenario"] == "T" and row["arrivals"] == 20 and "delay_p95_s" in row
    assert list(a.movement_rows()[0])[:3] == ["scenario", "seed", "controller"]


# ================================================================ across seeds
def test_mean_ci95_matches_scipy():
    xs = [3.1, 4.7, 5.0, 2.2, 9.9, 4.4]
    r = mean_ci95(xs)
    lo, hi = st.t.interval(0.95, len(xs) - 1, loc=np.mean(xs), scale=st.sem(xs))
    assert r["mean"] == pytest.approx(np.mean(xs)) and r["low"] == pytest.approx(lo) and r["high"] == pytest.approx(hi)


def test_mean_ci95_small_and_degenerate():
    with pytest.raises(ValueError):
        mean_ci95([])
    one = mean_ci95([5.0])
    assert one["n"] == 1 and one["half_width"] is None and "n = 1" in one["note"]
    flat = mean_ci95([2.0, 2.0, 2.0])
    assert flat["half_width"] == 0.0 and flat["note"] == "zero variance"


def test_paired_comparison_matches_scipy_ttest_rel():
    fixed = [40.1, 38.2, 45.0, 39.9, 41.3, 44.4, 37.7, 42.0, 40.5, 43.3]
    smart = [36.0, 35.1, 44.0, 33.2, 40.0, 39.9, 36.1, 41.8, 35.5, 40.2]
    r = paired_comparison(fixed, smart)
    ref = st.ttest_rel(smart, fixed)
    assert r["test_valid"] and r["t"] == pytest.approx(ref.statistic) and r["p"] == pytest.approx(ref.pvalue)
    assert r["mean_diff"] == pytest.approx(np.mean(np.array(smart) - np.array(fixed)))


def test_paired_comparison_degenerate_cases():
    with pytest.raises(ValueError):
        paired_comparison([1, 2], [1])
    one = paired_comparison([1.0], [2.0])
    assert one["test_valid"] is False and one["t"] is None and one["mean_diff"] == 1.0
    same = paired_comparison([1.0, 2.0, 3.0], [2.0, 3.0, 4.0])
    assert same["test_valid"] is False and same["half_width"] == 0.0 and "zero variance" in same["note"]


# ================================================================ observational only
MUTATING = {"advance", "update", "start_green", "end_green", "poll", "start_service", "start_waiting",
            "edge", "set_permission", "watch_positions", "check_invariants", "close", "pump", "send"}


def test_stats_module_never_calls_state_changing_methods():
    tree = ast.parse(inspect.getsource(S))
    calls = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert not calls & MUTATING, calls & MUTATING
    imported = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    assert not {m for m in imported if m} & {"smarttraffic.controller_smart", "smarttraffic.controller_fixed",
                                             "smarttraffic.hardware", "smarttraffic.traffic_gen"}
