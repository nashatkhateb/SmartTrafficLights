"""Tests for smarttraffic.runner (Stage 12): the shared run, measured with the real models.

Short windows (10 s warm-up, 60 s measurement, 60 s flush) with scripted arrivals give
exactly known timings under FT-A (first cycle at -10 s: NS+SS RED_YELLOW 5.0, GREEN 6.0,
FLASHING_GREEN 28.0, YELLOW 31.0; next NS GREEN 99.0 s).
"""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from smarttraffic import runner as R
from smarttraffic.conflicts import MOVES
from smarttraffic.controller_smart import SmartParams
from smarttraffic.runner import CONTROLLERS, RunConfig, StepView, arrivals_sha256, run, run_pair, simulate
from smarttraffic.stats import Window
from smarttraffic.traffic_gen import Arrival, generate_arrivals

ROOT = Path(__file__).resolve().parent.parent
SHORT = dict(warmup_s=10, measure_s=60, flush_s=60)


def arrivals(spec):
    seq, out = {}, []
    for i, (m, t) in enumerate(sorted(spec, key=lambda x: (x[1], MOVES.index(x[0])))):
        out.append(Arrival(i, t, m, seq.get(m, 0), "random"))
        seq[m] = seq.get(m, 0) + 1
    return out


def short(controller="FT-A", **kw):
    return RunConfig("S1", 1, controller, **{**SHORT, **kw})


@pytest.fixture(scope="module")
def s1_pair():
    arr = generate_arrivals("S1", 3, -300, 3600)
    return arr, run_pair(RunConfig("S1", 3, "FT-A", measure_s=900, flush_s=900), ("FT-A", "Smart"),
                         arrivals=generate_arrivals("S1", 3, -300, 900))


# ================================================================ config
def test_config_validation():
    with pytest.raises(ValueError):
        RunConfig("S1", 1, "Clever")
    with pytest.raises(ValueError):
        RunConfig("S1", 1, "FT-A", dt_ms=0)
    with pytest.raises(ValueError):
        RunConfig("S1", 1, "FT-A", measure_s=10.5)
    c = RunConfig("S1", 1, "Smart")
    assert (c.dt_ms, c.window()) == (100, Window(300_000, 3_600_000, 900_000))
    assert c.with_controller("FT-B").controller == "FT-B" and set(CONTROLLERS) == {"FT-A", "FT-B", "FT-TOD", "Smart"}


def test_arrivals_outside_window_rejected():
    with pytest.raises(ValueError, match="outside"):
        run(short(), arrivals=arrivals([("NS", 60_000)]))          # arrival at measure end
    with pytest.raises(ValueError, match="outside"):
        run(short(), arrivals=arrivals([("NS", -10_001)]))


# ================================================================ known timings
def test_zero_vehicles():
    r = run(short(), arrivals=[])
    i = r.intersection
    assert (i.arrivals, i.crossed, i.throughput_veh_h, i.queue_total_max_veh) == (0, 0, 0.0, 0)
    assert i.jain_fairness is None and i.safety_violations == 0
    assert r.end_ms == 60_000 and r.ended_early is True
    assert [t for t, _, _ in r.queue_samples] == list(range(0, 60_000, 1000))


def test_one_vehicle_on_green_has_zero_delay():
    r = run(short(), arrivals=arrivals([("NS", 10_000)]))
    v = r.vehicles[0]
    assert (v.cross_ms, v.control_delay_ms, v.stopped_on_road_ms, v.waited_outside_ms) == (20_800, 0, 0, 0)
    assert r.end_ms == 60_000 and r.ended_early


def test_one_vehicle_on_red_known_delay_stops_and_queue():
    r = run(short(), arrivals=arrivals([("NS", 30_000)]))
    v = r.vehicles[0]
    # line at 40.8 s after YELLOW (31.0 + 2.0 s window ended); next NS GREEN 99.0 -> 99.0 + 3.9
    assert v.cross_ms == 102_900 and v.control_delay_ms == 62_100
    assert v.stopped_on_road_ms == 62_100
    q = {t: road[0] for t, road, _ in r.queue_samples}
    assert [t for t in q if q[t]] == list(range(41_000, 60_000, 1000))      # stopped from 40.764 s
    ms = r.movement("NS")
    assert ms.queue_mean_veh == 19 / 60 and ms.queue_max_veh == 1
    assert ms.throughput_veh_h == 0.0 and ms.crossed == 1 and ms.served_in_hour == 0
    assert r.end_ms == 102_900 and r.ended_early


def test_warmup_vehicle_affects_queue_and_throughput_but_not_measured_set():
    r = run(short(), arrivals=arrivals([("NS", -5_000)]))
    v = r.vehicles[0]
    assert not v.measured and v.cross_ms == 9_900            # waits for GREEN at 6.0 s
    assert r.intersection.arrivals == 0 and r.intersection.delay_mean_s is None
    assert r.movement("NS").throughput_veh_h == 60.0         # 1 crossing in 60 s
    assert [t for t, road, _ in r.queue_samples if road[0]] == [6_000, 7_000, 8_000, 9_000]


def test_measured_vehicle_in_flush_contributes_delay_not_throughput():
    r = run(short(), arrivals=arrivals([("NS", 10_000), ("NS", 30_000), ("NS", -5_000)]))
    ns = r.movement("NS")
    assert (ns.arrivals, ns.crossed, ns.served_in_hour) == (2, 2, 1)
    assert ns.delay_mean_s == (0 + 62_100) / 2 / 1000
    assert ns.throughput_veh_h == 2 * 60.0                   # warm-up car + 20.8 s crossing only
    assert ns.delay_p95_s == pytest.approx(0.95 * 62.1)
    assert ns.green_s == 25.0 and ns.green_utilization == pytest.approx(2 * 1.9 / 25.0)


def test_unfinished_vehicle_at_hard_end():
    r = run(short(flush_s=20), arrivals=arrivals([("NS", 30_000)]))
    assert r.end_ms == 80_000 and r.ended_early is False
    i = r.intersection
    assert (i.arrivals, i.crossed, i.unfinished) == (1, 0, 1)
    assert i.delay_mean_s is None
    assert i.delay_mean_lower_bound_s == (80_000 - (30_000 + 10_800)) / 1000
    assert r.vehicles[0].control_delay_ms is None


def test_ten_car_queue_discharges_like_the_study():
    spec = [("SS", 32_000 + 2_000 * k) for k in range(10)]
    r = run(short(), arrivals=arrivals(spec))
    crosses = [v.cross_ms for v in r.vehicles]
    assert crosses == [99_000 + 2_000 + 1_900 * n for n in range(1, 11)]      # last at 99 + 21.0 s
    assert [v.control_delay_ms for v in r.vehicles] == \
        [c - (s + 10_800) for c, (_, s) in zip(crosses, spec)]
    ss = r.movement("SS")
    assert ss.queue_max_veh == 10                                              # queue built
    assert r.queue_samples[-1][1][MOVES.index("SS")] == 10


def test_queue_outside_entry_is_part_of_the_queue():
    spec = [("WS", 32_000 + 300 * k) for k in range(40)]                      # faster than the road admits
    r = run(short(), arrivals=arrivals(spec))
    ws = r.movement("WS")
    assert ws.queue_outside_mean_veh > 0
    assert ws.queue_mean_veh == pytest.approx(ws.queue_road_mean_veh + ws.queue_outside_mean_veh)
    assert any(v.waited_outside_ms > 0 for v in r.vehicles)


def test_irregular_step_sizes_keep_exact_sampling_and_fixed_results():
    arr = generate_arrivals("S1", 2, -10, 60)
    a = run(short(), arrivals=arr)
    b = run(short(dt_ms=270), arrivals=arr)
    c = run(short(dt_ms=1_700), arrivals=arr)
    for x in (b, c):
        assert [t for t, _, _ in x.queue_samples] == list(range(0, 60_000, 1000))
        # vehicles, sensors and the fixed controller are exact at any step size
        assert x.intersection == a.intersection and x.movements == a.movements
        assert x.vehicles == a.vehicles and x.signal_log == a.signal_log


def test_smart_run_with_irregular_steps_is_safe():
    arr = generate_arrivals("S1", 2, -10, 60)
    r = run(short("Smart", dt_ms=333), arrivals=arr)
    assert r.intersection.safety_violations == 0
    assert [t for t, _, _ in r.queue_samples] == list(range(0, 60_000, 1000))


# ================================================================ fairness / pairing
def test_pair_uses_byte_identical_arrivals(s1_pair):
    _, res = s1_pair
    fa, sm = res["FT-A"], res["Smart"]
    assert fa.metadata["arrivals_sha256"] == sm.metadata["arrivals_sha256"]
    assert [(v.id, v.movement, v.scheduled_ms, v.source) for v in fa.vehicles] == \
           [(v.id, v.movement, v.scheduled_ms, v.source) for v in sm.vehicles]
    for k in ("scenario", "seed", "dt_ms", "warmup_ms", "measure_ms", "flush_ms", "arrivals_count", "step_order"):
        assert fa.metadata[k] == sm.metadata[k]
    assert fa.metadata["controller"] == "FT-A" and sm.metadata["controller"] == "Smart"


def test_pair_uses_the_given_arrivals_for_every_controller():
    custom = arrivals([("NS", 1_000), ("EL", 2_000), ("WS", 30_000), ("SL", -4_000)])
    res = run_pair(short(), ("FT-A", "Smart", "FT-B"), arrivals=custom)
    want = [(a.id, a.movement, a.time_ms) for a in custom]
    for r in res.values():
        assert [(v.id, v.movement, v.scheduled_ms) for v in r.vehicles] == want
        assert r.metadata["arrivals_sha256"] == arrivals_sha256(custom)


def test_pair_from_arrival_file_is_the_same_file(tmp_path):
    cfg = RunConfig("S4", 7, "FT-A", measure_s=60, flush_s=60)
    res = run_pair(cfg, ("FT-A", "Smart", "FT-B"), arrivals_dir=tmp_path)
    files = list(tmp_path.iterdir())
    assert len(files) == 1                                       # one arrival file for all controllers
    shas = {r.metadata["arrivals_sha256"] for r in res.values()}
    assert len(shas) == 1
    again = run(cfg.with_controller("Smart"), arrivals_dir=tmp_path)
    assert again.metadata["arrivals_sha256"] in shas


def test_same_metric_code_path_for_every_controller(s1_pair):
    _, res = s1_pair
    for r in res.values():
        assert type(r.intersection).__name__ == "IntersectionStats"
        assert r.intersection.safety_violations == 0
    assert res["FT-A"].diagnostics["controller_terminations"] is None
    assert set(res["Smart"].diagnostics["controller_terminations"]) >= {"gap", "max", "hold"}


def test_fixed_controllers_start_at_warmup_start():
    r = run(short(), arrivals=[])
    assert r.signal_log[0] == (-10_000, "NL", "RED_YELLOW")


def test_metadata_is_sufficient_to_reproduce():
    arr = generate_arrivals("S1", 2, -10, 60)
    r = run(short("Smart", smart_params=SmartParams(tau_s=20.0)), arrivals=arr)
    m = r.metadata
    assert m["arrivals_sha256"] == arrivals_sha256(arr) and m["arrivals_count"] == len(arr.arrivals)
    assert (m["scenario"], m["seed"], m["controller"], m["dt_ms"]) == ("S1", 1, "Smart", 100)
    assert (m["warmup_ms"], m["measure_ms"], m["flush_ms"]) == (10_000, 60_000, 60_000)
    assert m["controller_params"]["tau_s"] == 20.0
    f = run(short("FT-B"), arrivals=arr).metadata["controller_params"]
    assert f["schedule"] == "FT-B/S1" and "25000" in f["plans"]


# ================================================================ reproducibility / isolation
def test_deterministic_replay_identical_json():
    arr = generate_arrivals("S9", 4, -10, 60)
    a = run(short("Smart"), arrivals=arr)
    b = run(short("Smart"), arrivals=arr)
    assert a.to_json() == b.to_json()


def test_deterministic_across_hash_seeds():
    code = ("from smarttraffic.runner import RunConfig, run; from smarttraffic.traffic_gen import generate_arrivals; "
            "import hashlib; r = run(RunConfig('S2', 5, 'Smart', warmup_s=10, measure_s=60, flush_s=60), "
            "arrivals=generate_arrivals('S2', 5, -10, 60)); print(hashlib.sha256(r.to_json().encode()).hexdigest())")
    outs = set()
    for seed in ("0", "1", "4242"):
        env = dict(os.environ, PYTHONHASHSEED=seed, PYTHONPATH=str(ROOT))
        outs.add(subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True,
                                text=True, check=True).stdout.strip())
    assert len(outs) == 1


@pytest.mark.parametrize("controller", ["FT-A", "Smart"])
def test_statistics_do_not_change_behaviour(controller):
    arr = generate_arrivals("S7a", 3, -10, 60)
    seen = []
    with_stats = simulate(short(controller), arrivals=arr, collect_stats=True,
                          observers=[lambda v: seen.append(v.now_ms)])
    without = simulate(short(controller), arrivals=arr, collect_stats=False)
    assert with_stats.signals.log == without.signals.log
    assert with_stats.controller.command_log == without.controller.command_log
    assert [v.cross_ms for v in with_stats.vehicles.all_vehicles()] == \
           [v.cross_ms for v in without.vehicles.all_vehicles()]
    assert (with_stats.end_ms, with_stats.steps) == (without.end_ms, without.steps)
    assert seen == sorted(seen) and len(seen) == with_stats.steps + 1


def test_observer_receives_read_only_step_views():
    views = []
    simulate(short(), arrivals=[], observers=[views.append])
    assert all(isinstance(v, StepView) for v in views)
    assert views[0].now_ms == -10_000 and views[-1].now_ms == 60_000
    assert {type(v.controller).__name__ for v in views} == {"FixedTimeController"}


def test_runner_step_order_is_vehicles_sensors_signals_controller():
    import ast, inspect
    src = inspect.getsource(R.simulate)
    body = src[src.index("while True"):]
    order = [body.index(s) for s in ("vm.advance", "sb.advance", "bank.advance", "ctrl.update", "after_step()")]
    assert order == sorted(order)
    assert R.STEP_ORDER == "vehicles>sensors>signals>controller>stats>observers"


# ================================================================ long runs
def test_stage10_fta_s1_validation_reproduced():
    arr = generate_arrivals("S1", 1, -300, 3600)
    r = run(RunConfig("S1", 1, "FT-A"), arrivals=arr)
    st = [v.control_delay_ms for v in r.vehicles if v.measured and v.control_delay_ms is not None and v.movement[1] == "S"]
    lf = [v.control_delay_ms for v in r.vehicles if v.measured and v.control_delay_ms is not None and v.movement[1] == "L"]
    # the independent Stage 10 validation loop gives exactly these for seed 1
    assert sum(st) / len(st) / 1000 == pytest.approx(35.15183541927409, abs=1e-9)
    assert sum(lf) / len(lf) / 1000 == pytest.approx(61.94129914529915, abs=1e-9)
    i = r.intersection
    assert i.unfinished == 0 and r.ended_early and i.safety_violations == 0
    assert i.signal_changes_per_hour == 308.0                     # 8 onsets per 93 s cycle


def test_long_run_sanity_oversaturated():
    arr = generate_arrivals("S7b", 1, -300, 900)
    for ctrl in ("FT-A", "Smart"):
        r = run(RunConfig("S7b", 1, ctrl, measure_s=900, flush_s=300), arrivals=arr)
        i = r.intersection
        assert i.safety_violations == 0
        assert i.arrivals == i.crossed + i.unfinished
        assert i.unfinished > 0 and not r.ended_early and r.end_ms == 1_200_000
        assert i.delay_mean_lower_bound_s >= i.delay_mean_s
        assert len(r.queue_samples) == 900
        assert sum(m.arrivals for m in r.movements) == i.arrivals
