"""Tests for smarttraffic.traffic_gen (study §5, §10)."""
import ast
import json
import random
import time
from pathlib import Path

import numpy as np
import pytest

from smarttraffic import traffic_gen as tg
from smarttraffic.conflicts import MOVES
from smarttraffic.traffic_gen import (ArrivalFileError, ArrivalFileMismatch, ScenarioConfigError,
                                      constant_rate, generate_arrivals, generate_movement_times,
                                      get_or_create_arrivals, load_arrivals, save_arrivals)

STRAIGHTS = ["NS", "SS", "ES", "WS"]
LEFTS = ["NL", "SL", "EL", "WL"]


@pytest.fixture(scope="module")
def sc():
    return tg.load_scenarios()


def rates(sc, sid, t=0.0):
    return {m: sc[sid].rate_at(m, t) for m in MOVES}


def uniform(s, l):
    return {**{m: s for m in STRAIGHTS}, **{m: l for m in LEFTS}}


# ------------------------------------------------------------ movements
def test_all_eight_movements_supported():
    sched = generate_arrivals("S1", 1)
    assert {a.movement for a in sched.arrivals} == set(MOVES)
    for m in MOVES:
        assert generate_movement_times(m, constant_rate(400), 1, 0, 600)


def test_movements_come_from_conflicts_module():
    assert tg.MOVES is MOVES


# ------------------------------------------------------------ rate validation
def test_q_zero_gives_no_arrivals():
    for m in MOVES:
        assert generate_movement_times(m, constant_rate(0), 7, -300, 3600) == []


@pytest.mark.parametrize("q", [-1, -0.001, 2400, 2400.0, 5000, float("inf"), float("nan"),
                               "400", None, True, [400]])
def test_invalid_q_rejected(q):
    with pytest.raises(ValueError):
        tg.validate_rate(q)
    with pytest.raises((ValueError, ScenarioConfigError)):
        constant_rate(q)


@pytest.mark.parametrize("q", [0, 0.5, 1, 120, 400, 2399, 2399.9])
def test_valid_q_accepted(q):
    assert tg.validate_rate(q) == q


def test_exp_mean_formula():
    assert tg.exp_mean(400) == 3600 / 400 - 1.5 == 7.5
    assert tg.exp_mean(120) == 28.5
    with pytest.raises(ValueError):
        tg.exp_mean(0)


# ------------------------------------------------------------ headway model
@pytest.mark.parametrize("q", [60, 400, 1200, 2399])
def test_min_headway_at_least_1_5_s(q):
    for seed in range(5):
        t = generate_movement_times("NS", constant_rate(q), seed, 0, 3600)
        assert all(b - a >= 1500 for a, b in zip(t, t[1:]))
        assert t[0] >= 1500                     # stream starts as if a car arrived at `start`


def test_headway_equals_formula_with_same_random_numbers():
    q, seed, m = 400, 3, "SS"
    t = generate_movement_times(m, constant_rate(q), seed, 0, 1000)
    rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, tg._stream_key(m)])))
    prev, expect = 0, []
    while True:
        s = prev + 1500
        nxt = max(s, round((s / 1000 + rng.standard_exponential() * (3600 / q - 1.5)) * 1000))
        if nxt >= 1_000_000:
            break
        expect.append(nxt)
        prev = nxt
    assert t == expect


@pytest.mark.parametrize("q", [120, 400, 700])
def test_long_run_rate_and_mean_headway(q):
    hours = 100
    t = generate_movement_times("EL", constant_rate(q), 42, 0, 3600 * hours)
    assert abs(len(t) / hours - q) / q < 0.03
    h = np.diff(t) / 1000
    assert abs(h.mean() - 3600 / q) / (3600 / q) < 0.03
    assert h.min() >= 1.5


def test_time_varying_long_run_rates():
    segs = (tg.RateSegment(None, 180_000_000, constant_rate(600)[0].rates),
            tg.RateSegment(180_000_000, None, constant_rate(150)[0].rates))
    t = np.array(generate_movement_times("NS", segs, 5, 0, 360_000)) / 1000
    first, second = (t < 180_000).sum() / 50, (t >= 180_000).sum() / 50
    assert abs(first - 600) / 600 < 0.03 and abs(second - 150) / 150 < 0.03


def test_zero_rate_segment_produces_nothing_inside_it():
    segs = (tg.RateSegment(None, 1_000_000, constant_rate(400)[0].rates),
            tg.RateSegment(1_000_000, 2_000_000, constant_rate(0)[0].rates),
            tg.RateSegment(2_000_000, None, constant_rate(400)[0].rates))
    t = generate_movement_times("WS", segs, 1, 0, 3000)
    assert t and not any(1_000_000 <= x < 2_000_000 for x in t)
    assert any(x >= 2_000_000 for x in t)


# ------------------------------------------------------------ reproducibility
def test_same_seed_identical_records(sc):
    for sid in sc:
        assert generate_arrivals(sid, 4) == generate_arrivals(sid, 4)


def test_reproducible_in_fresh_instances(sc):
    a = generate_arrivals("S7a", 9, scenarios=tg.load_scenarios())
    b = generate_arrivals("S7a", 9, scenarios=tg.load_scenarios())
    assert a == b


def test_reproducible_in_fresh_interpreter(tmp_path):
    import subprocess, sys
    code = ("from smarttraffic import traffic_gen as tg;"
            "print(tg._render(tg.generate_arrivals('S9', 3)), end='')")
    root = Path(tg.__file__).resolve().parent.parent
    outs = [subprocess.run([sys.executable, "-c", code], cwd=root, capture_output=True,
                           text=True, check=True).stdout for _ in range(2)]
    assert outs[0] == outs[1] == tg._render(generate_arrivals("S9", 3))


def test_movement_streams_independent_of_iteration_order():
    segs = constant_rate(400)
    fwd = {m: generate_movement_times(m, segs, 11, -300, 3600) for m in MOVES}
    rev = {m: generate_movement_times(m, segs, 11, -300, 3600) for m in reversed(MOVES)}
    assert fwd == rev
    only = generate_movement_times("WL", segs, 11, -300, 3600)   # generated alone
    assert only == fwd["WL"]


def test_movement_streams_are_distinct():
    segs = constant_rate(400)
    streams = [tuple(generate_movement_times(m, segs, 1, 0, 3600)) for m in MOVES]
    assert len(set(streams)) == 8


def test_changing_one_rate_does_not_touch_other_movements():
    a = generate_arrivals("S1", 2)
    b = generate_arrivals("S2", 2)   # only NS and NL differ from S1
    for m in ["SS", "SL", "ES", "EL", "WS", "WL"]:
        assert [x.time_ms for x in a.for_movement(m)] == [x.time_ms for x in b.for_movement(m)]


def test_different_seeds_differ(sc):
    for sid in sc:
        schedules = {tuple((a.time_ms, a.movement) for a in generate_arrivals(sid, s).arrivals)
                     for s in tg.FINAL_SEEDS}
        assert len(schedules) == 10


def test_longer_interval_extends_shorter_one():
    short = generate_arrivals("S1", 1, -300, 1800)
    long = generate_arrivals("S1", 1, -300, 3600)
    key = lambda s: [(a.time_ms, a.movement) for a in s.arrivals]
    assert key(long)[:len(short.arrivals)] == key(short)
    assert all(a.time_ms >= 1_800_000 for a in long.arrivals[len(short.arrivals):])


@pytest.mark.parametrize("seed", [-1, 1.0, "1", None, True])
def test_bad_seed_rejected(seed):
    with pytest.raises(ValueError):
        generate_arrivals("S1", seed)


# ------------------------------------------------------------ ordering / records
def test_sorted_ids_and_seq(sc):
    for sid in sc:
        s = generate_arrivals(sid, 1)
        keys = [(a.time_ms, MOVES.index(a.movement), a.seq) for a in s.arrivals]
        assert keys == sorted(keys)
        assert [a.id for a in s.arrivals] == list(range(len(s.arrivals)))
        for m in MOVES:
            ms = s.for_movement(m)
            assert [a.seq for a in ms] == list(range(len(ms)))
            assert [a.time_ms for a in ms] == sorted(a.time_ms for a in ms)


def test_arrival_record_is_immutable():
    a = generate_arrivals("S1", 1).arrivals[0]
    with pytest.raises(Exception):
        a.time_ms = 0
    assert a.time == a.time_ms / 1000


# ------------------------------------------------------------ interval handling
def test_default_interval_is_warmup_plus_measurement_no_flush(sc):
    for sid in sc:
        s = generate_arrivals(sid, 1)
        assert (s.meta.start_ms, s.meta.end_ms) == (-300_000, 3_600_000)
        assert all(-300_000 <= a.time_ms < 3_600_000 for a in s.arrivals)


@pytest.mark.parametrize("start,end", [(0, 60), (-300, 0), (100.5, 200.25), (3600, 4500)])
def test_no_arrival_outside_interval(sc, start, end):
    for sid in sc:
        s = generate_arrivals(sid, 3, start, end)
        assert all(start * 1000 <= a.time_ms < end * 1000 for a in s.arrivals)


def test_flush_only_if_explicitly_requested():
    assert not any(a.time_ms >= 3_600_000 for a in generate_arrivals("S7b", 1).arrivals)
    assert any(a.time_ms >= 3_600_000 for a in generate_arrivals("S7b", 1, -300, 4500).arrivals)


@pytest.mark.parametrize("start,end", [(10, 10), (10, 5), (0, 0.0001), (float("nan"), 10)])
def test_bad_interval_rejected(start, end):
    with pytest.raises(ValueError):
        generate_arrivals("S1", 1, start, end)


def test_event_boundaries_half_open(sc):
    ev = sc["S10"].events[0]                     # WL at 60, 240, ... 3480
    assert tg.event_times(ev, 60, 61) == [60_000]          # start inclusive
    assert tg.event_times(ev, 0, 60) == []                 # end exclusive
    assert tg.event_times(ev, 0, 60.001) == [60_000]
    assert tg.event_times(ev, 3480, 3480.001) == [3_480_000]
    assert tg.event_times(ev, 3480.001, 9000) == []
    s5 = sc["S5"].events[0]                      # ES 1200..1238
    assert tg.event_times(s5, 1200, 1238) == list(range(1_200_000, 1_238_000, 2000))
    assert len(tg.event_times(s5, 1200, 1238.001)) == 20


def test_random_arrival_boundary_exact():
    t = generate_movement_times("NS", constant_rate(400), 1, 0, 3600)
    x = t[5]
    assert x not in generate_movement_times("NS", constant_rate(400), 1, 0, x / 1000)
    assert x in generate_movement_times("NS", constant_rate(400), 1, 0, x / 1000 + 0.001)


# ------------------------------------------------------------ scenario configuration
def test_all_study_scenarios_present(sc):
    assert list(sc) == ["S1", "S2", "S3", "S4", "S5", "S6", "S7a", "S7b", "S8", "S9", "S10"]


def test_S1(sc):
    assert rates(sc, "S1") == uniform(400, 120) and sc["S1"].events == ()


def test_S2(sc):
    assert rates(sc, "S2") == {**uniform(400, 120), "NS": 700, "NL": 200}


def test_S3(sc):
    assert rates(sc, "S3") == {**uniform(400, 120), "WS": 30, "WL": 10}


def test_S4(sc):
    assert rates(sc, "S4") == uniform(60, 20)


def test_S5_config_and_exact_bursts(sc):
    assert rates(sc, "S5") == uniform(400, 120)
    for seed in (1, 2, 3):
        s = generate_arrivals("S5", seed)
        ev = [a for a in s.arrivals if a.source == "event"]
        es = [a.time_ms for a in ev if a.movement == "ES"]
        nl = [a.time_ms for a in ev if a.movement == "NL"]
        assert es == [1_200_000 + 2000 * k for k in range(20)]    # 1200.0 .. 1238.0
        assert nl == [2_400_000 + 2500 * k for k in range(12)]    # 2400.0 .. 2427.5
        assert len(ev) == 32
        base = generate_arrivals("S1", seed)
        assert [(a.time_ms, a.movement) for a in s.arrivals if a.source == "random"] == \
               [(a.time_ms, a.movement) for a in base.arrivals]   # bursts are added on top


def test_S6_step_change(sc):
    before = {"NS": 600, "SS": 600, "ES": 150, "WS": 150, "NL": 150, "SL": 150, "EL": 40, "WL": 40}
    after = {"NS": 150, "SS": 150, "ES": 600, "WS": 600, "NL": 40, "SL": 40, "EL": 150, "WL": 150}
    assert rates(sc, "S6", -300) == rates(sc, "S6", 1799.999) == before
    assert rates(sc, "S6", 1800) == rates(sc, "S6", 3599.999) == after


def test_S7a_S7b_S8(sc):
    assert rates(sc, "S7a") == uniform(550, 160)
    assert rates(sc, "S7b") == uniform(700, 220)
    assert rates(sc, "S8") == uniform(250, 330)


def test_S9(sc):
    assert rates(sc, "S9") == {"NS": 450, "SS": 120, "ES": 420, "WS": 100,
                               "NL": 60, "SL": 250, "EL": 50, "WL": 230}


def test_S10_config_and_exact_WL(sc):
    assert rates(sc, "S10") == {**uniform(550, 160), "WL": 0}
    for seed in (1, 5):
        wl = generate_arrivals("S10", seed).for_movement("WL")
        assert [a.time_ms for a in wl] == [60_000 + 180_000 * k for k in range(20)]
        assert wl[-1].time == 3480.0 and all(a.source == "event" for a in wl)


def test_scenario_rates_constant_over_time_except_S6(sc):
    for sid in sc:
        if sid != "S6":
            assert len(sc[sid].segments) == 1


# ------------------------------------------------------------ config validation
def _write_cfg(tmp_path, scen):
    p = tmp_path / "c.json"
    p.write_text(json.dumps({"scenarios": {"X": scen}}))
    return p


GOOD_RATES = {m: 100 for m in MOVES}


@pytest.mark.parametrize("scen", [
    {"segments": [{"from": None, "until": None, "rates": {**GOOD_RATES, "NS": 2400}}]},
    {"segments": [{"from": None, "until": None, "rates": {**GOOD_RATES, "NS": -5}}]},
    {"segments": [{"from": None, "until": None, "rates": {k: v for k, v in GOOD_RATES.items() if k != "WL"}}]},
    {"segments": [{"from": None, "until": None, "rates": {**GOOD_RATES, "XX": 1}}]},
    {"segments": [{"from": 0, "until": None, "rates": GOOD_RATES}]},
    {"segments": [{"from": None, "until": 10, "rates": GOOD_RATES},
                  {"from": 20, "until": None, "rates": GOOD_RATES}]},
    {"segments": [{"from": None, "until": None, "rates": GOOD_RATES}],
     "events": [{"movement": "ZZ", "count": 1, "start": 0, "interval": 1}]},
    {"segments": [{"from": None, "until": None, "rates": GOOD_RATES}],
     "events": [{"movement": "NS", "count": 0, "start": 0, "interval": 1}]},
    {"segments": [{"from": None, "until": None, "rates": GOOD_RATES}],
     "events": [{"movement": "NS", "count": 3, "start": 0, "interval": 0}]},
    {"segments": [{"from": None, "until": None, "rates": GOOD_RATES}], "typo": 1},
])
def test_bad_config_rejected(tmp_path, scen):
    with pytest.raises(ScenarioConfigError):
        tg.load_scenarios(_write_cfg(tmp_path, scen))


def test_unknown_scenario_rejected():
    with pytest.raises(KeyError):
        generate_arrivals("S11", 1)


# ------------------------------------------------------------ CSV files
def test_csv_round_trip_lossless(tmp_path, sc):
    for sid in sc:
        s = generate_arrivals(sid, 7)
        p = save_arrivals(s, tmp_path / f"{sid}.csv")
        assert load_arrivals(p) == s


def test_csv_bytes_deterministic_and_lf(tmp_path):
    s = generate_arrivals("S5", 1)
    a, b = save_arrivals(s, tmp_path / "a.csv"), save_arrivals(s, tmp_path / "b.csv")
    assert a.read_bytes() == b.read_bytes()
    assert b"\r\n" not in a.read_bytes()


def test_csv_readable_header(tmp_path):
    p = save_arrivals(generate_arrivals("S1", 1, 0, 30), tmp_path / "x.csv")
    lines = p.read_text().splitlines()
    assert lines[0].startswith("# SmartTrafficLights arrival file")
    assert "# scenario=S1" in lines and "# seed=1" in lines and "# start=0.000" in lines
    assert "id,time,movement,seq,source" in lines


def test_save_existing_identical_is_noop(tmp_path):
    s = generate_arrivals("S1", 1)
    p = save_arrivals(s, tmp_path / "x.csv")
    before = p.stat().st_mtime_ns
    assert save_arrivals(s, p) == p
    assert p.stat().st_mtime_ns == before


def test_save_refuses_to_overwrite_mismatch(tmp_path):
    p = save_arrivals(generate_arrivals("S1", 1), tmp_path / "x.csv")
    original = p.read_bytes()
    for other in (generate_arrivals("S1", 2), generate_arrivals("S2", 1),
                  generate_arrivals("S1", 1, -300, 1800)):
        with pytest.raises(ArrivalFileMismatch):
            save_arrivals(other, p)
    assert p.read_bytes() == original
    save_arrivals(generate_arrivals("S1", 2), p, overwrite=True)    # explicit only
    assert load_arrivals(p).meta.seed == 2


def test_load_detects_metadata_mismatch(tmp_path):
    s = generate_arrivals("S3", 4)
    p = save_arrivals(s, tmp_path / "x.csv")
    load_arrivals(p, expected=s.meta)
    for field, value in [("seed", 5), ("scenario", "S1"), ("start_ms", 0), ("end_ms", 4_500_000),
                         ("config_sha256", "0" * 64), ("generator_version", "999")]:
        from dataclasses import replace
        with pytest.raises(ArrivalFileMismatch, match=field):
            load_arrivals(p, expected=replace(s.meta, **{field: value}))


def test_load_detects_tampering(tmp_path):
    p = save_arrivals(generate_arrivals("S1", 1, 0, 120), tmp_path / "x.csv")
    text = p.read_text()
    row = text.splitlines()[-1]
    parts = row.split(",")
    parts[1] = f"{float(parts[1]) + 0.001:.3f}"
    p.write_text(text.replace(row, ",".join(parts)), newline="")
    with pytest.raises(ArrivalFileError, match="rows_sha256"):
        load_arrivals(p)


def test_load_rejects_non_arrival_file(tmp_path):
    p = tmp_path / "x.csv"
    p.write_text("a,b\n1,2\n")
    with pytest.raises(ArrivalFileError):
        load_arrivals(p)


def test_get_or_create(tmp_path):
    s1, p1 = get_or_create_arrivals("S10", 3, directory=tmp_path)
    assert p1.name == "S10_seed3_t-300_3600.csv" and p1.exists()
    s2, p2 = get_or_create_arrivals("S10", 3, directory=tmp_path)
    assert p1 == p2 and s1 == s2 == generate_arrivals("S10", 3)


def test_get_or_create_detects_changed_config(tmp_path, sc):
    get_or_create_arrivals("S1", 1, directory=tmp_path)
    raw = json.loads(tg.SCENARIO_FILE.read_text())
    raw["scenarios"]["S1"]["segments"][0]["rates"]["NS"] = 401
    cfg = tmp_path / "changed.json"
    cfg.write_text(json.dumps(raw))
    with pytest.raises(ArrivalFileMismatch, match="config_sha256"):
        get_or_create_arrivals("S1", 1, directory=tmp_path, scenarios=tg.load_scenarios(cfg))


def test_description_change_does_not_invalidate_files(tmp_path):
    raw = json.loads(tg.SCENARIO_FILE.read_text())
    raw["scenarios"]["S1"]["description"] = "renamed"
    cfg = tmp_path / "c.json"
    cfg.write_text(json.dumps(raw))
    assert tg.load_scenarios(cfg)["S1"].config_sha256 == tg.load_scenarios()["S1"].config_sha256


# ------------------------------------------------------------ independence
def test_no_global_random_state(monkeypatch):
    ref = generate_arrivals("S6", 2)
    random.seed(12345)
    np.random.seed(54321)
    def boom(*a, **k):
        raise AssertionError("global RNG used")
    for name in ("random", "uniform", "expovariate", "gauss", "shuffle", "choice"):
        monkeypatch.setattr(random, name, boom)
    for name in ("random", "rand", "exponential", "standard_exponential", "seed", "uniform"):
        monkeypatch.setattr(np.random, name, boom)
    assert generate_arrivals("S6", 2) == ref


def test_no_wall_clock(monkeypatch):
    ref = generate_arrivals("S1", 1)
    def boom(*a, **k):
        raise AssertionError("wall clock used")
    for name in ("time", "monotonic", "perf_counter", "time_ns", "sleep"):
        monkeypatch.setattr(time, name, boom)
    assert generate_arrivals("S1", 1) == ref


def test_no_controller_or_simulation_imports_and_no_hash():
    src = Path(tg.__file__).read_text(encoding="utf-8")
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add(node.module)
    project = {m for m in imported if m and m.startswith("smarttraffic")}
    assert project == {"smarttraffic.conflicts"}
    assert not imported & {"random", "time", "datetime"}
    calls = {n.func.id for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
    assert "hash" not in calls
