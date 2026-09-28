"""Stage 14: experiment infrastructure, pre-registered tuning plan, selection rule, lock.

Short simulation windows keep these tests fast; the study windows are enforced for the real
plan and the official matrix, which are tested separately.
"""
from dataclasses import asdict, replace
import gzip
import hashlib
import json
from pathlib import Path

import pytest

from smarttraffic import experiments as ex
from smarttraffic import tuning as tu
from smarttraffic.controller_smart import SmartParams
from smarttraffic.runner import RunConfig, run
from smarttraffic.traffic_gen import FINAL_SEEDS, TUNING_SEEDS, generate_arrivals

ROOT = Path(__file__).resolve().parent.parent
SHORT = dict(warmup_s=20.0, measure_s=120.0, flush_s=120.0)


def short(sc, seed, ctrl, label, params=None):
    return ex.RunSpec(sc, seed, ctrl, label, params or SmartParams(), **SHORT)


# ================================================================ plan
def test_frozen_plan_is_exactly_the_approved_one():
    plan = tu.load_plan()
    assert plan["scenario"] == "S1"
    assert plan["seeds"] == [101, 102, 103, 104, 105] == list(TUNING_SEEDS)
    assert plan["grid"] == {"tau_s": (5.0, 10.0, 15.0, 20.0, 30.0), "w_max_s": (90.0, 105.0, 120.0, 150.0)}
    assert plan["defaults"] == {"tau_s": 10, "w_max_s": 90}
    assert plan["guardrails"] == {"p95_mean_max_increase": 0.0, "max_delay_worst_max_increase": 0.10}
    assert plan["tie_band"] == 0.01
    assert plan["references"] == ["FT-A", "FT-B"]
    assert len(tu.candidates(plan)) == 20
    assert len(tu.plan_sha256()) == 64


def _raw_plan():
    return json.loads(tu.PLAN_FILE.read_text(encoding="utf-8"))


@pytest.mark.parametrize("change, msg", [
    (lambda p: p.__setitem__("scenario", "S7a"), "S1 only"),
    (lambda p: p.__setitem__("seeds", [1, 2, 3, 4, 5]), "tuning seeds"),
    (lambda p: p.__setitem__("seeds", [101, 102, 103, 104, 1]), "tuning seeds"),
    (lambda p: p.__setitem__("seeds", [101, 102, 103, 104]), "tuning seeds"),
    (lambda p: p["grid"].__setitem__("max_green_straight_s", [45, 60]), "only"),
    (lambda p: p["grid"].pop("w_max_s"), "only"),
    (lambda p: p["grid"].__setitem__("tau_s", [5, 15, 20]), "study default"),
    (lambda p: p["grid"].__setitem__("tau_s", [10, 5]), "increasing"),
    (lambda p: p["defaults"].__setitem__("tau_s", 20), "study value"),
    (lambda p: p.__setitem__("windows", {"warmup_s": 300.0, "measure_s": 1800.0, "flush_s": 900.0, "dt_ms": 100}), "windows"),
    (lambda p: p.__setitem__("tie_band", 0.05), "tie band"),
    (lambda p: p["guardrails"].__setitem__("max_delay_worst_max_increase", 0.5), "guardrails"),
    (lambda p: p.__setitem__("references", ["FT-A"]), "references"),
])
def test_plan_validation_rejects_deviations(change, msg):
    p = _raw_plan()
    change(p)
    with pytest.raises(tu.TuningPlanError, match=msg):
        tu.validate_plan(p)


def test_tuning_specs_use_only_s1_tuning_seeds_and_no_evaluation_seed():
    plan = tu.load_plan()
    specs = tu.tuning_specs(plan)
    assert len(specs) == 5 * (2 + 1 + 20)
    assert {s.scenario for s in specs} == {"S1"}
    assert {s.seed for s in specs} == set(TUNING_SEEDS)
    assert not ({s.seed for s in specs} & set(FINAL_SEEDS))
    assert all(s.is_study_window() for s in specs)
    for s in specs:
        if s.controller == "Smart":
            changed = {k for k, v in asdict(s.smart_params).items() if v != asdict(SmartParams())[k]}
            assert changed <= set(tu.TUNED_PARAMS)


def test_run_key_identity():
    a = ex.RunSpec("S1", 101, "Smart", "Smart-study")
    b = ex.RunSpec("S1", 101, "Smart", "tau10_wmax90", replace(SmartParams(), tau_s=10.0, w_max_s=90.0))
    assert ex.run_key(a, "x") == ex.run_key(b, "x")                    # label is not identity
    c = ex.RunSpec("S1", 101, "Smart", "tau5", replace(SmartParams(), tau_s=5.0))
    assert ex.run_key(a, "x") != ex.run_key(c, "x")
    assert ex.run_key(a, "x") != ex.run_key(a, "y")                    # arrivals are identity
    f1 = ex.RunSpec("S1", 101, "FT-A", "FT-A")
    f2 = ex.RunSpec("S1", 101, "FT-A", "FT-A", replace(SmartParams(), tau_s=5.0))
    assert ex.run_key(f1, "x") == ex.run_key(f2, "x")                  # FT ignores Smart params
    assert ex.run_key(f1, "x") != ex.run_key(ex.RunSpec("S1", 102, "FT-A", "FT-A"), "x")


def test_study_defaults_unchanged():
    p = SmartParams()
    assert (p.tau_s, p.w_max_s, p.max_green_straight_s, p.max_green_left_s) == (10.0, 90.0, 45.0, 30.0)
    assert (p.min_green_straight_s, p.min_green_left_s, p.extended_initial_s) == (7.0, 5.0, 12.0)
    assert (p.green_bonus_qhat, p.fault_qhat, p.use_max_green, p.use_aging, p.use_w_max) == (2, 2, True, True, True)


# ================================================================ signal-log check
def test_signal_log_check():
    good = [(0, "NS", "RED_YELLOW"), (1000, "NS", "GREEN"), (5000, "NS", "FLASHING_GREEN"),
            (8000, "NS", "YELLOW"), (11500, "NS", "ALL_RED"), (13500, "NS", "RED")]
    assert ex.check_signal_log(good) == []
    short_green = [(0, "NS", "RED_YELLOW"), (1000, "NS", "GREEN"), (2000, "NS", "FLASHING_GREEN"),
                   (5000, "NS", "YELLOW")]
    assert any("minimum" in p for p in ex.check_signal_log(short_green))
    assert any("illegal" in p for p in ex.check_signal_log([(0, "NL", "GREEN")]))
    bad_y = [(0, "NL", "RED_YELLOW"), (1000, "NL", "GREEN"), (9000, "NL", "FLASHING_GREEN"),
             (12000, "NL", "YELLOW"), (14000, "NL", "ALL_RED")]
    assert any("YELLOW lasted 2000" in p for p in ex.check_signal_log(bad_y))


# ================================================================ runs
def test_execute_spec_equals_the_stage12_runner(tmp_path):
    s = short("S1", 101, "Smart", "Smart-study")
    arr = ex.prepare_arrivals([s], tmp_path)
    path, sha, n = arr[("S1", 101, -20.0, 120.0)]
    rec = ex.execute_spec(s, path, sha)
    assert rec["error"] is None and rec["signal_check"] == []
    direct = run(RunConfig("S1", 101, "Smart", **SHORT), arrivals=generate_arrivals("S1", 101, -20.0, 120.0))
    assert rec["result_sha256"] == hashlib.sha256(direct.to_json().encode()).hexdigest()
    assert rec["result"] == json.loads(direct.to_json())


def test_record_metadata_is_complete(tmp_path):
    s = short("S1", 102, "Smart", "tau5", replace(SmartParams(), tau_s=5.0))
    path, sha, n = ex.prepare_arrivals([s], tmp_path)[("S1", 102, -20.0, 120.0)]
    rec = ex.execute_spec(s, path, sha)
    for k in ("engine", "key", "label", "spec", "metadata", "summary", "movements", "diagnostics",
              "signal_check", "result_sha256", "result", "arrivals_path"):
        assert k in rec, k
    md = rec["metadata"]
    for k in ("scenario", "seed", "controller", "controller_params", "dt_ms", "warmup_ms", "measure_ms",
              "flush_ms", "arrivals_sha256", "arrivals_count", "step_order"):
        assert k in md, k
    assert md["controller_params"]["tau_s"] == 5.0 and md["arrivals_sha256"] == sha and md["arrivals_count"] == n
    assert rec["spec"]["smart_params"]["tau_s"] == 5.0 and rec["label"] == "tau5"


def test_changed_arrival_file_is_recorded_as_an_error(tmp_path):
    s = short("S1", 103, "FT-A", "FT-A")
    path, sha, _ = ex.prepare_arrivals([s], tmp_path)[("S1", 103, -20.0, 120.0)]
    rec = ex.execute_spec(s, path, "0" * 64)
    assert rec["error"] and "changed" in rec["error"]


def _specs():
    out = []
    for seed in (101, 102):
        out += [short("S1", seed, "FT-A", "FT-A"), short("S1", seed, "FT-B", "FT-B"),
                short("S1", seed, "Smart", "Smart-study"),
                short("S1", seed, "Smart", "tau10_wmax90", replace(SmartParams(), tau_s=10.0, w_max_s=90.0)),
                short("S1", seed, "Smart", "tau20_wmax120", replace(SmartParams(), tau_s=20.0, w_max_s=120.0))]
    return out


def _summary_bytes(records):
    return json.dumps(ex.summary_rows(records), sort_keys=True).encode()


def test_parallel_sequential_and_resumed_campaigns_are_identical(tmp_path):
    arr = tmp_path / "arr"
    seq = ex.run_campaign(_specs(), tmp_path / "seq", workers=1, arrivals_dir=arr, log=lambda *a: None)
    par = ex.run_campaign(list(reversed(_specs())), tmp_path / "par", workers=2, arrivals_dir=arr,
                          log=lambda *a: None)
    assert _summary_bytes(seq) == _summary_bytes(par)
    h1 = ex.write_csv(tmp_path / "a.csv", ex.summary_rows(seq), ex.SUMMARY_FIELDS)
    h2 = ex.write_csv(tmp_path / "b.csv", ex.summary_rows(par), ex.SUMMARY_FIELDS)
    assert h1 == h2
    # Smart-study and tau10_wmax90 are one simulation: 4 distinct runs per seed
    assert len({r["key"] for r in seq}) == 8 and len(seq) == 10
    files = sorted((tmp_path / "seq" / "runs").glob("*.json.gz"))
    assert len(files) == 8
    # resume: remove one result, corrupt another, leave a stray temp file
    files[0].unlink()
    files[1].write_bytes(gzip.compress(b'{"key": "truncated'))
    (tmp_path / "seq" / "runs" / ".stray.123.tmp").write_bytes(b"partial")
    ran = []
    again = ex.run_campaign(_specs(), tmp_path / "seq", workers=1, arrivals_dir=arr, log=ran.append)
    assert "2 to run" in ran[0]
    assert _summary_bytes(again) == _summary_bytes(seq)
    assert len(list((tmp_path / "seq" / "runs").glob("*.json.gz"))) == 8     # no duplicates
    # a complete campaign resumes with nothing to do
    ran = []
    ex.run_campaign(_specs(), tmp_path / "seq", workers=2, arrivals_dir=arr, log=ran.append)
    assert "0 to run" in ran[0]


def test_every_controller_of_a_seed_reads_the_same_arrivals(tmp_path):
    recs = ex.run_campaign(_specs(), tmp_path / "o", workers=1, arrivals_dir=tmp_path / "a", log=lambda *a: None)
    per = {}
    for r in recs:
        per.setdefault(r["spec"]["seed"], set()).add(r["metadata"]["arrivals_sha256"])
    assert all(len(v) == 1 for v in per.values()) and per[101] != per[102]


def test_workers_only_read_arrival_files(tmp_path):
    specs = _specs()
    arr = ex.prepare_arrivals(specs, tmp_path / "a")
    stamp = {p: Path(p).stat().st_mtime_ns for p, _, _ in arr.values()}
    ex.run_campaign(specs, tmp_path / "o", workers=2, arrivals_dir=tmp_path / "a", log=lambda *a: None)
    assert {p: Path(p).stat().st_mtime_ns for p in stamp} == stamp
    assert len(list((tmp_path / "a").glob("*.csv"))) == 2


def test_duplicate_spec_is_rejected(tmp_path):
    s = short("S1", 101, "FT-A", "FT-A")
    with pytest.raises(ValueError, match="duplicate"):
        ex.run_campaign([s, s], tmp_path, workers=1, arrivals_dir=tmp_path / "a", log=lambda *a: None)


def test_default_workers():
    assert 1 <= ex.default_workers() <= 4


# ================================================================ selection rule (pure)
def _fake(label, seed, delay, p95=100.0, dmax=200.0, sha="A", unfinished=0, safety=0, error=None,
          signal_check=(), scenario="S1"):
    tau, w = 10.0, 90.0
    if label.startswith("tau"):
        a, b = label[3:].split("_wmax")
        tau, w = float(a), float(b)
    rec = {"label": label, "key": f"{label}-{seed}", "error": error,
           "spec": {"scenario": scenario, "seed": seed, "controller": "Smart",
                    "smart_params": {"tau_s": tau, "w_max_s": w}},
           "metadata": {"arrivals_sha256": f"{sha}{seed}"}, "signal_check": list(signal_check),
           "movements": [{"movement": m, "delay_mean_s": delay} for m in ("NS", "NL", "SS", "SL", "ES", "EL", "WS", "WL")],
           "diagnostics": {"detector_call_wait_max_s": {"NS": 100.0}, "detector_calls_over_120s": {"NS": 0},
                           "controller_hold_decisions": 3}}
    rec["summary"] = {"delay_mean_s": delay, "delay_p95_s": p95, "delay_max_s": dmax,
                      "queue_mean_per_movement_veh": 1.0, "throughput_veh_h": 2000.0, "unfinished": unfinished,
                      "vehicles_control_delay_over_90s": 1, "jain_fairness": 0.9,
                      "signal_changes_per_hour": 200.0, "safety_violations": safety}
    return rec


def _world(plan, delays, **over):
    """Records for every needed label on the plan's seeds; delays: label -> mean delay."""
    recs = []
    labels = ["FT-A", "FT-B", "Smart-study"] + [lab for lab, _ in tu.candidates(plan)]
    for lab in labels:
        for seed in plan["seeds"]:
            kw = dict(over.get(lab, {}))
            d = delays.get(lab, 60.0 if lab != "Smart-study" else delays.get("tau10_wmax90", 60.0))
            recs.append(_fake(lab, seed, d, **kw))
    return recs


def test_selection_minimises_delay_and_is_order_independent():
    plan = tu.load_plan()
    recs = _world(plan, {"tau10_wmax90": 60.0, "tau20_wmax120": 50.0, "tau15_wmax105": 55.0})
    a = tu.evaluate(plan, recs)
    b = tu.evaluate(plan, list(reversed(recs)))
    assert a["selected"]["label"] == b["selected"]["label"] == "tau20_wmax120"
    assert [r["label"] for r in a["candidates"]] == [r["label"] for r in b["candidates"]]
    assert a["candidates"][0]["label"] == "tau20_wmax120" and a["candidates"][0]["rank"] == 1
    assert tu.candidate_table(a) == tu.candidate_table(b)


def test_tie_band_prefers_fewer_changes_then_distance_then_grid_order():
    plan = tu.load_plan()
    # best 50.0; 50.4 is within 1 %, one changed parameter -> preferred over the two-parameter best
    r = tu.evaluate(plan, _world(plan, {"tau20_wmax120": 50.0, "tau15_wmax90": 50.4, "tau10_wmax90": 60.0}))
    assert r["selected"]["label"] == "tau15_wmax90"
    # same number of changes: the smaller relative distance wins (tau 15 vs 30)
    r = tu.evaluate(plan, _world(plan, {"tau30_wmax90": 50.0, "tau15_wmax90": 50.3, "tau10_wmax90": 60.0}))
    assert r["selected"]["label"] == "tau15_wmax90"
    # the study default itself wins a tie (zero changes)
    r = tu.evaluate(plan, _world(plan, {"tau20_wmax120": 50.0, "tau10_wmax90": 50.2}))
    assert r["selected"]["label"] == "tau10_wmax90"
    # outside the band the best wins
    r = tu.evaluate(plan, _world(plan, {"tau20_wmax120": 50.0, "tau15_wmax90": 50.6, "tau10_wmax90": 60.0}))
    assert r["selected"]["label"] == "tau20_wmax120"
    # equal delay, equal changes, equal distance: fixed grid order (tau 5 before tau 15)
    r = tu.evaluate(plan, _world(plan, {"tau5_wmax90": 50.0, "tau15_wmax90": 50.0, "tau10_wmax90": 60.0}))
    assert r["selected"]["label"] == "tau5_wmax90"


def test_hard_constraints_and_guardrails_reject():
    plan = tu.load_plan()
    over = {"tau5_wmax90": {"p95": 100.5}, "tau15_wmax90": {"dmax": 221.0},
            "tau20_wmax90": {"unfinished": 1}, "tau30_wmax90": {"safety": 1},
            "tau5_wmax105": {"error": "IllegalTransition: refused"},
            "tau5_wmax120": {"signal_check": ["NS: green time 4000 ms < minimum 7000 ms"]}}
    delays = {lab: 40.0 for lab in over}
    delays.update({"tau10_wmax90": 60.0, "tau15_wmax150": 55.0})
    r = tu.evaluate(plan, _world(plan, delays, **over))
    rows = {c["label"]: c for c in r["candidates"]}
    assert "P95 guardrail" in rows["tau5_wmax90"]["reject_reasons"]
    assert "max-delay guardrail" in rows["tau15_wmax90"]["reject_reasons"]
    assert "unfinished" in rows["tau20_wmax90"]["reject_reasons"]
    assert "safety" in rows["tau30_wmax90"]["reject_reasons"]
    assert "simulation error" in rows["tau5_wmax105"]["reject_reasons"]
    assert "signal-log" in rows["tau5_wmax120"]["reject_reasons"]
    assert all(not rows[k]["feasible"] for k in over)
    assert r["selected"]["label"] == "tau15_wmax150"
    # +10 % exactly is allowed
    r = tu.evaluate(plan, _world(plan, {"tau20_wmax150": 40.0, "tau10_wmax90": 60.0},
                                 **{"tau20_wmax150": {"dmax": 220.0}}))
    assert r["selected"]["label"] == "tau20_wmax150"


def test_arrival_mismatch_or_wrong_seeds_or_scenario_are_refused():
    plan = tu.load_plan()
    r = tu.evaluate(plan, _world(plan, {"tau20_wmax120": 50.0}, **{"FT-B": {"sha": "B"}}))
    assert r["selected"] is None and not r["arrivals_ok"]
    recs = _world(plan, {})
    recs.append(_fake("tau5_wmax90", 1, 10.0))
    with pytest.raises(tu.TuningPlanError, match="not a tuning seed"):
        tu.evaluate(plan, recs)
    recs = [x for x in _world(plan, {}) if not (x["label"] == "tau5_wmax90" and x["spec"]["seed"] == 105)]
    with pytest.raises(tu.TuningPlanError, match="expected"):
        tu.evaluate(plan, recs)
    recs = _world(plan, {})
    recs[0]["spec"]["scenario"] = "S7a"
    with pytest.raises(tu.TuningPlanError, match="S7a"):
        tu.evaluate(plan, recs)


# ================================================================ lock
def _lock(tmp_path, tau=20.0, w=120.0):
    p = tmp_path / "lock.json"
    sha = tu.write_lock(p, replace(SmartParams(), tau_s=tau, w_max_s=w), "a" * 64, "b" * 64,
                        {"label": f"tau{tau:g}_wmax{w:g}"})
    return p, sha


def test_lock_roundtrip_and_refusals(tmp_path):
    p, sha = _lock(tmp_path)
    lock = tu.load_lock(p, expected_sha=sha)
    assert lock["smart_params"] == replace(SmartParams(), tau_s=20.0, w_max_s=120.0)
    assert lock["name"] == "Smart-T1" and lock["tuning_plan_sha256"] == "a" * 64
    assert tu.write_lock(p, lock["smart_params"], "a" * 64, "b" * 64, {"label": "tau20_wmax120"}) == sha
    with pytest.raises(tu.LockError, match="different lock"):
        tu.write_lock(p, replace(SmartParams(), tau_s=5.0), "a" * 64, "b" * 64, {"label": "x"})
    with pytest.raises(tu.LockError, match="expected"):
        tu.load_lock(p, expected_sha="c" * 64)
    with pytest.raises(tu.LockError, match="no locked"):
        tu.load_lock(tmp_path / "missing.json")
    raw = json.loads(p.read_text())
    raw["params"]["tau_s"] = 15.0
    p.write_text(json.dumps(raw))
    with pytest.raises(tu.LockError, match="edited"):
        tu.load_lock(p)


def test_lock_may_not_change_untuned_parameters(tmp_path):
    p = tmp_path / "lock.json"
    tu.write_lock(p, replace(SmartParams(), tau_s=20.0, max_green_straight_s=60.0), "a" * 64, "b" * 64, {})
    with pytest.raises(tu.LockError, match="may not be tuned"):
        tu.load_lock(p)


def test_official_matrix_uses_only_the_lock(tmp_path):
    p, sha = _lock(tmp_path)
    lock = tu.load_lock(p)
    specs = tu.official_specs(lock)
    assert len(specs) == 11 * 10 * 4 + 10
    assert {s.seed for s in specs} == set(FINAL_SEEDS)
    assert all(s.is_study_window() for s in specs)
    t1 = [s for s in specs if s.label == "Smart-T1"]
    assert len(t1) == 110 and all(s.smart_params == lock["smart_params"] for s in t1)
    assert all(s.smart_params == SmartParams() for s in specs if s.label == "Smart-study")
    assert {s.scenario for s in specs if s.controller == "FT-TOD"} == {"S6"}
    tu.check_official(specs, lock)
    with pytest.raises(tu.LockError):
        tu.official_specs({"smart_params": replace(SmartParams(), tau_s=5.0)})      # not a loaded lock
    with pytest.raises(tu.LockError, match="seeds"):
        tu.official_specs(lock, seeds=TUNING_SEEDS)
    with pytest.raises(tu.LockError, match="official"):
        tu.official_specs(lock, scenarios=("CUSTOM",))
    for bad in (ex.RunSpec("S2", 1, "Smart", "Smart-T1", replace(SmartParams(), tau_s=5.0)),     # custom params
                ex.RunSpec("S2", 1, "Smart", "Smart-S2", lock["smart_params"]),                   # per-scenario label
                ex.RunSpec("S2", 1, "Smart", "Smart-T1", lock["smart_params"], measure_s=1800.0),  # windows
                ex.RunSpec("CUSTOM", 1, "FT-A", "FT-A")):                                        # custom scenario
        with pytest.raises(tu.LockError):
            tu.check_official([bad], lock)


# ================================================================ end to end (tiny grid, short windows)
def test_tune_end_to_end_writes_auditable_outputs(tmp_path, monkeypatch):
    from smarttraffic import run_experiments as rx
    plan = tu.load_plan()
    plan["windows"] = {"warmup_s": 20.0, "measure_s": 120.0, "flush_s": 120.0, "dt_ms": 100}
    plan["seeds"] = [101, 102]
    plan["grid"] = {"tau_s": (10.0, 20.0), "w_max_s": (90.0,)}
    monkeypatch.setattr(tu, "load_plan", lambda path=None: plan)
    out = rx.tune(tmp_path / "t", workers=2, lock_path=tmp_path / "lock.json",
                  arrivals_dir=tmp_path / "arr", log=lambda *a: None)
    sel, man = out["selection"], out["manifest"]
    for f in ("runs.csv", "candidates.csv", "selection.json", "manifest.json"):
        assert (tmp_path / "t" / f).exists()
    for k in ("tuning_plan_sha256", "source_sha256", "git", "environment", "engine", "runs_csv_sha256",
              "candidates_csv_sha256", "lock_sha256", "created_utc", "workers"):
        assert k in man, k
    assert man["tuning_plan_sha256"] == tu.plan_sha256()
    assert all(sel["determinism"].values()) and len(sel["determinism"]) in (2, 4)
    lock = tu.load_lock(tmp_path / "lock.json", expected_sha=man["lock_sha256"])
    assert lock["selection"]["label"] == sel["selected_label"]
    assert lock["candidates_csv_sha256"] == man["candidates_csv_sha256"]
    assert man["lock_mode"] == "written" and (tmp_path / "t" / "fingerprints.csv").exists()
    # re-running (reproduction) resumes everything, verifies the lock and never rewrites it
    lock_bytes = (tmp_path / "lock.json").read_bytes()
    again = rx.tune(tmp_path / "t", workers=1, lock_path=tmp_path / "lock.json",
                    arrivals_dir=tmp_path / "arr", verify_lock_sha=man["lock_sha256"], log=lambda *a: None)
    assert again["manifest"]["runs_csv_sha256"] == man["runs_csv_sha256"]
    assert again["manifest"]["candidates_csv_sha256"] == man["candidates_csv_sha256"]
    assert again["manifest"]["fingerprints_csv_sha256"] == man["fingerprints_csv_sha256"]
    assert again["manifest"]["lock_sha256"] == man["lock_sha256"]
    assert again["manifest"]["lock_mode"] == "verified (not written)"
    assert (tmp_path / "lock.json").read_bytes() == lock_bytes


def test_tie_break_order_is_changes_then_distance_not_grid_order():
    plan = tu.load_plan()
    # grid order would pick tau5_wmax105 (earlier), the rule prefers one changed parameter
    r = tu.evaluate(plan, _world(plan, {"tau5_wmax105": 50.0, "tau15_wmax90": 50.2, "tau10_wmax90": 60.0}))
    assert r["selected"]["label"] == "tau15_wmax90"
    # both one change: tau10_wmax150 comes first in the grid, but tau15_wmax90 is closer (0.5 < 0.667)
    r = tu.evaluate(plan, _world(plan, {"tau10_wmax150": 50.0, "tau15_wmax90": 50.2, "tau10_wmax90": 60.0}))
    assert r["selected"]["label"] == "tau15_wmax90"


def test_summary_rows_order_does_not_depend_on_input_order():
    plan = tu.load_plan()
    recs = _world(plan, {"tau20_wmax120": 50.0})
    for r in recs:
        r["summary"].update({"end_ms": 0, "ended_early": True})
    assert ex.summary_rows(recs) == ex.summary_rows(list(reversed(recs)))
    rows = ex.summary_rows(list(reversed(recs)))
    assert [(x["label"], x["seed"]) for x in rows] == sorted((x["label"], x["seed"]) for x in rows)


def test_seed_leak_guard_is_independent_of_the_seed_constant(monkeypatch):
    """Defence in depth: even if TUNING_SEEDS were wrongly edited to overlap the evaluation
    seeds, the plan would still be refused."""
    monkeypatch.setattr(tu, "TUNING_SEEDS", (1, 102, 103, 104, 105))
    p = _raw_plan()
    p["seeds"] = [1, 102, 103, 104, 105]
    with pytest.raises(tu.TuningPlanError, match="leak"):
        tu.validate_plan(p)


# ================================================================ official campaign (Stage 14 step G)
def test_approved_lock_is_smart_t1():
    lock = tu.load_lock()
    assert lock["lock_sha256"] == "3b3da0427f7f965f23a1c586a782fbeb3be8445608dc2eaf4135f5512e3ca84c"
    assert lock["smart_params"] == replace(SmartParams(), tau_s=15.0, w_max_s=90.0)
    assert lock["tuning_plan_sha256"] == tu.plan_sha256()


def test_campaign_matrix_and_extras():
    from smarttraffic import campaign as cp
    lock = tu.load_lock()
    specs = cp.campaign_specs(lock)
    assert len(specs) == 450 + 10 + 30
    keys = {(s.scenario, s.label, s.seed) for s in specs}
    assert len(keys) == len(specs)
    nowmax = [s for s in specs if s.label == "Smart-T1-noWMAX"]
    noage = [s for s in specs if s.label == "Smart-T1-noAging"]
    assert {s.scenario for s in nowmax} == {"S10"} and len(nowmax) == 10
    assert {s.scenario for s in noage} == {"S1", "S6", "S9"} and len(noage) == 30
    assert all(s.smart_params == replace(lock["smart_params"], use_w_max=False) for s in nowmax)
    assert all(s.smart_params == replace(lock["smart_params"], use_aging=False) for s in noage)
    assert {s.seed for s in specs} == set(FINAL_SEEDS)
    bad = [ex.RunSpec("S2", 1, "Smart", "Smart-T1-noWMAX", replace(lock["smart_params"], use_w_max=False)),
           ex.RunSpec("S10", 1, "Smart", "Smart-T1-noWMAX", replace(lock["smart_params"], use_aging=False)),
           ex.RunSpec("S1", 101, "FT-A", "FT-A"),
           ex.RunSpec("S1", 1, "FT-TOD", "FT-TOD"),
           ex.RunSpec("S1", 1, "FT-A", "Smart-T1")]
    for b in bad:
        with pytest.raises(tu.LockError):
            tu.check_official([b], lock)


def test_official_refuses_wrong_lock_hash(tmp_path):
    from smarttraffic import campaign as cp
    with pytest.raises(tu.LockError, match="expected"):
        cp.run_official(tmp_path, "0" * 64, workers=1)
    with pytest.raises(tu.LockError, match="no locked"):
        cp.run_official(tmp_path, "0" * 64, workers=1, lock_path=tmp_path / "none.json")


def test_official_statistics_tables(tmp_path):
    from smarttraffic import campaign as cp
    specs = []
    for seed in (1, 2, 3):
        specs += [short("S6", seed, "FT-A", "FT-A"), short("S6", seed, "FT-B", "FT-B"),
                  short("S6", seed, "FT-TOD", "FT-TOD"), short("S6", seed, "Smart", "Smart-study"),
                  short("S6", seed, "Smart", "Smart-T1", replace(SmartParams(), tau_s=15.0))]
    recs = ex.run_campaign(specs, tmp_path / "o", workers=2, arrivals_dir=tmp_path / "a", log=lambda *a: None)
    cp.check_pairing(recs)
    agg = cp.aggregate_rows(recs)
    assert len(agg) == 5 * len(cp.METRICS)
    row = next(r for r in agg if r["label"] == "Smart-T1" and r["metric"] == "delay_mean_s")
    vals = [r["summary"]["delay_mean_s"] for r in recs if r["label"] == "Smart-T1"]
    assert row["n"] == 3 and row["mean"] == pytest.approx(sum(vals) / 3)
    assert row["ci_low"] < row["mean"] < row["ci_high"]
    pr = cp.paired_rows(recs)
    pairs = {(r["a"], r["b"]) for r in pr}
    assert pairs == {("FT-A", "Smart-T1"), ("FT-B", "Smart-T1"), ("Smart-study", "Smart-T1"), ("FT-B", "FT-TOD")}
    d = next(r for r in pr if (r["a"], r["b"], r["metric"]) == ("FT-A", "Smart-T1", "delay_mean_s"))
    a = {r["spec"]["seed"]: r["summary"]["delay_mean_s"] for r in recs if r["label"] == "FT-A"}
    b = {r["spec"]["seed"]: r["summary"]["delay_mean_s"] for r in recs if r["label"] == "Smart-T1"}
    assert d["mean_diff_b_minus_a"] == pytest.approx(sum(b[s] - a[s] for s in (1, 2, 3)) / 3)
    assert "unadjusted" in d["note"] or d["test_valid"] is False
    mv = cp.movement_rows(recs)
    assert len(mv) == 15 * 8
    # tables are independent of record order
    assert cp.aggregate_rows(list(reversed(recs))) == agg and cp.paired_rows(list(reversed(recs))) == pr
    recs[0]["metadata"]["arrivals_sha256"] = "x"
    with pytest.raises(RuntimeError, match="arrivals differ"):
        cp.check_pairing(recs)


# ================================================================ reproduction never rewrites the lock
def _tiny_plan(monkeypatch):
    plan = tu.load_plan()
    plan["windows"] = {"warmup_s": 20.0, "measure_s": 120.0, "flush_s": 120.0, "dt_ms": 100}
    plan["seeds"] = [101, 102]
    plan["grid"] = {"tau_s": (10.0, 20.0), "w_max_s": (90.0,)}
    monkeypatch.setattr(tu, "load_plan", lambda path=None: plan)
    return plan


def test_pc_reproduction_with_an_existing_approved_lock_verifies_it_and_never_overwrites_it(tmp_path, monkeypatch):
    """Regression (Stage 14 PC run): reproducing the tuning into ANOTHER output folder must verify
    the approved lock, not try to write a new one (the lock names its tuning campaign, so a lock
    rebuilt from a differently named folder had a different hash)."""
    from smarttraffic import run_experiments as rx
    _tiny_plan(monkeypatch)
    lock = tmp_path / "lock.json"
    first = rx.tune(tmp_path / "S1_T1", workers=1, lock_path=lock, arrivals_dir=tmp_path / "arr",
                    log=lambda *a: None)
    sha = first["manifest"]["lock_sha256"]
    original = lock.read_bytes()
    # 1. a reproduction in another folder without the approved SHA is refused BEFORE any run
    with pytest.raises(tu.LockError, match="never overwritten"):
        rx.tune(tmp_path / "S1_T1_pc", workers=1, lock_path=lock, arrivals_dir=tmp_path / "arr", log=lambda *a: None)
    assert not (tmp_path / "S1_T1_pc").exists() and lock.read_bytes() == original
    # 2. with the approved SHA it runs, verifies and leaves the lock byte-identical
    rep = rx.tune(tmp_path / "S1_T1_pc", workers=2, lock_path=lock, arrivals_dir=tmp_path / "arr",
                  verify_lock_sha=sha, log=lambda *a: None)
    assert rep["manifest"]["lock_mode"] == "verified (not written)" and rep["manifest"]["lock_sha256"] == sha
    assert lock.read_bytes() == original
    assert (tmp_path / "S1_T1_pc" / "candidates.csv").read_bytes() == (tmp_path / "S1_T1" / "candidates.csv").read_bytes()
    assert (tmp_path / "S1_T1_pc" / "fingerprints.csv").read_bytes() == (tmp_path / "S1_T1" / "fingerprints.csv").read_bytes()
    # 3. a wrong approved SHA is refused before any run
    with pytest.raises(tu.LockError, match="expected"):
        rx.tune(tmp_path / "x", workers=1, lock_path=lock, verify_lock_sha="0" * 64, log=lambda *a: None)
    assert lock.read_bytes() == original


def test_reproduction_that_selects_other_parameters_stops_and_keeps_the_lock(tmp_path, monkeypatch):
    from smarttraffic import run_experiments as rx
    _tiny_plan(monkeypatch)
    first = rx.tune(tmp_path / "ref", workers=1, lock_path=tmp_path / "ref_lock.json",
                    arrivals_dir=tmp_path / "arr", log=lambda *a: None)
    lock = tmp_path / "lock.json"
    # an "approved" lock from the same candidate table but with parameters that were not selected
    sha = tu.write_lock(lock, replace(SmartParams(), tau_s=30.0), tu.plan_sha256(),
                        first["manifest"]["candidates_csv_sha256"], {"label": "tau30_wmax90"})
    original = lock.read_bytes()
    with pytest.raises(SystemExit, match="REPRODUCTION MISMATCH: selected"):
        rx.tune(tmp_path / "rep", workers=1, lock_path=lock, arrivals_dir=tmp_path / "arr", verify_lock_sha=sha,
                log=lambda *a: None)
    assert lock.read_bytes() == original


def test_sim_fingerprint_ignores_last_bit_float_differences():
    rec = {"result": {"metadata": {"seed": 1}, "end_ms": 5, "ended_early": True, "vehicles": [[1, "NS", 3]],
                      "signal_log": [[0, "NS", "GREEN"]], "queue_samples": [[0, [1], [0]]],
                      "intersection": {"jain_fairness": 0.9006683388966531}}}
    other = json.loads(json.dumps(rec))
    other["result"]["intersection"]["jain_fairness"] = 0.900668338896653        # seen on Windows vs Linux
    assert ex.sim_fingerprint(rec) == ex.sim_fingerprint(other)
    other["result"]["vehicles"][0][2] = 4                                       # a real simulation change
    assert ex.sim_fingerprint(rec) != ex.sim_fingerprint(other)


def _copy_reference(dst):
    import shutil
    src = ROOT / "results" / "tuning" / "S1_T1"
    dst.mkdir()
    for f in ("candidates.csv", "runs.csv", "selection.json", "fingerprints.csv"):
        shutil.copy(src / f, dst / f)
    man = json.loads((src / "manifest.json").read_text())
    man.update(lock_mode="verified (not written)")
    (dst / "manifest.json").write_text(json.dumps(man))
    return src


def test_verify_tuning_accepts_platform_float_noise_and_rejects_real_differences(tmp_path):
    from smarttraffic import run_experiments as rx
    sha = tu.load_lock()["lock_sha256"]
    ref = _copy_reference(tmp_path / "new")
    new = tmp_path / "new"
    assert rx.verify_tuning(ref, new, sha, log=lambda *a: None)
    # only full-result hashes differ (last-bit floats): still OK
    runs = (new / "runs.csv").read_text().splitlines()
    head = runs[0].split(",")
    i = head.index("result_sha256")
    rows = [r.split(",") for r in runs[1:]]
    for r in rows:
        r[i] = "f" * 64
    (new / "runs.csv").write_text("\n".join([runs[0]] + [",".join(r) for r in rows]) + "\n")
    sel = json.loads((new / "selection.json").read_text())
    sel["paired_vs_study"]["t"] *= (1 + 1e-15)
    (new / "selection.json").write_text(json.dumps(sel))
    assert rx.verify_tuning(ref, new, sha, log=lambda *a: None)
    # a real metric difference fails
    j = head.index("delay_mean_s")
    rows[0][j] = str(float(rows[0][j]) + 0.01)
    (new / "runs.csv").write_text("\n".join([runs[0]] + [",".join(r) for r in rows]) + "\n")
    assert not rx.verify_tuning(ref, new, sha, log=lambda *a: None)
    # a trajectory difference fails
    _copy_reference(tmp_path / "new2")
    fp = (tmp_path / "new2" / "fingerprints.csv").read_text().splitlines()
    fp[1] = fp[1][:-1] + ("0" if fp[1][-1] != "0" else "1")
    (tmp_path / "new2" / "fingerprints.csv").write_text("\n".join(fp) + "\n")
    assert not rx.verify_tuning(ref, tmp_path / "new2", sha, log=lambda *a: None)
    # a manifest that wrote (rather than verified) a lock fails
    _copy_reference(tmp_path / "new3")
    man = json.loads((tmp_path / "new3" / "manifest.json").read_text())
    man["lock_mode"] = "written"
    (tmp_path / "new3" / "manifest.json").write_text(json.dumps(man))
    assert not rx.verify_tuning(ref, tmp_path / "new3", sha, log=lambda *a: None)
