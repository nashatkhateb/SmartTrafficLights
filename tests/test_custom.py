"""Stage 14b: Custom / Demo mode, and the project-wide vehicle policy.

What is checked:
* custom demand: validation (numeric, finite, >= 0, < 2400; UI cap 1800), S1 demands
  reproduce S1's arrivals, zero and heavy demand run, results are tagged CUSTOM / DEMO;
* separation: official mode refuses custom demand; campaigns refuse custom output folders;
  custom output refuses official folders and never writes arrival files;
* vehicle policy: STANDARD (Stop on Yellow + immediate first vehicle) is the default for
  official, custom and UI runs; the explicit legacy-study compatibility policy reproduces the
  historical Stage 14 official baseline (golden fingerprints) exactly; exact, strict yellow
  boundary (no entry at or after yellow onset) vs the legacy inclusive + 2.0 s window;
* immediate first vehicle: first car starts at green onset, following headway unchanged,
  cars start one after another, exact even when GREEN begins inside a step;
* visualization: on/off equivalence for custom runs; panel, scenario cycling, badge; the UI
  never offers or runs the legacy policy.
"""
import csv
from dataclasses import replace
import json
import math
import os
from pathlib import Path

import pytest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from smarttraffic import custom as cu
from smarttraffic import experiments as ex
from smarttraffic import tuning as tu
from smarttraffic.conflicts import MOVES
from smarttraffic.controller_smart import SmartParams
from smarttraffic.custom import CustomDemand, CustomInputError, CustomSpec
from smarttraffic.runner import RunConfig, _load_arrivals, arrivals_sha256, run
from smarttraffic.signals import FLASHING_GREEN_MS, RED_YELLOW_MS, SignalBank, SignalState
from smarttraffic.traffic_gen import ARRIVALS_DIR, Arrival, generate_arrivals
from smarttraffic.vehicles import LEGACY_STUDY, STANDARD, VehicleModel

ROOT = Path(__file__).resolve().parent.parent
GOLDEN = ROOT / "tests" / "golden" / "official_T1_baseline.csv"
S1 = {"NS": 400, "SS": 400, "ES": 400, "WS": 400, "NL": 120, "SL": 120, "EL": 120, "WL": 120}


def spec(demand=None, **kw):
    kw.setdefault("measure_s", 60)
    return CustomSpec(CustomDemand.from_mapping(demand or S1), **kw)


# ================================================================ input validation
@pytest.mark.parametrize("bad", ["abc", "", float("nan"), float("inf"), -float("inf"), -1, -0.5, 2400, 2400.0,
                                 2500, 1e9, True, None, [400]])
def test_invalid_demand_values_are_rejected(bad):
    with pytest.raises(CustomInputError):
        cu.validate_demand(bad, "NS")


@pytest.mark.parametrize("good,expect", [(0, 0.0), (0.0, 0.0), (1, 1.0), ("400", 400.0), (1800, 1800.0),
                                         (2399.9, 2399.9)])
def test_valid_demand_values(good, expect):
    assert cu.validate_demand(good, "NS") == expect


def test_demand_needs_all_eight_movements_and_the_ui_caps_at_1800():
    with pytest.raises(CustomInputError, match="missing"):
        CustomDemand.from_mapping({m: 100 for m in MOVES if m != "WL"})
    with pytest.raises(CustomInputError, match="unknown"):
        CustomDemand.from_mapping({**S1, "NR": 10})
    assert cu.UI_LIMIT_VEH_H == 1800 and cu.HARD_LIMIT_VEH_H == 2400
    assert cu.ui_clamp(2500) == 1800 and cu.ui_clamp(-5) == 0 and cu.ui_clamp(700) == 700
    from smarttraffic.viz_custom import CustomDraft
    d = CustomDraft()
    d.set_demand("NS", 2500)
    assert d.demand["NS"] == 1800 and "capped" in d.note
    d.focus_field("SS")
    for ch in "12a.5.x":                       # letters and a second '.' are ignored
        d.type_char(ch)
    assert d.text == "12.5"
    d.focus_field(None)
    assert d.demand["SS"] == 12.5
    for _ in range(3):
        d.step_demand("NL", -1)                # never below 0
    d.set_demand("NL", 0)
    d.step_demand("NL", -1)
    assert d.demand["NL"] == 0


@pytest.mark.parametrize("kw,match", [
    (dict(controller="FT-B"), "FT-A"), (dict(controller="FT-TOD"), "FT-A"), (dict(seed=-1), "seed"),
    (dict(seed=1.5), "seed"), (dict(measure_s=59), "measurement"), (dict(measure_s=7201), "measurement"),
    (dict(measure_s=600.5), "measurement"), (dict(param_set="S2"), "parameter set"),
    (dict(param_set="T1", smart_overrides=(("tau_s", 12.0),)), "custom"),
    (dict(param_set="custom", smart_overrides=(("tau_s", 0.5),)), "tau_s"),
    (dict(param_set="custom", smart_overrides=(("min_green_straight_s", 5.0),)), "may be changed"),
    (dict(param_set="custom", smart_overrides=(("max_green_left_s", "x"),)), "max_green_left_s"),
])
def test_invalid_custom_specs_are_rejected(kw, match):
    with pytest.raises(CustomInputError, match=match):
        spec(**kw)


def test_custom_spec_has_no_per_run_vehicle_options():
    for old in ("yellow_policy", "immediate_first_vehicle"):
        with pytest.raises(TypeError):
            spec(**{old: "stop"})
    assert not hasattr(cu, "YELLOW_POLICY_SECONDS") and not hasattr(cu, "yellow_policy_name")
    assert spec().config().vehicle_policy == "standard"


def test_parameter_sets():
    lock = tu.load_lock(tu.LOCK_FILE)
    assert spec(param_set="T1").smart_params() == lock["smart_params"]
    assert spec(param_set="study").smart_params() == SmartParams()
    p = spec(param_set="custom", smart_overrides=(("max_green_straight_s", 60.0), ("tau_s", 12.0))).smart_params()
    assert p == replace(lock["smart_params"], tau_s=12.0, max_green_straight_s=60.0)
    assert spec(controller="FT-A").to_dict().get("smart_params") is None


def test_saturation_warning_levels_match_the_official_scenarios():
    assert cu.saturation_report(CustomDemand.from_mapping(S1)).level == "ok"
    near = cu.saturation_report(CustomDemand.uniform(550, 160))            # S7a demands
    assert near.level == "near" and "Near capacity" in near.warning
    over = cu.saturation_report(CustomDemand.uniform(700, 220))            # S7b demands
    assert over.level == "over" and over.degree > 1.2 and "Oversaturated" in over.warning
    lane = cu.saturation_report(CustomDemand.from_mapping({**S1, "NS": 2000}))
    assert lane.level == "over" and lane.movements_over == ("NS",)
    assert cu.saturation_report(CustomDemand.uniform(0, 0)).level == "ok"


# ================================================================ arrivals
@pytest.mark.parametrize("seed", [1, 7, 101])
def test_custom_s1_demand_reproduces_s1_arrivals(seed):
    c = spec(seed=seed, measure_s=3600).config()
    a_custom = _load_arrivals(c, None, None)
    a_s1 = generate_arrivals("S1", seed, -300, 3600)
    assert [(a.id, a.time_ms, a.movement, a.seq, a.source) for a in a_custom] == \
           [(a.id, a.time_ms, a.movement, a.seq, a.source) for a in a_s1.arrivals]
    assert arrivals_sha256(a_custom) == arrivals_sha256(a_s1)


def test_custom_arrivals_are_never_written_to_the_arrival_directory(tmp_path, monkeypatch):
    before = sorted(p.name for p in ARRIVALS_DIR.glob("*")) if ARRIVALS_DIR.exists() else []
    import smarttraffic.traffic_gen as tg
    monkeypatch.setattr(tg, "save_arrivals", lambda *a, **k: pytest.fail("custom run wrote an arrival file"))
    res, path = cu.run_custom(spec(measure_s=60), save=False)
    assert path is None and res.metadata["arrivals_count"] > 0
    after = sorted(p.name for p in ARRIVALS_DIR.glob("*")) if ARRIVALS_DIR.exists() else []
    assert before == after


# ================================================================ custom runs
def test_zero_demand_custom_run():
    for ctrl in ("FT-A", "Smart"):
        res, _ = cu.run_custom(spec(cu.CustomDemand.uniform(0, 0).as_dict(), controller=ctrl), save=False)
        ir = res.intersection
        assert ir.arrivals == ir.crossed == ir.unfinished == 0 and ir.delay_mean_s is None
        assert ir.safety_violations == 0 and res.metadata["tag"] == cu.CUSTOM_TAG
        assert res.metadata["arrivals_count"] == 0 and res.ended_early


def test_heavy_custom_run():
    heavy = {m: (1800 if m[1] == "S" else 600) for m in MOVES}
    s = spec(heavy, controller="Smart", measure_s=120)
    assert s.saturation().level == "over"
    for ctrl in ("FT-A", "Smart"):
        res, _ = cu.run_custom(replace(s, controller=ctrl), save=False)
        ir = res.intersection
        assert ir.safety_violations == 0 and ir.arrivals > 100 and ir.unfinished > 0
        assert ex.check_signal_log(res.signal_log, s.smart_params()) == []
        assert res.metadata["mode"] == "custom"


def test_custom_metadata_records_the_standard_vehicle_policy(tmp_path):
    s = spec(controller="FT-A", seed=3)
    res, path = cu.run_custom(s, out_dir=tmp_path)
    m = res.metadata
    assert m["mode"] == "custom" and m["tag"] == "CUSTOM / DEMO"
    assert (m["vehicle_policy"], m["stop_on_yellow"], m["immediate_first_vehicle"]) == ("standard", True, True)
    assert not {"yellow_policy", "yellow_extension_s"} & set(m)
    assert m["demand_veh_h"] == {k: float(v) for k, v in S1.items()} and m["scenario"] == "CUSTOM"
    doc = json.loads(path.read_text(encoding="utf-8"))
    assert doc["tag"] == "CUSTOM / DEMO" and doc["official"] is False and doc["mode"] == "custom"
    assert doc["spec"]["vehicle_policy"] == "standard" and doc["spec"]["stop_on_yellow"] is True
    assert doc["result"]["metadata"]["vehicle_policy"] == "standard"
    assert path.parent == tmp_path and path.name.startswith("custom_")
    s2 = spec(controller="Smart", param_set="custom", smart_overrides=(("tau_s", 12.0),))
    res2, path2 = cu.run_custom(s2, out_dir=tmp_path)
    d2 = json.loads(path2.read_text(encoding="utf-8"))
    assert d2["spec"]["vehicle_policy"] == "standard" and d2["spec"]["smart_param_set"] == "custom"
    assert d2["spec"]["smart_params"]["tau_s"] == 12.0 and res2.metadata["controller_params"]["tau_s"] == 12.0
    # official metadata: the vehicle policy, none of the custom keys
    cfg = RunConfig("S4", 1, "FT-A", warmup_s=0, measure_s=60, flush_s=60)
    off = run(cfg, arrivals=generate_arrivals("S4", 1, 0, 60)).metadata
    assert off["vehicle_policy"] == "standard" and not {"mode", "tag", "demand_veh_h"} & set(off)
    # legacy-study metadata is exactly Stage 14's: no policy or custom keys at all
    leg = run(replace(cfg, vehicle_policy="legacy-study"), arrivals=generate_arrivals("S4", 1, 0, 60)).metadata
    assert not {"mode", "tag", "vehicle_policy", "stop_on_yellow", "immediate_first_vehicle", "yellow_policy",
                "yellow_extension_s", "demand_veh_h"} & set(leg)


def test_command_line(tmp_path):
    import subprocess, sys
    base = [sys.executable, "-m", "smarttraffic.custom", "--demand",
            "NS=300,SS=300,ES=300,WS=300,NL=90,SL=90,EL=90,WL=90", "--measure", "60"]
    r = subprocess.run(base + ["--controller", "FT-A", "--out", str(tmp_path)],
                       cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stderr
    assert "CUSTOM / DEMO" in r.stdout and "wrote" in r.stdout
    (f,) = list(tmp_path.glob("custom_*.json"))
    doc = json.loads(f.read_text(encoding="utf-8"))
    assert doc["tag"] == "CUSTOM / DEMO" and doc["spec"]["vehicle_policy"] == "standard"
    for gone in (["--yellow", "study"], ["--immediate-first"]):          # no per-run vehicle options
        r = subprocess.run(base + gone + ["--out", str(tmp_path)], cwd=ROOT, capture_output=True, text=True,
                           timeout=300)
        assert r.returncode == 2 and "unrecognized" in r.stderr
    r = subprocess.run(base + ["--out", str(ROOT / "results" / "official" / "x")], cwd=ROOT,
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 2 and "refused" in r.stdout and "Traceback" not in r.stderr
    assert not (ROOT / "results" / "official" / "x").exists()
    r = subprocess.run([sys.executable, "-m", "smarttraffic.custom", "--demand",
                        "NS=2400,SS=0,ES=0,WS=0,NL=0,SL=0,EL=0,WL=0"], cwd=ROOT,
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 2 and "invalid input" in r.stdout


# ================================================================ separation from the official experiments
def test_official_mode_rejects_custom_inputs():
    d = CustomDemand.from_mapping(S1)
    with pytest.raises(ValueError, match="custom demand"):
        RunConfig("S1", 1, "Smart", demand=d)
    with pytest.raises(ValueError, match="standard"):
        RunConfig("CUSTOM", 1, "Smart", mode="custom", demand=d, vehicle_policy="legacy-study")
    with pytest.raises(ValueError, match="mode"):
        RunConfig("S1", 1, "Smart", mode="demo")
    with pytest.raises(ValueError, match="needs demand"):
        RunConfig("CUSTOM", 1, "Smart", mode="custom")
    with pytest.raises(ValueError, match="scenario"):
        RunConfig("S1", 1, "Smart", mode="custom", demand=d)
    for ctrl in ("FT-B", "FT-TOD"):
        with pytest.raises(ValueError, match="FT-A"):
            RunConfig("CUSTOM", 1, ctrl, mode="custom", demand=d)
    # the official experiment path cannot express a custom run at all
    assert "mode" not in ex.RunSpec.__dataclass_fields__ and "demand" not in ex.RunSpec.__dataclass_fields__
    assert ex.RunSpec("S1", 1, "Smart", "Smart-T1").config().mode == "official"
    lock = tu.load_lock(tu.LOCK_FILE)
    with pytest.raises(tu.LockError):
        tu.check_official([ex.RunSpec("S2", 1, "Smart", "Smart-T1", replace(SmartParams(), tau_s=12.0))], lock)


def test_official_mode_defaults_to_standard_and_legacy_is_explicit_only():
    assert RunConfig("S1", 1, "FT-A").vehicle_policy == "standard"
    assert ex.RunSpec("S1", 1, "FT-A", "FT-A").vehicle_policy == "standard"
    assert ex.RunSpec("S1", 1, "FT-A", "FT-A").config().policy is STANDARD
    assert RunConfig("S1", 1, "FT-A", vehicle_policy="legacy-study").policy is LEGACY_STUDY
    with pytest.raises(TypeError):
        RunConfig("S1", 1, "FT-A", vehicle=None)                  # the old per-run options are gone
    with pytest.raises(ValueError):
        ex.RunSpec("S1", 1, "FT-A", "FT-A", vehicle_policy="study")


def test_custom_mode_never_contaminates_official_output_folders(tmp_path):
    s = spec(controller="FT-A")
    for bad in (ROOT / "results" / "official", ROOT / "results" / "official" / "campaign_T1",
                ROOT / "results" / "tuning" / "S1_T1", ROOT / "data" / "arrivals", ROOT / "config"):
        with pytest.raises(CustomInputError, match="official"):
            cu.run_custom(s, out_dir=bad)                         # refused before running
        with pytest.raises(CustomInputError):
            cu.check_output_dir(bad)
    res, _ = cu.run_custom(s, save=False)
    off = run(RunConfig("S4", 1, "FT-A", warmup_s=0, measure_s=60, flush_s=60),
              arrivals=generate_arrivals("S4", 1, 0, 60))
    with pytest.raises(CustomInputError, match="CUSTOM / DEMO"):
        cu.save_custom(s, off, tmp_path)                          # an official result is not saved as custom
    assert cu.check_output_dir(tmp_path / "x") == (tmp_path / "x").resolve()
    # and the official campaign refuses the custom folder
    with pytest.raises(ValueError, match="Custom/Demo"):
        ex.run_campaign([ex.RunSpec("S4", 1, "FT-A", "FT-A")], cu.CUSTOM_RESULTS_DIR / "x")
    assert cu.CUSTOM_RESULTS_DIR == ROOT / "results" / "custom"
    gi = (ROOT / ".gitignore").read_text(encoding="utf-8")
    assert "results/custom/" in gi


# ================================================================ yellow policy
def _golden():
    with open(GOLDEN, newline="", encoding="utf-8") as fh:
        return {(r["scenario"], r["label"], int(r["seed"])): r for r in csv.DictReader(fh)}


def test_golden_baseline_covers_the_whole_official_campaign():
    g = _golden()
    assert len(g) == 490
    lock = tu.load_lock(tu.LOCK_FILE)
    from smarttraffic import campaign as cp
    assert {(s.scenario, s.label, s.seed) for s in cp.campaign_specs(lock)} == set(g)


def test_legacy_study_policy_reproduces_the_historical_stage14_official_outputs(tmp_path):
    """The explicit legacy-study policy (permissive first 2.0 s of yellow, study start-up lost
    time) gives the historical Stage 14 official campaign's simulated trajectories exactly
    (golden fingerprints recorded from the approved PC run), with the same run keys."""
    g = _golden()
    lock = tu.load_lock(tu.LOCK_FILE)
    from smarttraffic import campaign as cp
    specs = [s for s in cp.campaign_specs(lock, "legacy-study") if s.scenario == "S4" and s.seed == 1]
    assert {s.label for s in specs} == {"FT-A", "FT-B", "Smart-T1", "Smart-study"}
    arr = ex.prepare_arrivals(specs, tmp_path)
    for s in specs:
        assert s.config().policy is LEGACY_STUDY and "vehicle_policy" not in s.identity()
        path, sha, _ = arr[(s.scenario, s.seed, -s.warmup_s, s.measure_s)]
        rec = ex.execute_spec(s, path, sha)
        ref = g[(s.scenario, s.label, s.seed)]
        assert rec["error"] is None and rec["key"] == ref["run_key"]
        assert rec["metadata"]["arrivals_sha256"] == ref["arrivals_sha256"]
        assert ex.sim_fingerprint(rec) == ref["sim_sha256"], s.label


def test_standard_policy_runs_never_collide_with_the_historical_records(tmp_path):
    g = _golden()
    lock = tu.load_lock(tu.LOCK_FILE)
    from smarttraffic import campaign as cp
    std = cp.campaign_specs(lock)                                    # the default: standard
    assert all(s.vehicle_policy == "standard" and s.identity()["vehicle_policy"] == "standard" for s in std)
    arr = ex.prepare_arrivals([s for s in std if s.scenario == "S4" and s.seed == 1], tmp_path)
    keys = {r["run_key"] for r in g.values()}
    for s in std:
        if (s.scenario, s.seed) == ("S4", 1):
            path, sha, _ = arr[(s.scenario, s.seed, -s.warmup_s, s.measure_s)]
            assert ex.run_key(s, sha) not in keys
            rec = ex.execute_spec(s, path, sha)
            assert rec["error"] is None and rec["summary"]["safety_violations"] == 0
            assert ex.sim_fingerprint(rec) != g[(s.scenario, s.label, s.seed)]["sim_sha256"]


def test_custom_mode_with_default_options_simulates_exactly_like_official():
    c_off = RunConfig("S1", 2, "Smart", measure_s=600.0, smart_params=tu.load_lock(tu.LOCK_FILE)["smart_params"])
    c_cus = spec(seed=2, measure_s=600).config()
    a, b = run(c_off, arrivals=generate_arrivals("S1", 2, -300, 600)), run(c_cus)
    assert a.vehicles == b.vehicles and a.signal_log == b.signal_log and a.queue_samples == b.queue_samples
    assert a.intersection == b.intersection


def _single_car_run(policy, offset_ms: int):
    """One NS car whose own stop-line arrival is exactly (yellow onset + offset_ms)."""
    t_end = 20_000                                     # end_green -> flashing green -> yellow
    t_y = t_end + FLASHING_GREEN_MS
    arrive = t_y + offset_ms - 10_800
    vm = VehicleModel([Arrival(0, arrive, "NS", 0, "random")], start=0, policy=policy)
    bank = SignalBank(start_time=0)
    bank.start_green("NS")
    t = 0
    while t < 40_000:
        if t == t_end:
            bank.end_green("NS")
        vm.advance(0.1, bank)
        bank.advance(0.1)
        t += 100
    v = vm.vehicle(0)
    return v.cross_ms, t_y


@pytest.mark.parametrize("policy,offset,crosses", [
    (LEGACY_STUDY, 2000, True), (LEGACY_STUDY, 2001, False),       # legacy: inclusive at yellow onset + 2.0 s
    (STANDARD, -1, True), (STANDARD, 0, False), (STANDARD, 1, False), (STANDARD, 500, False),
    (STANDARD, 2000, False)])                                       # standard: strictly before yellow onset
def test_exact_yellow_boundary(policy, offset, crosses):
    cross, t_y = _single_car_run(policy, offset)
    if crosses:
        assert cross == t_y + offset
    else:
        assert cross is None                           # stopped at the line; no new entry


@pytest.mark.parametrize("policy,d_offset,crosses", [(STANDARD, -1, True), (STANDARD, 0, False),
                                                     (LEGACY_STUDY, 2000, True), (LEGACY_STUDY, 2001, False)])
def test_exact_yellow_boundary_when_the_crossing_time_is_granted_after_the_window_is_known(policy, d_offset, crosses):
    """GREEN at 100 s is ended at once (flashing green 100-103 s, yellow at 103 s), so the car's
    crossing time is first granted when the window end is already known."""
    t_y = 103_000
    sched = t_y + d_offset - 10_800
    vm = VehicleModel([Arrival(0, sched, "SS", 0, "random")], start=0, policy=policy)
    bank = SignalBank(start_time=0)
    t = 0
    while t < 130_000:
        if t == 99_000:
            bank.start_green("SS")
        if t == 100_000:
            bank.end_green("SS")
        vm.advance(0.1, bank)
        bank.advance(0.1)
        t += 100
    assert vm.vehicle(0).cross_ms == (t_y + d_offset if crosses else None)


def test_stop_on_yellow_prevents_new_entry_after_yellow_begins():
    """Queued straight traffic, green then yellow: under the standard policy nobody crosses at
    or after yellow onset; the legacy policy still lets cars cross during the first 2.0 s
    (inclusive) and never later."""
    arrivals = [Arrival(i, i * 1_500, "NS", i, "random") for i in range(30)]
    out = {}
    for pol in (STANDARD, LEGACY_STUDY):
        vm = VehicleModel(arrivals, start=0, policy=pol)
        bank = SignalBank(start_time=0)
        t, yellow = 0, None
        while t < 80_000:
            if t == 20_000:
                bank.start_green("NS")
            if t == 40_000:
                bank.end_green("NS")
            if yellow is None and bank.state("NS") is SignalState.YELLOW:
                yellow = t
            vm.advance(0.1, bank)
            bank.advance(0.1)
            t += 100
        assert yellow == 40_000 + FLASHING_GREEN_MS
        out[pol.name] = ([v.cross_ms for v in vm.crossed("NS")], yellow)
    (std, y), (leg, _) = out["standard"], out["legacy-study"]
    assert std and all(c < y for c in std)
    assert all(c <= y + 2000 for c in leg) and any(y < c <= y + 2000 for c in leg)


# ================================================================ immediate first vehicle
def _queue_discharge(policy, move="NS", n=10):
    arrivals = [Arrival(i, i * 2_000, move, i, "random") for i in range(n)]
    vm = VehicleModel(arrivals, start=0, policy=policy)
    bank = SignalBank(start_time=0)
    t, t_g, starts, last = 0, None, {}, {}
    while t < 100_000:
        if t == 50_000:
            bank.start_green(move)
            t_g = t + RED_YELLOW_MS
        vm.advance(0.1, bank)
        bank.advance(0.1)
        t += 100
        for v in list(vm.vehicles(move)) + list(vm.crossed(move)):
            x = v.position
            if t_g is not None and t > t_g and v.id in last and last[v.id] is not None and x is not None \
                    and x < last[v.id] - 1e-9 and v.id not in starts:
                starts[v.id] = t
            last[v.id] = x
    return [v.cross_ms - t_g for v in vm.crossed(move)], {k: s - t_g for k, s in starts.items()}


@pytest.mark.parametrize("move,h", [("NS", 1900), ("WL", 2000)])
def test_immediate_first_vehicle_starts_at_green_and_keeps_the_headway(move, h):
    study, _ = _queue_discharge(LEGACY_STUDY, move)
    imm, starts = _queue_discharge(STANDARD, move)
    assert study[0] == 2000 + h                               # legacy: t_g + l1 + h
    assert imm[0] == 36                                       # 0.5 m at 50 km/h: moves at green onset
    for seq in (study, imm):
        assert [b - a for a, b in zip(seq, seq[1:])] == [h] * (len(seq) - 1)
    # not a simultaneous release: each car starts after the one ahead of it
    ordered = [starts[k] for k in sorted(starts)]
    assert ordered == sorted(ordered) and len(set(ordered)) == len(ordered)
    assert ordered[0] <= 100 and ordered[1] >= 1000


def test_immediate_first_vehicle_is_exact_when_green_begins_inside_a_step():
    # a car queued at slot 1; one 1.5 s step contains the GREEN onset at 13.0 s
    vm = VehicleModel([Arrival(0, 0, "NS", 0, "random")], start=0)
    bank = SignalBank(start_time=0)
    for _ in range(12):
        vm.advance(1.0, bank)
        bank.advance(1.0)
    assert vm.vehicle(0).position == 0.5
    bank.start_green("NS")                                   # RED_YELLOW 12-13, GREEN at 13.0 s
    vm.advance(1.5, bank)
    assert vm.vehicle(0).cross_ms == 13_036


def test_standard_policy_is_safe_in_a_full_custom_run():
    s = spec(controller="Smart", measure_s=600)
    res, _ = cu.run_custom(s, save=False)
    assert res.intersection.safety_violations == 0
    assert ex.check_signal_log(res.signal_log, s.smart_params()) == []
    assert res.metadata["vehicle_policy"] == "standard"


# ================================================================ visualization
def _cfg_small(**kw):
    kw.setdefault("measure_s", 60)
    return spec(**kw).config()


@pytest.mark.parametrize("ctrl", ["FT-A", "Smart"])
def test_custom_visualization_on_off_equivalence(ctrl):
    from smarttraffic.viz_session import VizSession
    c = _cfg_small(controller=ctrl)
    s = VizSession(c, speed=None, running=True).start()
    assert s.wait_finished(120) and s.error is None
    assert s.result.to_json() == run(c).to_json()
    assert s.latest().config.mode == "custom"


def _state(config):
    from smarttraffic.viz import UIState
    from smarttraffic.viz_session import VizSession
    def factory(cc, **k):                     # official arrivals in memory: nothing written to data/
        a = None if cc.mode == "custom" else generate_arrivals(cc.scenario, cc.seed, -cc.warmup_s, cc.measure_s)
        return VizSession(cc, arrivals=a, **k)
    return UIState(config, session_factory=factory)


def test_ui_custom_panel_apply_cancel_and_scenario_cycling():
    st = _state(RunConfig("S10", 1, "Smart", warmup_s=20, measure_s=60, flush_s=60))
    try:
        assert not st.is_custom and not st.custom_open
        st.next_scenario()                                    # S10 -> Custom...: no spec yet -> panel
        assert st.custom_open and st.config.scenario == "S10"
        st.cancel_custom()
        assert not st.custom_open and st.config.scenario == "S10"
        st.open_custom()
        d = st.custom_draft
        d.set_demand("NS", 2500)                              # capped by the interface
        d.set_controller("FT-A")
        d.set_measure(300)
        assert st.apply_custom()
        c = st.config
        assert c.mode == "custom" and c.scenario == "CUSTOM" and c.controller == "FT-A"
        assert c.demand.rate("NS") == 1800 and c.measure_s == 300.0
        assert c.vehicle_policy == "standard" and not st.custom_open
        assert st.session.config == c
        st.set_controller("Smart")
        assert st.config.controller == "Smart" and st.config.mode == "custom"
        st.set_controller("FT-B")                             # not offered in Custom mode
        assert st.config.controller == "Smart"
        st.next_seed()
        assert st.config.seed == 2 and st.custom_spec.seed == 2
        st.next_scenario()                                    # Custom -> S1: a clean official config
        assert st.config.scenario == "S1" and st.config.mode == "official"
        assert st.config.vehicle_policy == "standard" and st.config.demand is None
        st.prev_scenario()                                    # back to Custom: last spec, no panel
        assert st.is_custom and not st.custom_open and st.config.vehicle_policy == "standard"
    finally:
        st.close()


pygame = pytest.importorskip("pygame")


@pytest.fixture(scope="module")
def screen():
    pygame.display.init()
    pygame.font.init()
    yield pygame.display.set_mode((1600, 900))
    pygame.display.quit()


def test_render_custom_badge_and_panel(screen):
    from smarttraffic.viz import Fonts, Renderer, _StillState, capture
    from smarttraffic.viz_custom import CustomDraft
    c = _cfg_small(controller="Smart")
    a, b = capture(c, [10_000])[10_000]
    r = Renderer(Fonts())
    surf = pygame.Surface((1600, 900))
    r.draw(surf, _StillState(c), a, b, a.t_ms)
    names = {n for _, _, n in r.buttons}
    assert {"FT-A", "Smart", "scenario+", "run"} <= names and "FT-B" not in names
    # the panel: only its own controls are clickable, advanced collapsed by default
    d = CustomDraft()
    r.draw(surf, _StillState(c, custom_draft=d), a, b, a.t_ms)
    names = {n for _, _, n in r.buttons}
    assert {"custom:apply", "custom:cancel", "advanced", "demand:NS+", "demand:WL-", "seed+",
            "controller:FT-A", "measure:3600", "preset:S1"} <= names
    assert "run" not in names and not any(n.startswith("yellow:") for n in names)
    d.toggle_advanced()
    r.draw(surf, _StillState(c, custom_draft=d), a, b, a.t_ms)
    names = {n for _, _, n in r.buttons}
    assert "params:custom" in names
    assert not any(n.startswith(("yellow:", "startup:")) for n in names)    # no per-run vehicle options
    d.set_param_set("custom")
    for size in ((1100, 680), (1366, 768), (1920, 1080)):
        s2 = pygame.Surface(size)
        r.draw(s2, _StillState(c, custom_draft=d), a, b, a.t_ms)
        assert {"smart:tau_s+", "custom:apply"} <= {n for _, _, n in r.buttons}
        apply_rect = next(rect for rect, _, n in r.buttons if n == "custom:apply")
        assert s2.get_rect().contains(apply_rect)


def test_app_keys_while_the_panel_is_open(screen):
    from smarttraffic.viz import App
    st = _state(RunConfig("S1", 1, "Smart", warmup_s=20, measure_s=60, flush_s=60))
    app = App(st, size=(1400, 820))
    try:
        st.open_custom()
        app.frame()
        buttons = {n: rect for rect, _, n in app.r.buttons}
        app.click(buttons["demand:NS+"].center)
        assert st.custom_draft.demand["NS"] == 450
        app.click(buttons["demand:SS:edit"].center)
        for ch in "650":
            app.custom_key(0, ch)
        app.custom_key(pygame.K_TAB, "")
        assert st.custom_draft.demand["SS"] == 650 and st.custom_draft.focus == "ES"
        assert app.custom_key(pygame.K_ESCAPE, "") and not st.custom_open   # Esc closes, never quits
        st.open_custom()
        app.custom_key(pygame.K_RETURN, "")
        assert st.is_custom and st.config.demand.rate("NS") == 400          # reopened: fresh draft
    finally:
        st.close()


# ================================================================ Smart in the UI = Smart-T1
def _lock_params():
    return tu.load_lock(tu.LOCK_FILE)["smart_params"]


def test_ui_official_smart_is_always_smart_t1():
    from smarttraffic.viz import official_config, ui_config
    t1 = _lock_params()
    assert t1.tau_s == 15.0 and t1.w_max_s == 90.0 and t1.max_green_straight_s == 45.0 and t1.max_green_left_s == 30.0
    assert t1 != SmartParams()                                       # Smart-study stays different
    assert official_config("S1", 1, "Smart").smart_params == t1
    base = RunConfig("S3", 2, "Smart")                               # study defaults in, T1 out
    assert ui_config(base) == replace(base, smart_params=t1)
    c = spec(controller="Smart", param_set="custom", smart_overrides=(("tau_s", 12.0),)).config()
    assert ui_config(c) is c                                         # custom keeps its own values
    st = _state(RunConfig("S1", 1, "Smart", warmup_s=20, measure_s=60, flush_s=60))
    try:
        assert st.config.smart_params == t1 and st.session.config.smart_params == t1
        st.set_controller("FT-A")
        st.set_controller("Smart")
        assert st.config.smart_params == t1
        st.next_scenario()
        st.next_seed()
        st.reset()
        assert st.config.scenario == "S2" and st.session.config.smart_params == t1
        st.open_custom()
        st.custom_draft.set_param_set("custom")
        st.custom_draft.set_smart("tau_s", 20)
        assert st.apply_custom() and st.config.smart_params.tau_s == 20.0
        st.prev_scenario()                                           # back to the study: T1 again
        assert st.config.mode == "official" and st.config.smart_params == t1
    finally:
        st.close()


def test_ui_smart_run_equals_headless_smart_t1_run():
    from smarttraffic.viz import UIState
    from smarttraffic.viz_session import VizSession
    c = RunConfig("S1", 3, "Smart", warmup_s=20, measure_s=90, flush_s=60)
    a = generate_arrivals("S1", 3, -20, 90)
    st = UIState(c, speed=None, session_factory=lambda cc, **k: VizSession(cc, arrivals=a, **k))
    try:
        st.toggle_run()
        assert st.session.wait_finished(120) and st.session.error is None
        ref = run(replace(c, smart_params=_lock_params()), arrivals=a)
        assert st.result.to_json() == ref.to_json()
        assert st.result.metadata["controller_params"]["tau_s"] == 15.0
        assert st.result.to_json() != run(c, arrivals=a).to_json()   # not Smart-study
    finally:
        st.close()


def test_viz_command_line_uses_smart_t1(monkeypatch, tmp_path):
    from smarttraffic import viz
    seen = {}
    monkeypatch.setattr(viz, "render_to_file", lambda path, config, *a, **k: seen.setdefault("c", config))
    assert viz.main(["--screenshot", str(tmp_path / "x.png"), "--scenario", "S5", "--controller", "Smart"]) == 0
    assert seen["c"].scenario == "S5" and seen["c"].smart_params == _lock_params()


def test_smart_study_is_not_a_user_facing_option(screen):
    from smarttraffic.viz import Fonts, Renderer, _StillState, capture
    from smarttraffic.viz_custom import CustomDraft
    from smarttraffic.viz_render import smart_label
    assert cu.UI_PARAM_SETS == ("T1", "custom")
    d = CustomDraft(advanced_open=True)
    with pytest.raises(ValueError):
        d.set_param_set("study")
    c = _cfg_small(controller="Smart")
    a, b = capture(c, [10_000])[10_000]
    r = Renderer(Fonts())
    r.draw(pygame.Surface((1600, 900)), _StillState(c, custom_draft=d), a, b, a.t_ms)
    names = {n for _, _, n in r.buttons}
    assert {"params:T1", "params:custom"} <= names and "params:study" not in names
    assert smart_label(_lock_params()) == "Smart-T1" and smart_label(SmartParams()) == "custom parameters"
    import subprocess, sys
    rr = subprocess.run([sys.executable, "-m", "smarttraffic.custom", "--demand",
                         "NS=1,SS=1,ES=1,WS=1,NL=1,SL=1,EL=1,WL=1", "--params", "study"],
                        cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert rr.returncode == 2 and "invalid choice" in rr.stderr


# ================================================================ the UI always runs the standard policy
def test_ui_always_runs_the_standard_vehicle_policy(monkeypatch, tmp_path):
    from smarttraffic import viz
    from smarttraffic.viz import official_config, ui_config
    assert official_config("S7a", 3, "FT-B").vehicle_policy == "standard"
    legacy = RunConfig("S1", 1, "FT-A", vehicle_policy="legacy-study")
    assert ui_config(legacy).vehicle_policy == "standard"               # never shown / run in the UI
    st = _state(replace(legacy, warmup_s=20, measure_s=60, flush_s=60))
    try:
        seen = [st.config.vehicle_policy, st.session.config.vehicle_policy]
        for act in (lambda: st.set_controller("FT-B"), lambda: st.set_controller("Smart"), st.next_scenario,
                    st.next_seed, st.prev_seed, st.reset, st.prev_scenario):
            act()
            seen += [st.config.vehicle_policy, st.session.config.vehicle_policy]
        assert set(seen) == {"standard"}
    finally:
        st.close()
    got = {}
    monkeypatch.setattr(viz, "render_to_file", lambda path, config, *a, **k: got.setdefault("c", config))
    assert viz.main(["--screenshot", str(tmp_path / "x.png"), "--scenario", "S2", "--controller", "FT-A"]) == 0
    assert got["c"].vehicle_policy == "standard"


def test_ready_banner_states_the_traffic_rules(screen):
    from smarttraffic.viz import Fonts, Renderer, _StillState, capture
    assert cu.VEHICLE_POLICY_LABEL == "Stop on Yellow · first queued car starts at green"
    c = RunConfig("S1", 1, "FT-A", warmup_s=20, measure_s=60, flush_s=60)
    a, b = capture(c, [0])[0]
    r = Renderer(Fonts())
    r.draw(pygame.Surface((1600, 900)), _StillState(c), a, b, a.t_ms)    # draws without error
