"""Stage 15: result figures (smarttraffic.make_graphs).

The figure pipeline only reads committed, hash-checked summaries; it must build every figure,
cover every scenario and controller, plot only finite numbers, be deterministic, label the
vehicle policy of its data and never combine data of different vehicle policies."""
import csv
import json
import math
import shutil
from pathlib import Path

import pytest

from smarttraffic import make_graphs as mg

ROOT = Path(__file__).resolve().parent.parent
OUT = mg.FIGURES_ROOT / "campaign_T1__legacy-study"          # the committed figure set


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("fig") / "campaign_T1__legacy-study"
    shutil.copytree(OUT / "data", out / "data")                  # the committed S6 extract
    res = mg.build_all(out=out, log=lambda *a: None)
    return out, res


def _values(out):
    with open(out / "figures_values.csv", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def test_generation_completes_and_writes_every_file(built):
    out, res = built
    ids = [fid for fid, _, _ in mg.FIGURES]
    assert len(ids) == 15 and len(set(ids)) == 15
    for fid in ids:
        for ext in ("png", "pdf", "svg"):
            assert (out / f"{fid}.{ext}").stat().st_size > 10_000, (fid, ext)
    for f in ("figures_values.csv", "figures_manifest.json", "README.md"):
        assert (out / f).exists()
    man = res["manifest"]
    assert [f["id"] for f in man["figures"]] == ids
    readme = (out / "README.md").read_text(encoding="utf-8")
    assert all(fid in readme for fid in ids)


def test_required_scenarios_and_controllers_are_plotted(built):
    out, _ = built
    rows = _values(out)
    by = {}
    for r in rows:
        by.setdefault(r["figure"], set()).add((r["scenario"], r["label"]))
    need = {(sc, lb) for sc in mg.SCENARIOS for lb in mg.CORE}
    for fid in ("F01_mean_delay_by_scenario", "F02_p95_delay", "F03_max_delay", "F05_mean_queue",
                "F06_jain_fairness", "F07a_long_waits", "F08a_signal_changes", "F08b_green_utilization"):
        assert need <= by[fid], fid
    assert ("S6", "FT-TOD") in by["F01_mean_delay_by_scenario"]
    assert {(sc, "Smart-T1 - FT-A") for sc in mg.SCENARIOS} == by["F09_SmartT1_vs_FTA"]
    assert {(sc, "Smart-T1 - FT-B") for sc in mg.SCENARIOS} == by["F10_SmartT1_vs_FTB"]
    assert {lb for _, lb in by["F12_S6_time_varying_demand"]} == set(mg.S6_LABELS)
    assert {lb for _, lb in by["F07b_S10_starvation_WMAX"]} == set(mg.S10_LABELS)
    assert {sc for sc, _ in by["F13_key_scenarios_mean_delay"]} == set(mg.KEY_SCENARIOS)
    # Smart-study appears only in the explicitly historical comparison
    assert {r["figure"] for r in rows if "Smart-study" in r["label"]} == {"F11_SmartT1_vs_SmartStudy_historical"}


def test_plotted_values_are_finite_and_match_the_official_aggregates(built):
    out, _ = built
    rows = _values(out)
    assert len(rows) > 900
    for r in rows:
        for k in ("value", "ci_low", "ci_high"):
            if r[k] != "":
                assert math.isfinite(float(r[k])), r
    agg = {(a["scenario"], a["label"], a["metric"]): a for a in
           csv.DictReader(open(mg.CAMPAIGN_DIR / "aggregates.csv", encoding="utf-8", newline=""))}
    f01 = [r for r in rows if r["figure"] == "F01_mean_delay_by_scenario"]
    for r in f01:
        assert float(r["value"]) == pytest.approx(float(agg[(r["scenario"], r["label"], "delay_mean_s")]["mean"]))
    s4 = {r["label"]: float(r["value"]) for r in f01 if r["scenario"] == "S4"}
    assert s4["Smart-T1"] < s4["FT-B"] < s4["FT-A"]


def test_outputs_are_deterministic_and_carry_no_timestamps(built, tmp_path):
    out, res = built
    out2 = tmp_path / "again"
    shutil.copytree(OUT / "data", out2 / "data")
    res2 = mg.build_all(out=out2, log=lambda *a: None)
    for f in ("figures_values.csv", "figures_manifest.json", "README.md"):
        assert (out / f).read_bytes() == (out2 / f).read_bytes(), f
    for fid, _, _ in mg.FIGURES:                        # vector files are reproducible on one machine
        assert (out / f"{fid}.svg").read_bytes() == (out2 / f"{fid}.svg").read_bytes(), fid
        assert (out / f"{fid}.pdf").read_bytes() == (out2 / f"{fid}.pdf").read_bytes(), fid
    man = json.loads((out / "figures_manifest.json").read_text(encoding="utf-8"))
    assert not {"created_utc", "date", "timestamp"} & set(man)
    assert man["figures_values_csv_sha256"] == mg._sha(out / "figures_values.csv")
    for rel, h in man["inputs_sha256"].items():
        assert mg._sha((out if rel.startswith("data/") else ROOT) / rel) == h, rel
    assert res["manifest"]["inputs_sha256"] == res2["manifest"]["inputs_sha256"]


def test_committed_figure_set_is_current():
    """The committed results/figures set is what the code produces from the committed data."""
    man = json.loads((OUT / "figures_manifest.json").read_text(encoding="utf-8"))
    assert man["vehicle_policy"] == "legacy-study" and man["campaign"] == "campaign_T1"
    assert man["figures_version"] == mg.FIG_VERSION
    for rel, h in man["inputs_sha256"].items():
        assert mg._sha((OUT if rel.startswith("data/") else ROOT) / rel) == h, rel
    assert mg._sha(OUT / "figures_values.csv") == man["figures_values_csv_sha256"]


def test_every_figure_states_the_legacy_study_policy(built):
    out, res = built
    assert {r["vehicle_policy"] for r in _values(out)} == {"legacy-study"}
    assert all(f["vehicle_policy"] == "legacy-study" for f in res["manifest"]["figures"])
    readme = (out / "README.md").read_text(encoding="utf-8")
    assert "legacy-study" in readme and "not** standard-policy results" in readme
    from PIL import Image
    for fid, _, _ in mg.FIGURES:
        desc = Image.open(out / f"{fid}.png").text.get("Description", "")
        assert "Vehicle policy: legacy-study" in desc, fid
        assert "legacy-study" in (out / f"{fid}.svg").read_text(encoding="utf-8")
    assert mg.default_out(mg.load_campaign()).name == "campaign_T1__legacy-study"


def _campaign_copy(tmp_path, policy=None):
    dst = tmp_path / "campaign_T1"
    dst.mkdir()
    for f in ("manifest.json", "runs.csv", "aggregates.csv", "paired.csv"):
        shutil.copy(mg.CAMPAIGN_DIR / f, dst / f)
    if policy:
        man = json.loads((dst / "manifest.json").read_text(encoding="utf-8"))
        man["vehicle_policy"] = policy
        (dst / "manifest.json").write_text(json.dumps(man), encoding="utf-8")
    return dst


def test_figures_are_never_built_from_mixed_policy_data(tmp_path):
    std = _campaign_copy(tmp_path, "standard")
    ds = mg.load_campaign(std)
    assert ds.policy == "standard"
    out = tmp_path / "out"
    shutil.copytree(OUT / "data", out / "data")
    with pytest.raises(mg.PolicyMixError, match="tuning"):             # legacy tuning + standard campaign
        mg.build_all(std, out=out, log=lambda *a: None)
    with pytest.raises(mg.PolicyMixError, match="S6 extract"):           # legacy S6 extract, standard figures
        mg.load_s6(out, ds)
    (out / "figures_manifest.json").write_text(json.dumps({"campaign": "campaign_T1",
                                                           "vehicle_policy": "legacy-study"}))
    with pytest.raises(mg.PolicyMixError, match="own folder"):          # existing legacy figure folder
        mg._check_out(out, ds)
    marker = std / "vehicle_policy.json"
    marker.write_text(json.dumps({"vehicle_policy": "legacy-study"}))
    with pytest.raises(mg.PolicyMixError, match="marker"):
        mg.load_campaign(std)


def test_changed_inputs_are_refused(tmp_path):
    c = _campaign_copy(tmp_path)
    assert mg.load_campaign(c).policy == "legacy-study"                  # no field = Stage 14 legacy-study
    (c / "aggregates.csv").write_bytes((c / "aggregates.csv").read_bytes().replace(b"43.467454033", b"43.0"))
    with pytest.raises(mg.InputChangedError, match="aggregates.csv"):
        mg.load_campaign(c)
    out = tmp_path / "out"
    shutil.copytree(OUT / "data", out / "data")
    p = out / "data" / "S6_timeseries.csv"
    p.write_text(p.read_text(encoding="utf-8").replace("queue_total_veh,FT-A,0,30,10,", "queue_total_veh,FT-A,0,30,9,"),
                 encoding="utf-8")
    with pytest.raises(mg.InputChangedError):
        mg.load_s6(out, mg.load_campaign())


def test_s6_extract_provenance_matches_the_official_runs():
    prov = json.loads((OUT / "data" / "S6_timeseries.json").read_text(encoding="utf-8"))
    assert prov["vehicle_policy"] == "legacy-study" and prov["campaign"] == "campaign_T1"
    runs = {(r["label"], int(r["seed"])): r["run_key"] for r in
            csv.DictReader(open(mg.CAMPAIGN_DIR / "runs.csv", encoding="utf-8", newline=""))
            if r["scenario"] == "S6" and r["label"] in mg.S6_LABELS}
    assert len(runs) == 40 and len(prov["runs"]) == 40
    assert {k: v for k, v in prov["runs"].items()} == {f"{lb} seed {s}": k for (lb, s), k in runs.items()}
    assert mg._sha(OUT / "data" / "S6_timeseries.csv") == prov["csv_sha256"]


def test_only_the_official_run_is_accepted_for_the_s6_extract(tmp_path):
    """Re-simulating one official S6 run under legacy-study gives the golden trajectory; a changed
    record or the standard policy is refused."""
    from smarttraffic import campaign as cp
    from smarttraffic import experiments as ex
    from smarttraffic import tuning as tu
    ds = mg.load_campaign()
    lock = tu.load_lock(tu.LOCK_FILE)
    spec = next(s for s in cp.campaign_specs(lock, "legacy-study") if (s.scenario, s.label, s.seed) == ("S6", "FT-TOD", 1))
    key = next(r["run_key"] for r in ds.runs if (r["scenario"], r["label"], r["seed"]) == ("S6", "FT-TOD", "1"))
    arr = ex.prepare_arrivals([spec], tmp_path)
    path, sha, _ = arr[(spec.scenario, spec.seed, -spec.warmup_s, spec.measure_s)]
    rec = ex.execute_spec(spec, path, sha)
    mg.verify_record(ds, "S6", "FT-TOD", 1, rec, key)                   # accepted
    bad = json.loads(json.dumps(rec))
    bad["result"]["vehicles"][0]["cross_ms"] += 1
    with pytest.raises(mg.InputChangedError, match="trajectory"):
        mg.verify_record(ds, "S6", "FT-TOD", 1, bad, key)
    std = next(s for s in cp.campaign_specs(lock) if (s.scenario, s.label, s.seed) == ("S6", "FT-TOD", 1))
    rec_std = ex.execute_spec(std, path, sha)
    with pytest.raises(mg.InputChangedError, match="not the official run"):
        mg.verify_record(ds, "S6", "FT-TOD", 1, rec_std, key)


@pytest.mark.skipif(not (mg.CAMPAIGN_DIR / "runs").is_dir(), reason="per-run records are not on this machine")
def test_s6_extract_from_the_local_records_reproduces_the_committed_extract(tmp_path):
    ds = mg.load_campaign()
    mg.extract_s6(ds, tmp_path, log=lambda *a: None)
    assert (tmp_path / "data" / "S6_timeseries.csv").read_bytes() == (OUT / "data" / "S6_timeseries.csv").read_bytes()


def test_command_line(tmp_path):
    import subprocess
    import sys
    out = tmp_path / "cli"
    shutil.copytree(OUT / "data", out / "data")
    r = subprocess.run([sys.executable, "-m", "smarttraffic.make_graphs", "--out", str(out)], cwd=ROOT,
                       capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr
    assert "15 figures (legacy-study)" in r.stdout
    assert (out / "figures_values.csv").read_bytes() == (OUT / "figures_values.csv").read_bytes()
