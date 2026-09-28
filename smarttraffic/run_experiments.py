"""run_experiments -- command line for the Stage 14 tuning campaign (and, later, the official one).

Spec: study §10, §16 steps 9-10; docs/ARCHITECTURE.md ("Experiments and tuning (Stage 14)").

    python -m smarttraffic.run_experiments tune [--policy P] [--workers N] [--out DIR]

`tune` runs the pre-registered plan (config/tuning_plan.json: S1, seeds 101-105, tau_s x w_max_s
grid, FT-A / FT-B references), resuming completed runs, applies the pre-registered selection
rule, verifies determinism of the selected candidate and Smart-study by re-running them, and
writes:
    <out>/runs/<key>.json.gz   full per-run records (not committed)
    <out>/runs.csv             one row per (label, seed)
    <out>/candidates.csv       ranked candidate table with reject reasons
    <out>/selection.json       winner, guardrails, comparisons
    <out>/manifest.json        plan hash, code fingerprint, commit, environment, CSV hashes
    config/smart_params_locked.json   the Smart-T1 lock (written once)

    python -m smarttraffic.run_experiments verify-tuning --new <dir> --lock-sha <sha>
    python -m smarttraffic.run_experiments official --lock-sha <sha> [--policy P] [--workers N]

`official` runs the locked campaign (campaign.py): S1-S10 x {FT-A, FT-B, Smart-T1, Smart-study},
FT-TOD on S6, and the approved extras, on seeds 1-10, and writes runs.csv, movements.csv,
aggregates.csv, paired.csv and manifest.json.

Vehicle policy (--policy, default "standard"): the standard policy writes to
results/tuning/S1_T1_standard and results/official/campaign_T1_standard. The historical Stage 14
results (results/tuning/S1_T1, results/official/campaign_T1) were produced with the
"legacy-study" policy; reproducing them requires --policy legacy-study explicitly (their folders
refuse standard runs). The approved Smart-T1 lock was selected under legacy-study, so
`tune --verify-lock` (reproduction) requires --policy legacy-study; Smart-T1 is not retuned.
"""
import argparse
from dataclasses import asdict
import datetime as _dt
import json
from pathlib import Path
import sys
import time

from smarttraffic import experiments as ex
from smarttraffic import tuning as tu
from smarttraffic.stats import paired_comparison

from smarttraffic.runner import DEFAULT_VEHICLE_POLICY
from smarttraffic.vehicles import LEGACY_STUDY, POLICIES

LEGACY_TUNING_OUT = tu.ROOT / "results" / "tuning" / "S1_T1"                  # Stage 14 (historical)
LEGACY_OFFICIAL_OUT = tu.ROOT / "results" / "official" / "campaign_T1"        # Stage 14 (historical)
DEFAULT_TUNING_OUT = tu.ROOT / "results" / "tuning" / "S1_T1_standard"
DEFAULT_OFFICIAL_OUT = tu.ROOT / "results" / "official" / "campaign_T1_standard"


def default_out(kind: str, policy: str) -> Path:
    legacy = policy == LEGACY_STUDY.name
    if kind == "tune":
        return LEGACY_TUNING_OUT if legacy else DEFAULT_TUNING_OUT
    return LEGACY_OFFICIAL_OUT if legacy else DEFAULT_OFFICIAL_OUT


def _now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _paired(result, label_a, label_b, records):
    """Paired (same seed) comparison of mean control delay: b - a."""
    by = {}
    for r in records:
        by.setdefault(r["label"], {})[r["spec"]["seed"]] = r["summary"]["delay_mean_s"]
    seeds = sorted(by[label_a])
    return paired_comparison([by[label_a][s] for s in seeds], [by[label_b][s] for s in seeds])


def tune(out_dir: Path, workers, plan_path=tu.PLAN_FILE, lock_path=tu.LOCK_FILE,
         arrivals_dir=None, verify=True, write_lock=True, verify_lock_sha=None, log=print,
         vehicle_policy: str = DEFAULT_VEHICLE_POLICY) -> dict:
    """Run (or resume) the tuning campaign.

    Lock handling -- the approved lock is immutable:
      * verify_lock_sha given  -> REPRODUCTION: the existing lock must load and have exactly this
        SHA; the reproduced selection must equal the locked parameters; nothing is written to the
        lock file (lock_mode "verified").
      * no lock file yet       -> first tuning: the selection is written as the lock ("written").
      * lock exists, no sha    -> refused before any run: an existing lock is never rewritten.
    """
    plan = tu.load_plan(plan_path)
    plan_sha = tu.plan_sha256(plan_path)
    lock_path = Path(lock_path)
    approved = None
    if verify_lock_sha is not None:
        approved = tu.load_lock(lock_path, expected_sha=verify_lock_sha)
        if approved["tuning_plan_sha256"] != plan_sha:
            raise tu.LockError("the approved lock was made from a different tuning plan")
        if tu.lock_policy(approved) != vehicle_policy:
            raise tu.LockError(f"the approved lock was selected under the {tu.lock_policy(approved)} vehicle "
                               f"policy; reproducing it requires --policy {tu.lock_policy(approved)} "
                               f"(Smart-T1 is not retuned under {vehicle_policy})")
    elif write_lock and lock_path.exists():
        raise tu.LockError(f"{lock_path} already exists and is never overwritten: to reproduce the "
                           f"tuning and verify it, pass the approved lock SHA (--verify-lock <sha>)")
    specs = tu.tuning_specs(plan, vehicle_policy)
    t0 = time.time()
    kw = {} if arrivals_dir is None else {"arrivals_dir": arrivals_dir}
    records = ex.run_campaign(specs, out_dir, workers=workers, log=log, **kw)
    elapsed = time.time() - t0
    result = tu.evaluate(plan, records)
    runs_sha = ex.write_csv(out_dir / "runs.csv", ex.summary_rows(records), ex.SUMMARY_FIELDS)
    cand_sha = ex.write_csv(out_dir / "candidates.csv", tu.candidate_table(result), tu.CANDIDATE_FIELDS)
    sel = result["selected"]
    if sel is None:
        raise SystemExit("no feasible candidate: nothing is locked (see candidates.csv)")
    # determinism: re-run the winner and Smart-study sequentially in this process
    det = {}
    if verify:
        by_key = {r["key"]: r for r in records}
        check = [s for s in specs if s.label in (sel["label"], tu.STUDY_LABEL)]
        arr = ex.prepare_arrivals(check, **kw)
        for s in check:
            path, sha, _ = arr[(s.scenario, s.seed, -s.warmup_s, s.measure_s)]
            again = ex.execute_spec(s, path, sha)
            det[f"{s.label} seed {s.seed}"] = again["result_sha256"] == by_key[again["key"]]["result_sha256"]
        if not all(det.values()):
            raise SystemExit(f"determinism check failed: {det}")
    study = result["study"]
    selection = {
        "selected_label": sel["label"], "tau_s": sel["tau_s"], "w_max_s": sel["w_max_s"],
        "delay_mean_s": sel["delay_mean_s"], "study_delay_mean_s": study["delay_mean_s"],
        "delay_p95_mean_s": sel["delay_p95_mean_s"], "study_delay_p95_mean_s": study["delay_p95_mean_s"],
        "delay_max_worst_s": sel["delay_max_worst_s"], "study_delay_max_worst_s": study["delay_max_worst_s"],
        "tie_band_labels": [r["label"] for r in result["candidates"] if r["in_tie_band"]],
        "feasible": sum(1 for r in result["candidates"] if r["feasible"]),
        "rejected": {r["label"]: r["reject_reasons"] for r in result["candidates"] if not r["feasible"]},
        "paired_vs_study": _paired(result, tu.STUDY_LABEL, sel["label"], records),
        "paired_vs_FT-A": _paired(result, "FT-A", sel["label"], records),
        "paired_vs_FT-B": _paired(result, "FT-B", sel["label"], records),
        "references": {k: {kk: v.get(kk) for kk in ("delay_mean_s", "delay_p95_mean_s", "delay_max_worst_s")}
                       for k, v in result["references"].items()},
        "arrivals_sha256": {str(k): v for k, v in result["arrivals_sha256"].items()},
        "determinism": det,
        "note": "tuning seeds 101-105 only; unadjusted paired t-tests (no multiple-comparison correction)",
    }
    sel_path = out_dir / "selection.json"
    sel_path.write_text(json.dumps(selection, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    fp_sha = ex.write_csv(out_dir / "fingerprints.csv", ex.fingerprint_rows(records), ex.FINGERPRINT_FIELDS)
    lock_sha, lock_mode = None, None
    if approved is not None:
        if sel["params"] != approved["smart_params"]:
            raise SystemExit(f"REPRODUCTION MISMATCH: selected {sel['label']} but the approved lock is "
                             f"{approved['selection'].get('label')}; the lock was not touched")
        if cand_sha != approved["candidates_csv_sha256"]:
            raise SystemExit("REPRODUCTION MISMATCH: candidates.csv differs from the one the lock was made "
                             "from; the lock was not touched")
        lock_sha, lock_mode = approved["lock_sha256"], "verified (not written)"
    elif write_lock:
        lock_sha = tu.write_lock(lock_path, sel["params"], plan_sha, cand_sha,
                                 {"label": sel["label"], "tau_s": sel["tau_s"], "w_max_s": sel["w_max_s"],
                                  "tuning_campaign": out_dir.name, "vehicle_policy": vehicle_policy})
        tu.load_lock(lock_path, expected_sha=lock_sha)
        lock_mode = "written"
    manifest = {"campaign": plan["name"], "kind": "tuning", "mode": "official-tuning",
                "vehicle_policy": vehicle_policy,
                "created_utc": _now(), "elapsed_s": round(elapsed, 1), "workers": workers,
                "tuning_plan_sha256": plan_sha, "source_sha256": ex.source_fingerprint(),
                "git": ex.git_commit(), "environment": ex.environment(), "engine": ex.ENGINE_VERSION,
                "runs": len(records), "distinct_runs": len({r["key"] for r in records}),
                "runs_csv_sha256": runs_sha, "candidates_csv_sha256": cand_sha,
                "fingerprints_csv_sha256": fp_sha, "lock_sha256": lock_sha, "lock_mode": lock_mode}
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                                           encoding="utf-8")
    return {"result": result, "selection": selection, "manifest": manifest, "records": records}


def _close(a, b, rel=1e-9) -> bool:
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_close(a[k], b[k], rel) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_close(x, y, rel) for x, y in zip(a, b))
    if isinstance(a, float) or isinstance(b, float):
        if a is None or b is None or isinstance(a, bool) or isinstance(b, bool):
            return a == b
        return abs(a - b) <= rel * max(1.0, abs(a), abs(b))
    return a == b


def verify_tuning(ref_dir: Path, new_dir: Path, lock_sha: str, lock_path=tu.LOCK_FILE, log=print) -> bool:
    """Compare a re-run tuning campaign (e.g. on another machine) with the reference one.

    Exact: the simulated trajectories (fingerprints.csv: every vehicle, signal change and queue
    sample, integer ms), arrival checksums, candidates.csv (ranking, reject reasons, rounded
    metrics), every runs.csv column except result_sha256, the selected parameters, and the lock.
    Tolerant (relative 1e-9): the floating-point statistics in selection.json. result_sha256 hashes
    the full RunResult JSON including derived floats whose last bit can differ between operating
    systems (C library pow); those differences are reported, not treated as failures."""
    import csv
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        log(f"  {'OK  ' if cond else 'FAIL'} {name}{(': ' + detail) if detail else ''}")
    lock = tu.load_lock(lock_path, expected_sha=lock_sha)
    p = lock["smart_params"]
    check("existing lock is the approved Smart-T1 (tau 15, W_MAX 90, all else study defaults)",
          p.tau_s == 15.0 and p.w_max_s == 90.0, lock["lock_sha256"])
    new_man = json.loads((new_dir / "manifest.json").read_text(encoding="utf-8"))
    check("reproduction verified the lock without writing it",
          new_man.get("lock_sha256") == lock_sha and new_man.get("lock_mode") == "verified (not written)")
    check("same tuning plan", new_man.get("tuning_plan_sha256") == lock["tuning_plan_sha256"])
    # a manifest without "vehicle_policy" was written before the field existed (Stage 14: legacy-study)
    new_policy = new_man.get("vehicle_policy", LEGACY_STUDY.name)
    check("reproduction used the vehicle policy the lock was selected under",
          new_policy == tu.lock_policy(lock), new_policy)
    ref_sel = json.loads((ref_dir / "selection.json").read_text(encoding="utf-8"))
    new_sel = json.loads((new_dir / "selection.json").read_text(encoding="utf-8"))
    check("selected candidate tau 15 / W_MAX 90", (new_sel["tau_s"], new_sel["w_max_s"]) == (15.0, 90.0) ==
          (ref_sel["tau_s"], ref_sel["w_max_s"]), new_sel["selected_label"])
    check("selection statistics equal (relative 1e-9)", _close(ref_sel, new_sel))
    ref_c = list(csv.DictReader(open(ref_dir / "candidates.csv", encoding="utf-8")))
    new_c = list(csv.DictReader(open(new_dir / "candidates.csv", encoding="utf-8")))
    check("candidate ranking identical (20 candidates)",
          [r["label"] for r in ref_c] == [r["label"] for r in new_c] and len(new_c) == 20)
    check("candidates.csv byte-identical", (ref_dir / "candidates.csv").read_bytes() ==
          (new_dir / "candidates.csv").read_bytes())
    ref_r = list(csv.DictReader(open(ref_dir / "runs.csv", encoding="utf-8")))
    new_r = list(csv.DictReader(open(new_dir / "runs.csv", encoding="utf-8")))
    strip = lambda rows: [{k: v for k, v in r.items() if k != "result_sha256"} for r in rows]
    check("runs.csv identical in every metric column (115 rows)", strip(ref_r) == strip(new_r) and len(new_r) == 115)
    diff_full = sum(1 for a, b in zip(ref_r, new_r) if a["result_sha256"] != b["result_sha256"])
    log(f"  info full-result JSON hashes differing only in last-bit derived floats: {diff_full} of {len(new_r)}")
    ref_f = list(csv.DictReader(open(ref_dir / "fingerprints.csv", encoding="utf-8")))
    new_f = list(csv.DictReader(open(new_dir / "fingerprints.csv", encoding="utf-8")))
    check("fingerprints.csv byte-identical", (ref_dir / "fingerprints.csv").read_bytes() ==
          (new_dir / "fingerprints.csv").read_bytes())
    for seed in ("101", "102", "103", "104", "105"):
        a = {(r["label"], r["sim_sha256"], r["arrivals_sha256"]) for r in ref_f if r["seed"] == seed}
        b = {(r["label"], r["sim_sha256"], r["arrivals_sha256"]) for r in new_f if r["seed"] == seed}
        shas = {r["arrivals_sha256"] for r in new_f if r["seed"] == seed}
        check(f"seed {seed}: all 23 simulated trajectories identical, one arrival checksum",
              a == b and len(a) == 23 and len(shas) == 1)
    check("safety violations 0", all(r["safety_violations"] == "0" for r in new_r))
    check("no errors / signal-log problems / unfinished",
          all(not r["error"] and r["signal_check_problems"] == "0" and r["unfinished"] == "0" for r in new_r))
    log("REPRODUCTION OK" if ok else "REPRODUCTION FAILED")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m smarttraffic.run_experiments")
    sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("tune", help="run the pre-registered S1 tuning campaign and lock Smart-T1")
    t.add_argument("--workers", type=int, default=None, help=f"default {ex.default_workers()} on this machine")
    pol = dict(choices=sorted(POLICIES), default=DEFAULT_VEHICLE_POLICY,
               help="vehicle policy (default standard; legacy-study reproduces Stage 14)")
    t.add_argument("--policy", **pol)
    t.add_argument("--out", default=None, help="default depends on --policy")
    t.add_argument("--no-verify", action="store_true", help="skip the determinism re-runs")
    t.add_argument("--lock", default=str(tu.LOCK_FILE), help="lock file (written only if it does not exist)")
    t.add_argument("--verify-lock", default=None, metavar="SHA",
                   help="reproduction: verify the existing approved lock (never written) instead of creating one")
    v = sub.add_parser("verify-tuning", help="compare a re-run tuning campaign with the reference")
    v.add_argument("--ref", default=str(LEGACY_TUNING_OUT))
    v.add_argument("--new", required=True)
    v.add_argument("--lock-sha", required=True)
    o = sub.add_parser("official", help="run the locked official campaign (seeds 1-10)")
    o.add_argument("--lock-sha", required=True, help="the approved Smart-T1 lock_sha256")
    o.add_argument("--workers", type=int, default=None)
    o.add_argument("--policy", **pol)
    o.add_argument("--out", default=None, help="default depends on --policy")
    args = ap.parse_args(argv)
    if args.cmd == "verify-tuning":
        return 0 if verify_tuning(Path(args.ref), Path(args.new), args.lock_sha) else 1
    if args.cmd == "official":
        from smarttraffic.campaign import run_official
        w = args.workers if args.workers is not None else ex.default_workers()
        out = run_official(Path(args.out or default_out("official", args.policy)), args.lock_sha, workers=w,
                           vehicle_policy=args.policy)
        m = out["manifest"]
        print(f"official campaign: {m['runs']} runs ({m['distinct_runs']} distinct), {m['elapsed_s']} s, "
              f"safety violations {m['safety_violations_total']}, signal-log problems {m['signal_log_problems_total']}")
        return 0
    if args.cmd == "tune":
        w = args.workers if args.workers is not None else ex.default_workers()
        out = tune(Path(args.out or default_out("tune", args.policy)), w, verify=not args.no_verify,
                   lock_path=Path(args.lock), verify_lock_sha=args.verify_lock, vehicle_policy=args.policy)
        s = out["selection"]
        print(f"selected {s['selected_label']}: tau = {s['tau_s']:g} s, W_MAX = {s['w_max_s']:g} s, "
              f"mean delay {s['delay_mean_s']:.2f} s (Smart-study {s['study_delay_mean_s']:.2f} s)")
        print(f"lock: {out['manifest']['lock_sha256']} ({out['manifest']['lock_mode']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
