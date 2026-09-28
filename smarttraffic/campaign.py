"""campaign -- the locked official experiment campaign and its statistics (Stage 14).

Spec: study §10 (seeds 1-10, paired comparisons, report every scenario), §11 (metrics, mean
± 95 % CI), §12 (G7 / G12 extras); the approved Stage 14 design.

Matrix (tuning.official_specs + tuning.extra_specs, checked by tuning.check_official):
    S1-S10 (11 IDs) x {FT-A, FT-B, Smart-T1 (locked), Smart-study} x seeds 1-10, FT-TOD on S6;
    extras: Smart-T1 without W_MAX on S10 (G7), Smart-T1 without aging on S1, S6, S9 (G12).
Every run of one scenario and seed reads the same arrival file (checked per run and again
here). Smart-T1 always uses the lock whose hash the caller states; nothing is tuned here.

Outputs (<out>/): runs.csv (one row per run), movements.csv (one row per run and movement),
aggregates.csv (per scenario x label x metric: n, mean, sd, 95 % t-interval), paired.csv
(same-seed differences B - A with 95 % CI and paired t-test; UNADJUSTED for multiple
comparisons), manifest.json. Full records stay in <out>/runs/ (not committed).
"""
from dataclasses import asdict
import datetime as _dt
import json
from pathlib import Path
import time
from typing import Dict, List, Mapping, Optional, Sequence, Union

from smarttraffic import experiments as ex
from smarttraffic import tuning as tu
from smarttraffic.conflicts import MOVES
from smarttraffic.stats import mean_ci95, paired_comparison

__all__ = ["METRICS", "PAIRS", "campaign_specs", "aggregate_rows", "paired_rows", "movement_rows",
           "run_official", "AGG_FIELDS", "PAIR_FIELDS", "MOVEMENT_FIELDS"]

# (column, description) of every reported intersection metric
METRICS = (
    ("delay_mean_s", "mean control delay (s/veh)"),
    ("delay_p95_s", "95th-percentile control delay (s)"),
    ("delay_max_s", "maximum control delay (s)"),
    ("queue_mean_per_movement_veh", "mean queue, mean of the 8 movement means (veh)"),
    ("queue_max_movement_veh", "maximum queue of any movement (veh)"),
    ("queue_total_mean_veh", "mean intersection queue (veh)"),
    ("queue_total_max_veh", "maximum intersection queue (veh)"),
    ("throughput_veh_h", "throughput in the measurement hour (veh/h)"),
    ("pct_served", "% of measured arrivals served within the hour"),
    ("delay_total_veh_h", "total control delay (veh·h)"),
    ("green_utilization_mean", "green utilization, mean over movements with green time"),
    ("signal_changes_per_hour", "green onsets per hour"),
    ("vehicles_control_delay_over_90s", "vehicles with control delay > 90 s"),
    ("jain_fairness", "Jain's index over movement mean delays"),
    ("unfinished", "measured vehicles not served by the end of the flush"),
    ("safety_violations", "conflicting active signals (must be 0)"),
    ("waiting_mean_s", "mean waiting time (s)"),
    ("delay_mean_lower_bound_s", "mean delay incl. unfinished at their end-of-run delay (s)"),
    ("detector_call_wait_max_s", "diagnostic: longest detector-call wait (s)"),
    ("detector_calls_over_120s", "diagnostic: detector calls waiting over 120 s"),
)
# (A, B): report B - A on the same seeds
PAIRS = (("FT-A", "Smart-T1", None), ("FT-B", "Smart-T1", None), ("Smart-study", "Smart-T1", None),
         ("FT-B", "FT-TOD", ("S6",)),
         ("Smart-T1", "Smart-T1-noWMAX", ("S10",)), ("Smart-T1", "Smart-T1-noAging", ("S1", "S6", "S9")))

AGG_FIELDS = ["scenario", "label", "metric", "n", "n_missing", "mean", "sd", "half_width", "ci_low",
              "ci_high", "note"]
PAIR_FIELDS = ["scenario", "a", "b", "metric", "n", "mean_a", "mean_b", "mean_diff_b_minus_a", "sd_diff",
               "ci_low", "ci_high", "t", "p_unadjusted", "test_valid", "note"]
MOVEMENT_FIELDS = ["scenario", "label", "seed", "movement"] + [
    "arrivals", "crossed", "unfinished", "served_in_hour", "pct_served", "throughput_veh_h", "delay_mean_s",
    "delay_p95_s", "delay_max_s", "delay_total_veh_h", "delay_mean_lower_bound_s", "waiting_mean_s",
    "vehicles_control_delay_over_90s", "queue_mean_veh", "queue_max_veh", "queue_road_mean_veh",
    "queue_outside_mean_veh", "green_s", "green_onsets", "green_utilization"]


def campaign_specs(lock: Mapping) -> List[ex.RunSpec]:
    specs = tu.official_specs(lock) + tu.extra_specs(lock)
    tu.check_official(specs, lock)
    return specs


# ================================================================ statistics
def _metric_values(rec) -> Dict[str, Optional[float]]:
    s = rec["summary"]
    out = {k: s.get(k) for k, _ in METRICS if k in s}
    utils = [m["green_utilization"] for m in rec["movements"] if m.get("green_utilization") is not None]
    out["green_utilization_mean"] = sum(utils) / len(utils) if utils else None
    d = rec.get("diagnostics") or {}
    cw, co = d.get("detector_call_wait_max_s"), d.get("detector_calls_over_120s")
    out["detector_call_wait_max_s"] = max(cw.values()) if cw else None
    out["detector_calls_over_120s"] = sum(co.values()) if co else None
    return out


def _by(records):
    by: Dict[tuple, Dict[int, Dict[str, Optional[float]]]] = {}
    for r in records:
        if r.get("error"):
            raise RuntimeError(f"run {r['spec']['scenario']} {r['label']} seed {r['spec']['seed']} failed: "
                               f"{r['error']}")
        by.setdefault((r["spec"]["scenario"], r["label"]), {})[r["spec"]["seed"]] = _metric_values(r)
    return by


def _order(sc):
    return tu.OFFICIAL_SCENARIOS.index(sc) if sc in tu.OFFICIAL_SCENARIOS else 99


LABEL_ORDER = ("FT-A", "FT-B", "FT-TOD", "Smart-study", "Smart-T1", "Smart-T1-noWMAX", "Smart-T1-noAging")


def _lorder(label):
    return LABEL_ORDER.index(label) if label in LABEL_ORDER else 99


def aggregate_rows(records) -> List[Dict[str, object]]:
    rows = []
    for (sc, label), seeds in sorted(_by(records).items(), key=lambda kv: (_order(kv[0][0]), _lorder(kv[0][1]))):
        for metric, _ in METRICS:
            vals = [seeds[s].get(metric) for s in sorted(seeds)]
            have = [v for v in vals if v is not None]
            row = {"scenario": sc, "label": label, "metric": metric, "n": len(have),
                   "n_missing": len(vals) - len(have)}
            if have:
                c = mean_ci95(have)
                row.update(mean=c["mean"], sd=c["sd"], half_width=c["half_width"], ci_low=c["low"],
                           ci_high=c["high"], note=c["note"])
            else:
                row["note"] = "no value (e.g. no measured vehicle)"
            rows.append(row)
    return rows


def paired_rows(records) -> List[Dict[str, object]]:
    by = _by(records)
    rows = []
    scenarios = sorted({sc for sc, _ in by}, key=_order)
    for sc in scenarios:
        for a, b, only in PAIRS:
            if only is not None and sc not in only:
                continue
            if (sc, a) not in by or (sc, b) not in by:
                continue
            A, B = by[(sc, a)], by[(sc, b)]
            if sorted(A) != sorted(B):
                raise RuntimeError(f"{sc}: {a} and {b} were run on different seeds")
            for metric, _ in METRICS:
                pairs = [(A[s].get(metric), B[s].get(metric)) for s in sorted(A)]
                pairs = [(x, y) for x, y in pairs if x is not None and y is not None]
                row = {"scenario": sc, "a": a, "b": b, "metric": metric, "n": len(pairs)}
                if pairs:
                    c = paired_comparison([x for x, _ in pairs], [y for _, y in pairs])
                    row.update(mean_a=sum(x for x, _ in pairs) / len(pairs),
                               mean_b=sum(y for _, y in pairs) / len(pairs),
                               mean_diff_b_minus_a=c["mean_diff"], sd_diff=c["sd_diff"], ci_low=c["low"],
                               ci_high=c["high"], t=c["t"], p_unadjusted=c["p"], test_valid=c["test_valid"],
                               note=c["note"] or "unadjusted for multiple comparisons")
                else:
                    row["note"] = "no paired values"
                rows.append(row)
    return rows


def movement_rows(records) -> List[Dict[str, object]]:
    rows = []
    for r in records:
        for m in r.get("movements") or []:
            rows.append({"scenario": r["spec"]["scenario"], "label": r["label"], "seed": r["spec"]["seed"], **m})
    rows.sort(key=lambda x: (_order(x["scenario"]), _lorder(x["label"]), x["seed"], MOVES.index(x["movement"])))
    return rows


def check_pairing(records) -> None:
    """Every run of one scenario and seed used the same arrival checksum."""
    sha: Dict[tuple, set] = {}
    for r in records:
        sha.setdefault((r["spec"]["scenario"], r["spec"]["seed"]), set()).add(r["metadata"]["arrivals_sha256"])
    bad = {k: v for k, v in sha.items() if len(v) != 1}
    if bad:
        raise RuntimeError(f"arrivals differ between controllers: {bad}")


# ================================================================ run
def run_official(out_dir: Union[str, Path], lock_sha: str, *, workers: int, lock_path=tu.LOCK_FILE,
                 arrivals_dir=None, specs: Optional[Sequence[ex.RunSpec]] = None, log=print) -> Dict:
    lock = tu.load_lock(lock_path, expected_sha=lock_sha)
    all_specs = campaign_specs(lock) if specs is None else list(specs)
    tu.check_official(all_specs, lock)
    out_dir = Path(out_dir)
    t0 = time.time()
    kw = {} if arrivals_dir is None else {"arrivals_dir": arrivals_dir}
    records = ex.run_campaign(all_specs, out_dir, workers=workers, log=log, **kw)
    elapsed = time.time() - t0
    check_pairing(records)
    for r in records:
        if r.get("error"):
            raise RuntimeError(f"{r['spec']['scenario']} {r['label']} seed {r['spec']['seed']}: {r['error']}")
    hashes = {
        "runs_csv_sha256": ex.write_csv(out_dir / "runs.csv", _official_summary(records), ex.SUMMARY_FIELDS),
        "movements_csv_sha256": ex.write_csv(out_dir / "movements.csv", movement_rows(records), MOVEMENT_FIELDS),
        "aggregates_csv_sha256": ex.write_csv(out_dir / "aggregates.csv", aggregate_rows(records), AGG_FIELDS),
        "paired_csv_sha256": ex.write_csv(out_dir / "paired.csv", paired_rows(records), PAIR_FIELDS),
    }
    problems = sum(len(r.get("signal_check") or []) for r in records)
    manifest = {"campaign": out_dir.name, "kind": "official", "mode": "official",
                "created_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                "elapsed_s": round(elapsed, 1), "workers": workers, "lock_sha256": lock["lock_sha256"],
                "smart_t1_params": asdict(lock["smart_params"]),
                "tuning_plan_sha256": lock["tuning_plan_sha256"], "source_sha256": ex.source_fingerprint(),
                "git": ex.git_commit(), "environment": ex.environment(), "engine": ex.ENGINE_VERSION,
                "runs": len(records), "distinct_runs": len({r["key"] for r in records}),
                "seeds": sorted({r["spec"]["seed"] for r in records}),
                "safety_violations_total": sum(r["summary"]["safety_violations"] for r in records),
                "signal_log_problems_total": problems,
                "multiple_comparisons": "none (unadjusted p-values, as the study specifies no correction)",
                **hashes}
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {"records": records, "manifest": manifest}


def _official_summary(records):
    rows = ex.summary_rows(records)
    rows.sort(key=lambda x: (_order(x["scenario"]), _lorder(x["label"]), x["seed"]))
    return rows
