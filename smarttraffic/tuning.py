"""tuning -- pre-registered Smart-controller tuning and the parameter lock (Stage 14).

Spec: study §10 ("Freeze every smart-controller parameter before running experiments. Tune
only on S1 with seeds 101-105, then run all scenarios on seeds 1-10"), §14, §15 ("τ, Q̂ and
W_MAX are our choices. Freeze them before testing"), §16 step 9; the approved Stage 14 design.

Workflow
--------
1. config/tuning_plan.json is written and frozen BEFORE any tuning run. load_plan() validates
   it (S1 only, seeds exactly 101-105, no evaluation seed, only tau_s and w_max_s varied) and
   plan_sha256() identifies it.
2. tuning_specs(plan): every grid candidate (Smart) and the references FT-A / FT-B on the
   tuning seeds. Smart-study = SmartParams() is the grid point (10, 90) and shares its runs.
3. evaluate(plan, records): per candidate the means over the 5 seeds, hard constraints and
   guardrails against Smart-study, then the deterministic ranking and the selection rule.
4. write_lock(): config/smart_params_locked.json = the selected SmartParams ("Smart-T1") with
   the plan hash, the candidate-table hash and its own SHA-256. load_lock() refuses a missing,
   edited or inconsistent lock; official_specs() refuses anything but the locked parameters.

Selection rule (approved):
  primary     minimise the mean control delay over the 5 tuning seeds;
  hard        safety violations 0, no simulation error (e.g. SignalBank refusal), no signal-log
              problem (illegal transition, wrong interval, green < minimum), unfinished 0,
              one arrival checksum per seed for every candidate and reference;
  guardrails  vs Smart-study: mean P95 delay must not increase; the worst maximum delay over
              the 5 seeds may increase by at most 10 %;
  ties        candidates within 1 % of the best feasible mean delay are equal; among them
              prefer fewer changed parameters, then the smallest relative distance to the study
              defaults, then the fixed grid order (tau ascending, W_MAX ascending).

Vehicle policy. Every spec builder takes `vehicle_policy` (default "standard"). The approved
Smart-T1 lock (config/smart_params_locked.json) was SELECTED under the legacy-study policy
(lock_policy(): a lock without "vehicle_policy" in its selection is legacy-study); its
parameters are kept unchanged under the standard policy (no retuning). Reproducing the Stage 14
tuning therefore requires the explicit legacy-study policy.
"""
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple, Union

from smarttraffic.controller_smart import SmartParams
from smarttraffic.experiments import RunSpec, _canonical, write_csv
from smarttraffic.runner import DEFAULT_VEHICLE_POLICY
from smarttraffic.vehicles import LEGACY_STUDY
from smarttraffic.traffic_gen import FINAL_SEEDS, TUNING_SEEDS, load_scenarios

__all__ = ["TuningPlanError", "LockError", "PLAN_FILE", "LOCK_FILE", "TUNED_PARAMS", "load_plan",
           "plan_sha256", "candidates", "candidate_label", "tuning_specs", "evaluate",
           "CANDIDATE_FIELDS", "write_lock", "load_lock", "official_specs", "OFFICIAL_SCENARIOS",
           "STUDY_LABEL", "TUNED_LABEL", "EXTRAS", "extra_specs", "check_official", "lock_policy"]

ROOT = Path(__file__).resolve().parent.parent
PLAN_FILE = ROOT / "config" / "tuning_plan.json"
LOCK_FILE = ROOT / "config" / "smart_params_locked.json"
TUNED_PARAMS = ("tau_s", "w_max_s")            # the only parameters tuning may change
STUDY_LABEL = "Smart-study"
TUNED_LABEL = "Smart-T1"
OFFICIAL_SCENARIOS = ("S1", "S2", "S3", "S4", "S5", "S6", "S7a", "S7b", "S8", "S9", "S10")


class TuningPlanError(ValueError):
    """The tuning plan violates the study's tuning rules or the approved design."""


class LockError(ValueError):
    """The locked Smart parameter file is missing, edited or inconsistent."""


# ================================================================ plan
def _num_list(v, what) -> Tuple[float, ...]:
    if not isinstance(v, list) or not v:
        raise TuningPlanError(f"{what} must be a non-empty list")
    out = []
    for x in v:
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0:
            raise TuningPlanError(f"{what}: bad value {x!r}")
        out.append(float(x))
    if len(set(out)) != len(out) or out != sorted(out):
        raise TuningPlanError(f"{what} must be strictly increasing without duplicates")
    return tuple(out)


def load_plan(path: Union[str, Path] = PLAN_FILE) -> Dict[str, object]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return validate_plan(raw)


def validate_plan(raw: Mapping) -> Dict[str, object]:
    need = {"name", "scenario", "seeds", "grid", "defaults", "references", "objective", "guardrails",
            "tie_band", "windows"}
    missing = need - set(raw)
    if missing:
        raise TuningPlanError(f"plan is missing {sorted(missing)}")
    if raw["scenario"] != "S1":
        raise TuningPlanError("the study tunes on S1 only")
    seeds = raw["seeds"]
    if not isinstance(seeds, list) or any(isinstance(s, bool) or not isinstance(s, int) for s in seeds):
        raise TuningPlanError("seeds must be a list of ints")
    if tuple(seeds) != tuple(TUNING_SEEDS):
        raise TuningPlanError(f"tuning seeds must be exactly {list(TUNING_SEEDS)}, got {seeds}")
    if set(seeds) & set(FINAL_SEEDS):
        raise TuningPlanError("an evaluation seed would leak into tuning")
    grid = raw["grid"]
    if set(grid) != set(TUNED_PARAMS):
        raise TuningPlanError(f"only {list(TUNED_PARAMS)} may be tuned, plan varies {sorted(grid)}")
    g = {k: _num_list(grid[k], f"grid.{k}") for k in TUNED_PARAMS}
    study = SmartParams()
    defaults = raw["defaults"]
    for k in TUNED_PARAMS:
        if float(defaults.get(k, -1)) != float(getattr(study, k)):
            raise TuningPlanError(f"defaults.{k} must be the study value {getattr(study, k)}")
        if float(getattr(study, k)) not in g[k]:
            raise TuningPlanError(f"the grid must contain the study default of {k}")
    if list(raw["references"]) != ["FT-A", "FT-B"]:
        raise TuningPlanError("references must be ['FT-A', 'FT-B']")
    if raw["objective"] != "min mean control delay (delay_mean_s) over the tuning seeds":
        raise TuningPlanError("objective must be the approved primary objective")
    gr = raw["guardrails"]
    if gr != {"p95_mean_max_increase": 0.0, "max_delay_worst_max_increase": 0.10}:
        raise TuningPlanError("guardrails must be the approved ones")
    if raw["tie_band"] != 0.01:
        raise TuningPlanError("tie band must be 0.01 (1 %)")
    if raw["windows"] != {"warmup_s": 300.0, "measure_s": 3600.0, "flush_s": 900.0, "dt_ms": 100}:
        raise TuningPlanError("tuning must use the study windows and dt")
    return {**dict(raw), "grid": g}


def plan_sha256(path: Union[str, Path] = PLAN_FILE) -> str:
    """SHA-256 of the plan file bytes (line ends normalised)."""
    return hashlib.sha256(Path(path).read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def candidate_label(tau: float, w_max: float) -> str:
    return f"tau{tau:g}_wmax{w_max:g}"


def candidates(plan) -> List[Tuple[str, SmartParams]]:
    """(label, params) in the fixed grid order: tau ascending, then W_MAX ascending."""
    out = []
    for tau in plan["grid"]["tau_s"]:
        for w in plan["grid"]["w_max_s"]:
            out.append((candidate_label(tau, w), replace(SmartParams(), tau_s=tau, w_max_s=w)))
    return out


def tuning_specs(plan, vehicle_policy: str = DEFAULT_VEHICLE_POLICY) -> List[RunSpec]:
    w = plan["windows"]
    kw = dict(warmup_s=w["warmup_s"], measure_s=w["measure_s"], flush_s=w["flush_s"], dt_ms=w["dt_ms"],
              vehicle_policy=vehicle_policy)
    specs = []
    for seed in plan["seeds"]:
        for ref in plan["references"]:
            specs.append(RunSpec(plan["scenario"], seed, ref, ref, **kw))
        specs.append(RunSpec(plan["scenario"], seed, "Smart", STUDY_LABEL, SmartParams(), **kw))
        for label, p in candidates(plan):
            specs.append(RunSpec(plan["scenario"], seed, "Smart", label, p, **kw))
    return specs


# ================================================================ evaluation
CANDIDATE_FIELDS = ["rank", "label", "tau_s", "w_max_s", "n_changed", "distance", "feasible",
                    "reject_reasons", "delay_mean_s", "delay_mean_seeds", "delta_vs_study_s",
                    "delay_p95_mean_s", "delay_max_worst_s", "straight_delay_s", "left_delay_s",
                    "queue_mean_per_movement_veh", "throughput_veh_h", "unfinished",
                    "vehicles_control_delay_over_90s", "jain_fairness", "signal_changes_per_hour",
                    "detector_call_wait_max_s", "detector_calls_over_120s", "holds",
                    "safety_violations", "signal_check_problems", "in_tie_band", "selected"]


def _mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs)


def _movement_mean(rec, kind):
    ms = [m for m in rec["movements"] if (m["movement"][1] == "S") == (kind == "straight")]
    vals = [m["delay_mean_s"] for m in ms if m["delay_mean_s"] is not None]
    return _mean(vals) if vals else None


def _aggregate(recs) -> Dict[str, object]:
    ok = [r for r in recs if r.get("error") is None]
    agg = {"errors": [r["error"] for r in recs if r.get("error")],
           "seeds": [r["spec"]["seed"] for r in recs]}
    if len(ok) != len(recs):
        return agg
    S = [r["summary"] for r in ok]
    diag = [r.get("diagnostics") or {} for r in ok]

    def dsum(key):
        vals = [d.get(key) for d in diag]
        return None if any(v is None for v in vals) else vals
    calls = dsum("detector_call_wait_max_s")
    over = dsum("detector_calls_over_120s")
    holds = [d.get("controller_hold_decisions") for d in diag]
    agg.update({
        "delay_mean_s": _mean(s["delay_mean_s"] for s in S),
        "delay_mean_seeds": [round(s["delay_mean_s"], 4) for s in S],
        "delay_p95_mean_s": _mean(s["delay_p95_s"] for s in S),
        "delay_max_worst_s": max(s["delay_max_s"] for s in S),
        "straight_delay_s": _mean(_movement_mean(r, "straight") for r in ok),
        "left_delay_s": _mean(_movement_mean(r, "left") for r in ok),
        "queue_mean_per_movement_veh": _mean(s["queue_mean_per_movement_veh"] for s in S),
        "throughput_veh_h": _mean(s["throughput_veh_h"] for s in S),
        "unfinished": sum(s["unfinished"] for s in S),
        "vehicles_control_delay_over_90s": _mean(s["vehicles_control_delay_over_90s"] for s in S),
        "jain_fairness": _mean(s["jain_fairness"] for s in S),
        "signal_changes_per_hour": _mean(s["signal_changes_per_hour"] for s in S),
        "safety_violations": sum(s["safety_violations"] for s in S),
        "signal_check_problems": sum(len(r.get("signal_check") or []) for r in ok),
        "detector_call_wait_max_s": max(max(c.values()) for c in calls) if calls else None,
        "detector_calls_over_120s": _mean(sum(o.values()) for o in over) if over else None,
        "holds": _mean(holds) if all(h is not None for h in holds) else None,
    })
    return agg


def evaluate(plan, records: Sequence[Mapping]) -> Dict[str, object]:
    """Apply the pre-registered rule to the tuning records. Pure and deterministic: the result
    does not depend on the order of `records`."""
    by_label: Dict[str, List[Mapping]] = {}
    for r in records:
        if r["spec"]["scenario"] != plan["scenario"]:
            raise TuningPlanError(f"record from {r['spec']['scenario']} in an S1 tuning evaluation")
        if r["spec"]["seed"] not in plan["seeds"]:
            raise TuningPlanError(f"record with seed {r['spec']['seed']} is not a tuning seed")
        by_label.setdefault(r["label"], []).append(r)
    for recs in by_label.values():
        recs.sort(key=lambda r: r["spec"]["seed"])
    # arrival checksums: one per seed across every candidate and reference
    per_seed: Dict[int, set] = {}
    for r in records:
        sha = (r.get("metadata") or {}).get("arrivals_sha256")
        per_seed.setdefault(r["spec"]["seed"], set()).add(sha)
    arrivals_ok = all(len(v) == 1 and None not in v for v in per_seed.values()) and \
        set(per_seed) == set(plan["seeds"])
    needed = [STUDY_LABEL] + list(plan["references"]) + [lab for lab, _ in candidates(plan)]
    for lab in needed:
        got = [r["spec"]["seed"] for r in by_label.get(lab, [])]
        if got != list(plan["seeds"]):
            raise TuningPlanError(f"{lab}: runs for seeds {got}, expected {list(plan['seeds'])}")
    study = _aggregate(by_label[STUDY_LABEL])
    if study.get("errors"):
        raise TuningPlanError(f"Smart-study failed: {study['errors']}")
    refs = {lab: _aggregate(by_label[lab]) for lab in plan["references"]}
    gr = plan["guardrails"]
    defaults = {k: float(plan["defaults"][k]) for k in TUNED_PARAMS}
    rows = []
    for order, (label, p) in enumerate(candidates(plan)):
        agg = _aggregate(by_label[label])
        reasons = []
        if agg.get("errors"):
            reasons.append("simulation error: " + "; ".join(agg["errors"]))
        else:
            if agg["safety_violations"]:
                reasons.append(f"safety violations {agg['safety_violations']}")
            if agg["signal_check_problems"]:
                reasons.append(f"signal-log problems {agg['signal_check_problems']}")
            if agg["unfinished"]:
                reasons.append(f"unfinished vehicles {agg['unfinished']}")
            if agg["delay_p95_mean_s"] > study["delay_p95_mean_s"] * (1 + gr["p95_mean_max_increase"]) + 1e-9:
                reasons.append(f"P95 guardrail: {agg['delay_p95_mean_s']:.3f} s > study "
                               f"{study['delay_p95_mean_s']:.3f} s")
            lim = study["delay_max_worst_s"] * (1 + gr["max_delay_worst_max_increase"])
            if agg["delay_max_worst_s"] > lim + 1e-9:
                reasons.append(f"max-delay guardrail: {agg['delay_max_worst_s']:.1f} s > "
                               f"{lim:.1f} s (study {study['delay_max_worst_s']:.1f} s + 10 %)")
        if not arrivals_ok:
            reasons.append("arrival checksum differs between runs of one seed")
        vals = {"tau_s": p.tau_s, "w_max_s": p.w_max_s}
        n_changed = sum(1 for k in TUNED_PARAMS if vals[k] != defaults[k])
        distance = sum(abs(vals[k] - defaults[k]) / defaults[k] for k in TUNED_PARAMS)
        row = {"label": label, "order": order, "tau_s": p.tau_s, "w_max_s": p.w_max_s,
               "n_changed": n_changed, "distance": round(distance, 12), "feasible": not reasons,
               "reject_reasons": " | ".join(reasons), "params": p}
        for k in CANDIDATE_FIELDS:
            if k in agg:
                row[k] = agg[k]
        if agg.get("delay_mean_s") is not None:
            row["delta_vs_study_s"] = agg["delay_mean_s"] - study["delay_mean_s"]
        rows.append(row)
    feasible = [r for r in rows if r["feasible"]]
    selected = None
    if feasible:
        best = min(r["delay_mean_s"] for r in feasible)
        band = best * (1 + plan["tie_band"])
        for r in rows:
            r["in_tie_band"] = r["feasible"] and r["delay_mean_s"] <= band + 1e-12
        tied = [r for r in rows if r["in_tie_band"]]
        selected = min(tied, key=lambda r: (r["n_changed"], r["distance"], r["order"]))
        selected["selected"] = True
    # rank: feasible by mean delay (ties: grid order), then infeasible by mean delay
    ordered = sorted(rows, key=lambda r: (not r["feasible"],
                                          r.get("delay_mean_s") if r.get("delay_mean_s") is not None else math.inf,
                                          r["order"]))
    for i, r in enumerate(ordered, 1):
        r["rank"] = i
        r.setdefault("in_tie_band", False)
        r.setdefault("selected", False)
    return {"study": study, "references": refs, "candidates": ordered, "selected": selected,
            "arrivals_ok": arrivals_ok, "arrivals_sha256": {s: sorted(v)[0] for s, v in sorted(per_seed.items())}}


def candidate_table(result) -> List[Dict[str, object]]:
    out = []
    for r in result["candidates"]:
        row = {k: r.get(k) for k in CANDIDATE_FIELDS}
        row["delay_mean_seeds"] = ";".join(f"{x:.4f}" for x in r.get("delay_mean_seeds", []))
        out.append(row)
    return out


# ================================================================ lock
def _lock_body(params: SmartParams, plan_sha: str, candidates_sha: str, selection: Mapping) -> Dict:
    return {"name": TUNED_LABEL, "controller": "Smart", "params": asdict(params),
            "tuned_parameters": list(TUNED_PARAMS),
            "study_defaults": {k: getattr(SmartParams(), k) for k in TUNED_PARAMS},
            "tuning_plan_sha256": plan_sha, "candidates_csv_sha256": candidates_sha,
            "selection": dict(selection)}


def write_lock(path: Union[str, Path], params: SmartParams, plan_sha: str, candidates_sha: str,
               selection: Mapping, *, overwrite: bool = False) -> str:
    """Write the lock once. Returns its lock_sha256. Refuses to overwrite a different lock."""
    body = _lock_body(params, plan_sha, candidates_sha, selection)
    sha = hashlib.sha256(_canonical(body).encode("utf-8")).hexdigest()
    p = Path(path)
    if p.exists() and not overwrite:
        old = load_lock(p)
        if old["lock_sha256"] != sha:
            raise LockError(f"{p} already holds a different lock ({old['lock_sha256']})")
        return sha
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({**body, "lock_sha256": sha}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return sha


def load_lock(path: Union[str, Path] = LOCK_FILE, *, expected_sha: Optional[str] = None) -> Dict[str, object]:
    """The validated lock: {..., 'smart_params': SmartParams}. Refuses missing / edited locks and
    locks that change anything but the tuned parameters."""
    p = Path(path)
    if not p.exists():
        raise LockError(f"no locked Smart parameters at {p}: run the tuning campaign first")
    try:
        raw = json.loads(p.read_text(encoding="utf-8"))
    except ValueError as e:
        raise LockError(f"{p} is not valid JSON: {e}")
    sha = raw.pop("lock_sha256", None)
    if sha != hashlib.sha256(_canonical(raw).encode("utf-8")).hexdigest():
        raise LockError(f"{p} was edited after locking (lock_sha256 does not match its content)")
    if expected_sha is not None and sha != expected_sha:
        raise LockError(f"{p} is not the expected lock {expected_sha}")
    if raw.get("name") != TUNED_LABEL or raw.get("controller") != "Smart":
        raise LockError("lock must be Smart-T1 for the Smart controller")
    try:
        params = SmartParams(**raw["params"])
    except (TypeError, ValueError) as e:
        raise LockError(f"invalid parameters in the lock: {e}")
    study = asdict(SmartParams())
    changed = [k for k, v in asdict(params).items() if v != study[k] and k not in TUNED_PARAMS]
    if changed:
        raise LockError(f"the lock changes parameters that may not be tuned: {changed}")
    return {**raw, "lock_sha256": sha, "smart_params": params}


def lock_policy(lock: Mapping) -> str:
    """The vehicle policy the lock's parameters were selected under (Stage 14: legacy-study)."""
    return (lock.get("selection") or {}).get("vehicle_policy", LEGACY_STUDY.name)


# ================================================================ official matrix (guarded)
def official_specs(lock: Mapping, seeds: Sequence[int] = FINAL_SEEDS,
                   scenarios: Sequence[str] = OFFICIAL_SCENARIOS,
                   vehicle_policy: str = DEFAULT_VEHICLE_POLICY) -> List[RunSpec]:
    """The approved official matrix: every scenario x {FT-A, FT-B, Smart-T1, Smart-study},
    plus FT-TOD for S6, on the evaluation seeds, study windows only. Smart-T1 always uses the
    locked parameters; there are no per-scenario or custom parameters."""
    if not isinstance(lock.get("smart_params"), SmartParams) or "lock_sha256" not in lock:
        raise LockError("official runs need a lock loaded with load_lock()")
    known = set(load_scenarios())
    for sc in scenarios:
        if sc not in OFFICIAL_SCENARIOS or sc not in known:
            raise LockError(f"{sc!r} is not an official study scenario")
    if tuple(seeds) != tuple(FINAL_SEEDS):
        raise LockError(f"official runs use seeds {list(FINAL_SEEDS)}")
    specs = []
    for sc in scenarios:
        for seed in seeds:
            vp = dict(vehicle_policy=vehicle_policy)
            specs.append(RunSpec(sc, seed, "FT-A", "FT-A", **vp))
            specs.append(RunSpec(sc, seed, "FT-B", "FT-B", **vp))
            specs.append(RunSpec(sc, seed, "Smart", TUNED_LABEL, lock["smart_params"], **vp))
            specs.append(RunSpec(sc, seed, "Smart", STUDY_LABEL, SmartParams(), **vp))
            if sc == "S6":
                specs.append(RunSpec(sc, seed, "FT-TOD", "FT-TOD", **vp))
    return specs


# approved study extras (run after the main matrix): (label, scenarios, switch turned off)
EXTRAS = (("Smart-T1-noWMAX", ("S10",), "use_w_max"),            # study G7
          ("Smart-T1-noAging", ("S1", "S6", "S9"), "use_aging"))    # study G12 ablation


def extra_specs(lock: Mapping, seeds: Sequence[int] = FINAL_SEEDS,
                vehicle_policy: str = DEFAULT_VEHICLE_POLICY) -> List[RunSpec]:
    """The approved study extras: Smart-T1 with exactly one §9 mechanism switched off."""
    if not isinstance(lock.get("smart_params"), SmartParams) or "lock_sha256" not in lock:
        raise LockError("official runs need a lock loaded with load_lock()")
    if tuple(seeds) != tuple(FINAL_SEEDS):
        raise LockError(f"official runs use seeds {list(FINAL_SEEDS)}")
    out = []
    for label, scenarios, switch in EXTRAS:
        params = replace(lock["smart_params"], **{switch: False})
        for sc in scenarios:
            for seed in seeds:
                out.append(RunSpec(sc, seed, "Smart", label, params, vehicle_policy=vehicle_policy))
    return out


def check_official(specs: Sequence[RunSpec], lock: Mapping) -> None:
    """Refuse any official spec that is not exactly the approved matrix entry."""
    extras = {label: (scs, replace(lock["smart_params"], **{sw: False})) for label, scs, sw in EXTRAS}
    for s in specs:
        if not s.is_study_window():
            raise LockError(f"{s.label}: official runs use the study windows and dt")
        if s.scenario not in OFFICIAL_SCENARIOS:
            raise LockError(f"{s.scenario!r} is not an official scenario")
        if s.seed not in FINAL_SEEDS:
            raise LockError(f"seed {s.seed} is not an evaluation seed")
        if s.controller != "Smart" and s.label != s.controller:
            raise LockError(f"{s.label}: fixed-time runs are labelled by their controller")
        if s.controller == "FT-TOD" and s.scenario != "S6":
            raise LockError("FT-TOD exists for S6 only")
        if s.controller == "Smart" and s.label in extras:
            scs, want = extras[s.label]
            if s.scenario not in scs or s.smart_params != want:
                raise LockError(f"{s.label} on {s.scenario}: not an approved study extra")
            continue
        if s.controller == "Smart":
            want = {TUNED_LABEL: lock["smart_params"], STUDY_LABEL: SmartParams()}.get(s.label)
            if want is None or s.smart_params != want:
                raise LockError(f"{s.scenario} seed {s.seed}: Smart parameters are not the locked "
                                f"{TUNED_LABEL} or the study defaults ({s.label})")
