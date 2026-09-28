"""experiments -- reproducible, resumable, parallel experiment runs (Stage 14).

Spec: study §10 (paired runs, seeds, frozen parameters), §11 (metrics), §16 steps 9-10;
docs/ARCHITECTURE.md ("Experiments and tuning (Stage 14)").

A run is a RunSpec: scenario, seed, controller (FT-A / FT-B / FT-TOD / Smart), the Smart
parameters, a presentation label (e.g. "Smart-study", "Smart-T1", a tuning candidate) and the
official study windows. Everything the simulation depends on goes into the run KEY:

    key = sha256(scenario, seed, controller, SmartParams (Smart only), dt, windows,
                 arrival checksum, ENGINE_VERSION)

The label is NOT part of the key: two labels with the same parameters (e.g. Smart-study and
the tuning candidate tau=10, W_MAX=90) are the same simulation and share one run file.

Execution (run_campaign):
  1. the parent creates or loads every arrival file SERIALLY (traffic_gen, one file per
     scenario and seed, shared by all controllers) and records each checksum;
  2. runs whose result file exists and is valid are skipped (resume); a corrupt or partial file
     is removed and run again;
  3. the remaining runs execute in a process pool (spawn, Windows-safe) or sequentially;
     workers only READ arrival files, verify the checksum and run the unchanged Stage 12
     runner; each result is written to <out>/runs/<key>.json.gz via a temp file and an atomic
     rename, so a killed run never leaves a valid-looking file;
  4. the summary is rebuilt from the files in a fixed order (scenario, label, seed), so
     sequential, parallel and resumed campaigns give byte-identical summaries.

A failed simulation (e.g. a refused SignalBank command) is recorded as a run with an "error"
field; it is never silently dropped. Nothing here changes how a run is simulated.
"""
from dataclasses import asdict, dataclass, field
import csv
import gzip
import hashlib
import io
import json
import multiprocessing
import os
from pathlib import Path
import platform
import subprocess
import sys
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from smarttraffic.conflicts import MOVES
from smarttraffic.controller_smart import SmartParams
from smarttraffic.runner import CONTROLLERS, RunConfig, arrivals_sha256, run
from smarttraffic.signals import (ALL_RED_MS, FLASHING_GREEN_MS, RED_YELLOW_MS, YELLOW_MS,
                                  movement_type)
from smarttraffic.traffic_gen import (ARRIVALS_DIR, FINAL_SEEDS, TUNING_SEEDS, get_or_create_arrivals,
                                      load_arrivals)

__all__ = ["ENGINE_VERSION", "RunSpec", "run_key", "prepare_arrivals", "execute_spec",
           "run_campaign", "load_record", "summary_rows", "write_csv", "default_workers",
           "source_fingerprint", "git_commit", "check_signal_log", "SUMMARY_FIELDS",
           "FINGERPRINT_FIELDS", "sim_fingerprint", "fingerprint_rows",
           "TUNING_SEEDS", "FINAL_SEEDS"]

ENGINE_VERSION = "stage14-1"            # bump if the run record format changes
_STUDY_WINDOW = (300.0, 3600.0, 900.0, 100)   # warm-up s, measure s, flush s, dt ms


# ================================================================ run specification
@dataclass(frozen=True)
class RunSpec:
    scenario: str
    seed: int
    controller: str                          # FT-A | FT-B | FT-TOD | Smart
    label: str                               # FT-A, FT-B, FT-TOD, Smart-study, Smart-T1, candidate id
    smart_params: SmartParams = field(default_factory=SmartParams)
    warmup_s: float = 300.0
    measure_s: float = 3600.0
    flush_s: float = 900.0
    dt_ms: int = 100

    def __post_init__(self):
        if self.controller not in CONTROLLERS:
            raise ValueError(f"unknown controller {self.controller!r}")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or self.seed < 0:
            raise ValueError(f"seed must be a non-negative int, got {self.seed!r}")
        if not isinstance(self.smart_params, SmartParams):
            raise TypeError("smart_params must be a SmartParams")
        if not self.label or not isinstance(self.label, str):
            raise ValueError("label must be a non-empty string")
        self.config()                         # validates windows / dt

    def config(self) -> RunConfig:
        return RunConfig(self.scenario, self.seed, self.controller, dt_ms=self.dt_ms,
                         warmup_s=self.warmup_s, measure_s=self.measure_s, flush_s=self.flush_s,
                         smart_params=self.smart_params)

    def identity(self) -> Dict[str, object]:
        """Everything the simulation depends on (except the arrival checksum)."""
        return {"scenario": self.scenario, "seed": self.seed, "controller": self.controller,
                "smart_params": asdict(self.smart_params) if self.controller == "Smart" else None,
                "warmup_s": self.warmup_s, "measure_s": self.measure_s, "flush_s": self.flush_s,
                "dt_ms": self.dt_ms}

    def is_study_window(self) -> bool:
        return (self.warmup_s, self.measure_s, self.flush_s, self.dt_ms) == _STUDY_WINDOW


def _canonical(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False)


def run_key(spec: RunSpec, arrivals_sha: str) -> str:
    body = {"engine": ENGINE_VERSION, "spec": spec.identity(), "arrivals_sha256": arrivals_sha}
    return hashlib.sha256(_canonical(body).encode("utf-8")).hexdigest()


# ================================================================ provenance
def source_fingerprint(root: Optional[Union[str, Path]] = None) -> str:
    """SHA-256 over every smarttraffic/*.py and config/*.json (sorted by name): identifies the
    exact code and configuration that produced a campaign, committed or not."""
    root = Path(root) if root else Path(__file__).resolve().parent.parent
    h = hashlib.sha256()
    for p in sorted(list((root / "smarttraffic").glob("*.py")) + list((root / "config").glob("*.json"))):
        if p.name == "smart_params_locked.json":
            continue                          # written by the tuning campaign itself
        h.update(p.relative_to(root).as_posix().encode())
        h.update(b"\0")
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
        h.update(b"\0")
    return h.hexdigest()


def git_commit(root: Optional[Union[str, Path]] = None) -> Dict[str, object]:
    root = Path(root) if root else Path(__file__).resolve().parent.parent
    try:
        head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True,
                              timeout=10)
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True,
                               timeout=10)
        if head.returncode == 0:
            return {"commit": head.stdout.strip(), "dirty": bool(dirty.stdout.strip())}
    except (OSError, subprocess.SubprocessError):
        pass
    return {"commit": None, "dirty": None}


def default_workers() -> int:
    """4 workers by default, fewer on small machines (never more than the CPU count)."""
    return max(1, min(4, os.cpu_count() or 1))


# ================================================================ arrivals (parent only)
def prepare_arrivals(specs: Iterable[RunSpec], arrivals_dir: Union[str, Path] = ARRIVALS_DIR
                     ) -> Dict[Tuple[str, int, float, float], Tuple[str, str, int]]:
    """Create/load the arrival file of every (scenario, seed, window) serially.
    Returns {(scenario, seed, start_s, end_s): (path, sha256, count)}."""
    out = {}
    for spec in specs:
        k = (spec.scenario, spec.seed, -spec.warmup_s, spec.measure_s)
        if k in out:
            continue
        sched, path = get_or_create_arrivals(spec.scenario, spec.seed, -spec.warmup_s, spec.measure_s,
                                             directory=arrivals_dir)
        out[k] = (str(path), arrivals_sha256(sched), len(sched.arrivals))
    return out


# ================================================================ one run (worker)
def check_signal_log(log: Sequence[Sequence], params: Optional[SmartParams] = None) -> List[str]:
    """Independent check of the signal log: legal transitions, exact timed intervals, and
    GREEN + FLASHING_GREEN >= the standard minimum green (7 s straight / 5 s left, which the
    12 s extended initial only raises). Returns a list of problems (empty = OK)."""
    nxt = {"RED": "RED_YELLOW", "RED_YELLOW": "GREEN", "GREEN": "FLASHING_GREEN",
           "FLASHING_GREEN": "YELLOW", "YELLOW": "ALL_RED", "ALL_RED": "RED"}
    p = params or SmartParams()
    problems: List[str] = []
    state = {m: "RED" for m in MOVES}
    since: Dict[str, Optional[int]] = {m: None for m in MOVES}
    green_on: Dict[str, Optional[int]] = {m: None for m in MOVES}
    for t, m, new in log:
        old = state[m]
        if nxt[old] != new:
            problems.append(f"{m}: illegal transition {old}->{new} at {t} ms")
        if since[m] is not None:
            dur = t - since[m]
            kind = movement_type(m)
            exact = {"RED_YELLOW": RED_YELLOW_MS, "FLASHING_GREEN": FLASHING_GREEN_MS,
                     "YELLOW": YELLOW_MS[kind], "ALL_RED": ALL_RED_MS[kind]}.get(old)
            if exact is not None and dur != exact:
                problems.append(f"{m}: {old} lasted {dur} ms, expected {exact} at {t} ms")
        if new == "GREEN":
            green_on[m] = t
        if new == "YELLOW" and green_on[m] is not None:
            g = t - green_on[m]
            need = p.min_green_ms(movement_type(m), False)
            if g < need:
                problems.append(f"{m}: green time {g} ms < minimum {need} ms at {t} ms")
        state[m], since[m] = new, t
    return problems


def _summary_from_result(res) -> Dict[str, object]:
    I = asdict(res.intersection)
    return {**I, "end_ms": res.end_ms, "ended_early": res.ended_early}


def execute_spec(spec: RunSpec, arrivals_path: str, arrivals_sha: str) -> Dict[str, object]:
    """Run one spec from a READ-ONLY arrival file. Returns the complete run record."""
    rec: Dict[str, object] = {"engine": ENGINE_VERSION, "key": run_key(spec, arrivals_sha),
                              "label": spec.label, "spec": spec.identity(),
                              "arrivals_path": Path(arrivals_path).name}
    try:
        sched = load_arrivals(arrivals_path)
        sha = arrivals_sha256(sched)
        if sha != arrivals_sha:
            raise RuntimeError(f"arrival file {arrivals_path} changed: {sha} != {arrivals_sha}")
        res = run(spec.config(), arrivals=sched)
        if res.metadata["arrivals_sha256"] != arrivals_sha:
            raise RuntimeError("runner used different arrivals")
        rec["metadata"] = res.metadata
        rec["summary"] = _summary_from_result(res)
        rec["movements"] = [asdict(m) for m in res.movements]
        rec["diagnostics"] = res.diagnostics
        rec["signal_check"] = check_signal_log(res.signal_log, spec.smart_params)
        rec["result_sha256"] = hashlib.sha256(res.to_json().encode("utf-8")).hexdigest()
        rec["result"] = json.loads(res.to_json())
        rec["error"] = None
    except Exception as e:                    # a refused command etc.: recorded, never dropped
        rec["error"] = f"{type(e).__name__}: {e}"
    return rec


def _worker(job):
    spec, path, sha, out_dir = job
    rec = execute_spec(spec, path, sha)
    _write_record(Path(out_dir), rec)
    return rec["key"]


def _init_worker():
    for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[var] = "1"


# ================================================================ storage
def _record_path(out_dir: Path, key: str) -> Path:
    return out_dir / "runs" / f"{key}.json.gz"


def _write_record(out_dir: Path, rec: Dict[str, object]) -> Path:
    runs = out_dir / "runs"
    runs.mkdir(parents=True, exist_ok=True)
    final = _record_path(out_dir, rec["key"])
    tmp = runs / f".{rec['key']}.{os.getpid()}.tmp"
    data = gzip.compress(_canonical(rec).encode("utf-8"), mtime=0)
    with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, final)                   # atomic: the final name only ever holds a full record
    return final


def load_record(out_dir: Union[str, Path], key: str) -> Optional[Dict[str, object]]:
    """The stored record for `key`, or None if missing, unreadable, truncated or mismatched."""
    p = _record_path(Path(out_dir), key)
    if not p.exists():
        return None
    try:
        rec = json.loads(gzip.decompress(p.read_bytes()).decode("utf-8"))
    except (OSError, EOFError, ValueError):
        return None
    if not isinstance(rec, dict) or rec.get("key") != key or rec.get("engine") != ENGINE_VERSION:
        return None
    if rec.get("error") is None and ("summary" not in rec or "result_sha256" not in rec):
        return None
    return rec


# ================================================================ campaign
def run_campaign(specs: Sequence[RunSpec], out_dir: Union[str, Path], *, workers: Optional[int] = None,
                 arrivals_dir: Union[str, Path] = ARRIVALS_DIR, log=print) -> List[Dict[str, object]]:
    """Run every spec (resuming completed ones) and return the records in a fixed order
    (scenario, label, seed). Duplicate labels for one run key are allowed (shared run)."""
    out_dir = Path(out_dir)
    from smarttraffic.custom import CUSTOM_RESULTS_DIR
    cust = CUSTOM_RESULTS_DIR.resolve()
    if out_dir.resolve() == cust or cust in out_dir.resolve().parents:
        raise ValueError(f"official/tuning campaigns must not be written under {cust} (Custom/Demo only)")
    specs = list(specs)
    labels = {}
    for s in specs:
        if not isinstance(s, RunSpec) or s.config().mode != "official":
            raise ValueError(f"campaigns run official RunSpecs only, got {s!r}")
        k = (s.scenario, s.label, s.seed)
        if k in labels:
            raise ValueError(f"duplicate spec {k}")
        labels[k] = s
    arrivals = prepare_arrivals(specs, arrivals_dir)
    todo, keys, seen = [], {}, set()
    for s in specs:
        path, sha, _ = arrivals[(s.scenario, s.seed, -s.warmup_s, s.measure_s)]
        k = run_key(s, sha)
        keys[(s.scenario, s.label, s.seed)] = k
        if k in seen:
            continue
        seen.add(k)
        if load_record(out_dir, k) is None:
            stale = _record_path(out_dir, k)
            if stale.exists():
                stale.unlink()                # corrupt / partial: run again
            todo.append((s, path, sha, str(out_dir)))
    todo.sort(key=lambda j: run_key(j[0], j[2]))
    n = default_workers() if workers is None else int(workers)
    if n < 1:
        raise ValueError("workers must be >= 1")
    log(f"{len(seen)} distinct runs, {len(seen) - len(todo)} already done, {len(todo)} to run "
        f"({n} worker{'s' if n != 1 else ''})")
    if todo:
        if n == 1:
            for i, job in enumerate(todo, 1):
                _worker(job)
                log(f"  [{i}/{len(todo)}] {job[0].scenario} {job[0].label} seed {job[0].seed}")
        else:
            ctx = multiprocessing.get_context("spawn")
            with ctx.Pool(n, initializer=_init_worker) as pool:
                for i, _ in enumerate(pool.imap_unordered(_worker, todo), 1):
                    log(f"  [{i}/{len(todo)}] done")
    out = []
    for (sc, label, seed), k in sorted(keys.items()):
        rec = load_record(out_dir, k)
        if rec is None:
            raise RuntimeError(f"run {sc} {label} seed {seed} has no valid result")
        out.append({**rec, "label": label})
    return out


# ================================================================ summaries
SUMMARY_FIELDS = ["scenario", "label", "controller", "seed", "run_key", "arrivals_sha256", "error",
                  "tau_s", "w_max_s", "max_green_straight_s", "max_green_left_s",
                  "delay_mean_s", "delay_p95_s", "delay_max_s", "delay_total_veh_h",
                  "delay_mean_lower_bound_s", "waiting_mean_s", "vehicles_control_delay_over_90s",
                  "queue_mean_per_movement_veh", "queue_max_movement_veh", "queue_total_mean_veh",
                  "queue_total_max_veh", "throughput_veh_h", "arrivals", "crossed", "unfinished",
                  "served_in_hour", "pct_served", "green_onsets", "signal_changes_per_hour",
                  "jain_fairness", "safety_violations", "signal_check_problems",
                  "detector_call_wait_max_s", "detector_calls_over_120s", "holds",
                  "term_gap", "term_max", "term_hold", "end_ms", "ended_early", "result_sha256"]


def summary_rows(records: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    rows = []
    for r in records:
        spec, s = r["spec"], r.get("summary") or {}
        sp = spec.get("smart_params") or {}
        d = r.get("diagnostics") or {}
        calls = d.get("detector_call_wait_max_s")
        over = d.get("detector_calls_over_120s")
        term = d.get("controller_terminations") or {}
        row = {"scenario": spec["scenario"], "label": r["label"], "controller": spec["controller"],
               "seed": spec["seed"], "run_key": r["key"],
               "arrivals_sha256": (r.get("metadata") or {}).get("arrivals_sha256"),
               "error": r.get("error") or "",
               "tau_s": sp.get("tau_s"), "w_max_s": sp.get("w_max_s"),
               "max_green_straight_s": sp.get("max_green_straight_s"),
               "max_green_left_s": sp.get("max_green_left_s"),
               "signal_check_problems": len(r.get("signal_check") or []),
               "detector_call_wait_max_s": max(calls.values()) if calls else None,
               "detector_calls_over_120s": sum(over.values()) if over else None,
               "holds": d.get("controller_hold_decisions"),
               "term_gap": term.get("gap"), "term_max": term.get("max"), "term_hold": term.get("hold"),
               "result_sha256": r.get("result_sha256")}
        for k in SUMMARY_FIELDS:
            if k not in row:
                row[k] = s.get(k)
        rows.append(row)
    rows.sort(key=lambda x: (x["scenario"], x["label"], x["seed"]))
    return rows


# ---------------------------------------------------------------- platform-independent fingerprint
FINGERPRINT_FIELDS = ["scenario", "label", "controller", "seed", "run_key", "arrivals_sha256", "sim_sha256"]


def sim_fingerprint(rec: Mapping[str, object]) -> Optional[str]:
    """SHA-256 of the simulated trajectory of a run: metadata, end time, every vehicle record
    (integer ms), the signal log (integer ms) and the 1 s queue samples (integers). These are
    exact on every platform. The derived floating-point statistics are excluded: some of them
    (e.g. Jain's index, x ** 2 via the C library's pow) may differ in the last bit between
    operating systems, which changes result_sha256 without any change in the simulation."""
    res = rec.get("result")
    if not res:
        return None
    body = {k: res[k] for k in ("metadata", "end_ms", "ended_early", "vehicles", "signal_log", "queue_samples")}
    return hashlib.sha256(_canonical(body).encode("utf-8")).hexdigest()


def fingerprint_rows(records: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    rows = [{"scenario": r["spec"]["scenario"], "label": r["label"], "controller": r["spec"]["controller"],
             "seed": r["spec"]["seed"], "run_key": r["key"],
             "arrivals_sha256": (r.get("metadata") or {}).get("arrivals_sha256"),
             "sim_sha256": sim_fingerprint(r)} for r in records]
    rows.sort(key=lambda x: (x["scenario"], x["label"], x["seed"]))
    return rows


def _fmt(v) -> str:
    if v is None:
        return ""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(round(v, 9))
    return str(v)


def write_csv(path: Union[str, Path], rows: Sequence[Mapping[str, object]], fields: Sequence[str]) -> str:
    """Deterministic CSV (LF line ends, fixed column order). Returns its SHA-256."""
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(fields)
    for r in rows:
        w.writerow([_fmt(r.get(f)) for f in fields])
    data = buf.getvalue().encode("utf-8")
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, p)
    return hashlib.sha256(data).hexdigest()


def environment() -> Dict[str, object]:
    import numpy
    return {"python": sys.version.split()[0], "numpy": numpy.__version__, "platform": platform.platform()}
