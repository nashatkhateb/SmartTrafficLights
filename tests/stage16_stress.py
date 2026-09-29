"""Stage 16 -- full-length end-to-end matrix and stress runs (not part of pytest: ~20-40 min).

    python -m tests.stage16_stress [--jobs 2] [--seeds 1,2,3] [--out results/stage16]

Every run is audited step by step by tests/stage16_checks.Audit (safety invariants, sensor
consistency, Smart end-of-green reasons, first-car / headway rules, finite numbers). Writes
stress_report.json (simulation results are deterministic; only the memory peaks and the number
of snapshots kept at Max speed depend on the machine) and stress_timing.json (wall-clock times).

Groups
  matrix      S1-S10 (11 IDs) x FT-A / FT-B / Smart-T1 (+ FT-TOD on S6) x seeds, 300 + 3600 + 900 s
  custom      Custom/Demo (FT-A and Smart-T1): zero demand, one movement only, all movements,
              high valid demand, near capacity, oversaturated
  long        zero demand 7200 s, one movement only 7200 s, virtual-only S1 Smart-T1 7200 s,
              S7b / S8 / S10 Smart-T1 seeds 1-5 (heavy, left-heavy, starvation)
  hardware    Smart-T1 1 h on the SimulatedEsp32 chain (2 real + 14 virtual, real-only NS),
              with a mid-run USB disconnect and reconnect
  determinism selected runs repeated: identical RunResult JSON
  max speed   the visualization session at Max speed gives the identical RunResult
  memory      tracemalloc peak of a 1 h and a 2 h run (linear growth check)
"""
import argparse
import json
import multiprocessing as mp
import sys
import time
import tracemalloc
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.stage16_checks import HardwareChain, audited_run, ns_car_script   # noqa: E402

SCENARIOS = ("S1", "S2", "S3", "S4", "S5", "S6", "S7a", "S7b", "S8", "S9", "S10")
FULL = dict(warmup_s=300.0, measure_s=3600.0, flush_s=900.0)


def _t1():
    from smarttraffic.viz import smart_t1_params
    return smart_t1_params()


def _cfg(sc, seed, ctrl, **kw):
    from smarttraffic.runner import RunConfig
    c = "Smart" if ctrl == "Smart-T1" else ctrl
    return RunConfig(sc, seed, c, smart_params=_t1(), **{**FULL, **kw})


def _custom_cfg(rates, ctrl, measure_s=3600, seed=1):
    from smarttraffic.custom import CustomDemand, CustomSpec
    from smarttraffic.conflicts import MOVES
    d = CustomDemand.from_mapping({m: rates.get(m, 0.0) for m in MOVES})
    spec = CustomSpec(d, seed=seed, controller="Smart" if ctrl == "Smart-T1" else ctrl, measure_s=measure_s)
    return spec.config(), spec.saturation()


def _summ(res):
    d = json.loads(res.to_json())
    return d


def _row(label, res, rep, extra=None):
    d = _summ(res)
    s = d["intersection"]
    keys = ("arrivals", "crossed", "unfinished", "delay_mean_s", "delay_p95_s", "delay_max_s",
            "queue_total_max_veh", "safety_violations", "green_onsets", "pct_served")
    out = {"label": label, **{k: s.get(k) for k in keys if k in s}}
    out.update({k: rep[k] for k in ("steps", "vehicles", "crossings", "transitions", "detector_edges",
                                    "conflicts", "first_car_checks", "smart_end_checks", "end_reasons",
                                    "max_queue", "max_outside", "max_wait_s", "hold_seen", "level2_latches")})
    out["ended_early"] = d["ended_early"]
    out["end_s"] = d["end_ms"] / 1000
    out["ns_arrivals"] = [m["arrivals"] for m in d["movements"] if m["movement"] == "NS"][0]
    out["errors"] = rep["errors"]
    if extra:
        out.update(extra)
    return out


# ------------------------------------------------------------------ jobs
def job_matrix(args):
    sc, seed, ctrl = args
    t = time.time()
    res, rep = audited_run(_cfg(sc, seed, ctrl), check_every=5)
    return "matrix", _row(f"{sc}/{ctrl}/seed{seed}", res, rep), time.time() - t


CUSTOM_CASES = {
    "zero demand": {},
    "one movement only (NS 600)": {"NS": 600},
    "all movements (500/150)": {m: (500 if m[1] == "S" else 150) for m in
                                ("NS", "SS", "ES", "WS", "NL", "SL", "EL", "WL")},
    "high valid (NS 1800 only)": {"NS": 1800},
    "near capacity (560/165)": {m: (560 if m[1] == "S" else 165) for m in
                                ("NS", "SS", "ES", "WS", "NL", "SL", "EL", "WL")},
    "oversaturated (750/240)": {m: (750 if m[1] == "S" else 240) for m in
                                ("NS", "SS", "ES", "WS", "NL", "SL", "EL", "WL")},
}


def job_custom(args):
    name, ctrl = args
    t = time.time()
    cfg, sat = _custom_cfg(CUSTOM_CASES[name], ctrl)
    res, rep = audited_run(cfg, check_every=5)
    return "custom", _row(f"CUSTOM {name}/{ctrl}", res, rep,
                          {"saturation": sat.level, "X": round(sat.degree, 3)}), time.time() - t


def job_long(args):
    kind = args[0]
    t = time.time()
    if kind == "zero":
        cfg, _ = _custom_cfg({}, "Smart-T1", measure_s=7200)
        label = "LONG zero demand 7200 s / Smart-T1"
    elif kind == "one":
        cfg, _ = _custom_cfg({"WL": 400}, "Smart-T1", measure_s=7200)
        label = "LONG one movement only (WL 400) 7200 s / Smart-T1"
    elif kind == "virtual":
        cfg = _cfg("S1", 1, "Smart-T1", measure_s=7200.0)
        label = "LONG virtual-only S1 7200 s / Smart-T1"
    else:
        sc, seed = args[1], args[2]
        cfg = _cfg(sc, seed, "Smart-T1")
        label = f"HEAVY {sc}/Smart-T1/seed{seed}"
    res, rep = audited_run(cfg, check_every=5)
    return "long", _row(label, res, rep), time.time() - t


def job_hardware(args):
    from smarttraffic.hardware import load_hardware_config
    t = time.time()
    cfg = _cfg("S1", 1, "Smart-T1", warmup_s=0.0)
    chain = HardwareChain(cars=ns_car_script(5_000, 3_590_000),
                          script={1_200_000: lambda e: e.unplug(), 1_320_000: lambda e: e.plug()})
    res, rep = audited_run(cfg, source_factory=chain.factory, observers=[chain.observe], virtual=False,
                           exclude_movements=load_hardware_config().real_only_movements, check_every=5)
    hs = chain.summary()
    link = chain.hw.hub.link("esp32_a")
    extra = {"hardware": hs, "link_stats": {k: link.stats[k] for k in sorted(link.stats)}}
    return "hardware", _row("HARDWARE chain S1 1 h / Smart-T1 (disconnect 1200-1320 s)", res, rep, extra), time.time() - t


def job_determinism(args):
    sc, ctrl = args
    t = time.time()
    a = __import__("smarttraffic.runner", fromlist=["run"]).run(_cfg(sc, 2, ctrl)).to_json()
    b = __import__("smarttraffic.runner", fromlist=["run"]).run(_cfg(sc, 2, ctrl)).to_json()
    return "determinism", {"label": f"{sc}/{ctrl}/seed2 run twice", "identical": a == b}, time.time() - t


def job_maxspeed(args):
    from smarttraffic.runner import run
    from smarttraffic.viz_session import VizSession
    sc, ctrl = args
    t = time.time()
    cfg = _cfg(sc, 1, ctrl)
    ref = run(cfg).to_json()
    s = VizSession(cfg, speed=None, running=True).start()
    ok = s.wait_finished(900)
    same = ok and s.error is None and s.result.to_json() == ref
    snaps = len(getattr(s, "_snaps", []))
    s.stop() if hasattr(s, "stop") else None
    return "maxspeed", {"label": f"VizSession Max speed {sc}/{ctrl}", "finished": ok, "error": s.error,
                        "identical_to_run": same, "snapshots_kept": snaps}, time.time() - t


def job_memory(args):
    from smarttraffic.runner import run
    t = time.time()
    out = {}
    for meas in (3600.0, 7200.0):
        cfg = _cfg("S7a", 1, "Smart-T1", measure_s=meas)
        tracemalloc.start()
        res = run(cfg)
        cur, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        n = json.loads(res.to_json())["intersection"]["arrivals"]
        out[f"{int(meas)}s"] = {"peak_MB": round(peak / 1e6, 1), "arrivals": n,
                                "bytes_per_arrival": round(peak / max(1, n))}
    ratio = out["7200s"]["peak_MB"] / out["3600s"]["peak_MB"]
    return "memory", {"label": "tracemalloc peak S7a Smart-T1 1 h vs 2 h", **out,
                      "peak_ratio_2h_vs_1h": round(ratio, 2)}, time.time() - t


def _dispatch(job):
    fn, args = job
    return globals()[fn](args)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--out", default=str(ROOT / "results" / "stage16"))
    ap.add_argument("--only", default="")
    a = ap.parse_args(argv)
    seeds = [int(s) for s in a.seeds.split(",")]
    jobs = []
    for seed in seeds:
        for sc in SCENARIOS:
            for ctrl in ("FT-A", "FT-B", "Smart-T1") + (("FT-TOD",) if sc == "S6" else ()):
                jobs.append(("job_matrix", (sc, seed, ctrl)))
    for name in CUSTOM_CASES:
        for ctrl in ("FT-A", "Smart-T1"):
            jobs.append(("job_custom", (name, ctrl)))
    jobs += [("job_long", ("zero",)), ("job_long", ("one",)), ("job_long", ("virtual",))]
    for sc in ("S7b", "S8", "S10"):
        for seed in range(1, 6):
            if seed not in seeds:
                jobs.append(("job_long", ("heavy", sc, seed)))
    jobs.append(("job_hardware", ()))
    for sc, ctrl in (("S1", "Smart-T1"), ("S7b", "Smart-T1"), ("S10", "FT-A"), ("S6", "FT-TOD")):
        jobs.append(("job_determinism", (sc, ctrl)))
    jobs.append(("job_maxspeed", ("S7b", "Smart-T1")))
    jobs.append(("job_memory", ()))
    if a.only:
        jobs = [j for j in jobs if j[0] in a.only.split(",")]
    t0 = time.time()
    results = {}
    timing = {}
    with mp.get_context("fork").Pool(a.jobs) as pool:
        for i, (group, row, dt) in enumerate(pool.imap(_dispatch, jobs)):
            results.setdefault(group, []).append(row)
            timing[row["label"]] = round(dt, 1)
            bad = row.get("errors")
            print(f"[{i + 1}/{len(jobs)}] {row['label']}: {dt:.1f}s"
                  + (f"  ERRORS {len(bad)}: {bad[:2]}" if bad else ""), flush=True)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "stress_report.json").write_bytes((json.dumps({"runs": results}, indent=1, sort_keys=True)
                                              + "\n").encode("utf-8"))
    (out / "stress_timing.json").write_bytes((json.dumps({"total_s": round(time.time() - t0, 1),
                                                          "per_run_s": timing}, indent=1) + "\n").encode("utf-8"))
    n_err = sum(len(r.get("errors") or []) for g in results.values() for r in g)
    print(f"done: {sum(len(g) for g in results.values())} runs, {n_err} audit errors, "
          f"{time.time() - t0:.0f} s")
    return 0 if n_err == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
