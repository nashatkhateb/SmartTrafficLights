"""make_graphs -- Stage 15 result figures (study §12 graph plan, §16 step 11).

    python -m smarttraffic.make_graphs                       # all figures from the committed CSVs
    python -m smarttraffic.make_graphs extract-s6 [--records DIR]   # (re)build the S6 time series

What it reads (never writes)
----------------------------
* the official campaign folder (default results/official/campaign_T1): manifest.json,
  runs.csv, aggregates.csv, paired.csv. Every CSV is checked against the SHA-256 recorded in
  the manifest before use, so a figure can only come from the exact committed results.
* the tuning folder results/tuning/S1_T1 (manifest.json, runs.csv, selection.json) for the
  historical Smart-T1 vs Smart-study figure only.
* for the S6 time series: the extract <out>/data/S6_timeseries.csv, produced by
  `extract-s6` from the campaign's per-run records (<campaign>/runs/<key>.json.gz) or, when
  those are not on this machine, by re-simulating exactly those runs. Either way every run is
  accepted only if its simulated-trajectory fingerprint equals the golden fingerprint of the
  official run (tests/golden/official_T1_baseline.csv), so the extract is the official data.

Vehicle policy
--------------
Every input carries the vehicle policy it was produced with (manifest "vehicle_policy";
missing = legacy-study, i.e. Stage 14). One figure set is built from ONE policy: inputs with
different policies are refused (PolicyMixError), and the output folder is named after the
campaign and its policy (default results/figures/campaign_T1__legacy-study). Every figure
states its source and policy in its caption and in the file metadata; nothing is relabelled.

Outputs (<out>/)
----------------
F01..F13 as .png (300 dpi), .pdf and .svg; figures_values.csv (every plotted number with its
source, policy and CI); figures_manifest.json (input hashes, policy, figure list, hash of the
values; no timestamps, so identical inputs give an identical manifest); README.md (index:
file, source, metric, interpretation, policy). Figures use only the data; no simulation is run
except the fingerprint-checked S6 re-simulation fallback of `extract-s6`.
"""
import argparse
import csv
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import sys
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from smarttraffic.stats import mean_ci95  # noqa: E402
from smarttraffic.vehicles import LEGACY_STUDY, POLICIES  # noqa: E402

__all__ = ["FIG_VERSION", "PolicyMixError", "Dataset", "load_campaign", "load_tuning", "extract_s6",
           "load_s6", "build_all", "FIGURES", "SCENARIOS", "CORE", "main"]

ROOT = Path(__file__).resolve().parent.parent
CAMPAIGN_DIR = ROOT / "results" / "official" / "campaign_T1"
TUNING_DIR = ROOT / "results" / "tuning" / "S1_T1"
FIGURES_ROOT = ROOT / "results" / "figures"
GOLDEN = ROOT / "tests" / "golden" / "official_T1_baseline.csv"
FIG_VERSION = "stage15-1"

SCENARIOS = ("S1", "S2", "S3", "S4", "S5", "S6", "S7a", "S7b", "S8", "S9", "S10")
CORE = ("FT-A", "FT-B", "Smart-T1")
KEY_SCENARIOS = ("S4", "S6", "S7b", "S8", "S10")
S6_LABELS = ("FT-A", "FT-B", "Smart-T1", "FT-TOD")
S6_SHIFT_S = 1800
SCENARIO_NAMES = {
    "S1": "balanced moderate", "S2": "one heavy approach", "S3": "near-empty approach",
    "S4": "very low traffic", "S5": "sudden burst", "S6": "demand shifts sides",
    "S7a": "heavy (near capacity)", "S7b": "oversaturated", "S8": "strong left turns",
    "S9": "unequal straight vs left", "S10": "low-demand movement"}
FTB_EQUALS_FTA = ("S1", "S3", "S5")          # config/fixed_plans.json: FT-B/Sx greens = FT-A

# ------------------------------------------------------------------ style (validated palette)
# Categorical slots 1-3 (FT-A, FT-B, Smart-T1) validate all-pairs for colour-vision deficiency;
# FT-TOD takes slot 4 and is always drawn after Smart-T1 (adjacent pairs validated). Aqua and
# yellow are below 3:1 on the surface, so every figure has a legend and figures_values.csv is
# the table view. Smart-study (historical) = violet; Smart-T1 without W_MAX = magenta + hatch.
COLORS = {"FT-A": "#2a78d6", "FT-B": "#eb6834", "Smart-T1": "#1baf7a", "FT-TOD": "#eda100",
          "Smart-study": "#4a3aa7", "Smart-T1-noWMAX": "#e87ba4"}
HATCH = {"Smart-T1-noWMAX": "////"}
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8983", "#e6e5e1", "#ffffff"
FONT = "DejaVu Sans"                           # ships with matplotlib: identical on every OS
DPI = 300
FIGSIZE = (12.0, 6.75)                         # 16:9


class PolicyMixError(ValueError):
    """Figure inputs were produced with different vehicle policies."""


class InputChangedError(ValueError):
    """An input file does not match the hash recorded in its manifest."""


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _rel(path: Path) -> str:
    try:
        return Path(path).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return Path(path).as_posix()


def _policy_of(manifest: Mapping, folder: Path) -> str:
    """The folder's vehicle policy: manifest field (missing = legacy-study, Stage 14); a
    vehicle_policy.json marker, if present, must agree."""
    pol = manifest.get("vehicle_policy", LEGACY_STUDY.name)
    marker = folder / "vehicle_policy.json"
    if marker.exists():
        m = json.loads(marker.read_text(encoding="utf-8"))["vehicle_policy"]
        if m != pol:
            raise PolicyMixError(f"{folder}: manifest says {pol}, marker says {m}")
    if pol not in POLICIES:
        raise PolicyMixError(f"{folder}: unknown vehicle policy {pol!r}")
    return pol


def _num(v) -> Optional[float]:
    if v is None or v == "":
        return None
    x = float(v)
    return x


# ================================================================== inputs
class Dataset:
    """The official campaign summaries of one folder, verified against its manifest."""

    def __init__(self, folder: Path):
        self.folder = Path(folder)
        self.manifest = json.loads((self.folder / "manifest.json").read_text(encoding="utf-8"))
        self.policy = _policy_of(self.manifest, self.folder)
        self.name = self.manifest.get("campaign", self.folder.name)
        self.inputs: Dict[str, str] = {_rel(self.folder / "manifest.json"): _sha(self.folder / "manifest.json")}
        for fname, key in (("runs.csv", "runs_csv_sha256"), ("aggregates.csv", "aggregates_csv_sha256"),
                           ("paired.csv", "paired_csv_sha256")):
            p = self.folder / fname
            h = _sha(p)
            if self.manifest.get(key) != h:
                raise InputChangedError(f"{p} does not match {key} in its manifest")
            self.inputs[_rel(p)] = h
        self.runs = list(csv.DictReader(open(self.folder / "runs.csv", encoding="utf-8", newline="")))
        self.agg = {(r["scenario"], r["label"], r["metric"]): r
                    for r in csv.DictReader(open(self.folder / "aggregates.csv", encoding="utf-8", newline=""))}
        self.paired = {(r["scenario"], r["a"], r["b"], r["metric"]): r
                       for r in csv.DictReader(open(self.folder / "paired.csv", encoding="utf-8", newline=""))}
        self.source = _rel(self.folder)

    def mean_ci(self, sc: str, label: str, metric: str) -> Tuple[float, Optional[float], Optional[float], int]:
        r = self.agg[(sc, label, metric)]
        return float(r["mean"]), _num(r["ci_low"]), _num(r["ci_high"]), int(r["n"])

    def has(self, sc: str, label: str, metric: str) -> bool:
        r = self.agg.get((sc, label, metric))
        return r is not None and r["mean"] not in ("", None)

    def per_seed(self, sc: str, label: str, col: str) -> List[Tuple[int, float]]:
        return sorted((int(r["seed"]), float(r[col])) for r in self.runs
                      if r["scenario"] == sc and r["label"] == label and r[col] != "")

    def pair(self, sc: str, a: str, b: str, metric: str = "delay_mean_s") -> Dict[str, float]:
        r = self.paired[(sc, a, b, metric)]
        return {"diff": float(r["mean_diff_b_minus_a"]), "low": float(r["ci_low"]), "high": float(r["ci_high"]),
                "p": float(r["p_unadjusted"]), "n": int(r["n"]), "a": float(r["mean_a"]), "b": float(r["mean_b"])}

    @property
    def policy_text(self) -> str:
        if self.policy == LEGACY_STUDY.name:
            return "legacy-study vehicle policy (Stage 14 historical results)"
        return f"{self.policy} vehicle policy"


def load_campaign(folder: Path = CAMPAIGN_DIR) -> Dataset:
    return Dataset(folder)


def load_tuning(folder: Path = TUNING_DIR) -> Dict[str, object]:
    folder = Path(folder)
    man = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    pol = _policy_of(man, folder)
    runs_p = folder / "runs.csv"
    if man.get("runs_csv_sha256") != _sha(runs_p):
        raise InputChangedError(f"{runs_p} does not match its manifest")
    sel_p = folder / "selection.json"
    return {"folder": folder, "policy": pol, "manifest": man,
            "selection": json.loads(sel_p.read_text(encoding="utf-8")),
            "runs": list(csv.DictReader(open(runs_p, encoding="utf-8", newline=""))),
            "inputs": {_rel(folder / "manifest.json"): _sha(folder / "manifest.json"), _rel(runs_p): _sha(runs_p),
                       _rel(sel_p): _sha(sel_p)}}


# ================================================================== S6 time series extract
S6_FIELDS = ["series", "label", "t_start_s", "t_end_s", "n_seeds", "mean", "ci_low", "ci_high"]
QUEUE_BIN_S, DELAY_BIN_S = 30, 300


def _golden() -> Dict[Tuple[str, str, int], Dict[str, str]]:
    with open(GOLDEN, encoding="utf-8", newline="") as fh:
        return {(r["scenario"], r["label"], int(r["seed"])): r for r in csv.DictReader(fh)}


def _s6_records(ds: Dataset, records_dir: Optional[Path], log) -> Dict[Tuple[str, int], dict]:
    """The official S6 run records (label, seed) -> record, each checked against the golden
    fingerprint. Missing record files are re-simulated with the dataset's policy."""
    from smarttraffic import experiments as ex
    golden = _golden() if ds.policy == LEGACY_STUDY.name else {}
    want = [r for r in ds.runs if r["scenario"] == "S6" and r["label"] in S6_LABELS]
    out, missing = {}, []
    rdir = Path(records_dir) if records_dir else ds.folder / "runs"
    for r in want:
        p = rdir / f"{r['run_key']}.json.gz"
        if p.exists():
            out[(r["label"], int(r["seed"]))] = json.loads(gzip.decompress(p.read_bytes()).decode("utf-8"))
        else:
            missing.append(r)
    if missing:
        from smarttraffic import campaign as cp
        from smarttraffic import tuning as tu
        lock = tu.load_lock(tu.LOCK_FILE, expected_sha=ds.manifest["lock_sha256"])
        specs = {(s.label, s.seed): s for s in cp.campaign_specs(lock, ds.policy) if s.scenario == "S6"}
        need = [specs[(r["label"], int(r["seed"]))] for r in missing]
        log(f"re-simulating {len(need)} S6 runs ({ds.policy}); each must match its official fingerprint")
        arr = ex.prepare_arrivals(need)
        for s in need:
            path, sha, _ = arr[(s.scenario, s.seed, -s.warmup_s, s.measure_s)]
            out[(s.label, s.seed)] = ex.execute_spec(s, path, sha)
    keys = {(r["label"], int(r["seed"])): r["run_key"] for r in want}
    for (label, seed), rec in out.items():
        verify_record(ds, "S6", label, seed, rec, keys[(label, seed)], golden)
    if len(out) != len(S6_LABELS) * 10:
        raise InputChangedError(f"S6: expected {len(S6_LABELS) * 10} runs, found {len(out)}")
    return out


def verify_record(ds: Dataset, sc: str, label: str, seed: int, rec: Mapping, run_key: str,
                  golden: Optional[Mapping] = None) -> None:
    """A per-run record is accepted only if it is the official run: same run key, no error and (for
    the legacy-study campaign) the golden simulated-trajectory fingerprint."""
    from smarttraffic import experiments as ex
    if rec.get("key") != run_key or rec.get("error"):
        raise InputChangedError(f"{sc} {label} seed {seed}: record is not the official run")
    if golden is None and ds.policy == LEGACY_STUDY.name:
        golden = _golden()
    if golden and ex.sim_fingerprint(rec) != golden[(sc, label, seed)]["sim_sha256"]:
        raise InputChangedError(f"{sc} {label} seed {seed}: trajectory differs from the official run")


def extract_s6(ds: Dataset, out_dir: Path, records_dir: Optional[Path] = None, log=print) -> Path:
    """Write <out_dir>/data/S6_timeseries.csv (+ .json provenance) from the official S6 runs."""
    recs = _s6_records(ds, records_dir, log)
    rows = []
    for label in S6_LABELS:
        seeds = sorted(s for (lb, s) in recs if lb == label)
        # total queued vehicles (road + held outside the entry), 30 s bin means of the 1 s samples
        per_seed = {}
        for s in seeds:
            q = [(t, sum(road) + sum(outside)) for t, road, outside in recs[(label, s)]["result"]["queue_samples"]]
            per_seed[s] = [sum(v for t, v in q if b * 1000 <= t < (b + QUEUE_BIN_S) * 1000) / QUEUE_BIN_S
                           for b in range(0, 3600, QUEUE_BIN_S)]
        for i, b in enumerate(range(0, 3600, QUEUE_BIN_S)):
            c = mean_ci95([per_seed[s][i] for s in seeds])
            rows.append({"series": "queue_total_veh", "label": label, "t_start_s": b, "t_end_s": b + QUEUE_BIN_S,
                         "n_seeds": c["n"], "mean": c["mean"], "ci_low": c["low"], "ci_high": c["high"]})
        # mean control delay of measured vehicles by arrival (scheduled) time window
        for b in range(0, 3600, DELAY_BIN_S):
            vals = []
            for s in seeds:
                d = [v["control_delay_ms"] / 1000 for v in recs[(label, s)]["result"]["vehicles"]
                     if v["measured"] and v["control_delay_ms"] is not None
                     and b * 1000 <= v["scheduled_ms"] < (b + DELAY_BIN_S) * 1000]
                vals.append(sum(d) / len(d))
            c = mean_ci95(vals)
            rows.append({"series": "delay_by_arrival_s", "label": label, "t_start_s": b, "t_end_s": b + DELAY_BIN_S,
                         "n_seeds": c["n"], "mean": c["mean"], "ci_low": c["low"], "ci_high": c["high"]})
    data = Path(out_dir) / "data"
    data.mkdir(parents=True, exist_ok=True)
    p = data / "S6_timeseries.csv"
    _write_csv(p, rows, S6_FIELDS)
    prov = {"campaign": ds.name, "campaign_folder": ds.source, "vehicle_policy": ds.policy,
            "runs": {f"{lb} seed {s}": recs[(lb, s)]["key"] for lb, s in sorted(recs)},
            "fingerprints_checked_against": _rel(GOLDEN) if ds.policy == LEGACY_STUDY.name else None,
            "queue_bin_s": QUEUE_BIN_S, "delay_bin_s": DELAY_BIN_S, "csv_sha256": _sha(p)}
    _write_text(data / "S6_timeseries.json", json.dumps(prov, indent=2, sort_keys=True) + "\n")
    return p


def load_s6(out_dir: Path, ds: Dataset) -> Tuple[Dict[Tuple[str, str], List[dict]], Dict[str, str]]:
    data = Path(out_dir) / "data"
    prov = json.loads((data / "S6_timeseries.json").read_text(encoding="utf-8"))
    if prov["vehicle_policy"] != ds.policy or prov["campaign"] != ds.name:
        raise PolicyMixError(f"S6 extract is {prov['campaign']} / {prov['vehicle_policy']}, figures are "
                             f"{ds.name} / {ds.policy}")
    p = data / "S6_timeseries.csv"
    if _sha(p) != prov["csv_sha256"]:
        raise InputChangedError(f"{p} does not match its provenance file")
    series: Dict[Tuple[str, str], List[dict]] = {}
    for r in csv.DictReader(open(p, encoding="utf-8", newline="")):
        series.setdefault((r["series"], r["label"]), []).append(
            {k: (float(r[k]) if k in ("mean", "ci_low", "ci_high") and r[k] != "" else r[k]) for k in r})
    # keys relative to the figure folder, so the manifest does not depend on where the folder is
    return series, {"data/S6_timeseries.csv": _sha(p), "data/S6_timeseries.json": _sha(data / "S6_timeseries.json")}


# ================================================================== drawing helpers
def _style():
    plt.rcdefaults()
    plt.rcParams.update({
        "font.family": FONT, "font.size": 11, "axes.titlesize": 12, "axes.titleweight": "bold",
        "axes.labelsize": 11, "axes.labelcolor": INK2, "axes.edgecolor": MUTED, "axes.linewidth": 0.8,
        "xtick.color": INK2, "ytick.color": INK2, "xtick.labelsize": 10, "ytick.labelsize": 10,
        "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "axes.grid.axis": "y",
        "grid.color": GRID, "grid.linewidth": 0.8, "grid.linestyle": "-", "axes.axisbelow": True,
        "legend.frameon": False, "legend.fontsize": 10, "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE, "text.color": INK,
        "svg.hashsalt": FIG_VERSION, "svg.fonttype": "path", "pdf.fonttype": 42,
        "errorbar.capsize": 3, "lines.linewidth": 2.0, "lines.solid_capstyle": "round",
    })


class Fig:
    """A figure plus the values it plots (for figures_values.csv and the tests)."""

    def __init__(self, fid: str, title: str, subtitle: str, source: str, ds: Dataset, nrows=1, ncols=1,
                 figsize=FIGSIZE, **kw):
        self.fid, self.title, self.source, self.policy = fid, title, source, ds.policy
        self.fig, axes = plt.subplots(nrows, ncols, figsize=figsize, **kw)
        self.axes = list(axes.flat) if hasattr(axes, "flat") else [axes]
        self.values: List[dict] = []
        h = self.fig.get_figheight()
        self.h = h
        tag = ("STAGE 14 OFFICIAL CAMPAIGN  ·  LEGACY-STUDY VEHICLE POLICY (HISTORICAL RESULTS)"
               if ds.policy == LEGACY_STUDY.name else f"{ds.name.upper()}  ·  {ds.policy.upper()} VEHICLE POLICY")
        self.fig.text(0.01, 1 - 0.12 / h, tag, ha="left", va="top", fontsize=8.5, color=INK2, fontweight="bold",
                      bbox={"boxstyle": "round,pad=0.3", "facecolor": "#f0efec", "edgecolor": "none"})
        self.fig.text(0.01, 1 - 0.42 / h, title, ha="left", va="top", fontsize=15, fontweight="bold", color=INK)
        self.fig.text(0.01, 1 - 0.8 / h, subtitle, ha="left", va="top", fontsize=10.5, color=INK2, wrap=True)
        self.fig.text(0.01, 0.08 / h, f"Source: {source}  ·  {ds.policy_text}", ha="left", va="bottom",
                      fontsize=8.5, color=MUTED)

    def layout(self, bottom_in: float = 0.35, top_in: float = 1.25):
        self.fig.tight_layout(rect=(0, bottom_in / self.h, 1, 1 - top_in / self.h))

    def add(self, panel: str, scenario: str, label: str, metric: str, value: float,
            ci_low: Optional[float] = None, ci_high: Optional[float] = None, n: Optional[int] = None, unit=""):
        for v in (value, ci_low, ci_high):
            if v is not None and not math.isfinite(v):
                raise ValueError(f"{self.fid}: non-finite value for {scenario} {label} {metric}")
        self.values.append({"figure": self.fid, "panel": panel, "scenario": scenario, "label": label,
                            "metric": metric, "unit": unit, "value": value, "ci_low": ci_low,
                            "ci_high": ci_high, "n": n, "vehicle_policy": self.policy, "source": self.source})


def _bars(ax, fig: Fig, ds: Dataset, scenarios: Sequence[str], labels: Sequence[str], metric: str,
          unit: str, panel: str = "all", show_values=False, fmt="{:.0f}", width=0.8):
    """Grouped bars (mean over seeds) with 95% CI error bars, starting at zero."""
    k = len(labels)
    bw = min(width / k, 0.28)
    xs = range(len(scenarios))
    top = 0.0
    for j, lb in enumerate(labels):
        for i, sc in enumerate(scenarios):
            if not ds.has(sc, lb, metric):
                continue
            m, lo, hi, n = ds.mean_ci(sc, lb, metric)
            x = i + (j - (k - 1) / 2) * bw
            ax.bar(x, m, bw, color=COLORS[lb], edgecolor=SURFACE, linewidth=1.0, hatch=HATCH.get(lb),
                   label=lb if i == 0 else None, zorder=2)
            if lo is not None and hi is not None:
                ax.errorbar(x, m, yerr=[[m - lo], [hi - m]], fmt="none", ecolor=INK2, elinewidth=1.0,
                            capsize=3, zorder=3)
            top = max(top, hi if hi is not None else m)
            if show_values:
                ax.annotate(fmt.format(m), (x, hi if hi is not None else m), xytext=(0, 3),
                            textcoords="offset points", ha="center", va="bottom", fontsize=8.5, color=INK)
            fig.add(panel, sc, lb, metric, m, lo, hi, n, unit)
    ax.set_xticks(list(xs))
    ax.set_xlim(-0.6, len(scenarios) - 0.4)
    ax.set_ylim(0, top * (1.14 if show_values else 1.06) if top > 0 else 1)
    return top


def _scenario_ticks(ax, scenarios, names=False):
    ax.set_xticks(range(len(scenarios)))
    import textwrap
    ax.set_xticklabels([f"{s}\n" + "\n".join(textwrap.wrap(SCENARIO_NAMES[s], 11, break_long_words=False)) if names else s
                        for s in scenarios], fontsize=8.5 if names else 10)
    ax.tick_params(axis="x", length=0)


def _legend_handles(labels):
    return [Patch(facecolor=COLORS[lb], edgecolor=SURFACE, hatch=HATCH.get(lb), label=lb) for lb in labels]


def _small_multiples(fid, title, subtitle, ds: Dataset, metric, unit, ylabel, fmt="{:.0f}",
                     scenarios=SCENARIOS, labels_for=lambda sc: CORE, ncols=4, note=None):
    nrows = math.ceil((len(scenarios) + 1) / ncols)
    f = Fig(fid, title, subtitle, f"{ds.source}/aggregates.csv ({metric}; n = 10 seeds, mean ± 95 % CI)", ds,
            nrows, ncols, figsize=(12.0, 3.0 * nrows + 1.1))
    for ax, sc in zip(f.axes, scenarios):
        labels = labels_for(sc)
        _bars(ax, f, ds, [sc], labels, metric, unit, sc, show_values=True, fmt=fmt, width=0.9)
        ax.set_xticks([])
        ax.set_title(f"{sc} · {SCENARIO_NAMES[sc]}", loc="left", fontsize=10.5)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xlim(-0.55, 0.55)
    for ax in f.axes[len(scenarios):]:
        ax.axis("off")
    leg_ax = f.axes[len(scenarios)]
    all_labels = [lb for lb in ("FT-A", "FT-B", "Smart-T1", "FT-TOD", "Smart-T1-noWMAX")
                  if any(lb in labels_for(sc) for sc in scenarios)]
    leg_ax.legend(handles=_legend_handles(all_labels), loc="upper left", fontsize=10.5,
                  title="Controller", title_fontsize=10.5, alignment="left")
    txt = "Each panel has its own y-scale,\nstarting at zero.\nError bars: 95 % CI over 10 seeds."
    if note:
        txt += "\n" + note
    leg_ax.text(0.0, 0.35, txt, transform=leg_ax.transAxes, fontsize=9, color=INK2, va="top")
    f.layout()
    return f


def _core_or_tod(sc):
    return CORE + ("FT-TOD",) if sc == "S6" else CORE


# ================================================================== the figures
def fig_mean_delay(ds, ctx):
    return _small_multiples(
        "F01_mean_delay_by_scenario", "Mean control delay by scenario",
        "Average control delay per vehicle (s/veh) · FT-A, FT-B, Smart-T1 (and FT-TOD in S6)",
        ds, "delay_mean_s", "s/veh", "Mean control delay (s/veh)", labels_for=_core_or_tod,
        note="FT-B uses the FT-A plan in S1, S3, S5.")


def fig_p95(ds, ctx):
    return _small_multiples(
        "F02_p95_delay", "95th-percentile control delay",
        "Delay of the worst-served 5 % of vehicles (s/veh), mean of the per-run P95",
        ds, "delay_p95_s", "s/veh", "P95 control delay (s/veh)", labels_for=_core_or_tod)


def fig_max(ds, ctx):
    return _small_multiples(
        "F03_max_delay", "Maximum control delay",
        "Largest single-vehicle control delay per run (s/veh), mean over seeds",
        ds, "delay_max_s", "s/veh", "Max control delay (s/veh)", labels_for=_core_or_tod)


def fig_throughput(ds, ctx):
    f = Fig("F04_throughput_unfinished", "Throughput and unfinished vehicles",
            "Top: vehicles crossing in the measurement hour (veh/h), with the hour's arrivals as a marker. "
            "Bottom: measured vehicles still waiting when the run ended.",
            f"{ds.source}/aggregates.csv (throughput_veh_h, unfinished) and runs.csv (arrivals)", ds, 2, 1,
            figsize=(12.0, 8.0), gridspec_kw={"height_ratios": [1.6, 1.0]})
    ax, bx = f.axes
    _bars(ax, f, ds, SCENARIOS, CORE, "throughput_veh_h", "veh/h", "throughput")
    arr_label_done = False
    for i, sc in enumerate(SCENARIOS):
        a = [v for _, v in ds.per_seed(sc, "FT-A", "arrivals")]
        c = mean_ci95(a)
        ax.plot([i - 0.42, i + 0.42], [c["mean"]] * 2, color=INK, linewidth=1.4, zorder=4,
                label=None if arr_label_done else "Arrivals in the hour (demand)")
        arr_label_done = True
        f.add("throughput", sc, "arrivals", "arrivals_per_hour", c["mean"], c["low"], c["high"], c["n"], "veh/h")
    ax.set_ylabel("Throughput (veh/h)")
    ax.set_ylim(0, max(v["value"] for v in f.values) * 1.12)
    _scenario_ticks(ax, SCENARIOS)
    h = _legend_handles(CORE) + [Line2D([0], [0], color=INK, lw=1.4, label="Arrivals in the hour (demand)")]
    ax.legend(handles=h, loc="upper left", ncols=4)
    _bars(bx, f, ds, SCENARIOS, CORE, "unfinished", "veh", "unfinished")
    bx.set_ylabel("Unfinished vehicles (veh)")
    _scenario_ticks(bx, SCENARIOS)
    bx.set_xlabel("Scenario")
    f.layout()
    return f


def fig_queue(ds, ctx):
    return _small_multiples(
        "F05_mean_queue", "Average queue length",
        "Mean number of queued vehicles per movement (veh), 1 s samples over the measurement hour "
        "(includes vehicles held outside the 150 m road)",
        ds, "queue_mean_per_movement_veh", "veh", "Mean queue (veh/movement)", fmt="{:.1f}",
        labels_for=_core_or_tod)


def fig_fairness(ds, ctx):
    f = Fig("F06_jain_fairness", "Fairness between movements (Jain's index)",
            "Jain's index of the 8 movements' mean delays: 1 = every movement served equally; "
            "lower = some movements wait much longer than others. Higher is more even service.",
            f"{ds.source}/aggregates.csv (jain_fairness; n = 10 seeds, mean ± 95 % CI)", ds)
    ax = f.axes[0]
    _bars(ax, f, ds, SCENARIOS, CORE, "jain_fairness", "index")
    ax.set_ylim(0, 1.05)
    ax.axhline(1.0, color=MUTED, lw=0.8, zorder=1)
    ax.set_ylabel("Jain's fairness index (0-1, higher = more even)")
    _scenario_ticks(ax, SCENARIOS, names=True)
    ax.legend(handles=_legend_handles(CORE), loc="lower right", bbox_to_anchor=(1.0, 1.0), ncols=3)
    f.layout()
    return f


def fig_long_waits(ds, ctx):
    f = Fig("F07a_long_waits", "Vehicles delayed more than 90 s",
            "Measured vehicles per run with control delay > 90 s (study G7 threshold)",
            f"{ds.source}/aggregates.csv (vehicles_control_delay_over_90s; n = 10 seeds, mean ± 95 % CI)", ds)
    ax = f.axes[0]
    _bars(ax, f, ds, SCENARIOS, CORE, "vehicles_control_delay_over_90s", "veh")
    ax.set_ylabel("Vehicles with delay > 90 s (veh per run)")
    _scenario_ticks(ax, SCENARIOS, names=True)
    ax.legend(handles=_legend_handles(CORE), loc="upper left")
    f.layout()
    return f


S10_LABELS = ("FT-A", "FT-B", "Smart-T1", "Smart-T1-noWMAX")


def fig_s10_wmax(ds, ctx):
    f = Fig("F07b_S10_starvation_WMAX", "S10: starvation hold (W_MAX) is a threshold, not a hard bound",
            "Left: longest detector-call wait per run (s), one dot per seed. Right: mean control delay. "
            "Smart-T1-noWMAX = Smart-T1 with the W_MAX hold switched off (study G7 extra).",
            f"{ds.source}/runs.csv (detector_call_wait_max_s, per seed) and aggregates.csv (delay_mean_s)", ds,
            1, 2, gridspec_kw={"width_ratios": [1.35, 1.0]})
    ax, bx = f.axes
    top = 0
    for i, lb in enumerate(S10_LABELS):
        pts = ds.per_seed("S10", lb, "detector_call_wait_max_s")
        for j, (seed, v) in enumerate(pts):
            ax.scatter(i + (j - 4.5) * 0.045, v, s=42, color=COLORS[lb], edgecolor=SURFACE, linewidth=1.2,
                       zorder=3, hatch=None)
            f.add("call_wait", "S10", lb, f"detector_call_wait_max_s seed {seed}", v, None, None, 1, "s")
            top = max(top, v)
        c = mean_ci95([v for _, v in pts])
        ax.plot([i - 0.3, i + 0.3], [c["mean"]] * 2, color=INK, lw=1.4, zorder=4)
    ax.axhline(90, color=INK2, lw=1.2, ls=(0, (4, 3)), zorder=2)
    ax.annotate("W_MAX = 90 s (hold threshold)", (0.99, 90), xycoords=ax.get_yaxis_transform(), xytext=(0, 4),
                textcoords="offset points", ha="right", va="bottom", fontsize=9, color=INK2)
    ax.set_xticks(range(len(S10_LABELS)))
    ax.set_xticklabels(S10_LABELS)
    ax.tick_params(axis="x", length=0)
    ax.set_ylim(0, top * 1.1)
    ax.set_ylabel("Longest detector-call wait per run (s)")
    ax.set_title("Detector-call waits exceed 90 s", loc="left")
    ax.text(0.0, -0.16, "Black bar: mean of the 10 seeds.", transform=ax.transAxes, fontsize=8.5, color=INK2)
    _bars(bx, f, ds, ["S10"], S10_LABELS, "delay_mean_s", "s/veh", "delay", show_values=True, width=0.9)
    bx.set_xticks([])
    bx.set_xlim(-0.55, 0.55)
    bx.set_ylabel("Mean control delay (s/veh)")
    bx.set_title("…but removing W_MAX is much worse", loc="left")
    bx.legend(handles=_legend_handles(S10_LABELS), loc="upper left", fontsize=9)
    f.layout()
    return f


def fig_signal_changes(ds, ctx):
    f = Fig("F08a_signal_changes", "Signal changes per hour",
            "Green onsets of the 8 signal heads in the measurement hour (fewer = calmer, longer greens; "
            "not by itself better or worse)",
            f"{ds.source}/aggregates.csv (signal_changes_per_hour; n = 10 seeds, mean ± 95 % CI)", ds)
    ax = f.axes[0]
    _bars(ax, f, ds, SCENARIOS, CORE, "signal_changes_per_hour", "1/h")
    ax.set_ylabel("Signal changes (green onsets per hour)")
    _scenario_ticks(ax, SCENARIOS, names=True)
    ax.legend(handles=_legend_handles(CORE), loc="upper left", ncols=3)
    f.layout()
    return f


def fig_green_util(ds, ctx):
    f = Fig("F08b_green_utilization", "Green utilization",
            "Share of displayed green (incl. flashing green) used at the saturation headway, mean of the "
            "8 movements: n × h / G",
            f"{ds.source}/aggregates.csv (green_utilization_mean; n = 10 seeds, mean ± 95 % CI)", ds)
    ax = f.axes[0]
    _bars(ax, f, ds, SCENARIOS, CORE, "green_utilization_mean", "fraction")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("Green utilization (fraction of green time)")
    _scenario_ticks(ax, SCENARIOS, names=True)
    ax.legend(handles=_legend_handles(CORE), loc="lower right", bbox_to_anchor=(1.0, 1.0), ncols=3)
    f.layout()
    return f


def _verdict(p: Mapping[str, float]) -> str:
    if p["high"] < 0:
        return "lower"
    if p["low"] > 0:
        return "higher"
    return "no significant difference"


def _forest(fid, title, subtitle, ds: Dataset, ref: str, cmp: str, xlabel: str, note: str):
    f = Fig(fid, title, subtitle, f"{ds.source}/paired.csv (delay_mean_s, same-seed pairs, n = 10, "
            "95 % CI, unadjusted)", ds)
    ax = f.axes[0]
    ax.grid(axis="x", color=GRID)
    ax.grid(axis="y", visible=False)
    ys = list(range(len(SCENARIOS)))[::-1]
    lo_all, hi_all = 0.0, 0.0
    marks = {"lower": "o", "higher": "s", "no significant difference": "D"}
    for y, sc in zip(ys, SCENARIOS):
        p = ds.pair(sc, ref, cmp)
        v = _verdict(p)
        ax.plot([p["low"], p["high"]], [y, y], color=INK2, lw=1.6, zorder=2)
        ax.scatter(p["diff"], y, s=70, marker=marks[v], color=COLORS[cmp], edgecolor=SURFACE, linewidth=1.5,
                   zorder=3)
        pct = 100 * p["diff"] / p["a"]
        ax.text(1.02, y, f"{p['diff']:+.1f} s/veh  ({pct:+.0f} %)", transform=ax.get_yaxis_transform(),
                va="center", ha="left", fontsize=9.5, color=INK)
        ax.text(1.34, y, {"no significant difference": "includes 0"}.get(v, v), transform=ax.get_yaxis_transform(),
                va="center", ha="left", fontsize=9.5, color=INK2)
        lo_all, hi_all = min(lo_all, p["low"]), max(hi_all, p["high"])
        f.add("paired", sc, f"{cmp} - {ref}", "delay_mean_s difference", p["diff"], p["low"], p["high"], p["n"],
              "s/veh")
    ax.axvline(0, color=INK, lw=1.0, zorder=1)
    span = hi_all - lo_all
    ax.set_xlim(lo_all - 0.05 * span, hi_all + 0.08 * span)
    ax.text(1.02, 1.01, f"{cmp} − {ref}", transform=ax.transAxes, ha="left", va="bottom", fontsize=9.5,
            fontweight="bold", color=INK)
    ax.text(1.34, 1.01, "95 % CI", transform=ax.transAxes, ha="left", va="bottom", fontsize=9.5,
            fontweight="bold", color=INK)
    ax.set_yticks(ys)
    ax.set_yticklabels([f"{sc} · {SCENARIO_NAMES[sc]}" for sc in SCENARIOS])
    ax.tick_params(axis="y", length=0)
    ax.set_xlabel(f"{xlabel}\n◀ {cmp} lower delay   |   {cmp} higher delay ▶")
    h = [Line2D([0], [0], marker=marks[k], color="none", markerfacecolor=COLORS[cmp], markeredgecolor=SURFACE,
                markersize=9, label=f"CI {d}") for k, d in
         (("lower", "entirely below 0: lower delay"), ("higher", "entirely above 0: higher delay"),
          ("no significant difference", "includes 0: no significant difference"))]
    f.fig.legend(handles=h, loc="lower center", fontsize=9, ncols=3, bbox_to_anchor=(0.5, 0.55 / f.h))
    f.fig.text(0.01, 0.32 / f.h, note, ha="left", va="bottom", fontsize=8.5, color=INK2)
    f.layout(bottom_in=1.0)
    return f


def fig_vs_fta(ds, ctx):
    return _forest("F09_SmartT1_vs_FTA", "Smart-T1 vs FT-A: difference in mean control delay",
                   "Smart-T1 − FT-A on the same seeds (s/veh). Left of zero: Smart-T1 has lower delay. "
                   "All scenarios shown, including those where Smart-T1 is worse.",
                   ds, "FT-A", "Smart-T1", "Paired difference in mean control delay, Smart-T1 − FT-A (s/veh)",
                   "Line: 95 % CI of the paired difference (t-interval, 10 seeds, not adjusted for multiple "
                   "comparisons). Percentages are relative to FT-A.")


def fig_vs_ftb(ds, ctx):
    return _forest("F10_SmartT1_vs_FTB", "Smart-T1 vs FT-B: difference in mean control delay",
                   "Smart-T1 − FT-B on the same seeds (s/veh). FT-B is the per-scenario Webster plan "
                   "(FT-B = FT-A in S1, S3, S5).",
                   ds, "FT-B", "Smart-T1", "Paired difference in mean control delay, Smart-T1 − FT-B (s/veh)",
                   "Classification is only the position of the 95 % CI relative to zero (not adjusted for "
                   "multiple comparisons); no scoring. Percentages are relative to FT-B.")


def fig_vs_study(ds, ctx):
    tun = ctx["tuning"]
    f = Fig("F11_SmartT1_vs_SmartStudy_historical",
            "Historical: Smart-T1 vs Smart-study (tuning vs final evaluation)",
            "Both controllers as run in Stage 14 under the legacy-study vehicle policy. Left: S1 tuning "
            "seeds 101-105 (selection). Right: evaluation seeds 1-10, all scenarios.",
            f"{_rel(tun['folder'])}/selection.json and {ds.source}/paired.csv (Smart-study vs Smart-T1)", ds,
            1, 2, gridspec_kw={"width_ratios": [0.8, 1.6]})
    ax, bx = f.axes
    sel = tun["selection"]
    pv = sel["paired_vs_study"]
    for i, (lb, m) in enumerate((("Smart-study", sel["study_delay_mean_s"]), ("Smart-T1", sel["delay_mean_s"]))):
        ax.bar(i, m, 0.55, color=COLORS[lb], edgecolor=SURFACE, zorder=2)
        ax.annotate(f"{m:.1f}", (i, m), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=9)
        f.add("tuning", "S1 (seeds 101-105)", lb, "delay_mean_s", m, None, None, 5, "s/veh")
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["Smart-study", "Smart-T1"])
    ax.tick_params(axis="x", length=0)
    ax.set_ylim(0, max(sel["study_delay_mean_s"], sel["delay_mean_s"]) * 1.25)
    ax.set_ylabel("Mean control delay, S1 tuning seeds (s/veh)")
    ax.set_title("Tuning (S1, 5 seeds)", loc="left")
    ax.text(0.5, 0.93, f"Smart-T1 − Smart-study = {pv['mean_diff']:+.2f} s\n95 % CI [{pv['low']:.2f}, "
                       f"{pv['high']:.2f}]", transform=ax.transAxes, ha="center", va="top", fontsize=9, color=INK)
    f.add("tuning", "S1 (seeds 101-105)", "Smart-T1 - Smart-study", "delay_mean_s difference",
          pv["mean_diff"], pv["low"], pv["high"], pv["n"], "s/veh")
    ys = list(range(len(SCENARIOS)))[::-1]
    lo_all, hi_all = 0.0, 0.0
    nsig = 0
    for y, sc in zip(ys, SCENARIOS):
        p = ds.pair(sc, "Smart-study", "Smart-T1")
        v = _verdict(p)
        nsig += v != "no significant difference"
        bx.plot([p["low"], p["high"]], [y, y], color=INK2, lw=1.6, zorder=2)
        bx.scatter(p["diff"], y, s=60, color=COLORS["Smart-T1"], edgecolor=SURFACE, linewidth=1.5, zorder=3,
                   marker={"lower": "o", "higher": "s"}.get(v, "D"))
        bx.annotate(f"{p['diff']:+.1f} · {v}", (p["high"], y), xytext=(6, 0), textcoords="offset points",
                    va="center", fontsize=8.5)
        lo_all, hi_all = min(lo_all, p["low"]), max(hi_all, p["high"])
        f.add("evaluation", sc, "Smart-T1 - Smart-study", "delay_mean_s difference", p["diff"], p["low"],
              p["high"], p["n"], "s/veh")
    bx.axvline(0, color=INK, lw=1.0)
    span = hi_all - lo_all
    bx.set_xlim(lo_all - 0.05 * span, hi_all + 0.7 * span)
    bx.set_yticks(ys)
    bx.set_yticklabels(SCENARIOS)
    bx.tick_params(axis="y", length=0)
    bx.grid(axis="x", color=GRID)
    bx.grid(axis="y", visible=False)
    bx.set_xlabel("Smart-T1 − Smart-study, mean control delay (s/veh)")
    bx.set_title(f"Evaluation (seeds 1-10): CI excludes 0 in {nsig} of {len(SCENARIOS)} scenarios", loc="left")
    f.fig.text(0.01, 0.32 / f.h, "The tuning gain on S1 (seeds 101-105) was not robust: on the evaluation seeds "
               "most differences are small and their 95 % CI includes zero (S9 is worse). Unadjusted CIs.",
               ha="left", va="bottom", fontsize=8.5, color=INK2)
    f.layout(bottom_in=0.65)
    return f


def fig_s6(ds, ctx):
    series = ctx["s6"]
    f = Fig("F12_S6_time_varying_demand", "S6: demand shifts sides at t = 1800 s",
            "Top: total queued vehicles (30 s bins). Bottom: mean control delay by arrival time (5 min "
            "windows). Mean of 10 seeds; bands/bars = 95 % CI.",
            f"{ctx['s6_source']} (from the official S6 runs, fingerprint-checked)", ds, 2, 1, sharex=True,
            figsize=(12.0, 8.0), gridspec_kw={"height_ratios": [1.3, 1.0]})
    ax, bx = f.axes
    order = ("FT-A", "FT-B", "Smart-T1", "FT-TOD")
    qtop = 0
    for lb in order:
        rows = series[("queue_total_veh", lb)]
        t = [(float(r["t_start_s"]) + float(r["t_end_s"])) / 2 for r in rows]
        m = [r["mean"] for r in rows]
        ax.fill_between(t, [r["ci_low"] for r in rows], [r["ci_high"] for r in rows], color=COLORS[lb],
                        alpha=0.12, linewidth=0)
        ax.plot(t, m, color=COLORS[lb], lw=2, label=lb)
        qtop = max(qtop, max(r["ci_high"] for r in rows))
        for r, tt in zip(rows, t):
            f.add("queue", "S6", lb, f"queue_total_veh t={int(tt)}s", r["mean"], r["ci_low"], r["ci_high"],
                  int(r["n_seeds"]), "veh")
    ax.set_ylim(0, qtop * 1.08)
    ax.set_ylabel("Total queued vehicles (veh)")
    ax.legend(loc="upper left", ncols=4)
    bw = 300 / (len(order) + 1)
    dtop = 0
    for j, lb in enumerate(order):
        rows = series[("delay_by_arrival_s", lb)]
        for r in rows:
            x = float(r["t_start_s"]) + (j + 1) * bw
            bx.bar(x, r["mean"], bw, color=COLORS[lb], edgecolor=SURFACE, linewidth=0.8, zorder=2)
            bx.errorbar(x, r["mean"], yerr=[[r["mean"] - r["ci_low"]], [r["ci_high"] - r["mean"]]], fmt="none",
                        ecolor=INK2, elinewidth=0.9, capsize=2, zorder=3)
            dtop = max(dtop, r["ci_high"])
            f.add("delay", "S6", lb, f"delay_by_arrival {r['t_start_s']}-{r['t_end_s']}s", r["mean"],
                  r["ci_low"], r["ci_high"], int(r["n_seeds"]), "s/veh")
    bx.set_ylim(0, dtop * 1.08)
    bx.set_ylabel("Mean control delay (s/veh)")
    bx.set_xlabel("Simulation time in the measurement hour (s) — bottom: vehicle arrival time")
    bx.set_xlim(0, 3600)
    bx.set_xticks(range(0, 3601, 600))
    for a in (ax, bx):
        a.axvline(S6_SHIFT_S, color=INK, lw=1.2, ls=(0, (4, 3)), zorder=1)
    ax.text(S6_SHIFT_S + 20, qtop * 1.0, "demand shift: N-S heavy → E-W heavy", fontsize=9, color=INK,
            va="top")
    f.layout()
    return f


def fig_key(ds, ctx):
    f = Fig("F13_key_scenarios_mean_delay", "Key presentation scenarios: mean control delay",
            "S4 low demand · S6 changing demand · S7b oversaturated · S8 left-heavy · S10 low-demand "
            "movement (starvation hold). Each panel has its own y-scale from zero.",
            f"{ds.source}/aggregates.csv (delay_mean_s; n = 10 seeds, mean ± 95 % CI)", ds, 1, len(KEY_SCENARIOS))
    for ax, sc in zip(f.axes, KEY_SCENARIOS):
        _bars(ax, f, ds, [sc], _core_or_tod(sc), "delay_mean_s", "s/veh", sc, show_values=True, width=0.9)
        ax.set_xticks([])
        ax.set_xlim(-0.55, 0.55)
        ax.set_title(f"{sc}\n{SCENARIO_NAMES[sc]}", fontsize=10.5)
    f.axes[0].set_ylabel("Mean control delay (s/veh)")
    f.fig.legend(handles=_legend_handles(CORE + ("FT-TOD",)), loc="lower center", ncols=4,
                 bbox_to_anchor=(0.5, 0.3 / f.h))
    f.layout(bottom_in=0.9)
    return f


# id -> (builder, metric, interpretation template key)
FIGURES: Tuple[Tuple[str, Callable, str], ...] = (
    ("F01_mean_delay_by_scenario", fig_mean_delay, "Mean control delay (s/veh), 95 % CI"),
    ("F02_p95_delay", fig_p95, "P95 control delay (s/veh), 95 % CI"),
    ("F03_max_delay", fig_max, "Maximum control delay per run (s/veh), mean and 95 % CI over seeds"),
    ("F04_throughput_unfinished", fig_throughput, "Throughput (veh/h) with arrivals; unfinished vehicles"),
    ("F05_mean_queue", fig_queue, "Mean queue per movement (veh), 95 % CI"),
    ("F06_jain_fairness", fig_fairness, "Jain's fairness index (0-1)"),
    ("F07a_long_waits", fig_long_waits, "Vehicles with control delay > 90 s per run"),
    ("F07b_S10_starvation_WMAX", fig_s10_wmax, "S10 longest detector-call wait per seed (s); mean delay"),
    ("F08a_signal_changes", fig_signal_changes, "Signal changes (green onsets) per hour"),
    ("F08b_green_utilization", fig_green_util, "Green utilization (fraction)"),
    ("F09_SmartT1_vs_FTA", fig_vs_fta, "Paired difference Smart-T1 − FT-A in mean delay (s/veh)"),
    ("F10_SmartT1_vs_FTB", fig_vs_ftb, "Paired difference Smart-T1 − FT-B in mean delay (s/veh)"),
    ("F11_SmartT1_vs_SmartStudy_historical", fig_vs_study, "Smart-T1 vs Smart-study mean delay (tuning, evaluation)"),
    ("F12_S6_time_varying_demand", fig_s6, "S6 queue over time and delay by arrival window"),
    ("F13_key_scenarios_mean_delay", fig_key, "Mean control delay in S4, S6, S7b, S8, S10"),
)


# ================================================================== observations (from the data)
def observations(ds: Dataset, ctx) -> Dict[str, List[str]]:
    o: Dict[str, List[str]] = {}
    m = lambda sc, lb, k="delay_mean_s": ds.mean_ci(sc, lb, k)[0]  # noqa: E731
    def best_of(sc):
        lows = sorted(_core_or_tod(sc), key=lambda lb: m(sc, lb))
        tied = [lb for lb in lows if abs(m(sc, lb) - m(sc, lows[0])) < 1e-9]
        return " = ".join(tied)
    best = {sc: best_of(sc) for sc in SCENARIOS}
    o["F01_mean_delay_by_scenario"] = [
        "Lowest mean delay per scenario: " + ", ".join(f"{sc} {best[sc]}" for sc in SCENARIOS) + ".",
        f"S4 (very low demand): Smart-T1 {m('S4', 'Smart-T1'):.1f} s vs FT-A {m('S4', 'FT-A'):.1f} s and "
        f"FT-B {m('S4', 'FT-B'):.1f} s.",
        f"S6: Smart-T1 {m('S6', 'Smart-T1'):.1f} s vs FT-A {m('S6', 'FT-A'):.1f} s, FT-B {m('S6', 'FT-B'):.1f} s; "
        f"the two-plan FT-TOD reaches {m('S6', 'FT-TOD'):.1f} s.",
        f"S1 and S5: Smart-T1 is higher than FT-A/FT-B ({m('S1', 'Smart-T1'):.1f} vs {m('S1', 'FT-A'):.1f} s; "
        f"{m('S5', 'Smart-T1'):.1f} vs {m('S5', 'FT-A'):.1f} s)."]
    o["F02_p95_delay"] = [f"S6 P95: Smart-T1 {m('S6', 'Smart-T1', 'delay_p95_s'):.0f} s vs FT-A "
                          f"{m('S6', 'FT-A', 'delay_p95_s'):.0f} s, FT-B {m('S6', 'FT-B', 'delay_p95_s'):.0f} s, "
                          f"FT-TOD {m('S6', 'FT-TOD', 'delay_p95_s'):.0f} s."]
    o["F03_max_delay"] = [f"S8 FT-A max {m('S8', 'FT-A', 'delay_max_s'):.0f} s (left-turn demand exceeds "
                          f"FT-A's 8 s left greens) vs Smart-T1 {m('S8', 'Smart-T1', 'delay_max_s'):.0f} s."]
    u = {lb: m("S7b", lb, "unfinished") for lb in CORE}
    t = {lb: m("S7b", lb, "throughput_veh_h") for lb in CORE}
    o["F04_throughput_unfinished"] = [
        "S7b (oversaturated): throughput " + ", ".join(f"{lb} {t[lb]:.0f}" for lb in CORE) + " veh/h; unfinished "
        + ", ".join(f"{lb} {u[lb]:.0f}" for lb in CORE) + " vehicles per run.",
        "Scenarios with unfinished vehicles (mean > 0.5 per run): " + "; ".join(
            f"{lb}: " + (", ".join(sc for sc in SCENARIOS if m(sc, lb, "unfinished") > 0.5) or "none")
            for lb in CORE) + "."]
    o["F05_mean_queue"] = [f"S8 mean queue FT-A {m('S8', 'FT-A', 'queue_mean_per_movement_veh'):.1f} veh vs "
                           f"Smart-T1 {m('S8', 'Smart-T1', 'queue_mean_per_movement_veh'):.1f} veh per movement."]
    jf = lambda sc, lb: m(sc, lb, "jain_fairness")  # noqa: E731
    low = min(((sc, lb) for sc in SCENARIOS for lb in CORE), key=lambda k: jf(*k))
    o["F06_jain_fairness"] = [
        f"Lowest fairness: {low[0]} {low[1]} {jf(*low):.2f}.",
        "Smart-T1 mean index higher than FT-B in " + ", ".join(sc for sc in SCENARIOS if jf(sc, "Smart-T1") >
                                                               jf(sc, "FT-B")) +
        "; lower in " + ", ".join(sc for sc in SCENARIOS if jf(sc, "Smart-T1") < jf(sc, "FT-B")) +
        " (point estimates; see the CIs)."]
    o["F07a_long_waits"] = [f"S1: Smart-T1 {m('S1', 'Smart-T1', 'vehicles_control_delay_over_90s'):.0f} vs FT-A "
                            f"{m('S1', 'FT-A', 'vehicles_control_delay_over_90s'):.0f} vehicles over 90 s per run "
                            "(more long waits with Smart-T1 at moderate balanced demand)."]
    cw = mean_ci95([v for _, v in ds.per_seed("S10", "Smart-T1", "detector_call_wait_max_s")])
    cwn = mean_ci95([v for _, v in ds.per_seed("S10", "Smart-T1-noWMAX", "detector_call_wait_max_s")])
    o["F07b_S10_starvation_WMAX"] = [
        f"Smart-T1 longest call wait in S10: mean {cw['mean']:.0f} s (max "
        f"{max(v for _, v in ds.per_seed('S10', 'Smart-T1', 'detector_call_wait_max_s')):.0f} s): above 90 s "
        "in every seed, so W_MAX = 90 s is a hold threshold, not a hard bound.",
        f"Without W_MAX the longest wait rises to {cwn['mean']:.0f} s and mean delay to "
        f"{m('S10', 'Smart-T1-noWMAX'):.0f} s (Smart-T1 {m('S10', 'Smart-T1'):.0f} s)."]
    sc_ = lambda lb: [m(sc, lb, "signal_changes_per_hour") for sc in SCENARIOS]  # noqa: E731
    o["F08a_signal_changes"] = [
        f"FT-A {min(sc_('FT-A')):.0f}-{max(sc_('FT-A')):.0f}, FT-B {min(sc_('FT-B')):.0f}-{max(sc_('FT-B')):.0f}, "
        f"Smart-T1 {min(sc_('Smart-T1')):.0f}-{max(sc_('Smart-T1')):.0f} green onsets per hour across scenarios."]
    gu = lambda sc, lb: m(sc, lb, "green_utilization_mean")  # noqa: E731
    o["F08b_green_utilization"] = [
        "Smart-T1 utilization below both fixed plans in " + ", ".join(
            sc for sc in SCENARIOS if gu(sc, "Smart-T1") < min(gu(sc, "FT-A"), gu(sc, "FT-B"))) +
        "; above both in " + (", ".join(sc for sc in SCENARIOS if gu(sc, "Smart-T1") > max(gu(sc, "FT-A"),
                                                                                           gu(sc, "FT-B"))) or "none") +
        ". Lower utilization means more of the displayed green passes without a saturated discharge."]
    for fid, ref in (("F09_SmartT1_vs_FTA", "FT-A"), ("F10_SmartT1_vs_FTB", "FT-B")):
        groups: Dict[str, List[str]] = {}
        for sc in SCENARIOS:
            groups.setdefault(_verdict(ds.pair(sc, ref, "Smart-T1")), []).append(sc)
        o[fid] = [f"Smart-T1 {k} than {ref}: {', '.join(v)}." if k != "no significant difference" else
                  f"No significant difference from {ref}: {', '.join(v)}." for k, v in sorted(groups.items())]
    sel = ctx["tuning"]["selection"]
    o["F11_SmartT1_vs_SmartStudy_historical"] = [
        f"Tuning (S1, seeds 101-105): Smart-T1 {sel['delay_mean_s']:.2f} s vs Smart-study "
        f"{sel['study_delay_mean_s']:.2f} s.",
        "Evaluation (seeds 1-10): " + ", ".join(
            f"{sc} {ds.pair(sc, 'Smart-study', 'Smart-T1')['diff']:+.1f} s" for sc in SCENARIOS) +
        "; the improvement is not robust (CI includes 0 in most scenarios, S9 worse)."]
    s6 = ctx["s6"]
    def half(lb, lo, hi):  # noqa: E306
        rows = [r for r in s6[("queue_total_veh", lb)] if lo <= float(r["t_start_s"]) < hi]
        return sum(r["mean"] for r in rows) / len(rows)
    o["F12_S6_time_varying_demand"] = [
        "Mean total queue before / after the shift (veh): " + "; ".join(
            f"{lb} {half(lb, 0, 1800):.0f} / {half(lb, 1800, 3600):.0f}" for lb in S6_LABELS) + ".",
        "FT-A and FT-B keep one plan for the whole hour; with N-S heavy (first half) and E-W heavy (second "
        "half) the heavy axis exceeds that plan's capacity, so their queues grow in each half and only fall "
        "while the previously heavy side clears after the shift. Smart-T1 re-allocates green from detector "
        "calls and FT-TOD switches to a second plan at 1800 s; both keep the total queue roughly flat."]
    o["F13_key_scenarios_mean_delay"] = [
        ", ".join(f"{sc}: " + " / ".join(f"{lb} {m(sc, lb):.0f}" for lb in _core_or_tod(sc)) for sc in KEY_SCENARIOS)
        + " s/veh."]
    return o


# ================================================================== output
def _write_text(path: Path, text: str) -> None:
    """UTF-8 with LF line ends on every OS (Path.write_text would write CRLF on Windows)."""
    Path(path).write_bytes(text.encode("utf-8"))


def _write_csv(path: Path, rows: Sequence[Mapping], fields: Sequence[str]) -> str:
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=list(fields), lineterminator="\n")
    w.writeheader()
    for r in rows:
        w.writerow({k: _fmt(r.get(k)) for k in fields})
    data = buf.getvalue().encode("utf-8")
    Path(path).write_bytes(data)
    return hashlib.sha256(data).hexdigest()


def _fmt(v) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        return f"{v:.6f}".rstrip("0").rstrip(".") if math.isfinite(v) else "nan"
    return str(v)


VALUE_FIELDS = ["figure", "panel", "scenario", "label", "metric", "unit", "value", "ci_low", "ci_high", "n",
                "vehicle_policy", "source"]


def default_out(ds: Dataset) -> Path:
    return FIGURES_ROOT / f"{ds.name}__{ds.policy}"


def _check_out(out: Path, ds: Dataset) -> None:
    man = out / "figures_manifest.json"
    if man.exists():
        old = json.loads(man.read_text(encoding="utf-8"))
        if old.get("vehicle_policy") != ds.policy or old.get("campaign") != ds.name:
            raise PolicyMixError(f"{out} holds figures of {old.get('campaign')} / {old.get('vehicle_policy')}; "
                                 f"{ds.name} / {ds.policy} figures need their own folder")


def _save(f: Fig, out: Path) -> List[str]:
    meta_desc = f"{f.title}. Source: {f.source}. Vehicle policy: {f.policy}."
    files = []
    for ext, meta in (("png", {"Title": f.title, "Description": meta_desc, "Software": None}),
                      ("pdf", {"Title": f.title, "Subject": meta_desc, "Creator": None, "Producer": None,
                               "CreationDate": None}),
                      ("svg", {"Title": f.title, "Description": meta_desc, "Date": None, "Creator": None})):
        p = out / f"{f.fid}.{ext}"
        f.fig.savefig(p, dpi=DPI if ext == "png" else None, metadata=meta)
        files.append(p.name)
    plt.close(f.fig)
    return files


def build_all(campaign_dir: Path = CAMPAIGN_DIR, out: Optional[Path] = None, tuning_dir: Path = TUNING_DIR,
              only: Optional[Sequence[str]] = None, log=print) -> Dict[str, object]:
    ds = load_campaign(campaign_dir)
    tun = load_tuning(tuning_dir)
    if tun["policy"] != ds.policy:
        raise PolicyMixError(f"tuning results are {tun['policy']}, the campaign is {ds.policy}: not mixed")
    out = Path(out) if out else default_out(ds)
    _check_out(out, ds)
    out.mkdir(parents=True, exist_ok=True)
    s6, s6_inputs = load_s6(out, ds)
    ctx = {"tuning": tun, "s6": s6, "s6_source": "data/S6_timeseries.csv (this folder)"}
    _style()
    values, index = [], []
    for fid, builder, metric in FIGURES:
        if only and fid not in only:
            continue
        f = builder(ds, ctx)
        assert f.fid == fid, (f.fid, fid)
        files = _save(f, out)
        values += f.values
        index.append({"id": fid, "title": f.title, "files": files, "metric": metric, "source": f.source,
                      "vehicle_policy": f.policy, "values": len(f.values)})
        log(f"  {fid}: {len(f.values)} values")
    obs = observations(ds, ctx)
    vals_sha = _write_csv(out / "figures_values.csv", values, VALUE_FIELDS)
    inputs = {**ds.inputs, **tun["inputs"], **s6_inputs}
    manifest = {"figures_version": FIG_VERSION, "campaign": ds.name, "campaign_folder": ds.source,
                "vehicle_policy": ds.policy, "lock_sha256": ds.manifest.get("lock_sha256"),
                "inputs_sha256": dict(sorted(inputs.items())), "figures": index,
                "figures_values_csv_sha256": vals_sha, "values": len(values),
                "matplotlib": matplotlib.__version__, "font": FONT, "png_dpi": DPI}
    _write_text(out / "figures_manifest.json", json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    _write_text(out / "README.md", _readme(ds, manifest, obs))
    return {"out": out, "manifest": manifest, "values": values, "observations": obs}


def _readme(ds: Dataset, manifest, obs) -> str:
    legacy = ds.policy == LEGACY_STUDY.name
    L = [f"# Result figures — {ds.name} ({ds.policy})", ""]
    if legacy:
        L += ["> **Historical results.** Every figure here is built from the approved Stage 14 official campaign "
              "(490 runs, seeds 1-10), which was produced with the **legacy-study** vehicle policy (first queued "
              "car at green + 3.9 s, permissive first 2.0 s of yellow). The project's current standard policy "
              "(Stop on Yellow, first car at green onset) was adopted afterwards; these figures are **not** "
              "standard-policy results and must not be presented as such.", ""]
    L += ["Generated by `python -m smarttraffic.make_graphs` (Stage 15) from the committed CSV summaries; "
          "each CSV is verified against the SHA-256 in its manifest. Re-running gives the same "
          "`figures_values.csv` and `figures_manifest.json`.", "",
          f"* Campaign folder: `{ds.source}` · Smart-T1 lock `{manifest['lock_sha256']}`",
          "* Statistics: mean over 10 seeds, 95 % t-interval; paired differences use the same seeds; "
          "no multiple-comparison adjustment (as the study specifies).",
          "* Formats: `.png` (300 dpi) for slides, `.pdf` / `.svg` (vector) for the report.",
          "* Controllers: FT-A (study fixed plan), FT-B (per-scenario Webster plan; equal to FT-A in S1, S3, "
          "S5), Smart-T1 (locked τ 15 s, W_MAX 90 s), FT-TOD (two-plan time-of-day, S6 only), Smart-study "
          "(historical comparison only), Smart-T1-noWMAX (S10 extra only).",
          "* `figures_values.csv`: every plotted value (table view of all figures).", "",
          "| Figure | Files | Data source | Metric | Policy |", "|---|---|---|---|---|"]
    for f in manifest["figures"]:
        L.append(f"| {f['title']} | `{f['id']}`.png/.pdf/.svg | {f['source']} | {f['metric']} | "
                 f"{f['vehicle_policy']} |")
    L += ["", "## Interpretation", ""]
    for f in manifest["figures"]:
        L.append(f"### {f['id']} — {f['title']}")
        for line in obs.get(f["id"], []):
            L.append(f"* {line}")
        L.append("")
    L += ["## Notes", "",
          "* W_MAX = 90 s is a starvation-hold threshold, not a proven upper bound on waiting: detector-call "
          "waits exceed 90 s (F07b), yet removing W_MAX makes S10 substantially worse.",
          "* The Smart-T1 parameters were selected on S1 seeds 101-105; F11 shows that the gain over "
          "Smart-study did not carry over robustly to the evaluation seeds.",
          "* Figures use no truncated axes: every bar axis starts at zero; small multiples state that each "
          "panel has its own scale.", ""]
    return "\n".join(L)


# ================================================================== CLI
def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m smarttraffic.make_graphs", description="Stage 15 result figures")
    ap.add_argument("cmd", nargs="?", default="figures", choices=("figures", "extract-s6"))
    ap.add_argument("--campaign", default=str(CAMPAIGN_DIR), help="official campaign folder (read only)")
    ap.add_argument("--tuning", default=str(TUNING_DIR))
    ap.add_argument("--out", default=None, help="default results/figures/<campaign>__<policy>")
    ap.add_argument("--records", default=None, help="extract-s6: folder of per-run records (default <campaign>/runs)")
    a = ap.parse_args(argv)
    ds = load_campaign(Path(a.campaign))
    out = Path(a.out) if a.out else default_out(ds)
    if a.cmd == "extract-s6":
        _check_out(out, ds)
        p = extract_s6(ds, out, Path(a.records) if a.records else None)
        print(f"wrote {p}")
        return 0
    res = build_all(Path(a.campaign), out, Path(a.tuning))
    print(f"{len(res['manifest']['figures'])} figures ({ds.policy}) -> {res['out']}")
    return 0


if __name__ == "__main__":
    from smarttraffic.make_graphs import main as _package_main
    sys.exit(_package_main())
