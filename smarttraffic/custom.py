"""custom -- Custom / Demo mode (Stage 14b): user-chosen demands, clearly separate from the
official experiments.

    python -m smarttraffic.custom --demand NS=400,SS=400,ES=400,WS=400,NL=120,SL=120,EL=120,WL=120 \
        --controller Smart --seed 1 --measure 600 [--params T1|custom] \
        [--tau 15 --wmax 90 --max-straight 45 --max-left 30] [--yellow study|stop] \
        [--immediate-first] [--out results/custom]

What a Custom/Demo run is
-------------------------
* Eight constant demands (veh/h), a seed, FT-A or Smart, and a measurement duration. The
  study's warm-up (300 s), flush (<= 900 s) and dt (100 ms) are kept.
* Arrivals come from the same generator and the same per-movement random streams as the
  official scenarios (streams depend only on seed and movement, not on the scenario), so a
  custom run with S1's demands and seed reproduces S1's arrivals exactly. They are generated
  in memory and never written to data/arrivals.
* Smart parameter set: "T1" (the approved lock, default) or
  "custom" (T1 with tau, W_MAX and the two max greens changed; everything else locked).
  The study-default set ("study", Smart-study) stays available internally only
  (smart_params_for) for historical comparison; it is not offered to users.
* Vehicle options (advanced): yellow policy "study" (permissive first 2.0 s of yellow, the
  official behaviour) or "stop" (Stop on Yellow, yellow_extension_s = 0.0), and the
  immediate-first-vehicle start (vehicles.py). Official runs refuse both non-defaults.
* Every result is tagged "CUSTOM / DEMO" (metadata "mode": "custom") and is written only
  under results/custom (or another folder that is not an official one). The official
  experiment code never reads it: official runs are built from experiments.RunSpec, which
  cannot express a custom run, and runner.RunConfig refuses custom inputs in official mode.

Demand validation: a number (not bool), finite, >= 0, and < 2400 veh/h (the arrival model
needs 3600/q - 1.5 s > 0); 0 is allowed (no arrivals on that movement). The user interface
caps input at 1800 veh/h (UI_LIMIT_VEH_H). A saturation check warns when the demand is near
or above what any signal plan can serve (see saturation_report).
"""
import argparse
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Dict, Mapping, Optional, Sequence, Tuple, Union

from smarttraffic.conflicts import MOVES

__all__ = ["CUSTOM_TAG", "CUSTOM_SCENARIO", "CUSTOM_CONTROLLERS", "HARD_LIMIT_VEH_H", "UI_LIMIT_VEH_H",
           "YELLOW_POLICY_SECONDS", "YELLOW_POLICY_LABELS", "SMART_PARAM_SETS", "UI_PARAM_SETS", "CUSTOM_SMART_FIELDS",
           "MEASURE_MIN_S", "MEASURE_MAX_S", "CustomInputError", "CustomDemand", "CustomSpec",
           "SaturationReport", "validate_demand", "ui_clamp", "yellow_policy_name", "saturation_report",
           "smart_params_for", "run_custom", "save_custom", "check_output_dir", "CUSTOM_RESULTS_DIR"]

CUSTOM_TAG = "CUSTOM / DEMO"
CUSTOM_SCENARIO = "CUSTOM"
CUSTOM_CONTROLLERS = ("FT-A", "Smart")
HARD_LIMIT_VEH_H = 2400          # rejected at and above (arrival model limit)
UI_LIMIT_VEH_H = 1800            # the user interface never offers more
MEASURE_MIN_S, MEASURE_MAX_S = 60, 7200
YELLOW_POLICY_SECONDS = {"study": 2.0, "stop": 0.0}
YELLOW_POLICY_LABELS = {"study": "Study default (permissive first 2.0 s of yellow)",
                        "stop": "Stop on Yellow (no entry after yellow onset)"}
SMART_PARAM_SETS = ("T1", "study", "custom")      # "study" is internal (historical comparison) only
UI_PARAM_SETS = ("T1", "custom")                  # what the user interface and the command line offer
# the only Smart parameters Custom mode may change, with the UI's ranges (seconds)
CUSTOM_SMART_FIELDS: Dict[str, Tuple[float, float]] = {
    "tau_s": (1.0, 60.0), "w_max_s": (30.0, 300.0),
    "max_green_straight_s": (10.0, 120.0), "max_green_left_s": (8.0, 90.0)}

_ROOT = Path(__file__).resolve().parent.parent
CUSTOM_RESULTS_DIR = _ROOT / "results" / "custom"
_PROTECTED = (_ROOT / "results" / "official", _ROOT / "results" / "tuning", _ROOT / "data" / "arrivals",
              _ROOT / "config")

# saturation flow (veh/h) from the study headways, lost time per cycle and reference cycle
_SAT = {"straight": 3600 / 1.9, "left": 3600 / 2.0}
_LOST_S, _CYCLE_REF_S = 23.0, 120.0
_CAP_Y = 1 - _LOST_S / _CYCLE_REF_S          # 0.808: largest serviceable critical flow ratio


class CustomInputError(ValueError):
    """A Custom/Demo input is invalid (message names the field)."""


# ================================================================ inputs
def _is_num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def validate_demand(value, movement: str = "demand") -> float:
    """A valid custom demand in veh/h, or CustomInputError."""
    if isinstance(value, str):
        try:
            value = float(value.strip())
        except ValueError:
            raise CustomInputError(f"{movement}: {value!r} is not a number") from None
    if not _is_num(value):
        raise CustomInputError(f"{movement}: demand must be a number of veh/h, got {value!r}")
    if not math.isfinite(value):
        raise CustomInputError(f"{movement}: demand must be finite, got {value!r}")
    if value < 0:
        raise CustomInputError(f"{movement}: demand must not be negative, got {value!r}")
    if value >= HARD_LIMIT_VEH_H:
        raise CustomInputError(f"{movement}: {value:g} veh/h is not allowed (must be below "
                               f"{HARD_LIMIT_VEH_H} veh/h: the 1.5 s minimum headway allows at most "
                               f"{HARD_LIMIT_VEH_H} veh/h)")
    return float(value)


def ui_clamp(value: float) -> float:
    """The value the user interface shows/accepts: [0, UI_LIMIT_VEH_H]."""
    return float(min(UI_LIMIT_VEH_H, max(0.0, value)))


def yellow_policy_name(seconds: float) -> str:
    for name, s in YELLOW_POLICY_SECONDS.items():
        if seconds == s:
            return name
    raise CustomInputError(f"yellow_extension_s must be one of {sorted(YELLOW_POLICY_SECONDS.values())} "
                           f"(study default 2.0, stop on yellow 0.0), got {seconds!r}")


@dataclass(frozen=True)
class CustomDemand:
    """Constant demand (veh/h) for the 8 movements, MOVES order."""
    rates: Tuple[Tuple[str, float], ...]

    def __post_init__(self):
        if not isinstance(self.rates, tuple) or [m for m, _ in self.rates] != list(MOVES):
            raise CustomInputError(f"rates must list the 8 movements in the order {MOVES}")
        for m, q in self.rates:
            validate_demand(q, m)

    @classmethod
    def from_mapping(cls, rates: Mapping[str, object]) -> "CustomDemand":
        keys = set(rates)
        if keys != set(MOVES):
            raise CustomInputError(f"demand must give all 8 movements (missing {sorted(set(MOVES) - keys)}, "
                                   f"unknown {sorted(keys - set(MOVES))})")
        return cls(tuple((m, validate_demand(rates[m], m)) for m in MOVES))

    @classmethod
    def uniform(cls, straight: float, left: float) -> "CustomDemand":
        return cls.from_mapping({m: (straight if m[1] == "S" else left) for m in MOVES})

    def rate(self, m: str) -> float:
        return dict(self.rates)[m]

    def as_dict(self) -> Dict[str, float]:
        return {m: q for m, q in self.rates}

    def scenario(self):
        """traffic_gen.Scenario 'CUSTOM' with these constant rates (no events)."""
        from smarttraffic.traffic_gen import Scenario, constant_rate
        raw = {"segments": [{"from": None, "until": None, "rates": self.as_dict()}], "events": []}
        canon = json.dumps(raw, sort_keys=True, separators=(",", ":"))
        return Scenario(CUSTOM_SCENARIO, "Custom / Demo demand", constant_rate(self.as_dict()), (),
                        hashlib.sha256(canon.encode("utf-8")).hexdigest())


# ================================================================ saturation check
@dataclass(frozen=True)
class SaturationReport:
    level: str                         # "ok" | "near" | "over"
    critical_ratio: float              # Y: dual-ring critical flow ratio
    degree: float                      # X = Y / (1 - L/C) with L = 23 s, C = 120 s
    messages: Tuple[str, ...]
    movements_over: Tuple[str, ...]    # movements whose demand alone exceeds their lane's saturation flow

    @property
    def warning(self) -> Optional[str]:
        return None if self.level == "ok" else " ".join(self.messages)


def saturation_report(demand: CustomDemand) -> SaturationReport:
    """Warn when the demand is near or above what any signal plan can serve.

    Critical flow ratio of the conflict model's dual ring: per barrier the larger of
    (left of one approach + opposing straight) and (the other left + its opposing straight);
    Y = N/S barrier + E/W barrier, y = q / saturation flow (1895 veh/h straight, 1800 left).
    With the study's lost time L = 23 s per cycle and a 120 s reference cycle, at most
    Y = 0.808 can be served: X = Y / 0.808. X >= 0.85 near capacity, X >= 1 oversaturated.
    (Official scenarios: S1 X = 0.69, S2/S7a/S10 0.88-0.94, S7b 1.22.)
    """
    y = {m: demand.rate(m) / _SAT["straight" if m[1] == "S" else "left"] for m in MOVES}
    ns = max(y["NL"] + y["SS"], y["SL"] + y["NS"])
    ew = max(y["EL"] + y["WS"], y["WL"] + y["ES"])
    Y = ns + ew
    X = Y / _CAP_Y
    over = tuple(m for m in MOVES if y[m] >= 1.0)
    msgs = []
    if over:
        msgs.append(f"{', '.join(over)}: demand exceeds the lane's saturation flow (queues grow without limit).")
    if X >= 1.0:
        level = "over"
        msgs.append(f"Oversaturated (X = {X:.2f}): demand exceeds intersection capacity; queues will grow and "
                    f"vehicles may be left unfinished.")
    elif X >= 0.85 or over:
        level = "over" if over else "near"
        if not over:
            msgs.append(f"Near capacity (X = {X:.2f}): expect long queues and delays.")
    else:
        level = "ok"
    return SaturationReport(level, Y, X, tuple(msgs), over)


# ================================================================ Smart parameters
def smart_params_for(param_set: str, overrides: Optional[Mapping[str, float]] = None, *, lock_path=None):
    """SmartParams for a Custom run. 'T1' = the approved lock, 'study' = study defaults,
    'custom' = T1 with only CUSTOM_SMART_FIELDS changed (within their UI ranges)."""
    from smarttraffic.controller_smart import SmartParams
    from dataclasses import replace
    if param_set not in SMART_PARAM_SETS:
        raise CustomInputError(f"Smart parameter set must be one of {SMART_PARAM_SETS}, got {param_set!r}")
    overrides = dict(overrides or {})
    if param_set != "custom" and overrides:
        raise CustomInputError(f"parameter overrides need the 'custom' set (got set {param_set!r})")
    if param_set == "study":
        return SmartParams()
    from smarttraffic import tuning as tu
    t1 = tu.load_lock(lock_path or tu.LOCK_FILE)["smart_params"]
    if param_set == "T1":
        return t1
    bad = set(overrides) - set(CUSTOM_SMART_FIELDS)
    if bad:
        raise CustomInputError(f"only {sorted(CUSTOM_SMART_FIELDS)} may be changed, got {sorted(bad)}")
    clean = {}
    for k, v in overrides.items():
        lo, hi = CUSTOM_SMART_FIELDS[k]
        if not _is_num(v) or not math.isfinite(v) or not lo <= v <= hi:
            raise CustomInputError(f"{k} must be a number in [{lo:g}, {hi:g}] s, got {v!r}")
        clean[k] = float(v)
    try:
        return replace(t1, **clean)
    except ValueError as e:                     # SmartParams' own checks (e.g. max >= min green)
        raise CustomInputError(str(e)) from None


# ================================================================ the spec
@dataclass(frozen=True)
class CustomSpec:
    demand: CustomDemand
    seed: int = 1
    controller: str = "Smart"
    measure_s: int = 3600
    param_set: str = "T1"
    smart_overrides: Tuple[Tuple[str, float], ...] = ()
    yellow_policy: str = "study"
    immediate_first_vehicle: bool = False

    def __post_init__(self):
        if not isinstance(self.demand, CustomDemand):
            raise CustomInputError("demand must be a CustomDemand")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or self.seed < 0:
            raise CustomInputError(f"seed must be an int >= 0, got {self.seed!r}")
        if self.controller not in CUSTOM_CONTROLLERS:
            raise CustomInputError(f"Custom/Demo supports {CUSTOM_CONTROLLERS} only (no FT-B custom planning), "
                                   f"got {self.controller!r}")
        if isinstance(self.measure_s, bool) or not isinstance(self.measure_s, int) or \
                not MEASURE_MIN_S <= self.measure_s <= MEASURE_MAX_S:
            raise CustomInputError(f"measurement duration must be a whole number of seconds in "
                                   f"[{MEASURE_MIN_S}, {MEASURE_MAX_S}], got {self.measure_s!r}")
        if self.yellow_policy not in YELLOW_POLICY_SECONDS:
            raise CustomInputError(f"yellow policy must be one of {sorted(YELLOW_POLICY_SECONDS)}, "
                                   f"got {self.yellow_policy!r}")
        if not isinstance(self.immediate_first_vehicle, bool):
            raise CustomInputError("immediate_first_vehicle must be a bool")
        self.smart_params()                     # validates the set and overrides

    def smart_params(self):
        return smart_params_for(self.param_set, dict(self.smart_overrides))

    @property
    def yellow_extension_s(self) -> float:
        return YELLOW_POLICY_SECONDS[self.yellow_policy]

    def config(self):
        from smarttraffic.runner import RunConfig, VehicleOptions
        return RunConfig(CUSTOM_SCENARIO, self.seed, self.controller, measure_s=float(self.measure_s),
                         smart_params=self.smart_params(), mode="custom", demand=self.demand,
                         vehicle=VehicleOptions(self.yellow_extension_s, self.immediate_first_vehicle))

    def saturation(self) -> SaturationReport:
        return saturation_report(self.demand)

    def to_dict(self) -> Dict[str, object]:
        d = {"tag": CUSTOM_TAG, "mode": "custom", "demand_veh_h": self.demand.as_dict(), "seed": self.seed,
             "controller": self.controller, "measure_s": self.measure_s,
             "yellow_policy": self.yellow_policy, "yellow_extension_s": self.yellow_extension_s,
             "immediate_first_vehicle": self.immediate_first_vehicle}
        if self.controller == "Smart":
            d["smart_param_set"] = self.param_set
            d["smart_overrides"] = dict(self.smart_overrides)
            d["smart_params"] = asdict(self.smart_params())
        return d

    def fingerprint(self) -> str:
        body = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(body.encode("utf-8")).hexdigest()


# ================================================================ output (never official)
def check_output_dir(out_dir: Union[str, Path]) -> Path:
    """The folder a Custom/Demo result may be written to; official folders are refused."""
    p = Path(out_dir).resolve()
    for prot in _PROTECTED:
        prot = prot.resolve()
        if p == prot or prot in p.parents:
            raise CustomInputError(f"Custom/Demo results must not be written to {prot} (official data); "
                                   f"use {CUSTOM_RESULTS_DIR}")
    return p


def save_custom(spec: CustomSpec, result, out_dir: Union[str, Path] = CUSTOM_RESULTS_DIR) -> Path:
    from smarttraffic.experiments import check_signal_log
    d = check_output_dir(out_dir)
    if result.metadata.get("mode") != "custom" or result.metadata.get("tag") != CUSTOM_TAG:
        raise CustomInputError("only a Custom/Demo result (metadata tag 'CUSTOM / DEMO') can be saved here")
    res_json = result.to_json()
    sat = spec.saturation()
    doc = {"tag": CUSTOM_TAG, "mode": "custom", "official": False,
           "note": "Custom/Demo run: not part of the study's official results.",
           "spec": spec.to_dict(), "saturation": asdict(sat),
           "summary": asdict(result.intersection),
           "signal_check": check_signal_log(result.signal_log, spec.smart_params()),
           "result_sha256": hashlib.sha256(res_json.encode("utf-8")).hexdigest(),
           "result": json.loads(res_json)}
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"custom_{spec.fingerprint()[:12]}.json"
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, sort_keys=True, indent=1) + "\n", encoding="utf-8")
    tmp.replace(path)
    return path


def run_custom(spec: CustomSpec, *, save: bool = True, out_dir: Union[str, Path] = CUSTOM_RESULTS_DIR,
               observers=()):
    """Run a Custom/Demo spec -> (RunResult, saved path or None)."""
    from smarttraffic.runner import run
    if save:
        check_output_dir(out_dir)               # refuse before spending time on the run
    res = run(spec.config(), observers=observers)
    return res, (save_custom(spec, res, out_dir) if save else None)


# ================================================================ CLI
def _parse_demand(text: str) -> CustomDemand:
    parts = {}
    for item in text.split(","):
        if "=" not in item:
            raise CustomInputError(f"bad demand item {item!r} (use NS=400,...)")
        k, v = item.split("=", 1)
        parts[k.strip().upper()] = v
    return CustomDemand.from_mapping(parts)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m smarttraffic.custom", description="Custom / Demo run (not official)")
    ap.add_argument("--demand", required=True, help="NS=..,SS=..,ES=..,WS=..,NL=..,SL=..,EL=..,WL=.. (veh/h)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--controller", default="Smart", choices=CUSTOM_CONTROLLERS)
    ap.add_argument("--measure", type=int, default=3600, help=f"measurement duration s [{MEASURE_MIN_S}, {MEASURE_MAX_S}]")
    ap.add_argument("--params", default="T1", choices=UI_PARAM_SETS)
    ap.add_argument("--tau", type=float)
    ap.add_argument("--wmax", type=float)
    ap.add_argument("--max-straight", type=float)
    ap.add_argument("--max-left", type=float)
    ap.add_argument("--yellow", default="study", choices=sorted(YELLOW_POLICY_SECONDS))
    ap.add_argument("--immediate-first", action="store_true", help="first queued car starts at green onset")
    ap.add_argument("--out", default=str(CUSTOM_RESULTS_DIR))
    a = ap.parse_args(argv)
    try:
        ov = {k: v for k, v in (("tau_s", a.tau), ("w_max_s", a.wmax), ("max_green_straight_s", a.max_straight),
                                ("max_green_left_s", a.max_left)) if v is not None}
        spec = CustomSpec(_parse_demand(a.demand), a.seed, a.controller, a.measure, a.params,
                          tuple(sorted(ov.items())), a.yellow, a.immediate_first)
    except CustomInputError as e:
        print(f"invalid input: {e}")
        return 2
    print(f"*** {CUSTOM_TAG} *** (not an official result)")
    sat = spec.saturation()
    if sat.warning:
        print(f"WARNING: {sat.warning}")
    try:
        res, path = run_custom(spec, out_dir=a.out)
    except CustomInputError as e:
        print(f"refused: {e}")
        return 2
    ir = res.intersection
    print(f"mean control delay {ir.delay_mean_s} s, P95 {ir.delay_p95_s} s, throughput {ir.throughput_veh_h} veh/h, "
          f"unfinished {ir.unfinished}, safety violations {ir.safety_violations}")
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    # run the package module's main(), not this __main__ copy: the runner checks the classes of
    # smarttraffic.custom (a second copy of CustomDemand would not be accepted)
    from smarttraffic.custom import main as _package_main
    sys.exit(_package_main())
