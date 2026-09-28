"""runner -- one simulation run, identical for every controller (Stage 12).

Spec: study §10 (warm-up 300 s, measurement 3600 s, flush <= 900 s, paired runs), §13
(step order); docs/ARCHITECTURE.md ("Statistics (Stage 12)", "Shared step order").

A run is fully described by RunConfig (scenario, seed, controller, dt, windows, Smart
parameters). Everything except the controller is built the same way for every run:

    arrivals   traffic_gen.get_or_create_arrivals(scenario, seed, -warm-up, measure end)
               (the same file for every controller, study R2), or an explicit schedule
    vehicles   VehicleModel(arrivals, start = -warm-up)
    detectors  VirtualDetectors(vehicles) (or source_factory(vehicles), e.g. hybrid)
    sensors    SensorBank(detectors)
    signals    SignalBank(start = -warm-up)
    controller FT-A / FT-B / FT-TOD (FixedTimeController, first cycle at -warm-up) or
               Smart (SmartController(sensors, SmartParams))

Step order (approved): vehicles -> sensors -> SignalBank.advance -> controller.update ->
statistics observe -> read-only observers. Steps are dt_ms long, shortened only so that
every 1 s queue-sample instant, the measurement end and the fixed controller's next
command time are step boundaries (at dt = 100 ms nothing is ever shortened).

The run ends at the first step >= measurement end at which every measured vehicle has
crossed the stop line, and never later than measurement end + flush.

Modes (Stage 14b). mode="official" (default) is the study: a scenario from
config/scenarios.json, the study's vehicle behaviour (VehicleOptions() = permissive first
2.0 s of yellow, study start-up lost time) and no custom demand; anything else is refused.
Official runs are unchanged, including their metadata. mode="custom" (Custom/Demo) takes
the 8 demands from `demand` (custom.CustomDemand, scenario "CUSTOM"), FT-A or Smart only,
and may change VehicleOptions; its metadata is tagged "CUSTOM / DEMO" and records the
demand and the vehicle options. Custom arrivals are generated in memory and never written
to the shared arrival directory.
"""
from dataclasses import asdict, dataclass, field, replace
import hashlib
from pathlib import Path
from typing import Callable, Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple, Union

from smarttraffic.controller_fixed import load_fixed_plans
from smarttraffic.controller_smart import SmartController, SmartParams
from smarttraffic.custom import (CUSTOM_CONTROLLERS, CUSTOM_SCENARIO, CUSTOM_TAG, CustomDemand,
                                 yellow_policy_name)
from smarttraffic.sensors import SensorBank, VirtualDetectors
from smarttraffic.signals import SignalBank
from smarttraffic.stats import SAMPLE_MS, RunResult, StatsCollector, Window
from smarttraffic.traffic_gen import ArrivalSchedule, generate_arrivals, get_or_create_arrivals
from smarttraffic.vehicles import YELLOW_EXTENSION, VehicleModel

__all__ = ["CONTROLLERS", "MODES", "RunConfig", "VehicleOptions", "StepView", "Simulation",
           "arrivals_sha256", "build_controller", "simulate", "run", "run_pair", "STEP_ORDER"]

CONTROLLERS = ("FT-A", "FT-B", "FT-TOD", "Smart")
MODES = ("official", "custom")
STEP_ORDER = "vehicles>sensors>signals>controller>stats>observers"


def _whole_ms(seconds: float, what: str) -> int:
    ms = round(seconds * 1000)
    if isinstance(seconds, bool) or abs(seconds * 1000 - ms) > 1e-6:
        raise ValueError(f"{what} must be a whole number of milliseconds")
    return ms


@dataclass(frozen=True)
class VehicleOptions:
    """Vehicle-model behaviour. The defaults are the study's and the only values official
    runs accept: yellow_extension_s 2.0 (permissive first 2.0 s of yellow, inclusive) and the
    study start-up (first car at t_g + l1 + h)."""
    yellow_extension_s: float = YELLOW_EXTENSION
    immediate_first_vehicle: bool = False

    def __post_init__(self):
        if isinstance(self.yellow_extension_s, bool) or not isinstance(self.yellow_extension_s, (int, float)):
            raise ValueError("yellow_extension_s must be a number of seconds")
        yellow_policy_name(self.yellow_extension_s)          # only the named policies exist
        if not isinstance(self.immediate_first_vehicle, bool):
            raise ValueError("immediate_first_vehicle must be a bool")

    @property
    def is_default(self) -> bool:
        return self == VehicleOptions()


@dataclass(frozen=True)
class RunConfig:
    scenario: str
    seed: int
    controller: str
    dt_ms: int = 100
    warmup_s: float = 300.0
    measure_s: float = 3600.0
    flush_s: float = 900.0
    smart_params: SmartParams = field(default_factory=SmartParams)
    mode: str = "official"
    demand: Optional[CustomDemand] = None
    vehicle: VehicleOptions = field(default_factory=VehicleOptions)

    def __post_init__(self):
        if self.controller not in CONTROLLERS:
            raise ValueError(f"controller must be one of {CONTROLLERS}, got {self.controller!r}")
        if isinstance(self.dt_ms, bool) or not isinstance(self.dt_ms, int) or self.dt_ms <= 0:
            raise ValueError("dt_ms must be a positive int")
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {self.mode!r}")
        if not isinstance(self.vehicle, VehicleOptions):
            raise TypeError("vehicle must be a VehicleOptions")
        if self.mode == "official":
            if self.demand is not None:
                raise ValueError("official mode refuses custom demand; use mode='custom' (Custom/Demo)")
            if not self.vehicle.is_default:
                raise ValueError("official mode requires the study vehicle behaviour "
                                 "(yellow_extension_s = 2.0, study start-up); Stop on Yellow and "
                                 "immediate first vehicle are Custom/Demo options only")
        else:
            if not isinstance(self.demand, CustomDemand):
                raise ValueError("custom mode needs demand = custom.CustomDemand(...)")
            if self.scenario != CUSTOM_SCENARIO:
                raise ValueError(f"custom mode runs scenario {CUSTOM_SCENARIO!r}, got {self.scenario!r}")
            if self.controller not in CUSTOM_CONTROLLERS:
                raise ValueError(f"custom mode supports {CUSTOM_CONTROLLERS} only, got {self.controller!r}")
        self.window()                                   # validates the windows

    def window(self) -> Window:
        return Window(_whole_ms(self.warmup_s, "warmup_s"), _whole_ms(self.measure_s, "measure_s"),
                      _whole_ms(self.flush_s, "flush_s"))

    def with_controller(self, name: str) -> "RunConfig":
        return replace(self, controller=name)


class StepView(NamedTuple):
    """What an observer sees after each step. Observers must only read these objects."""
    now_ms: int
    vehicles: object
    sensors: object
    signals: object
    controller: object


@dataclass
class Simulation:
    config: RunConfig
    vehicles: VehicleModel
    sensors: SensorBank
    signals: SignalBank
    controller: object
    collector: Optional[StatsCollector]
    arrivals_sha256: str
    arrivals_count: int
    end_ms: int = 0
    ended_early: bool = False
    steps: int = 0


def arrivals_sha256(arrivals: Union[ArrivalSchedule, Iterable]) -> str:
    """Checksum of the arrival data itself (id, time, movement, seq, source per row)."""
    items = arrivals.arrivals if isinstance(arrivals, ArrivalSchedule) else arrivals
    body = "".join(f"{a.id},{a.time_ms},{a.movement},{a.seq},{a.source}\n" for a in items)
    return hashlib.sha256(body.encode("utf-8")).hexdigest()


def build_controller(config: RunConfig, sensors, start_ms: int):
    """The only part of a run that depends on the controller choice."""
    if config.controller == "Smart":
        return SmartController(sensors, config.smart_params)
    return load_fixed_plans().controller(config.controller, config.scenario, start_time=start_ms / 1000)


def _controller_params(config: RunConfig, ctrl) -> Dict[str, object]:
    if config.controller == "Smart":
        return asdict(config.smart_params)
    return {"schedule": ctrl.schedule.name,
            "plans": ";".join(f"{'start' if t is None else t}:{p.name}:"
                              + "/".join(str(g) for g in p.greens_ms)
                              for t, p in ctrl.schedule.entries)}


def _load_arrivals(config: RunConfig, arrivals, arrivals_dir):
    w = config.window()
    if arrivals is None and config.mode == "custom":
        # in memory only: custom arrivals never enter the shared arrival directory
        arrivals = generate_arrivals(config.demand.scenario(), config.seed, w.start_ms / 1000,
                                     w.measure_ms / 1000)
    if arrivals is None:
        kw = {} if arrivals_dir is None else {"directory": arrivals_dir}
        arrivals, _ = get_or_create_arrivals(config.scenario, config.seed, w.start_ms / 1000,
                                             w.measure_ms / 1000, **kw)
    items = tuple(arrivals.arrivals if isinstance(arrivals, ArrivalSchedule) else arrivals)
    for a in items:
        if not (w.start_ms <= a.time_ms < w.measure_ms):
            raise ValueError(f"arrival {a.id} at {a.time_ms} ms is outside the run's arrival "
                             f"window [{w.start_ms}, {w.measure_ms}) (no arrivals during flush)")
    return items


def simulate(config: RunConfig, *, arrivals=None, arrivals_dir: Optional[Union[str, Path]] = None,
             source_factory: Optional[Callable] = None, observers: Sequence[Callable[[StepView], None]] = (),
             collect_stats: bool = True) -> Simulation:
    """Run the simulation; statistics are collected only if collect_stats (the run itself is
    identical either way)."""
    w = config.window()
    items = _load_arrivals(config, arrivals, arrivals_dir)
    start = w.start_ms
    vm = VehicleModel(list(items), start=start / 1000,
                      yellow_extension_s=config.vehicle.yellow_extension_s,
                      immediate_first_vehicle=config.vehicle.immediate_first_vehicle)
    src = source_factory(vm) if source_factory is not None else VirtualDetectors(vm)
    sb = SensorBank(src)
    bank = SignalBank(start_time=start / 1000)
    ctrl = build_controller(config, sb, start)
    collector = StatsCollector(vm, sb, bank, ctrl, w) if collect_stats else None
    sim = Simulation(config, vm, sb, bank, ctrl, collector, arrivals_sha256(items), len(items))
    pending = [a.id for a in items if 0 <= a.time_ms < w.measure_ms]
    next_cmd = getattr(ctrl, "next_command_ms", None)

    def after_step():
        if collector is not None:
            collector.observe()
        if observers:
            view = StepView(vm.now_ms, vm, sb, bank, ctrl)
            for obs in observers:
                obs(view)

    ctrl.update(bank)
    after_step()
    now = start
    while True:
        if now >= w.measure_ms:
            pending = [i for i in pending if vm.vehicle(i).cross_ms is None]
            if not pending:
                sim.ended_early = now < w.hard_end_ms
                break
            if now >= w.hard_end_ms:
                break
        target = now + config.dt_ms
        nb = (now // SAMPLE_MS + 1) * SAMPLE_MS              # next 1 s instant
        if nb <= w.measure_ms:
            target = min(target, nb)
        if callable(next_cmd):
            nc = next_cmd()
            if nc > now:
                target = min(target, nc)
        target = min(target, w.hard_end_ms)
        dt = (target - now) / 1000
        vm.advance(dt, bank)
        sb.advance(dt, bank)
        bank.advance(dt)
        ctrl.update(bank)
        now = target
        sim.steps += 1
        after_step()
    sim.end_ms = now
    return sim


def run(config: RunConfig, **kw) -> RunResult:
    """simulate() + statistics -> RunResult (with full reproducibility metadata)."""
    sim = simulate(config, collect_stats=True, **kw)
    w = config.window()
    meta = {
        "scenario": config.scenario, "seed": config.seed, "controller": config.controller,
        "controller_params": _controller_params(config, sim.controller),
        "dt_ms": config.dt_ms, "warmup_ms": w.warmup_ms, "measure_ms": w.measure_ms,
        "flush_ms": w.flush_ms, "arrivals_sha256": sim.arrivals_sha256,
        "arrivals_count": sim.arrivals_count, "step_order": STEP_ORDER,
        "detector_source": "custom" if kw.get("source_factory") else "virtual",
        "steps": sim.steps,
    }
    if config.mode == "custom":            # official metadata is left exactly as before
        meta.update({
            "mode": "custom", "tag": CUSTOM_TAG,
            "demand_veh_h": config.demand.as_dict(),
            "yellow_policy": yellow_policy_name(config.vehicle.yellow_extension_s),
            "yellow_extension_s": config.vehicle.yellow_extension_s,
            "immediate_first_vehicle": config.vehicle.immediate_first_vehicle,
        })
    return sim.collector.finalize(sim.end_ms, meta, sim.ended_early)


def run_pair(config: RunConfig, controllers: Sequence[str] = ("FT-A", "Smart"), **kw) -> Dict[str, RunResult]:
    """Run the same config with each controller: same arrivals (checked by checksum),
    seed, scenario, windows and dt. Only the controller differs."""
    if "arrivals" not in kw or kw["arrivals"] is None:
        kw["arrivals"] = _load_arrivals(config, None, kw.pop("arrivals_dir", None))
    out = {name: run(config.with_controller(name), **kw) for name in controllers}
    shas = {r.metadata["arrivals_sha256"] for r in out.values()}
    if len(shas) != 1:
        raise RuntimeError("paired runs did not use identical arrivals")
    return out
