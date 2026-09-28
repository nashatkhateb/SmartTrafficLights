"""stats -- observational statistics: per-vehicle records, 1 s samples, run summary (Stage 12).

Spec: docs/Eight-Movement_Intersection_Study.html §10 (run length, paired comparisons),
§11 (metrics), §13; docs/ARCHITECTURE.md ("Statistics (Stage 12)").
Responsibility: OBSERVE a simulation and compute the study metrics identically for every
controller. Must not: change vehicles, sensors, signals, controllers or traffic generation
(it only reads their public state and logs).

Windows (integer ms, t = 0 is the start of measurement)
--------------------------------------------------------
    warm-up      [-warmup_ms, 0)
    measurement  [0, measure_ms)          measured vehicles V: scheduled arrival in here
    flush        [measure_ms, measure_ms + flush_ms]   no new arrivals; ends early once
                                                       every vehicle in V has crossed

Official metrics (study §11), per movement and intersection-wide
----------------------------------------------------------------
* control delay d_v = cross_ms - (scheduled_ms + 10 800) over C = crossed vehicles of V;
  mean, P95 (numpy-compatible linear interpolation), max, total (veh*h).
* unfinished = V not crossed at the end of the run (hard limit): excluded from the delay
  statistics above, always counted, and included in delay_mean_lower_bound_s with
  d = end_ms - free-flow crossing time (a censored lower bound).
* vehicle waiting time = stopped time on the road + time held outside the road entry (R3),
  over C. (Not the controller's detector-call wait.)
* vehicles_control_delay_over_90s = |{v in C : d_v > 90 000 ms}| (study "vehicles waiting
  > 90 s", G7 threshold; not stopped time, not detector-call wait, not W_MAX).
* queue Q_m(k) = stopped vehicles of m on the road + vehicles of m held outside the entry,
  sampled at exactly t_k = k * 1000 ms, k = 0 .. measure_ms/1000 - 1 (study: 1 s samples).
  Per movement: mean and max; intersection: mean of the 8 movement means (headline, G3),
  max over movements and samples, mean and max of the total.
* throughput = crossings (of ANY vehicle, warm-up arrivals included) with cross_ms in
  [0, measure_ms), expressed in veh/h. served_in_hour / pct_served = V crossing before
  measure_ms.
* green utilization_m = n_m * h_m / G_m: n_m = crossings of m in [0, measure_ms) (under the
  legacy-study vehicle policy including crossings during the first 2 s of YELLOW), h_m =
  1.9 / 2.0 s, G_m = displayed GREEN + FLASHING_GREEN seconds of m inside [0, measure_ms).
  Under the standard policy a saturated green serves about G/h + 1 cars (the first car leaves
  at green onset), so the value can slightly exceed 1.
* signal changes = GREEN onsets (RED_YELLOW -> GREEN) of the 8 signals in [0, measure_ms), per hour.
* Jain's fairness index over the movement mean delays of movements with >= 1 crossed
  measured vehicle; 1.0 if all those means are 0; None if no movement qualifies.
* safety violations = steps at which conflicting movements were active (must be 0).

Diagnostics (not study metrics; any may be None) are kept in RunResult.diagnostics.
"""
from dataclasses import asdict, dataclass, field
import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

from smarttraffic import conflicts
from smarttraffic.conflicts import MOVES
from smarttraffic.signals import FLASHING_GREEN, GREEN, SignalState
from smarttraffic.vehicles import FREE_TRAVEL_TIME, SATURATION_HEADWAY

__all__ = [
    "SAMPLE_MS", "DELAY_OVER_90S_MS", "LONG_CALL_WAIT_MS", "Window", "VehicleRecord",
    "MovementStats", "IntersectionStats", "RunResult", "StatsCollector",
    "percentile_linear", "mean_ci95", "paired_comparison",
]

MS = 1000
SAMPLE_MS = 1000                      # study §11: queue sampled every 1 s
DELAY_OVER_90S_MS = 90_000            # study §9/§11, G7 threshold
LONG_CALL_WAIT_MS = 120_000           # diagnostic: Stage 11 single-hold bound
_FREE_MS = round(FREE_TRAVEL_TIME * MS)
_SERVICE = (GREEN, FLASHING_GREEN)
_H_MS = {m: round(SATURATION_HEADWAY["straight" if m[1] == "S" else "left"] * MS) for m in MOVES}
_STRAIGHT_PAIRS = (("NS", "SS"), ("ES", "WS"))


# =============================================================== helpers
def percentile_linear(values: Sequence[float], p: float) -> Optional[float]:
    """numpy.percentile(values, p) with the default 'linear' method; None if empty."""
    if not values:
        return None
    if not 0 <= p <= 100:
        raise ValueError("p must be in [0, 100]")
    xs = sorted(values)
    pos = (len(xs) - 1) * p / 100
    lo = math.floor(pos)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def _mean(xs) -> Optional[float]:
    xs = list(xs)
    return sum(xs) / len(xs) if xs else None


def _s(ms: Optional[float]) -> Optional[float]:
    return None if ms is None else ms / MS


@dataclass(frozen=True)
class Window:
    warmup_ms: int = 300_000
    measure_ms: int = 3_600_000
    flush_ms: int = 900_000

    def __post_init__(self):
        for name in ("warmup_ms", "measure_ms", "flush_ms"):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, int) or v < 0:
                raise ValueError(f"{name} must be a non-negative int (ms)")
        if self.measure_ms <= 0 or self.measure_ms % SAMPLE_MS:
            raise ValueError("measure_ms must be a positive multiple of 1000 ms")

    @property
    def start_ms(self) -> int:
        return -self.warmup_ms

    @property
    def hard_end_ms(self) -> int:
        return self.measure_ms + self.flush_ms

    def sample_times(self) -> range:
        return range(0, self.measure_ms, SAMPLE_MS)


# =============================================================== records
@dataclass(frozen=True)
class VehicleRecord:
    id: int
    movement: str
    source: str
    measured: bool                    # scheduled arrival in [0, measure_ms)
    scheduled_ms: int
    entry_ms: Optional[int]
    cross_ms: Optional[int]
    exit_ms: Optional[int]
    stopped_on_road_ms: int
    waited_outside_ms: int
    control_delay_ms: Optional[int]   # None if not crossed

    @property
    def waiting_ms(self) -> int:
        return self.stopped_on_road_ms + self.waited_outside_ms


@dataclass(frozen=True)
class MovementStats:
    movement: str
    arrivals: int
    crossed: int
    unfinished: int
    served_in_hour: int
    pct_served: Optional[float]
    throughput_veh_h: float
    delay_mean_s: Optional[float]
    delay_p95_s: Optional[float]
    delay_max_s: Optional[float]
    delay_total_veh_h: float
    delay_mean_lower_bound_s: Optional[float]
    waiting_mean_s: Optional[float]
    vehicles_control_delay_over_90s: int
    queue_mean_veh: float
    queue_max_veh: int
    queue_road_mean_veh: float
    queue_outside_mean_veh: float
    green_s: float
    green_onsets: int
    green_utilization: Optional[float]


@dataclass(frozen=True)
class IntersectionStats:
    arrivals: int
    crossed: int
    unfinished: int
    served_in_hour: int
    pct_served: Optional[float]
    throughput_veh_h: float
    delay_mean_s: Optional[float]
    delay_p95_s: Optional[float]
    delay_max_s: Optional[float]
    delay_total_veh_h: float
    delay_mean_lower_bound_s: Optional[float]
    waiting_mean_s: Optional[float]
    vehicles_control_delay_over_90s: int
    queue_mean_per_movement_veh: float    # headline (G3): mean of the 8 movement means
    queue_max_movement_veh: int           # max over movements and samples
    queue_total_mean_veh: float
    queue_total_max_veh: int              # max over samples of the intersection total
    green_onsets: int
    signal_changes_per_hour: float
    jain_fairness: Optional[float]
    safety_violations: int


@dataclass(frozen=True)
class RunResult:
    metadata: Dict[str, object]
    end_ms: int
    ended_early: bool                     # all measured vehicles crossed before the hard end
    intersection: IntersectionStats
    movements: Tuple[MovementStats, ...]  # MOVES order
    diagnostics: Dict[str, object]
    queue_samples: Tuple[Tuple[int, Tuple[int, ...], Tuple[int, ...]], ...]  # (t_ms, road[8], outside[8])
    signal_log: Tuple[Tuple[int, str, str], ...]                           # (t_ms, movement, new state)
    vehicles: Tuple[VehicleRecord, ...]

    def movement(self, m: str) -> MovementStats:
        return self.movements[MOVES.index(m)]

    # ------------------------------------------------------------ exports
    def to_dict(self) -> Dict[str, object]:
        d = asdict(self)
        d["queue_samples"] = [[t, list(r), list(o)] for t, r, o in self.queue_samples]
        d["signal_log"] = [list(x) for x in self.signal_log]
        return d

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False)

    def summary_row(self) -> Dict[str, object]:
        flat = {f"cfg_{k}": v for k, v in sorted(self.metadata.items()) if not isinstance(v, dict)}
        nested = {f"cfg_{k}_{kk}": vv for k, v in sorted(self.metadata.items()) if isinstance(v, dict)
                  for kk, vv in sorted(v.items())}
        return {**flat, **nested, "end_ms": self.end_ms, "ended_early": self.ended_early,
                **asdict(self.intersection)}

    def _key(self) -> Dict[str, object]:
        return {"scenario": self.metadata.get("scenario"), "seed": self.metadata.get("seed"),
                "controller": self.metadata.get("controller")}

    def movement_rows(self) -> List[Dict[str, object]]:
        return [{**self._key(), **asdict(ms)} for ms in self.movements]

    def vehicle_rows(self) -> List[Dict[str, object]]:
        return [{**self._key(), **asdict(v), "waiting_ms": v.waiting_ms} for v in self.vehicles]

    def queue_rows(self) -> List[Dict[str, object]]:
        return [{**self._key(), "t_ms": t, "movement": m, "road": road[i], "outside": out[i],
                 "queue": road[i] + out[i]}
                for t, road, out in self.queue_samples for i, m in enumerate(MOVES)]


# ============================================================= collector
class StatsCollector:
    """Observes one run. Call observe() after every simulation step (after the controller)."""

    def __init__(self, vehicles, sensors, signals, controller, window: Window = Window()):
        self.vm, self.sensors, self.signals, self.controller = vehicles, sensors, signals, controller
        self.window = window
        self._samples: List[Tuple[int, Tuple[int, ...], Tuple[int, ...]]] = []
        self._next_sample = 0
        self._last_now: Optional[int] = None
        self.safety_violations = 0
        # diagnostics (observational, optional)
        self._call_wait_max = {m: 0 for m in MOVES}
        self._long_calls = set()
        self._sat_s = {m: 0 for m in MOVES}
        self._fault_s = {m: 0 for m in MOVES}
        self._fault_green_s = {m: 0 for m in MOVES}
        self._diag_ok = {"call_wait": True, "qhat_saturation": True, "fault": True}

    def next_sample_ms(self) -> Optional[int]:
        """Next official queue-sample instant (None when all samples are taken)."""
        return self._next_sample if self._next_sample < self.window.measure_ms else None

    def observe(self) -> None:
        now = self.vm.now_ms
        if self._last_now is not None and now < self._last_now:
            raise ValueError("time went backwards")
        self._last_now = now
        if conflicts.conflicting_pairs(self.signals.active()):
            self.safety_violations += 1
        nxt = self.next_sample_ms()
        if nxt is not None and now > nxt:
            raise ValueError(f"queue sample at {nxt} ms was skipped (now {now} ms): "
                             f"the runner must make every 1 s instant a step boundary")
        if 0 <= now < self.window.measure_ms:
            self._observe_calls(now)
        if nxt is not None and now == nxt:
            self._sample(now)
            self._next_sample += SAMPLE_MS

    # ----------------------------------------------------------- sampling
    def _sample(self, t: int) -> None:
        road = tuple(self.vm.queue_length(m) for m in MOVES)
        out = tuple(len(self.vm.waiting_outside(m)) for m in MOVES)
        self._samples.append((t, road, out))
        if self._diag_ok["qhat_saturation"]:
            f = getattr(self.sensors, "level2_latched", None)
            if callable(f):
                for i, m in enumerate(MOVES):
                    if f(m) and road[i] + out[i] > 6:
                        self._sat_s[m] += 1
            else:
                self._diag_ok["qhat_saturation"] = False
        if self._diag_ok["fault"]:
            f = getattr(self.sensors, "faulted_detectors", None)
            if callable(f):
                faulted = {m for m, _ in f()}
                for m in faulted:
                    self._fault_s[m] += 1
                    if self.signals.state(m) in _SERVICE:
                        self._fault_green_s[m] += 1
            else:
                self._diag_ok["fault"] = False

    def _observe_calls(self, now: int) -> None:
        if not self._diag_ok["call_wait"]:
            return
        has, since = getattr(self.sensors, "has_call", None), getattr(self.sensors, "call_since_ms", None)
        if not (callable(has) and callable(since)):
            self._diag_ok["call_wait"] = False
            return
        for m in MOVES:
            if has(m):
                t0 = since(m)
                w = now - t0
                if w > self._call_wait_max[m]:
                    self._call_wait_max[m] = w
                if w > LONG_CALL_WAIT_MS:
                    self._long_calls.add((m, t0))

    # ------------------------------------------------------------ results
    def finalize(self, end_ms: int, metadata: Dict[str, object], ended_early: bool) -> RunResult:
        w = self.window
        if len(self._samples) != w.measure_ms // SAMPLE_MS:
            raise ValueError(f"expected {w.measure_ms // SAMPLE_MS} queue samples, got {len(self._samples)}")
        records = tuple(self._record(v, end_ms) for v in self.vm.all_vehicles())
        log = tuple((round(e.time * MS), e.movement, e.new.name) for e in self.signals.log)
        service = self._service_intervals(end_ms)
        movements = tuple(self._movement_stats(m, records, service, end_ms) for m in MOVES)
        inter = self._intersection_stats(records, movements, end_ms)
        return RunResult(metadata=dict(metadata), end_ms=end_ms, ended_early=ended_early,
                         intersection=inter, movements=movements,
                         diagnostics=self._diagnostics(service),
                         queue_samples=tuple(self._samples), signal_log=log, vehicles=records)

    def _record(self, v, end_ms: int) -> VehicleRecord:
        measured = 0 <= v.scheduled_ms < self.window.measure_ms
        if v.entry_ms is not None:
            outside = v.entry_ms - v.scheduled_ms
        else:
            outside = max(0, end_ms - v.scheduled_ms)          # still held outside at the end
        delay = None if v.cross_ms is None else v.cross_ms - (v.scheduled_ms + _FREE_MS)
        return VehicleRecord(v.id, v.movement, v.source, measured, v.scheduled_ms, v.entry_ms,
                             v.cross_ms, v.exit_ms, round(v.stopped_time * MS), outside, delay)

    def _service_intervals(self, end_ms: int) -> Dict[str, List[Tuple[int, int]]]:
        """[start, end) intervals of GREEN or FLASHING_GREEN per movement, from the signal log."""
        out: Dict[str, List[Tuple[int, int]]] = {m: [] for m in MOVES}
        start: Dict[str, Optional[int]] = {m: None for m in MOVES}
        for e in self.signals.log:
            t, m = round(e.time * MS), e.movement
            if e.new in _SERVICE and start[m] is None:
                start[m] = t
            elif e.new not in _SERVICE and start[m] is not None:
                out[m].append((start[m], t))
                start[m] = None
        for m in MOVES:
            if start[m] is not None:
                out[m].append((start[m], end_ms))
        return out

    def _clip(self, iv: List[Tuple[int, int]]) -> int:
        a0, b0 = 0, self.window.measure_ms
        return sum(max(0, min(b, b0) - max(a, a0)) for a, b in iv)

    def _delay_block(self, recs: Sequence[VehicleRecord], end_ms: int) -> Dict[str, object]:
        meas = [r for r in recs if r.measured]
        done = [r for r in meas if r.cross_ms is not None]
        d = [r.control_delay_ms for r in done]
        lower = d + [end_ms - (r.scheduled_ms + _FREE_MS) for r in meas if r.cross_ms is None]
        served = sum(1 for r in done if r.cross_ms < self.window.measure_ms)
        return dict(
            arrivals=len(meas), crossed=len(done), unfinished=len(meas) - len(done),
            served_in_hour=served, pct_served=(100.0 * served / len(meas)) if meas else None,
            throughput_veh_h=sum(1 for r in recs if r.cross_ms is not None
                                 and 0 <= r.cross_ms < self.window.measure_ms)
            * 3_600_000 / self.window.measure_ms,
            delay_mean_s=_s(_mean(d)), delay_p95_s=_s(percentile_linear(d, 95)),
            delay_max_s=_s(max(d) if d else None), delay_total_veh_h=sum(d) / 3_600_000,
            delay_mean_lower_bound_s=_s(_mean(lower)),
            waiting_mean_s=_s(_mean(r.waiting_ms for r in done)),
            vehicles_control_delay_over_90s=sum(1 for x in d if x > DELAY_OVER_90S_MS))

    def _movement_stats(self, m, records, service, end_ms) -> MovementStats:
        i = MOVES.index(m)
        recs = [r for r in records if r.movement == m]
        blk = self._delay_block(recs, end_ms)
        q = [road[i] + out[i] for _, road, out in self._samples]
        green_ms = self._clip(service[m])
        n_cross = sum(1 for r in recs if r.cross_ms is not None and 0 <= r.cross_ms < self.window.measure_ms)
        onsets = sum(1 for a, _ in service[m] if 0 <= a < self.window.measure_ms)
        return MovementStats(
            movement=m, **blk,
            queue_mean_veh=sum(q) / len(q), queue_max_veh=max(q),
            queue_road_mean_veh=sum(road[i] for _, road, _ in self._samples) / len(q),
            queue_outside_mean_veh=sum(out[i] for _, _, out in self._samples) / len(q),
            green_s=green_ms / MS, green_onsets=onsets,
            green_utilization=(n_cross * _H_MS[m] / green_ms) if green_ms > 0 else None)

    def _intersection_stats(self, records, movements, end_ms) -> IntersectionStats:
        blk = self._delay_block(records, end_ms)
        totals = [sum(road) + sum(out) for _, road, out in self._samples]
        means = [ms.delay_mean_s for ms in movements if ms.delay_mean_s is not None]
        if not means:
            jain = None
        elif all(x == 0 for x in means):
            jain = 1.0
        else:
            jain = sum(means) ** 2 / (len(means) * sum(x * x for x in means))
        onsets = sum(ms.green_onsets for ms in movements)
        return IntersectionStats(
            **blk,
            queue_mean_per_movement_veh=sum(ms.queue_mean_veh for ms in movements) / len(movements),
            queue_max_movement_veh=max(ms.queue_max_veh for ms in movements),
            queue_total_mean_veh=sum(totals) / len(totals), queue_total_max_veh=max(totals),
            green_onsets=onsets, signal_changes_per_hour=onsets * 3_600_000 / self.window.measure_ms,
            jain_fairness=jain, safety_violations=self.safety_violations)

    def _diagnostics(self, service) -> Dict[str, object]:
        diag: Dict[str, object] = {}
        ok = self._diag_ok
        diag["detector_call_wait_max_s"] = ({m: self._call_wait_max[m] / MS for m in MOVES}
                                            if ok["call_wait"] else None)
        diag["detector_calls_over_120s"] = ({m: sum(1 for x, _ in self._long_calls if x == m) for m in MOVES}
                                            if ok["call_wait"] else None)
        diag["qhat_saturation_s"] = dict(self._sat_s) if ok["qhat_saturation"] else None
        diag["faulted_s"] = dict(self._fault_s) if ok["fault"] else None
        diag["green_while_faulted_s"] = dict(self._fault_green_s) if ok["fault"] else None
        split = {}
        for a, b in _STRAIGHT_PAIRS:
            both, one = _overlap(service[a], service[b], 0, self.window.measure_ms)
            split[f"{a}+{b}"] = {"both_green_s": both / MS, "one_green_s": one / MS}
        diag["straight_pair_service"] = split
        log = getattr(self.controller, "decision_log", None)
        if isinstance(log, list) and all(hasattr(d, "stopped") for d in log):
            win = [d for d in log if 0 <= d.time_ms < self.window.measure_ms]
            reasons = {"gap": 0, "max": 0, "hold": 0}
            for d in win:
                for _, r in d.stopped:
                    reasons[r] = reasons.get(r, 0) + 1
            diag["controller_terminations"] = reasons
            diag["controller_hold_decisions"] = sum(1 for d in win if getattr(d, "hold", None) is not None)
        else:
            diag["controller_terminations"] = None
            diag["controller_hold_decisions"] = None
        return diag


def _overlap(ia: List[Tuple[int, int]], ib: List[Tuple[int, int]], lo: int, hi: int) -> Tuple[int, int]:
    """(ms both in service, ms exactly one in service) inside [lo, hi)."""
    pts = sorted({lo, hi} | {x for a, b in ia + ib for x in (a, b) if lo < x < hi})
    both = one = 0

    def inside(iv, t):
        return any(a <= t < b for a, b in iv)

    for a, b in zip(pts, pts[1:]):
        k = inside(ia, a) + inside(ib, a)
        if k == 2:
            both += b - a
        elif k == 1:
            one += b - a
    return both, one


# ========================================================= across seeds
def _tcrit(df: int) -> float:
    from scipy import stats as st
    return float(st.t.ppf(0.975, df))


def mean_ci95(values: Sequence[float]) -> Dict[str, object]:
    """Mean and two-sided 95% t confidence interval. n = 1 -> no interval (None)."""
    xs = [float(x) for x in values]
    n = len(xs)
    if n == 0:
        raise ValueError("mean_ci95 needs at least one value")
    m = sum(xs) / n
    if n == 1:
        return {"n": 1, "mean": m, "sd": None, "half_width": None, "low": None, "high": None,
                "note": "n = 1: no confidence interval"}
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))
    hw = _tcrit(n - 1) * sd / math.sqrt(n)
    return {"n": n, "mean": m, "sd": sd, "half_width": hw, "low": m - hw, "high": m + hw,
            "note": "zero variance" if sd == 0 else ""}


def paired_comparison(a: Sequence[float], b: Sequence[float]) -> Dict[str, object]:
    """Paired comparison of b against a (e.g. a = Fixed, b = Smart, same seeds in the same
    order): differences d = b - a, mean, 95% CI and a paired two-sided t-test when valid."""
    if len(a) != len(b):
        raise ValueError("paired samples must have the same length (same seeds)")
    d = [float(y) - float(x) for x, y in zip(a, b)]
    out = mean_ci95(d)
    out = {"n": out["n"], "mean_diff": out["mean"], "sd_diff": out["sd"],
           "half_width": out["half_width"], "low": out["low"], "high": out["high"]}
    n = len(d)
    if n < 2:
        out = {**out, "t": None, "p": None, "test_valid": False, "note": "n < 2: no paired t-test"}
    elif out["sd_diff"] == 0:
        out = {**out, "t": None, "p": None, "test_valid": False,
               "note": "zero variance of the differences: t-test undefined"}
    else:
        from scipy import stats as st
        t = out["mean_diff"] / (out["sd_diff"] / math.sqrt(n))
        out = {**out, "t": t, "p": float(2 * st.t.sf(abs(t), n - 1)), "test_valid": True, "note": ""}
    return out
