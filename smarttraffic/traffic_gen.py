"""traffic_gen -- deterministic vehicle-arrival schedules for the 8 movements.

Spec: docs/Eight-Movement_Intersection_Study.html, §5 (arrival model),
§10 (scenarios, run windows, seeds), §13 (module table).
Responsibility: create arrival lists per movement from scenario + seed; write CSV.
Must not: know anything about control. This module imports nothing from
signals, vehicles, sensors or the controllers, so Fixed and Smart are
guaranteed to receive the same arrivals from the same file.

Time axis
---------
Study §10: t = 0 is the start of measurement, so the warm-up is
-300 <= t < 0 and measurement is 0 <= t < 3600. Arrivals are generated on a
half-open interval [start, end); the default is [-300, 3600), and nothing is
generated in the flush (t >= 3600) unless a caller explicitly asks for it.
An arrival time is the moment the vehicle appears 150 m upstream (§5).

All times are whole milliseconds internally (the same clock as signals.py),
so CSV files round-trip exactly and boundary comparisons are exact.

Arrival model (study §5)
------------------------
Headway h = 1.5 s + Exp(mean = 3600/q - 1.5), q in veh/h.
* q = 0 -> no random arrivals.
* Valid only when 3600/q - 1.5 > 0, i.e. 0 < q < 2400; anything else is rejected.
* The stream starts as if a vehicle had arrived at `start`, so the first
  arrival is at start + h.
* Time-varying rates (S6 step change): after the 1.5 s minimum gap, the
  exponential part runs on the rate in force at each instant (exact
  time-change of a unit exponential). For a constant q this reduces exactly
  to the formula above.
* Each arrival time is rounded to the nearest millisecond; the 1.5 s minimum
  is enforced on the integer clock, so it can never be undercut by rounding.

Determinism
-----------
Every movement has its own numpy Generator (PCG64) seeded from
SeedSequence([seed, movement_key]), where movement_key is the ASCII bytes of
the movement name as an integer ("NS" -> 0x4E53). No global random state, no
built-in hash(), no wall clock. The order in which movements are generated
cannot affect any stream. Streams do not depend on the scenario or on `end`,
so a longer run with the same start extends a shorter one.

Deterministic events (S5 bursts, S10 WL cars) are added on top of the random
streams: `count` cars, the first at exactly `start`, then every `interval` s.
Only events inside [start, end) are included.
"""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

from smarttraffic.conflicts import MOVES, validate_movement

__all__ = [
    "Arrival", "ScheduleMeta", "ArrivalSchedule", "Scenario", "RateSegment", "EventSpec",
    "load_scenarios", "validate_rate", "exp_mean", "constant_rate",
    "generate_movement_times", "event_times", "generate_arrivals",
    "save_arrivals", "load_arrivals", "get_or_create_arrivals", "default_arrival_path",
    "ArrivalFileError", "ArrivalFileMismatch", "ScenarioConfigError",
    "MIN_HEADWAY_S", "WARMUP_S", "MEASUREMENT_S", "FLUSH_MAX_S",
    "DEFAULT_START_S", "DEFAULT_END_S", "FINAL_SEEDS", "TUNING_SEEDS",
    "GENERATOR_VERSION", "FORMAT_VERSION",
]

# ---------------------------------------------------------- study constants
MIN_HEADWAY_S = 1.5            # §5
WARMUP_S = 300                 # §10
MEASUREMENT_S = 3600           # §10
FLUSH_MAX_S = 900              # §10 (no arrivals are generated here by default)
DEFAULT_START_S = -WARMUP_S
DEFAULT_END_S = MEASUREMENT_S
FINAL_SEEDS = tuple(range(1, 11))      # §10
TUNING_SEEDS = tuple(range(101, 106))  # §10, S1 only

GENERATOR_VERSION = "1"
FORMAT_VERSION = "1"

MS = 1000
_MIN_HEADWAY_MS = 1500
_MAX_Q = 3600 / MIN_HEADWAY_S  # 2400 veh/h: exponential mean would be <= 0

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SCENARIO_FILE = PROJECT_ROOT / "config" / "scenarios.json"
ARRIVALS_DIR = PROJECT_ROOT / "data" / "arrivals"

SOURCE_RANDOM, SOURCE_EVENT = "random", "event"
_SOURCE_RANK = {SOURCE_RANDOM: 0, SOURCE_EVENT: 1}


# ----------------------------------------------------------------- errors
class ScenarioConfigError(ValueError):
    """The scenario configuration is malformed or violates the study model."""


class ArrivalFileError(ValueError):
    """An arrival file is malformed or fails its integrity checks."""


class ArrivalFileMismatch(ArrivalFileError):
    """An existing arrival file does not match the requested configuration."""


# ------------------------------------------------------------ validation
def _is_number(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def validate_rate(q) -> float:
    """Return q if it is a supported demand in veh/h, else raise ValueError.

    Supported: q == 0 (no arrivals) or 0 < q < 2400 (3600/q - 1.5 > 0).
    """
    if not _is_number(q):
        raise ValueError(f"demand q must be a number of veh/h, got {q!r}")
    if not math.isfinite(q):
        raise ValueError(f"demand q must be finite, got {q!r}")
    if q < 0:
        raise ValueError(f"demand q must not be negative, got {q!r}")
    if q > 0 and not (3600.0 / q - MIN_HEADWAY_S > 0):
        raise ValueError(
            f"demand q={q!r} veh/h is unsupported: the headway model needs "
            f"3600/q - {MIN_HEADWAY_S} > 0, i.e. q < {_MAX_Q:g}")
    return q


def exp_mean(q) -> float:
    """Mean of the exponential part of the headway, 3600/q - 1.5 (seconds)."""
    validate_rate(q)
    if q == 0:
        raise ValueError("q = 0 has no headway (no arrivals)")
    return 3600.0 / q - MIN_HEADWAY_S


def _to_ms(seconds, what: str) -> int:
    if not _is_number(seconds) or not math.isfinite(seconds):
        raise ValueError(f"{what} must be a finite number of seconds, got {seconds!r}")
    ms = round(seconds * MS)
    if abs(seconds * MS - ms) > 1e-6:
        raise ValueError(f"{what} must be a whole number of milliseconds, got {seconds!r}")
    return ms


def _validate_seed(seed) -> int:
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError(f"seed must be a non-negative int, got {seed!r}")
    return seed


def _validate_interval(start, end) -> Tuple[int, int]:
    s, e = _to_ms(start, "start"), _to_ms(end, "end")
    if e <= s:
        raise ValueError(f"end ({end}) must be greater than start ({start})")
    return s, e


# ---------------------------------------------------------- scenario model
@dataclass(frozen=True)
class RateSegment:
    """Rates (veh/h per movement) in force on [from_ms, until_ms); None = unbounded."""
    from_ms: Optional[int]
    until_ms: Optional[int]
    rates: Tuple[Tuple[str, float], ...]      # in MOVES order

    def rate(self, m: str) -> float:
        return dict(self.rates)[validate_movement(m)]


@dataclass(frozen=True)
class EventSpec:
    """`count` extra cars on `movement`: first at start_ms, then every interval_ms."""
    movement: str
    count: int
    start_ms: int
    interval_ms: int


@dataclass(frozen=True)
class Scenario:
    id: str
    description: str
    segments: Tuple[RateSegment, ...]
    events: Tuple[EventSpec, ...]
    config_sha256: str     # hash of the generation-relevant config (segments + events)

    def rate_at(self, m: str, t: float) -> float:
        validate_movement(m)
        t_ms = _to_ms(t, "t")
        for seg in self.segments:
            if (seg.from_ms is None or seg.from_ms <= t_ms) and \
               (seg.until_ms is None or t_ms < seg.until_ms):
                return seg.rate(m)
        raise AssertionError("segments do not cover the time axis")  # prevented by validation


def _parse_rates(raw, where: str) -> Tuple[Tuple[str, float], ...]:
    if not isinstance(raw, dict):
        raise ScenarioConfigError(f"{where}: 'rates' must be an object")
    keys = set(raw)
    if keys != set(MOVES):
        missing, extra = set(MOVES) - keys, keys - set(MOVES)
        raise ScenarioConfigError(
            f"{where}: rates must list exactly the 8 movements "
            f"(missing {sorted(missing)}, unknown {sorted(extra)})")
    out = []
    for m in MOVES:
        try:
            out.append((m, validate_rate(raw[m])))
        except ValueError as e:
            raise ScenarioConfigError(f"{where}: {m}: {e}") from None
    return tuple(out)


def _opt_ms(v, what):
    return None if v is None else _to_ms(v, what)


def _parse_scenario(sid: str, raw: dict) -> Scenario:
    where = f"scenario {sid}"
    allowed = {"description", "study", "segments", "events"}
    if not isinstance(raw, dict) or set(raw) - allowed or "segments" not in raw:
        raise ScenarioConfigError(f"{where}: keys must be a subset of {sorted(allowed)} incl. 'segments'")
    segs = []
    for i, s in enumerate(raw["segments"]):
        if set(s) != {"from", "until", "rates"}:
            raise ScenarioConfigError(f"{where} segment {i}: needs exactly from/until/rates")
        try:
            segs.append(RateSegment(_opt_ms(s["from"], "from"), _opt_ms(s["until"], "until"),
                                    _parse_rates(s["rates"], f"{where} segment {i}")))
        except ValueError as e:
            raise ScenarioConfigError(f"{where} segment {i}: {e}") from None
    # segments must tile (-inf, +inf) in order, without gaps or overlaps
    if not segs or segs[0].from_ms is not None or segs[-1].until_ms is not None:
        raise ScenarioConfigError(f"{where}: segments must start at null and end at null")
    for a, b in zip(segs, segs[1:]):
        if a.until_ms is None or a.until_ms != b.from_ms:
            raise ScenarioConfigError(f"{where}: segments must be contiguous ({a.until_ms} vs {b.from_ms})")
    for s in segs:
        if s.from_ms is not None and s.until_ms is not None and s.until_ms <= s.from_ms:
            raise ScenarioConfigError(f"{where}: empty or reversed segment")
    evs = []
    for i, e in enumerate(raw.get("events", [])):
        if set(e) != {"movement", "count", "start", "interval"}:
            raise ScenarioConfigError(f"{where} event {i}: needs exactly movement/count/start/interval")
        try:
            validate_movement(e["movement"])
        except ValueError as err:
            raise ScenarioConfigError(f"{where} event {i}: {err}") from None
        if not isinstance(e["count"], int) or isinstance(e["count"], bool) or e["count"] < 1:
            raise ScenarioConfigError(f"{where} event {i}: count must be a positive int")
        try:
            start_ms, iv_ms = _to_ms(e["start"], "start"), _to_ms(e["interval"], "interval")
        except ValueError as err:
            raise ScenarioConfigError(f"{where} event {i}: {err}") from None
        if iv_ms <= 0:
            raise ScenarioConfigError(f"{where} event {i}: interval must be > 0")
        evs.append(EventSpec(e["movement"], e["count"], start_ms, iv_ms))
    canon = json.dumps({"segments": raw["segments"], "events": raw.get("events", [])},
                       sort_keys=True, separators=(",", ":"))
    return Scenario(sid, raw.get("description", ""), tuple(segs), tuple(evs),
                    hashlib.sha256(canon.encode("utf-8")).hexdigest())


def load_scenarios(path: Union[str, Path] = SCENARIO_FILE) -> Dict[str, Scenario]:
    """Load and validate all scenarios from the JSON config."""
    with open(path, encoding="utf-8") as f:
        raw = json.load(f)
    if "scenarios" not in raw or not isinstance(raw["scenarios"], dict):
        raise ScenarioConfigError("config must contain a 'scenarios' object")
    return {sid: _parse_scenario(sid, s) for sid, s in raw["scenarios"].items()}


def constant_rate(rates: Union[float, Mapping[str, float]]) -> Tuple[RateSegment, ...]:
    """Segments for a constant demand: one q for all movements, or a per-movement mapping."""
    if _is_number(rates):
        r = {m: rates for m in MOVES}
    elif isinstance(rates, Mapping):
        r = dict(rates)
    else:
        raise ValueError(f"rates must be a number of veh/h or a movement->q mapping, got {rates!r}")
    return (RateSegment(None, None, _parse_rates(r, "constant_rate")),)


# ------------------------------------------------------------- generation
def _stream_key(m: str) -> int:
    return int.from_bytes(validate_movement(m).encode("ascii"), "big")


def _rng(seed: int, m: str) -> np.random.Generator:
    return np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, _stream_key(m)])))


def generate_movement_times(m: str, segments: Sequence[RateSegment], seed: int,
                            start: float = DEFAULT_START_S,
                            end: float = DEFAULT_END_S) -> List[int]:
    """Random-stream arrival times (ms) for one movement on [start, end)."""
    validate_movement(m)
    _validate_seed(seed)
    start_ms, end_ms = _validate_interval(start, end)
    # piecewise schedule of exponential means (None where q == 0)
    pieces = [((-math.inf if s.from_ms is None else s.from_ms / MS),
               (math.inf if s.until_ms is None else s.until_ms / MS),
               None if s.rate(m) == 0 else exp_mean(s.rate(m))) for s in segments]
    if all(p[2] is None for p in pieces):
        return []
    rng = _rng(seed, m)
    end_s = end_ms / MS
    out: List[int] = []
    prev_ms = start_ms
    while True:
        s_ms = prev_ms + _MIN_HEADWAY_MS
        if s_ms >= end_ms:
            break
        u = rng.standard_exponential()          # unit exponential "clock"
        t = s_ms / MS
        while True:
            lo, hi, mean = next(p for p in pieces if p[0] <= t < p[1])
            if mean is not None:
                need = u * mean                  # seconds at this rate
                if t + need < hi:
                    t += need
                    break
                u -= (hi - t) / mean
            t = hi
            if t >= end_s:
                break
        if t >= end_s:
            break
        t_ms = max(s_ms, round(t * MS))
        if t_ms >= end_ms:
            break
        out.append(t_ms)
        prev_ms = t_ms
    return out


def event_times(ev: EventSpec, start: float = DEFAULT_START_S,
                end: float = DEFAULT_END_S) -> List[int]:
    """Deterministic event times (ms) that fall inside [start, end)."""
    start_ms, end_ms = _validate_interval(start, end)
    ts = (ev.start_ms + k * ev.interval_ms for k in range(ev.count))
    return [t for t in ts if start_ms <= t < end_ms]


# ------------------------------------------------------------ records
@dataclass(frozen=True)
class Arrival:
    """One vehicle arrival.

    id       -- 0..N-1 in the schedule's canonical order
    time_ms  -- appearance time 150 m upstream, integer milliseconds
    movement -- one of conflicts.MOVES
    seq      -- 0-based position of this vehicle within its own movement
    source   -- "random" (headway stream) or "event" (scenario burst/schedule)
    """
    id: int
    time_ms: int
    movement: str
    seq: int
    source: str

    @property
    def time(self) -> float:
        return self.time_ms / MS


@dataclass(frozen=True)
class ScheduleMeta:
    scenario: str
    seed: int
    start_ms: int
    end_ms: int
    config_sha256: str
    generator_version: str = GENERATOR_VERSION
    format_version: str = FORMAT_VERSION


@dataclass(frozen=True)
class ArrivalSchedule:
    meta: ScheduleMeta
    arrivals: Tuple[Arrival, ...]

    def for_movement(self, m: str) -> Tuple[Arrival, ...]:
        validate_movement(m)
        return tuple(a for a in self.arrivals if a.movement == m)


def _build(meta: ScheduleMeta, per_move: Dict[str, List[Tuple[int, str]]]) -> ArrivalSchedule:
    rows = []
    for m in MOVES:
        items = sorted(per_move.get(m, []), key=lambda x: (x[0], _SOURCE_RANK[x[1]]))
        rows.extend((t, MOVES.index(m), seq, m, src) for seq, (t, src) in enumerate(items))
    rows.sort()
    return ArrivalSchedule(meta, tuple(Arrival(i, t, m, seq, src)
                                       for i, (t, _, seq, m, src) in enumerate(rows)))


def generate_arrivals(scenario: Union[str, Scenario], seed: int,
                      start: float = DEFAULT_START_S, end: float = DEFAULT_END_S,
                      scenarios: Optional[Mapping[str, Scenario]] = None) -> ArrivalSchedule:
    """Full arrival schedule for one scenario and seed on [start, end)."""
    sc = _resolve(scenario, scenarios)
    _validate_seed(seed)
    start_ms, end_ms = _validate_interval(start, end)
    per_move: Dict[str, List[Tuple[int, str]]] = {m: [] for m in MOVES}
    for m in MOVES:
        per_move[m] += [(t, SOURCE_RANDOM) for t in
                        generate_movement_times(m, sc.segments, seed, start, end)]
    for ev in sc.events:
        per_move[ev.movement] += [(t, SOURCE_EVENT) for t in event_times(ev, start, end)]
    meta = ScheduleMeta(sc.id, seed, start_ms, end_ms, sc.config_sha256)
    return _build(meta, per_move)


def _resolve(scenario, scenarios) -> Scenario:
    if isinstance(scenario, Scenario):
        return scenario
    table = scenarios if scenarios is not None else load_scenarios()
    if scenario not in table:
        raise KeyError(f"unknown scenario {scenario!r}; known: {', '.join(table)}")
    return table[scenario]


# -------------------------------------------------------------- CSV files
_COLUMNS = "id,time,movement,seq,source"
_META_KEYS = ("format_version", "generator_version", "scenario", "seed",
              "start", "end", "scenario_config_sha256", "count", "rows_sha256")


def _fmt_ms(ms: int) -> str:
    sign = "-" if ms < 0 else ""
    return f"{sign}{abs(ms) // MS}.{abs(ms) % MS:03d}"


def _parse_ms(text: str, what: str) -> int:
    try:
        d = Decimal(text)
    except InvalidOperation:
        raise ArrivalFileError(f"bad {what} value {text!r}") from None
    ms = d * MS
    if ms != ms.to_integral_value():
        raise ArrivalFileError(f"{what} {text!r} is not a whole millisecond")
    return int(ms)


def _data_lines(s: ArrivalSchedule) -> List[str]:
    return [_COLUMNS] + [f"{a.id},{_fmt_ms(a.time_ms)},{a.movement},{a.seq},{a.source}"
                         for a in s.arrivals]


def _render(s: ArrivalSchedule) -> str:
    data = _data_lines(s)
    body = "\n".join(data) + "\n"
    m = s.meta
    meta = {
        "format_version": m.format_version, "generator_version": m.generator_version,
        "scenario": m.scenario, "seed": str(m.seed),
        "start": _fmt_ms(m.start_ms), "end": _fmt_ms(m.end_ms),
        "scenario_config_sha256": m.config_sha256, "count": str(len(s.arrivals)),
        "rows_sha256": hashlib.sha256(body.encode("utf-8")).hexdigest(),
    }
    head = "# SmartTrafficLights arrival file (times in seconds, t = 0 is start of measurement)\n"
    head += "".join(f"# {k}={meta[k]}\n" for k in _META_KEYS)
    return head + body


def default_arrival_path(scenario: str, seed: int, start: float = DEFAULT_START_S,
                         end: float = DEFAULT_END_S,
                         directory: Union[str, Path] = ARRIVALS_DIR) -> Path:
    s_ms, e_ms = _validate_interval(start, end)
    name = lambda ms: str(ms // MS) if ms % MS == 0 else _fmt_ms(ms)
    return Path(directory) / f"{scenario}_seed{seed}_t{name(s_ms)}_{name(e_ms)}.csv"


def load_arrivals(path: Union[str, Path],
                  expected: Optional[ScheduleMeta] = None) -> ArrivalSchedule:
    """Read and fully validate an arrival file. If `expected` is given,
    raise ArrivalFileMismatch unless the file's metadata equals it."""
    with open(path, encoding="utf-8", newline="") as f:
        text = f.read()
    lines = text.split("\n")
    meta: Dict[str, str] = {}
    i = 0
    if not lines or not lines[0].startswith("# SmartTrafficLights arrival file"):
        raise ArrivalFileError(f"{path}: not a SmartTrafficLights arrival file")
    i = 1
    while i < len(lines) and lines[i].startswith("# "):
        k, sep, v = lines[i][2:].partition("=")
        if not sep:
            raise ArrivalFileError(f"{path}: bad metadata line {lines[i]!r}")
        meta[k] = v
        i += 1
    if set(meta) != set(_META_KEYS):
        raise ArrivalFileError(f"{path}: metadata keys {sorted(meta)} != {sorted(_META_KEYS)}")
    if meta["format_version"] != FORMAT_VERSION:
        raise ArrivalFileError(f"{path}: unsupported format_version {meta['format_version']}")
    body = "\n".join(lines[i:])
    if hashlib.sha256(body.encode("utf-8")).hexdigest() != meta["rows_sha256"]:
        raise ArrivalFileError(f"{path}: rows_sha256 does not match the data (file edited or corrupted)")
    data = [ln for ln in lines[i:] if ln != ""]
    if not data or data[0] != _COLUMNS:
        raise ArrivalFileError(f"{path}: expected column header {_COLUMNS!r}")
    try:
        seed = int(meta["seed"])
    except ValueError:
        raise ArrivalFileError(f"{path}: bad seed {meta['seed']!r}") from None
    fmeta = ScheduleMeta(meta["scenario"], seed, _parse_ms(meta["start"], "start"),
                         _parse_ms(meta["end"], "end"), meta["scenario_config_sha256"],
                         meta["generator_version"], meta["format_version"])
    per_move: Dict[str, List[Tuple[int, str]]] = {m: [] for m in MOVES}
    parsed = []
    for ln in data[1:]:
        parts = ln.split(",")
        if len(parts) != 5:
            raise ArrivalFileError(f"{path}: bad row {ln!r}")
        aid, t, m, seq, src = parts
        try:
            validate_movement(m)
        except ValueError as e:
            raise ArrivalFileError(f"{path}: {e}") from None
        if src not in _SOURCE_RANK:
            raise ArrivalFileError(f"{path}: bad source {src!r}")
        t_ms = _parse_ms(t, "time")
        if not (fmeta.start_ms <= t_ms < fmeta.end_ms):
            raise ArrivalFileError(f"{path}: arrival at {t} outside [start, end)")
        per_move[m].append((t_ms, src))
        parsed.append(Arrival(int(aid), t_ms, m, int(seq), src))
    if len(parsed) != int(meta["count"]):
        raise ArrivalFileError(f"{path}: count {meta['count']} != {len(parsed)} rows")
    rebuilt = _build(fmeta, per_move)
    if rebuilt.arrivals != tuple(parsed):
        raise ArrivalFileError(f"{path}: rows are not in canonical order / ids or seq inconsistent")
    if expected is not None and expected != fmeta:
        raise ArrivalFileMismatch(f"{path}: metadata mismatch: {_diff(expected, fmeta)}")
    return rebuilt


def _diff(a: ScheduleMeta, b: ScheduleMeta) -> str:
    fields = a.__dataclass_fields__
    return "; ".join(f"{k}: expected {getattr(a, k)!r}, file has {getattr(b, k)!r}"
                     for k in fields if getattr(a, k) != getattr(b, k))


def save_arrivals(schedule: ArrivalSchedule, path: Optional[Union[str, Path]] = None,
                  overwrite: bool = False) -> Path:
    """Write the schedule as CSV (LF line endings, identical bytes on every OS).

    If the file already exists and overwrite is False: it is left untouched
    when it holds exactly this schedule, otherwise ArrivalFileMismatch is raised.
    """
    m = schedule.meta
    p = Path(path) if path is not None else default_arrival_path(
        m.scenario, m.seed, m.start_ms / MS, m.end_ms / MS)
    if p.exists() and not overwrite:
        try:
            existing = load_arrivals(p, expected=m)
        except ArrivalFileMismatch:
            raise
        except ArrivalFileError as e:
            raise ArrivalFileMismatch(f"{p}: existing file is not a valid match ({e})") from None
        if existing.arrivals != schedule.arrivals:
            raise ArrivalFileMismatch(f"{p}: existing file has different arrivals")
        return p
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + ".tmp")
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        f.write(_render(schedule))
    os.replace(tmp, p)
    return p


def get_or_create_arrivals(scenario: str, seed: int, start: float = DEFAULT_START_S,
                           end: float = DEFAULT_END_S,
                           directory: Union[str, Path] = ARRIVALS_DIR,
                           scenarios: Optional[Mapping[str, Scenario]] = None
                           ) -> Tuple[ArrivalSchedule, Path]:
    """Load the arrival file for (scenario, seed, interval), creating it if absent.

    This is the single entry point the experiments should use, so Fixed and
    Smart read the same file. A file whose metadata no longer matches the
    current config or generator version raises ArrivalFileMismatch.
    """
    sc = _resolve(scenario, scenarios)
    _validate_seed(seed)
    s_ms, e_ms = _validate_interval(start, end)
    path = default_arrival_path(sc.id, seed, start, end, directory)
    expected = ScheduleMeta(sc.id, seed, s_ms, e_ms, sc.config_sha256)
    if path.exists():
        return load_arrivals(path, expected=expected), path
    sched = generate_arrivals(sc, seed, start, end)
    save_arrivals(sched, path)
    return sched, path
