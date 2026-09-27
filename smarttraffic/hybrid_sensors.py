"""hybrid_sensors -- compose several DetectorSources into the 16 logical detectors (Stage 9).

Every logical detector (movement, "D1"/"D2") is assigned to exactly ONE source.
HybridDetectorSource forwards each source's edges only for the detectors
assigned to it, so a virtual source that also computes an overridden detector
never produces a controller-visible event for it. SensorBank consumes the
result like any other DetectorSource and cannot tell real from virtual.

No hardware, serial or clock code here.
"""
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from smarttraffic.conflicts import MOVES
from smarttraffic.sensors import DETECTORS, SENSOR_IDS, DetectorEdge

__all__ = ["HybridDetectorSource", "SensorAssignmentError", "parse_sensor_id", "format_sensor_id"]

SensorId = Tuple[str, str]


class SensorAssignmentError(ValueError):
    """The logical-detector -> source assignment is incomplete, duplicated or invalid."""


def parse_sensor_id(text: str) -> SensorId:
    """'NS:D1' -> ('NS', 'D1'), validated against the 16 logical detectors."""
    if not isinstance(text, str) or text.count(":") != 1:
        raise SensorAssignmentError(f"sensor id must look like 'NS:D1', got {text!r}")
    m, d = text.split(":")
    if (m, d) not in SENSOR_IDS:
        raise SensorAssignmentError(f"unknown sensor id {text!r}; movements {', '.join(MOVES)}, "
                                    f"detectors {', '.join(DETECTORS)}")
    return (m, d)


def format_sensor_id(sid: SensorId) -> str:
    return f"{sid[0]}:{sid[1]}"


def _order(sid: SensorId) -> Tuple[int, int]:
    return MOVES.index(sid[0]), DETECTORS.index(sid[1])


class HybridDetectorSource:
    """DetectorSource built from an assignment {sensor_id: source} covering all 16 detectors."""
    sensor_ids = SENSOR_IDS

    def __init__(self, assignment: Mapping[SensorId, object]):
        if not isinstance(assignment, Mapping):
            raise SensorAssignmentError("assignment must be a mapping {(movement, detector): source}")
        keys = list(assignment)
        unknown = [k for k in keys if k not in SENSOR_IDS]
        if unknown:
            raise SensorAssignmentError(f"unknown sensor ids: {unknown}")
        missing = [s for s in SENSOR_IDS if s not in assignment]
        if missing:
            raise SensorAssignmentError("every detector needs a source; missing: "
                                        + ", ".join(format_sensor_id(s) for s in missing))
        self._assign: Dict[SensorId, object] = dict(assignment)
        self._sources: List[object] = []
        for sid in SENSOR_IDS:
            src = self._assign[sid]
            if sid not in tuple(getattr(src, "sensor_ids", ())):
                raise SensorAssignmentError(f"{format_sensor_id(sid)} is assigned to a source "
                                            f"that does not provide it")
            if not any(src is s for s in self._sources):
                self._sources.append(src)
        nows = {s.now_ms for s in self._sources}
        if len(nows) != 1:
            raise SensorAssignmentError(f"sources start at different times: {sorted(nows)}")
        self._now = nows.pop()
        self.dropped_overridden = 0          # edges computed by a source for a detector it does not own

    @classmethod
    def with_overrides(cls, default: object,
                       overrides: Iterable[Tuple[SensorId, object]]) -> "HybridDetectorSource":
        """All detectors from `default`, except the (sensor_id, source) pairs in `overrides`.
        A sensor id listed twice is rejected."""
        assignment: Dict[SensorId, object] = {sid: default for sid in SENSOR_IDS}
        seen = set()
        for sid, src in overrides:
            if sid not in SENSOR_IDS:
                raise SensorAssignmentError(f"unknown sensor id {sid!r}")
            if sid in seen:
                raise SensorAssignmentError(f"sensor {format_sensor_id(sid)} assigned twice")
            seen.add(sid)
            assignment[sid] = src
        return cls(assignment)

    # -------------------------------------------------- DetectorSource API
    @property
    def now_ms(self) -> int:
        return self._now

    def initial_occupancy(self) -> Dict[SensorId, bool]:
        occ: Dict[SensorId, bool] = {}
        for src in self._sources:
            init = src.initial_occupancy()
            for sid in SENSOR_IDS:
                if self._assign[sid] is src:
                    occ[sid] = bool(init[sid])
        return occ

    def poll(self, until_ms: int) -> List[DetectorEdge]:
        if until_ms < self._now:
            raise ValueError(f"cannot poll backwards ({until_ms} < {self._now})")
        merged = []
        for k, src in enumerate(self._sources):
            for i, e in enumerate(src.poll(until_ms)):
                sid = (e.movement, e.detector)
                if self._assign.get(sid) is not src:
                    self.dropped_overridden += 1
                    continue
                merged.append(((e.time_ms,) + _order(sid) + (k, i), e))
        self._now = until_ms
        merged.sort(key=lambda x: x[0])
        return [e for _, e in merged]

    def faults(self) -> Tuple[SensorId, ...]:
        """Detectors whose assigned source currently reports a fault (never the virtual ones)."""
        out = set()
        for src in self._sources:
            f = getattr(src, "faults", None)
            if f is None:
                continue
            out |= {sid for sid in f() if self._assign.get(sid) is src}
        return tuple(sorted(out, key=_order))

    # -------------------------------------------------------- inspection
    def source_of(self, sid: SensorId) -> object:
        """The source object assigned to `sid` (for setup/diagnostics, not for controllers)."""
        if sid not in self._assign:
            raise SensorAssignmentError(f"unknown sensor id {sid!r}")
        return self._assign[sid]
