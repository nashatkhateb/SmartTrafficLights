"""viz_geometry -- pure drawing geometry of the 8-movement intersection (Stage 13).

No pygame, no simulation state: only where things are, in metres. Used by viz.py and
tested directly (tests/test_viz.py).

World frame
-----------
Origin at the centre of the junction box, x to the EAST, y to the SOUTH (screen
orientation, north is up). Right-hand traffic. Every approach is built in the local
frame of the NORTH approach and rotated:

    local North frame: vehicles travel +y (southbound) on the WEST half of the north arm
        LEFT lane      x = -2.4 m   (inner lane, next to the painted median)
        STRAIGHT lane  x = -6.0 m   (outer lane; a 2.2 m cycle lane lies between it and the kerb)
        stop line      y = -HALF (-10 m), box edge
    approach rotation (clockwise, screen): N 0, E 90, S 180, W 270 degrees

Paths are parametrised by u = metres of travel with u = 0 at the stop line:
    u = -position on the approach (VehicleModel.position is metres upstream, > 0),
    0 <= u <= box length inside the box (position = -u, BOX_LENGTH straight 20 / left 25),
    u > box length on the exit arm (purely visual continuation).
Straight: one line through the box (the 20 m box length equals the box width).
Left: a smooth cubic curve (tangent to both lanes) from the approach's inner lane into
the inner outbound lane of the arm on the left; the model's 25 m box length is mapped
proportionally onto the curve's arc length. The curve is tighter than a quarter circle so
two opposing left turns (a compatible pair) pass each other with room to spare.

Signals: 8 heads, two per approach (STRAIGHT and LEFT), no right-turn head. The two heads
of an approach sit on the kerb corner on the driver's right, just upstream of the stop
line, in driver's-view order (LEFT head nearer the road centre, like the lanes).
Detectors: 16 logical positions, D1 at 2 m and D2 at 32 m upstream in each lane
(sensors.DETECTOR_POSITION); a sensor id listed as real (config/hardware.json) is
drawn as the real detector IN PLACE OF its virtual twin -- never as a 17th marker.
"""
from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, NamedTuple, Optional, Tuple

from smarttraffic.conflicts import MOVES, validate_movement
from smarttraffic.sensors import DETECTOR_POSITION, DETECTORS, SENSOR_IDS
from smarttraffic.signals import movement_type
from smarttraffic.vehicles import BOX_LENGTH, VEHICLE_LENGTH

__all__ = ["APPROACHES", "APPROACH_NAMES", "ROTATION_DEG", "LANE_WIDTH", "HALF", "LANE_OFFSET",
           "MEDIAN_HALF", "LANE_DIVIDER", "EDGE_LINE",
           "LEFT_CONTROL", "VEHICLE_WIDTH", "Vec", "SignalHead", "DetectorSpot", "Lane",
           "approach_of", "movement_of", "rotate", "local_to_world", "travel_dir",
           "right_dir", "path_point", "path_heading", "vehicle_pose", "lanes",
           "signal_heads", "detector_spots", "left_turn_exit_arm", "straight_exit_arm",
           "box_polygon", "arm_polygon", "left_turn_arc"]

Vec = Tuple[float, float]

APPROACHES: Tuple[str, ...] = ("N", "E", "S", "W")
APPROACH_NAMES = {"N": "North", "E": "East", "S": "South", "W": "West"}
ROTATION_DEG = {"N": 0, "E": 90, "S": 180, "W": 270}

HALF = 10.0                            # m: half width of the box and of each arm (kerb to kerb)
LANE_WIDTH = 3.6                       # m: one traffic lane
MEDIAN_HALF = 0.6                      # m: painted median, half width (double centre line)
LANE_DIVIDER = MEDIAN_HALF + LANE_WIDTH            # |x| of the line between the two lanes (4.2 m)
EDGE_LINE = MEDIAN_HALF + 2 * LANE_WIDTH           # |x| of the lane edge line (7.8 m)
# beyond the edge line, up to the kerb at HALF: a 2.2 m cycle lane (drawing only)
LANE_OFFSET = {"left": -(MEDIAN_HALF + 0.5 * LANE_WIDTH),         # -2.4 m (inner lane)
               "straight": -(MEDIAN_HALF + 1.5 * LANE_WIDTH)}     # -6.0 m (outer lane)
LEFT_CONTROL = 4.5                     # m: Bezier control distance of the left-turn curve
VEHICLE_WIDTH = 1.85                   # m (drawing only)

assert BOX_LENGTH["straight"] == 2 * HALF, "straight path must span the box exactly"

_CW = ("N", "E", "S", "W")             # clockwise order on screen


def approach_of(m: str) -> str:
    return validate_movement(m)[0]


def movement_of(approach: str, kind: str) -> str:
    if approach not in APPROACHES or kind not in ("straight", "left"):
        raise ValueError(f"bad approach/kind {approach!r}/{kind!r}")
    return approach + ("S" if kind == "straight" else "L")


def rotate(p: Vec, deg: int) -> Vec:
    """Exact rotation by a multiple of 90 degrees (clockwise on screen)."""
    c = {0: 1, 90: 0, 180: -1, 270: 0}[deg % 360]
    s = {0: 0, 90: 1, 180: 0, 270: -1}[deg % 360]
    x, y = p
    return (x * c - y * s, x * s + y * c)


def local_to_world(approach: str, p: Vec) -> Vec:
    return rotate(p, ROTATION_DEG[approach])


def travel_dir(approach: str) -> Vec:
    """Unit vector in which the approach's vehicles travel towards the box."""
    return local_to_world(approach, (0.0, 1.0))


def right_dir(approach: str) -> Vec:
    """Unit vector to the driver's right (towards the kerb of the inbound half)."""
    return local_to_world(approach, (-1.0, 0.0))


def straight_exit_arm(approach: str) -> str:
    return _CW[(_CW.index(approach) + 2) % 4]


def left_turn_exit_arm(approach: str) -> str:
    """Right-hand traffic: a left turn from N (southbound) ends eastbound on the E arm."""
    return _CW[(_CW.index(approach) + 1) % 4]


# ------------------------------------------------------------------ paths
# left turn (North frame): from (-2.5, -10) heading south to (10, 2.5) heading east
_L0 = (LANE_OFFSET["left"], -HALF)
_L3 = (HALF, -LANE_OFFSET["left"])
_L1 = (_L0[0], _L0[1] + LEFT_CONTROL)
_L2 = (_L3[0] - LEFT_CONTROL, _L3[1])


def _bez(s: float) -> Vec:
    a, b, c, d = (1 - s) ** 3, 3 * (1 - s) ** 2 * s, 3 * (1 - s) * s ** 2, s ** 3
    return (a * _L0[0] + b * _L1[0] + c * _L2[0] + d * _L3[0],
            a * _L0[1] + b * _L1[1] + c * _L2[1] + d * _L3[1])


def _bez_d(s: float) -> Vec:
    x = 3 * (1 - s) ** 2 * (_L1[0] - _L0[0]) + 6 * (1 - s) * s * (_L2[0] - _L1[0]) + 3 * s ** 2 * (_L3[0] - _L2[0])
    y = 3 * (1 - s) ** 2 * (_L1[1] - _L0[1]) + 6 * (1 - s) * s * (_L2[1] - _L1[1]) + 3 * s ** 2 * (_L3[1] - _L2[1])
    n = math.hypot(x, y)
    return (x / n, y / n)


_N_TABLE = 400
_ARC: List[float] = [0.0]
for _i in range(1, _N_TABLE + 1):
    _p, _q = _bez((_i - 1) / _N_TABLE), _bez(_i / _N_TABLE)
    _ARC.append(_ARC[-1] + math.hypot(_q[0] - _p[0], _q[1] - _p[1]))
LEFT_ARC_LENGTH = _ARC[-1]


def _bez_param(frac: float) -> float:
    """Curve parameter at a fraction of the arc length (monotone table lookup)."""
    target = frac * LEFT_ARC_LENGTH
    lo, hi = 0, _N_TABLE
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if _ARC[mid] <= target:
            lo = mid
        else:
            hi = mid
    seg = _ARC[hi] - _ARC[lo]
    f = 0.0 if seg <= 0 else (target - _ARC[lo]) / seg
    return (lo + f) / _N_TABLE


def _local_path(kind: str, u: float) -> Tuple[Vec, Vec]:
    """(point, unit heading) in the North local frame."""
    x0 = LANE_OFFSET[kind]
    if kind == "straight" or u <= 0:
        return (x0, -HALF + u), (0.0, 1.0)
    box = BOX_LENGTH["left"]
    if u >= box:                        # exit arm: keep the on-screen speed of the curve
        return (_L3[0] + (u - box) * LEFT_ARC_LENGTH / box, _L3[1]), (1.0, 0.0)
    s = _bez_param(u / box)
    return _bez(s), _bez_d(s)


def path_point(m: str, u: float) -> Vec:
    p, _ = _local_path(movement_type(m), u)
    return local_to_world(approach_of(m), p)


def path_heading(m: str, u: float) -> Vec:
    _, h = _local_path(movement_type(m), u)
    return local_to_world(approach_of(m), h)


def vehicle_pose(m: str, u_front: float) -> Tuple[Vec, Vec]:
    """(centre, heading) of a vehicle body whose FRONT bumper is at path coordinate
    u_front; the body is VEHICLE_LENGTH long and follows the path (centre at mid-body)."""
    uc = u_front - VEHICLE_LENGTH / 2
    return path_point(m, uc), path_heading(m, uc)


def left_turn_arc(m: str, n: int = 24) -> List[Vec]:
    """Polyline of m's path through the box (straight: 2 points)."""
    box = BOX_LENGTH[movement_type(m)]
    k = 1 if movement_type(m) == "straight" else n
    return [path_point(m, box * i / k) for i in range(k + 1)]


# ------------------------------------------------------------------ static layout
class Lane(NamedTuple):
    movement: str
    approach: str
    kind: str
    stop_line: Tuple[Vec, Vec]        # world endpoints across the lane at the stop line
    centre_at_stop: Vec


def lanes() -> Tuple[Lane, ...]:
    out = []
    for m in MOVES:
        a, kind = approach_of(m), movement_type(m)
        x0 = LANE_OFFSET[kind]
        ends = (local_to_world(a, (x0 - LANE_WIDTH / 2, -HALF)),
                local_to_world(a, (x0 + LANE_WIDTH / 2, -HALF)))
        out.append(Lane(m, a, kind, ends, local_to_world(a, (x0, -HALF))))
    return tuple(out)


@dataclass(frozen=True)
class SignalHead:
    """One of the 8 logical signal heads (one per movement)."""
    movement: str
    approach: str
    kind: str                         # "straight" | "left"
    corner: Vec                       # kerb corner at the stop line on the driver's right
    slot: int                         # 0 = nearer the road (LEFT head), 1 = outer (STRAIGHT)
    travel: Vec                       # lamps run red (upstream) -> green (towards the box)
    outward: Vec                      # from the road towards the kerb side


def signal_heads() -> Tuple[SignalHead, ...]:
    heads = []
    for m in MOVES:
        a, kind = approach_of(m), movement_type(m)
        heads.append(SignalHead(m, a, kind, local_to_world(a, (-HALF, -HALF)),
                                0 if kind == "left" else 1, travel_dir(a), right_dir(a)))
    return tuple(heads)


@dataclass(frozen=True)
class DetectorSpot:
    movement: str
    detector: str                     # "D1" | "D2"
    upstream_m: float                 # metres before the stop line
    centre: Vec                       # world
    along: Vec                        # travel direction
    real: bool                        # the physical detector's slot (replaces the virtual one)


def detector_spots(real_ids: Iterable[Tuple[str, str]] = ()) -> Tuple[DetectorSpot, ...]:
    """The 16 logical detector positions, MOVES x (D1, D2) order. `real_ids` marks which
    of them are served by real hardware; they REPLACE the virtual position (never added)."""
    real = set()
    for sid in real_ids:
        if tuple(sid) not in SENSOR_IDS:
            raise ValueError(f"unknown sensor id {sid!r}")
        real.add(tuple(sid))
    out = []
    for m in MOVES:
        for d in DETECTORS:
            up = float(DETECTOR_POSITION[d])
            out.append(DetectorSpot(m, d, up, path_point(m, -up), path_heading(m, -up),
                                    (m, d) in real))
    return tuple(out)


def box_polygon() -> List[Vec]:
    return [(-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF)]


def arm_polygon(approach: str, length: float) -> List[Vec]:
    """The full road arm (inbound + outbound halves) from the box edge outwards."""
    pts = [(-HALF, -HALF), (HALF, -HALF), (HALF, -HALF - length), (-HALF, -HALF - length)]
    return [local_to_world(approach, p) for p in pts]
