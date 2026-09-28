"""viz_render -- pygame drawing for the Stage 13 visualization.

Pure presentation: every function here draws a Snapshot (viz_session) with the static
geometry of viz_geometry. Nothing reads or writes simulation objects, and there is no
traffic-light logic: lamps come from Snapshot.lamps() = signals.lamps_for().

Scene (world metres, x east, y south, light from the north-west):
    asphalt with fine grain, wheel tracks and oil marks; worn white road paint (stop
    lines, lane lines, zebra crossings, lane-use arrows); kerbs with rounded returns;
    paved sidewalks; city blocks with buildings and street trees; per approach a signal
    pole carrying the STRAIGHT and the LEFT head; inductive loops for the 16 detectors
    and the real IR detector drawn as a kerb-side sensor with its beam in its own slot.
UI: top bar, a sidebar of cards (controls, live metrics, controller) and floating glass
cards on the canvas (hardware, legend, scale).
"""
import functools
import math
import random
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import pygame
import pygame.gfxdraw as gfx

from smarttraffic.conflicts import MOVES
from smarttraffic.custom import VEHICLE_POLICY_LABEL
from smarttraffic.runner import CONTROLLERS
from smarttraffic.signals import SignalState, movement_type
from smarttraffic.vehicles import VEHICLE_LENGTH
from smarttraffic.viz_geometry import (EDGE_LINE, LANE_DIVIDER, MEDIAN_HALF, VEHICLE_WIDTH)
from smarttraffic.viz_geometry import (APPROACH_NAMES, APPROACHES, HALF, LANE_OFFSET, LANE_WIDTH,
                                       detector_spots, left_turn_arc, local_to_world, path_heading,
                                       path_point, right_dir, signal_heads, travel_dir, vehicle_pose)
from smarttraffic.viz_session import HARDWARE_SPEED, SPEEDS, Snapshot, run_phase

__all__ = ["Theme", "T", "Fonts", "View", "Renderer", "SCENARIOS", "controllers_for", "lamp_colour",
           "vehicle_paint"]


def _load_scenarios() -> List[Tuple[str, str]]:
    from smarttraffic.traffic_gen import load_scenarios
    return [(k, v.description) for k, v in load_scenarios().items()]


SCENARIOS: List[Tuple[str, str]] = _load_scenarios()
CUSTOM_ID, CUSTOM_BADGE = "CUSTOM", "CUSTOM / DEMO"


def smart_label(params) -> str:
    """'Smart-T1' for the locked parameters, otherwise 'custom parameters'."""
    try:
        from smarttraffic.viz import smart_t1_params
        return "Smart-T1" if params == smart_t1_params() else "custom parameters"
    except Exception:
        return "Smart"


def scenario_title(config) -> Tuple[str, str]:
    """(badge, description) of the running configuration, official or Custom / Demo."""
    if getattr(config, "mode", "official") == "custom":
        return "CUSTOM", "Custom / Demo demand"
    return config.scenario, dict(SCENARIOS).get(config.scenario, "")


@functools.lru_cache(maxsize=None)
def controllers_for(scenario: str) -> Tuple[str, ...]:
    """Controllers the runner supports for a scenario (FT-TOD exists for S6 only)."""
    from smarttraffic.controller_fixed import load_fixed_plans
    book = load_fixed_plans()
    out = []
    for c in CONTROLLERS:
        if c == "Smart":
            out.append(c)
            continue
        sc = book.scenarios(c)
        if "*" in sc or scenario in sc:
            out.append(c)
    return tuple(out)


# ================================================================ theme
def _hex(h: str):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def _mix(a, b, t: float):
    return tuple(round(a[i] * (1 - t) + b[i] * t) for i in range(3))


class Theme:
    # interface
    bg = _hex("0a0e13")
    bar = _hex("0f141a")
    panel = _hex("121820")
    inset = _hex("0c1116")
    raised = _hex("1a222c")
    border = _hex("222b36")
    border_hi = _hex("2e3946")
    text = _hex("e9eef3")
    text2 = _hex("a3adb8")
    muted = _hex("67737f")
    faint = _hex("3b4550")
    accent = _hex("4f9dff")
    accent_soft = _hex("15273d")
    # signals
    red = _hex("ff4a4d")
    amber = _hex("ffb21e")
    green = _hex("27d17a")
    lamp_off = _hex("171c22")
    housing = _hex("08090b")
    # scene
    ground = _hex("161c22")
    block = _hex("171d24")
    sidewalk = _hex("2a3037")
    sidewalk_joint = _hex("252a31")
    kerb = _hex("4a535d")
    kerb_shadow = _hex("121518")
    asphalt = _hex("22272d")
    paint = _hex("c3c9cf")
    roof = (_hex("1e242c"), _hex("21272f"), _hex("1b2129"), _hex("242a33"), _hex("1f252d"))
    roof_edge = _hex("2c343e")
    tree = _hex("1d3529")
    tree_hi = _hex("284a38")
    det_loop = _hex("101418")
    det_on = _hex("38d3c6")
    real = _hex("ff78c4")
    fault = _hex("ff4a4d")
    glass = _hex("1a2530")
    custom = _hex("c08cff")               # CUSTOM / DEMO badge (Stage 14b)


T = Theme

PAINTS = [(_hex("e6e9ec"), 5), (_hex("b8bfc7"), 4), (_hex("6d7680"), 3), (_hex("2a2f36"), 3),
          (_hex("3c5f93"), 2), (_hex("a8403c"), 2), (_hex("4d7fb8"), 1), (_hex("3e7f7a"), 1),
          (_hex("c7b48b"), 1), (_hex("5b7a55"), 1), (_hex("7b5d8f"), 1)]
_PAINT_TABLE = [c for c, w in PAINTS for _ in range(w)]


def vehicle_paint(vid: int):
    """Deterministic car colour per vehicle id (display only)."""
    return _PAINT_TABLE[(vid * 2654435761 >> 7) % len(_PAINT_TABLE)]


def lamp_colour(state: SignalState, lit_green: bool = True):
    if state in (SignalState.RED, SignalState.ALL_RED, SignalState.RED_YELLOW):
        return T.red
    if state is SignalState.GREEN:
        return T.green
    if state is SignalState.FLASHING_GREEN:
        return T.green if lit_green else _mix(T.green, T.panel, 0.7)
    return T.amber


STATE_SHORT = {SignalState.RED: "RED", SignalState.RED_YELLOW: "RED+YEL", SignalState.GREEN: "GREEN",
               SignalState.FLASHING_GREEN: "FLASH", SignalState.YELLOW: "YELLOW",
               SignalState.ALL_RED: "ALL-RED"}


def fmt_clock(ms: float) -> str:
    neg = ms < 0
    s = abs(ms) / 1000
    m, s = divmod(s, 60)
    return f"{'-' if neg else ''}{int(m):02d}:{s:04.1f}"


# ================================================================ fonts
class Fonts:
    FAMILY = "segoeui,inter,roboto,helveticaneue,arial,poppins,dejavusans"
    FAMILY_BOLD = "segoeuisemibold,segoeui,inter,roboto,helveticaneue,arial,poppins,dejavusans"
    _semi = None

    def __init__(self):
        self._f: Dict[Tuple[int, bool], object] = {}
        self._cache: Dict[tuple, object] = {}

    def font(self, size: int, bold: bool = False):
        key = (size, bold)
        if key not in self._f:
            if bold:
                if Fonts._semi is None:
                    Fonts._semi = pygame.font.match_font("segoeuisemibold") is not None
                f = pygame.font.SysFont(self.FAMILY_BOLD, size, bold=not Fonts._semi)
            else:
                f = pygame.font.SysFont(self.FAMILY, size)
            self._f[key] = f
        return self._f[key]

    def render(self, text: str, size: int, colour, bold: bool = False, spacing: int = 0,
               tabular: bool = False):
        key = (text, size, colour, bold, spacing, tabular)
        s = self._cache.get(key)
        if s is None:
            if len(self._cache) > 4000:
                self._cache.clear()
            f = self.font(size, bold)
            if spacing or tabular:
                parts = [f.render(ch, True, colour) for ch in text]
                dw = max(f.size(d)[0] for d in "0123456789") if tabular else 0
                widths = [dw if (tabular and ch.isdigit()) else p.get_width() for ch, p in zip(text, parts)]
                w = sum(widths) + spacing * max(0, len(parts) - 1)
                s = pygame.Surface((max(1, w), f.get_height()), pygame.SRCALPHA)
                x = 0
                for p, pw in zip(parts, widths):
                    s.blit(p, (x + (pw - p.get_width()) // 2, 0))
                    x += pw + spacing
            else:
                s = f.render(text, True, colour)
            self._cache[key] = s
        return s


def text(surf, fonts: Fonts, s: str, pos, size: int, colour, bold=False, anchor="topleft", spacing=0,
         tabular=False):
    img = fonts.render(s, size, colour, bold, spacing, tabular)
    r = img.get_rect(**{anchor: (round(pos[0]), round(pos[1]))})
    surf.blit(img, r)
    return r


# ================================================================ primitives
def aa_poly(surf, pts, colour):
    ip = [(round(x), round(y)) for x, y in pts]
    if surf.get_flags() & pygame.SRCALPHA:
        pygame.draw.polygon(surf, colour, ip)          # gfxdraw would corrupt the alpha channel
        return
    gfx.aapolygon(surf, ip, colour)
    gfx.filled_polygon(surf, ip, colour)


def aa_circle(surf, c, r, colour):
    x, y, r = round(c[0]), round(c[1]), max(1, round(r))
    if surf.get_flags() & pygame.SRCALPHA:
        pygame.draw.circle(surf, colour, (x, y), r)
        return
    gfx.aacircle(surf, x, y, r, colour)
    gfx.filled_circle(surf, x, y, r, colour)


def thick_line(surf, a, b, w, colour):
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return
    ox, oy = -dy / n * w / 2, dx / n * w / 2
    aa_poly(surf, [(a[0] + ox, a[1] + oy), (b[0] + ox, b[1] + oy), (b[0] - ox, b[1] - oy),
                   (a[0] - ox, a[1] - oy)], colour)


def thick_polyline(surf, pts, w, colour):
    for a, b in zip(pts, pts[1:]):
        thick_line(surf, a, b, w, colour)
    if w >= 3:
        for p in pts[1:-1]:
            aa_circle(surf, p, w / 2 - 0.5, colour)


def strip_polygon(pts, w: float):
    left, right = [], []
    for i, p in enumerate(pts):
        a, b = pts[max(0, i - 1)], pts[min(len(pts) - 1, i + 1)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy) or 1.0
        ox, oy = -dy / n * w / 2, dx / n * w / 2
        left.append((p[0] + ox, p[1] + oy))
        right.append((p[0] - ox, p[1] - oy))
    return left + right[::-1]


def rounded_poly(cx, cy, hx, hy, nx, ny, half_l, half_w, rf, rr, steps=3):
    """Rounded rectangle oriented by heading (hx, hy): front corner radius rf, rear rr (world)."""
    pts = []
    # corners in angular order: front/+b (0-90), rear/+b (90-180), rear/-b (180-270), front/-b
    corners = [(half_l - rf, half_w - rf, rf, 0), (-half_l + rr, half_w - rr, rr, 1),
               (-half_l + rr, -half_w + rr, rr, 2), (half_l - rf, -half_w + rf, rf, 3)]
    for a0, b0, r, q in corners:
        for i in range(steps + 1):
            ang = (q * 90 + 90 * i / steps) * math.pi / 180
            a = a0 + r * math.cos(ang)
            b = b0 + r * math.sin(ang)
            pts.append((cx + hx * a + nx * b, cy + hy * a + ny * b))
    return pts


_glow_cache: Dict[tuple, object] = {}


def glow(surf, c, r: int, colour, strength: int = 110):
    r = max(2, int(r))
    key = (r, colour, strength)
    g = _glow_cache.get(key)
    if g is None:
        size = r * 2 + 2
        g = pygame.Surface((size, size), pygame.SRCALPHA)
        steps = max(8, r)
        for i in range(steps, 0, -1):
            a = round(strength * (1 - i / steps) ** 1.8)
            if a > 0:
                pygame.draw.circle(g, colour + (a,), (r + 1, r + 1), r * i / steps)
        _glow_cache[key] = g
    surf.blit(g, (round(c[0]) - r - 1, round(c[1]) - r - 1))


def rrect(surf, rect, colour, radius: int, border=None, width: int = 1):
    pygame.draw.rect(surf, colour, rect, border_radius=radius)
    if border is not None:
        pygame.draw.rect(surf, border, rect, width=width, border_radius=radius)


_glass_cache: Dict[tuple, object] = {}


def glass(surf, rect, radius: int, alpha: int = 232):
    """Floating translucent card on the canvas (cached per size)."""
    key = (rect.size, radius, alpha)
    g = _glass_cache.get(key)
    if g is None:
        g = pygame.Surface(rect.size, pygame.SRCALPHA)
        pygame.draw.rect(g, T.panel + (alpha,), g.get_rect(), border_radius=radius)
        pygame.draw.rect(g, T.border_hi + (255,), g.get_rect(), width=1, border_radius=radius)
        pygame.draw.line(g, (255, 255, 255, 14), (radius, 1), (rect.w - radius, 1))
        _glass_cache[key] = g
    sh = rect.move(0, 4)
    shadow = _glass_cache.get(("sh",) + key)
    if shadow is None:
        shadow = pygame.Surface(rect.inflate(16, 16).size, pygame.SRCALPHA)
        for i in range(8, 0, -1):
            pygame.draw.rect(shadow, (0, 0, 0, 10), shadow.get_rect().inflate(-2 * i, -2 * i),
                             border_radius=radius + i)
        _glass_cache[("sh",) + key] = shadow
    surf.blit(shadow, sh.inflate(16, 16))
    surf.blit(g, rect)


def arrow_glyph(kind: str, travel, left):
    """Unit-size lens/lane arrow: [stroke points, head triangle], +travel = direction of travel."""
    tx, ty = travel
    lx, ly = left

    def P(a, b):
        return (a * tx + b * lx, a * ty + b * ly)
    if kind == "straight":
        return [P(-0.62, 0), P(0.18, 0)], [P(0.64, 0), P(0.12, 0.36), P(0.12, -0.36)]
    arc = []
    for i in range(7):
        ang = math.pi / 2 * i / 6
        arc.append(P(-0.02 + 0.3 * math.sin(ang), -0.28 + 0.3 * (1 - math.cos(ang))))
    stroke = [P(-0.62, -0.28), P(-0.02, -0.28)] + arc[1:] + [P(0.28, 0.16)]
    return stroke, [P(0.28, 0.64), P(-0.06, 0.14), P(0.62, 0.14)]


def draw_arrow(surf, centre, size, kind, travel, left, colour, width):
    stroke, head = arrow_glyph(kind, travel, left)
    thick_polyline(surf, [(centre[0] + x * size, centre[1] + y * size) for x, y in stroke], width, colour)
    aa_poly(surf, [(centre[0] + x * size, centre[1] + y * size) for x, y in head], colour)


ARROW_HALF_W = 0.12                     # half width of the arrow shaft (unit lens radius)


def lens_arrow(kind: str, travel, left):
    """Arrow symbol of a signal lens, unit size, screen space: (shaft centre line, head triangle).

    Drawn like an Israeli arrow signal (sign 704 style): a solid shaft and a solid triangular
    head, lit in the lamp colour on the black lens mask. +travel is the direction the controlled
    vehicles drive (N approach: down, S: up, E: left, W: right) and `left` the driver's left.
    STRAIGHT points along travel; LEFT runs along travel and turns 90 degrees (sharp corner)
    towards the driver's left."""
    tx, ty = travel
    lx, ly = left

    def P(a, b):                        # a along travel, b towards the driver's left
        return (a * tx + b * lx, a * ty + b * ly)
    if kind == "straight":
        return [P(-0.64, 0.0), P(0.12, 0.0)], [P(0.66, 0.0), P(0.10, 0.42), P(0.10, -0.42)]
    ca, cb = 0.20, -0.30                # the corner of the L
    return ([P(-0.64, cb), P(ca, cb), P(ca, 0.14)],
            [P(ca, 0.68), P(ca - 0.40, 0.12), P(ca + 0.40, 0.12)])


def draw_lens_arrow(surf, centre, size, kind, travel, left, colour, width=None):
    """Solid arrow: square-ended shaft segments with a mitred corner, then the head."""
    shaft, head = lens_arrow(kind, travel, left)
    S = lambda p: (centre[0] + p[0] * size, centre[1] + p[1] * size)
    hw = ARROW_HALF_W
    for i, (p, q) in enumerate(zip(shaft, shaft[1:])):
        dx, dy = q[0] - p[0], q[1] - p[1]
        n = math.hypot(dx, dy)
        ux, uy = dx / n, dy / n
        nx, ny = -uy * hw, ux * hw
        ext = hw if i < len(shaft) - 2 else 0.0          # overlap into the corner (mitre)
        q2 = (q[0] + ux * ext, q[1] + uy * ext)
        aa_poly(surf, [S((p[0] + nx, p[1] + ny)), S((q2[0] + nx, q2[1] + ny)),
                       S((q2[0] - nx, q2[1] - ny)), S((p[0] - nx, p[1] - ny))], colour)
    aa_poly(surf, [S(p) for p in head], colour)


# ================================================================ view
class View:
    """World metres -> screen pixels for the intersection canvas."""

    def __init__(self, rect, visible_m: float = 44.0):
        self.rect = pygame.Rect(rect)
        self.s = min(self.rect.w, self.rect.h) / (2 * (HALF + visible_m))
        self.cx, self.cy = self.rect.centerx, self.rect.centery
        self.arm_len = {"N": (self.rect.h / 2) / self.s - HALF, "S": (self.rect.h / 2) / self.s - HALF,
                        "E": (self.rect.w / 2) / self.s - HALF, "W": (self.rect.w / 2) / self.s - HALF}

    def p(self, w):
        return (self.cx + w[0] * self.s, self.cy + w[1] * self.s)

    def m(self, metres: float) -> float:
        return metres * self.s


SIDEWALK = 4.5            # m
FILLET = 6.0              # m kerb return radius at the corners
SETBACK = 2.0             # m sidewalk -> building line


# ================================================================ static scene
def _quadrant_curves(sx: int, sy: int, far: float):
    """Kerb line and building line of one quadrant (world points), with the rounded return."""
    c = (HALF + FILLET, HALF + FILLET)

    def arc(r, n=14):
        return [(c[0] - r * math.cos(math.radians(a)), c[1] - r * math.sin(math.radians(a)))
                for a in [90 * i / n for i in range(n + 1)]]
    kerb = [(far, HALF), (c[0], HALF)] + [(c[0] - FILLET * math.sin(math.radians(90 * i / 14)),
                                          c[1] - FILLET * math.cos(math.radians(90 * i / 14)))
                                         for i in range(15)] + [(HALF, c[1]), (HALF, far)]
    ri = max(0.0, FILLET - SIDEWALK)
    inner = [(HALF + SIDEWALK, far), (HALF + SIDEWALK, c[1])] + \
            [(c[0] - ri * math.cos(math.radians(90 * i / 10)), c[1] - ri * math.sin(math.radians(90 * i / 10)))
             for i in range(11)][::1] + [(c[0], HALF + SIDEWALK), (far, HALF + SIDEWALK)]
    fl = lambda pts: [(x * sx, y * sy) for x, y in pts]
    return fl(kerb), fl(inner)


def build_scene(size, view: View, seed: int = 7):
    """Static canvas background (numpy-textured asphalt, paint, kerbs, blocks)."""
    import numpy as np
    W, H = size
    s = pygame.Surface(size)
    P = lambda w: view.p(w)
    M = view.m
    far = max(view.arm_len.values()) + 40
    s.fill(T.block)
    # ---- road surface
    for a in APPROACHES:
        pts = [local_to_world(a, p) for p in ((-HALF, -HALF), (HALF, -HALF), (HALF, -HALF - far), (-HALF, -HALF - far))]
        aa_poly(s, [P(p) for p in pts], T.asphalt)
    aa_poly(s, [P(p) for p in ((-HALF, -HALF), (HALF, -HALF), (HALF, HALF), (-HALF, HALF))], T.asphalt)
    for sx in (-1, 1):
        for sy in (-1, 1):
            kerb, inner = _quadrant_curves(sx, sy, far)
            fillet = [(sx * HALF, sy * HALF)] + kerb[1:-1]
            aa_poly(s, [P(p) for p in fillet], T.asphalt)
            aa_poly(s, [P(p) for p in kerb + inner], T.sidewalk)
    # ---- asphalt grain (only where the colour is exactly asphalt)
    arr = pygame.surfarray.pixels3d(s)
    rng = np.random.default_rng(seed)
    mask = (arr[:, :, 0] == T.asphalt[0]) & (arr[:, :, 1] == T.asphalt[1]) & (arr[:, :, 2] == T.asphalt[2])
    fine = rng.normal(0, 3.2, (W, H))
    coarse_small = rng.normal(0, 3.0, (W // 24 + 2, H // 24 + 2))
    coarse = np.kron(coarse_small, np.ones((24, 24)))[:W, :H]
    k = np.ones(5) / 5
    coarse = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 0, coarse)
    coarse = np.apply_along_axis(lambda v: np.convolve(v, k, "same"), 1, coarse)
    noise = (fine + coarse)[mask]
    for ch in range(3):
        sub = arr[:, :, ch]
        sub[mask] = np.clip(sub[mask].astype(np.int16) + noise.astype(np.int16), 0, 255)
    del arr, sub
    # ---- sidewalk paving joints
    joint = T.sidewalk_joint
    for a in APPROACHES:
        L = view.arm_len[a] + 30
        y = HALF + FILLET + 1.5
        while y < L + HALF:
            for x0, x1 in ((-HALF - SIDEWALK, -HALF), (HALF, HALF + SIDEWALK)):
                thick_line(s, P(local_to_world(a, (x0, -y))), P(local_to_world(a, (x1, -y))), 1, joint)
            y += 3.0
    # ---- blocks: buildings and trees
    rng2 = random.Random(seed)
    _draw_blocks(s, view, rng2, far)
    # ---- kerbs (with a thin shadow line on the road side)
    for sx in (-1, 1):
        for sy in (-1, 1):
            kerb, _ = _quadrant_curves(sx, sy, far)
            pts = [P(p) for p in kerb]
            thick_polyline(s, pts, max(2, M(0.42)), T.kerb)
            shadow = [(x - sx * 0.32, y - sy * 0.32) for x, y in kerb]
            # the shadow follows the kerb on the road side; straight parts only
            for i in (0, len(kerb) - 2):
                thick_line(s, P(shadow[i]), P(shadow[i + 1]), max(1, M(0.18)), _mix(T.asphalt, (0, 0, 0), 0.35))
    # ---- wear: wheel tracks and oil marks on the approaches
    _draw_wear(s, view)
    # ---- paint
    _draw_paint(s, view)
    # ---- vignette
    arr = pygame.surfarray.pixels3d(s)
    xs = (np.arange(W) - view.cx) / (W * 0.62)
    ys = (np.arange(H) - view.cy) / (H * 0.62)
    r2 = xs[:, None] ** 2 + ys[None, :] ** 2
    fac = np.clip(1.0 - 0.32 * r2 ** 1.3, 0.62, 1.0)
    arr[:] = (arr * fac[:, :, None]).astype(np.uint8)
    del arr
    return s


def _draw_blocks(s, view: View, rng: random.Random, far: float):
    P, M = view.p, view.m
    a0 = HALF + SIDEWALK + SETBACK
    for sx in (-1, 1):
        for sy in (-1, 1):
            rects = []
            # corner building, then rows along both streets
            cw, ch = rng.uniform(14, 20), rng.uniform(14, 20)
            rects.append((a0, a0, cw, ch))
            x = a0 + cw + rng.uniform(2.5, 4)
            while x < far:
                w, d = rng.uniform(11, 20), rng.uniform(11, 17)
                rects.append((x, a0, w, d))
                x += w + rng.uniform(2.5, 5)
            y = a0 + ch + rng.uniform(2.5, 4)
            while y < far:
                w, d = rng.uniform(11, 17), rng.uniform(11, 20)
                rects.append((a0, y, w, d))
                y += d + rng.uniform(2.5, 5)
            for (x, y, w, d) in rects:
                x0, y0, x1, y1 = x * sx, y * sy, (x + w) * sx, (y + d) * sy
                corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                shadow = [(cx + 1.1, cy + 1.5) for cx, cy in corners]
                aa_poly(s, [P(p) for p in shadow], _mix(T.block, (0, 0, 0), 0.45))
                roof = rng.choice(T.roof)
                aa_poly(s, [P(p) for p in corners], roof)
                pygame.draw.lines(s, T.roof_edge, True, [tuple(map(round, P(p))) for p in corners], 1)
                inset = 1.2
                ix0, iy0 = x0 + inset * sx, y0 + inset * sy
                ix1, iy1 = x1 - inset * sx, y1 - inset * sy
                pygame.draw.lines(s, _mix(roof, (0, 0, 0), 0.25), True,
                                  [tuple(map(round, P(p))) for p in ((ix0, iy0), (ix1, iy0), (ix1, iy1), (ix0, iy1))], 1)
                for _ in range(rng.randint(1, 3)):          # rooftop units
                    ux = rng.uniform(min(ix0, ix1) + 1, max(ix0, ix1) - 3)
                    uy = rng.uniform(min(iy0, iy1) + 1, max(iy0, iy1) - 3)
                    uw, uh = rng.uniform(1.5, 3.0), rng.uniform(1.2, 2.4)
                    aa_poly(s, [P((ux + 0.3, uy + 0.4)), P((ux + uw + 0.3, uy + 0.4)), P((ux + uw + 0.3, uy + uh + 0.4)),
                                P((ux + 0.3, uy + uh + 0.4))], _mix(roof, (0, 0, 0), 0.3))
                    aa_poly(s, [P((ux, uy)), P((ux + uw, uy)), P((ux + uw, uy + uh)), P((ux, uy + uh))],
                            _mix(roof, (255, 255, 255), 0.07))
    # street trees on the sidewalks, away from the junction corners
    for a in APPROACHES:
        L = view.arm_len[a] + 20
        for side in (-1, 1):
            d = 26.0 + (3.0 if side > 0 else 0.0)
            while d < L + HALF:
                c = local_to_world(a, (side * (HALF + SIDEWALK * 0.5), -d))
                r = rng.uniform(2.0, 2.6)
                aa_circle(s, P((c[0] + 0.9, c[1] + 1.2)), M(r), _mix(T.sidewalk, (0, 0, 0), 0.45))
                aa_circle(s, P(c), M(r), T.tree)
                aa_circle(s, P((c[0] - r * 0.25, c[1] - r * 0.25)), M(r * 0.62), T.tree_hi)
                aa_circle(s, P((c[0] - r * 0.38, c[1] - r * 0.38)), M(r * 0.28), _mix(T.tree_hi, (255, 255, 255), 0.06))
                d += rng.uniform(10.5, 12.5)


def _draw_wear(s, view: View):
    P, M = view.p, view.m
    track = _mix(T.asphalt, (0, 0, 0), 0.16)
    oil = _mix(T.asphalt, (0, 0, 0), 0.30)
    for m in MOVES:
        a, kind = m[0], movement_type(m)
        x0 = LANE_OFFSET[kind]
        L = view.arm_len[a] + 10
        for off in (-0.78, 0.78):
            pts = [P(local_to_world(a, (x0 + off, -HALF - 0.6))), P(local_to_world(a, (x0 + off, -HALF - L)))]
            thick_line(s, pts[0], pts[1], max(1, M(0.45)), track)
        for d, rx, ry in ((2.6, 0.55, 0.9), (10.1, 0.4, 0.7), (17.6, 0.35, 0.55)):
            c = P(local_to_world(a, (x0 + 0.1, -HALF - d)))
            horiz = a in ("E", "W")
            ex, ey = (M(ry), M(rx)) if horiz else (M(rx), M(ry))
            gfx.filled_ellipse(s, round(c[0]), round(c[1]), max(1, round(ex)), max(1, round(ey)), oil + (90,))


def _draw_paint(s, view: View):
    P, M = view.p, view.m
    paint = _mix(T.paint, T.asphalt, 0.10)
    worn = _mix(T.paint, T.asphalt, 0.42)
    zebra = _mix(T.paint, T.asphalt, 0.58)
    faint = _mix(T.paint, T.asphalt, 0.80)
    cycle = _mix(T.asphalt, _hex("3a5a4a"), 0.30)
    lw = max(1, M(0.16))
    for a in APPROACHES:
        L = view.arm_len[a] + 20
        W = lambda p: P(local_to_world(a, p))

        def rect(x0, y0, x1, y1, col):
            aa_poly(s, [W((x0, y0)), W((x1, y0)), W((x1, y1)), W((x0, y1))], col)
        # cycle lanes between the edge lines and the kerbs (both directions)
        for sx in (-1, 1):
            rect(sx * (EDGE_LINE + 0.1), -HALF - 0.2, sx * (HALF - 0.05), -HALF - L, cycle)
            y = 9.0
            while y < L:                                   # cycle symbol dots
                c = W((sx * (EDGE_LINE + 1.1), -HALF - y))
                gfx.aacircle(s, round(c[0]), round(c[1]), max(2, round(M(0.45))), faint)
                y += 24.0
        # painted median: double centre line with chevron hatching near the junction
        for x in (-MEDIAN_HALF + 0.05, MEDIAN_HALF - 0.05):
            rect(x - 0.08, -HALF - 0.4, x + 0.08, -HALF - L, paint)
        y = 1.4
        while y < 20.0:                                    # chevrons pointing at the junction
            thick_polyline(s, [W((-MEDIAN_HALF + 0.12, -HALF - y - 0.7)), W((0.0, -HALF - y)),
                               W((MEDIAN_HALF - 0.12, -HALF - y - 0.7))], max(1, M(0.12)), zebra)
            y += 2.4
        # lane edge lines (solid) on both halves
        for sx in (-1, 1):
            rect(sx * EDGE_LINE - 0.07, -HALF - 0.4, sx * EDGE_LINE + 0.07, -HALF - L, paint)
        # inbound lane divider: solid over the storage/detector zone, dashed beyond
        rect(-LANE_DIVIDER - 0.07, -HALF - 0.5, -LANE_DIVIDER + 0.07, -HALF - 36.0, paint)
        y = 39.0
        while y < L:
            rect(-LANE_DIVIDER - 0.07, -HALF - y, -LANE_DIVIDER + 0.07, -HALF - y - 3.0, paint)
            y += 9.0
        # outbound lane divider: dashed
        y = 4.0
        while y < L:
            rect(LANE_DIVIDER - 0.07, -HALF - y, LANE_DIVIDER + 0.07, -HALF - y - 3.0, paint)
            y += 9.0
        # stop line across the inbound lanes (vehicles stop 0.5 m before it)
        rect(-EDGE_LINE, -HALF - 0.05, -MEDIAN_HALF, -HALF - 0.50, paint)
        # advanced stop box for cycles: short line 4 m before, over the cycle lane
        rect(-HALF + 0.1, -HALF - 3.4, -EDGE_LINE - 0.1, -HALF - 3.6, worn)
        # zebra crossing just inside the junction (kerb to kerb)
        # (stripes stop short of the neighbouring crossings, so the corners stay clear)
        x = -6.75
        while x + 0.5 <= 6.8:
            rect(x, -HALF + 0.7, x + 0.5, -HALF + 3.0, zebra)
            x += 1.0
        # lane-use arrows, twice per lane
        tr, rt = travel_dir(a), right_dir(a)
        lf = (-rt[0], -rt[1])
        for kind in ("straight", "left"):
            for d in (13.0, 44.0):
                c = W((LANE_OFFSET[kind], -HALF - d))
                draw_arrow(s, c, M(2.5), kind, tr, lf, worn, max(2, M(0.42)))
    # dotted guide lines of the left turns through the junction
    for m in MOVES:
        if movement_type(m) != "left":
            continue
        pts = [path_point(m, 25.0 * i / 40) for i in range(41)]
        for i in range(6, 36, 3):
            thick_line(s, P(pts[i]), P(pts[i + 1]), lw, faint)


# ================================================================ renderer
class Renderer:
    SIDEBAR_MIN, SIDEBAR_MAX = 388, 476

    def __init__(self, fonts: Fonts):
        self.fonts = fonts
        self._static = None
        self._static_key = None
        self.buttons: List[Tuple[object, Callable[[], None], str]] = []   # (rect, action, name)
        self.hover = (-1, -1)
        self.view: Optional[View] = None
        self.k = 1.0

    real_ids_hint: Tuple = ()

    # ------------------------------------------------------------ layout
    def layout(self, size):
        W, H = size
        k = max(0.78, min(1.3, min(W / 1600, H / 920)))
        top = round(64 * k)
        side = round(max(self.SIDEBAR_MIN, min(self.SIDEBAR_MAX, 0.27 * W)) * min(1.0, max(0.86, k)))
        canvas = pygame.Rect(0, top, W - side, H - top)
        sidebar = pygame.Rect(W - side, top, side, H - top)
        return k, pygame.Rect(0, 0, W, top), canvas, sidebar

    def _z(self, v: float) -> int:
        return max(1, round(v * self.k))

    def draw(self, surf, st, a: Optional[Snapshot], b: Optional[Snapshot], t: float):
        self.buttons = []
        k, top, canvas, side = self.layout(surf.get_size())
        self.k = k
        surf.fill(T.bg)
        view = View(canvas.inflate(-round(20 * k), -round(20 * k)))
        self.view = view
        key = (surf.get_size(), tuple(canvas))
        if self._static_key != key:
            full = build_scene(surf.get_size(), view)
            self._static = full.subsurface(canvas).copy()
            self._static_key = key
        surf.blit(self._static, canvas.topleft)
        if a is not None:
            clip = surf.get_clip()
            surf.set_clip(canvas)
            self._scene(surf, view, a, b, t)
            surf.set_clip(clip)
        self._topbar(surf, top, st, a, t)
        self._sidebar(surf, side, st, a, t)
        self._overlays(surf, canvas, st, a, t)
        if getattr(st, "custom_draft", None) is not None:
            self._custom_panel(surf, st)

    # ================================================================ dynamic scene
    def _scene(self, surf, view: View, a: Snapshot, b: Optional[Snapshot], t: float):
        P, M = view.p, view.m
        ti = int(t)
        lamps = {m: a.lamps(m, ti) for m in MOVES}
        self._active_paths(surf, view, a, lamps)
        self._signal_strips(surf, view, a, lamps)
        self._detectors(surf, view, a)
        self._vehicles(surf, view, a, b, t)
        self._heads(surf, view, a, lamps)
        self._overflow(surf, view, a)

    def _layer(self, view: View):
        r = view.rect.inflate(round(20 * self.k), round(20 * self.k))
        lay = getattr(self, "_layer_surf", None)
        if lay is None or lay.get_size() != r.size:
            lay = pygame.Surface(r.size, pygame.SRCALPHA)
            self._layer_surf = lay
        lay.fill((0, 0, 0, 0))
        return lay, r.topleft

    def _active_paths(self, surf, view: View, a: Snapshot, lamps):
        lay, (ox, oy) = self._layer(view)
        P = lambda w: (view.p(w)[0] - ox, view.p(w)[1] - oy)
        any_ = False
        for m in MOVES:
            s = a.signals[m].state
            if s not in (SignalState.GREEN, SignalState.FLASHING_GREEN, SignalState.YELLOW):
                continue
            any_ = True
            col = T.amber if s is SignalState.YELLOW else T.green
            on = s is not SignalState.FLASHING_GREEN or lamps[m].green
            alpha = 44 if on else 18
            box = 20.0 if movement_type(m) == "straight" else 25.0
            n = 30
            pts = [path_point(m, -0.2 + (box + 3.0) * i / n) for i in range(n + 1)]
            aa_poly(lay, [P(p) for p in strip_polygon(pts, 2.2)], col + (alpha,))
            aa_poly(lay, [P(p) for p in strip_polygon(pts, 0.35)], col + (min(255, alpha * 3),))
            hd = path_heading(m, box + 2.8)
            base = path_point(m, box + 2.8)
            tip = (base[0] + hd[0] * 2.4, base[1] + hd[1] * 2.4)
            nx, ny = -hd[1], hd[0]
            aa_poly(lay, [P(tip), P((base[0] + nx * 1.5, base[1] + ny * 1.5)),
                          P((base[0] - nx * 1.5, base[1] - ny * 1.5))], col + (min(255, alpha * 3),))
        if any_:
            surf.blit(lay, (ox, oy))

    def _signal_strips(self, surf, view: View, a: Snapshot, lamps):
        """A coloured strip on the junction side of each stop line: that lane's signal."""
        P, M = view.p, view.m
        for m in MOVES:
            s = a.signals[m].state
            ap, x0 = m[0], LANE_OFFSET[movement_type(m)]
            w = LANE_WIDTH / 2 - 0.45
            y0, y1 = -HALF + 0.12, -HALF + 0.62
            col = lamp_colour(s, lamps[m].green)
            quad = [P(local_to_world(ap, p)) for p in ((x0 - w, y0), (x0 + w, y0), (x0 + w, y1), (x0 - w, y1))]
            aa_poly(surf, quad, col)
            if s is SignalState.RED_YELLOW:
                half = [P(local_to_world(ap, p)) for p in ((x0, y0), (x0 + w, y0), (x0 + w, y1), (x0, y1))]
                aa_poly(surf, half, T.amber)

    def _detectors(self, surf, view: View, a: Snapshot):
        P, M = view.p, view.m
        real = set(a.hardware.real_ids) or set(self.real_ids_hint)
        hw_on = a.hardware.enabled
        for d in detector_spots(real):
            sv = a.sensors[d.movement]
            occ = sv.d1 if d.detector == "D1" else sv.d2
            faulted = d.detector in sv.faulted
            al = d.along
            ac = (-al[1], al[0])                     # across (towards the driver's right)
            c = d.centre
            if d.real:
                self._ir_sensor(surf, view, d, occ, faulted, hw_on)
            else:
                hl, hw = 0.95, LANE_WIDTH / 2 - 0.85
                pts = [P((c[0] + al[0] * sa * hl + ac[0] * sb * hw, c[1] + al[1] * sa * hl + ac[1] * sb * hw))
                       for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                l2 = d.detector == "D2" and sv.level2
                if occ:
                    aa_poly(surf, pts, T.det_on + (95,))
                ip = [tuple(map(round, p)) for p in pts]
                edge = T.amber if l2 else (T.det_on if occ else _mix(T.det_loop, T.asphalt, 0.2))
                pygame.draw.lines(surf, edge, True, ip, 2 if (l2 or occ) else 1)
                if not (l2 or occ):                      # saw-cut feeder to the kerb
                    e = (c[0] + ac[0] * (hw + 0.1), c[1] + ac[1] * (hw + 0.1))
                    k2 = (c[0] + ac[0] * (LANE_WIDTH / 2 + 0.2 if d.movement[1] == "S" else hw + 0.6),
                          c[1] + ac[1] * (LANE_WIDTH / 2 + 0.2 if d.movement[1] == "S" else hw + 0.6))
                    pygame.draw.line(surf, _mix(T.det_loop, T.asphalt, 0.35), P(e), P(k2), 1)
            if d.detector == "D1" and sv.call:
                off = LANE_WIDTH / 2 - 0.45
                dot = (c[0] - ac[0] * off, c[1] - ac[1] * off)
                glow(surf, P(dot), max(4, M(0.9)), T.text, 60)
                aa_circle(surf, P(dot), max(2, M(0.36)), T.text)

    def _ir_sensor(self, surf, view: View, d, occ, faulted, hw_on):
        """The real detector: kerb-side IR sensor and its beam across the lane (same slot)."""
        P, M = view.p, view.m
        al = d.along
        rt = (-al[1], al[0])                         # driver's right = towards the kerb for NS
        # distance from the lane centre to the kerb on the driver's right
        kd = HALF + LANE_OFFSET[movement_type(d.movement)]      # lane centre -> kerb
        c = d.centre
        dev = (c[0] + rt[0] * (kd + 0.9), c[1] + rt[1] * (kd + 0.9))
        far_end = (c[0] - rt[0] * (LANE_WIDTH / 2 - 0.2), c[1] - rt[1] * (LANE_WIDTH / 2 - 0.2))
        col = T.fault if (hw_on and faulted) else T.real
        if not hw_on:
            col = _mix(T.real, T.asphalt, 0.5)
        if occ and not faulted:
            glow(surf, P(((dev[0] + far_end[0]) / 2, (dev[1] + far_end[1]) / 2)), max(6, M(2.2)), col, 70)
            thick_line(surf, P(dev), P(far_end), max(2, M(0.28)), col)
        else:
            n = 10
            for i in range(0, n, 2):
                p0 = (dev[0] + (far_end[0] - dev[0]) * i / n, dev[1] + (far_end[1] - dev[1]) * i / n)
                p1 = (dev[0] + (far_end[0] - dev[0]) * (i + 1) / n, dev[1] + (far_end[1] - dev[1]) * (i + 1) / n)
                thick_line(surf, P(p0), P(p1), max(1, M(0.14)), col)
        # the sensor housing on the sidewalk
        hl, hw = 0.55, 0.45
        pts = [P((dev[0] + al[0] * sa * hl + rt[0] * sb * hw, dev[1] + al[1] * sa * hl + rt[1] * sb * hw))
               for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        aa_poly(surf, pts, T.housing)
        pygame.draw.lines(surf, col, True, [tuple(map(round, p)) for p in pts], 2)
        chip = P((dev[0] + rt[0] * 2.0, dev[1] + rt[1] * 2.0))
        self._chip(surf, "IR", chip, col)

    def _chip(self, surf, s, centre, colour):
        ts = self.fonts.render(s, self._z(10), T.bg, True)
        r = ts.get_rect(center=(round(centre[0]), round(centre[1])))
        rrect(surf, r.inflate(self._z(8), self._z(2)), colour, self._z(4))
        surf.blit(ts, r)

    # ------------------------------------------------------------ vehicles
    @staticmethod
    def _interp(a: Snapshot, b: Optional[Snapshot], t: float):
        if b is None or b.t_ms <= a.t_ms:
            return [(v.id, v.movement, v.u, v.state) for v in a.vehicles]
        al = max(0.0, min(1.0, (t - a.t_ms) / (b.t_ms - a.t_ms)))
        nb = {v.id: v for v in b.vehicles}
        out = []
        for v in a.vehicles:
            w = nb.get(v.id)
            u = v.u if w is None else v.u + (w.u - v.u) * al
            st = v.state if (w is None or al <= 0.5) else w.state
            out.append((v.id, v.movement, u, st))
        if al > 0.5:
            seen = {v.id for v in a.vehicles}
            out += [(w.id, w.movement, w.u, w.state) for w in b.vehicles if w.id not in seen]
        return out

    def _vehicles(self, surf, view: View, a, b, t):
        P, M = view.p, view.m
        L2, W2 = VEHICLE_LENGTH / 2, VEHICLE_WIDTH / 2
        blink = (int(t) // 380) % 2 == 0
        shade = _mix(T.asphalt, (0, 0, 0), 0.5)
        cars = []
        for vid, m, u, state in self._interp(a, b, t):
            if -u > view.arm_len[m[0]] + 6:
                continue
            (cx, cy), (hx, hy) = vehicle_pose(m, u)
            nx, ny = -hy, hx
            cars.append((vid, m, u, state, cx, cy, hx, hy, nx, ny))
        for vid, m, u, state, cx, cy, hx, hy, nx, ny in cars:             # shadows first
            sh = rounded_poly(cx + 0.28, cy + 0.42, hx, hy, nx, ny, L2 + 0.05, W2 + 0.08, 0.55, 0.45, 2)
            aa_poly(surf, [P(p) for p in sh], shade)
        for vid, m, u, state, cx, cy, hx, hy, nx, ny in cars:
            paint = vehicle_paint(vid)
            light = sum(paint) > 360

            def Q(fa, fb):                         # body frame: fa along (front +1), fb across
                return P((cx + hx * fa * L2 + nx * fb * W2, cy + hy * fa * L2 + ny * fb * W2))
            body = rounded_poly(cx, cy, hx, hy, nx, ny, L2, W2, 0.55, 0.42, 3)
            aa_poly(surf, [P(p) for p in body], paint)
            edge = _mix(paint, (0, 0, 0), 0.45)
            gfx.aapolygon(surf, [tuple(map(round, P(p))) for p in body], edge)
            glass_c = _mix(T.glass, paint, 0.12)
            roof = _mix(paint, (255, 255, 255), 0.10) if not light else _mix(paint, (0, 0, 0), 0.07)
            aa_poly(surf, [Q(0.50, -0.74), Q(0.50, 0.74), Q(0.20, 0.80), Q(0.20, -0.80)], glass_c)    # windscreen
            aa_poly(surf, [Q(0.20, -0.76), Q(0.20, 0.76), Q(-0.46, 0.76), Q(-0.46, -0.76)], roof)
            aa_poly(surf, [Q(-0.46, -0.74), Q(-0.46, 0.74), Q(-0.64, 0.68), Q(-0.64, -0.68)], glass_c)  # rear window
            hr = max(1.0, M(0.17))
            for fb in (-0.62, 0.62):
                aa_circle(surf, Q(0.96, fb), hr, _hex("f2e8c9"))
            stopped = state == "STOPPED"
            for fb in (-0.66, 0.66):
                p = Q(-0.97, fb)
                if stopped:
                    glow(surf, p, max(3, M(0.8)), (255, 50, 50), 70)
                    aa_circle(surf, p, hr + 0.4, (255, 60, 60))
                else:
                    aa_circle(surf, p, hr, _hex("7a2327"))
            if movement_type(m) == "left" and state != "EXITED" and blink:
                for fa in (0.9, -0.9):             # driver's left side = -n
                    p = Q(fa, -0.92)
                    glow(surf, p, max(3, M(0.9)), T.amber, 90)
                    aa_circle(surf, p, hr + 0.3, T.amber)

    # ------------------------------------------------------------ signal heads
    def head_layout(self, view: View):
        """Screen layout of the 8 heads: {movement: (housing rect, [(lamp, centre)] top to bottom)}.
        Every head is VERTICAL with the standard order RED (top), YELLOW, GREEN (bottom),
        whatever the approach; the two heads of an approach stand side by side on one pole."""
        M, z = view.m, self._z
        r = z(9.5)                                 # lens radius (300 mm lens)
        module = 2 * r + z(9)                      # one square lamp module
        head_len = 3 * module                      # three modules stacked
        head_w = module
        sep = z(13)                                # between the two heads (incl. backplates)
        out, poles = {}, {}
        by_app: Dict[str, list] = {}
        for h in signal_heads():
            by_app.setdefault(h.approach, []).append(h)
        for ap, heads in by_app.items():
            h0 = heads[0]
            tr, ow = h0.travel, h0.outward
            up = (-tr[0], -tr[1])
            corner = view.p(h0.corner)
            pole = (corner[0] + ow[0] * M(1.1) + up[0] * M(FILLET + 1.2),
                    corner[1] + ow[1] * M(1.1) + up[1] * M(FILLET + 1.2))
            poles[ap] = pole
            # the pair sits diagonally away from the junction, off the road
            sx = 1 if (ow[0] + up[0]) > 0 else -1
            sy = 1 if (ow[1] + up[1]) > 0 else -1
            pw, ph = 2 * head_w + sep, head_len
            px = pole[0] + z(12) if sx > 0 else pole[0] - z(12) - pw
            py = pole[1] + z(12) if sy > 0 else pole[1] - z(12) - ph
            # LEFT head on the side nearer the junction centre, STRAIGHT head outside it
            left_first = sx < 0            # pair left of the pole: its right end is nearer the junction
            order = ["straight", "left"] if left_first else ["left", "straight"]
            for i, kind in enumerate(order):
                h = next(x for x in heads if x.kind == kind)
                rect = pygame.Rect(round(px + i * (head_w + sep)), round(py), round(head_w), round(ph))
                cx = rect.centerx
                lamps_ = [(name, (cx, rect.y + module / 2 + j * module))
                          for j, name in enumerate(("red", "yellow", "green"))]
                out[h.movement] = (rect, lamps_)
        self._head_poles = poles
        self._head_r = r
        self._head_module = module
        return out

    def _heads(self, surf, view: View, a: Snapshot, lamps):
        """Israeli-style vehicle signal heads: black housing (RAL 9005) of three stacked lamp
        modules on a black aluminium backplate, 300 mm lenses under cylindrical visors, arrow
        signals lit in the lamp colour on a black mask; heads on galvanised pole brackets."""
        z = self._z
        layout = self.head_layout(view)
        r, module = self._head_r, self._head_module
        cols = {"red": T.red, "yellow": T.amber, "green": T.green}
        heads = {h.movement: h for h in signal_heads()}
        galv, galv_hi, galv_lo = _hex("7d858d"), _hex("aab2ba"), _hex("4a5057")
        ground_shadow = _mix(T.block, (0, 0, 0), 0.55)
        black, black_hi, plate = _hex("0b0c0e"), _hex("24272c"), _hex("050607")
        for ap, pole in self._head_poles.items():
            pair = [layout[m][0] for m in MOVES if m[0] == ap]
            box = pair[0].union(pair[1])
            near_x = box.x if abs(box.x - pole[0]) < abs(box.right - pole[0]) else box.right
            mid_y = box.centery
            # pole shadow, galvanised pole with cap, bracket arm to the heads, and the
            # horizontal mounting rails across the back of the pair
            thick_polyline(surf, [(pole[0] + z(3), pole[1] + z(4)), (pole[0] + z(3), mid_y + z(4)),
                                  (near_x + z(3), mid_y + z(4))], z(5), ground_shadow)
            for yb in (box.y + box.h * 0.22, box.y + box.h * 0.78):
                thick_line(surf, (box.x - z(3), yb), (box.right + z(3), yb), z(3), galv_lo)
            thick_polyline(surf, [pole, (pole[0], mid_y), (near_x, mid_y)], z(4.5), galv)
            thick_line(surf, (pole[0] - z(1.2), pole[1]), (pole[0] - z(1.2), mid_y), z(1.2), galv_hi)
            aa_circle(surf, (pole[0] + z(2), pole[1] + z(3)), z(7.5), ground_shadow)
            aa_circle(surf, pole, z(7), galv_lo)
            aa_circle(surf, pole, z(5.5), galv)
            aa_circle(surf, (pole[0] - z(1.8), pole[1] - z(1.8)), z(2.2), galv_hi)
        for m in MOVES:
            body, lamp_pos = layout[m]
            h = heads[m]
            tr, ow = h.travel, h.outward
            lf = (-ow[0], -ow[1])                    # driver's left
            # shadow on the ground, then the black backplate (slightly rounded), then the housing
            bp = body.inflate(z(12), z(12))
            shadow = bp.move(z(4), z(5))
            shs = pygame.Surface(shadow.size, pygame.SRCALPHA)
            pygame.draw.rect(shs, (0, 0, 0, 105), shs.get_rect(), border_radius=z(5))
            surf.blit(shs, shadow)
            rrect(surf, bp, plate, z(4), _hex("1c1f23"), 1)
            rrect(surf, body, black, z(3))
            pygame.draw.line(surf, black_hi, (body.x + z(3), body.y), (body.right - z(3), body.y))
            pygame.draw.line(surf, black_hi, (body.x, body.y + z(3)), (body.x, body.bottom - z(3)))
            for j in (1, 2):                                            # module seams
                yy = round(body.y + j * module)
                pygame.draw.line(surf, _hex("1a1d21"), (body.x + z(2), yy), (body.right - z(3), yy))
            lp = lamps[m]
            lit = {"red": lp.red, "yellow": lp.yellow, "green": lp.green}
            for name, lc in lamp_pos:
                col = cols[name]
                on = lit[name]
                if on:
                    glow(surf, lc, round(r * 2.3), col, 85)
                aa_circle(surf, lc, r + z(1.5), _hex("16181b"))           # lens ring
                aa_circle(surf, lc, r, _mix(col, black, 0.90 if on else 0.965))   # black mask
                arrow_col = _mix(col, (255, 255, 255), 0.18) if on else _mix(col, black, 0.86)
                draw_lens_arrow(surf, lc, r * 0.84, h.kind, tr, lf, arrow_col)
                # cylindrical visor seen from the front: a black hood over the top of the lens
                ro, ri = r + z(4), r + z(1.5)
                hood = ([(lc[0] + ro * math.cos(t), lc[1] - ro * math.sin(t))
                         for t in [math.radians(d) for d in range(-8, 189, 12)]] +
                        [(lc[0] + ri * math.cos(t), lc[1] - ri * math.sin(t) + z(0.5))
                         for t in [math.radians(d) for d in range(188, -9, -12)]])
                aa_poly(surf, hood, _hex("111316"))
                pts = [(lc[0] + ro * math.cos(t), lc[1] - ro * math.sin(t))
                       for t in [math.radians(d) for d in range(30, 151, 10)]]
                pygame.draw.lines(surf, _hex("3a4048"), False, [tuple(map(round, q)) for q in pts], 2)
            # small movement tag above the backplate
            state = a.signals[m].state
            active = state not in (SignalState.RED, SignalState.ALL_RED)
            col = lamp_colour(state, lp.green) if active else T.text2
            ts = self.fonts.render(m, z(10), col, True)
            tr_ = ts.get_rect(midbottom=(body.centerx, bp.y - z(5)))
            rrect(surf, tr_.inflate(z(8), z(2)), _hex("0d1115"), z(4), _mix(col, T.bg, 0.55) if active else T.border)
            surf.blit(ts, tr_)

    def _overflow(self, surf, view: View, a: Snapshot):
        P, z = view.p, self._z
        beyond = {m: 0 for m in MOVES}
        for v in a.vehicles:
            if v.state in ("MOVING", "STOPPED") and -v.u - VEHICLE_LENGTH > view.arm_len[v.movement[0]]:
                beyond[v.movement] += 1
        for m in MOVES:
            n = beyond[m] + a.outside[m]
            if n <= 0:
                continue
            ap = m[0]
            c = P(local_to_world(ap, (LANE_OFFSET[movement_type(m)], -HALF - view.arm_len[ap] + 3.2)))
            ts = self.fonts.render(f"+{n}", z(11), T.text, True, tabular=True)
            r = ts.get_rect(center=(round(c[0]), round(c[1])))
            box = r.inflate(z(12), z(5))
            rrect(surf, box.move(0, z(2)), (0, 0, 0), z(9))
            rrect(surf, box, _hex("1b2330"), z(9), _hex("3a4757"))
            surf.blit(ts, r)

    # ================================================================ top bar
    def _topbar(self, surf, rect, st, a, t):
        z, f = self._z, self.fonts
        pygame.draw.rect(surf, T.bar, rect)
        pygame.draw.line(surf, T.border, (0, rect.bottom - 1), (rect.right, rect.bottom - 1))
        cy = rect.centery
        # logo mark
        lg = pygame.Rect(0, 0, z(36), z(36))
        lg.midleft = (rect.x + z(20), cy)
        rrect(surf, lg, T.raised, z(10), T.border_hi)
        for i, col in enumerate((T.red, T.amber, T.green)):
            aa_circle(surf, (lg.centerx, lg.y + z(9) + i * z(9)), z(3.2), col if i != 1 else _mix(col, T.raised, 0.6))
        x = lg.right + z(14)
        text(surf, f, "Smart Traffic Lights", (x, cy + z(1)), z(19), T.text, True, "bottomleft")
        text(surf, f, "Eight-movement signalised intersection  ·  sensor-actuated control",
             (x, cy + z(3)), z(12), T.muted, anchor="topleft")
        # clock
        rx = rect.right - z(20)
        clock = pygame.Rect(0, 0, z(168), z(40))
        clock.midright = (rx, cy)
        rrect(surf, clock, T.inset, z(10), T.border)
        text(surf, f, "SIM", (clock.x + z(14), cy), z(10), T.muted, True, "midleft", spacing=1)
        text(surf, f, fmt_clock(t), (clock.right - z(14), cy), z(22), T.text, True, "midright", tabular=True)
        rx = clock.x - z(12)
        # status pills
        phase = run_phase(st.config, int(t)) if a is not None else "READY"
        if st.error:
            state = ("ERROR", T.red)
        elif st.finished:
            state = ("COMPLETE", T.accent)
        elif not st.running:
            state = ("PAUSED", T.amber)
        else:
            state = ("RUNNING", T.green)
        custom = getattr(st.config, "mode", "official") == "custom"
        pills = [(state[0], state[1], True), (phase, T.text2, False)]
        pills.append((CUSTOM_BADGE, T.custom, False) if custom else ("OFFICIAL · LOCKED", T.muted, False))
        for label, col, dot in pills:
            ts = f.render(label, z(11), col, True, spacing=1)
            w = ts.get_width() + z(24) + (z(12) if dot else 0)
            pill = pygame.Rect(0, 0, w, z(28))
            pill.midright = (rx, cy)
            rrect(surf, pill, _mix(col, T.bar, 0.88), z(14), _mix(col, T.bar, 0.6))
            tx = pill.x + z(12)
            if dot:
                aa_circle(surf, (tx + z(3), cy), z(3.5), col)
                tx += z(12)
            surf.blit(ts, ts.get_rect(midleft=(tx, cy)))
            rx = pill.x - z(8)
        # controller / scenario
        rx -= z(10)
        pygame.draw.line(surf, T.border, (rx, cy - z(16)), (rx, cy + z(16)))
        rx -= z(14)
        ctrl = f"Fixed-time · {st.config.controller}"
        if st.config.controller == "Smart":
            ctrl = f"Smart actuated · {smart_label(st.config.smart_params)}"
        sid, sc = scenario_title(st.config)
        text(surf, f, ctrl, (rx, cy + z(1)), z(14), T.text, True, "bottomright")
        text(surf, f, f"{sid} · {sc} · seed {st.config.seed}", (rx, cy + z(3)), z(12), T.muted,
             anchor="topright")

    # ================================================================ widgets
    def _hot(self, rect) -> bool:
        return rect.collidepoint(self.hover)

    def _register(self, rect, action, name, enabled=True):
        if enabled:
            self.buttons.append((rect, action, name))

    def segmented(self, surf, rect, items, current, on_pick, *, names=None, enabled=None, size=None):
        z, f = self._z, self.fonts
        rrect(surf, rect, T.inset, z(10), T.border)
        n = len(items)
        inner = rect.inflate(-z(6), -z(6))
        w = inner.w / n
        for i, (value, label) in enumerate(items):
            seg = pygame.Rect(round(inner.x + i * w), inner.y, round(w), inner.h)
            ok = enabled is None or enabled(value)
            sel = value == current
            if sel:
                rrect(surf, seg, T.raised, z(8), T.border_hi)
                pygame.draw.line(surf, T.accent, (seg.x + z(10), seg.bottom - z(2)), (seg.right - z(10), seg.bottom - z(2)), 2)
            elif ok and self._hot(seg):
                rrect(surf, seg, _mix(T.inset, (255, 255, 255), 0.04), z(8))
            col = T.text if sel else (T.text2 if ok else T.faint)
            text(surf, f, label, seg.center, size or z(13), col, True, "center")
            if i and not sel and items[i - 1][0] != current:
                pygame.draw.line(surf, T.border, (seg.x, seg.y + z(8)), (seg.x, seg.bottom - z(8)))
            self._register(seg, (lambda v=value: on_pick(v)), names(value, label) if names else label, ok)

    def icon_button(self, surf, rect, icon, action, name, enabled=True, primary=False, label=None):
        z, f = self._z, self.fonts
        hot = enabled and self._hot(rect)
        if primary:
            bg = T.accent if enabled else T.raised
            if hot:
                bg = _mix(bg, (255, 255, 255), 0.12)
            fg = _hex("06101c") if enabled else T.muted
            rrect(surf, rect, bg, z(10))
        else:
            bg = T.raised if not hot else _mix(T.raised, (255, 255, 255), 0.05)
            fg = T.text if enabled else T.faint
            rrect(surf, rect, bg, z(10), T.border_hi)
        iw = z(12)
        if label:
            ts = f.render(label, z(14), fg, True)
            total = iw + z(10) + ts.get_width()
            x0 = rect.centerx - total / 2
            icon(surf, (x0 + iw / 2, rect.centery), iw, fg)
            surf.blit(ts, ts.get_rect(midleft=(round(x0 + iw + z(10)), rect.centery)))
        else:
            icon(surf, rect.center, iw, fg)
        self._register(rect, action, name, enabled)

    def _card(self, surf, rect, title, right=None):
        z, f = self._z, self.fonts
        rrect(surf, rect, T.panel, z(14), T.border)
        pygame.draw.line(surf, (30, 38, 48), (rect.x + z(14), rect.y + 1), (rect.right - z(14), rect.y + 1))
        text(surf, f, title.upper(), (rect.x + z(18), rect.y + z(16)), z(11), T.text2, True, spacing=z(1.4))
        if right:
            text(surf, f, right, (rect.right - z(18), rect.y + z(15)), z(11), T.muted, anchor="topright")
        return rect.y + z(42)

    # ================================================================ sidebar
    def _sidebar(self, surf, rect, st, a, t):
        z = self._z
        pygame.draw.rect(surf, T.bg, rect)
        pad = z(14)
        x, w = rect.x + z(4), rect.w - z(4) - pad
        y = rect.y + pad
        y = self._controls(surf, pygame.Rect(x, y, w, 0), st) + z(12)
        y = self._metrics(surf, pygame.Rect(x, y, w, 0), st, a) + z(12)
        self._controller(surf, pygame.Rect(x, y, w, rect.bottom - pad - y), st, a, t)

    def _controls(self, surf, r, st):
        z, f = self._z, self.fonts
        bh, gap = z(38), z(10)
        r.h = z(42) + 4 * bh + 3 * gap + z(18)
        y = self._card(surf, r, "Simulation", "Space · R · 1–6")
        x0, w = r.x + z(16), r.w - z(32)
        # controller
        ctrls = controllers_for(st.config.scenario)
        self.segmented(surf, pygame.Rect(x0, y, w, bh), [(c, c) for c in ctrls], st.config.controller,
                       st.set_controller)
        y += bh + gap
        # scenario selector
        sel = pygame.Rect(x0, y, w, bh)
        rrect(surf, sel, T.inset, z(10), T.border)
        aw = z(34)
        self.icon_button(surf, pygame.Rect(sel.x + z(3), y + z(3), aw, bh - z(6)), _icon_chev_left,
                         st.prev_scenario, "scenario-")
        self.icon_button(surf, pygame.Rect(sel.right - z(3) - aw, y + z(3), aw, bh - z(6)), _icon_chev_right,
                         st.next_scenario, "scenario+")
        mid = pygame.Rect(sel.x + aw + z(10), y, sel.w - 2 * aw - z(20), bh)
        custom = getattr(st.config, "mode", "official") == "custom"
        sid, desc = scenario_title(st.config)
        badge = f.render(sid, z(12), T.custom if custom else T.accent, True)
        if custom:
            desc = "Custom… (edit)"
        size = z(13)
        while f.render(desc, size, T.text).get_width() > mid.w - badge.get_width() - z(26) and size > z(10):
            size -= 1
        dsurf = f.render(desc, size, T.text)
        total = badge.get_width() + z(18) + dsurf.get_width()
        bx = mid.centerx - total / 2
        br = badge.get_rect(midleft=(round(bx + z(6)), mid.centery))
        rrect(surf, br.inflate(z(12), z(4)), _mix(T.custom, T.inset, 0.8) if custom else T.accent_soft, z(6))
        surf.blit(badge, br)
        surf.blit(dsurf, dsurf.get_rect(midleft=(round(br.right + z(12)), mid.centery)))
        if custom and hasattr(st, "open_custom"):
            self._register(mid, st.open_custom, "custom:edit")
        y += bh + gap
        # seed stepper + hardware switch
        step = pygame.Rect(x0, y, z(150), bh)
        rrect(surf, step, T.inset, z(10), T.border)
        text(surf, f, "Seed", (step.x + z(12), step.centery), z(12), T.muted, anchor="midleft")
        sb = z(28)
        self.icon_button(surf, pygame.Rect(step.x + z(48), y + z(5), sb, bh - z(10)), _icon_minus, st.prev_seed, "seed-")
        text(surf, f, str(st.config.seed), (step.x + z(48) + sb + z(18), step.centery), z(14), T.text, True,
             "center", tabular=True)
        self.icon_button(surf, pygame.Rect(step.right - z(5) - sb, y + z(5), sb, bh - z(10)), _icon_plus,
                         st.next_seed, "seed+")
        hw = pygame.Rect(step.right + z(10), y, x0 + w - step.right - z(10), bh)
        on = bool(st.hardware)
        hot = self._hot(hw)
        rrect(surf, hw, _mix(T.inset, (255, 255, 255), 0.03) if hot else T.inset, z(10),
              _mix(T.real, T.bg, 0.45) if on else T.border)
        track = pygame.Rect(0, 0, z(34), z(18))
        track.midleft = (hw.x + z(12), hw.centery)
        rrect(surf, track, _mix(T.real, T.bg, 0.35) if on else T.raised, z(9), T.border_hi)
        knob = (track.right - z(9) if on else track.x + z(9), track.centery)
        aa_circle(surf, knob, z(6.5), T.text if on else T.text2)
        text(surf, f, "ESP32 hardware", (track.right + z(10), hw.centery - z(1)), z(12), T.text, True, "bottomleft")
        text(surf, f, "on · real IR + head" if on else "off · all simulated", (track.right + z(10), hw.centery + z(1)),
             z(11), T.muted, anchor="topleft")
        self._register(hw, st.toggle_hardware, "hardware")
        y += bh + gap
        # run / reset / speed
        rw = z(104)
        running = st.running
        self.icon_button(surf, pygame.Rect(x0, y, rw, bh), _icon_pause if running else _icon_play, st.toggle_run,
                         "run", enabled=not st.finished and not st.error, primary=True,
                         label="Pause" if running else "Run")
        self.icon_button(surf, pygame.Rect(x0 + rw + z(8), y, bh, bh), _icon_reset, st.reset, "reset")
        sx = x0 + rw + z(8) + bh + z(8)
        items = [(sp, "Max" if sp is None else f"{sp:g}×") for sp in SPEEDS]
        self.segmented(surf, pygame.Rect(sx, y, x0 + w - sx, bh), items, st.speed, st.set_speed,
                       names=lambda v, l: f"speed:{l}",
                       enabled=lambda v: not st.hardware or v == HARDWARE_SPEED, size=z(12))
        return r.bottom

    def _metrics(self, surf, r, st, a):
        z, f = self._z, self.fonts
        th, gap = z(62), z(8)
        r.h = z(42) + 2 * th + gap + z(16)
        y = self._card(surf, r, "Live", "measured from t = 0")
        x0, w = r.x + z(16), r.w - z(32)
        m = a.metrics if a is not None else None

        def val(x, fmt):
            return ("–", "") if x is None else fmt(x)
        tiles = [
            ("IN SYSTEM", val(m and m.in_system, lambda v: (f"{v}", "veh"))),
            ("QUEUED", val(m and m.queue, lambda v: (f"{v}", "veh"))),
            ("CROSSED", val(m and m.crossed, lambda v: (f"{v}", "veh"))),
            ("FLOW", val(m and m.flow_veh_h, lambda v: (f"{v:.0f}", "veh/h"))),
            ("AVG DELAY", val(m and m.avg_delay_s, lambda v: (f"{v:.1f}", "s"))),
            ("LONGEST WAIT", val(m and m.longest_wait_s, lambda v: (f"{v:.0f}", "s"))),
        ]
        tw = (w - 2 * gap) / 3
        for i, (lab, (v, unit)) in enumerate(tiles):
            tr = pygame.Rect(round(x0 + (i % 3) * (tw + gap)), y + (i // 3) * (th + gap), round(tw), th)
            rrect(surf, tr, T.inset, z(10), T.border)
            text(surf, f, lab, (tr.x + z(12), tr.y + z(10)), z(10), T.muted, True, spacing=1)
            vr = text(surf, f, v, (tr.x + z(12), tr.bottom - z(8)), z(21), T.text, True, "bottomleft", tabular=True)
            if unit and v != "–":
                text(surf, f, unit, (vr.right + z(4), vr.bottom - z(4)), z(11), T.text2, anchor="bottomleft")
        return r.bottom

    def _controller(self, surf, r, st, a, t):
        z, f = self._z, self.fonts
        if r.h < z(120):
            return
        smart = st.config.controller == "Smart"
        title = "Smart controller" if smart else "Fixed-time controller"
        right = "actuated · sensor input" if smart else f"plan {st.config.controller} · no sensor input"
        y = self._card(surf, r, title, right)
        x0, w = r.x + z(16), r.w - z(32)
        if a is None:
            return
        if smart and a.smart is not None:
            y = self._smart_block(surf, x0, y, w, a)
        elif a.fixed is not None:
            y = self._fixed_block(surf, x0, y, w, a, t)
        y += z(10)
        # all 8 movement rows always fit: the row height adapts to the space left
        avail = r.bottom - z(14) - y - z(28)
        rh = int(max(z(20), min(z(28), avail / 8)))
        y = self._table(surf, x0, y, w, a, t, smart, r.bottom - z(8), rh)
        if smart and a.smart is not None and r.bottom - y > z(70):
            self._decisions(surf, x0, y + z(12), w, a, r.bottom - z(14))

    def _wrap(self, surf, s, x, y, w, size, colour, bold=False, lead=None):
        f = self.fonts
        words, line, first = s.split(" "), "", True
        lh = round(size * 1.38)
        indent = z_ = 0
        if lead is not None:
            aa_circle(surf, (x + self._z(3), y + lh / 2), self._z(2.5), lead)
            indent = self._z(14)
        for word in words:
            trial = (line + " " + word).strip()
            if f.render(trial, size, colour, bold).get_width() > w - indent and line:
                text(surf, f, line, (x + indent, y), size, colour, bold)
                y += lh
                line = word
            else:
                line = trial
        if line:
            text(surf, f, line, (x + indent, y), size, colour, bold)
            y += lh
        return y

    def _smart_block(self, surf, x0, y, w, a: Snapshot):
        z, f = self._z, self.fonts
        sm = a.smart
        text(surf, f, "TARGET", (x0, y + z(13)), z(10), T.muted, True, "midleft", spacing=1)
        cx = x0 + z(62)
        if sm.target:
            for m in sm.target:
                col = T.amber if m == sm.hold else T.accent
                ts = f.render(m, z(13), T.text, True)
                box = pygame.Rect(cx, y, ts.get_width() + z(22), z(26))
                rrect(surf, box, _mix(col, T.panel, 0.8), z(13), _mix(col, T.panel, 0.25))
                surf.blit(ts, ts.get_rect(center=box.center))
                cx = box.right + z(6)
        else:
            text(surf, f, "none — no waiting calls", (cx, y + z(13)), z(12), T.muted, anchor="midleft")
        if sm.hold:
            ts = f.render(f"HOLD {sm.hold}", z(10), T.amber, True, spacing=1)
            box = ts.get_rect(midright=(x0 + w - z(8), y + z(13))).inflate(z(14), z(8))
            rrect(surf, box, _mix(T.amber, T.panel, 0.85), z(8), _mix(T.amber, T.panel, 0.4))
            surf.blit(ts, ts.get_rect(center=box.center))
        y += z(36)
        for i, line in enumerate(sm.why[:3]):
            y = self._wrap(surf, line, x0, y, w, z(13), T.text if i == 0 else T.text2,
                           lead=T.accent if i == 0 else T.faint) + z(1)
        return y

    def _fixed_block(self, surf, x0, y, w, a: Snapshot, t: float):
        z, f = self._z, self.fonts
        fx = a.fixed
        cur = fx.current
        total = sum(d for _, _, d in fx.phases)
        text(surf, f, f"{fx.plan}", (x0, y), z(13), T.text, True)
        text(surf, f, f"cycle {fx.cycle_ms / 1000:g} s", (x0 + w, y + z(1)), z(12), T.muted, anchor="topright")
        bar = pygame.Rect(x0, y + z(24), w, z(30))
        rrect(surf, bar, T.inset, z(8), T.border)
        inner = bar.inflate(-z(4), -z(4))
        xs, acc, pos = inner.x, 0, None
        for name, moves, dur in fx.phases:
            pw = inner.w * dur / total
            seg = pygame.Rect(round(xs), inner.y, max(1, round(pw) - z(3)), inner.h)
            is_cur = cur is not None and cur.phase == name and t < cur.next_start_ms
            if is_cur:
                rrect(surf, seg, T.accent_soft, z(6), _mix(T.accent, T.panel, 0.3))
                pos = acc + max(0, t - cur.start_ms)
            else:
                rrect(surf, seg, T.raised, z(6))
            text(surf, f, name, seg.center, z(12), T.text if is_cur else T.text2, True, "center")
            cap = "+".join(moves)
            if f.render(cap, z(10), T.muted).get_width() <= seg.w:
                text(surf, f, cap, (seg.centerx, bar.bottom + z(4)), z(10), T.text2 if is_cur else T.muted,
                     anchor="midtop")
            acc += dur
            xs += pw
        if pos is not None:
            px = round(inner.x + inner.w * min(1.0, pos / total))
            pygame.draw.line(surf, T.text, (px, bar.y - z(3)), (px, bar.bottom + z(2)), 2)
            aa_poly(surf, [(px - z(4), bar.y - z(7)), (px + z(4), bar.y - z(7)), (px, bar.y - z(2))], T.text)
        y = bar.bottom + z(22)
        if cur is None:
            line = "Waiting for the first phase"
        else:
            ph = " + ".join(cur.movements)
            if t < cur.green_ms:
                line = f"Phase {cur.phase} ({ph}) · red + yellow · green in {(cur.green_ms - t) / 1000:.1f} s"
            elif t < cur.end_green_ms:
                line = f"Phase {cur.phase} ({ph}) · green · ends in {(cur.end_green_ms - t) / 1000:.1f} s"
            elif t < cur.yellow_ms:
                line = f"Phase {cur.phase} ({ph}) · flashing green · {(cur.yellow_ms - t) / 1000:.1f} s"
            else:
                names = [p[0] for p in fx.phases]
                nxt = fx.phases[(names.index(cur.phase) + 1) % len(names)]
                line = (f"Clearance (yellow / all-red) · next {nxt[0]} ({' + '.join(nxt[1])}) "
                        f"in {(cur.next_start_ms - t) / 1000:.1f} s")
        y = self._wrap(surf, line, x0, y, w, z(13), T.text, lead=T.accent)
        return self._wrap(surf, "Fixed timing: the calls and queues below are shown for comparison only.",
                          x0, y + z(2), w, z(11), T.muted)

    def _table(self, surf, x0, y, w, a: Snapshot, t: float, smart: bool, bottom: int, rh: int = 0):
        z, f = self._z, self.fonts
        rh = rh or z(27)
        if smart:
            cols = {"Signal": 0.0, "State": 0.16, "Queue": 0.56, "Call": 0.635, "Qhat": 0.725, "Wait": 0.86, "P": 1.0}
        else:
            cols = {"Signal": 0.0, "State": 0.17, "Queue": 0.64, "Call": 0.735, "Qhat": 0.845, "Wait": 1.0}
        X = {k: x0 + v * w for k, v in cols.items()}
        hdr = pygame.Rect(x0 - z(8), y, w + z(16), z(24))
        rrect(surf, hdr, T.inset, z(6))
        hy = hdr.centery
        for name, px in X.items():
            label = name.upper()
            if name in ("Signal", "State"):
                text(surf, f, label, (px + (z(4) if name == "Signal" else 0), hy), z(10), T.muted, True, "midleft",
                     spacing=1)
            elif name == "Call":
                text(surf, f, label, (px, hy), z(10), T.muted, True, "center", spacing=1)
            elif name == "Qhat":
                rr = text(surf, f, "Q", (px, hy + z(1)), z(11), T.muted, True, "midright")
                cx = rr.centerx
                pygame.draw.lines(surf, T.muted, False, [(cx - z(3), rr.y + z(1)), (cx, rr.y - z(2)),
                                                         (cx + z(3), rr.y + z(1))], 1)
            else:
                text(surf, f, label, (px, hy), z(10), T.muted, True, "midright", spacing=1)
        y = hdr.bottom + z(4)
        ti = int(t)
        tgt = set(a.smart.target or ()) if (smart and a.smart) else set()
        for i, m in enumerate(MOVES):
            if y + rh > bottom:
                break
            sv, sn = a.signals[m], a.sensors[m]
            lp = a.lamps(m, ti)
            row = pygame.Rect(x0 - z(8), y, w + z(16), rh)
            if m in tgt:
                rrect(surf, row, _mix(T.accent, T.panel, 0.9), z(6))
                pygame.draw.rect(surf, T.accent, (row.x, row.y + z(5), z(3), row.h - z(10)), border_radius=2)
            elif i % 2:
                rrect(surf, row, _mix(T.panel, (255, 255, 255), 0.015), z(6))
            cy = y + rh / 2
            col = lamp_colour(sv.state, lp.green)
            dim = sv.state is SignalState.FLASHING_GREEN and not lp.green
            glow_ok = sv.state not in (SignalState.RED, SignalState.ALL_RED) and not dim
            if glow_ok:
                glow(surf, (x0 + z(6), cy), z(9), col, 70)
            aa_circle(surf, (x0 + z(6), cy), z(4.5), col)
            if sv.state is SignalState.RED_YELLOW:
                aa_circle(surf, (x0 + z(6), cy), z(2.2), T.amber)
            text(surf, f, m, (x0 + z(17), cy), z(13), T.text, True, "midleft")
            # state pill
            el = max(0, ti - sv.since_ms) / 1000
            label = STATE_SHORT[sv.state]
            if sv.state in (SignalState.GREEN, SignalState.RED):
                timer = f"{el:.0f}s"
            elif sv.remaining_ms is not None:
                timer = f"{max(0.0, (sv.remaining_ms - (ti - a.t_ms)) / 1000):.1f}s"
            else:
                timer = ""
            red = sv.state in (SignalState.RED, SignalState.ALL_RED)
            pcol = T.text2 if red else col
            ls = f.render(label, z(10), pcol, True, spacing=1)
            pill = ls.get_rect(midleft=(round(X["State"] + z(8)), round(cy)))
            pr = pill.inflate(z(14), z(8))
            pr.x = round(X["State"])
            rrect(surf, pr, _mix(pcol, T.panel, 0.9 if red else 0.84), z(6))
            surf.blit(ls, ls.get_rect(midleft=(pr.x + z(7), round(cy))))
            text(surf, f, timer, (pr.right + z(7), cy), z(12), T.muted if red else T.text2, anchor="midleft",
                 tabular=True)
            qn = a.queue[m] + a.outside[m]
            text(surf, f, str(qn) if qn else "·", (X["Queue"], cy), z(13), T.text if qn else T.faint,
                 anchor="midright", tabular=True)
            ccx = X["Call"]
            if sn.call:
                aa_circle(surf, (ccx, cy), z(4.5), T.amber if sn.level2 else T.text)
            elif sn.faulted:
                text(surf, f, "!", (ccx, cy), z(13), T.fault, True, "center")
            else:
                aa_circle(surf, (ccx, cy), z(2), T.faint)
            text(surf, f, str(sn.qhat) if sn.call else "·", (X["Qhat"], cy), z(13), T.text if sn.call else T.faint,
                 anchor="midright", tabular=True)
            wait = None
            if sv.state is SignalState.RED and sn.call and sn.call_since_ms is not None:
                wait = (ti - sn.call_since_ms) / 1000
            if smart and a.smart and m in a.smart.wait_ms:
                wait = (a.smart.wait_ms[m] + (ti - a.t_ms)) / 1000
            hot = wait is not None and smart and a.smart and wait * 1000 >= a.smart.w_max_ms
            text(surf, f, "·" if wait is None else f"{wait:.0f}s", (X["Wait"], cy), z(12),
                 T.amber if hot else (T.text if wait is not None else T.faint), hot, "midright", tabular=True)
            if smart and a.smart:
                p = a.smart.priority.get(m)
                s = "·" if p is None else f"{p:.1f}"
                if m in a.smart.recall:
                    s = "recall" if p is None else f"R {p:.1f}"
                text(surf, f, s, (X["P"], cy), z(12), T.text if p is not None else T.faint, True, "midright",
                     tabular=True)
            y += rh
        return y

    def _decisions(self, surf, x0, y, w, a: Snapshot, bottom):
        z, f = self._z, self.fonts
        pygame.draw.line(surf, T.border, (x0, y - z(4)), (x0 + w, y - z(4)))
        text(surf, f, "RECENT DECISIONS", (x0, y + z(4)), z(10), T.muted, True, spacing=1)
        y += z(24)
        names = {"gap": "gap-out", "max": "max-out", "hold": "hold"}
        if not a.smart.decisions:
            text(surf, f, "No switches yet", (x0, y), z(12), T.faint)
        for d in reversed(a.smart.decisions):
            if y + z(18) > bottom:
                break
            text(surf, f, fmt_clock(d.time_ms), (x0, y), z(12), T.muted, tabular=True)
            ended = ", ".join(f"{g} {names.get(r, r)}" for g, r in d.stopped) or "—"
            text(surf, f, f"{ended}  →  {' + '.join(d.to_start)}", (x0 + z(62), y), z(12), T.text2)
            y += z(20)

    # ================================================================ canvas overlays
    def _overlays(self, surf, canvas, st, a, t):
        z = self._z
        m = z(16)
        lw, lh = z(262), z(148)
        self._legend(surf, pygame.Rect(canvas.right - m - lw, canvas.bottom - m - lh, lw, lh))
        hw = a.hardware if a is not None else None
        self._hardware(surf, pygame.Rect(canvas.x + m, canvas.bottom - m - z(132), z(300), z(132)), st, hw)
        self._scale(surf, canvas)
        bx, by = canvas.x + m, canvas.y + m
        if st.error:
            self._banner(surf, (bx, by), "The run stopped with an error", st.error, T.red)
        elif st.finished and st.result is not None:
            ir = st.result.intersection
            lines = [f"Mean control delay {ir.delay_mean_s:.1f} s  ·  P95 {ir.delay_p95_s:.1f} s"
                     if ir.delay_mean_s is not None else "No measured vehicles",
                     f"Throughput {ir.throughput_veh_h:.0f} veh/h  ·  {ir.crossed} crossed  ·  "
                     f"mean queue {ir.queue_mean_per_movement_veh:.2f} veh/movement"]
            if getattr(st.config, "mode", "official") == "custom":
                self._banner(surf, (bx, by), f"Run complete · {CUSTOM_BADGE} (not an official result)",
                             "\n".join(lines), T.custom)
            else:
                self._banner(surf, (bx, by), "Run complete · Stage 12 statistics", "\n".join(lines), T.accent)
        elif a is not None and not st.running and a.step <= 1:
            if getattr(st.config, "mode", "official") == "custom":
                self._banner(surf, (bx, by), f"{CUSTOM_BADGE} ready",
                             "Custom demand · not an official result. Press Run (Space) to start.\n"
                             f"Traffic rules: {VEHICLE_POLICY_LABEL}", T.custom)
            else:
                self._banner(surf, (bx, by), "Ready", "Press Run (Space) to start. All 8 signals start red.\n"
                             f"Traffic rules: {VEHICLE_POLICY_LABEL}", T.accent)
        if st.message:
            self._banner(surf, (bx, by + z(96)), "Hardware", st.message, T.amber)

    def _scale(self, surf, canvas):
        z, f = self._z, self.fonts
        view = self.view
        if view is None:
            return
        px10 = view.m(10.0)
        w = round(px10 + z(76))
        r = pygame.Rect(canvas.right - z(16) - w, canvas.y + z(16), w, z(44))
        glass(surf, r, z(10))
        nx = r.x + z(22)
        cy = r.centery
        aa_poly(surf, [(nx, cy - z(11)), (nx + z(7), cy + z(7)), (nx, cy + z(2)), (nx - z(7), cy + z(7))], T.text)
        text(surf, f, "N", (nx + z(14), cy - z(10)), z(10), T.text2, True)
        x1 = r.right - z(14)
        x0 = x1 - px10
        yb = cy + z(6)
        pygame.draw.line(surf, T.text2, (x0, yb), (x1, yb), 2)
        for xx in (x0, x1):
            pygame.draw.line(surf, T.text2, (xx, yb - z(5)), (xx, yb + z(1)), 2)
        text(surf, f, "10 m", ((x0 + x1) / 2, yb - z(4)), z(10), T.text2, anchor="midbottom")

    def _banner(self, surf, pos, title, body, colour):
        z, f = self._z, self.fonts
        lines = body.split("\n")
        w = max([f.render(l, z(13), T.text).get_width() for l in lines] +
                [f.render(title, z(14), T.text, True).get_width()]) + z(40)
        h = z(44) + len(lines) * z(19)
        r = pygame.Rect(pos[0], pos[1], min(w, z(640)), h)
        glass(surf, r, z(12))
        pygame.draw.rect(surf, colour, (r.x + z(1), r.y + z(12), z(3), r.h - z(24)), border_radius=2)
        text(surf, f, title, (r.x + z(18), r.y + z(12)), z(14), T.text, True)
        yy = r.y + z(36)
        for l in lines:
            text(surf, f, l, (r.x + z(18), yy), z(13), T.text2)
            yy += z(19)

    def _legend(self, surf, r):
        z, f = self._z, self.fonts
        glass(surf, r, z(12))
        x, y = r.x + z(16), r.y + z(14)
        text(surf, f, "LEGEND", (x, y), z(10), T.muted, True, spacing=1)
        y += z(22)
        col2 = r.x + r.w // 2 + z(2)
        rowh = z(22)

        def car(cx, cy, blink):
            body = pygame.Rect(0, 0, z(20), z(10))
            body.center = (cx, cy)
            rrect(surf, body, _hex("b8bfc7"), z(4))
            pygame.draw.rect(surf, T.glass, (body.right - z(7), body.y + z(2), z(3), body.h - z(4)))
            if blink:
                glow(surf, (body.x + z(2), body.y), z(5), T.amber, 90)
                aa_circle(surf, (body.x + z(2), body.y + z(1)), z(2), T.amber)
                aa_circle(surf, (body.right - z(2), body.y + z(1)), z(2), T.amber)
        items = [
            (lambda cx, cy: car(cx, cy, False), "Vehicle"),
            (lambda cx, cy: car(cx, cy, True), "Turning left"),
            (lambda cx, cy: pygame.draw.rect(surf, T.faint, pygame.Rect(cx - z(8), cy - z(5), z(16), z(10)), 1), "Loop, clear"),
            (lambda cx, cy: (pygame.draw.rect(surf, _mix(T.det_on, T.panel, 0.5), pygame.Rect(cx - z(8), cy - z(5), z(16), z(10))),
                             pygame.draw.rect(surf, T.det_on, pygame.Rect(cx - z(8), cy - z(5), z(16), z(10)), 2)), "Occupied"),
            (lambda cx, cy: aa_circle(surf, (cx, cy), z(4), T.text), "Call"),
            (lambda cx, cy: pygame.draw.rect(surf, T.amber, pygame.Rect(cx - z(8), cy - z(5), z(16), z(10)), 2), "Level 2"),
        ]
        for i, (icon, lab) in enumerate(items):
            xx = x if i % 2 == 0 else col2
            yy = y + (i // 2) * rowh
            icon(xx + z(10), yy + z(8))
            text(surf, f, lab, (xx + z(28), yy + z(8)), z(12), T.text2, anchor="midleft")
        y += 3 * rowh + z(4)
        self._chip(surf, "IR", (x + z(10), y + z(6)), T.real)
        text(surf, f, "Real IR detector (in its own D1 slot)", (x + z(28), y + z(6)), z(11), T.text2, anchor="midleft")

    # ================================================================ Custom / Demo panel (Stage 14b)
    def text_button(self, surf, rect, label, action, name, enabled=True, primary=False, size=None):
        z, f = self._z, self.fonts
        hot = enabled and self._hot(rect)
        if primary:
            bg = T.accent if enabled else T.raised
            rrect(surf, rect, _mix(bg, (255, 255, 255), 0.12) if hot else bg, z(10))
            fg = _hex("06101c") if enabled else T.muted
        else:
            rrect(surf, rect, _mix(T.raised, (255, 255, 255), 0.05) if hot else T.raised, z(10), T.border_hi)
            fg = T.text if enabled else T.faint
        text(surf, f, label, rect.center, size or z(13), fg, True, "center")
        self._register(rect, action, name, enabled)

    def _stepper(self, surf, rect, value, focused, on_minus, on_plus, on_focus, name, enabled=True):
        z, f = self._z, self.fonts
        rrect(surf, rect, T.inset, z(8), T.accent if focused else T.border)
        bw = rect.h - z(8)
        minus = pygame.Rect(rect.x + z(4), rect.y + z(4), bw, rect.h - z(8))
        plus = pygame.Rect(rect.right - z(4) - bw, rect.y + z(4), bw, rect.h - z(8))
        self.icon_button(surf, minus, _icon_minus, on_minus, f"{name}-", enabled)
        self.icon_button(surf, plus, _icon_plus, on_plus, f"{name}+", enabled)
        mid = pygame.Rect(minus.right + z(4), rect.y, plus.x - minus.right - z(8), rect.h)
        col = T.text if enabled else T.faint
        r = text(surf, f, value, mid.center, z(14), col, True, "center", tabular=True)
        if focused:
            pygame.draw.line(surf, T.accent, (r.right + z(2), r.y + z(3)), (r.right + z(2), r.bottom - z(3)), 2)
        self._register(mid, on_focus, f"{name}:edit", enabled)

    def custom_panel_height(self, d) -> int:
        z = self._z
        bh, gap, lab = z(34), z(8), z(20)
        h = z(20) + z(58) + z(26) + 4 * (bh + gap) + (bh + gap) + (lab + bh + gap) + z(32) + z(36)
        if d.advanced_open:
            h += 3 * (lab + bh + gap) + z(24)
            if d.controller == "Smart" and d.param_set == "custom":
                h += lab + bh + gap
        return h + z(26) + bh + z(22)

    def _custom_panel(self, surf, st):
        """Modal Custom / Demo editor. Only its own controls are clickable while it is open."""
        from smarttraffic.conflicts import MOVES
        from smarttraffic.custom import CUSTOM_CONTROLLERS, UI_LIMIT_VEH_H
        from smarttraffic.viz_custom import (MEASURE_CHOICES, MOVEMENT_NAMES, PRESETS, SMART_FIELD_LABELS)
        d = st.custom_draft
        z, f = self._z, self.fonts
        W, H = surf.get_size()
        dim = pygame.Surface((W, H), pygame.SRCALPHA)
        dim.fill((3, 5, 8, 185))
        surf.blit(dim, (0, 0))
        self.buttons = []
        bh, gap, lab = z(34), z(8), z(20)
        pw = min(z(780), W - z(40))
        ph = min(self.custom_panel_height(d), H - z(16))
        r = pygame.Rect((W - pw) // 2, max(z(8), (H - ph) // 2), pw, ph)
        rrect(surf, r, T.panel, z(16), _mix(T.custom, T.panel, 0.55))
        x0, w = r.x + z(24), r.w - z(48)
        y = r.y + z(20)
        # ---- header
        badge = f.render(CUSTOM_BADGE, z(13), _hex("14081f"), True, spacing=1)
        br = badge.get_rect(topleft=(x0 + z(10), y + z(6)))
        rrect(surf, br.inflate(z(20), z(10)), T.custom, z(8))
        surf.blit(badge, br)
        text(surf, f, "Custom demand", (br.right + z(24), y), z(19), T.text, True)
        text(surf, f, "Not an official result · FT-A or Smart · results are tagged CUSTOM / DEMO",
             (br.right + z(24), y + z(26)), z(12), T.muted)
        y += z(58)
        # ---- demand grid
        text(surf, f, f"DEMAND (veh/h, max {UI_LIMIT_VEH_H})", (x0, y), z(11), T.text2, True, spacing=1)
        y += z(26)
        colw = (w - z(24)) // 2
        sw = min(z(190), colw - z(150))
        for ci, group in enumerate((("NS", "SS", "ES", "WS"), ("NL", "SL", "EL", "WL"))):
            cx = x0 + ci * (colw + z(24))
            for ri, m in enumerate(group):
                ry = y + ri * (bh + gap)
                text(surf, f, m, (cx, ry + bh / 2), z(14), T.text, True, "midleft")
                text(surf, f, MOVEMENT_NAMES[m], (cx + z(40), ry + bh / 2), z(12), T.muted, anchor="midleft")
                self._stepper(surf, pygame.Rect(cx + colw - sw, ry, sw, bh), d.display(m), d.focus == m,
                              (lambda m=m: d.step_demand(m, -1)), (lambda m=m: d.step_demand(m, +1)),
                              (lambda m=m: d.focus_field(m)), f"demand:{m}")
        y += 4 * (bh + gap)
        # ---- presets
        text(surf, f, "Presets", (x0, y + bh / 2), z(12), T.muted, anchor="midleft")
        px = x0 + z(70)
        pbw = (w - z(70) - 3 * gap) // len(PRESETS)
        for pid, plabel in PRESETS:
            self.text_button(surf, pygame.Rect(px, y, pbw, bh), plabel, (lambda p=pid: d.load_preset(p)),
                             f"preset:{pid}", size=z(12))
            px += pbw + gap
        y += bh + gap
        # ---- seed / controller / measurement
        seedw, ctlw = z(150), z(170)
        text(surf, f, "Seed", (x0, y), z(11), T.text2, True, spacing=1)
        text(surf, f, "Controller", (x0 + seedw + z(16), y), z(11), T.text2, True, spacing=1)
        text(surf, f, "Measurement", (x0 + seedw + ctlw + z(32), y), z(11), T.text2, True, spacing=1)
        y += lab
        self._stepper(surf, pygame.Rect(x0, y, seedw, bh), str(d.seed), False,
                      (lambda: d.set_seed(d.seed - 1)), (lambda: d.set_seed(d.seed + 1)), (lambda: None), "seed")
        self.segmented(surf, pygame.Rect(x0 + seedw + z(16), y, ctlw, bh), [(c, c) for c in CUSTOM_CONTROLLERS],
                       d.controller, d.set_controller, names=lambda v, l: f"controller:{v}")
        mx = x0 + seedw + ctlw + z(32)
        self.segmented(surf, pygame.Rect(mx, y, x0 + w - mx, bh),
                       [(s_, f"{s_ // 60} min") for s_ in MEASURE_CHOICES], d.measure_s, d.set_measure,
                       names=lambda v, l: f"measure:{v}", size=z(12))
        y += bh + gap
        # ---- saturation check
        sat = d.saturation()
        if sat is not None:
            col = {"ok": T.green, "near": T.amber, "over": T.red}[sat.level]
            msg = (f"Load X = {sat.degree:.2f} · within capacity" if sat.level == "ok"
                   else sat.messages[-1] if sat.messages else "")
            aa_circle(surf, (x0 + z(6), y + z(12)), z(5), col)
            self._wrap(surf, msg, x0 + z(20), y + z(3), w - z(20), z(12), col if sat.level != "ok" else T.text2)
        y += z(32)
        # ---- advanced (collapsed by default)
        self.icon_button(surf, pygame.Rect(x0, y, z(230), z(30)), _icon_chev_down if d.advanced_open else _icon_chev_right,
                         d.toggle_advanced, "advanced", label="Advanced settings")
        y += z(36)
        if d.advanced_open:
            rows = (("Smart parameters", [("T1", "Smart-T1 (locked)"), ("custom", "Custom τ / W_MAX / max green")],
                     d.param_set, d.set_param_set, "params", (lambda v: d.controller == "Smart")),)
            for title, items, cur, pick, nm, en in rows:
                text(surf, f, title.upper(), (x0, y), z(11), T.text2, True, spacing=1)
                y += lab
                self.segmented(surf, pygame.Rect(x0, y, w, bh), items, cur, pick,
                               names=(lambda v, l, nm=nm: f"{nm}:{v}"), enabled=en, size=z(12))
                y += bh + gap
            if d.controller == "Smart" and d.param_set == "custom":
                fw = (w - 3 * gap) // 4
                for i, name in enumerate(("tau_s", "w_max_s", "max_green_straight_s", "max_green_left_s")):
                    fx = x0 + i * (fw + gap)
                    text(surf, f, SMART_FIELD_LABELS[name], (fx, y), z(11), T.text2, True)
                    self._stepper(surf, pygame.Rect(fx, y + lab, fw, bh), d.display(name), d.focus == name,
                                  (lambda n=name: d.step_smart(n, -1)), (lambda n=name: d.step_smart(n, +1)),
                                  (lambda n=name: d.focus_field(n)), f"smart:{name}")
                y += lab + bh + gap
            text(surf, f, f"Traffic rules (all runs): {VEHICLE_POLICY_LABEL}.", (x0, y + z(2)), z(11), T.muted)
            y += z(24)
        # ---- message + footer
        fy = r.bottom - z(22) - bh
        msg = d.error or d.note
        if msg:
            self._wrap(surf, msg, x0, fy - z(24), w, z(12), T.red if d.error else T.amber)
        bw2 = z(120)
        self.text_button(surf, pygame.Rect(x0 + w - bw2, fy, bw2, bh), "Apply", st.apply_custom, "custom:apply",
                         primary=True)
        self.text_button(surf, pygame.Rect(x0 + w - 2 * bw2 - gap, fy, bw2, bh), "Cancel", st.cancel_custom,
                         "custom:cancel")
        text(surf, f, "Tab next field · type digits · Enter apply · Esc cancel", (x0, fy + bh / 2), z(11),
             T.muted, anchor="midleft")

    def _hardware(self, surf, r, st, hw):
        z, f = self._z, self.fonts
        glass(surf, r, z(12))
        x, y = r.x + z(16), r.y + z(14)
        text(surf, f, "HYBRID HARDWARE", (x, y), z(10), T.muted, True, spacing=1)
        y += z(24)
        right = r.right - z(16)
        if hw is None or not hw.enabled:
            aa_circle(surf, (x + z(4), y + z(8)), z(4), T.muted)
            text(surf, f, "Off · 16 simulated detectors", (x + z(16), y + z(8)), z(13), T.text, True, "midleft")
            y += z(24)
            self._wrap(surf, "IR marks the real detector's slot (NS:D1). Press H to connect the ESP32.",
                       x, y, r.w - z(32), z(12), T.text2)
            return
        for dev, port, state in hw.links:
            ok = state == "CONNECTED"
            col = T.green if ok else T.red
            glow(surf, (x + z(4), y + z(8)), z(8), col, 80)
            aa_circle(surf, (x + z(4), y + z(8)), z(4), col)
            text(surf, f, f"ESP32 {dev}", (x + z(16), y + z(8)), z(13), T.text, True, "midleft")
            text(surf, f, port, (x + z(16) + f.render(f"ESP32 {dev}", z(13), T.text, True).get_width() + z(8),
                                 y + z(8)), z(11), T.muted, anchor="midleft")
            text(surf, f, state.replace("_", " ").title(), (right, y + z(8)), z(12), col, True, "midright")
            y += z(24)
        for sid, occ, faulted in hw.real:
            s = "Fault · held" if faulted else ("Occupied" if occ else "Clear")
            col = T.fault if faulted else (T.det_on if occ else T.text2)
            text(surf, f, f"Real IR {sid}", (x + z(16), y + z(8)), z(12), T.text2, anchor="midleft")
            text(surf, f, s, (right, y + z(8)), z(12), col, True, "midright")
            y += z(21)
        for m, sent, acked, link in hw.mirror:
            if link != "CONNECTED" or not sent:
                s, col = "waiting for link", T.muted
            else:
                s = sent.replace("_", " ").title() + (" · acked" if acked == sent else " · sent")
                col = T.text
            text(surf, f, f"Physical head mirrors {m}", (x + z(16), y + z(8)), z(12), T.text2, anchor="midleft")
            text(surf, f, s, (right, y + z(8)), z(12), col, True, "midright")
            y += z(21)


# ================================================================ icons
def _icon_play(s, c, w, col):
    aa_poly(s, [(c[0] - w * 0.32, c[1] - w * 0.45), (c[0] + w * 0.48, c[1]), (c[0] - w * 0.32, c[1] + w * 0.45)], col)


def _icon_pause(s, c, w, col):
    for dx in (-0.24, 0.24):
        pygame.draw.rect(s, col, (round(c[0] + dx * w - w * 0.12), round(c[1] - w * 0.44), max(2, round(w * 0.24)),
                                  round(w * 0.88)), border_radius=2)


def _icon_reset(s, c, w, col):
    r = w * 0.46
    pts = [(c[0] + r * math.cos(math.radians(d)), c[1] + r * math.sin(math.radians(d))) for d in range(-60, 231, 15)]
    thick_polyline(s, pts, max(2, w * 0.15), col)
    e = pts[0]
    aa_poly(s, [(e[0] + w * 0.24, e[1] + w * 0.02), (e[0] - w * 0.12, e[1] - w * 0.12), (e[0] + w * 0.06, e[1] + w * 0.32)], col)


def _icon_chev_left(s, c, w, col):
    thick_polyline(s, [(c[0] + w * 0.18, c[1] - w * 0.38), (c[0] - w * 0.2, c[1]), (c[0] + w * 0.18, c[1] + w * 0.38)],
                   max(2, w * 0.17), col)


def _icon_chev_down(s, c, w, col):
    thick_polyline(s, [(c[0] - w * 0.38, c[1] - w * 0.18), (c[0], c[1] + w * 0.2), (c[0] + w * 0.38, c[1] - w * 0.18)],
                   max(2, w * 0.17), col)


def _icon_none(s, c, w, col):
    pass


def _icon_chev_right(s, c, w, col):
    thick_polyline(s, [(c[0] - w * 0.18, c[1] - w * 0.38), (c[0] + w * 0.2, c[1]), (c[0] - w * 0.18, c[1] + w * 0.38)],
                   max(2, w * 0.17), col)


def _icon_minus(s, c, w, col):
    thick_line(s, (c[0] - w * 0.36, c[1]), (c[0] + w * 0.36, c[1]), max(2, w * 0.16), col)


def _icon_plus(s, c, w, col):
    _icon_minus(s, c, w, col)
    thick_line(s, (c[0], c[1] - w * 0.36), (c[0], c[1] + w * 0.36), max(2, w * 0.16), col)
