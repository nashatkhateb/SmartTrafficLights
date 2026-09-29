"""viz -- real-time visualization of the 8-movement intersection (Stage 13).

Spec: docs/Eight-Movement_Intersection_Study.html §13; docs/ARCHITECTURE.md
("Visualization (Stage 13)").

    python -m smarttraffic.viz                      # window, S1, seed 1, Smart, paused at start
    python -m smarttraffic.viz --controller FT-A --scenario S5 --speed 5
    python -m smarttraffic.viz --hardware [--port COM5]   # 2 real IR detectors (NS:D1, NS:D2) + mirrored R/Y/G head
    python -m smarttraffic.viz --screenshot out.png --at 120   # headless still frame

Presentation layer only. The simulation runs unchanged in viz_session (Stage 12 runner
+ a read-only observer); this module only draws Snapshots. Nothing here reads or
writes the simulation objects, and no traffic-light logic lives here: every lamp is
SignalBank state passed through signals.lamps_for().

Keys: Space run/pause, R reset, 1-6 speed, C cycle controller, Left/Right scenario,
Up/Down seed, H hardware on/off, F11 fullscreen, Esc quit.

Custom / Demo (Stage 14b): "Custom..." after S10 in the scenario selector (or the
Custom... entry itself) opens the Custom / Demo panel: 8 demands (veh/h, capped at 1800),
seed, FT-A or Smart, measurement duration, and collapsed Advanced settings (Smart parameter
set T1 / custom). Apply loads it; a CUSTOM / DEMO badge is shown while it is active.
Official scenarios S1-S10 keep the locked Smart-T1 settings.

Traffic rules: every run the UI starts (official scenarios, Custom / Demo, screenshots,
resets, scenario / controller / seed switching) uses the standard vehicle policy -- Stop
on Yellow and the first queued car starting at green onset (vehicles.STANDARD). ui_config
enforces it; the legacy-study policy is never shown in the UI.
In the panel: Tab next field, digits type, Enter apply, Esc cancel.

Smart in the UI is always the approved Smart-T1 (config/smart_params_locked.json: tau 15 s,
W_MAX 90 s, all other parameters locked): every official configuration the UI runs gets the
locked parameters (ui_config), and the Custom panel offers Smart-T1 or custom values only.
Smart-study remains an internal/historical comparison (official campaign), not a UI option.
"""
import argparse
import os
import sys
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from dataclasses import replace as _replace

from smarttraffic.custom import CUSTOM_CONTROLLERS, CUSTOM_SCENARIO
from smarttraffic.runner import CONTROLLERS, DEFAULT_VEHICLE_POLICY, RunConfig
from smarttraffic.viz_custom import CustomDraft
from smarttraffic.viz_session import HARDWARE_SPEED, SPEEDS, Snapshot, VizSession

__all__ = ["main", "App", "Renderer", "Theme", "capture", "render_to_file", "SCENARIOS",
           "controllers_for"]

try:                                    # the drawing lives in viz_render (needs pygame)
    import pygame
    from smarttraffic.viz_render import (SCENARIOS, Fonts, Renderer, T, Theme, View,
                                         controllers_for)
except ImportError:                     # pragma: no cover - reported by main()
    pygame = None



# ================================================================ Smart-T1 in the UI
_T1_CACHE: Dict[str, object] = {}


def smart_t1_params():
    """The locked Smart-T1 parameters (verified lock; loaded once)."""
    if "p" not in _T1_CACHE:
        from smarttraffic import tuning as tu
        _T1_CACHE["p"] = tu.load_lock(tu.LOCK_FILE)["smart_params"]
    return _T1_CACHE["p"]


def ui_config(config: RunConfig) -> RunConfig:
    """What the UI actually runs: always the standard vehicle policy; official configurations
    always use Smart-T1 (for FT controllers the Smart parameters are unused, so switching back
    to Smart stays on T1). Custom / Demo configurations keep their own validated parameters."""
    if config.vehicle_policy != DEFAULT_VEHICLE_POLICY:
        config = _replace(config, vehicle_policy=DEFAULT_VEHICLE_POLICY)
    if config.mode != "official" or config.smart_params == smart_t1_params():
        return config
    return _replace(config, smart_params=smart_t1_params())


def official_config(scenario: str, seed: int, controller: str) -> RunConfig:
    return ui_config(RunConfig(scenario, seed, controller))


# ================================================================ application state
class UIState:
    """What the user has chosen, and the session that is running it."""

    def __init__(self, config: RunConfig, speed: Optional[float] = 1.0, hardware: bool = False,
                 hw_path: Optional[str] = None, port: Optional[str] = None, transport_factory=None,
                 session_factory: Optional[Callable[..., VizSession]] = None):
        self.config = config
        self.speed = speed
        self.hardware = hardware
        self.hw_path = hw_path
        self.port = port
        self.transport_factory = transport_factory
        self.session_factory = session_factory or VizSession
        self.message: Optional[str] = None
        self.session: Optional[VizSession] = None
        self.custom_spec = None                          # last applied custom.CustomSpec
        self.custom_draft: Optional[CustomDraft] = None  # not None while the panel is open
        self._start(running=False)

    # ---------------------------------------------------- session
    def _hw_config(self):
        from smarttraffic.hardware import DEFAULT_CONFIG_FILE, load_hardware_config
        cfg = load_hardware_config(self.hw_path or DEFAULT_CONFIG_FILE)
        if self.port:
            for dev in cfg.devices:
                cfg = cfg.with_port(dev, self.port)
        return cfg

    def _start(self, running: bool) -> None:
        if self.session is not None:
            self.session.stop()
        self.config = ui_config(self.config)             # official Smart = Smart-T1, always
        hw = None
        if self.hardware:
            try:
                hw = self._hw_config()
                self.message = None
            except Exception as e:
                self.hardware = False
                self.message = f"Hardware config could not be loaded ({e}); running without hardware."
        speed = HARDWARE_SPEED if hw is not None else self.speed
        self.session = self.session_factory(self.config, hw_config=hw, transport_factory=self.transport_factory,
                                            speed=speed, running=running).start()
        if hw is not None:
            self.speed = HARDWARE_SPEED

    @property
    def running(self) -> bool:
        return self.session is not None and self.session.running and not self.finished

    @property
    def finished(self) -> bool:
        return self.session is not None and self.session.finished

    @property
    def result(self):
        return self.session.result if self.session is not None else None

    @property
    def error(self) -> Optional[str]:
        return self.session.error if self.session is not None else None

    # ---------------------------------------------------- actions
    def toggle_run(self) -> None:
        if self.session is not None and not self.finished:
            self.session.set_running(not self.session.running)

    def reset(self) -> None:
        self.manual_message = None                   # a new session: no pending manual vehicles
        self._start(running=False)

    def set_speed(self, sp: Optional[float]) -> None:
        if self.hardware:
            return
        self.speed = sp
        self.session.set_speed(sp)

    def _reconfigure(self, **kw) -> None:
        from dataclasses import replace
        was = self.running
        self.config = replace(self.config, **kw)
        self._start(running=was)

    def set_controller(self, c: str) -> None:
        if c != self.config.controller and c in controllers_for(self.config.scenario):
            if self.is_custom:
                self._load_custom(_replace(self.custom_spec, controller=c), running=self.running)
            else:
                self._reconfigure(controller=c)

    def cycle_controller(self) -> None:
        cs = controllers_for(self.config.scenario)
        i = cs.index(self.config.controller) if self.config.controller in cs else -1
        self.set_controller(cs[(i + 1) % len(cs)])

    def _set_scenario(self, step: int) -> None:
        ids = [s for s, _ in SCENARIOS] + [CUSTOM_SCENARIO]
        i = ids.index(self.config.scenario) if self.config.scenario in ids else 0
        sc = ids[(i + step) % len(ids)]
        if sc == CUSTOM_SCENARIO:
            if self.custom_spec is None:
                self.open_custom()                   # nothing to show yet: ask for demands
            else:
                self._load_custom(self.custom_spec, running=self.running)
            return
        ctrl = self.config.controller if self.config.controller in controllers_for(sc) else "FT-A"
        if self.is_custom:                           # back to the study: a clean official config
            was = self.running
            self.config = official_config(sc, self.config.seed, ctrl)
            self._start(running=was)
        else:
            self._reconfigure(scenario=sc, controller=ctrl)

    def next_scenario(self) -> None:
        self._set_scenario(1)

    def prev_scenario(self) -> None:
        self._set_scenario(-1)

    def next_seed(self) -> None:
        self._set_seed(self.config.seed + 1)

    def prev_seed(self) -> None:
        if self.config.seed > 0:
            self._set_seed(self.config.seed - 1)

    def _set_seed(self, seed: int) -> None:
        if self.is_custom:
            self._load_custom(_replace(self.custom_spec, seed=seed), running=self.running)
        else:
            self._reconfigure(seed=seed)

    # ---------------------------------------------------- Custom / Demo
    @property
    def is_custom(self) -> bool:
        return self.config.mode == "custom"

    @property
    def custom_open(self) -> bool:
        return self.custom_draft is not None

    def open_custom(self) -> None:
        if self.custom_spec is not None:
            self.custom_draft = CustomDraft.from_spec(self.custom_spec)
        else:
            ctrl = self.config.controller if self.config.controller in CUSTOM_CONTROLLERS else "Smart"
            self.custom_draft = CustomDraft(seed=self.config.seed, controller=ctrl)

    def cancel_custom(self) -> None:
        self.custom_draft = None

    def apply_custom(self) -> bool:
        """Validate the panel; on success load it (paused) and close the panel."""
        if self.custom_draft is None:
            return False
        spec, err = self.custom_draft.to_spec()
        if err is not None:
            return False
        self.custom_draft = None
        self._load_custom(spec, running=False)
        return True

    def _load_custom(self, spec, running: bool) -> None:
        self.custom_spec = spec
        self.config = spec.config()
        self._start(running=running)

    # ---------------------------------------------------- manual vehicles (Custom / Demo)
    manual_count = 10                                    # the exact-count field (1 .. MAX_BATCH)
    manual_message: Optional[str] = None                 # last manual-injection result (card text)

    def set_manual_count(self, n: int) -> None:
        from smarttraffic.manual import MAX_BATCH
        self.manual_count = int(min(MAX_BATCH, max(1, n)))

    def inject(self, m: str, n: int) -> bool:
        """Add exactly n manual vehicles to m (Custom / Demo only). Sets a short message."""
        why = "Custom / Demo mode only" if not self.is_custom else self.session.inject(m, n)
        self.manual_message = (f"+{n} {m} added" if why is None else f"{m}: {why}")
        return why is None

    def manual_state(self):
        """(injected per movement, blocked movements, pending count, exact count) or None."""
        inj = getattr(self.session, "injector", None) if self.session is not None else None
        if inj is None:
            return None
        return inj.counts(), inj.blocked, inj.pending(), self.manual_count, self.manual_message

    def toggle_hardware(self) -> None:
        self.hardware = not self.hardware
        self.message = None
        self._start(running=False)

    def close(self) -> None:
        if self.session is not None:
            self.session.stop()


# ================================================================ app
class App:
    def __init__(self, st: UIState, size=None, fullscreen=False):
        _dpi_aware()
        pygame.init()
        pygame.display.set_caption("Smart Traffic Lights — 8-movement intersection")
        info = pygame.display.Info()
        if size is None:
            size = (min(1760, int(info.current_w * 0.92)), min(990, int(info.current_h * 0.88)))
        self.windowed = size
        flags = pygame.RESIZABLE
        self.screen = pygame.display.set_mode(size, pygame.FULLSCREEN if fullscreen else flags)
        self.fonts = Fonts()
        self.r = Renderer(self.fonts)
        self.r.real_ids_hint = _real_ids_hint()
        self.st = st
        self.clock = pygame.time.Clock()

    def handle_key(self, key) -> bool:
        st = self.st
        speeds = {pygame.K_1: 0, pygame.K_2: 1, pygame.K_3: 2, pygame.K_4: 3, pygame.K_5: 4, pygame.K_6: 5}
        if key == pygame.K_ESCAPE:
            return False
        if key == pygame.K_SPACE:
            st.toggle_run()
        elif key == pygame.K_r:
            st.reset()
        elif key in speeds:
            st.set_speed(SPEEDS[speeds[key]])
        elif key == pygame.K_c:
            st.cycle_controller()
        elif key == pygame.K_RIGHT:
            st.next_scenario()
        elif key == pygame.K_LEFT:
            st.prev_scenario()
        elif key == pygame.K_UP:
            st.next_seed()
        elif key == pygame.K_DOWN:
            st.prev_seed()
        elif key == pygame.K_h:
            st.toggle_hardware()
        elif key == pygame.K_F11:
            if self.screen.get_flags() & pygame.FULLSCREEN:
                self.screen = pygame.display.set_mode(self.windowed, pygame.RESIZABLE)
            else:
                self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        return True

    def custom_key(self, key, char: str = "") -> bool:
        """Keys while the Custom / Demo panel is open. Always True (Esc closes the panel only)."""
        d = self.st.custom_draft
        if key == pygame.K_ESCAPE:
            self.st.cancel_custom()
        elif key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            if d.focus is not None:
                d.focus_field(None)
            else:
                self.st.apply_custom()
        elif key == pygame.K_BACKSPACE:
            d.backspace()
        elif key == pygame.K_TAB:
            from smarttraffic.viz_custom import TAB_ORDER
            order = list(TAB_ORDER)
            nxt = order[(order.index(d.focus) + 1) % len(order)] if d.focus in order else order[0]
            d.focus_field(nxt)
        else:
            d.type_char(char)
        return True

    def click(self, pos) -> bool:
        """Run the action of the button under pos (buttons of the last drawn frame)."""
        for rect, action, _ in self.r.buttons:
            if rect.collidepoint(pos):
                action()
                return True
        return False

    def frame(self) -> bool:
        """Handle pending events and draw one frame. False when the user quits."""
        alive = True
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                alive = False
            elif ev.type == pygame.KEYDOWN:
                if self.st.custom_open:
                    self.custom_key(ev.key, getattr(ev, "unicode", ""))
                else:
                    alive = self.handle_key(ev.key) and alive
            elif ev.type == pygame.MOUSEMOTION:
                self.r.hover = ev.pos
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                self.click(ev.pos)
            elif ev.type == pygame.VIDEORESIZE and not (self.screen.get_flags() & pygame.FULLSCREEN):
                self.windowed = (max(1100, ev.w), max(680, ev.h))
                self.screen = pygame.display.set_mode(self.windowed, pygame.RESIZABLE)
        a, b, t = self.st.session.frame()
        self.r.draw(self.screen, self.st, a, b, t)
        pygame.display.flip()
        return alive

    def run(self) -> int:
        try:
            while self.frame():
                self.clock.tick(60)
        finally:
            self.close()
        return 0

    def close(self) -> None:
        self.st.close()
        pygame.quit()


def _dpi_aware() -> None:
    if sys.platform.startswith("win"):
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass


def _real_ids_hint() -> Tuple:
    """Which detector slot is the real one (config/hardware.json), for drawing the IR slot
    while hardware is off. Missing/invalid config -> none."""
    try:
        from smarttraffic.hardware import load_hardware_config
        return tuple(load_hardware_config().sensors)
    except Exception:
        return ()


# ================================================================ headless capture
def capture(config: RunConfig, times_ms: Sequence[int], *, hw_config=None, transport_factory=None,
            arrivals=None) -> Dict[int, Tuple[Snapshot, Optional[Snapshot]]]:
    """Snapshots at (and one step after) each requested time, from one run of the shared
    runner with the same read-only snapshot builder the live session uses. For stills and
    tests. The run is stopped after the last requested time."""
    from smarttraffic.runner import simulate
    from smarttraffic.viz_session import SnapshotBuilder
    want = sorted(set(int(t) for t in times_ms))
    out: Dict[int, List] = {}
    state = {"builder": None, "hw": None, "step": 0}

    class Done(Exception):
        pass

    def factory(vm):
        from smarttraffic.hardware import build_hardware
        state["hw"] = build_hardware(hw_config, vm, transport_factory)
        return state["hw"].detectors

    def obs(view):
        state["step"] += 1
        if state["hw"] is not None and state["hw"].mirror is not None:
            state["hw"].mirror.update(view.signals)
        if state["builder"] is None:
            state["builder"] = SnapshotBuilder(config, state["hw"], hw_config)
        b = state["builder"]
        b.tracker.update(view.vehicles, view.now_ms)
        for t in want:
            if t in out and len(out[t]) == 1:
                out[t].append(b.build(view, state["step"]))
            elif t not in out and view.now_ms >= t:
                out[t] = [b.build(view, state["step"])]
        if len(out) == len(want) and all(len(v) == 2 for v in out.values()):
            raise Done()
    kw = {"observers": [obs]}
    if hw_config is not None:
        kw["source_factory"] = factory
        kw["exclude_movements"] = hw_config.real_only_movements   # same rule as VizSession
    if arrivals is not None:
        kw["arrivals"] = arrivals
    try:
        simulate(config, collect_stats=False, **kw)
    except Done:
        pass
    finally:
        if state["hw"] is not None:
            state["hw"].close()
    return {t: (v[0], v[1] if len(v) > 1 else None) for t, v in out.items()}


class _StillState:
    """UIState stand-in for still frames."""

    def __init__(self, config, running=True, speed=1.0, hardware=False, finished=False, result=None,
                 custom_draft=None):
        self.config, self.running, self.speed, self.hardware = config, running, speed, hardware
        self.finished, self.result, self.error, self.message = finished, result, None, None
        self.custom_draft = custom_draft
        self.custom_open = custom_draft is not None
        self.is_custom = config.mode == "custom"

    def manual_state(self):                      # still frames: an empty manual-injection card
        if self.config.mode != "custom":
            return None
        from smarttraffic.conflicts import MOVES
        blocked = ("NS",) if self.hardware else ()
        return {m: 0 for m in MOVES}, frozenset(blocked), 0, 10, None

    def __getattr__(self, name):                 # button actions do nothing in a still
        return lambda *a, **k: None


def render_to_file(path: str, config: RunConfig, t_s: float, size=(1600, 900), *, frac: float = 0.0,
                   hw_config=None, transport_factory=None, running=True, arrivals=None,
                   custom_draft=None) -> Snapshot:
    """Render one frame at simulation time t_s (+ frac of a step, for interpolation)."""
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode(size)
    t = round(t_s * 1000)
    snaps = capture(config, [t], hw_config=hw_config, transport_factory=transport_factory, arrivals=arrivals)
    a, b = snaps[t]
    r = Renderer(Fonts())
    r.real_ids_hint = _real_ids_hint()
    st = _StillState(config, running=running, hardware=hw_config is not None, custom_draft=custom_draft)
    disp = a.t_ms + frac * ((b.t_ms - a.t_ms) if b is not None else 0)
    r.draw(screen, st, a, b, disp)
    pygame.image.save(screen, path)
    return a


# ================================================================ entry point
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m smarttraffic.viz", description=__doc__.split("\n")[0])
    ap.add_argument("--scenario", default="S1", choices=[s for s, _ in SCENARIOS])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--controller", default="Smart", choices=CONTROLLERS)
    ap.add_argument("--speed", default="1", help="0.5, 1, 2, 5, 10 or max")
    ap.add_argument("--hardware", action="store_true", help="use config/hardware.json (real IR + mirror)")
    ap.add_argument("--hw-config", default=None, help="hardware config file (default config/hardware.json)")
    ap.add_argument("--port", default=None, help="override the ESP32 serial port, e.g. COM5")
    ap.add_argument("--size", default=None, help="window size, e.g. 1600x900")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--screenshot", default=None, help="headless: write one frame to this PNG and exit")
    ap.add_argument("--at", type=float, default=0.0, help="with --screenshot: simulation time in s")
    args = ap.parse_args(argv)
    if pygame is None:
        print("pygame is not installed (pip install -r requirements.txt)")
        return 2
    if args.controller not in controllers_for(args.scenario):
        ap.error(f"{args.controller} has no plan for {args.scenario}")
    speed = None if args.speed.lower() == "max" else float(args.speed)
    if speed not in SPEEDS:
        ap.error(f"--speed must be one of 0.5, 1, 2, 5, 10, max")
    size = tuple(int(v) for v in args.size.lower().split("x")) if args.size else None
    config = official_config(args.scenario, args.seed, args.controller)
    if args.screenshot:
        hw = None
        if args.hardware:
            from smarttraffic.hardware import DEFAULT_CONFIG_FILE, load_hardware_config
            hw = load_hardware_config(args.hw_config or DEFAULT_CONFIG_FILE)
        render_to_file(args.screenshot, config, args.at, size or (1600, 900), hw_config=hw)
        print(f"wrote {args.screenshot}")
        return 0
    st = UIState(config, speed=speed, hardware=args.hardware, hw_path=args.hw_config, port=args.port)
    return App(st, size=size, fullscreen=args.fullscreen).run()


if __name__ == "__main__":
    sys.exit(main())
