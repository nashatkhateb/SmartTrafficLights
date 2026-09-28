"""viz_session -- runs one simulation for the visualization (Stage 13). No pygame here.

The simulation is the unmodified Stage 12 shared runner: runner.run(config,
observers=[...]). This module only adds a READ-ONLY observer (the last thing in every
step, after statistics) that

  * builds immutable Snapshots of what the screen needs (vehicles, the 8 signal heads,
    the 16 detectors, controller reasoning, live metrics, hardware status),
  * paces the run to the wall clock at the chosen speed (sleeping between steps), and
  * blocks between steps while paused, or aborts the run when the session is stopped.

Sleeping or blocking between two complete steps cannot change what the steps compute:
step sizes, step order and every model call are the runner's, so a run shown on screen
gives the same RunResult as the same RunConfig without a screen (tests/test_viz.py).
Rendering never touches the simulation objects: it only reads Snapshots, which are
built on the simulation thread and handed over under a lock.

Display time: the renderer asks display_time() for the simulation time to draw. It
follows the wall clock at the chosen speed one step behind the newest snapshot, so
drawing can interpolate between two snapshots (purely visual). Paused -> frozen.

Hardware mode (optional): build_hardware() wires the configured real detectors into
the run through the runner's source_factory (Stage 9 hybrid source: the real detector
REPLACES its virtual twin), and the SignalMirror is updated from this observer, after
the controller (study step order). Hardware mode always runs at 1x (the link maps
device time to simulation time, which must follow the wall clock). A missing or
unplugged ESP32 is a fault the Stage 9 code already handles (the detector is faulted,
Smart puts that movement on fail-safe recall); the UI keeps running.
"""
from collections import deque
from dataclasses import dataclass, field
import threading
import time
from typing import Callable, Deque, Dict, List, NamedTuple, Optional, Sequence, Tuple

from smarttraffic.conflicts import MOVES, compatible
from smarttraffic.runner import RunConfig, StepView, run
from smarttraffic.sensors import DETECTORS, PASSAGE_TIME_MS
from smarttraffic.signals import (FLASHING_GREEN_MS, GREEN, RED, RED_YELLOW, SignalState,
                                  lamps_for, movement_type)
from smarttraffic.stats import RunResult
from smarttraffic.vehicles import BOX_LENGTH, VehicleState

__all__ = ["SPEEDS", "HARDWARE_SPEED", "VehicleView", "SignalView", "SensorView", "Metrics",
           "SmartView", "FixedView", "HardwareView", "Snapshot", "SnapshotBuilder",
           "MetricTracker", "VizSession", "signal_display", "run_phase"]

SPEEDS: Tuple[Optional[float], ...] = (0.5, 1.0, 2.0, 5.0, 10.0, None)    # None = as fast as possible
HARDWARE_SPEED = 1.0
EXIT_TRAIL_MS = 6000              # draw exited vehicles this long on the exit arm (visual)
FLOW_WINDOW_MS = 300_000          # live flow: vehicles crossed in the last 5 minutes
DECISIONS_SHOWN = 4


# ======================================================== snapshot records
class VehicleView(NamedTuple):
    id: int
    movement: str
    u: float                      # path coordinate of the FRONT bumper (viz_geometry), metres
    state: str                    # VehicleState name (EXITED: drawn on the exit arm only)


class SignalView(NamedTuple):
    movement: str
    state: SignalState
    since_ms: int
    remaining_ms: Optional[int]   # timed states only


class SensorView(NamedTuple):
    movement: str
    d1: bool
    d2: bool
    call: bool
    call_since_ms: Optional[int]
    qhat: int
    level2: bool
    in_service: bool
    gap_out: bool
    faulted: Tuple[str, ...]      # detectors of this movement reported faulted


class Metrics(NamedTuple):
    in_system: int                # scheduled and not yet left the box (road + box + held outside)
    queue: int                    # stopped on the approaches + held outside (Stage 12 queue)
    crossed: int                  # crossed the stop line since t = 0
    flow_veh_h: Optional[float]   # crossings in the last 5 min (or since t = 0), per hour
    avg_delay_s: Optional[float]  # mean control delay of measured vehicles that have crossed
    delayed_n: int                # how many vehicles that mean is over
    longest_wait_s: Optional[float]   # longest current call (RED with a call), seconds


class SmartView(NamedTuple):
    target: Optional[Tuple[str, ...]]
    hold: Optional[str]
    committed: Optional[Tuple[Tuple[str, ...], Tuple[str, ...]]]
    priority: Dict[str, float]            # waiting movements: P = Qhat + wait / tau
    wait_ms: Dict[str, int]
    recall: Tuple[str, ...]               # fail-safe recall (faulted detector)
    green_info: Dict[str, Tuple[Optional[int], Optional[int], Optional[int]]]  # onset, min G, max start
    decisions: Tuple[object, ...]         # newest last (controller_smart.Decision)
    why: Tuple[str, ...]                  # plain-language explanation lines
    tau_ms: int
    w_max_ms: int
    max_green_ms: Dict[str, int]


class FixedView(NamedTuple):
    plan: str
    schedule: str
    phases: Tuple[Tuple[str, Tuple[str, ...], int], ...]      # (name, movements, duration ms)
    cycle_ms: int
    current: Optional[object]             # controller_fixed.PhaseRecord
    next_command_ms: int


class HardwareView(NamedTuple):
    enabled: bool
    links: Tuple[Tuple[str, str, str], ...]            # (device, port, LinkState name)
    real: Tuple[Tuple[str, bool, bool], ...]           # (sensor id "NS:D1", occupied, faulted)
    mirror: Tuple[Tuple[str, Optional[str], Optional[str], str], ...]   # (movement, sent, acked, link)
    real_ids: Tuple[Tuple[str, str], ...]              # detector positions served by hardware
    note: str
    real_only: Tuple[str, ...] = ()                    # movements with real demand only (no virtual vehicles)


NO_HARDWARE = HardwareView(False, (), (), (), (), "Hardware off: all 16 detectors are simulated")


@dataclass(frozen=True)
class Snapshot:
    t_ms: int
    step: int
    config: RunConfig
    vehicles: Tuple[VehicleView, ...]
    signals: Dict[str, SignalView]
    sensors: Dict[str, SensorView]
    queue: Dict[str, int]                 # stopped on the road, per movement
    outside: Dict[str, int]               # held outside (no room at the 150 m entry)
    metrics: Metrics
    controller_kind: str                  # "Smart" | "Fixed"
    smart: Optional[SmartView]
    fixed: Optional[FixedView]
    hardware: HardwareView

    def lamps(self, m: str, t_ms: Optional[int] = None):
        """Lamp outputs of m's head at t_ms (default: the snapshot time), computed with the
        SignalBank's own lamps_for() from the state recorded in this snapshot."""
        s = self.signals[m]
        t = self.t_ms if t_ms is None else t_ms
        return lamps_for(s.state, max(0, int(t) - s.since_ms))


def run_phase(config: RunConfig, t_ms: int) -> str:
    w = config.window()
    if t_ms < 0:
        return "WARM-UP"
    if t_ms < w.measure_ms:
        return "MEASUREMENT"
    return "FLUSH"


_DISPLAY_NAMES = {
    SignalState.RED: "Red", SignalState.RED_YELLOW: "Red + Yellow", SignalState.GREEN: "Green",
    SignalState.FLASHING_GREEN: "Flashing green", SignalState.YELLOW: "Yellow",
    SignalState.ALL_RED: "All-red clearance",
}


def signal_display(state: SignalState) -> str:
    return _DISPLAY_NAMES[state]


# ======================================================== live metrics
class MetricTracker:
    """Incremental live metrics from the VehicleModel's public accessors (read-only)."""

    def __init__(self, config: RunConfig):
        self.measure_ms = config.window().measure_ms
        self._seen = {m: 0 for m in MOVES}
        self._flow: Deque[int] = deque()
        self.crossed = 0
        self._delay_sum_ms = 0
        self._delay_n = 0

    def update(self, vm, now: int) -> None:
        for m in MOVES:
            cr = vm.crossed(m)
            for v in cr[self._seen[m]:]:
                if v.cross_ms >= 0:
                    self.crossed += 1
                    self._flow.append(v.cross_ms)
                if 0 <= v.scheduled_ms < self.measure_ms:
                    self._delay_sum_ms += v.cross_ms - v.free_flow_cross_ms
                    self._delay_n += 1
            self._seen[m] = len(cr)
        while self._flow and self._flow[0] <= now - FLOW_WINDOW_MS:
            self._flow.popleft()

    def metrics(self, vm, sensors, signals, now: int) -> Metrics:
        in_sys = queue = 0
        for m in MOVES:
            road = vm.vehicles(m)
            held = len(vm.waiting_outside(m))
            in_sys += len(road) + len(vm.in_intersection(m)) + held
            queue += sum(v.state is VehicleState.STOPPED for v in road) + held
        span = min(FLOW_WINDOW_MS, now) if now > 0 else 0
        flow = len(self._flow) * 3_600_000 / span if span >= 10_000 else None
        avg = self._delay_sum_ms / self._delay_n / 1000 if self._delay_n else None
        waits = [now - sensors.call_since_ms(m) for m in MOVES
                 if signals.state(m) is RED and sensors.has_call(m)]
        return Metrics(in_sys, queue, self.crossed, flow, avg, self._delay_n,
                       max(waits) / 1000 if waits else None)


# ======================================================== snapshot builder
def _fmt_s(ms: int) -> str:
    return f"{ms / 1000:.1f} s"


def _join(ms: Sequence[str]) -> str:
    return " + ".join(ms)


class SnapshotBuilder:
    """Reads the simulation objects of one StepView into a Snapshot. Read-only."""

    def __init__(self, config: RunConfig, hardware=None, hw_config=None):
        self.config = config
        self.tracker = MetricTracker(config)
        self.hw = hardware
        self.hw_config = hw_config
        self.real_ids: Tuple[Tuple[str, str], ...] = tuple(hw_config.sensors) if hw_config else ()
        self.real_only: Tuple[str, ...] = hw_config.real_only_movements if hw_config else ()

    # -------------------------------------------------------------- vehicles
    @staticmethod
    def _vehicles(vm, now: int) -> Tuple[Tuple[VehicleView, ...], Dict[str, int], Dict[str, int]]:
        out: List[VehicleView] = []
        queue, outside = {}, {}
        for m in MOVES:
            box = BOX_LENGTH[movement_type(m)]
            road = vm.vehicles(m)
            queue[m] = sum(v.state is VehicleState.STOPPED for v in road)
            outside[m] = len(vm.waiting_outside(m))
            for v in road:
                out.append(VehicleView(v.id, m, -v.position, v.state.name))
            for v in vm.in_intersection(m):
                out.append(VehicleView(v.id, m, -v.position, v.state.name))
            done = vm.completed(m)
            for v in reversed(done):
                if v.exit_ms < now - EXIT_TRAIL_MS:
                    break
                # purely visual: keep moving at the box speed along the exit arm
                out.append(VehicleView(v.id, m, box + 9.0 * (now - v.exit_ms) / 1000, "EXITED"))
        return tuple(out), queue, outside

    # -------------------------------------------------------------- controllers
    def _smart(self, ctrl, sensors, signals, now: int) -> SmartView:
        p = ctrl.params
        st = signals.states()
        recall = ctrl.faulted_movements()
        waiting = [m for m in MOVES if st[m] is RED and (sensors.has_call(m) or m in recall)]
        wait_ms, prio = {}, {}
        for m in waiting:
            since = sensors.call_since_ms(m) if sensors.has_call(m) else None
            w = now - since if since is not None else 0
            q = sensors.qhat(m) if sensors.has_call(m) else 0
            if m in recall:
                q = max(q, p.fault_qhat)
            wait_ms[m] = w
            prio[m] = q + (w / p.tau_ms if p.use_aging else 0.0)
        info = {m: (ctrl.green_onset_ms(m), ctrl.min_green_of(m), ctrl.max_start_ms(m)) for m in MOVES}
        decisions = tuple(ctrl.decision_log[-DECISIONS_SHOWN:])
        why = self._smart_why(ctrl, sensors, st, now, waiting, wait_ms, info)
        return SmartView(ctrl.last_target, ctrl.last_hold, ctrl.committed(), prio, wait_ms, recall,
                         info, decisions, why, p.tau_ms, p.w_max_ms,
                         {t: p.max_green_ms(t) for t in ("straight", "left")})

    def _smart_why(self, ctrl, sensors, st, now, waiting, wait_ms, info) -> Tuple[str, ...]:
        p = ctrl.params
        lines: List[str] = []
        hold = ctrl.last_hold
        if hold is not None:
            lines.append(f"Starvation hold: {hold} has waited {wait_ms.get(hold, 0) / 1000:.0f} s "
                         f"(limit {p.w_max_ms / 1000:.0f} s), so the target must include it")
        com = ctrl.committed()
        if com is not None:
            stopped, to_start = com
            last = ctrl.decision_log[-1] if ctrl.decision_log else None
            reasons = dict(last.stopped) if last is not None else {}
            names = {"gap": "gap-out", "max": "max-out", "hold": "starvation hold"}
            if stopped:
                ended = ", ".join(f"{g} ({names.get(reasons.get(g), 'ended')})" for g in stopped)
                lines.append(f"Switching: ended {ended}")
            lines.append(f"Clearing, then starting {_join(to_start)}")
            return tuple(lines)
        target = ctrl.last_target
        if target is None:
            serving = [m for m in MOVES if st[m] in (RED_YELLOW, GREEN)]
            if serving:
                lines.append(f"No waiting demand: resting in green ({_join(serving)})")
            else:
                lines.append("No calls: the intersection rests in red")
            return tuple(lines)
        to_start = [m for m in target if m in waiting and st[m] is RED]
        if not to_start:
            lines.append(f"Serving the target {_join(target)}")
            return tuple(lines)
        blockers = [x for x in MOVES if st[x] in (RED_YELLOW, GREEN)
                    and any(not compatible(x, s) for s in to_start)]
        if not blockers:
            lines.append(f"Target {_join(target)}: waiting for clearance to start {_join(to_start)}")
            return tuple(lines)
        lines.append(f"Target {_join(target)}; {_join(blockers)} must end first")
        for g in blockers:
            onset, min_g, max_start = info[g]
            if st[g] is RED_YELLOW or onset is None:
                lines.append(f"{g}: starting up (red + yellow)")
                continue
            min_left = onset + min_g - FLASHING_GREEN_MS - now
            if min_left > 0:
                lines.append(f"{g}: minimum green, {_fmt_s(min_left)} left")
                continue
            parts = []
            gt = sensors.gap_timer_ms(g)
            if gt is not None and not sensors.gap_out(g):
                parts.append("traffic still arriving" if gt == 0 else
                             f"gap {gt / 1000:.1f} s of {PASSAGE_TIME_MS / 1000:g} s")
            if p.use_max_green and max_start is not None:
                mx = max_start + p.max_green_ms(movement_type(g)) - FLASHING_GREEN_MS - now
                parts.append(f"max-out in {_fmt_s(max(0, mx))}")
            lines.append(f"{g}: " + ("; ".join(parts) if parts else "releasable"))
        return tuple(lines)

    @staticmethod
    def _fixed(ctrl, now: int) -> FixedView:
        cur = ctrl.current_phase()
        plan = ctrl.schedule.plan_at(cur.start_ms if cur is not None else now)
        phases = tuple((ph.name, tuple(ph.movements), plan.phase_duration_ms(i))
                       for i, ph in enumerate(plan.phases))
        return FixedView(plan.name, ctrl.schedule.name, phases, plan.cycle_ms, cur, ctrl.next_command_ms())

    # -------------------------------------------------------------- hardware
    def _hardware(self) -> HardwareView:
        if self.hw is None:
            return NO_HARDWARE
        cfg = self.hw_config
        links = tuple((d, cfg.devices[d].port, l.state.name) for d, l in self.hw.hub.links.items())
        real = ()
        if self.hw.real is not None:
            st = self.hw.real.status()
            real = tuple((sid, bool(v["occupied"]), bool(v["fault"])) for sid, v in st.items())
        mirror = ()
        if self.hw.mirror is not None:
            mirror = tuple((m, v["sent"], v["acked"], v["link"]) for m, v in self.hw.mirror.status().items())
        bad = [d for d, _, s in links if s != "CONNECTED"]
        note = ("All hardware links connected" if not bad else
                "ESP32 not connected: real detectors faulted (held), movement on fail-safe recall")
        return HardwareView(True, links, real, mirror, self.real_ids, note, self.real_only)

    # -------------------------------------------------------------- snapshot
    def build(self, view: StepView, step: int) -> Snapshot:
        vm, sb, bank, ctrl, now = view.vehicles, view.sensors, view.signals, view.controller, view.now_ms
        self.tracker.update(vm, now)
        vehicles, queue, outside = self._vehicles(vm, now)
        signals = {m: SignalView(m, bank.state(m), round(bank.since(m) * 1000),
                                 None if bank.remaining(m) is None else round(bank.remaining(m) * 1000))
                   for m in MOVES}
        faults = sb.faulted_detectors()
        sensors = {}
        for m in MOVES:
            s = sb.status(m)
            sensors[m] = SensorView(m, s.d1, s.d2, s.has_call, s.call_since_ms, s.qhat, s.level2_latched,
                                    s.in_service, s.gap_out, tuple(d for mm, d in faults if mm == m))
        smart = fixed = None
        if self.config.controller == "Smart":
            smart = self._smart(ctrl, sb, bank, now)
            kind = "Smart"
        else:
            fixed = self._fixed(ctrl, now)
            kind = "Fixed"
        return Snapshot(now, step, self.config, vehicles, signals, sensors, queue, outside,
                        self.tracker.metrics(vm, sb, bank, now), kind, smart, fixed, self._hardware())


# ======================================================== the session
class _Abort(Exception):
    """Raised inside the observer to end the run early (session stopped / reset)."""


class VizSession:
    """One run of RunConfig on a background thread, paced for display.

    start() launches it paused at the first instant (all heads RED). The UI then calls
    set_running / set_speed, reads snapshots with frame(), and stop() ends it (reset =
    stop + a new session). Nothing here changes what the simulation computes.
    """

    def __init__(self, config: RunConfig, *, hw_config=None, transport_factory=None,
                 arrivals=None, speed: Optional[float] = 1.0, running: bool = False,
                 clock: Callable[[], float] = time.monotonic, snapshot_every_ms: float = 30.0,
                 keep: int = 40):
        if speed not in SPEEDS:
            raise ValueError(f"speed must be one of {SPEEDS}")
        self.config = config
        self.hw_config = hw_config
        self.transport_factory = transport_factory
        self.arrivals = arrivals
        self.hardware = hw_config is not None
        self._clock = clock
        self._cond = threading.Condition()
        self._running = running
        self._abort = False
        self._speed = HARDWARE_SPEED if self.hardware else speed
        self._anchor: Optional[Tuple[float, int]] = None      # (wall s, sim ms)
        self._frozen_ms: Optional[float] = None
        self._snaps: Deque[Snapshot] = deque(maxlen=keep)
        self._throttle_s = snapshot_every_ms / 1000
        self._last_snap_wall = -1e9
        self._steps = 0
        self._builder: Optional[SnapshotBuilder] = None
        self._hw = None
        self._last_view: Optional[StepView] = None
        self.result: Optional[RunResult] = None
        self.error: Optional[str] = None
        self.finished = False
        self._thread: Optional[threading.Thread] = None
        self.dt_ms = config.dt_ms

    # ------------------------------------------------------------ control
    def start(self) -> "VizSession":
        self._thread = threading.Thread(target=self._main, name="sim", daemon=True)
        self._thread.start()
        return self

    @property
    def running(self) -> bool:
        return self._running

    @property
    def speed(self) -> Optional[float]:
        return self._speed

    def set_running(self, on: bool) -> None:
        with self._cond:
            if on == self._running:
                return
            now = self._clock()
            if on:
                self._anchor = (now, self._frozen_ms if self._frozen_ms is not None else self._newest_ms())
                self._frozen_ms = None
            else:
                self._frozen_ms = self._display_ms_locked(now)
            self._running = on
            self._cond.notify_all()

    def set_speed(self, speed: Optional[float]) -> None:
        if speed not in SPEEDS:
            raise ValueError(f"speed must be one of {SPEEDS}")
        if self.hardware:
            return                                    # real-time only with hardware
        with self._cond:
            now = self._clock()
            if self._running:
                self._anchor = (now, self._display_ms_locked(now))
            self._speed = speed
            self._cond.notify_all()

    def stop(self, timeout: float = 5.0) -> None:
        with self._cond:
            self._abort = True
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout)

    def wait_finished(self, timeout: Optional[float] = None) -> bool:
        if self._thread is not None:
            self._thread.join(timeout)
        return self.finished

    # ------------------------------------------------------------ display
    def _newest_ms(self) -> int:
        return self._snaps[-1].t_ms if self._snaps else self.config.window().start_ms

    def _display_ms_locked(self, now: float) -> float:
        if not self._snaps:
            return float(self.config.window().start_ms)
        newest = self._snaps[-1].t_ms
        if self._frozen_ms is not None:
            return min(self._frozen_ms, newest)
        if not self._running or self._anchor is None:
            return float(newest)
        if self._speed is None or self.finished:
            return float(newest)
        wall, sim = self._anchor
        d = sim + (now - wall) * self._speed * 1000
        oldest = self._snaps[0].t_ms
        return float(min(max(d, oldest), newest))

    def display_time(self) -> float:
        with self._cond:
            return self._display_ms_locked(self._clock())

    def frame(self) -> Tuple[Optional[Snapshot], Optional[Snapshot], float]:
        """(snapshot at or before the display time, the next one or None, display time ms)."""
        with self._cond:
            t = self._display_ms_locked(self._clock())
            if not self._snaps:
                return None, None, t
            prev = self._snaps[0]
            nxt = None
            for s in self._snaps:
                if s.t_ms <= t:
                    prev = s
                else:
                    nxt = s
                    break
            return prev, nxt, t

    def latest(self) -> Optional[Snapshot]:
        with self._cond:
            return self._snaps[-1] if self._snaps else None

    # ------------------------------------------------------------ simulation thread
    def _source_factory(self):
        if not self.hardware:
            return None
        from smarttraffic.hardware import build_hardware

        def factory(vm):
            self._hw = build_hardware(self.hw_config, vm, self.transport_factory)
            return self._hw.detectors
        return factory

    def _observe(self, view: StepView) -> None:
        if self._abort:
            raise _Abort()
        self._last_view = view
        if self._hw is not None and self._hw.mirror is not None:
            self._hw.mirror.update(view.signals)       # read-only on the SignalBank
        if self._builder is None:
            self._builder = SnapshotBuilder(self.config, self._hw, self.hw_config)
        first = self._steps == 0
        self._steps += 1
        wall = self._clock()
        paced = self._speed is not None
        if first or paced or wall - self._last_snap_wall >= self._throttle_s:
            snap = self._builder.build(view, self._steps)
            self._last_snap_wall = wall
            with self._cond:
                self._snaps.append(snap)
        else:
            self._builder.tracker.update(view.vehicles, view.now_ms)   # keep metrics exact
        self._pace(view.now_ms)

    def _pace(self, t_ms: int) -> None:
        with self._cond:
            while True:
                if self._abort:
                    raise _Abort()
                if not self._running:
                    self._cond.wait(0.1)
                    continue
                if self._speed is None:
                    return
                now = self._clock()
                if self._anchor is None:
                    self._anchor = (now, t_ms)
                wall, sim = self._anchor
                # compute the next step once the display reaches t - dt: the snapshot after
                # it is ready before the display needs it (smooth interpolation)
                due = wall + (t_ms - self.dt_ms - sim) / 1000 / self._speed
                late = now - due
                if late >= 0:
                    if late > 2.0:                          # fell far behind: re-anchor, no burst
                        self._anchor = (now, t_ms)
                    return
                self._cond.wait(min(-late, 0.05))

    def _main(self) -> None:
        try:
            kw = {"observers": [self._observe]}
            sf = self._source_factory()
            if sf is not None:
                kw["source_factory"] = sf
            if self.arrivals is not None:
                kw["arrivals"] = self.arrivals
            if self.hw_config is not None:
                # Hardware mode (fixed for the whole session, connected or not): movements whose
                # detectors are all real get NO virtual vehicles; their demand comes only from the
                # physical sensors. Other movements keep the full scenario arrivals.
                kw["exclude_movements"] = self.hw_config.real_only_movements
            self.result = run(self.config, **kw)
            if self._last_view is not None:              # the run is over: nothing moves any more
                snap = self._builder.build(self._last_view, self._steps)
                with self._cond:
                    if not self._snaps or self._snaps[-1].t_ms != snap.t_ms:
                        self._snaps.append(snap)
            self.finished = True
        except _Abort:
            pass
        except Exception as e:                              # shown in the UI, never a crash
            self.error = f"{type(e).__name__}: {e}"
        finally:
            if self._hw is not None:
                try:
                    self._hw.close()
                except Exception:
                    pass
            with self._cond:
                self._cond.notify_all()
