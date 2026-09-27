"""Stage 13: visualization (viz_geometry, viz_session, viz). No pixel comparisons.

What is checked: the drawn model is the project's model (8 heads = 8 movements, two per
approach, no right turn; 16 detector positions, a real detector replaces its virtual
slot), the display state is SignalBank's, the visualization never changes a run
(RunResult JSON identical with and without it, also with pauses and at every speed),
pause/resume/reset semantics, Fixed and Smart both render, a missing ESP32 does not
crash anything. Rendering tests draw into an off-screen surface (SDL dummy driver).
"""
import ast
import math
import os
from pathlib import Path
import threading
import time

import pytest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from smarttraffic import esp_protocol as proto
from smarttraffic import viz_geometry as g
from smarttraffic.conflicts import MOVES, compatible
from smarttraffic.hardware import load_hardware_config, parse_hardware_config
from smarttraffic.runner import RunConfig, run, simulate
from smarttraffic.sensors import DETECTOR_POSITION, SENSOR_IDS
from smarttraffic.signals import FLASH_ON_MS, FLASH_PERIOD_MS, SignalState, lamps_for, movement_type
from smarttraffic.traffic_gen import generate_arrivals
from smarttraffic.vehicles import BOX_LENGTH, VEHICLE_LENGTH
from smarttraffic.viz_session import SPEEDS, HARDWARE_SPEED, SnapshotBuilder, VizSession

ROOT = Path(__file__).resolve().parent.parent
VIZ_FILES = ["smarttraffic/viz.py", "smarttraffic/viz_render.py", "smarttraffic/viz_session.py",
             "smarttraffic/viz_geometry.py"]


def cfg(sc="S1", seed=1, ctrl="Smart", warm=20, meas=120, flush=120):
    return RunConfig(sc, seed, ctrl, warmup_s=warm, measure_s=meas, flush_s=flush)


def arr(c: RunConfig):
    return generate_arrivals(c.scenario, c.seed, -c.warmup_s, c.measure_s)


def reference(c: RunConfig, **kw):
    return run(c, arrivals=arr(c), **kw).to_json()


def run_session(c: RunConfig, speed=None, **kw) -> VizSession:
    s = VizSession(c, arrivals=arr(c), speed=speed, running=True, **kw).start()
    assert s.wait_finished(120), "session did not finish"
    return s


# ================================================================ geometry: 8 heads
def test_exactly_eight_signal_heads_one_per_movement():
    heads = g.signal_heads()
    assert len(heads) == 8
    assert [h.movement for h in heads] == list(MOVES)
    assert len({h.movement for h in heads}) == 8


def test_every_approach_has_separate_straight_and_left_heads():
    heads = g.signal_heads()
    for a in g.APPROACHES:
        mine = [h for h in heads if h.approach == a]
        assert len(mine) == 2
        assert sorted(h.kind for h in mine) == ["left", "straight"]
        assert {h.movement for h in mine} == {a + "S", a + "L"}
        assert {h.slot for h in mine} == {0, 1}              # two separate heads, not one
        assert len({h.corner for h in mine}) == 1            # on the same kerb corner


def test_head_mapping_and_no_right_turn():
    for h in g.signal_heads():
        assert h.kind == movement_type(h.movement)
        assert g.movement_of(h.approach, h.kind) == h.movement
        assert h.kind in ("straight", "left")
    assert not any(m.endswith("R") for m in MOVES)
    with pytest.raises(ValueError):
        g.movement_of("N", "right")


def test_approach_corners_are_distinct_and_on_the_drivers_right():
    heads = g.signal_heads()
    corners = {h.approach: h.corner for h in heads}
    assert len(set(corners.values())) == 4
    for h in heads:
        # the corner is at the stop line (HALF upstream of the centre) on the outward side
        assert math.isclose(h.corner[0] * h.outward[0] + h.corner[1] * h.outward[1], g.HALF)
        assert math.isclose(h.corner[0] * h.travel[0] + h.corner[1] * h.travel[1], -g.HALF)


# ================================================================ geometry: 16 detectors
def test_exactly_sixteen_detector_positions():
    spots = g.detector_spots()
    assert len(spots) == 16
    assert [(d.movement, d.detector) for d in spots] == list(SENSOR_IDS)
    assert not any(d.real for d in spots)
    for d in spots:
        assert d.upstream_m == DETECTOR_POSITION[d.detector]


def test_real_detector_replaces_its_virtual_slot_not_a_seventeenth():
    real = tuple(load_hardware_config().sensors)
    assert real == (("NS", "D1"),)
    spots = g.detector_spots(real)
    assert len(spots) == 16
    assert [(d.movement, d.detector) for d in spots if d.real] == [("NS", "D1")]
    assert sum(not d.real for d in spots) == 15
    plain = {(d.movement, d.detector): d.centre for d in g.detector_spots()}
    assert {(d.movement, d.detector): d.centre for d in spots} == plain     # same place
    two = g.detector_spots([("NS", "D1"), ("EL", "D2")])
    assert len(two) == 16 and sum(d.real for d in two) == 2
    with pytest.raises(ValueError):
        g.detector_spots([("NR", "D1")])


def test_detectors_lie_in_their_own_lane():
    for d in g.detector_spots():
        assert d.centre == pytest.approx(g.path_point(d.movement, -d.upstream_m))
        lane = g.lanes()[MOVES.index(d.movement)]
        assert lane.movement == d.movement


# ================================================================ geometry: paths
def test_paths_follow_right_hand_traffic_and_the_right_exit_arm():
    for m in MOVES:
        a, kind = m[0], movement_type(m)
        box = BOX_LENGTH[kind]
        start, end = g.path_point(m, 0.0), g.path_point(m, box)
        # stop line on the approach side, driver's right half
        t, r = g.travel_dir(a), g.right_dir(a)
        assert math.isclose(start[0] * t[0] + start[1] * t[1], -g.HALF)
        assert start[0] * r[0] + start[1] * r[1] > 0                   # right-hand traffic
        exit_arm = g.straight_exit_arm(a) if kind == "straight" else g.left_turn_exit_arm(a)
        out = g.travel_dir(exit_arm)                                   # vehicles LEAVE this arm
        leave = (-out[0], -out[1])
        assert math.isclose(end[0] * leave[0] + end[1] * leave[1], g.HALF, abs_tol=1e-9)
        h = g.path_heading(m, box + 1)
        assert h == pytest.approx(leave)
    assert g.left_turn_exit_arm("N") == "E"          # southbound turning left goes east
    assert g.straight_exit_arm("N") == "S"


def test_paths_are_continuous_and_headings_are_unit():
    for m in MOVES:
        box = BOX_LENGTH[movement_type(m)]
        prev = g.path_point(m, -40.0)
        for i in range(1, 900):
            u = -40.0 + i * 0.1
            p = g.path_point(m, u)
            assert math.dist(prev, p) < 0.2
            prev = p
            hx, hy = g.path_heading(m, u)
            assert math.isclose(math.hypot(hx, hy), 1.0, abs_tol=1e-9)
        assert g.path_point(m, 0.0) == pytest.approx(g.path_point(m, -1e-9))
        assert g.path_point(m, box) == pytest.approx(g.path_point(m, box + 1e-9), abs=1e-6)


def test_compatible_movements_never_touch_inside_the_box():
    """Two compatible movements (which the controllers may run together) keep more than a
    vehicle width between their paths, so their vehicles never overlap on screen."""
    samples = {m: [g.path_point(m, BOX_LENGTH[movement_type(m)] * i / 60) for i in range(61)] for m in MOVES}
    for i, a in enumerate(MOVES):
        for b in MOVES[i + 1:]:
            if not compatible(a, b):
                continue
            d = min(math.dist(p, q) for p in samples[a] for q in samples[b])
            assert d > g.VEHICLE_WIDTH + 1.5, (a, b, d)


def test_each_exit_lane_is_used_by_one_movement():
    ends = {m: tuple(round(c, 6) for c in g.path_point(m, BOX_LENGTH[movement_type(m)] + 10)) for m in MOVES}
    assert len(set(ends.values())) == 8


def test_vehicle_pose_spans_its_length_behind_the_front():
    for m in MOVES:
        c, h = g.vehicle_pose(m, -10.0)
        front = g.path_point(m, -10.0)
        assert math.dist(c, front) == pytest.approx(VEHICLE_LENGTH / 2)


# ================================================================ snapshots mirror the model
def test_snapshot_signals_are_signalbank_state_every_step():
    c = cfg(meas=180)
    b = SnapshotBuilder(c)
    seen = set()
    n = {"steps": 0}

    def obs(view):
        n["steps"] += 1
        s = b.build(view, n["steps"])
        for m in MOVES:
            assert s.signals[m].state is view.signals.state(m)
            assert s.lamps(m) == view.signals.lamp_state(m)
            seen.add(s.signals[m].state)
        assert s.t_ms == view.now_ms
    simulate(c, arrivals=arr(c), observers=[obs], collect_stats=False)
    assert {SignalState.RED, SignalState.RED_YELLOW, SignalState.GREEN, SignalState.FLASHING_GREEN,
            SignalState.YELLOW, SignalState.ALL_RED} <= seen


def test_flashing_green_lamps_follow_lamps_for_between_steps():
    from smarttraffic.viz import capture
    c = cfg(meas=180)
    b = SnapshotBuilder(c)
    found = {}

    def obs(view):
        for m in MOVES:
            if view.signals.state(m) is SignalState.FLASHING_GREEN and m not in found:
                found[m] = b.build(view, 0)
    simulate(c, arrivals=arr(c), observers=[obs], collect_stats=False)
    assert found
    for m, s in found.items():
        since = s.signals[m].since_ms
        for dt in range(0, 3000, 50):
            on = s.lamps(m, since + dt).green
            assert on == (dt % FLASH_PERIOD_MS < FLASH_ON_MS)
            assert s.lamps(m, since + dt) == lamps_for(SignalState.FLASHING_GREEN, dt)


def test_snapshot_vehicles_detectors_and_queue_match_the_model():
    c = cfg("S7b", meas=240)
    b = SnapshotBuilder(c)
    n = {"k": 0}

    def obs(view):
        n["k"] += 1
        if n["k"] % 25:
            return
        s = b.build(view, n["k"])
        vm, sb = view.vehicles, view.sensors
        for m in MOVES:
            assert s.queue[m] == vm.queue_length(m)
            assert s.outside[m] == len(vm.waiting_outside(m))
            assert s.sensors[m].d1 == sb.d1(m) and s.sensors[m].d2 == sb.d2(m)
            assert s.sensors[m].call == sb.has_call(m) and s.sensors[m].qhat == sb.qhat(m)
            ids = {v.id for v in s.vehicles if v.movement == m and v.state != "EXITED"}
            assert ids == {v.id for v in vm.vehicles(m)} | {v.id for v in vm.in_intersection(m)}
            # no two vehicles of a lane overlap on screen (path distance >= one car length)
            us = sorted(v.u for v in s.vehicles if v.movement == m)
            assert all(b2 - a2 >= VEHICLE_LENGTH - 1e-6 for a2, b2 in zip(us, us[1:]))
    simulate(c, arrivals=arr(c), observers=[obs], collect_stats=False)
    assert n["k"] > 100


def test_live_metrics_agree_with_stage12_statistics():
    c = cfg(meas=180)
    ref = run(c, arrivals=arr(c))
    s = run_session(c)
    m = s.latest().metrics
    assert m.avg_delay_s == pytest.approx(ref.intersection.delay_mean_s, abs=1e-9)
    assert m.delayed_n == sum(1 for v in ref.vehicles if v.measured and v.cross_ms is not None)
    # queue metric = Stage 12 queue definition at the 1 s sample instants
    from smarttraffic.viz import capture
    times = [10_000, 60_000, 150_000]
    snaps = capture(c, times, arrivals=arr(c))
    samples = {t: r + o for t, r, o in ((t, sum(r), sum(o)) for t, r, o in ref.queue_samples)}
    for t in times:
        assert snaps[t][0].metrics.queue == samples[t]


# ================================================================ visualization on == off
@pytest.mark.parametrize("ctrl", ["FT-A", "FT-B", "Smart"])
def test_visualization_does_not_change_the_run(ctrl):
    c = cfg(ctrl=ctrl)
    s = run_session(c)
    assert s.error is None and s.finished
    assert s.result.to_json() == reference(c)


def test_ft_tod_is_visualised_and_unchanged():
    c = cfg("S6", ctrl="FT-TOD")
    s = run_session(c)
    assert s.latest().fixed is not None and s.latest().fixed.schedule.startswith("FT-TOD")
    assert s.result.to_json() == reference(c)


def test_pauses_at_random_moments_do_not_change_the_run():
    c = cfg(ctrl="Smart", meas=240)
    ref = reference(c)
    s = VizSession(c, arrivals=arr(c), speed=None, running=True).start()
    for k in range(6):
        time.sleep(0.05 + 0.03 * k)
        s.set_running(False)
        time.sleep(0.05)
        s.set_running(True)
    assert s.wait_finished(120)
    assert s.result.to_json() == ref


def test_speed_changes_do_not_change_the_run():
    c = cfg(ctrl="FT-A", warm=0, meas=30, flush=60)
    fast = lambda: time.monotonic() * 50            # 50x wall clock: paced but quick
    s = VizSession(c, arrivals=arr(c), speed=10.0, running=True, clock=fast).start()
    for sp in (0.5, 2.0, None, 5.0, 1.0, None):
        time.sleep(0.05)
        s.set_speed(sp)
    assert s.wait_finished(120)
    assert s.result.to_json() == reference(c)


def test_visualised_runs_are_deterministic():
    c = cfg("S5", ctrl="Smart")
    assert run_session(c).result.to_json() == run_session(c).result.to_json()


def test_observer_is_read_only_on_every_step():
    """Building a snapshot after EVERY step (the heaviest use) leaves the run identical."""
    c = cfg(ctrl="Smart")
    b = SnapshotBuilder(c)
    k = {"n": 0}

    def obs(view):
        k["n"] += 1
        b.build(view, k["n"])
    assert run(c, arrivals=arr(c), observers=[obs]).to_json() == reference(c)


def test_viz_code_never_commands_or_advances_the_simulation():
    forbidden = {"start_green", "end_green", "advance", "update"}
    for f in VIZ_FILES:
        tree = ast.parse((ROOT / f).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                name = node.func.attr
                if name in forbidden:
                    # the only allowed call is SignalMirror.update (hardware output, read-only
                    # on the SignalBank) and the metric tracker's own update
                    src = ast.unparse(node.func.value)
                    assert name == "update" and ("mirror" in src or "tracker" in src), (f, ast.unparse(node))


# ================================================================ pause / resume / reset
class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _settle(s, timeout=10.0):
    last, since = None, time.monotonic()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        snap = s.latest()
        cur = snap.step if snap else None
        if cur != last:
            last, since = cur, time.monotonic()
        elif time.monotonic() - since > 0.3:
            return snap
        time.sleep(0.02)
    raise AssertionError("session did not settle")


def test_session_starts_paused_at_the_start_all_red():
    c = cfg()
    s = VizSession(c, arrivals=arr(c)).start()
    try:
        snap = _settle(s)
        assert snap.t_ms == c.window().start_ms and snap.step == 1
        assert all(v.state is SignalState.RED for v in snap.signals.values())
        assert not s.running and s.display_time() == c.window().start_ms
    finally:
        s.stop()


def test_pacing_follows_the_clock_and_pause_freezes_time():
    c = cfg()
    clk = FakeClock()
    s = VizSession(c, arrivals=arr(c), speed=1.0, clock=clk).start()
    try:
        _settle(s)
        s.set_running(True)
        start = c.window().start_ms
        snap = _settle(s)
        assert snap.t_ms == start + 2 * c.dt_ms          # one step ahead of the display
        clk.t = 1.0
        snap = _settle(s)
        assert s.display_time() == start + 1000
        assert snap.t_ms == start + 1000 + 2 * c.dt_ms
        clk.t = 1.25
        _settle(s)
        assert s.display_time() == start + 1250          # smooth display between snapshots
        a, b, t = s.frame()
        assert a.t_ms <= t < b.t_ms
        # pause: nothing advances, whatever the clock does
        s.set_running(False)
        frozen = s.display_time()
        step = _settle(s).step
        clk.t = 50.0
        time.sleep(0.3)
        assert s.display_time() == frozen and s.latest().step == step
        # resume: the display continues from where it stopped (no jump)
        s.set_running(True)
        assert s.display_time() == frozen
        clk.t = 52.0
        _settle(s)
        assert s.display_time() == pytest.approx(frozen + 2000)
        # speed 5x
        s.set_speed(5.0)
        d0 = s.display_time()
        clk.t = 53.0
        _settle(s)
        assert s.display_time() == pytest.approx(d0 + 5000)
    finally:
        s.stop()


def test_stop_ends_the_thread_promptly_even_while_paused():
    c = cfg()
    s = VizSession(c, arrivals=arr(c)).start()
    _settle(s)
    t0 = time.monotonic()
    s.stop()
    assert time.monotonic() - t0 < 2.0
    assert not s._thread.is_alive()
    assert s.result is None and s.error is None and not s.finished


def _state(c, **kw):
    from smarttraffic.viz import UIState
    return UIState(c, session_factory=lambda cc, **k: VizSession(cc, arrivals=arr(cc), **k), **kw)


def test_reset_restarts_from_the_beginning():
    c = cfg()
    st = _state(c, speed=None)
    try:
        st.toggle_run()
        time.sleep(0.4)
        old = st.session
        assert old.latest().t_ms > c.window().start_ms
        st.reset()
        assert not old._thread.is_alive()
        snap = _settle(st.session)
        assert snap.t_ms == c.window().start_ms and snap.step == 1
        assert all(v.state is SignalState.RED for v in snap.signals.values())
        assert snap.metrics.crossed == 0 and not st.running
    finally:
        st.close()


def test_changing_controller_or_scenario_starts_a_new_run():
    c = cfg()
    st = _state(c, speed=None)
    try:
        st.set_controller("FT-A")
        assert st.config.controller == "FT-A" and _settle(st.session).fixed is not None
        st.set_controller("Smart")
        assert _settle(st.session).smart is not None
        st.set_controller("FT-TOD")                      # no FT-TOD plan for S1: ignored
        assert st.config.controller == "Smart"
        st.next_scenario()
        assert st.config.scenario == "S2"
        st.next_seed()
        assert st.config.seed == 2
    finally:
        st.close()


def test_speed_options():
    assert SPEEDS == (0.5, 1.0, 2.0, 5.0, 10.0, None)
    with pytest.raises(ValueError):
        VizSession(cfg(), speed=3.0)


# ================================================================ hardware
class DeadPort:
    """A serial port that is not there (ESP32 unplugged / wrong COM port)."""
    is_open = False

    def open(self):
        raise OSError("could not open port 'COM3'")

    def close(self):
        pass

    def read_available(self):
        return b""

    def write(self, data):
        raise OSError("not open")


class LiveEsp:
    """A fake ESP32 that answers like the firmware: HELLO on open, heartbeats, ACKs; the
    IR detector is occupied between device times 4000 and 6000 ms."""

    def __init__(self):
        self.is_open = False
        self.ms = 0
        self.seq = 0
        self.out = bytearray()
        self.written = []

    def _s(self):
        self.seq += 1
        return self.seq - 1

    def open(self):
        self.is_open = True
        self.out += proto.encode_hello(self._s(), self.ms, "esp32_a", "0000BEEF", 2, 2)
        self.out += proto.encode_sensor(self._s(), self.ms, 0, False)
        self.out += proto.encode_sensor(self._s(), self.ms, 1, False)

    def close(self):
        self.is_open = False

    def read_available(self):
        self.ms += 100
        if self.ms == 4000:
            self.out += proto.encode_sensor(self._s(), self.ms, 0, True)
        if self.ms == 6000:
            self.out += proto.encode_sensor(self._s(), self.ms, 0, False)
        self.out += proto.encode_heartbeat(self._s(), self.ms)
        data, self.out = bytes(self.out), bytearray()
        return data

    def write(self, data):
        self.written.append(data)
        f = proto.decode_frame(data)
        if f and f[0] == "SIG":
            self.out += proto.encode_ack(self._s(), self.ms, int(f[1]), f[2])


def test_hardware_disconnected_session_runs_and_reports_the_fault():
    c = cfg(warm=0, meas=30, flush=30)
    hw = load_hardware_config()
    fast = lambda: time.monotonic() * 200
    s = VizSession(c, arrivals=arr(c), hw_config=hw, transport_factory=lambda d: DeadPort(),
                   running=True, clock=fast).start()
    assert s.speed == HARDWARE_SPEED
    s.set_speed(None)                                    # ignored: hardware is real-time only
    assert s.speed == HARDWARE_SPEED
    assert s.wait_finished(60)
    assert s.error is None
    snap = s.latest()
    h = snap.hardware
    assert h.enabled and h.links == (("esp32_a", "COM3", "DISCONNECTED"),)
    assert h.real == (("NS:D1", False, True),)           # faulted, state held
    assert h.real_ids == (("NS", "D1"),)
    assert snap.sensors["NS"].faulted == ("D1",)
    assert "NS" in snap.smart.recall                     # Stage 11 fail-safe recall
    assert h.mirror[0][0] == "NS" and h.mirror[0][3] == "DISCONNECTED"


def test_hardware_connected_real_detector_and_mirror_are_shown():
    from smarttraffic.viz import capture
    c = cfg(warm=0, meas=30, flush=30)
    hw = load_hardware_config()
    port = LiveEsp()
    snaps = capture(c, [5000, 12000], hw_config=hw, transport_factory=lambda d: port, arrivals=arr(c))
    h5 = snaps[5000][0].hardware
    assert h5.links == (("esp32_a", "COM3", "CONNECTED"),)
    assert h5.real == (("NS:D1", True, False),)          # the real IR is occupied
    assert snaps[5000][0].sensors["NS"].d1 is True       # ... and SensorBank sees it
    assert snaps[12000][0].hardware.real == (("NS:D1", False, False),)
    m, sent, acked, link = snaps[12000][0].hardware.mirror[0]
    assert m == "NS" and link == "CONNECTED"
    assert sent == snaps[12000][0].signals["NS"].state.name
    assert any(proto.decode_frame(w)[0] == "SIG" for w in port.written)


def test_hardware_mode_does_not_change_the_run():
    """Same fake device with and without the visualization: identical RunResult."""
    from smarttraffic.hardware import build_hardware
    c = cfg(warm=0, meas=30, flush=30)
    hw = load_hardware_config()
    box = {}

    def factory(vm):
        box["hw"] = build_hardware(hw, vm, lambda d: LiveEsp())
        return box["hw"].detectors

    def mirror(view):
        box["hw"].mirror.update(view.signals)
    ref = run(c, arrivals=arr(c), source_factory=factory, observers=[mirror]).to_json()
    fast = lambda: time.monotonic() * 500
    s = VizSession(c, arrivals=arr(c), hw_config=hw, transport_factory=lambda d: LiveEsp(),
                   running=True, clock=fast).start()
    assert s.wait_finished(60) and s.error is None
    assert s.result.to_json() == ref


def test_bad_hardware_config_falls_back_without_crashing(tmp_path):
    bad = tmp_path / "hw.json"
    bad.write_text("{not json", encoding="utf-8")
    c = cfg()
    from smarttraffic.viz import UIState
    st = UIState(c, hardware=True, hw_path=str(bad),
                 session_factory=lambda cc, **k: VizSession(cc, arrivals=arr(cc), **k))
    try:
        assert not st.hardware and st.message and "could not be loaded" in st.message
        assert st.session is not None and st.session.hw_config is None
    finally:
        st.close()


# ================================================================ rendering (off-screen)
pygame = pytest.importorskip("pygame")


@pytest.fixture(scope="module")
def screen():
    pygame.display.init()
    pygame.font.init()
    yield pygame.display.set_mode((1600, 900))
    pygame.display.quit()


def _render(screen, c, t_ms, hw=None, tf=None, size=None):
    from smarttraffic.viz import Fonts, Renderer, _StillState, capture
    snaps = capture(c, [t_ms], hw_config=hw, transport_factory=tf, arrivals=arr(c))
    a, b = snaps[t_ms]
    r = Renderer(Fonts())
    surf = pygame.Surface(size or (1600, 900))
    for frac in (0.0, 0.5, 0.99):
        r.draw(surf, _StillState(c, hardware=hw is not None), a, b,
               a.t_ms + frac * ((b.t_ms - a.t_ms) if b else 0))
    return r, a


@pytest.mark.parametrize("ctrl", ["FT-A", "FT-B", "Smart"])
def test_fixed_and_smart_render(screen, ctrl):
    c = cfg(ctrl=ctrl, meas=240)
    r, a = _render(screen, c, 200_000)
    names = {n for _, _, n in r.buttons}
    assert {"FT-A", "FT-B", "Smart", "run", "reset", "hardware", "scenario+", "seed+"} <= names
    assert {f"speed:{x}" for x in ("0.5×", "1×", "2×", "5×", "10×", "Max")} <= names
    assert (a.smart is not None) == (ctrl == "Smart") and (a.fixed is not None) == (ctrl != "Smart")


def test_render_at_laptop_and_full_hd_sizes(screen):
    c = cfg(meas=240)
    for size in ((1280, 720), (1366, 768), (1920, 1080), (2560, 1440)):
        _render(screen, c, 150_000, size=size)


def test_render_hardware_disconnected(screen):
    c = cfg(warm=0, meas=30, flush=30)
    r, a = _render(screen, c, 10_000, hw=load_hardware_config(), tf=lambda d: DeadPort())
    assert a.hardware.links[0][2] == "DISCONNECTED"


def test_ft_tod_button_only_for_s6(screen):
    from smarttraffic.viz import controllers_for
    assert controllers_for("S1") == ("FT-A", "FT-B", "Smart")
    assert controllers_for("S6") == ("FT-A", "FT-B", "FT-TOD", "Smart")
    r, _ = _render(screen, cfg("S6", ctrl="FT-TOD", meas=240), 100_000)
    assert "FT-TOD" in {n for _, _, n in r.buttons}


def test_app_frames_and_clicks(screen):
    from smarttraffic.viz import App
    st = _state(cfg(), speed=10.0)
    app = App(st, size=(1400, 820))
    try:
        assert app.frame()
        buttons = {n: rect for rect, _, n in app.r.buttons}
        app.click(buttons["run"].center)
        assert st.running
        for _ in range(5):
            app.frame()
        app.click(buttons["FT-A"].center)
        assert st.config.controller == "FT-A"
        app.frame()
        app.click(buttons["speed:Max"].center)
        assert st.speed is None
        assert app.handle_key(pygame.K_SPACE) and not st.running
        assert app.handle_key(pygame.K_r)
        assert not app.handle_key(pygame.K_ESCAPE)
    finally:
        st.close()


def test_screenshot_cli(tmp_path, monkeypatch):
    from smarttraffic import viz
    out = tmp_path / "frame.png"
    c = RunConfig("S1", 1, "Smart")
    a = viz.render_to_file(str(out), c, 60.0, (1280, 720), arrivals=arr(c))
    assert out.exists() and out.stat().st_size > 10_000
    assert a.t_ms == 60_000


def test_all_eight_heads_are_vertical_red_top_yellow_middle_green_bottom():
    from smarttraffic.viz import Fonts, Renderer, View
    r = Renderer(Fonts())
    for size in ((1600, 900), (1920, 1080), (1366, 768)):
        k, _, canvas, _ = r.layout(size)
        r.k = k
        layout = r.head_layout(View(canvas.inflate(-round(20 * k), -round(20 * k))))
        assert sorted(layout) == sorted(MOVES)                      # exactly the 8 movements
        for m, (rect, lamps) in layout.items():
            assert [n for n, _ in lamps] == ["red", "yellow", "green"], m
            ys = [c[1] for _, c in lamps]
            assert ys[0] < ys[1] < ys[2], m                           # top -> bottom
            assert len({c[0] for _, c in lamps}) == 1, m              # one vertical column
            assert rect.h > rect.w, m                                 # upright housing
        for a in "NESW":                                              # two separate heads side by side
            s, l = layout[a + "S"][0], layout[a + "L"][0]
            assert not s.colliderect(l) and s.y == l.y


def test_displayed_lamps_for_every_signal_state():
    on = lambda lp: [n for n, v in zip(("red", "yellow", "green"), lp) if v]
    assert on(lamps_for(SignalState.RED, 0)) == ["red"]
    assert on(lamps_for(SignalState.ALL_RED, 0)) == ["red"]
    assert on(lamps_for(SignalState.RED_YELLOW, 0)) == ["red", "yellow"]
    assert on(lamps_for(SignalState.GREEN, 0)) == ["green"]
    assert on(lamps_for(SignalState.YELLOW, 0)) == ["yellow"]
    assert on(lamps_for(SignalState.FLASHING_GREEN, 0)) == ["green"]
    assert on(lamps_for(SignalState.FLASHING_GREEN, FLASH_ON_MS)) == []


def test_lens_arrows_point_the_way_each_movement_drives():
    """Screen y grows downwards: N approach drives down (southbound), S up, E left, W right.
    Left-turn arrows start along that direction and end pointing to the driver's left."""
    from smarttraffic.viz_render import lens_arrow
    down, up, left, right = (0, 1), (0, -1), (-1, 0), (1, 0)
    expect = {"NS": (down, down), "SS": (up, up), "ES": (left, left), "WS": (right, right),
              "NL": (down, right), "SL": (up, left), "EL": (left, down), "WL": (right, up)}
    unit = lambda v: (round(v[0] / math.hypot(*v), 6) + 0.0, round(v[1] / math.hypot(*v), 6) + 0.0)
    for h in g.signal_heads():
        shaft, head = lens_arrow(h.kind, h.travel, (-h.outward[0], -h.outward[1]))
        start = unit((shaft[1][0] - shaft[0][0], shaft[1][1] - shaft[0][1]))
        base = ((head[1][0] + head[2][0]) / 2, (head[1][1] + head[2][1]) / 2)
        tip = unit((head[0][0] - base[0], head[0][1] - base[1]))
        assert (start, tip) == expect[h.movement], h.movement


def test_heads_are_three_equal_stacked_modules_and_left_arrows_turn_at_right_angles():
    from smarttraffic.viz import Fonts, Renderer, View
    from smarttraffic.viz_render import lens_arrow
    r = Renderer(Fonts())
    k, _, canvas, _ = r.layout((1920, 1080))
    r.k = k
    for m, (rect, lamps) in r.head_layout(View(canvas.inflate(-round(20 * k), -round(20 * k)))).items():
        ys = [c[1] for _, c in lamps]
        step = rect.h / 3
        assert ys[1] - ys[0] == pytest.approx(step, abs=1) and ys[2] - ys[1] == pytest.approx(step, abs=1)
        assert all(rect.collidepoint(round(c[0]), round(c[1])) for _, c in lamps), m
    for h in g.signal_heads():
        shaft, _ = lens_arrow(h.kind, h.travel, (-h.outward[0], -h.outward[1]))
        if h.kind == "left":
            (a, b, c) = shaft
            d1 = (b[0] - a[0], b[1] - a[1])
            d2 = (c[0] - b[0], c[1] - b[1])
            assert abs(d1[0] * d2[0] + d1[1] * d2[1]) < 1e-9, h.movement      # 90 degree corner
        else:
            assert len(shaft) == 2
