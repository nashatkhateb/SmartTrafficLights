"""Hardware demo: the real movement (NS, with NS:D1 and NS:D2 real) gets real demand only.

Rule (hardware.real_only_movements + runner.simulate(exclude_movements=...)):
  * hardware mode on  -> no virtual NS vehicle is generated, drawn or detected; NS:D1 / NS:D2
                         change only from the physical IR sensors; the 14 other detectors and the
                         7 other movements stay fully virtual (same arrivals as without hardware).
  * hardware mode off -> exactly the previous virtual-only run (no parameter, no metadata key).
  * disconnect        -> NS:D1 / NS:D2 faulted, last state held, never virtual; NS still gets no
                         virtual vehicles; Smart serves NS by its existing fail-safe recall;
                         reconnect re-syncs from the HELLO + full-state report. Deterministic.
"""
import time

from smarttraffic import esp_sim
from smarttraffic.conflicts import MOVES
from smarttraffic.hardware import build_hardware, load_hardware_config, real_only_movements, parse_hardware_config
from smarttraffic.runner import RunConfig, run, simulate
from smarttraffic.traffic_gen import generate_arrivals
from smarttraffic.viz_session import NO_HARDWARE, VizSession

NS_DET = {("NS", "D1"), ("NS", "D2")}


def cfg(sc="S2", seed=1, ctrl="Smart", warm=0, meas=90, flush=60):
    return RunConfig(sc, seed, ctrl, warmup_s=warm, measure_s=meas, flush_s=flush)


def arr(c):
    return generate_arrivals(c.scenario, c.seed, -c.warmup_s, c.measure_s)


class Rig:
    """runner.simulate with the default hardware config on a SimulatedEsp32. `script` maps a
    simulation time (ms) to a callable(esp) applied at the end of that step (car on/off,
    unplug/plug). The board runs in device time = simulation time."""

    def __init__(self, c, script=None, esp=None, exclude=True):
        self.c, self.script = c, dict(script or {})
        self.esp = esp or esp_sim.SimulatedEsp32()
        self.hwcfg = load_hardware_config()
        self.exclude = exclude
        self.trace = []            # (ms, NS vehicles present, NS d1, NS d2, NS faulted, NS on recall)

    def factory(self, vm):
        self.hw = build_hardware(self.hwcfg, vm, lambda d: self.esp)
        return self.hw.detectors

    def observe(self, view):
        self.esp.advance_to(view.now_ms)
        for t in sorted(k for k in self.script if k <= view.now_ms):
            self.script.pop(t)(self.esp)
        self.hw.mirror.update(view.signals)
        sb, ctrl = view.sensors, view.controller
        self.trace.append((view.now_ms, len(view.vehicles.vehicles("NS")), sb.d1("NS"), sb.d2("NS"),
                           ("NS", "D1") in sb.faulted_detectors(), "NS" in ctrl.faulted_movements()))
        self.vm, self.sb = view.vehicles, view.sensors

    def run(self):
        kw = {"exclude_movements": real_only_movements(self.hwcfg)} if self.exclude else {}
        self.result = run(self.c, arrivals=arr(self.c), source_factory=self.factory,
                          observers=[self.observe], **kw)
        return self


# ------------------------------------------------------------------ the rule itself
def test_default_config_makes_ns_the_only_real_only_movement():
    hw = load_hardware_config()
    assert real_only_movements(hw) == ("NS",) == hw.real_only_movements
    assert len(hw.sensors) == 2                              # 2 real + 14 virtual, still 16 logical
    only_d1 = parse_hardware_config({"devices": {"esp32_a": {"port": "COM3"}},
                                     "sensors": {"SS:D1": {"device": "esp32_a", "channel": 0}}})
    assert real_only_movements(only_d1) == ()                # partly real -> keeps virtual vehicles
    assert real_only_movements(parse_hardware_config({"devices": {"esp32_a": {"port": "COM3"}}})) == ()


# ------------------------------------------------------------------ 1. no virtual NS vehicles
def test_hardware_connected_spawns_no_virtual_ns_vehicles():
    c = cfg()
    assert any(a.movement == "NS" for a in arr(c).arrivals)  # the scenario HAS NS traffic ...
    rig = Rig(c).run()
    assert rig.vm.vehicles("NS") == () and rig.vm.completed("NS") == ()          # ... none generated
    assert all(n == 0 for _, n, *_ in rig.trace)
    assert not any(v.movement == "NS" for v in rig.vm.all_vehicles())
    assert rig.result.to_json().count('"virtual_arrivals_excluded":["NS"]') == 1
    assert rig.hw.hub.link("esp32_a").healthy()


def test_visualization_draws_no_ns_vehicle_and_labels_real_demand_only():
    from tests.test_viz import LiveEsp
    from smarttraffic.viz import capture
    c = cfg("S1", warm=0, meas=30, flush=30)
    hw = load_hardware_config()
    snaps = capture(c, [5000, 15000, 25000], hw_config=hw, transport_factory=lambda d: LiveEsp(),
                    arrivals=arr(c))
    for t, (snap, _) in snaps.items():
        assert not [v for v in snap.vehicles if v.movement == "NS"], t
        assert snap.hardware.real_only == ("NS",)
    assert {v.movement for s, _ in snaps.values() for v in s.vehicles} - {"NS"}   # others are drawn
    assert NO_HARDWARE.real_only == ()


def test_visualization_session_uses_the_rule_even_without_a_device():
    from tests.test_viz import DeadPort
    c = cfg("S1", warm=0, meas=30, flush=30)
    s = VizSession(c, arrivals=arr(c), hw_config=load_hardware_config(),
                   transport_factory=lambda d: DeadPort(), running=True,
                   clock=lambda: time.monotonic() * 200).start()
    assert s.wait_finished(60) and s.error is None
    snap = s.latest()
    assert snap.hardware.real_only == ("NS",) and not [v for v in snap.vehicles if v.movement == "NS"]
    assert '"virtual_arrivals_excluded":["NS"]' in s.result.to_json()


# ------------------------------------------------------------------ 2. real input only
def test_ns_detectors_change_only_from_real_sensor_input():
    c = cfg()
    idle = Rig(c).run()                                     # sensors never triggered
    assert not [e for e in idle.sb.edge_log if (e.movement, e.detector) in NS_DET]
    assert not any(d1 or d2 for _, _, d1, d2, *_ in idle.trace)
    assert idle.hw.detectors.dropped_overridden == 0        # not even masked virtual NS edges exist

    script = {10_000: lambda e: e.car(0, True), 10_400: lambda e: e.car(1, True),
              11_000: lambda e: e.car(0, False), 11_400: lambda e: e.car(1, False)}
    live = Rig(c, script).run()
    ns = [(e.detector, e.occupied) for e in live.sb.edge_log if (e.movement, e.detector) in NS_DET]
    assert ns == [("D1", True), ("D2", True), ("D1", False), ("D2", False)]      # exactly the IR edges
    assert live.hw.detectors.dropped_overridden == 0


# ------------------------------------------------------------------ 3. other movements still spawn
def test_other_movements_keep_their_full_virtual_arrivals():
    c = cfg()
    rig = Rig(c).run()
    plain = simulate(c, arrivals=arr(c), collect_stats=False)
    ids = lambda vm, m: sorted(v.id for v in vm.all_vehicles() if v.movement == m)   # noqa: E731
    for m in MOVES:
        if m != "NS":
            assert ids(rig.vm, m) == ids(plain.vehicles, m) and ids(plain.vehicles, m), m
    assert ids(plain.vehicles, "NS") and not ids(rig.vm, "NS")
    others = [e for e in rig.sb.edge_log if e.movement != "NS"]
    assert others                                           # the 14 virtual detectors still fire


# ------------------------------------------------------------------ 4. virtual-only unchanged
def test_virtual_only_mode_is_exactly_as_before():
    c = cfg(meas=60, flush=60)
    ref = run(c, arrivals=arr(c)).to_json()
    assert run(c, arrivals=arr(c), exclude_movements=()).to_json() == ref
    assert "virtual_arrivals_excluded" not in ref
    sim = simulate(c, arrivals=arr(c), exclude_movements=(), collect_stats=False)
    assert sim.arrivals_count == len(arr(c).arrivals)
    s = VizSession(c, arrivals=arr(c), running=True, clock=lambda: time.monotonic() * 500).start()
    assert s.wait_finished(60) and s.error is None and s.result.to_json() == ref
    assert s.latest().hardware == NO_HARDWARE


# ------------------------------------------------------------------ 5. disconnect / reconnect
def _disconnect_script():
    return {8_000: lambda e: e.car(0, True),
            20_000: lambda e: e.unplug(),
            25_000: lambda e: (e.car(0, False), e.car(1, True)),     # unseen while unplugged
            40_000: lambda e: e.plug(),
            55_000: lambda e: e.car(1, False)}


def test_disconnect_and_reconnect_are_deterministic_and_safe():
    c = cfg()
    a = Rig(c, _disconnect_script()).run()
    b = Rig(c, _disconnect_script()).run()
    assert a.result.to_json() == b.result.to_json()
    assert a.trace == b.trace and a.esp.commands == b.esp.commands
    assert [tuple(e) for e in a.sb.edge_log] == [tuple(e) for e in b.sb.edge_log]

    at = {ms: row for ms, *row in a.trace}
    n, d1, d2, fault, recall = at[30_000]                  # unplugged
    assert n == 0 and fault and recall and d1 is True and d2 is False   # held, nothing invented
    assert all(r[0] == 0 for r in at.values())              # never a virtual NS vehicle
    n, d1, d2, fault, recall = at[50_000]                  # reconnected and re-synced
    assert not fault and not recall and d1 is False and d2 is True
    ns = [(e.detector, e.occupied) for e in a.sb.edge_log if e.movement == "NS"]
    assert ns == [("D1", True), ("D1", False), ("D2", True), ("D2", False)]   # re-sync edges, then real
    link = a.hw.hub.link("esp32_a")
    assert link.stats["disconnects"] >= 1 and link.stats["opens"] >= 2


def test_device_missing_for_the_whole_run_keeps_ns_without_virtual_demand():
    esp = esp_sim.SimulatedEsp32()
    esp.unplug()
    rig = Rig(cfg(), esp=esp).run()
    assert all(r[1] == 0 and r[4] and r[5] for r in rig.trace)   # no vehicles, faulted, on recall
    assert not [e for e in rig.sb.edge_log if e.movement == "NS"]
    assert esp.commands == []


# ------------------------------------------------------------------ 6. no duplicate NS demand
def test_no_duplicate_real_and_virtual_ns_demand():
    c = cfg()
    script = {10_000: lambda e: e.car(0, True), 10_500: lambda e: e.car(0, False)}
    new = Rig(c, dict(script)).run()
    old = Rig(c, dict(script), exclude=False).run()          # previous behaviour, for contrast
    assert old.hw.detectors.dropped_overridden > 0           # virtual NS vehicles were only masked
    assert any(v.movement == "NS" for v in old.vm.all_vehicles())
    assert new.hw.detectors.dropped_overridden == 0          # now there is nothing to mask
    assert not any(v.movement == "NS" for v in new.vm.all_vehicles())
    ns_new = [(e.detector, e.occupied) for e in new.sb.edge_log if e.movement == "NS"]
    assert ns_new == [("D1", True), ("D1", False)]           # one real vehicle = one D1 pulse
    # the result counts no virtual NS arrivals: every vehicle it counts comes from another movement
    assert new.result.to_json() != old.result.to_json()


def test_hw_smoke_hybrid_drops_ns_traffic_and_says_so():
    from smarttraffic import hw_smoke
    esp = esp_sim.SimulatedEsp32()
    seen = []

    class P(hw_smoke.Pacer):
        def wait_until(self, sim_ms):
            esp.advance_to(sim_ms)
    orig = hw_smoke.VehicleModel

    def spy(arrivals, **kw):
        seen.extend(arrivals)
        return orig(arrivals, **kw)
    traffic = generate_arrivals("S2", 1, 0, 20).arrivals
    assert any(a.movement == "NS" for a in traffic)
    lines = []
    hw_smoke.VehicleModel = spy
    try:
        hw_smoke.run_hybrid(load_hardware_config(), 20, out=lines.append, transport_factory=lambda d: esp,
                            pacer=P(realtime=False), arrivals=traffic)
    finally:
        hw_smoke.VehicleModel = orig
    assert "Real-only      : NS (no virtual vehicles)" in lines
    assert seen and not [a for a in seen if a.movement == "NS"]
    assert len(seen) == sum(a.movement != "NS" for a in traffic)
