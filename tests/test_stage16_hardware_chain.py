"""Stage 16 -- hardware end to end on the executable firmware model (automated part).

Chain: IR inputs GPIO27 / GPIO26 (active-low) -> SimulatedEsp32 firmware logic -> protocol v2
-> EspLink -> HardwareDetectorSource -> HybridDetectorSource (2 real + 14 virtual) ->
SensorBank -> Smart-T1 -> SignalBank -> simulator -> SignalMirror -> SIG -> R/Y/G outputs
GPIO32 / GPIO33 / GPIO25. Run by runner.run with the visualization's hardware rule
(no virtual NS vehicles). Every run is also audited (tests/stage16_checks.Audit).

Manual checks on the real board (D1, D2, light sequence, end-to-end, no NS virtual cars,
disconnect / reconnect) were done by the user and are recorded in docs/STAGE16_VERIFICATION.md;
they are not repeated or simulated here beyond this firmware model.
"""
from smarttraffic import esp_sim
from smarttraffic.hardware import LinkState, load_hardware_config
from smarttraffic.runner import RunConfig
from smarttraffic.signals import FLASHING_GREEN, RED_YELLOW, lamps_for
from smarttraffic.viz import smart_t1_params
from tests.stage16_checks import HardwareChain, audited_run, ns_car_script

T1 = smart_t1_params()
HW = load_hardware_config()


def run_chain(chain, measure_s=600.0, sc="S1"):
    cfg = RunConfig(sc, 1, "Smart", warmup_s=0.0, measure_s=measure_s, flush_s=300.0, smart_params=T1)
    res, rep = audited_run(cfg, source_factory=chain.factory, observers=[chain.observe], virtual=False,
                           exclude_movements=HW.real_only_movements, check_every=4)
    assert rep["errors"] == [], rep["errors"][:5]
    return res, rep


def test_connected_chain_mirrors_every_state_and_real_demand_drives_smart():
    chain = HardwareChain(cars=ns_car_script(20_000, 580_000, period_ms=41_000))
    res, rep = run_chain(chain)
    s = chain.summary()
    assert s["lamp_mismatches"] == 0 and s["connected_steps"] > 5000 and s["esp_errors"] == []
    assert s["states_seen"] == ["ALL_RED", "FLASHING_GREEN", "GREEN", "RED", "RED_YELLOW", "YELLOW"]
    assert s["ns_virtual_vehicles_seen"] == 0 and s["fault_steps"] == 0
    # discrete outputs: RED_YELLOW = red + yellow; FLASHING_GREEN follows the simulator's flash
    ry = [r for r in chain.trace if r[1] is RED_YELLOW]
    assert ry and all(r[3] == (True, True, False) for r in ry)
    fg = [r for r in chain.trace if r[1] is FLASHING_GREEN]
    assert {r[3][2] for r in fg} == {True, False}
    # NS gets GREEN only because of real IR demand: every NS green onset follows a real NS edge
    ns_edges = [e.time_ms for e in chain._sb.edge_log if e.movement == "NS" and e.occupied]
    onsets = [b[0] for a, b in zip(chain.trace, chain.trace[1:])
              if a[1].name == "RED_YELLOW" and b[1].name == "GREEN"]
    # the only exception is the start-up fail-safe recall: the controller's first decision
    # (t = 0) comes before the first serial read, so NS:D1/D2 are still "not synced" (faulted)
    assert onsets[0] == 1000 and not [t for t in ns_edges if t < 1000]
    assert all(any(t < o for t in ns_edges) for o in onsets[1:]) and len(onsets) >= 3
    assert len(onsets) - 1 <= len(ns_edges)
    # the 7 other movements stay virtual and busy
    import json
    moves = {m["movement"]: m["arrivals"] for m in json.loads(res.to_json())["movements"]}
    assert moves["NS"] == 0 and all(moves[m] > 0 for m in moves if m != "NS")


def test_without_ir_events_ns_is_served_only_by_the_start_up_recall():
    chain = HardwareChain(cars=())
    run_chain(chain, measure_s=300.0)
    starts = [c for c in chain.ctrl.command_log if c.movement == "NS" and c.kind == "start_green"]
    assert [c.time_ms for c in starts] == [0]              # before the first HELLO is read
    assert chain.summary()["ns_green_onsets"] <= 1
    assert not [e for e in chain._sb.edge_log if e.movement == "NS"]


def test_both_real_sensors_occupied_and_clear_reach_the_sensor_bank_exactly():
    cars = [(10_000, 0, True), (10_400, 0, False), (20_000, 1, True), (23_000, 1, False)]
    chain = HardwareChain(cars=cars)
    run_chain(chain, measure_s=120.0)
    ns = [(e.detector, e.occupied) for e in chain._sb.edge_log if e.movement == "NS"]
    assert ns == [("D1", True), ("D1", False), ("D2", True), ("D2", False)]


def test_disconnect_mid_run_reconnect_fail_safe_and_no_crash():
    chain = HardwareChain(cars=ns_car_script(10_000, 580_000, period_ms=33_000),
                          script={200_000: lambda e: e.unplug(), 260_000: lambda e: e.plug()})
    res, rep = run_chain(chain)
    s = chain.summary()
    link = chain.hw.hub.link("esp32_a")
    assert link.state is LinkState.CONNECTED and link.stats["disconnects"] == 1
    assert s["lamp_mismatches"] == 0                     # identical whenever connected
    assert s["fault_steps"] > 500 and s["fault_only_yellow"] and s["fault_yellow_blinks"]
    down = [r for r in chain.trace if 200_000 < r[0] < 260_000]
    assert all(r[5] for r in down[5:])                   # real NS detectors faulted (held)
    after = [r for r in chain.trace if r[0] > 265_000]
    assert after and not any(r[5] for r in after)        # re-synced after HELLO + full-state report
    # fail-safe yellow blink: 1 Hz, ON for the first 500 ms of each second (device time)
    blink = [r for r in chain.trace if r[6]]
    assert all(r[3][1] == (r[0] % 1000 < 500) for r in blink)


def test_missing_hardware_at_startup_keeps_running_with_recall():
    esp = esp_sim.SimulatedEsp32()
    esp.unplug()
    chain = HardwareChain(esp=esp)
    res, rep = run_chain(chain, measure_s=300.0)
    s = chain.summary()
    assert chain.hw.hub.link("esp32_a").state is LinkState.DISCONNECTED
    assert s["ns_faulted_steps"] == s["steps"] and esp.commands == [] and esp.in_fault()
    assert s["ns_green_onsets"] > 0                      # Smart serves NS by fail-safe recall
    assert s["ns_virtual_vehicles_seen"] == 0


def test_old_firmware_is_rejected_safely():
    esp = esp_sim.SimulatedEsp32(proto_version=1)
    chain = HardwareChain(esp=esp)
    run_chain(chain, measure_s=120.0)
    assert chain.hw.hub.link("esp32_a").state is LinkState.PROTOCOL_MISMATCH
    assert esp.commands == [] and chain.summary()["ns_faulted_steps"] == chain.summary()["steps"]


def test_flashing_green_physical_timing_equals_lamps_for():
    chain = HardwareChain(cars=ns_car_script(20_000, 300_000, period_ms=29_000))
    run_chain(chain, measure_s=300.0)
    fg = [r for r in chain.trace if r[1] is FLASHING_GREEN]
    assert fg
    for r in fg:
        assert r[3] == r[2]                                # physical == logical lamps
