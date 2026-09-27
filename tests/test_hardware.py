"""Tests for Stage 9 hardware integration -- all WITHOUT real hardware.

A FakeTransport stands in for the USB serial port and a FakeEsp produces
device frames with chosen sequence numbers and millis() timestamps, so every
receive time (the simulation time passed to pump/poll) and therefore every
mapped edge time is deterministic.
"""
import ast
from pathlib import Path

import pytest

from smarttraffic import esp_protocol as p
from smarttraffic import hw_smoke
from smarttraffic.conflicts import MOVES
from smarttraffic.hardware import (EspLink, HardwareConfigError, HardwareDetectorSource, HardwareHub,
                                   LinkSettings, LinkState, Target, build_hardware,
                                   load_hardware_config, parse_hardware_config)
from smarttraffic.hybrid_sensors import (HybridDetectorSource, SensorAssignmentError,
                                         parse_sensor_id)
from smarttraffic.sensors import (D1, D2, SENSOR_IDS, DemandLevel, DetectorEdge, SensorBank,
                                  VirtualDetectors)
from smarttraffic.signals import SignalBank, SignalState
from smarttraffic.traffic_gen import Arrival
from smarttraffic.vehicles import VehicleModel

ROOT = Path(__file__).resolve().parent.parent


# =================================================================== fakes
class FakeTransport:
    def __init__(self):
        self.is_open = False
        self.inbox = bytearray()
        self.sent = []
        self.fail_open = False
        self.fail_read = False
        self.opens = 0

    def open(self):
        if self.fail_open:
            raise OSError("could not open port")
        self.is_open = True
        self.opens += 1

    def close(self):
        self.is_open = False

    def read_available(self):
        if self.fail_read:
            raise OSError("device unplugged")
        data, self.inbox = bytes(self.inbox), bytearray()
        return data

    def write(self, data):
        if not self.is_open:
            raise OSError("not open")
        self.sent.append(data)

    def feed(self, *frames):
        for f in frames:
            self.inbox.extend(f)

    def commands(self):
        return [p.decode_frame(x) for x in self.sent]


class FakeEsp:
    """Builds frames like the firmware: seq counter per boot, explicit device ms."""
    def __init__(self, device_id="esp32_a", boot="0000BEEF", n_sensors=2, n_signals=2):
        self.device_id, self.boot, self.n_sensors, self.n_signals = device_id, boot, n_sensors, n_signals
        self.seq = 0

    def _s(self):
        s = self.seq
        self.seq += 1
        return s

    def hello(self, ms):
        return p.encode_hello(self._s(), ms, self.device_id, self.boot, self.n_sensors, self.n_signals)

    def sen(self, ms, ch, occ):
        return p.encode_sensor(self._s(), ms, ch, occ)

    def hb(self, ms):
        return p.encode_heartbeat(self._s(), ms)

    def ack(self, ms, ch, st):
        return p.encode_ack(self._s(), ms, ch, st)

    def reboot(self, boot):
        self.boot, self.seq = boot, 0


def one_device_cfg(sensors=None, signals=None):
    raw = {"devices": {"esp32_a": {"port": "COM3"}},
           "sensors": sensors if sensors is not None else {"NS:D1": {"device": "esp32_a", "channel": 0}},
           "signals": signals if signals is not None else {"NS": {"device": "esp32_a", "channel": 0}}}
    return parse_hardware_config(raw)


def fake_factory(store):
    def make(dev_cfg):
        t = FakeTransport()
        store[dev_cfg.device_id] = t
        return t
    return make


def edge(t, m, d, occ):
    return DetectorEdge(t, m, d, occ)


def connected_link(esp=None, now=100, dev_ms=1000, settings=LinkSettings()):
    t = FakeTransport()
    link = EspLink("esp32_a", t, settings)
    esp = esp or FakeEsp()
    link.pump(now)                                    # opens, sends QRY
    t.feed(esp.hello(dev_ms), esp.sen(dev_ms, 0, False), esp.sen(dev_ms, 1, False))
    link.pump(now + 100)                              # offset = (now + 100) - dev_ms
    assert link.state is LinkState.CONNECTED
    link.take_sensor_reports()
    return link, t, esp


# =================================================================== config
def test_default_config_is_the_demo_mapping():
    cfg = load_hardware_config()
    assert set(cfg.devices) == {"esp32_a"}
    assert dict(cfg.sensors) == {("NS", "D1"): Target("esp32_a", 0)}
    assert dict(cfg.signals) == {"NS": Target("esp32_a", 0)}
    assert cfg.devices["esp32_a"].baud == 115200


def test_with_port_override():
    cfg = load_hardware_config().with_port("esp32_a", "COM9")
    assert cfg.devices["esp32_a"].port == "COM9"
    with pytest.raises(HardwareConfigError):
        cfg.with_port("nope", "COM1")


@pytest.mark.parametrize("raw,msg", [
    ({"devices": {"a": {"port": "COM3"}}, "sensors": {"NS:D3": {"device": "a", "channel": 0}}}, "unknown sensor"),
    ({"devices": {"a": {"port": "COM3"}}, "sensors": {"NS:D1": {"device": "b", "channel": 0}}}, "unknown device"),
    ({"devices": {"a": {"port": "COM3"}}, "sensors": {"NS:D1": {"device": "a", "channel": -1}}}, "channel"),
    ({"devices": {"a": {"port": "COM3"}}, "sensors": {"NS:D1": {"device": "a"}}}, "exactly"),
    ({"devices": {"a": {"port": "COM3"}}, "sensors": {"NS:D1": {"device": "a", "channel": 0},
                                                      "NS:D2": {"device": "a", "channel": 0}}}, "already used"),
    ({"devices": {"a": {"port": "COM3"}}, "signals": {"XX": {"device": "a", "channel": 0}}}, "unknown movement"),
    ({"devices": {"a": {"port": "COM3"}}, "signals": {"NS": {"device": "a", "channel": 0},
                                                      "SS": {"device": "a", "channel": 0}}}, "already used"),
    ({"devices": {"bad id!": {"port": "COM3"}}}, "device id"),
    ({"devices": {"a": {"port": ""}}}, "port"),
    ({"devices": {"a": {"port": "COM3", "speed": 1}}}, "port"),
    ({"devices": {"a": {"port": "COM3"}}, "extra": 1}, "top-level"),
    ({"devices": {"a": {"port": "COM3"}}, "link": {"heartbeat_timeout_ms": 0}}, "positive"),
    ({"devices": {"a": {"port": "COM3"}}, "link": {"bogus": 1}}, "allowed keys"),
])
def test_config_errors(raw, msg):
    with pytest.raises(HardwareConfigError, match=msg):
        parse_hardware_config(raw)


def test_duplicate_json_key_rejected(tmp_path):
    f = tmp_path / "hw.json"
    f.write_text('{"devices": {"a": {"port": "COM3"}}, "sensors": {'
                 '"NS:D1": {"device": "a", "channel": 0}, "NS:D1": {"device": "a", "channel": 1}}}')
    with pytest.raises(HardwareConfigError, match="duplicate key"):
        load_hardware_config(f)


def test_sensor_and_signal_may_share_a_channel_number():
    cfg = one_device_cfg()
    assert cfg.sensors[("NS", "D1")].channel == cfg.signals["NS"].channel == 0


# =================================================================== link
def test_link_connects_after_hello_and_queries_on_open():
    t = FakeTransport()
    link = EspLink("esp32_a", t)
    link.pump(0)
    assert link.state is LinkState.WAITING_HELLO and t.commands() == [["QRY"]]
    esp = FakeEsp()
    t.feed(esp.hb(10))                                  # before HELLO: ignored
    link.pump(100)
    assert link.state is LinkState.WAITING_HELLO and link.stats["before_hello"] == 1
    t.feed(esp.hello(20))
    link.pump(200)
    assert link.state is LinkState.CONNECTED and link.healthy() and link.boot_id == "0000BEEF"


def test_open_failure_and_retry_interval():
    t = FakeTransport()
    t.fail_open = True
    link = EspLink("esp32_a", t, LinkSettings(reconnect_interval_ms=2000))
    link.pump(0)
    assert link.state is LinkState.DISCONNECTED and link.stats["open_failures"] == 1
    link.pump(1999)
    assert link.stats["open_failures"] == 1               # not retried yet
    t.fail_open = False
    link.pump(2000)
    assert link.state is LinkState.WAITING_HELLO and t.opens == 1


def test_wrong_device_and_protocol_mismatch():
    link, t, _ = connected_link()
    t.feed(FakeEsp(device_id="esp32_b", boot="1").hello(5))
    link.pump(300)
    assert link.state is LinkState.WRONG_DEVICE and not link.healthy()
    t2 = FakeTransport()
    l2 = EspLink("esp32_a", t2)
    l2.pump(0)
    t2.feed(p.encode_hello(0, 1, "esp32_a", "AB", 1, 1, proto=2))
    l2.pump(100)
    assert l2.state is LinkState.PROTOCOL_MISMATCH


def test_malformed_lines_are_counted_not_fatal():
    link, t, esp = connected_link()
    t.feed(b"garbage\n", b"$HB,1,2*00\n", b"\xff\xfe\n", esp.hb(1100))
    link.pump(300)
    assert link.stats["malformed"] == 3 and link.healthy()


def test_partial_lines_are_reassembled():
    link, t, esp = connected_link()
    frame = esp.sen(1100, 0, True)
    t.feed(frame[:7])
    link.pump(300)
    assert link.take_sensor_reports() == []
    t.feed(frame[7:])
    link.pump(400)
    assert [r[1:] for r in link.take_sensor_reports()] == [(0, True)]


def test_overlong_garbage_without_newline_is_discarded():
    link, t, esp = connected_link()
    t.feed(b"x" * 500)
    link.pump(300)
    assert link.stats["malformed"] == 1
    t.feed(b"\n", esp.hb(1200))
    link.pump(400)
    assert link.healthy()


def test_duplicate_messages_dropped():
    link, t, esp = connected_link()
    frame = esp.sen(1100, 0, True)
    t.feed(frame, frame)
    link.pump(300)
    assert link.stats["duplicates"] == 1 and len(link.take_sensor_reports()) == 1


def test_out_of_order_message_dropped():
    link, t, esp = connected_link()
    older = esp.sen(1100, 0, True)
    newer = esp.sen(1110, 0, False)
    t.feed(newer, older)
    link.pump(300)
    assert link.stats["duplicates"] == 1
    assert [r[2] for r in link.take_sensor_reports()] == [False]


def test_sequence_gap_counts_lost_and_requests_resync():
    link, t, esp = connected_link()
    esp.seq += 3                                # 3 messages lost in transit
    before = len(t.sent)
    t.feed(esp.hb(1200))
    link.pump(2000)
    assert link.stats["lost_messages"] == 3 and ["QRY"] in t.commands()[before:]


def test_stale_link_and_recovery():
    link, t, esp = connected_link(settings=LinkSettings(heartbeat_timeout_ms=1500))
    link.pump(1700)
    assert link.state is LinkState.CONNECTED            # exactly 1500 ms since the last message at 200
    link.pump(1701)
    assert link.state is LinkState.STALE and not link.healthy()
    t.feed(esp.hb(3000))
    link.pump(1800)
    assert link.state is LinkState.CONNECTED


def test_disconnect_and_reconnect_same_boot():
    link, t, esp = connected_link(settings=LinkSettings(reconnect_interval_ms=2000))
    t.fail_read = True
    link.pump(300)
    assert link.state is LinkState.DISCONNECTED and link.stats["disconnects"] == 1
    t.fail_read = False
    link.pump(1000)                              # too early to retry (last attempt at 100)
    assert link.state is LinkState.DISCONNECTED
    link.pump(2100)
    assert link.state is LinkState.WAITING_HELLO and t.commands()[-1] == ["QRY"]
    t.feed(esp.hello(3000), esp.sen(3000, 0, True))
    link.pump(2200)
    assert link.state is LinkState.CONNECTED and link.stats["device_resets"] == 0


def test_device_reset_new_boot_resets_time_base():
    link, t, esp = connected_link(now=100, dev_ms=1000)       # offset = 200 - 1000
    assert link.offset_ms == -800
    esp.reboot("0000CAFE")
    t.feed(esp.hello(5), esp.sen(5, 0, False))
    link.pump(10_000)
    assert link.stats["device_resets"] == 1 and link.offset_ms == 10_000 - 5
    assert link.boot_id == "0000CAFE"


def test_reset_with_lost_hello_is_detected():
    link, t, esp = connected_link(now=100, dev_ms=100_000)
    esp.seq = 1                                   # rebooted, HELLO lost: small seq, small time
    t.feed(esp.hb(50))
    link.pump(500)
    assert link.state is LinkState.WAITING_HELLO and link.stats["suspected_resets"] == 1
    assert t.commands()[-1] == ["QRY"]


def test_millis_wraparound_is_unwrapped():
    near = p.U32 - 300
    link, t, esp = connected_link(now=100, dev_ms=near)          # offset = 200 - near
    t.feed(esp.sen(near + 100, 0, True))
    link.pump(300)
    t.feed(esp.sen(50, 0, False))                                # wrapped: device time = U32 + 50
    link.pump(700)
    assert [x[0] for x in link.take_sensor_reports()] == [200 + 100, 200 + 300 + 50]


def test_timestamp_mapping_uses_min_latency():
    link, t, esp = connected_link(now=100, dev_ms=1000)          # sample 200 - 1000 = -800
    t.feed(esp.hb(1250))
    link.pump(400)                                               # sample 400 - 1250 = -850
    assert link.offset_ms == -850
    t.feed(esp.sen(1300, 0, True))
    link.pump(500)
    assert link.take_sensor_reports() == [(1300 - 850, 0, True)]


def test_older_report_timestamp_is_not_a_wrap():
    link, t, esp = connected_link(now=100, dev_ms=1000)
    t.feed(esp.hb(1200), esp.sen(1180, 0, True))                 # edge stamped at debounce start
    link.pump(300)                                               # HB sample: 300 - 1200 = -900 (new min)
    assert link.take_sensor_reports() == [(1180 - 900, 0, True)]
    assert link._wraps == 0 and link.stats["duplicates"] == 0


def test_unknown_channel_counted():
    link, t, esp = connected_link()
    t.feed(esp.sen(1100, 5, True))
    link.pump(300)
    assert link.stats["unknown_channel"] == 1 and link.take_sensor_reports() == []


def test_link_time_must_not_go_backwards():
    link, _, _ = connected_link()
    with pytest.raises(ValueError):
        link.pump(50)


def test_close_is_clean():
    link, t, _ = connected_link()
    link.close()
    assert link.state is LinkState.CLOSED and not t.is_open
    link.pump(10_000)                             # no reopen after close
    assert not t.is_open


# ============================================================ real detectors
def real_source(start=0, sensors=None):
    cfg = one_device_cfg(sensors=sensors)
    store = {}
    hub = HardwareHub(cfg, fake_factory(store))
    src = HardwareDetectorSource(hub, cfg.sensors, start)
    return src, store["esp32_a"], hub


def bring_up(src, t, esp, until=200, dev_ms=1000):
    src.poll(until - 100)
    t.feed(esp.hello(dev_ms), esp.sen(dev_ms, 0, False), esp.sen(dev_ms, 1, False))
    return src.poll(until)                                   # offset = until - dev_ms


def test_real_occupied_clear_edges_exact_times():
    src, t, hub = real_source()
    esp = FakeEsp()
    assert bring_up(src, t, esp) == []                       # offset -800, all clear
    t.feed(esp.sen(1050, 0, True))
    assert src.poll(300) == [edge(250, "NS", D1, True)]
    t.feed(esp.sen(1150, 0, False))
    assert src.poll(400) == [edge(350, "NS", D1, False)]


def test_late_report_is_clamped_to_next_ms():
    src, t, hub = real_source()
    esp = FakeEsp()
    bring_up(src, t, esp)
    src.poll(500)
    t.feed(esp.sen(1100, 0, True))                          # maps to 300 <= now (500)
    assert src.poll(600) == [edge(501, "NS", D1, True)]
    assert src.stats["late_edges"] == 1


def test_repeated_state_reports_are_not_edges():
    src, t, hub = real_source()
    esp = FakeEsp()
    bring_up(src, t, esp)
    t.feed(esp.sen(1050, 0, False), esp.sen(1060, 0, False))
    assert src.poll(300) == [] and src.stats["repeated_state"] >= 2


def test_faults_until_synced_and_on_disconnect_with_held_state():
    src, t, hub = real_source()
    esp = FakeEsp()
    assert src.faults() == (("NS", "D1"),)                  # never connected
    src.poll(100)
    t.feed(esp.hello(1000))
    src.poll(200)
    assert src.faults() == (("NS", "D1"),)                  # connected, state not reported yet
    t.feed(esp.sen(1000, 0, True))
    assert src.poll(300) == [edge(201, "NS", D1, True)]     # startup state (late -> next ms)
    assert src.faults() == ()
    t.fail_read = True
    assert src.poll(400) == []
    assert src.faults() == (("NS", "D1"),)
    assert src.status()["NS:D1"]["occupied"] is True        # last confirmed state held, not invented
    t.fail_read = False
    for now in (2500, 2600):
        src.poll(now)
    t.feed(esp.hello(3000), esp.sen(3000, 0, False))       # after reconnect the car is gone
    edges = src.poll(2700)
    assert [(e.detector, e.occupied) for e in edges] == [(D1, False)] and src.faults() == ()


def test_unmapped_channel_is_ignored():
    src, t, hub = real_source()
    esp = FakeEsp()
    bring_up(src, t, esp)
    t.feed(esp.sen(1050, 1, True))                          # channel 1 exists but is not mapped
    assert src.poll(300) == []
    assert src.stats["unmapped_reports"] == 2               # its start-up report and this one


def test_several_real_sensors_on_one_device():
    sensors = {"NS:D1": {"device": "esp32_a", "channel": 0}, "NS:D2": {"device": "esp32_a", "channel": 1}}
    src, t, hub = real_source(sensors=sensors)
    esp = FakeEsp()
    bring_up(src, t, esp)
    t.feed(esp.sen(1010, 1, True), esp.sen(1020, 0, True))
    assert src.poll(300) == [edge(210, "NS", D2, True), edge(220, "NS", D1, True)]
    assert src.sensor_ids == (("NS", "D1"), ("NS", "D2"))


def test_real_source_rejects_bad_mapping():
    cfg = one_device_cfg()
    hub = HardwareHub(cfg, fake_factory({}))
    with pytest.raises(HardwareConfigError):
        HardwareDetectorSource(hub, {}, 0)
    with pytest.raises(HardwareConfigError):
        HardwareDetectorSource(hub, {("NS", "D1"): Target("esp32_a", 0),
                                     ("NS", "D2"): Target("esp32_a", 0)}, 0)
    with pytest.raises(HardwareConfigError):
        HardwareDetectorSource(hub, {("NS", "D1"): Target("ghost", 0)}, 0)


# ============================================================ hybrid 15 + 1
def arrs(spec):
    seqs, out = {}, []
    for i, (m, t) in enumerate(sorted(spec, key=lambda x: x[1])):
        out.append(Arrival(i, round(t * 1000), m, seqs.get(m, 0), "random"))
        seqs[m] = seqs.get(m, 0) + 1
    return out


class Rig:
    """VehicleModel + build_hardware (fake serial) + SensorBank + SignalBank, 100 ms steps."""
    def __init__(self, arrivals=(), cfg=None, start=0):
        self.cfg = cfg or one_device_cfg()
        self.vm = VehicleModel(list(arrivals), start=start)
        self.store = {}
        self.hw = build_hardware(self.cfg, self.vm, fake_factory(self.store))
        self.sb = SensorBank(self.hw.detectors)
        self.bank = SignalBank(start_time=start)
        self.heartbeats = []                  # [(transport, FakeEsp)] fed one HB per step

    def step(self, cmds=()):
        for t, esp in self.heartbeats:
            t.feed(esp.hb(self.vm.now_ms))
        for m, c in cmds:
            (self.bank.start_green if c == "start" else self.bank.end_green)(m)
        self.vm.advance(0.1, self.bank)
        self.sb.advance(0.1, self.bank)
        self.bank.advance(0.1)
        if self.hw.mirror:
            self.hw.mirror.update(self.bank)

    def run_to(self, ms):
        while self.vm.now_ms < ms:
            self.step()


def test_hybrid_has_16_ids_and_assignment():
    rig = Rig()
    det = rig.hw.detectors
    assert det.sensor_ids == SENSOR_IDS and len(det.sensor_ids) == 16
    assert det.source_of(("NS", "D1")) is rig.hw.real
    assert all(det.source_of(s) is rig.hw.virtual for s in SENSOR_IDS if s != ("NS", "D1"))
    assert set(rig.sb.snapshot()) == set(MOVES)


def test_overridden_virtual_detector_produces_no_events():
    rig = Rig(arrivals=arrs([("NS", 0.0), ("SS", 0.5)]))
    rig.run_to(20_000)                                   # both cars stop at slot 1 on red
    assert [e for e in rig.sb.edge_log if (e.movement, e.detector) == ("NS", "D1")] == []
    assert rig.hw.detectors.dropped_overridden >= 1      # virtual NS:D1 edge computed but dropped
    assert rig.sb.d1("SS") is True                       # other virtual detectors still work
    assert any((e.movement, e.detector) == ("NS", "D2") for e in rig.sb.edge_log)
    assert rig.sb.d1("NS") is False                      # NS:D1 is the (silent) real sensor


def test_real_edge_drives_sensorbank_call():
    rig = Rig()
    t = rig.store["esp32_a"]
    esp = FakeEsp()
    rig.run_to(100)
    t.feed(esp.hello(5000), esp.sen(5000, 0, False), esp.sen(5000, 1, False))
    rig.run_to(300)                                      # received at 200: offset 200 - 5000
    assert not rig.sb.detector_fault("NS", D1)
    t.feed(esp.sen(5150, 0, True))
    rig.run_to(400)
    s = rig.sb.status("NS")
    assert s.d1 and s.has_call and s.call_since_ms == 350 and s.demand_level == DemandLevel.LEVEL_1
    t.feed(esp.sen(5300, 0, False))
    rig.run_to(600)
    assert not rig.sb.d1("NS") and rig.sb.has_call("NS")          # call stays latched
    edges = [e for e in rig.sb.edge_log if (e.movement, e.detector) == ("NS", "D1")]
    assert [(e.time_ms, e.occupied) for e in edges] == [(350, True), (500, False)]


def test_faults_visible_through_sensorbank_only_for_real():
    rig = Rig()
    rig.run_to(200)
    assert rig.sb.faulted_detectors() == (("NS", "D1"),)          # not connected yet
    assert rig.sb.detector_fault("NS", D1) and not rig.sb.detector_fault("NS", D2)
    assert all(not rig.sb.detector_fault(m, d) for m, d in SENSOR_IDS if (m, d) != ("NS", "D1"))


def test_hybrid_validation():
    vm = VehicleModel([], start=0)
    v = VirtualDetectors(vm)
    with pytest.raises(SensorAssignmentError, match="missing"):
        HybridDetectorSource({sid: v for sid in SENSOR_IDS[:-1]})
    with pytest.raises(SensorAssignmentError, match="assigned twice"):
        HybridDetectorSource.with_overrides(v, [(("NS", "D1"), v), (("NS", "D1"), v)])
    with pytest.raises(SensorAssignmentError, match="unknown"):
        HybridDetectorSource.with_overrides(v, [(("XX", "D1"), v)])
    src, _, _ = real_source()
    with pytest.raises(SensorAssignmentError, match="does not provide"):
        HybridDetectorSource.with_overrides(v, [(("EL", "D2"), src)])
    src2, _, _ = real_source(start=500)
    with pytest.raises(SensorAssignmentError, match="different times"):
        HybridDetectorSource.with_overrides(v, [(("NS", "D1"), src2)])
    for bad in ("NS", "NS:D3", "XX:D1", "NS-D1", None):
        with pytest.raises(SensorAssignmentError):
            parse_sensor_id(bad)


def test_multiple_real_sensors_and_devices():
    cfg = parse_hardware_config({
        "devices": {"esp32_a": {"port": "COM3"}, "esp32_b": {"port": "COM4"}},
        "sensors": {"NS:D1": {"device": "esp32_a", "channel": 0},
                    "NS:D2": {"device": "esp32_a", "channel": 1},
                    "EL:D1": {"device": "esp32_b", "channel": 0}},
        "signals": {"NS": {"device": "esp32_a", "channel": 0},
                    "EL": {"device": "esp32_b", "channel": 0}}})
    rig = Rig(cfg=cfg)
    a, b = rig.store["esp32_a"], rig.store["esp32_b"]
    ea, eb = FakeEsp("esp32_a"), FakeEsp("esp32_b", boot="B00B")
    rig.run_to(100)
    a.feed(ea.hello(0), ea.sen(0, 0, False), ea.sen(0, 1, False))
    b.feed(eb.hello(9000), eb.sen(9000, 0, False))
    rig.run_to(300)
    assert rig.sb.faulted_detectors() == ()
    a.feed(ea.hb(150))                     # esp32_a: sample 400 - 150 = 250 ... not the min (200)
    b.feed(eb.hb(9150))                    # esp32_b: sample 400 - 9150 = -8750 ... not the min (-8800)
    rig.run_to(400)
    a.feed(ea.sen(230, 1, True))           # esp32_a: 230 + 200   -> 430
    b.feed(eb.sen(9260, 0, True))          # esp32_b: 9260 - 8800 -> 460
    rig.run_to(500)
    got = [(e.time_ms, e.movement, e.detector) for e in rig.sb.edge_log]
    assert got == [(430, "NS", "D2"), (460, "EL", "D1")]    # each device on its own time base
    assert all(rig.hw.detectors.source_of(s) is rig.hw.real for s in cfg.sensors)
    b.fail_read = True
    rig.run_to(600)
    assert rig.sb.faulted_detectors() == (("EL", "D1"),)       # only device b's detector


def test_all_virtual_when_no_real_sensors():
    rig = Rig(cfg=one_device_cfg(sensors={}), arrivals=arrs([("NS", 0.0)]))
    assert rig.hw.real is None
    rig.run_to(15_000)
    assert rig.sb.d1("NS") and rig.sb.faulted_detectors() == ()


def test_hybrid_run_is_deterministic():
    def run():
        rig = Rig(arrivals=arrs([("NS", 0.0), ("ES", 1.0), ("WL", 2.0)]))
        t, esp = rig.store["esp32_a"], FakeEsp()
        script = {100: [esp.hello(0), esp.sen(0, 0, False), esp.sen(0, 1, False)],
                  3000: [esp.sen(2900, 0, True)], 6000: [esp.hb(5950), esp.sen(5990, 0, False)]}
        while rig.vm.now_ms < 20_000:
            for f in script.get(rig.vm.now_ms, []):
                t.feed(f)
            rig.step()
        return rig.sb.edge_log, rig.sb.snapshot()
    assert run() == run()


# ============================================================ signal mirror
def mirror_rig():
    rig = Rig()
    t, esp = rig.store["esp32_a"], FakeEsp()
    rig.run_to(100)
    t.feed(esp.hello(0), esp.sen(0, 0, False), esp.sen(0, 1, False))
    rig.heartbeats.append((t, esp))
    rig.run_to(200)
    return rig, t, esp


def sig_cmds(t):
    return [c[2] for c in t.commands() if c[0] == "SIG"]


def test_mirror_sends_every_v2_state_once_per_change():
    rig, t, esp = mirror_rig()
    rig.step([("NS", "start")])
    rig.run_to(5000)
    rig.step([("NS", "end")])
    rig.run_to(20_000)
    seq = []
    for s in sig_cmds(t):
        if not seq or seq[-1] != s:
            seq.append(s)
    assert seq == ["RED", "RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED", "RED"]
    assert all(c[1] == "0" for c in t.commands() if c[0] == "SIG")


def test_flashing_green_is_sent_as_a_state_not_lamp_toggles():
    rig, t, esp = mirror_rig()
    rig.step([("NS", "start")])
    rig.run_to(3000)
    rig.step([("NS", "end")])
    n0 = len(sig_cmds(t))
    rig.run_to(5900)                                           # FLASHING_GREEN runs 3.0 s -> 6.0 s
    fg = sig_cmds(t)[n0:]
    assert set(fg) == {"FLASHING_GREEN"} and len(fg) <= 3      # 1 change + <= 2 refreshes


def test_all_red_sent_as_all_red():
    rig, t, esp = mirror_rig()
    rig.step([("NS", "start")])
    rig.run_to(2000)
    rig.step([("NS", "end")])
    rig.run_to(2100 + 3000 + 3500 + 500)
    assert rig.bank.state("NS") is SignalState.ALL_RED
    assert sig_cmds(t)[-1] == "ALL_RED"


def test_refresh_interval():
    rig, t, esp = mirror_rig()
    n0 = len(sig_cmds(t))
    rig.run_to(200 + 3000)
    assert len(sig_cmds(t)) - n0 == 3                          # RED refreshed every 1000 ms


def test_nothing_sent_while_disconnected_and_resent_after_hello():
    rig, t, esp = mirror_rig()
    rig.heartbeats.clear()
    t.fail_read = True
    rig.run_to(300)
    n0 = len(t.sent)
    rig.run_to(1500)
    assert len(t.sent) == n0                                   # port closed: nothing written
    t.fail_read = False
    rig.run_to(2500)                                           # reopened, QRY sent
    t.feed(esp.hello(3000), esp.sen(3000, 0, False), esp.sen(3000, 1, False))
    rig.run_to(2700)
    assert t.commands()[-1] == ["SIG", "0", "RED"]


def test_no_signal_commands_before_device_is_identified():
    rig = Rig()                                   # port opens at t=100, no HELLO yet
    rig.run_to(3000)
    t = rig.store["esp32_a"]
    assert t.is_open and rig.hw.hub.link("esp32_a").state.name == "WAITING_HELLO"
    assert sig_cmds(t) == [] and ["QRY"] in t.commands()          # only queries until HELLO


def test_mirror_does_not_change_the_signal_bank():
    def states(with_mirror):
        cfg = one_device_cfg() if with_mirror else one_device_cfg(signals={})
        rig = Rig(cfg=cfg)
        out = []
        for k in range(300):
            cmds = [("NS", "start")] if k == 10 else [("NS", "end")] if k == 80 else []
            rig.step(cmds)
            out.append(rig.bank.states())
        return out
    assert states(True) == states(False)


def test_mirror_is_not_a_ninth_signal():
    rig, t, esp = mirror_rig()
    assert set(rig.bank.states()) == set(MOVES) and len(rig.bank.states()) == 8
    assert rig.hw.mirror.movements == ("NS",)


def test_multiple_physical_signals():
    cfg = parse_hardware_config({
        "devices": {"esp32_a": {"port": "COM3"}, "esp32_b": {"port": "COM4"}},
        "sensors": {},
        "signals": {"NS": {"device": "esp32_a", "channel": 0}, "SS": {"device": "esp32_a", "channel": 1},
                    "EL": {"device": "esp32_b", "channel": 0}}})
    rig = Rig(cfg=cfg)
    a, b = rig.store["esp32_a"], rig.store["esp32_b"]
    ea, eb = FakeEsp("esp32_a"), FakeEsp("esp32_b", boot="2")
    rig.run_to(100)
    a.feed(ea.hello(0))
    b.feed(eb.hello(0))
    rig.heartbeats += [(a, ea), (b, eb)]
    rig.step([("NS", "start"), ("SS", "start")])
    rig.run_to(1500)
    ca = [(c[1], c[2]) for c in a.commands() if c[0] == "SIG"]
    cb = [(c[1], c[2]) for c in b.commands() if c[0] == "SIG"]
    assert ("0", "GREEN") in ca and ("1", "GREEN") in ca
    assert set(cb) == {("0", "RED")}


def test_blackout_on_close():
    rig, t, esp = mirror_rig()
    rig.hw.close()
    assert ["SIG", "0", "OFF"] in t.commands() and not t.is_open


def test_mirror_status_reports_ack():
    rig, t, esp = mirror_rig()
    t.feed(esp.ack(300, 0, "RED"))
    rig.run_to(400)
    st = rig.hw.mirror.status()["NS"]
    assert st["sent"] == "RED" and st["acked"] == "RED" and st["link"] == "CONNECTED"


# ============================================================ smoke-test runner (fake, fast)
def test_hw_smoke_hybrid_runs_with_fake_device():
    cfg = one_device_cfg()
    store = {}
    lines = []
    esp = FakeEsp()

    class FeedingPacer(hw_smoke.Pacer):
        def wait_until(self, sim_ms):
            t = store.get("esp32_a")
            if t is None:
                return
            if sim_ms == 200:
                t.feed(esp.hello(0), esp.sen(0, 0, False), esp.sen(0, 1, False))
            if sim_ms == 2000:
                t.feed(esp.sen(1900, 0, True))
            if sim_ms % 200 == 0:
                t.feed(esp.hb(sim_ms))

    status = hw_smoke.run_hybrid(cfg, 12, out=lines.append, transport_factory=fake_factory(store),
                                 pacer=FeedingPacer(realtime=False))
    text = "\n".join(lines)
    assert "Real detectors : NS:D1" in text and "Virtual        : 15 detectors" in text
    assert "NS: D1=ON" in text and "signal NS: RED_YELLOW" in text
    assert status["faults"] == () and status["real"]["NS:D1"]["occupied"] is True
    assert ["SIG", "0", "OFF"] in store["esp32_a"].commands()


def test_hw_smoke_light_runs_full_sequence_with_fake_device():
    cfg = one_device_cfg(sensors={})
    store = {}
    esp = FakeEsp()

    class P(hw_smoke.Pacer):
        def wait_until(self, sim_ms):
            if sim_ms == 100:
                store["esp32_a"].feed(esp.hello(0))
            elif sim_ms % 200 == 0:
                store["esp32_a"].feed(esp.hb(sim_ms))
    log = hw_smoke.run_light(cfg, out=lambda *_: None, transport_factory=fake_factory(store),
                             pacer=P(realtime=False), cycles=1)
    states = []
    for *_, s in log:
        if not states or states[-1] != s:
            states.append(s)
    # the pattern starts green at t=0, before the device has said HELLO, so the head
    # first receives RED_YELLOW; then the full sequence back to RED
    assert states[:6] == ["RED_YELLOW", "GREEN", "FLASHING_GREEN", "YELLOW", "ALL_RED", "RED"]


def test_hw_smoke_port_override_parsing(monkeypatch):
    seen = {}
    def fake_run_link(cfg, seconds):
        seen["cfg"] = cfg
        return {}
    monkeypatch.setattr(hw_smoke, "run_link", fake_run_link)
    assert hw_smoke.main(["--mode", "link", "--port", "COM7", "--seconds", "1"]) == 0
    assert seen["cfg"].devices["esp32_a"].port == "COM7"
    assert hw_smoke.main(["--mode", "link", "--port", "esp32_a=COM8", "--seconds", "1"]) == 0
    assert seen["cfg"].devices["esp32_a"].port == "COM8"


# ============================================================ import hygiene
def _imports(path):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    top = {n.module for n in tree.body if isinstance(n, ast.ImportFrom)} | \
          {a.name for n in tree.body if isinstance(n, ast.Import) for a in n.names}
    anywhere = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
               {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    return top, anywhere


def test_hardware_imports_serial_only_lazily_and_no_clock():
    top, anywhere = _imports(ROOT / "smarttraffic" / "hardware.py")
    assert "serial" not in top and "serial" in anywhere          # only inside SerialTransport.open
    assert not anywhere & {"time", "random", "threading", "pygame"}
    assert not any("controller" in (m or "") for m in anywhere)


def test_sensor_layer_still_hardware_free():
    for f in ("sensors.py", "hybrid_sensors.py"):
        _, anywhere = _imports(ROOT / "smarttraffic" / f)
        assert not any(x in (m or "") for m in anywhere for x in ("serial", "hardware", "time", "controller"))
