"""Tests for smarttraffic.esp_protocol (v2) and its parity with the ESP32 firmware and the
firmware model smarttraffic.esp_sim (verified demo hardware: 2 digital active-low IR sensors on
GPIO27 / GPIO26, discrete R/Y/G outputs on GPIO32 / GPIO33 / GPIO25)."""
import ast
import json
import re
from pathlib import Path

import pytest

from smarttraffic import esp_protocol as p
from smarttraffic import esp_sim
from smarttraffic.signals import Lamps, SignalState, lamps_for

ROOT = Path(__file__).resolve().parent.parent
FIRMWARE = ROOT / "hardware" / "esp32_smart_traffic" / "esp32_smart_traffic.ino"


# ------------------------------------------------------------- framing
def test_checksum_known_value():
    assert p.checksum("QRY") == f"{ord('Q') ^ ord('R') ^ ord('Y'):02X}"
    assert p.encode_query() == f"$QRY*{p.checksum('QRY')}\n".encode()


def test_decode_frame_accepts_crlf_and_lowercase_checksum():
    body = "HB,1,2"
    cs = p.checksum(body).lower()
    assert p.decode_frame(f"${body}*{cs}\r\n".encode()) == ["HB", "1", "2"]


@pytest.mark.parametrize("line", [
    b"HB,1,2*00\n",                       # no $
    b"$HB,1,2*00\n",                      # wrong checksum
    b"$HB,1,2\n",                         # no checksum
    b"$HB,1,2*0\n",                       # short checksum
    "$HB,1,é2*00\n".encode("utf-8"), # non-ASCII
    b"$" + b"A" * 200 + b"*00\n",         # too long
    b"$*00\n",                            # empty type
])
def test_decode_frame_rejects(line):
    with pytest.raises(p.ProtocolError):
        p.decode_frame(line)


def test_encode_rejects_forbidden_characters():
    for bad in ("a,b", "a*b", "a$b", "a\nb"):
        with pytest.raises(p.ProtocolError):
            p.encode_frame(["ERR", 1, 2, bad])


# ------------------------------------------------------------- device messages
def test_round_trip_every_device_message():
    cases = [
        (p.encode_hello(0, 10, "esp32_a", "BEEF1234", 1, 1), p.Hello(0, 10, p.PROTOCOL_VERSION, "esp32_a", "BEEF1234", 1, 1)),
        (p.encode_sensor(5, 123456, 0, True), p.SensorReport(5, 123456, 0, True)),
        (p.encode_sensor(6, 123457, 3, False), p.SensorReport(6, 123457, 3, False)),
        (p.encode_heartbeat(7, 4294967295), p.Heartbeat(7, 4294967295)),
        (p.encode_ack(8, 9, 0, "FLASHING_GREEN", "000"), p.Ack(8, 9, 0, "FLASHING_GREEN", "000")),
        (p.encode_ack(8, 9, 0, "RED_YELLOW", Lamps(True, True, False)), p.Ack(8, 9, 0, "RED_YELLOW", "110")),
        (p.encode_error(9, 10, "bad_state"), p.DeviceError(9, 10, "bad_state")),
    ]
    for line, msg in cases:
        assert p.parse_device_message(line) == msg


@pytest.mark.parametrize("fields", [
    ["NOPE", 1, 2],                                  # unknown type
    ["HB", 1],                                       # arity
    ["SEN", 1, 2, 0, "OCC", "X"],                    # arity
    ["SEN", 1, 2, 0, "MAYBE"],                       # bad state
    ["SEN", -1, 2, 0, "OCC"],                        # negative seq
    ["SEN", 1, 4294967296, 0, "OCC"],                # > u32
    ["SEN", 1, "12a", 0, "OCC"],                     # not a number
    ["SEN", 1, 2, 256, "OCC"],                       # channel range
    ["HELLO", 0, 1, 1, "bad id!", "AB", 1, 1],       # device id
    ["HELLO", 0, 1, 1, "esp32_a", "XYZ", 1, 1],      # boot id not hex
    ["ACK", 1, 2, 0, "BLUE", "100"],                 # unknown signal state
    ["ACK", 1, 2, 0, "RED"],                         # v1 ACK without lamps
    ["ACK", 1, 2, 0, "RED", "001"],                  # lamps impossible for RED
])
def test_parse_rejects_bad_messages(fields):
    body = ",".join(str(f) for f in fields)
    line = f"${body}*{p.checksum(body)}\n"
    with pytest.raises(p.ProtocolError):
        p.parse_device_message(line)


# ------------------------------------------------------------- host commands
EXPECTED_LAMPS = {"RED": "100", "RED_YELLOW": "110", "GREEN": "001", "YELLOW": "010", "ALL_RED": "100"}


@pytest.mark.parametrize("state", list(SignalState))
def test_signal_command_carries_the_logical_lamps_for_every_v2_state(state):
    for elapsed in (0, 250, 499, 500, 999, 1000, 2700):
        lamps = lamps_for(state, elapsed)
        f = p.decode_frame(p.encode_signal_command(0, state, lamps))
        assert f == ["SIG", "0", state.name, p.encode_lamps(lamps)]
        if state.name in EXPECTED_LAMPS:
            assert f[3] == EXPECTED_LAMPS[state.name]
        else:                                          # FLASHING_GREEN: green on 0-499, off 500-999 ms
            assert f[3] == ("001" if elapsed % 1000 < 500 else "000")


def test_signal_command_states_and_lamp_patterns():
    assert set(p.SIGNAL_COMMAND_STATES) == {s.name for s in SignalState} | {"OFF"}
    assert set(p.LAMP_PATTERNS) == set(p.SIGNAL_COMMAND_STATES)
    assert p.decode_frame(p.encode_signal_command(3, "OFF", "000")) == ["SIG", "3", "OFF", "000"]
    assert p.decode_frame(p.encode_signal_command(1, "ALL_RED", "100")) == ["SIG", "1", "ALL_RED", "100"]
    assert p.decode_lamps("110") == (True, True, False) and p.encode_lamps(Lamps(False, False, True)) == "001"


@pytest.mark.parametrize("state,channel,lamps", [
    ("PURPLE", 0, "100"), ("green", 0, "001"), (SignalState.RED, -1, "100"),
    (SignalState.RED, 256, "100"), (SignalState.RED, True, "100"), (SignalState.RED, "0", "100"),
    (SignalState.RED, 0, "001"),                  # lamps the state cannot show
    (SignalState.GREEN, 0, "000"),                # only FLASHING_GREEN may be dark
    (SignalState.RED, 0, "1000"), (SignalState.RED, 0, "1x0"), (SignalState.RED, 0, (1, 0, 0)),
    ("OFF", 0, "100"),
])
def test_signal_command_rejects(state, channel, lamps):
    with pytest.raises(p.ProtocolError):
        p.encode_signal_command(channel, state, lamps)


def test_signal_command_needs_lamps():
    with pytest.raises(TypeError):
        p.encode_signal_command(0, SignalState.RED)


# ------------------------------------------------------------- firmware parity
def _fw():
    return FIRMWARE.read_text(encoding="utf-8")


def test_firmware_state_names_match_protocol():
    names = re.search(r"STATE_NAMES\[\]\s*=\s*\{([^}]*)\}", _fw()).group(1)
    assert tuple(re.findall(r'"([A-Z_]+)"', names)) == p.SIGNAL_COMMAND_STATES


def test_firmware_protocol_version_and_device_id():
    fw = _fw()
    assert int(re.search(r"PROTOCOL_VERSION\s*=\s*(\d+)", fw).group(1)) == p.PROTOCOL_VERSION
    dev = re.search(r'DEVICE_ID\s*=\s*"([^"]+)"', fw).group(1)
    cfg = json.loads((ROOT / "config" / "hardware.json").read_text())
    assert dev in cfg["devices"]


def test_firmware_sensor_table_is_two_digital_active_low_inputs():
    fw = _fw()
    sensors = re.search(r"SENSORS\[\]\s*=\s*\{(.*?)\};", fw, re.S).group(1)
    rows = re.findall(r"\{\s*(\d+)\s*,\s*(true|false)\s*,\s*(\d+)\s*\}", sensors)
    assert [(int(a), b == "true", int(c)) for a, b, c in rows] == \
        [(pin, esp_sim.ACTIVE_LOW, esp_sim.DEBOUNCE_MS) for pin in esp_sim.SENSOR_PINS] == \
        [(27, True, 20), (26, True, 20)]
    assert "digitalRead(SENSORS[i].pin)" in fw and "level == LOW" in fw        # active-low decode
    assert "INPUT_PULLUP" in fw                                                # unplugged = CLEAR
    assert "analogRead" not in fw and "threshold" not in fw.split("// ===")[1]   # old analog GPIO34 path gone


def test_firmware_signal_table_is_three_discrete_outputs():
    fw = _fw()
    signals = re.search(r"SIGNALS\[\]\s*=\s*\{(.*?)\};", fw, re.S).group(1)
    rows = re.findall(r"\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(true|false)\s*\}", signals)
    assert [((int(g), int(y), int(r)), h == "true") for g, y, r, h in rows] == \
        [(trio, esp_sim.LAMP_ACTIVE_HIGH) for trio in esp_sim.SIGNAL_PINS] == [((25, 33, 32), True)]
    assert "NeoPixel" not in fw.replace("NeoPixel head (obsolete)", "") and "#include" not in fw
    assert "digitalWrite" in fw and "pinMode(SIGNALS[i].redPin, OUTPUT)" in fw


def test_firmware_lamp_patterns_match_protocol():
    fw = _fw()
    a = re.findall(r'"([01]{3})"', re.search(r"STATE_LAMPS_A\[\]\s*=\s*\{([^}]*)\}", fw).group(1))
    b = re.findall(r'"([01]{3})"', re.search(r"STATE_LAMPS_B\[\]\s*=\s*\{([^}]*)\}", fw).group(1))
    got = {name: tuple(sorted({x, y})) for name, x, y in zip(p.SIGNAL_COMMAND_STATES, a, b)}
    assert got == {k: tuple(sorted(v)) for k, v in p.LAMP_PATTERNS.items()}


def test_firmware_runs_no_signal_timing_only_the_fail_safe():
    fw = _fw()
    assert "FLASH_PERIOD_MS" not in fw and "sigSince" not in fw               # no local flashing-green timer
    for name, val in (("FAULT_BLINK_PERIOD_MS", esp_sim.FAULT_BLINK_PERIOD_MS),
                      ("FAULT_BLINK_ON_MS", esp_sim.FAULT_BLINK_ON_MS),
                      ("SIGNAL_TIMEOUT_MS", esp_sim.SIGNAL_TIMEOUT_MS), ("HEARTBEAT_MS", esp_sim.HEARTBEAT_MS),
                      ("SAMPLE_MS", esp_sim.SAMPLE_MS)):
        assert int(re.search(name + r"\s*=\s*(\d+)", fw).group(1)) == val, name
    assert "lampR[ch] = lamps[0] == '1'" in fw and "lampG[ch] = lamps[2] == '1'" in fw


def test_active_low_decoding():
    assert esp_sim.decode_sensor_level(0) is True            # LOW  = car detected
    assert esp_sim.decode_sensor_level(1) is False           # HIGH = clear
    assert esp_sim.decode_sensor_level(0, active_low=False) is False
    for bad in (2, -1, "0", None):
        with pytest.raises(ValueError):
            esp_sim.decode_sensor_level(bad)


def test_led_wiring_test_sketch_is_kept_separate():
    t = (ROOT / "hardware" / "esp32_led_test" / "esp32_led_test.ino").read_text(encoding="utf-8")
    assert "GREEN_PIN  = 25" in t and "YELLOW_PIN = 33" in t and "RED_PIN    = 32" in t
    assert "OWN fixed timing" in t


def test_protocol_module_is_pure():
    tree = ast.parse((ROOT / "smarttraffic" / "esp_protocol.py").read_text(encoding="utf-8"))
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert not mods & {"serial", "time", "socket", "threading", "random"}
