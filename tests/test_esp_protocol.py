"""Tests for smarttraffic.esp_protocol and its parity with the ESP32 firmware (Stage 9)."""
import ast
import json
import re
from pathlib import Path

import pytest

from smarttraffic import esp_protocol as p
from smarttraffic.signals import SignalState

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
        (p.encode_hello(0, 10, "esp32_a", "BEEF1234", 1, 1), p.Hello(0, 10, 1, "esp32_a", "BEEF1234", 1, 1)),
        (p.encode_sensor(5, 123456, 0, True), p.SensorReport(5, 123456, 0, True)),
        (p.encode_sensor(6, 123457, 3, False), p.SensorReport(6, 123457, 3, False)),
        (p.encode_heartbeat(7, 4294967295), p.Heartbeat(7, 4294967295)),
        (p.encode_ack(8, 9, 0, "FLASHING_GREEN"), p.Ack(8, 9, 0, "FLASHING_GREEN")),
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
    ["ACK", 1, 2, 0, "BLUE"],                        # unknown signal state
])
def test_parse_rejects_bad_messages(fields):
    body = ",".join(str(f) for f in fields)
    line = f"${body}*{p.checksum(body)}\n"
    with pytest.raises(p.ProtocolError):
        p.parse_device_message(line)


# ------------------------------------------------------------- host commands
@pytest.mark.parametrize("state", list(SignalState))
def test_signal_command_for_every_v2_state(state):
    line = p.encode_signal_command(0, state)
    assert p.decode_frame(line) == ["SIG", "0", state.name]


def test_signal_command_states_cover_v2_and_off():
    assert set(p.SIGNAL_COMMAND_STATES) == {s.name for s in SignalState} | {"OFF"}
    assert p.decode_frame(p.encode_signal_command(3, "OFF")) == ["SIG", "3", "OFF"]
    assert p.decode_frame(p.encode_signal_command(1, "ALL_RED")) == ["SIG", "1", "ALL_RED"]


@pytest.mark.parametrize("state,channel", [("PURPLE", 0), ("green", 0), (SignalState.RED, -1),
                                           (SignalState.RED, 256), (SignalState.RED, True),
                                           (SignalState.RED, "0")])
def test_signal_command_rejects(state, channel):
    with pytest.raises(p.ProtocolError):
        p.encode_signal_command(channel, state)


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


def test_firmware_current_hardware_tables():
    fw = _fw()
    sensors = re.search(r"SENSORS\[\]\s*=\s*\{(.*?)\};", fw, re.S).group(1)
    assert re.search(r"\{\s*34\s*,\s*2000\s*,\s*true\s*,", sensors)          # GPIO34, >2000 = occupied
    signals = re.search(r"SIGNALS\[\]\s*=\s*\{(.*?)\};", fw, re.S).group(1)
    assert re.search(r"\{\s*13\s*,\s*3\s*,\s*0\s*,\s*1\s*,\s*2\s*,", signals)  # GPIO13, 3 LEDs, G0 Y1 R2


def test_firmware_flash_timing_matches_signals_v2():
    from smarttraffic import signals
    fw = _fw()
    assert int(re.search(r"FLASH_PERIOD_MS\s*=\s*(\d+)", fw).group(1)) == signals.FLASH_PERIOD_MS
    assert int(re.search(r"FLASH_ON_MS\s*=\s*(\d+)", fw).group(1)) == signals.FLASH_ON_MS


def test_protocol_module_is_pure():
    tree = ast.parse((ROOT / "smarttraffic" / "esp_protocol.py").read_text(encoding="utf-8"))
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert not mods & {"serial", "time", "socket", "threading", "random"}
