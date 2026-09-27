"""esp_protocol -- line protocol between Python and the ESP32 hardware nodes (Stage 9).

Pure encode/decode functions: no I/O, no clock, no serial import.

Frame format (ASCII, one message per line, NMEA-style checksum)
--------------------------------------------------------------
    $<FIELD>,<FIELD>,...*<CS>\\n

<CS> is the XOR of every byte between '$' and '*', as two upper-case hex
digits. A '\\r' before '\\n' is accepted. Fields never contain ',', '*' or '$'.
Maximum line length: MAX_LINE bytes.

Device -> host  (every message: TYPE, seq, device_ms, ...)
----------------------------------------------------------
    HELLO,seq,ms,proto,device_id,boot_id,n_sensors,n_signals
    SEN,seq,ms,channel,OCC|CLR        sensor state (edge, or a full-state report)
    HB,seq,ms                         heartbeat (default every 200 ms)
    ACK,seq,ms,channel,STATE          signal command applied
    ERR,seq,ms,text                   device-side problem (e.g. bad command)

seq is a 32-bit counter that restarts at 0 on every boot; ms is the device's
millis() (32-bit, wraps after ~49.7 days); boot_id is random hex chosen at boot.

Host -> device
--------------
    SIG,channel,STATE                 display STATE on physical signal `channel`
    QRY                               re-send HELLO and the state of every sensor

STATE is one of the Signals V2 states RED, RED_YELLOW, GREEN, FLASHING_GREEN,
YELLOW, ALL_RED, or OFF (all lamps dark; used only at shutdown). The ESP32
performs the FLASHING_GREEN blinking itself (see hardware/README.md).
"""
from dataclasses import dataclass
import re
from typing import List, Sequence, Union

from smarttraffic.signals import SignalState

__all__ = [
    "PROTOCOL_VERSION", "MAX_LINE", "U32", "ProtocolError", "SIGNAL_COMMAND_STATES",
    "checksum", "encode_frame", "decode_frame",
    "Hello", "SensorReport", "Heartbeat", "Ack", "DeviceError", "DeviceMessage",
    "parse_device_message", "encode_hello", "encode_sensor", "encode_heartbeat",
    "encode_ack", "encode_error", "encode_signal_command", "encode_query",
]

PROTOCOL_VERSION = 1
MAX_LINE = 120
U32 = 2 ** 32
SIGNAL_COMMAND_STATES = tuple(s.name for s in SignalState) + ("OFF",)

_FIELD_OK = re.compile(r"^[A-Za-z0-9_.\- ]*$")
_DEVICE_ID = re.compile(r"^[A-Za-z0-9_\-]{1,32}$")
_BOOT_ID = re.compile(r"^[0-9A-Fa-f]{1,8}$")


class ProtocolError(ValueError):
    """A line that is not a valid protocol frame/message."""


def checksum(body: str) -> str:
    cs = 0
    for b in body.encode("ascii"):
        cs ^= b
    return f"{cs:02X}"


def encode_frame(fields: Sequence[object]) -> bytes:
    parts = [str(f) for f in fields]
    for p in parts:
        if not _FIELD_OK.match(p):
            raise ProtocolError(f"field {p!r} contains characters not allowed in a frame")
    body = ",".join(parts)
    line = f"${body}*{checksum(body)}\n"
    if len(line) > MAX_LINE:
        raise ProtocolError("frame too long")
    return line.encode("ascii")


def decode_frame(line: Union[bytes, str]) -> List[str]:
    """Validate one line (with or without the trailing newline) and return its fields."""
    if isinstance(line, bytes):
        if len(line) > MAX_LINE + 2:
            raise ProtocolError("line too long")
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError:
            raise ProtocolError("non-ASCII bytes") from None
    line = line.rstrip("\r\n")
    if len(line) > MAX_LINE:
        raise ProtocolError("line too long")
    if not line.startswith("$"):
        raise ProtocolError("missing '$'")
    star = line.rfind("*")
    if star < 0 or len(line) - star != 3:
        raise ProtocolError("missing or malformed '*CS'")
    body, cs = line[1:star], line[star + 1:]
    if checksum(body) != cs.upper():
        raise ProtocolError(f"checksum mismatch ({cs} != {checksum(body)})")
    fields = body.split(",")
    if any("$" in f or "*" in f for f in fields) or not fields[0]:
        raise ProtocolError("bad field")
    return fields


# ------------------------------------------------------------- messages
@dataclass(frozen=True)
class Hello:
    seq: int
    device_ms: int
    proto: int
    device_id: str
    boot_id: str
    n_sensors: int
    n_signals: int


@dataclass(frozen=True)
class SensorReport:
    seq: int
    device_ms: int
    channel: int
    occupied: bool


@dataclass(frozen=True)
class Heartbeat:
    seq: int
    device_ms: int


@dataclass(frozen=True)
class Ack:
    seq: int
    device_ms: int
    channel: int
    state: str


@dataclass(frozen=True)
class DeviceError:
    seq: int
    device_ms: int
    text: str


DeviceMessage = Union[Hello, SensorReport, Heartbeat, Ack, DeviceError]


def _u32(s: str, what: str) -> int:
    if not s.isdigit() or len(s) > 10:
        raise ProtocolError(f"{what} must be an unsigned integer, got {s!r}")
    v = int(s)
    if v >= U32:
        raise ProtocolError(f"{what} out of 32-bit range")
    return v


def _small(s: str, what: str, hi: int = 255) -> int:
    v = _u32(s, what)
    if v > hi:
        raise ProtocolError(f"{what} out of range 0..{hi}")
    return v


_ARITY = {"HELLO": 8, "SEN": 5, "HB": 3, "ACK": 5, "ERR": 4}


def parse_device_message(line: Union[bytes, str]) -> DeviceMessage:
    f = decode_frame(line)
    kind = f[0]
    if kind not in _ARITY:
        raise ProtocolError(f"unknown message type {kind!r}")
    if len(f) != _ARITY[kind]:
        raise ProtocolError(f"{kind} needs {_ARITY[kind] - 1} fields, got {len(f) - 1}")
    seq, ms = _u32(f[1], "seq"), _u32(f[2], "device_ms")
    if kind == "HELLO":
        if not _DEVICE_ID.match(f[4]):
            raise ProtocolError(f"bad device id {f[4]!r}")
        if not _BOOT_ID.match(f[5]):
            raise ProtocolError(f"bad boot id {f[5]!r}")
        return Hello(seq, ms, _small(f[3], "proto"), f[4], f[5].upper(),
                     _small(f[6], "n_sensors"), _small(f[7], "n_signals"))
    if kind == "SEN":
        if f[4] not in ("OCC", "CLR"):
            raise ProtocolError(f"sensor state must be OCC or CLR, got {f[4]!r}")
        return SensorReport(seq, ms, _small(f[3], "channel"), f[4] == "OCC")
    if kind == "HB":
        return Heartbeat(seq, ms)
    if kind == "ACK":
        if f[4] not in SIGNAL_COMMAND_STATES:
            raise ProtocolError(f"unknown signal state {f[4]!r}")
        return Ack(seq, ms, _small(f[3], "channel"), f[4])
    return DeviceError(seq, ms, f[3])


# ------------------------------------------------------------- encoders
def encode_hello(seq, device_ms, device_id, boot_id, n_sensors, n_signals,
                 proto=PROTOCOL_VERSION) -> bytes:
    return encode_frame(["HELLO", seq, device_ms, proto, device_id, boot_id, n_sensors, n_signals])


def encode_sensor(seq, device_ms, channel, occupied: bool) -> bytes:
    return encode_frame(["SEN", seq, device_ms, channel, "OCC" if occupied else "CLR"])


def encode_heartbeat(seq, device_ms) -> bytes:
    return encode_frame(["HB", seq, device_ms])


def encode_ack(seq, device_ms, channel, state) -> bytes:
    return encode_frame(["ACK", seq, device_ms, channel, state])


def encode_error(seq, device_ms, text) -> bytes:
    return encode_frame(["ERR", seq, device_ms, text])


def encode_signal_command(channel: int, state: Union[SignalState, str]) -> bytes:
    """Host -> device: show `state` (a SignalState, or 'OFF') on physical signal `channel`."""
    name = state.name if isinstance(state, SignalState) else state
    if name not in SIGNAL_COMMAND_STATES:
        raise ProtocolError(f"unknown signal state {state!r}")
    if isinstance(channel, bool) or not isinstance(channel, int) or not 0 <= channel <= 255:
        raise ProtocolError(f"signal channel must be an int 0..255, got {channel!r}")
    return encode_frame(["SIG", channel, name])


def encode_query() -> bytes:
    return encode_frame(["QRY"])
