"""hardware -- ESP32 links, real detectors and physical signal mirrors (Stage 9).

Everything is driven by config/hardware.json (see load_hardware_config):

    devices : {device_id: {port, baud}}                       one entry per ESP32
    sensors : {"NS:D1": {device, channel}, ...}              real detectors; all others virtual
              (demo hardware: NS:D1 -> channel 0 = GPIO27, NS:D2 -> channel 1 = GPIO26,
              both digital active-low IR modules; the firmware reports OCC/CLR)
    signals : {"NS": {device, channel}, ...}                 physical heads mirroring a movement
    link    : optional timing overrides (LinkSettings)

Pieces
------
* Transport / SerialTransport -- byte pipe; pyserial is imported only in open().
* EspLink       -- one per device: connect/reconnect, framing, checksums, sequence
                   checks, device reset and millis() wraparound, time base, health.
* HardwareDetectorSource -- DetectorSource for the mapped real detectors.
* SignalMirror  -- sends the Python SignalBank state AND lamps of mapped movements to
                   physical heads (protocol v2: every lamp change, incl. each flashing-green
                   edge). Read-only on the SignalBank; a head is NOT a logical signal. The
                   ESP32 runs no signal timing: it only sets its R/Y/G outputs.
* build_hardware -- wires config + VehicleModel into a HybridDetectorSource (for
                   SensorBank) and a SignalMirror.

Time base
---------
The library never reads a clock. Each link is pumped with the current SIMULATION
time (ms). A message read during pump(now) is taken as received at `now`.
Device timestamps (millis) are mapped to simulation time with
    offset = min over the last N messages of (receive_sim_ms - device_ms)
because transport delay is never negative, so the minimum converges to the true
offset plus the smallest latency. An edge is placed at device_ms + offset; if
that instant was already handed to SensorBank (late), it is placed at the next
millisecond and counted as `late_edges`. In a live demo the runner
(hw_smoke) keeps simulation time in step with the wall clock; in tests, fake
transports and chosen poll times make everything deterministic.

Faults
------
A real detector is FAULTED while its device link is not CONNECTED (never opened,
disconnected, stale heartbeat, wrong device, protocol mismatch) or its current
state has not been reported since the last HELLO. A faulted detector produces
no new edges: its last confirmed state is held, nothing is invented, and it is
never switched to virtual. SensorBank.detector_fault() exposes this.
"""
from collections import Counter, deque
from dataclasses import dataclass, field
import enum
import json
import math
from pathlib import Path
from typing import (Callable, Deque, Dict, Iterable, List, Mapping, Optional, Protocol,
                    Sequence, Tuple, Union)

from smarttraffic import esp_protocol as proto
from smarttraffic.conflicts import MOVES, validate_movement
from smarttraffic.hybrid_sensors import (HybridDetectorSource, SensorAssignmentError,
                                         format_sensor_id, parse_sensor_id)
from smarttraffic.sensors import DETECTORS, SENSOR_IDS, DetectorEdge, VirtualDetectors
from smarttraffic.signals import SignalState

__all__ = [
    "HardwareConfigError", "LinkSettings", "DeviceConfig", "Target", "HardwareConfig",
    "load_hardware_config", "parse_hardware_config", "Transport", "SerialTransport",
    "LinkState", "EspLink", "HardwareHub", "HardwareDetectorSource", "SignalMirror",
    "HardwareSystem", "build_hardware", "DEFAULT_CONFIG_FILE", "real_only_movements",
]

SensorId = Tuple[str, str]
DEFAULT_CONFIG_FILE = Path(__file__).resolve().parent.parent / "config" / "hardware.json"
_DEVICE_ID_CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-"


# ================================================================ config
class HardwareConfigError(ValueError):
    """config/hardware.json is malformed or inconsistent."""


@dataclass(frozen=True)
class LinkSettings:
    heartbeat_timeout_ms: int = 1500     # no message for this long -> STALE (fault)
    reconnect_interval_ms: int = 2000    # retry opening a closed port this often
    query_interval_ms: int = 1000        # re-send QRY while waiting for HELLO / after gaps
    signal_refresh_ms: int = 1000        # re-send the current signal state this often
    time_sync_window: int = 64           # messages used for the min-offset estimate
    reset_backjump_ms: int = 5000        # device time jumping back more than this = reset

    def __post_init__(self):
        for k, v in self.__dict__.items():
            if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
                raise HardwareConfigError(f"link.{k} must be a positive int, got {v!r}")


@dataclass(frozen=True)
class DeviceConfig:
    device_id: str
    port: str
    baud: int = 115200


@dataclass(frozen=True)
class Target:
    device: str
    channel: int


@dataclass(frozen=True)
class HardwareConfig:
    devices: Mapping[str, DeviceConfig]
    sensors: Mapping[SensorId, Target]          # real detectors only
    signals: Mapping[str, Target]               # movement -> physical head
    link: LinkSettings = field(default_factory=LinkSettings)

    @property
    def real_only_movements(self) -> Tuple[str, ...]:
        return real_only_movements(self)

    def with_port(self, device_id: str, port: str) -> "HardwareConfig":
        if device_id not in self.devices:
            raise HardwareConfigError(f"unknown device {device_id!r}")
        devs = dict(self.devices)
        devs[device_id] = DeviceConfig(device_id, port, devs[device_id].baud)
        return HardwareConfig(devs, self.sensors, self.signals, self.link)


def real_only_movements(config: "HardwareConfig") -> Tuple[str, ...]:
    """Movements whose detectors are ALL real (demo config: NS, with NS:D1 and NS:D2 real).

    In a hardware run such a movement gets its demand ONLY from the physical sensors: its
    virtual vehicles are not generated (runner.simulate(exclude_movements=...)), so no virtual
    vehicle is drawn on it and no virtual detector event can exist for it. A movement with only
    some detectors real keeps its virtual vehicles (the virtual edges of the real detectors are
    still dropped by HybridDetectorSource)."""
    real = set(config.sensors)
    return tuple(m for m in MOVES if all((m, d) in real for d in DETECTORS))


def _no_duplicate_keys(pairs):
    out = {}
    for k, v in pairs:
        if k in out:
            raise HardwareConfigError(f"duplicate key {k!r} in hardware config")
        out[k] = v
    return out


def _target(raw, where: str, devices) -> Target:
    if not isinstance(raw, dict) or set(raw) != {"device", "channel"}:
        raise HardwareConfigError(f"{where}: needs exactly 'device' and 'channel'")
    dev, ch = raw["device"], raw["channel"]
    if dev not in devices:
        raise HardwareConfigError(f"{where}: unknown device {dev!r} (defined: {sorted(devices)})")
    if isinstance(ch, bool) or not isinstance(ch, int) or not 0 <= ch <= 255:
        raise HardwareConfigError(f"{where}: channel must be an int 0..255, got {ch!r}")
    return Target(dev, ch)


def parse_hardware_config(raw: Mapping) -> HardwareConfig:
    if not isinstance(raw, Mapping):
        raise HardwareConfigError("hardware config must be an object")
    allowed = {"_about", "devices", "sensors", "signals", "link"}
    if set(raw) - allowed or "devices" not in raw:
        raise HardwareConfigError(f"top-level keys must be a subset of {sorted(allowed)} incl. 'devices'")
    devices: Dict[str, DeviceConfig] = {}
    for dev_id, d in raw["devices"].items():
        if not (1 <= len(dev_id) <= 32 and all(c in _DEVICE_ID_CHARS for c in dev_id)):
            raise HardwareConfigError(f"bad device id {dev_id!r}")
        if not isinstance(d, dict) or not set(d) <= {"port", "baud"} or "port" not in d:
            raise HardwareConfigError(f"device {dev_id}: needs 'port' (and optional 'baud')")
        if not isinstance(d["port"], str) or not d["port"].strip():
            raise HardwareConfigError(f"device {dev_id}: port must be a non-empty string")
        baud = d.get("baud", 115200)
        if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
            raise HardwareConfigError(f"device {dev_id}: baud must be a positive int")
        devices[dev_id] = DeviceConfig(dev_id, d["port"].strip(), baud)
    used = {}
    sensors: Dict[SensorId, Target] = {}
    for key, t in raw.get("sensors", {}).items():
        try:
            sid = parse_sensor_id(key)
        except SensorAssignmentError as e:
            raise HardwareConfigError(f"sensors: {e}") from None
        tgt = _target(t, f"sensors.{key}", devices)
        if ("S", tgt) in used:
            raise HardwareConfigError(f"sensors.{key}: {tgt.device} sensor channel {tgt.channel} "
                                      f"already used by {used[('S', tgt)]}")
        used[("S", tgt)] = key
        sensors[sid] = tgt
    signals: Dict[str, Target] = {}
    for m, t in raw.get("signals", {}).items():
        try:
            validate_movement(m)
        except ValueError as e:
            raise HardwareConfigError(f"signals: {e}") from None
        tgt = _target(t, f"signals.{m}", devices)
        if ("G", tgt) in used:
            raise HardwareConfigError(f"signals.{m}: {tgt.device} signal channel {tgt.channel} "
                                      f"already used by {used[('G', tgt)]}")
        used[("G", tgt)] = m
        signals[m] = tgt
    link_raw = raw.get("link", {})
    if not isinstance(link_raw, dict) or set(link_raw) - set(LinkSettings.__dataclass_fields__):
        raise HardwareConfigError(f"link: allowed keys {sorted(LinkSettings.__dataclass_fields__)}")
    return HardwareConfig(devices, sensors, signals, LinkSettings(**link_raw))


def load_hardware_config(path: Union[str, Path] = DEFAULT_CONFIG_FILE) -> HardwareConfig:
    with open(path, encoding="utf-8") as f:
        try:
            raw = json.load(f, object_pairs_hook=_no_duplicate_keys)
        except json.JSONDecodeError as e:
            raise HardwareConfigError(f"{path}: invalid JSON ({e})") from None
    return parse_hardware_config(raw)


# ============================================================= transports
class Transport(Protocol):
    @property
    def is_open(self) -> bool: ...
    def open(self) -> None: ...
    def close(self) -> None: ...
    def read_available(self) -> bytes: ...
    def write(self, data: bytes) -> None: ...


class SerialTransport:
    """USB serial via pyserial (imported only when opened). Non-blocking reads."""

    def __init__(self, port: str, baud: int = 115200):
        self.port, self.baud = port, baud
        self._ser = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        import serial                                   # pyserial, only for real hardware
        s = serial.Serial()
        s.port, s.baudrate, s.timeout, s.write_timeout = self.port, self.baud, 0, 0.2
        s.dtr, s.rts = False, False                     # avoid resetting the board where possible
        s.open()
        self._ser = s

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def read_available(self) -> bytes:
        n = self._ser.in_waiting
        return self._ser.read(n) if n else b""

    def write(self, data: bytes) -> None:
        self._ser.write(data)


# ================================================================== link
class LinkState(enum.Enum):
    CLOSED = "CLOSED"                        # close() called
    DISCONNECTED = "DISCONNECTED"            # port not open (never opened, or lost)
    WAITING_HELLO = "WAITING_HELLO"          # port open, device not identified yet
    CONNECTED = "CONNECTED"                  # healthy
    STALE = "STALE"                          # no message for heartbeat_timeout_ms
    WRONG_DEVICE = "WRONG_DEVICE"            # HELLO from a different device id
    PROTOCOL_MISMATCH = "PROTOCOL_MISMATCH"  # HELLO with another protocol version


class EspLink:
    """Connection to one ESP32. Call pump(now_ms) with non-decreasing simulation time."""

    def __init__(self, device_id: str, transport: Transport, settings: LinkSettings = LinkSettings()):
        self.device_id = device_id
        self.transport = transport
        self.settings = settings
        self.state = LinkState.DISCONNECTED
        self.boot_id: Optional[str] = None
        self.n_sensors = 0
        self.n_signals = 0
        self.hello_count = 0
        self.stats: Counter = Counter()
        self.errors: Deque[str] = deque(maxlen=50)
        self.synced_channels = set()
        self.last_ack: Dict[int, Tuple[str, int]] = {}        # channel -> (state, host ms)
        self.last_ack_lamps: Dict[int, str] = {}              # channel -> lamps "RYG"
        self._buf = bytearray()
        self._last_pump: Optional[int] = None
        self._last_open_try: Optional[int] = None
        self._last_query: Optional[int] = None
        self._last_rx: Optional[int] = None
        self._last_seq: Optional[int] = None
        self._last_dev_raw: Optional[int] = None
        self._wraps = 0
        self._offsets: Deque[int] = deque(maxlen=settings.time_sync_window)
        self._reports: List[Tuple[int, int, bool]] = []     # (host ms, channel, occupied)

    # ---------------------------------------------------------- queries
    def healthy(self) -> bool:
        return self.state is LinkState.CONNECTED

    @property
    def offset_ms(self) -> Optional[int]:
        return min(self._offsets) if self._offsets else None

    def to_host_ms(self, device_ms_unwrapped: int) -> int:
        if not self._offsets:
            raise RuntimeError("no time base yet (no message received)")
        return device_ms_unwrapped + min(self._offsets)

    def status(self) -> Dict[str, object]:
        return {"device": self.device_id, "state": self.state.name, "boot_id": self.boot_id,
                "offset_ms": self.offset_ms, "hello_count": self.hello_count,
                "stats": dict(self.stats), "last_errors": list(self.errors)[-5:]}

    # ------------------------------------------------------------- I/O
    def pump(self, now_ms: int) -> None:
        """Open/reconnect if needed, read everything available, update health."""
        if self.state is LinkState.CLOSED:
            return
        if self._last_pump is not None and now_ms < self._last_pump:
            raise ValueError(f"link time went backwards ({now_ms} < {self._last_pump})")
        if now_ms == self._last_pump:
            return
        self._last_pump = now_ms
        if not self.transport.is_open:
            if self._last_open_try is None or now_ms - self._last_open_try >= self.settings.reconnect_interval_ms:
                self._last_open_try = now_ms
                try:
                    self.transport.open()
                except Exception as e:                  # port missing / busy
                    self.stats["open_failures"] += 1
                    self._error(f"open failed: {e}")
                    self.state = LinkState.DISCONNECTED
                    return
                self.stats["opens"] += 1
                self._buf.clear()
                self.state = LinkState.WAITING_HELLO
                self._send_query(now_ms, force=True)
            else:
                return
        try:
            data = self.transport.read_available()
        except Exception as e:
            self._drop(f"read failed: {e}")
            return
        if data:
            self._buf.extend(data)
            *lines, rest = bytes(self._buf).split(b"\n")
            self._buf = bytearray(rest)
            if len(self._buf) > proto.MAX_LINE + 2:        # garbage without newline
                self.stats["malformed"] += 1
                self._error("discarded over-long partial line")
                self._buf.clear()
            for line in lines:
                if line.strip():
                    self._line(line, now_ms)
        if self.state is LinkState.CONNECTED and self._last_rx is not None \
                and now_ms - self._last_rx > self.settings.heartbeat_timeout_ms:
            self.state = LinkState.STALE
            self.stats["stale"] += 1
        if self.state is LinkState.WAITING_HELLO:
            self._send_query(now_ms)

    def send(self, data: bytes) -> bool:
        if not self.transport.is_open or self.state in (LinkState.CLOSED, LinkState.WRONG_DEVICE,
                                                        LinkState.PROTOCOL_MISMATCH):
            return False
        try:
            self.transport.write(data)
            self.stats["tx"] += 1
            return True
        except Exception as e:
            self._drop(f"write failed: {e}")
            return False

    def take_sensor_reports(self) -> List[Tuple[int, int, bool]]:
        """Sensor reports since the last call as (host_ms, channel, occupied), in arrival order."""
        out = list(self._reports)
        self._reports.clear()
        return out

    def close(self) -> None:
        try:
            self.transport.close()
        finally:
            self.state = LinkState.CLOSED

    # -------------------------------------------------------- internals
    def _error(self, text: str) -> None:
        self.errors.append(text)

    def _drop(self, reason: str) -> None:
        self.stats["disconnects"] += 1
        self._error(reason)
        try:
            self.transport.close()
        except Exception:
            pass
        self.state = LinkState.DISCONNECTED
        self._buf.clear()

    def _send_query(self, now_ms: int, force: bool = False) -> None:
        if force or self._last_query is None or now_ms - self._last_query >= self.settings.query_interval_ms:
            self._last_query = now_ms
            self.send(proto.encode_query())

    def _line(self, line: bytes, now_ms: int) -> None:
        try:
            msg = proto.parse_device_message(line)
        except proto.ProtocolError as e:
            self.stats["malformed"] += 1
            self._error(f"malformed: {e}")
            return
        self.stats["rx"] += 1
        if isinstance(msg, proto.Hello):
            if msg.proto != proto.PROTOCOL_VERSION:
                self.state = LinkState.PROTOCOL_MISMATCH
                self._error(f"protocol {msg.proto} != {proto.PROTOCOL_VERSION}")
                return
            if msg.device_id != self.device_id:
                self.state = LinkState.WRONG_DEVICE
                self._error(f"expected device {self.device_id!r}, port has {msg.device_id!r}")
                return
            if msg.boot_id != self.boot_id:
                if self.boot_id is not None:
                    self.stats["device_resets"] += 1
                self.boot_id = msg.boot_id
                self._last_seq = None
                self._last_dev_raw = None
                self._wraps = 0
                self._offsets.clear()
            self.synced_channels = set()
            self.n_sensors, self.n_signals = msg.n_sensors, msg.n_signals
            self.hello_count += 1
            self.state = LinkState.CONNECTED
        elif self.state not in (LinkState.CONNECTED, LinkState.STALE):
            self.stats["before_hello"] += 1
            self._send_query(now_ms)
            return
        # ---- sequence ----
        if self._last_seq is not None and msg.seq <= self._last_seq:
            back = self._last_dev_raw is not None and msg.device_ms + self.settings.reset_backjump_ms < self._last_dev_raw \
                and self._last_dev_raw - msg.device_ms < proto.U32 // 2
            if back:                                   # rebooted and the HELLO was lost
                self.stats["suspected_resets"] += 1
                self._error("device time and sequence went back: waiting for HELLO")
                self.state = LinkState.WAITING_HELLO
                self._send_query(now_ms, force=True)
            else:
                self.stats["duplicates"] += 1
            return
        if self._last_seq is not None and msg.seq > self._last_seq + 1:
            self.stats["lost_messages"] += msg.seq - self._last_seq - 1
            self._send_query(now_ms)                   # re-sync sensor states
        self._last_seq = msg.seq
        # ---- device time (unwrap millis) ----
        # A report may carry a slightly OLDER timestamp than the previous message
        # (the firmware stamps an edge at the start of its debounce), so only a
        # jump of more than half the 32-bit range counts as a millis() wraparound.
        raw = msg.device_ms
        epoch = self._wraps
        if self._last_dev_raw is None:
            self._last_dev_raw = raw
        elif raw < self._last_dev_raw and self._last_dev_raw - raw > proto.U32 // 2:
            self._wraps += 1                          # wrapped forward
            epoch = self._wraps
            self._last_dev_raw = raw
        elif raw > self._last_dev_raw and raw - self._last_dev_raw > proto.U32 // 2:
            epoch = self._wraps - 1                   # late message from before the wrap
        elif raw > self._last_dev_raw:
            self._last_dev_raw = raw
        dev = epoch * proto.U32 + raw
        self._offsets.append(now_ms - dev)
        self._last_rx = now_ms
        if self.state is LinkState.STALE:
            self.state = LinkState.CONNECTED
        # ---- payload ----
        if isinstance(msg, proto.SensorReport):
            if msg.channel >= self.n_sensors:
                self.stats["unknown_channel"] += 1
                self._error(f"sensor channel {msg.channel} >= {self.n_sensors}")
                return
            # mapped at arrival, so a later device reset cannot re-map it with a new time base
            self._reports.append((self.to_host_ms(dev), msg.channel, msg.occupied))
            self.synced_channels.add(msg.channel)
        elif isinstance(msg, proto.Ack):
            self.last_ack[msg.channel] = (msg.state, now_ms)
            self.last_ack_lamps[msg.channel] = msg.lamps
        elif isinstance(msg, proto.DeviceError):
            self.stats["device_errors"] += 1
            self._error(f"device: {msg.text}")


class HardwareHub:
    """Owns one EspLink per configured device."""

    def __init__(self, config: HardwareConfig,
                 transport_factory: Optional[Callable[[DeviceConfig], Transport]] = None):
        factory = transport_factory or (lambda d: SerialTransport(d.port, d.baud))
        self.config = config
        self.links: Dict[str, EspLink] = {d: EspLink(d, factory(cfg), config.link)
                                          for d, cfg in config.devices.items()}

    def link(self, device_id: str) -> EspLink:
        if device_id not in self.links:
            raise HardwareConfigError(f"unknown device {device_id!r}")
        return self.links[device_id]

    def pump(self, now_ms: int) -> None:
        for l in self.links.values():
            l.pump(now_ms)

    def status(self) -> Dict[str, Dict[str, object]]:
        return {d: l.status() for d, l in self.links.items()}

    def close(self) -> None:
        for l in self.links.values():
            l.close()


# ================================================== real detector source
class HardwareDetectorSource:
    """DetectorSource for the real detectors in `mapping` {sensor_id: Target}."""

    def __init__(self, hub: HardwareHub, mapping: Mapping[SensorId, Target], start_ms: int):
        if not mapping:
            raise HardwareConfigError("HardwareDetectorSource needs at least one mapped sensor")
        self._hub = hub
        self._map: Dict[SensorId, Target] = {}
        self._rev: Dict[Tuple[str, int], SensorId] = {}
        for sid, t in mapping.items():
            if sid not in SENSOR_IDS:
                raise HardwareConfigError(f"unknown sensor id {sid!r}")
            hub.link(t.device)
            if (t.device, t.channel) in self._rev:
                raise HardwareConfigError(f"{t.device} channel {t.channel} mapped twice")
            self._map[sid] = t
            self._rev[(t.device, t.channel)] = sid
        self.sensor_ids = tuple(s for s in SENSOR_IDS if s in self._map)
        self._devices = sorted({t.device for t in self._map.values()})
        self._now = start_ms
        self._state: Dict[SensorId, bool] = {sid: False for sid in self.sensor_ids}
        self._last_t: Dict[SensorId, int] = {sid: start_ms for sid in self.sensor_ids}
        self._pending: List[Tuple[int, int, SensorId, bool]] = []
        self._counter = 0
        self.stats: Counter = Counter()

    @property
    def now_ms(self) -> int:
        return self._now

    def initial_occupancy(self) -> Dict[SensorId, bool]:
        return {sid: False for sid in self.sensor_ids}     # unknown until reported (see faults)

    def poll(self, until_ms: int) -> List[DetectorEdge]:
        if until_ms < self._now:
            raise ValueError(f"cannot poll backwards ({until_ms} < {self._now})")
        for dev in self._devices:
            link = self._hub.link(dev)
            link.pump(until_ms)
            for host_ms, ch, occ in link.take_sensor_reports():
                sid = self._rev.get((dev, ch))
                if sid is None:
                    self.stats["unmapped_reports"] += 1
                    continue
                t = host_ms
                if t <= self._now:
                    self.stats["late_edges"] += 1
                    t = self._now + 1
                t = max(t, self._last_t[sid])
                self._counter += 1
                self._pending.append((t, self._counter, sid, occ))
                self._last_t[sid] = t
        due = sorted(p for p in self._pending if p[0] <= until_ms)
        self._pending = [p for p in self._pending if p[0] > until_ms]
        edges: List[DetectorEdge] = []
        for t, _, sid, occ in due:
            if occ == self._state[sid]:
                self.stats["repeated_state"] += 1           # full-state report or duplicate
                continue
            self._state[sid] = occ
            edges.append(DetectorEdge(t, sid[0], sid[1], occ))
            self.stats["edges"] += 1
        self._now = until_ms
        edges.sort(key=lambda e: (e.time_ms, MOVES.index(e.movement), DETECTORS.index(e.detector)))
        return edges

    def faults(self) -> Tuple[SensorId, ...]:
        out = []
        for sid in self.sensor_ids:
            t = self._map[sid]
            link = self._hub.link(t.device)
            if not link.healthy() or t.channel not in link.synced_channels:
                out.append(sid)
        return tuple(out)

    def status(self) -> Dict[str, Dict[str, object]]:
        f = set(self.faults())
        return {format_sensor_id(sid): {"device": t.device, "channel": t.channel,
                                        "occupied": self._state[sid], "fault": sid in f}
                for sid, t in self._map.items()}


# ======================================================== signal mirror
class SignalMirror:
    """Shows the SignalBank lamps of mapped movements on physical heads (read-only).

    Every command carries the logical state and its lamps (signals.lamps_for at the
    current simulation time), so the head is an exact copy of the logical signal:
    RED / ALL_RED -> red, RED_YELLOW -> red + yellow, GREEN -> green, FLASHING_GREEN ->
    green ON/OFF on the logical flash edges, YELLOW -> yellow."""

    def __init__(self, hub: HardwareHub, mapping: Mapping[str, Target]):
        self._hub = hub
        self._map: Dict[str, Target] = {}
        for m, t in mapping.items():
            validate_movement(m)
            hub.link(t.device)
            self._map[m] = t
        self._sent: Dict[str, Optional[str]] = {m: None for m in self._map}
        self._sent_lamps: Dict[str, Optional[str]] = {m: None for m in self._map}
        self._sent_ms: Dict[str, Optional[int]] = {m: None for m in self._map}
        self._hello_seen: Dict[str, int] = {m: -1 for m in self._map}
        self.log: List[Tuple[int, str, str, int, str, str]] = []   # (ms, movement, device, channel, state, lamps)

    @property
    def movements(self) -> Tuple[str, ...]:
        return tuple(self._map)

    def update(self, signals) -> List[Tuple[str, str]]:
        """Call after SignalBank.advance(). Sends a command on every state or lamp change
        (flashing-green edges included), after a (re)connect, and every signal_refresh_ms.
        Returns [(movement, state)] sent."""
        now = round(signals.now * 1000)
        sent = []
        for m, t in self._map.items():
            link = self._hub.link(t.device)
            link.pump(now)
            if not link.healthy():
                self._sent[m] = self._sent_lamps[m] = None      # re-send once the link is back
                continue
            state = signals.state(m).name
            lamps = proto.encode_lamps(signals.lamp_state(m))
            due = (self._sent[m] != state or self._sent_lamps[m] != lamps
                   or link.hello_count != self._hello_seen[m]
                   or now - self._sent_ms[m] >= self._hub.config.link.signal_refresh_ms)
            if due and link.send(proto.encode_signal_command(t.channel, state, lamps)):
                self._sent[m], self._sent_lamps[m], self._sent_ms[m] = state, lamps, now
                self._hello_seen[m] = link.hello_count
                self.log.append((now, m, t.device, t.channel, state, lamps))
                sent.append((m, state))
        return sent

    def blackout(self) -> None:
        """Clean shutdown: turn the mapped physical heads dark (OFF)."""
        for m, t in self._map.items():
            link = self._hub.link(t.device)
            if link.transport.is_open:
                link.send(proto.encode_signal_command(t.channel, "OFF", "000"))

    def status(self) -> Dict[str, Dict[str, object]]:
        out = {}
        for m, t in self._map.items():
            link = self._hub.link(t.device)
            out[m] = {"device": t.device, "channel": t.channel, "sent": self._sent[m],
                      "lamps": self._sent_lamps[m],
                      "acked": link.last_ack.get(t.channel, (None, None))[0],
                      "acked_lamps": link.last_ack_lamps.get(t.channel),
                      "link": link.state.name}
        return out


# ================================================================ builder
@dataclass
class HardwareSystem:
    hub: HardwareHub
    virtual: VirtualDetectors
    real: Optional[HardwareDetectorSource]
    detectors: HybridDetectorSource        # give this to SensorBank
    mirror: Optional[SignalMirror]

    def close(self) -> None:
        if self.mirror is not None:
            self.mirror.blackout()
        self.hub.close()


def build_hardware(config: HardwareConfig, vehicle_model,
                   transport_factory: Optional[Callable[[DeviceConfig], Transport]] = None
                   ) -> HardwareSystem:
    """Wire the configured real detectors and physical heads around a VehicleModel.

    Must be called before the VehicleModel's first advance(). Every detector not
    listed under `sensors` stays virtual.
    """
    hub = HardwareHub(config, transport_factory)
    virtual = VirtualDetectors(vehicle_model)
    real = HardwareDetectorSource(hub, config.sensors, vehicle_model.now_ms) if config.sensors else None
    overrides = [(sid, real) for sid in config.sensors]
    detectors = HybridDetectorSource.with_overrides(virtual, overrides)
    mirror = SignalMirror(hub, config.signals) if config.signals else None
    return HardwareSystem(hub, virtual, real, detectors, mirror)
