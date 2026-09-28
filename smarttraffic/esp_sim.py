"""esp_sim -- executable model of the ESP32 firmware (protocol v2), for tests without hardware.

SimulatedEsp32 behaves like hardware/esp32_smart_traffic/esp32_smart_traffic.ino on the
verified demo wiring, and is also a Transport (open / close / read_available / write), so the
real host stack (EspLink, HardwareDetectorSource, SignalMirror, build_hardware) can be driven
end to end:

    sensor channel 0 = NS:D1 : digital input GPIO27, active-low (LOW = car, HIGH = clear)
    sensor channel 1 = NS:D2 : digital input GPIO26, active-low
    signal channel 0 = NS    : outputs G = GPIO25, Y = GPIO33, R = GPIO32, HIGH = lamp ON

Device time is explicit (no clock): advance_to(device_ms) samples the inputs every
SAMPLE_MS, debounces them (DEBOUNCE_MS; the edge is stamped with the first sample of the new
state), and emits heartbeats. Lamps are set exactly as commanded by SIG; the only local
timing is the fail-safe: no SIG for SIGNAL_TIMEOUT_MS (or none since boot) -> yellow blinks
at 1 Hz. unplug() / plug() emulate removing and re-inserting the USB cable (the board keeps
running; the host must reopen the port). tests/test_esp_protocol.py checks that the
firmware source uses the same pins, polarities and constants.
"""
from typing import Dict, List, Optional, Tuple

from smarttraffic import esp_protocol as proto

__all__ = ["SimulatedEsp32", "SENSOR_PINS", "SIGNAL_PINS", "ACTIVE_LOW", "LAMP_ACTIVE_HIGH",
           "decode_sensor_level", "SAMPLE_MS", "DEBOUNCE_MS", "HEARTBEAT_MS", "SIGNAL_TIMEOUT_MS",
           "FAULT_BLINK_PERIOD_MS", "FAULT_BLINK_ON_MS"]

SENSOR_PINS: Tuple[int, ...] = (27, 26)                 # channel 0 = NS:D1, channel 1 = NS:D2
ACTIVE_LOW = True                                       # LOW = car detected, HIGH = clear
SIGNAL_PINS: Tuple[Tuple[int, int, int], ...] = ((25, 33, 32),)   # (green, yellow, red) per channel
LAMP_ACTIVE_HIGH = True                                 # HIGH = lamp ON
SAMPLE_MS = 2
DEBOUNCE_MS = 20
HEARTBEAT_MS = 200
SIGNAL_TIMEOUT_MS = 3000
FAULT_BLINK_PERIOD_MS = 1000
FAULT_BLINK_ON_MS = 500
HIGH, LOW = 1, 0


def decode_sensor_level(level: int, active_low: bool = ACTIVE_LOW) -> bool:
    """Electrical input level (0/1) -> occupied? For the active-low IR modules LOW = car."""
    if level not in (0, 1):
        raise ValueError(f"digital level must be 0 or 1, got {level!r}")
    return (level == LOW) if active_low else (level == HIGH)


class SimulatedEsp32:
    def __init__(self, device_id: str = "esp32_a", boot_id: str = "0000BEEF", start_ms: int = 0,
                 proto_version: int = proto.PROTOCOL_VERSION):
        self.device_id, self.boot_id, self.proto_version = device_id, boot_id, proto_version
        self.ms = start_ms
        self.seq = 0
        self.levels: Dict[int, int] = {p: HIGH for p in SENSOR_PINS}      # pull-up: clear
        self.outputs: Dict[int, int] = {p: LOW for trio in SIGNAL_PINS for p in trio}
        self._lamps: List[Tuple[bool, bool, bool]] = [(False, False, False) for _ in SIGNAL_PINS]
        self._state = [decode_sensor_level(self.levels[p]) for p in SENSOR_PINS]
        self._cand = list(self._state)
        self._cand_since = [start_ms] * len(SENSOR_PINS)
        self._last_cmd: Optional[int] = None
        self._last_hb = start_ms
        self._last_sample = start_ms
        self.plugged = True
        self.is_open = False
        self._out = bytearray()          # bytes the device has written to USB
        self._rx = bytearray()
        self.errors: List[str] = []
        self.commands: List[Tuple[int, int, str, str]] = []   # (device ms, channel, state, lamps)
        self._hello()
        self._all_sensors()
        self._render()

    # ------------------------------------------------------------ physical side
    def set_input(self, pin: int, level: int) -> None:
        """Electrical level on a sensor input (the module's OUT pin)."""
        if pin not in self.levels:
            raise ValueError(f"GPIO{pin} is not a sensor input")
        if level not in (0, 1):
            raise ValueError("level must be 0 or 1")
        self.levels[pin] = level

    def car(self, channel: int, present: bool) -> None:
        """Put / remove an obstacle in front of sensor `channel` (drives its OUT pin)."""
        self.set_input(SENSOR_PINS[channel], LOW if present == ACTIVE_LOW else HIGH)

    def lamps(self, channel: int = 0) -> Tuple[bool, bool, bool]:
        """(red, yellow, green) lamp ON? from the output pin levels."""
        g, y, r = SIGNAL_PINS[channel]
        on = HIGH if LAMP_ACTIVE_HIGH else LOW
        return (self.outputs[r] == on, self.outputs[y] == on, self.outputs[g] == on)

    def in_fault(self) -> bool:
        return self._last_cmd is None or self.ms - self._last_cmd > SIGNAL_TIMEOUT_MS

    def advance_to(self, device_ms: int) -> None:
        if device_ms < self.ms:
            raise ValueError("device time cannot go backwards")
        while self._last_sample + SAMPLE_MS <= device_ms:
            self._last_sample += SAMPLE_MS
            self.ms = self._last_sample
            self._sample()
            while self._last_hb + HEARTBEAT_MS <= self.ms:
                self._last_hb += HEARTBEAT_MS
                self._send(["HB", self._s(), self._last_hb])
            self._render()
        self.ms = device_ms
        self._render()

    def unplug(self) -> None:
        """USB cable pulled: like pyserial, the port object still looks open until the next
        read / write fails; reopening fails until plug()."""
        self.plugged = False

    def plug(self) -> None:
        self.plugged = True

    # ------------------------------------------------------------ Transport (USB side)
    def open(self) -> None:
        if not self.plugged:
            raise OSError("could not open port (ESP32 unplugged)")
        self.is_open = True
        self._out.clear()              # nothing buffered survives a reconnect

    def close(self) -> None:
        self.is_open = False

    def read_available(self) -> bytes:
        if not self.plugged:
            raise OSError("device unplugged")
        data, self._out = bytes(self._out), bytearray()
        return data

    def write(self, data: bytes) -> None:
        if not (self.plugged and self.is_open):
            raise OSError("port not open")
        self._rx.extend(data)
        *lines, rest = bytes(self._rx).split(b"\n")
        self._rx = bytearray(rest)
        for line in lines:
            if line.strip():
                self._handle(line)

    # ------------------------------------------------------------ firmware logic
    def _s(self) -> int:
        s = self.seq
        self.seq = (self.seq + 1) % proto.U32
        return s

    def _send(self, fields) -> None:
        if self.plugged and self.is_open:
            self._out.extend(proto.encode_frame(fields))

    def _hello(self) -> None:
        self._send(["HELLO", self._s(), self.ms, self.proto_version, self.device_id, self.boot_id,
                    len(SENSOR_PINS), len(SIGNAL_PINS)])

    def _all_sensors(self) -> None:
        for ch, occ in enumerate(self._state):
            self._send(["SEN", self._s(), self.ms, ch, "OCC" if occ else "CLR"])

    def _sample(self) -> None:
        for ch, pin in enumerate(SENSOR_PINS):
            raw = decode_sensor_level(self.levels[pin])
            if raw != self._cand[ch]:
                self._cand[ch], self._cand_since[ch] = raw, self.ms
            if self._cand[ch] != self._state[ch] and self.ms - self._cand_since[ch] >= DEBOUNCE_MS:
                self._state[ch] = self._cand[ch]
                self._send(["SEN", self._s(), self._cand_since[ch], ch, "OCC" if self._state[ch] else "CLR"])

    def _set_outputs(self, ch: int, r: bool, y: bool, g: bool) -> None:
        gp, yp, rp = SIGNAL_PINS[ch]
        on, off = (HIGH, LOW) if LAMP_ACTIVE_HIGH else (LOW, HIGH)
        self.outputs[rp], self.outputs[yp], self.outputs[gp] = (on if r else off), (on if y else off), (on if g else off)

    def _render(self) -> None:
        for ch in range(len(SIGNAL_PINS)):
            if self.in_fault():
                self._set_outputs(ch, False, self.ms % FAULT_BLINK_PERIOD_MS < FAULT_BLINK_ON_MS, False)
            else:
                self._set_outputs(ch, *self._lamps[ch])

    def _error(self, text: str) -> None:
        self.errors.append(text)
        self._send(["ERR", self._s(), self.ms, text])

    def _handle(self, line: bytes) -> None:
        try:
            f = proto.decode_frame(line)
        except proto.ProtocolError:
            self._error("bad_frame")
            return
        if f == ["QRY"]:
            self._hello()
            self._all_sensors()
            return
        if f[0] != "SIG" or len(f) != 4:
            self._error("bad_sig" if f[0] == "SIG" else "unknown_cmd")
            return
        _, ch_s, name, lamps = f
        ch = int(ch_s) if ch_s.isdigit() else -1
        if not 0 <= ch < len(SIGNAL_PINS):
            self._error("bad_channel")
            return
        if name not in proto.SIGNAL_COMMAND_STATES:
            self._error("bad_state")
            return
        if lamps not in proto.LAMP_PATTERNS[name]:
            self._error("bad_lamps")
            return
        self._last_cmd = self.ms
        self._lamps[ch] = tuple(c == "1" for c in lamps)
        self.commands.append((self.ms, ch, name, lamps))
        self._render()
        self._send(["ACK", self._s(), self.ms, ch, name, lamps])
