# hardware/ — ESP32 integration (Stage 9)

The ESP32 **reports real IR detectors** and **displays commanded signal states**. It makes no
traffic decisions: Python's `SignalBank` is authoritative, and physical heads only mirror it.

| File | Purpose |
|---|---|
| `esp32_smart_traffic/esp32_smart_traffic.ino` | Firmware (Arduino IDE, ESP32 board, *Adafruit NeoPixel* library) |
| `../config/hardware.json` | Which logical detectors / movements are real, on which device and channel |
| `../smarttraffic/esp_protocol.py` | The serial protocol (Python side) |
| `../smarttraffic/hardware.py` | Links, real detector source, signal mirror, config |
| `../smarttraffic/hybrid_sensors.py` | Combines real and virtual detectors into the 16 logical ones |
| `../smarttraffic/hw_smoke.py` | Live integration check (`python -m smarttraffic.hw_smoke`) |

## Current wiring and mapping

| Physical | ESP32 | Firmware table | `hardware.json` | Logical |
|---|---|---|---|---|
| Analog IR sensor | GPIO 34, occupied when `analogRead > 2000` (≈1265 clear, ≈3600 car) | `SENSORS[0]` = channel 0 | `"NS:D1": {"device": "esp32_a", "channel": 0}` | NS D1 |
| NeoPixel head, 3 LEDs | data GPIO 13; GREEN=LED0, YELLOW=LED1, RED=LED2 | `SIGNALS[0]` = channel 0 | `"NS": {"device": "esp32_a", "channel": 0}` | mirrors movement NS |

The other 15 detectors are virtual (computed from `VehicleModel`). All 8 signals stay in the
simulation; the NeoPixel head is a mirror, not a ninth signal. The device id `esp32_a` is set
in the firmware (`DEVICE_ID`) and must match the config.

## Flashing the firmware

1. Arduino IDE → Boards Manager: install **esp32 by Espressif**. Library Manager: install
   **Adafruit NeoPixel**.
2. Open `esp32_smart_traffic/esp32_smart_traffic.ino`, select your ESP32 board and its COM
   port, then Upload.
3. **Close the Serial Monitor** afterwards: only one program can own the COM port.

## Serial protocol (115200 baud, protocol version 1)

Each message is one ASCII line: `$FIELD,FIELD,...*CS\n`. `CS` is the XOR of all bytes
between `$` and `*`, as 2 hex digits. Lines with a bad checksum or format are rejected
and counted.

| Direction | Message | Meaning |
|---|---|---|
| ESP32→PC | `HELLO,seq,ms,proto,device_id,boot_id,n_sensors,n_signals` | Sent at boot and on `QRY`; `boot_id` is random per boot |
| ESP32→PC | `SEN,seq,ms,channel,OCC\|CLR` | Detector edge (stamped at the start of the 20 ms debounce), or a full-state report after `HELLO` |
| ESP32→PC | `HB,seq,ms` | Heartbeat every 200 ms |
| ESP32→PC | `ACK,seq,ms,channel,STATE` | Signal command applied |
| ESP32→PC | `ERR,seq,ms,text` | `bad_frame`, `bad_channel`, `bad_state`, `unknown_cmd`, `line_too_long` |
| PC→ESP32 | `SIG,channel,STATE` | Show `STATE` on head `channel`: `RED`, `RED_YELLOW`, `GREEN`, `FLASHING_GREEN`, `YELLOW`, `ALL_RED`, or `OFF` (dark, shutdown only) |
| PC→ESP32 | `QRY` | Re-send `HELLO` and every sensor state |

- `seq` restarts at 0 on each boot. `ms` is the ESP32's `millis()`.
- Python drops duplicate or out-of-order `seq` values, counts gaps, and re-syncs with `QRY`.

### Signal display (ESP32 side)

- **FLASHING_GREEN:** Python sends the *logical state once*, and the ESP32 does the blinking.
  The period is 1 s, with green ON for the first 0.5 s, starting when the command arrives.
  This is the same rule as `signals.lamps_for`, apart from serial latency (a few ms).
  Python re-sends the current state every 1 s as a keep-alive; re-sending the same state does
  **not** restart the flash phase.
- **ALL_RED:** red lamp, identical to RED (the difference is only a safety state inside Python).
- **RED_YELLOW:** red + yellow.
- **Fault display:** if no `SIG` command arrives for 3 s, or none has arrived since boot, every
  head blinks **yellow**. This is only a "not controlled" indication, never a traffic decision.

## Timestamp strategy

- The ESP32 stamps each detector edge with its own `millis()`. Python never uses the moment it
  happened to read the serial port as the event time.
- Python maps device time to simulation time with
  `offset = min over the last 64 messages of (simulation receive time − device time)`.
  Transport delay is never negative, so the minimum converges to the true offset plus the
  smallest latency. Heartbeats every 200 ms keep the estimate fresh.
- `millis()` wraparound (after 49.7 days) is unwrapped. A new `boot_id` (ESP32 reset) resets
  the time base.
- An edge that maps into a simulation step already handed to `SensorBank` is placed at the next
  millisecond and counted as `late_edges`. With 100 ms steps and USB latency this is rare.
- The Python library never reads a clock. Only the live runner `hw_smoke` paces simulation
  time to the wall clock.
- Automated tests use fake serial transports with scripted timestamps, so they are fully
  deterministic and need no hardware.

## Disconnect and fault behaviour

A link is healthy only in state `CONNECTED`. Its other states are `DISCONNECTED`,
`WAITING_HELLO`, `STALE` (no message for 1.5 s), `WRONG_DEVICE` (the port has another
`device_id`) and `PROTOCOL_MISMATCH`.

- **Faulted real detectors.** While a link is unhealthy, or a detector's state hasn't been
  reported since the last `HELLO`, its real detectors are **faulted**. They are reported by
  `SensorBank.detector_fault(m, d)` / `faulted_detectors()`.
- **No new edges.** A faulted detector delivers no new edges; the last confirmed state is held.
  Nothing is invented, and it is **never** switched back to virtual. The Stage 11 controller
  will decide the fail-safe (for example, a permanent call).
- **Reconnect.** Python retries opening the port every 2 s and sends `QRY`. The `HELLO` plus
  full-state reply re-syncs everything; any change of state becomes a normal edge.
- **Signal commands.** They are only sent to an identified (`CONNECTED`) device. After a
  reconnect, the current state is re-sent immediately.
- **Clean shutdown.** Ctrl+C in `hw_smoke` sets every mapped head to `OFF` and closes the port.

## Adding hardware later (configuration only)

**Another real sensor on the same ESP32**

1. Add a row to `SENSORS[]` in the firmware, e.g. `{35, 2000, true, 20}`. Its index becomes
   the new channel (here 1). Re-flash.
2. Add `"NS:D2": {"device": "esp32_a", "channel": 1}` under `sensors` in `config/hardware.json`.
   That logical detector is now real and the virtual one is ignored.

**Another physical signal head**

1. Add a row to `SIGNALS[]`, e.g. `{14, 3, 0, 1, 2, 80}` (pin, LEDs, green, yellow and red
   index, brightness). Its index is the channel. Re-flash.
2. Add `"SS": {"device": "esp32_a", "channel": 1}` under `signals`.

**Another ESP32** (when GPIOs run out)

1. Copy the sketch, change `DEVICE_ID` (e.g. `"esp32_b"`) and its tables, and flash it to the
   second board.
2. Add `"esp32_b": {"port": "COM6"}` under `devices`, and map its channels under
   `sensors` / `signals`.
3. Each board is identified by `HELLO`, so swapped ports are detected (`WRONG_DEVICE`).

No Python code changes for any of these. `SensorBank`, the signals and the controllers are
unaffected.

## Hardware smoke test

```
python -m smarttraffic.hw_smoke --list-ports
python -m smarttraffic.hw_smoke --mode link   --port COM5 --seconds 30
python -m smarttraffic.hw_smoke --mode light  --port COM5
python -m smarttraffic.hw_smoke --mode hybrid --port COM5 --seconds 120
```

The `hybrid` mode repeats a 30 s **test pattern** on NS (not a controller): RED 8 s, then the
green sequence, GREEN until 9 s, and the full termination. Step-by-step instructions are in
the Stage 9 report.
