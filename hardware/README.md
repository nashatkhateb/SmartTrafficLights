# hardware/ — ESP32 integration (Stage 9; verified demo hardware, protocol v2)

The ESP32 **reports two real IR detectors** and **sets a physical traffic light exactly as
commanded**. It makes no traffic decisions and runs no traffic-light timing: the simulator's
`SignalBank` / controller is the only timing authority, and the physical head only mirrors the
logical NS signal.

```
real D1 (GPIO27) + real D2 (GPIO26) -> ESP32 -> serial (SEN) -> HybridDetectorSource -> SensorBank -> controller
SignalBank NS state + lamps -> SignalMirror -> serial (SIG) -> ESP32 -> R/Y/G outputs (GPIO32/33/25)
```

| File | Purpose |
|---|---|
| `esp32_smart_traffic/esp32_smart_traffic.ino` | Project firmware (Arduino IDE, ESP32 board; no extra library) |
| `esp32_led_test/esp32_led_test.ino` | Stand-alone wiring test of the LED module (own fixed timing; not for the project run) |
| `../config/hardware.json` | Which logical detectors / movements are real, on which device and channel |
| `../smarttraffic/esp_protocol.py` | The serial protocol (Python side) |
| `../smarttraffic/hardware.py` | Links, real detector source, signal mirror, config |
| `../smarttraffic/hybrid_sensors.py` | Combines real and virtual detectors into the 16 logical ones |
| `../smarttraffic/esp_sim.py` | Executable model of the firmware for tests (same pins, polarity, timing) |
| `../smarttraffic/hw_smoke.py` | Live integration check (`python -m smarttraffic.hw_smoke`) |

## Current wiring and mapping (physically verified)

| Physical | ESP32 | Firmware table | `hardware.json` | Logical |
|---|---|---|---|---|
| Digital IR obstacle sensor 1 (VCC 3.3 V, GND) | OUT → **GPIO27**, active-low: HIGH = clear, LOW = car | `SENSORS[0]` = sensor channel 0 | `"NS:D1": {"device": "esp32_a", "channel": 0}` | **NS D1** (2 m) |
| Digital IR obstacle sensor 2 (VCC 3.3 V, GND) | OUT → **GPIO26**, active-low | `SENSORS[1]` = sensor channel 1 | `"NS:D2": {"device": "esp32_a", "channel": 1}` | **NS D2** (32 m) |
| 3-LED traffic-light module (4 pins) | **G → GPIO25, Y → GPIO33, R → GPIO32**, GND → GND; HIGH = lamp ON | `SIGNALS[0]` = signal channel 0 | `"NS": {"device": "esp32_a", "channel": 0}` | mirrors movement NS |

So the hybrid system has **2 real detectors (NS:D1, NS:D2) + 14 virtual detectors = 16 logical
detectors**. All 8 signals stay in the simulation; the physical head is a mirror, not a ninth
signal. The device id `esp32_a` is set in the firmware (`DEVICE_ID`) and must match the config.

The sensor inputs use `INPUT_PULLUP`, so an unplugged sensor reads HIGH = **clear** (no phantom
vehicle). Each change must last 20 ms (debounce) before it is reported.

The earlier Stage 9 hardware (analog IR on GPIO34 with threshold 2000, NeoPixel head on GPIO13)
is obsolete; the firmware no longer contains that path.

## Real-demand-only NS in hardware mode

Because both NS detectors are real, NS is a **real-demand-only** movement
(`hardware.real_only_movements`) while hardware mode is on:

- **No virtual NS vehicles** are generated or drawn; there are no virtual NS:D1 / NS:D2 events.
  NS demand comes **only** from the two physical IR sensors (no duplicate real + virtual demand).
- The other 7 movements and their 14 detectors stay fully virtual, with the same arrivals as
  without hardware.
- The Smart controller, the 16 logical detectors and the signals are unchanged; the run
  metadata records `"virtual_arrivals_excluded": ["NS"]`.
- The rule is fixed at session start and holds for the whole hardware session, connected or
  not (see disconnect behaviour below). Switching hardware off (H) starts a new virtual-only
  session with the full scenario, including NS traffic, exactly as before.
- The visualization's hardware card shows "NS: real demand only · no virtual cars".

A movement with only one real detector would keep its virtual vehicles (its real detector's
virtual edges are still dropped by `HybridDetectorSource`).

## Physical head behaviour (exact copy of the logical NS signal)

| Logical NS state | Lamps (R Y G) |
|---|---|
| RED | red |
| RED_YELLOW | red + yellow |
| GREEN | green |
| FLASHING_GREEN | green ON / OFF with the logical flash: ON for the first 0.5 s of every 1 s from the onset of flashing green |
| YELLOW | yellow |
| ALL_RED (clearance) | red |
| OFF (shutdown only) | dark |

Python computes the lamps with `signals.lamps_for` (the same function the visualization uses)
and sends them in every `SIG` command, on **every lamp change** — including each flash edge —
and as a 1 s refresh. The ESP32 only switches its three outputs.

**Fail-safe:** if no `SIG` arrives for 3 s (USB unplugged, program stopped or paused), or none
has arrived since boot, the head shows the fault indication: **yellow blinking** at 1 Hz, red
and green off. This is the only timing on the ESP32 and it is never a traffic decision. Normal
display resumes with the next command.

## Flashing the firmware

1. Arduino IDE → Boards Manager: install **esp32 by Espressif**. No other library is needed.
2. Open `esp32_smart_traffic/esp32_smart_traffic.ino`, select your ESP32 board and its COM
   port, then Upload. (Do not leave the stand-alone `esp32_led_test` sketch on the board.)
3. **Close the Serial Monitor** afterwards: only one program can own the COM port.

## Serial protocol (115200 baud, protocol version 2)

Each message is one ASCII line: `$FIELD,FIELD,...*CS\n`. `CS` is the XOR of all bytes
between `$` and `*`, as 2 hex digits. Lines with a bad checksum or format are rejected
and counted.

| Direction | Message | Meaning |
|---|---|---|
| ESP32→PC | `HELLO,seq,ms,proto,device_id,boot_id,n_sensors,n_signals` | Sent at boot and on `QRY`; `proto` = 2, `n_sensors` = 2, `n_signals` = 1 |
| ESP32→PC | `SEN,seq,ms,channel,OCC\|CLR` | Detector edge (stamped at the start of the 20 ms debounce), or a full-state report after `HELLO`. Active-low decoding happens on the ESP32: LOW → `OCC`, HIGH → `CLR` |
| ESP32→PC | `HB,seq,ms` | Heartbeat every 200 ms |
| ESP32→PC | `ACK,seq,ms,channel,STATE,LAMPS` | Signal command applied |
| ESP32→PC | `ERR,seq,ms,text` | `bad_frame`, `bad_sig`, `bad_channel`, `bad_state`, `bad_lamps`, `unknown_cmd`, `line_too_long` |
| PC→ESP32 | `SIG,channel,STATE,LAMPS` | Set head `channel` to `LAMPS` (`RYG` as `0`/`1`, e.g. `100` red, `110` red+yellow, `001` green, `000` dark); `STATE` is the logical state and must be able to show those lamps |
| PC→ESP32 | `QRY` | Re-send `HELLO` and every sensor state |

- `seq` restarts at 0 on each boot. `ms` is the ESP32's `millis()`.
- Python drops duplicate or out-of-order `seq` values, counts gaps, and re-syncs with `QRY`.
- Version 1 (Stage 9: `SIG,channel,STATE`, blinking on the ESP32, NeoPixel) is obsolete. A board
  still running it answers `HELLO` with `proto` 1; Python then reports `PROTOCOL_MISMATCH`,
  treats both real detectors as faulted and sends nothing to the head. Re-flash the new firmware.

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
- The Python library never reads a clock. Only the live runner `hw_smoke` (and the visualization
  in hardware mode) paces simulation time to the wall clock.
- Automated tests use fake serial transports and the firmware model `esp_sim.SimulatedEsp32`, so
  they are fully deterministic and need no hardware.

## Disconnect and fault behaviour

A link is healthy only in state `CONNECTED`. Its other states are `DISCONNECTED`,
`WAITING_HELLO`, `STALE` (no message for 1.5 s), `WRONG_DEVICE` (the port has another
`device_id`) and `PROTOCOL_MISMATCH`.

- **NS stays real-demand-only.** A disconnect never brings virtual NS vehicles back, and a
  missing port at start does not either: NS then has no demand except the fail-safe recall.
- **Faulted real detectors.** While a link is unhealthy, or a detector's state hasn't been
  reported since the last `HELLO`, its real detectors (here NS:D1 and NS:D2) are **faulted**.
  They are reported by `SensorBank.detector_fault(m, d)` / `faulted_detectors()`, and the Smart
  controller puts NS on its fail-safe recall.
- **No new edges.** A faulted detector delivers no new edges; the last confirmed state is held.
  Nothing is invented, and it is **never** switched back to virtual.
- **Reconnect.** Python retries opening the port every 2 s and sends `QRY`. The `HELLO` plus
  full-state reply re-syncs both detectors; any change of state becomes a normal edge.
- **Signal commands.** They are only sent to an identified (`CONNECTED`) device. After a
  reconnect, the current state and lamps are re-sent immediately. Meanwhile the board shows the
  yellow-blinking fault indication.
- **No hardware.** A missing COM port, a wrong port or a bad `hardware.json` never stops the
  simulator: the visualization falls back to all-simulated or shows the link fault.
- **Clean shutdown.** Ctrl+C in `hw_smoke` (and closing the visualization) sets every mapped head
  to `OFF` and closes the port.

## Adding hardware later (configuration only)

**Another real sensor on the same ESP32**

1. Add a row to `SENSORS[]` in the firmware, e.g. `{14, true, 20}` (pin, active-low,
   debounce ms). Its index becomes the new channel (here 2). Re-flash.
2. Add e.g. `"SS:D1": {"device": "esp32_a", "channel": 2}` under `sensors` in
   `config/hardware.json`. That logical detector is now real and the virtual one is ignored.

**Another physical signal head**

1. Add a row to `SIGNALS[]`, e.g. `{21, 22, 23, true}` (green, yellow, red pin, HIGH = ON). Its
   index is the channel. Re-flash.
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

- `link`: hold a hand in front of sensor 1, then sensor 2; each prints `REAL NS:D1 OCCUPIED` /
  `REAL NS:D2 OCCUPIED` and `CLEAR` when removed.
- `light`: the head runs the full sequence (red, red+yellow, green, flashing green, yellow,
  red) twice, driven by Python.
- `hybrid`: `Real detectors : NS:D1, NS:D2`, `Virtual : 14 detectors`, `Real-only : NS`; a
  repeating 30 s **test pattern** on NS (not a controller) and the head mirroring it. With
  `--traffic`, NS arrivals are left out (real demand only).
