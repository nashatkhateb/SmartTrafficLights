# SmartTrafficLights

Sensor-actuated control for an eight-movement intersection (IoT university project).

The approved design specification is `docs/Eight-Movement_Intersection_Study.html`.
All parameters, algorithms and scenarios come from that document; do not change them here.

## Status

- Done: environment; `conflicts.py` (conflict model, study §3); `signals.py` (signal state machine
  with the visible sequence below, §6–§7);
  `traffic_gen.py` + `config/scenarios.json` (arrivals and scenarios S1–S10, §5, §10);
  `vehicles.py` (vehicle/queue model, §4–§5, R3); `sensors.py` (16 virtual IR detectors and
  demand logic, §4, §6); tests for all five.
- Stage 9 (awaiting physical verification): ESP32 hardware — real IR detector(s) and physical
  signal head(s) via `esp_protocol.py`, `hardware.py`, `hybrid_sensors.py`,
  `config/hardware.json`, firmware in `hardware/`. See `hardware/README.md`.
- Not yet implemented: controllers, statistics, visualization, experiments.

Extra binding requirements added after the study (for example the hybrid real/virtual
sensor source) are recorded in `docs/ARCHITECTURE.md`.

## Signal sequence (every movement)

```
RED --start_green()--> RED_YELLOW (1.0 s) --> GREEN
    --end_green()-->  FLASHING_GREEN (3.0 s) --> YELLOW (3.5 s / 3.0 s) --> ALL_RED (2.0 s / 3.0 s) --> RED
```

- Controllers issue only `start_green(m)` and `end_green(m)`; every other step runs on timers.
  `start_green` starts the sequence that *results* in green; it is not an instant green.
- Cars may cross in GREEN and FLASHING_GREEN, and in the first 2.0 s of YELLOW (study rule).
  They may not cross in RED_YELLOW, ALL_RED or RED.
- Every state except RED blocks conflicting movements. ALL_RED looks like RED, but it is the
  clearance interval and still blocks.
- `SignalBank.lamp_state(m)` returns `Lamps(red, yellow, green)` for display or hardware. The
  flashing green lamp is ON for the first 0.5 s of each 1.0 s, counted from flash onset.
- Yellow and all-red come from the study. Red+yellow (1.0 s) and flashing green (3.0 s) are
  project assumptions. Green time G counts steady green plus flashing green. Details are in
  `docs/ARCHITECTURE.md`.

## Sensors (Stage 8: virtual only)

- There are 16 point IR detectors: D1 at 2.0 m and D2 at 32.0 m before the stop line, for
  each of the 8 movements.
- A detector is occupied while a car's body covers its point, i.e. from when the front
  reaches it until the rear passes it. A stopped car #1 covers D1 and car #5 covers D2.
- `VirtualDetectors` derives the on/off edges, exact to the millisecond, from the vehicle
  model. `SensorBank` is the only object controllers use:
  - While a movement waits, any pulse latches a call at **LEVEL_1** (Q̂ = 2).
  - D2 occupied continuously for ≥ 2.0 s latches **LEVEL_2** (Q̂ = 6).
  - At the actual GREEN onset, `extended_initial_required` tells the controller whether to
    use the 12 s initial green.
  - During GREEN / FLASHING_GREEN, `gap_out` becomes true after 2.5 s with both detectors clear.
- Stage 9: real ESP32 detectors plug in behind the same `DetectorSource` interface through
  `HybridDetectorSource`. Currently NS:D1 is real and the other 15 are virtual; this is set in
  `config/hardware.json`. A physical NeoPixel head mirrors NS.
  `SensorBank.detector_fault()` reports hardware faults.
- Step order: `vehicles.advance` → `sensors.advance` → `signals.advance`.

## Setup (Windows)

1. Python 3.10–3.13 must be installed (3.14 is not yet supported by pygame).
2. Double-click `setup_env.bat`. It creates `.venv`, installs `requirements.txt`,
   and runs the environment checks. It should end with `SETUP OK`.
3. Later, activate with `.venv\Scripts\activate`, then run `python check_env.py` or `pytest`.

## Layout

| Path | Contents | Study |
|---|---|---|
| `smarttraffic/traffic_gen.py` | Seeded arrival lists per movement → CSV | §5, §10 |
| `smarttraffic/vehicles.py` | Positions, queue slots, discharge headways, yellow rule | §5 |
| `smarttraffic/sensors.py` | 16 point detectors, `DetectorSource` interface, `VirtualDetectors`, `SensorBank` demand logic | §4, §6 |
| `smarttraffic/conflicts.py` | Conflict matrix, 8 groups, `compatible()`, safety invariant | §3 |
| `smarttraffic/signals.py` | Per-head R→RY→G→FG→Y→AR→R state machine, lamp outputs | §6–§7 |
| `smarttraffic/controller_smart.py` | Actuated controller with partial transitions | §7, §9 |
| `smarttraffic/controller_fixed.py` | FT-A / FT-B / time-of-day baselines | §8 |
| `smarttraffic/stats.py` | Per-vehicle records, 1 s samples, summaries | §11 |
| `smarttraffic/viz.py` | Pygame view, frame capture | §13 |
| `smarttraffic/run_experiments.py` | Headless scenarios × controllers × seeds | §10 |
| `smarttraffic/make_graphs.py` | Graphs G1–G12 | §12 |
| `config/scenarios.json` | Scenario definitions S1–S10 | §10 |
| `config/hardware.json` | Real detectors / physical heads → ESP32 device + channel | Stage 9 |
| `smarttraffic/esp_protocol.py`, `hardware.py`, `hybrid_sensors.py`, `hw_smoke.py` | ESP32 protocol, links, hybrid 16-detector source, signal mirror, live smoke test | Stage 9 |
| `data/arrivals/` | Arrival CSVs, one per scenario × seed, read by every controller | §5 |
| `results/runs/` | Experiment result CSVs | §10–11 |
| `figures/` | Generated graphs | §12 |
| `media/frames`, `media/screenshots` | Pygame frame capture for video, report screenshots | §13, §16 |
| `hardware/` | ESP32 firmware + wiring/protocol documentation | §13, Stage 9 |
| `tests/` | pytest unit tests | §16 |
| `docs/ARCHITECTURE.md` | Requirements added after the study (R1: hybrid sensors) | — |
| `check_env.py` | Environment verification script | — |

Video export (frames → MP4) needs `ffmpeg` installed separately; it is not a pip package.
