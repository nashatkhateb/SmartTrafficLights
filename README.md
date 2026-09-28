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
- Stage 9 (physically verified): ESP32 hardware — real IR detector(s) and physical
  signal head(s) via `esp_protocol.py`, `hardware.py`, `hybrid_sensors.py`,
  `config/hardware.json`, firmware in `hardware/`. See `hardware/README.md`.
- Stage 10 (verified): `controller_fixed.py` + `config/fixed_plans.json` — fixed-time
  baselines FT-A, FT-B (per scenario) and S6 time-of-day. See "Fixed-time controller" below.
- Stage 11 (verified): `controller_smart.py` — the study §7 actuated controller.
  See "Smart controller" below.
- Stage 12 (verified): `stats.py` + `runner.py` — study §11 metrics and the shared
  run used by every controller. See "Statistics" below.
- Stage 13 (verified): `viz.py`, `viz_render.py`, `viz_session.py`, `viz_geometry.py` — real-time
  visualization. See "Visualization" below.
- Stage 14 (official results accepted 2026-09-28): `experiments.py`, `tuning.py`, `campaign.py`, `run_experiments.py` —
  resumable parallel experiment runs, the pre-registered S1 tuning
  (`python -m smarttraffic.run_experiments tune`), the approved Smart-T1 lock (τ 15 s, W_MAX 90 s)
  and the locked official campaign (`python -m smarttraffic.run_experiments official --lock-sha …`).
  A reproduction never rewrites the lock: `tune --out <dir> --verify-lock <sha>` only verifies it.
  W_MAX = 90 s is a starvation-hold threshold, not a hard 90 s guarantee: in the official results
  detector-call waits exceeded 90 s (up to about 156 s), yet removing W_MAX made S10 much worse.
  **The Stage 14 tuning and official results are historical: they were produced with the
  previous study vehicle policy (`legacy-study`, see "Vehicle policy" below).**
- Stage 14b: `custom.py`, `viz_custom.py` — Custom / Demo mode. See "Custom / Demo mode" below.
- Vehicle policy change (2026-09-28): Stop on Yellow + immediate first vehicle on green is the
  standard behaviour everywhere. See "Vehicle policy" below.
- Not yet implemented: graphs (Stage 15), demand sweep, remaining ablations and sensor-fault runs.

Extra binding requirements added after the study (for example the hybrid real/virtual
sensor source) are recorded in `docs/ARCHITECTURE.md`.

## Signal sequence (every movement)

```
RED --start_green()--> RED_YELLOW (1.0 s) --> GREEN
    --end_green()-->  FLASHING_GREEN (3.0 s) --> YELLOW (3.5 s / 3.0 s) --> ALL_RED (2.0 s / 3.0 s) --> RED
```

- Controllers issue only `start_green(m)` and `end_green(m)`; every other step runs on timers.
  `start_green` starts the sequence that *results* in green; it is not an instant green.
- Cars may cross in GREEN and FLASHING_GREEN only: once YELLOW begins no new car enters
  (Stop on Yellow; a car due exactly at yellow onset stops). They may not cross in YELLOW,
  RED_YELLOW, ALL_RED or RED. (The historical `legacy-study` policy also allowed the first
  2.0 s of YELLOW; see "Vehicle policy".)
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
- Step order: `vehicles.advance` → `sensors.advance` → `signals.advance` → controller `update(bank)`.

## Fixed-time controller (Stage 10)

- Phase order φA NL+SL → φB NS+SS → φC EL+WL → φD ES+WS, never skipped, never sensor-driven.
  It is the comparison baseline, so it deliberately takes no SensorBank at all.
- Only `start_green` at the phase start and `end_green` at G − 3.0 s after GREEN onset;
  RED_YELLOW, flashing green, yellow and all-red are SignalBank's. The next phase starts when
  the previous phase's movements are RED.
- FT-A: the study's table uses an 89 s cycle. Signals V2 added 1.0 s red+yellow before each of
  the 4 phases, so the implemented cycle is 93 s (15 + 31.5 + 15 + 31.5 s); the published greens
  8 / 25 / 8 / 25 s are unchanged. Validation and comparisons use the implemented cycle.
  FT-B: the §8 per-scenario plans. FT-TOD: S6 plan switch at 1800 s.
- Simulator check (study §16), FT-A on S1, 20 seeds: straight 36.0 s, left 65.8 s vs Webster at
  93 s 38.6 s / 66.4 s (study's 89 s figures: 35 / 58 s).
- Use: `load_fixed_plans().controller("FT-B", "S7a", start_time=-300)`, then `update(bank)`
  every step before `bank.advance`; never step past `next_command_ms()`.

## Smart controller (Stage 11)

- Study §7, as written: it scores every compatible group by P = Q̂ + wait/10 (plus 2 per
  movement already serving and not gapped out), adds compatible demand without stopping
  anything, and stops only the conflicting greens, once all of them are releasable (minimum
  green 7 / 5 s, or 12 s after LEVEL_2; then gap-out, max-out 45 / 30 s from a conflicting
  call, or a starvation hold after 90 s). It rests in green when nothing conflicting waits.
- It reads only `SensorBank` and never knows whether a detector is real or virtual. A
  faulted detector puts its movement on recall (fail-safe).
- `SmartController(sensor_bank)`; call `update(bank)` each step after the SignalBank advanced.
- Measured behaviour, limitations (Q̂ saturates at 6; waits can exceed the ~120 s single-hold
  bound) and an exploratory FT-A comparison are in `docs/ARCHITECTURE.md`.

## Statistics (Stage 12)

- `run(RunConfig("S1", 1, "Smart"))` runs one scenario/seed/controller (warm-up 300 s,
  measurement 3600 s, flush ≤ 900 s, dt 0.1 s) and returns a `RunResult`;
  `run_pair(config, ("FT-A", "Smart"))` runs controllers on identical arrivals.
- Metrics follow study §11: control delay (mean, P95, max), waiting time, queue (1 s samples,
  including cars held outside the road entry), throughput in the measurement hour, % served,
  green utilization, signal changes, delays over 90 s, Jain's index, safety violations.
  Definitions and diagnostics: `docs/ARCHITECTURE.md`.
- `RunResult.to_json()`, `summary_row()`, `movement_rows()`, `vehicle_rows()`, `queue_rows()`
  for later experiments and graphs; `mean_ci95` / `paired_comparison` across seeds.

## Visualization (Stage 13)

```
python -m smarttraffic.viz                                  # S1, seed 1, Smart; press Space to run
python -m smarttraffic.viz --controller FT-A --scenario S5 --speed 5
python -m smarttraffic.viz --hardware --port COM5            # real IR detector + mirrored head
python -m smarttraffic.viz --screenshot shot.png --at 120    # still frame, no window
```

- The intersection as modelled: 4 approaches, each with a STRAIGHT lane and a LEFT lane and
  its own signal head (8 heads with arrow lenses on one pole per approach, no right-turn head),
  right-hand traffic, 3.6 m lanes, painted median, cycle lanes, stop lines, zebra crossings,
  lane-use arrows, a coloured strip per lane showing its signal, and the active paths through
  the junction in the signal colour. Cars have varied paint; stopped cars show brake lights,
  left-turning cars blink their indicator.
- Every lamp is SignalBank state (FLASHING_GREEN blinks with `lamps_for`); vehicles are the
  VehicleModel's (positions interpolated between steps for smooth motion, display only);
  the 16 detector loops show clear / occupied / call / LEVEL_2; the real detector
  (config/hardware.json, NS:D1) is drawn in its own slot as "IR", never as a 17th.
- Side panel: run controls (Run/Pause, Reset, speed 0.5–10× and Max, controller FT-A / FT-B /
  Smart and FT-TOD for S6, scenario, seed, hardware on/off); live metrics; Smart: target,
  starvation hold and a plain-language reason (minimum green, gap-out, max-out), per-movement
  calls, Q̂, wait and priority P, recent decisions; Fixed: phase ring, current phase and time left.
- Keys: Space run/pause · R reset · 1–6 speed · C controller · ←/→ scenario (incl. Custom…) ·
  ↑/↓ seed · H hardware · F11 full screen · Esc quit. In the Custom panel: Tab next field,
  digits type, Enter apply, Esc cancel.
- The visualization is an observer of the Stage 12 runner: a run shown on screen gives the
  same `RunResult` as the same run without it (tested, also with pauses and speed changes).
  With hardware on, the speed is fixed at 1× (the ESP32 link follows the wall clock).

## Custom / Demo mode (Stage 14b)

```
python -m smarttraffic.custom --demand NS=400,SS=400,ES=400,WS=400,NL=120,SL=120,EL=120,WL=120 \
    --controller Smart --seed 1 --measure 600 [--params T1|custom]
```

- In the visualization: "Custom…" after S10 in the scenario selector opens the Custom / Demo
  panel (8 demands, seed, FT-A or Smart, measurement 5–60 min, presets, load warning; Advanced
  settings collapsed: Smart parameter set Smart-T1 or custom τ, W_MAX and max greens). Custom
  runs use the standard vehicle policy like every other run; there is no per-run option. In the UI Smart is always Smart-T1 for the official scenarios
  (locked τ 15 s, W_MAX 90 s); Smart-study is used only internally for comparisons. A purple CUSTOM / DEMO badge is shown while it is active;
  official scenarios show "OFFICIAL · LOCKED".
- Demands: numbers, finite, ≥ 0, rejected at ≥ 2400 veh/h (arrival-model limit); the
  interface caps input at 1800 veh/h; 0 is allowed. A load check warns near capacity
  (X ≥ 0.85) and when oversaturated (X ≥ 1).
- Custom results are tagged `CUSTOM / DEMO` (metadata `mode: custom`), are saved only under
  `results/custom/` (not committed) and never enter official results: official mode refuses
  custom demand, and campaigns refuse the custom folder. Custom arrivals are generated in memory (S1 demands + seed reproduce S1's
  arrivals exactly).

## Vehicle policy (standard since 2026-09-28)

Every run — official scenarios S1–S10, FT-A, FT-B, FT-TOD, Smart / Smart-T1, Custom / Demo, the
visualization (runs, screenshots, reset, scenario / controller / seed switching) and the
experiment command line — uses the **standard** policy (`vehicles.STANDARD`):

- **Stop on Yellow**: once YELLOW begins no new vehicle enters the intersection. The rule is
  strict and deterministic in integer milliseconds: a car must cross before yellow onset
  (last permitted crossing t_y − 1 ms); a car due exactly at t_y stops at the line. Cars whose
  crossing time would fall at or after t_y are stopped as soon as the yellow onset is known
  (at flashing-green onset, 3 s before yellow).
- **Immediate first vehicle on green**: the first queued car starts moving at green onset
  t_g (from slot 1, 0.5 m before the line, it crosses 36 ms later). Following cars keep the
  saturation headway (1.9 s straight / 2.0 s left) and start one after another, never all at
  once. A 10-car straight queue clears 17.136 s after green (legacy: 21.0 s). This is exact
  for any time step: a step containing a GREEN onset is split there.

The previous study rules are kept only as the explicitly named compatibility policy
**`legacy-study`** (first car at t_g + l1 + h = t_g + 3.9 s; permissive first 2.0 s of yellow,
inclusive). It is never a default and is not offered in the visualization. Use it only to
reproduce the historical Stage 14 results:

    python -m smarttraffic.run_experiments tune --policy legacy-study --out <dir> --verify-lock <sha>
    python -m smarttraffic.run_experiments official --policy legacy-study --lock-sha <sha> --out <dir>

Without `--policy`, `tune` / `official` use the standard policy and write to
`results/tuning/S1_T1_standard` and `results/official/campaign_T1_standard`; a campaign folder
holds one policy only and the Stage 14 folders refuse standard runs, so historical results are
never mixed or overwritten. Standard runs record `vehicle_policy`, `stop_on_yellow` and
`immediate_first_vehicle` in their metadata; legacy-study run keys and metadata are exactly the
Stage 14 ones (the golden fingerprints in `tests/golden/` still reproduce).

**Historical results.** `results/tuning/S1_T1` (and the PC reproduction `S1_T1_pc`),
`results/official/campaign_T1` and the Smart-T1 selection in
`config/smart_params_locked.json` were produced with `legacy-study`. The locked Smart-T1
parameters (τ 15 s, W_MAX 90 s) are kept unchanged; they were not retuned for the standard
policy. See `docs/ARCHITECTURE.md` ("Vehicle policy") for the exact list of assumptions.

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
| `smarttraffic/controller_smart.py` | Actuated controller: group priority, partial transitions, W_MAX hold, fault recall | §6, §7, §9 |
| `smarttraffic/controller_fixed.py` | FT-A / FT-B / time-of-day fixed-time controller | §8 |
| `smarttraffic/stats.py` | Per-vehicle records, 1 s queue samples, §11 metrics, run results, 95% CIs | §10–11 |
| `smarttraffic/runner.py` | One run, identical for every controller; paired runs | §10, §13 |
| `smarttraffic/viz.py` | Pygame application: window, controls, keyboard, still frames | §13 |
| `smarttraffic/viz_render.py` | Drawing: urban intersection scene (asphalt, paint, kerbs, blocks, signal poles, cars) and the UI panels | §13 |
| `smarttraffic/viz_session.py` | Runs the shared runner on a thread with a read-only observer: snapshots, pacing, pause, reset | §13 |
| `smarttraffic/viz_geometry.py` | Pure drawing geometry: lanes, paths, 8 heads, 16 detector positions | §13 |
| `smarttraffic/experiments.py`, `tuning.py`, `campaign.py`, `run_experiments.py` | Resumable parallel runs, pre-registered tuning, Smart-T1 lock, official campaign and statistics, CLI | §10–11 |
| `config/tuning_plan.json`, `config/smart_params_locked.json` | Frozen tuning plan; locked Smart-T1 parameters | §10 |
| `smarttraffic/custom.py`, `viz_custom.py` | Custom / Demo mode: validated demand, load check, parameter sets, tagged output; the panel's editable state | — |
| `tests/golden/official_T1_baseline.csv` | Simulation fingerprints of the 490 approved official runs (equivalence baseline) | §10 |
| `smarttraffic/make_graphs.py` | Graphs G1–G12 | §12 |
| `config/scenarios.json` | Scenario definitions S1–S10 | §10 |
| `config/fixed_plans.json` | Fixed-time phases and plans (FT-A, FT-B per scenario, S6 TOD) | §8 |
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
