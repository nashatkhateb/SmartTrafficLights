# Architecture notes and binding requirements

The approved design is `Eight-Movement_Intersection_Study.html` (§13 gives the module layout).
This file records requirements added by the project owner after approval. They extend
the study; they do not change any of its design decisions.

## R1 — Hybrid sensor source (added 2026-09-27)

Status: implemented in Stage 9 (awaiting physical verification). `HybridDetectorSource` assigns
each of the 16 detectors to exactly one source; `HardwareDetectorSource` provides the real ones;
`config/hardware.json` holds the mapping. Details are in the Stage 9 section below.

`sensors.py` must support a **hybrid** set of the 16 point sensors (D1 and D2 for each of the 8 movements):

- **Real** sensors: physical IR sensors read from the ESP32 over serial (pyserial).
- **Virtual** sensors: occupancy computed by the simulation from vehicle positions (study §4).
- Any sensor can be configured as real or virtual, one sensor at a time. For example, NS D1 and NS D2 can be real (the study §13 hardware demo) while the other 14 are virtual.

**The controller receives one unified sensor interface.** It must not know, check, or branch on whether a sensor is real or virtual. Planned shape (to be confirmed when `sensors.py` is implemented):

- One read per simulation step returns the 16 boolean occupancy states keyed by `(movement, "D1" | "D2")`. Movements come from `conflicts.MOVES`.
- Real and virtual sources implement the same small source interface. A composite sensor bank picks the source per sensor from configuration.
- The optional faults in §4 (missed detections, stuck-on, stuck-off) are applied inside the sensor layer, so the controller cannot see them either.

Consequences for other modules:

- `controller_smart.py` takes only sensor states and time as input (study §2, §13). It must never import `vehicles`, `serial`, or any sensor-source class.
- `controller_fixed.py` does not read sensors at all (study §13).
- The simulator still owns the true queue data for statistics. That data never passes through the sensor interface (study §2, "the controller sees only sensors").
- Serial I/O must not block the 0.1 s simulation step. The real source keeps the last known state and must define a fail-safe behaviour for a lost serial link (to be decided when `sensors.py` is implemented).

## R2 — One arrival file per run, shared by all controllers (study §5, §10)

Experiments must obtain arrivals only through `traffic_gen.get_or_create_arrivals(scenario, seed)`.
That function returns the file `data/arrivals/<scenario>_seed<seed>_t-300_3600.csv`, creating it
if it is absent and validating it (metadata and checksum) if it exists. Fixed and Smart runs
for the same scenario and seed therefore read identical arrivals. The generator never imports
signals, vehicles, sensors or the controllers.

## Approved traffic-generator decisions (2026-09-27)

- **S5**: random and burst arrivals are kept exactly as generated. Nothing is shifted,
  removed or suspended, even where two cars are scheduled less than 1.5 s apart.
- **S10**: WL has only the 20 deterministic arrivals at 60, 240, ..., 3480 s, with no random WL traffic.
- **S6**: demand changes exactly at t = 1800 s. The random stream switches rate at that instant.
- **RNG**: one deterministic stream per movement, seeded from the master seed plus the
  movement; streams do not depend on the scenario.

## R3 — Entry spacing in the vehicle model (from the S5 decision)

`vehicles.py` must enforce physical spacing at the 150 m entry point. If there is not
enough room when a car is scheduled to arrive, the car waits outside the modelled road
(an unbounded entry queue per movement, first in first out) until space is available.
The car keeps its original scheduled arrival time, and control delay is measured from
that time. Arrival files are never modified to achieve this.

## Vehicle model conventions (for sensors, controller loop and statistics)

- `VehicleModel.advance(dt, signals)` must be called BEFORE `SignalBank.advance(dt)` in each
  step. It reads each head's state, `since()` and, for RED_YELLOW/FLASHING_GREEN, `remaining()`
  at the start of the step, and checks that both clocks agree.
- `position` is the front bumper's distance upstream of the stop line. A car occupies
  [position, position + 4.5 m]. Stopped slot k sits at 0.5 + 7.5 (k - 1) m, so slot 1 covers
  D1 (2 m) and slot 5 covers D2 (32 m).
- "Stopped" means the car did not move during the last millisecond. `queue_length(m)` counts
  stopped cars on the approach; cars waiting outside (R3) are reported separately.
- Delay reference: `free_flow_cross_ms` = scheduled arrival + 10.8 s. Control delay =
  `cross_ms - free_flow_cross_ms`.

## Signal sequence (revision approved 2026-09-27)

Visible sequence for each movement:

    RED --start_green()--> RED_YELLOW --(timer)--> GREEN
        --end_green()--> FLASHING_GREEN --(timer)--> YELLOW --(timer)--> ALL_RED --(timer)--> RED

| State | Lamps R/Y/G | Vehicles may cross | Conflict-blocking | Ends |
|---|---|---|---|---|
| RED | 1/0/0 | no | **no** | on `start_green` |
| RED_YELLOW | 1/1/0 | no | yes | after `RED_YELLOW_S`, then GREEN |
| GREEN | 0/0/1 | yes | yes | on `end_green` |
| FLASHING_GREEN | 0/0/blinking | yes | yes | after `FLASHING_GREEN_S`, then YELLOW |
| YELLOW | 0/1/0 | only if the car reaches the line < 2.0 s after yellow onset (study §5; in `vehicles.py`) | yes | after yellow time, then ALL_RED |
| ALL_RED | 1/0/0 | no | yes | after all-red time, then RED |

**Safety**

- `start_green(m)` is accepted only if m is RED and every conflicting movement is RED
  (`conflicts.compatible`). A conflicting movement therefore cannot enter RED_YELLOW (or GREEN)
  until the terminating movement's ALL_RED has finished.
- Compatible movements are never affected, so partial transitions work exactly as before.
- RED and ALL_RED light the same lamp. ALL_RED is the terminating movement's own red
  clearance: vehicles may still be inside the junction, so it keeps blocking conflicting
  movements until its timer ends.
- RED_YELLOW blocks conflicting movements because the movement is committed to going green.
  `end_green` is refused during it.
- The invariant (no two conflicting movements in blocking states) is checked after every
  command and every `advance()`.

**Timing ownership**

| Value | Straight / left | Source |
|---|---|---|
| Yellow | 3.5 s / 3.0 s | study §6, unchanged |
| All-red | 2.0 s / 3.0 s | study §6, unchanged |
| Crossing allowed in first part of yellow | 2.0 s, end inclusive | study §5 (`vehicles.py`); boundary see "Yellow permissive boundary" |
| `RED_YELLOW_S` | 1.0 s | **project assumption.** Additional lost time before every green; no vehicle moves during it |
| `FLASHING_GREEN_S` | 3.0 s | **project assumption.** Permissive, so it counts as green time |
| `FLASH_PERIOD_S` / `FLASH_ON_S` | 1.0 s / 0.5 s | **project assumption.** Green lamp ON for the first 0.5 s of each period, from flash onset |

All timing lives in `signals.py`. The vehicle model knows only the study's 2.0 s yellow rule
and reads everything else from the signal (`since`, `remaining`).

**Green-time convention for Fixed and Smart**

- Green time G = GREEN + FLASHING_GREEN, measured from GREEN onset. Minimum green, maximum
  green and fixed-plan greens all refer to G.
- To end service at G, a controller calls `end_green` at G - `FLASHING_GREEN_S`.
- RED_YELLOW is not part of G; it is extra lost time on every start.
- With this convention the study's capacity arithmetic (s·G/C) and Webster greens keep their
  meaning. Each start costs `RED_YELLOW_S` more than in the study, and an actuated gap-out
  keeps serving for the 3.0 s of flashing green after the controller's decision.

**The study's §7 example under the new sequence.** SL's gap-out at t = 11 now starts FLASHING_GREEN
instead of YELLOW. SL is RED at 20, NS starts RED_YELLOW at 20 and turns GREEN at 21
(the study said 17). SS stays GREEN throughout.

**Lamp outputs for Pygame and ESP32.** `SignalBank.lamp_state(m)` returns an immutable
`Lamps(red, yellow, green)` for the current simulation time. `signals.lamps_for(state, elapsed_ms)`
is the same rule as a pure function. The flash depends only on simulation time since flash
onset, so a display or an ESP32 head fed the same state and elapsed time shows the same lamps.
`signals.py` imports no Pygame, serial or hardware code.

**How the vehicle model reads the states** (`VehicleModel.advance`, state at the start of the step)

- GREEN: crossing allowed; t_g = `since`.
- FLASHING_GREEN: crossing allowed if the planned crossing is before (step start + `remaining`)
  + 2.0 s, i.e. yellow onset + 2.0 s.
- YELLOW: crossing allowed if the planned crossing is before `since` + 2.0 s.
- RED_YELLOW: no crossing. The model records t_g = step start + `remaining`, the true GREEN
  onset, so the discharge rule is always measured from the instant GREEN begins.
- ALL_RED and RED: no crossing.

A signal view must therefore also provide `remaining(m)` when it reports RED_YELLOW or
FLASHING_GREEN; `SignalBank` does. Results are exact for any step of up to 3.9 s.

## Sensors (Stage 8, approved design 2026-09-27)

**Layers.** Controllers use `SensorBank` only, never `VehicleModel` or any detector source.

    VehicleModel --watch_positions() hook--> VirtualDetectors --+
    (Stage 9) ESP32 detectors -------------> <same interface> --+--> DetectorSource edges --> SensorBank --> controllers

- `DetectorSource` (interface): `sensor_ids` (all 16), `now_ms`, `initial_occupancy()`, and
  `poll(until_ms)`. `poll` returns exact timestamped `DetectorEdge(time_ms, movement, detector,
  occupied)` changes in (last poll, until_ms].
- `VirtualDetectors` implements it from the vehicle model. A Stage 9 ESP32 source must deliver
  the same edges. For R1, per-sensor mixing of real and virtual detectors can be a source that
  merges the edges of two sources; it is not built yet. **Stage 8 contains no serial, ESP32 or
  hardware code.**
- `SensorBank` holds all demand logic, so virtual and real detectors are interpreted
  identically. It exposes no vehicle objects and no true queue counts.

**Layout and raw occupancy**

- 16 sensors: D1 (2.0 m) and D2 (32.0 m) for each of the 8 movements.
- A point detector at p is occupied on [front reaches p, front reaches p - 4.5 m), i.e. until
  the rear passes. D1 therefore stays occupied until the car is 2.5 m into the junction, using
  the vehicle model's junction speed (9 m/s).
- Stopped slot 1 (0.5-5.0 m) covers D1 and slot 5 (30.5-35.0 m) covers D2; slots 2-4 and 6+
  cover neither. A car at 50 km/h covers D2 for 324 ms.
- Exact edges come from `VehicleModel.watch_positions()`, a passive hook recording the first ms
  each car's front reaches a position. Because cars only move forward, each car covers each
  detector during one interval. Edges are therefore exact to 1 ms for any step size, so a short
  pass is never missed at dt = 2 s.

**Demand lifecycle, per movement, integer ms**

Signal states split into two modes: WAITING (RED, RED_YELLOW, YELLOW, ALL_RED) and SERVICE
(GREEN, FLASHING_GREEN).

| When | Rule |
|---|---|
| WAITING | Any occupancy of D1 or D2 latches a call (`call_since_ms` = that instant, the study's t_call) at LEVEL_1, Q-hat 2. The call stays when the detectors clear. |
| WAITING | LEVEL_2 (Q-hat 6) latches when D2 has been continuously occupied for >= 2000 ms, counted from the later of D2's rising edge and the start of waiting. The count restarts when D2 clears. LEVEL_2 never falls back to LEVEL_1. |
| GREEN onset (actual RED_YELLOW -> GREEN instant) | `extended_initial_required` = LEVEL_2 latched at that instant (a filter completing exactly then counts). Then the call and level are cleared. |
| SERVICE | No calls. The gap timer runs while both detectors are clear, from the later of the last clearing and GREEN onset; any occupancy resets it. `gap_out` = gap timer >= 2500 ms (not latched). |
| Service end (FLASHING_GREEN -> YELLOW) | Back to WAITING. A detector already occupied places a new call at that instant. `extended_initial_required` returns to False. |
| Same instant | Thresholds reached by then are latched first, then the signal change, then detector edges. |

`extended_initial_required` only tells the controller to use `EXTENDED_INITIAL_GREEN_S` = 12.0 s;
the minimum-green timer itself belongs to the controller.

**Queries** (`SensorBank`)

- Raw occupancy: `occupied(m, d)`, `d1`, `d2`, `d2_occupied_ms`, `d2_filter_ms`.
- Demand: `has_call`, `call_since_ms`, `demand_level`, `qhat`, `level2_latched`.
- Service: `in_service`, `gap_timer_ms`, `gap_out`, `extended_initial_required`, `green_onset_ms`.
- Summaries: `status(m)` and `snapshot()`, both immutable `MovementSensors` records.

**Step order**

    vehicles.advance(dt, bank); sensors.advance(dt, bank); bank.advance(dt); controller decides at the new time

`SensorBank.advance` reads the signal state (and `remaining()`) at the start of the step, like
the vehicle model. RED_YELLOW -> GREEN and FLASHING_GREEN -> YELLOW are applied at their exact
instants inside the step.

## Hardware integration (Stage 9)

    VehicleModel -> VirtualDetectors ---------------------+   (computes all 16; only its assigned ones pass)
    ESP32 link(s) -> HardwareDetectorSource (mapped ids) -+-> HybridDetectorSource -> SensorBank -> controllers
    SignalBank --(read only)--> SignalMirror --SIG--> ESP32 NeoPixel head(s)

- **Configuration drives everything.** `config/hardware.json` lists devices (port, baud), real
  detectors (`"NS:D1": {device, channel}`) and physical heads (`"NS": {device, channel}`).
  Unlisted detectors are virtual. The config is validated: unknown ids, devices or keys,
  duplicate JSON keys, and a channel used twice all fail clearly. `build_hardware(config,
  vehicle_model)` wires it all together.
- **No duplicate events.** `HybridDetectorSource` forwards a source's edges only for the
  detectors assigned to it, so the virtual NS:D1 edges are dropped (and counted in
  `dropped_overridden`). `SensorBank` still sees exactly the 16 ids and cannot tell real from
  virtual.
- **Faults.** `SensorBank.detector_fault()` / `faulted_detectors()` (added in Stage 9) read an
  optional `faults()` from the source. A real detector is faulted while its link is not
  CONNECTED or its state hasn't been reported since `HELLO`. It then delivers no new edges and
  holds its last confirmed state; it is not invented and not switched to virtual. Stage 11
  decides the fail-safe.
- **Time.** Device `millis()` is mapped to simulation time as device time + the minimum over
  recent messages of (receive simulation time − device time). Wraparound is unwrapped, and a
  device reset re-syncs. Late edges go to the next ms. The library reads no clock; `hw_smoke`
  paces live runs; tests use fake transports.
- **Physical heads are mirrors.** `SignalMirror.update(bank)` sends `SIG,channel,STATE` on
  every change, after a reconnect and every 1 s. The ESP32 blinks FLASHING_GREEN itself (1 Hz,
  ON first, phase from the command) and shows ALL_RED as red. A head is not a ninth signal and
  never changes the `SignalBank`.
- **Step order with hardware:** `vehicles.advance` → `sensors.advance` (pumps the links) →
  `signals.advance` → `mirror.update(bank)` → controller decides.
- **Protocol, wiring, extension steps and the smoke test:** `hardware/README.md`.

## Fixed-time controller (Stage 10)

**What it is.** `controller_fixed.FixedTimeController` is the study §8 baseline: a conventional
four-phase, leading dual-left plan whose schedule depends only on simulation time, the
configured plan and SignalBank's own intervals. It never skips, shortens or extends a phase and
never reads detectors, queues, waiting times or hardware. That is deliberate: it is the
reference the Smart Controller is measured against, so it must not benefit from sensors.
It takes no SensorBank (constructor: plan schedule + start time) and imports only
`conflicts` and `signals`; tests check both, and check that heavy traffic, empty roads,
constantly occupied/toggling detectors and a "real" hybrid NS:D1 give identical signal logs.

**Phase order** (config/fixed_plans.json, validated with the `conflicts` API):
φA NL+SL -> φB NS+SS -> φC EL+WL -> φD ES+WS -> φA ...
Each phase is a compatible pair and every movement is served exactly once per cycle.

**One phase starting at s** (G = plan green = GREEN + FLASHING_GREEN):

| Time | Event | Issued by |
|---|---|---|
| s | `start_green` for both movements (RED -> RED_YELLOW) | controller |
| s + 1.0 | GREEN | SignalBank timer |
| s + 1.0 + G − 3.0 | `end_green` (GREEN -> FLASHING_GREEN) | controller |
| s + 1.0 + G | YELLOW (3.5 s straight / 3.0 s left) | SignalBank timer |
| + Y | ALL_RED (2.0 s straight / 3.0 s left) | SignalBank timer |
| + AR | RED; the next phase starts at this instant | SignalBank timer / controller |

Interval values are read from `signals.py`, never copied. SignalBank stays authoritative: the
controller calls only `start_green` / `end_green`, and SignalBank still refuses anything unsafe.

**FT-A cycle: study 89 s, implemented 93 s (approved 2026-09-27).**
- The study's FT-A table uses an **89 s** cycle: greens A 8 + B 25 + C 8 + D 25 = 66 s plus
  lost time L = 2 × 5.5 + 2 × 6.0 = 23 s (each phase's yellow + all-red; the study has no
  red+yellow).
- Signals V2 then added a **1.0 s RED_YELLOW before each of the four phases**. Under the
  approved safety rule it cannot overlap the previous phase's clearance, so it is extra lost time.
- Therefore the **implemented runtime FT-A cycle is 93 s** (phases 15.0 + 31.5 + 15.0 + 31.5 s),
  and every plan runs its study cycle + 4 s.
- The **published green durations are unchanged**. Greens, yellow and all-red are not shortened,
  and RED_YELLOW does not overlap clearance.
- Comparisons and validation must use the **actual implemented cycle** where the cycle matters
  (for example Webster's delay formula, capacity s·G/C, degree of saturation).

**Plans** (config/fixed_plans.json). All greens are the study §8 table; each plan stores the
study cycle and loading checks ΣG + 23 s against it. Plans must satisfy G >= FLASHING_GREEN
(3 s) and the study minimum greens (7 s straight, 5 s left).

| Baseline | How it is used |
|---|---|
| FT-A | `book.schedule("FT-A", any scenario)`: plan FT-A for every scenario |
| FT-B | `book.schedule("FT-B", "S7a")`: the scenario's own Webster plan, fixed for the whole run |
| FT-TOD | `book.schedule("FT-TOD", "S6")`: TOD/S6/first, then TOD/S6/second from 1800 s |

A plan switch takes effect at the first phase that STARTS at or after the switch time (phase
order never broken, no phase cut). The S9 FT-B cycle is 121 s as published (above the study's
60–120 s bound).

**Time and step size.** Integer ms. The runner calls `update(bank)` at the controller point of
each step (after vehicles and sensors, before `bank.advance`). A command cannot be issued late
without skipping safety intervals, so a runner must not advance past `next_command_ms()`
(`update` raises `ScheduleOvershootError` instead of drifting). At dt = 0.1 s every command time
is a multiple of 100 ms. `advance(bank, dt)` splits any dt at command times; one 3600 s call gives
the same log as 36 000 steps of 0.1 s. The first phase starts at the controller's start time
(the runner will use the warm-up start, −300 s) and requires an all-RED intersection. One
controller per run.

**Fair comparison.** Fixed and Smart runs will share the arrival file (R2), VehicleModel,
detector model, SignalBank and its intervals, step order and metrics. Only the controller
object differs.

**Simulator validation (study §16 step 5), FT-A on S1, 20 seeds.** Mean control delay of the
arrivals in [0, 3600) s (warm-up from −300 s; every car had crossed by 4500 s), 95% CI:

| | Straight | Left |
|---|---|---|
| Simulation (93 s cycle) | 36.0 s ± 0.5 | 65.8 s ± 3.2 |
| Webster's delay formula at the implemented 93 s | 38.6 s (−6.7%) | 66.4 s (−0.8%) |
| Study's published figures (89 s) | 35 s (+2.8%) | 58 s (+13.5%) |

All within the study's ±20%. This check needed the inclusive yellow boundary (below); with the
old strict boundary, lefts got only 3 cars per 8 s green (≈116 veh/h < 120 veh/h demand) and
the left delay was ≈245 s.

**Still open.** Reproducing the §8 greens with a Webster script (study §16): the study does not
state its rounding rule.

## Yellow permissive boundary (approved 2026-09-27, Stage 10 review)

The study (§5) says a vehicle "may still cross during the first 2.0 s of yellow". The vehicle
model now treats the end of that window as **inclusive**: a car whose crossing time D is
exactly yellow onset + 2000 ms crosses; D = onset + 2001 ms is blocked. The strict boundary
was an implementation choice in Stage 7 (not specified by the study). The inclusive one matches
the study's effective green = displayed green (l1 = e = 2.0 s) and its §8 capacities: an 8 s
left green serves 4 cars (4 × 3600 / 89 = 162 veh/h).

The only change is the comparison at that instant, made consistently in the three places in
`vehicles.py` (granting a crossing time, withdrawing it when the window end becomes known, and
the model's own check at the crossing). Nothing else in the vehicle model changed. A car that
crosses at the boundary is still in YELLOW and clears the box before its own all-red ends, so
signal safety is unaffected.

## Module status

| Module | Status |
|---|---|
| `conflicts.py` | Implemented and tested (study §3, §16 step 1) |
| `signals.py` | Implemented and tested (study §6, §7, §16 step 2; visible-sequence revision) |
| `traffic_gen.py` | Implemented and tested (study §5, §10, §16 step 3) |
| `vehicles.py` | Implemented and tested (study §4, §5, §16 step 4, R3); passive `watch_positions()` hook added in Stage 8; yellow window end made inclusive in the Stage 10 review |
| `sensors.py` | Stage 8: virtual detectors + demand logic; Stage 9 added `detector_fault()` / `faulted_detectors()` |
| `esp_protocol.py`, `hardware.py`, `hybrid_sensors.py`, `hw_smoke.py`, firmware | Stage 9: implemented, tested with fake serial, physically verified (commit 3cf4d68) |
| `controller_fixed.py` + `config/fixed_plans.json` | Stage 10: implemented and tested; awaiting review |
| everything else | Placeholder |
