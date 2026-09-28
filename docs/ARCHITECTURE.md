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
| YELLOW | 0/1/0 | **no** (standard policy, Stop on Yellow; legacy-study: first 2.0 s, see "Vehicle policy") | yes | after yellow time, then ALL_RED |
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
| Crossing allowed in yellow | none (standard: last crossing t_y − 1 ms); legacy-study 2.0 s, end inclusive | `vehicles.py`; see "Vehicle policy" |
| `RED_YELLOW_S` | 1.0 s | **project assumption.** Additional lost time before every green; no vehicle moves during it |
| `FLASHING_GREEN_S` | 3.0 s | **project assumption.** Permissive, so it counts as green time |
| `FLASH_PERIOD_S` / `FLASH_ON_S` | 1.0 s / 0.5 s | **project assumption.** Green lamp ON for the first 0.5 s of each period, from flash onset |

All timing lives in `signals.py`. The vehicle model knows only its policy's yellow rule (no
entry from yellow onset; legacy-study: + 2.0 s) and reads everything else from the signal
(`since`, `remaining`).

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
- FLASHING_GREEN: yellow onset t_y = step start + `remaining`; crossing allowed if the planned
  crossing D <= t_y − 1 ms (standard) or D <= t_y + 2.0 s (legacy-study).
- YELLOW: the same window end measured from `since` (standard: nothing new crosses).
- RED_YELLOW: no crossing. The model records t_g = step start + `remaining`, the true GREEN
  onset, so the discharge rule is always measured from the instant GREEN begins. Standard
  policy: if GREEN begins inside the step, the step is split there (first car starts exactly
  at t_g for any dt).
- ALL_RED and RED: no crossing.

A signal view must therefore also provide `remaining(m)` when it reports RED_YELLOW or
FLASHING_GREEN; `SignalBank` does. Standard results are exact for any step; legacy-study
results for any step of up to 3.9 s.

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

**Simulator validation (study §16 step 5), FT-A on S1, 20 seeds** — historical, made with the
study vehicle rules (now the `legacy-study` policy; `tests/test_runner.py` keeps it as a
legacy regression). Mean control delay of the
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

## Yellow permissive boundary (approved 2026-09-27, Stage 10 review) — legacy-study only

*Historical.* Since 2026-09-28 the standard policy lets no vehicle enter after yellow onset
(see "Vehicle policy"). This section describes the `legacy-study` compatibility policy, which
produced the Stage 10–14 results.

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

## Smart controller (Stage 11, approved design 2026-09-27)

`controller_smart.SmartController(sensors, params)` is the study §7 movement-based, fully
actuated controller, implemented as written (no tuning). It reads only `SensorBank`
(`has_call`, `call_since_ms`, `qhat`, `gap_out`, `extended_initial_required`,
`faulted_detectors`, `now_ms`) and `SignalBank` (`state`, `since`, `can_start_green`,
`start_green`, `end_green`). It imports only `conflicts` and `signals`, and never learns
whether a detector is real or virtual. `SmartParams` holds the study values and the three §9
ablation switches (max green, aging, W_MAX); rest-in-red is not implemented.

**Shared step order (approved; also for the Stage 14 runner and both controllers):**
vehicles → sensors → `SignalBank.advance` → controller `update(bank)` → mirror. The Smart
Controller checks that sensors and signals are at the same instant. This supersedes the
Stage 10 note ("before `bank.advance`"); the fixed controller's command times are the same
either way, and `FixedTimeController` is unchanged. Decisions happen at step boundaries; the
official step is 100 ms. Decision logs are deterministic for identical step sequences.

**Values** (study §6, §14): minimum green 7 s straight / 5 s left, 12 s when LEVEL_2 is latched
at the actual GREEN onset (`extended_initial_required`); maximum green 45 s / 30 s, timer
started at the first update (while GREEN) at which a conflicting movement has a call; passage
time 2.5 s (SensorBank's gap-out, not re-timed); τ = 10 s; Q̂ = 2 / 6; W_MAX = 90 000 ms.
Green time G = GREEN + FLASHING_GREEN: `end_green` for a minimum green MIN is allowed from
onset + MIN − 3 s, a max-out issues it at max start + MAX − 3 s, a gap-out issues it at once.

**Algorithm per update** (study pseudocode):
1. Track each movement's GREEN onset, chosen minimum green and max-green start. These survive
   partial transitions (a movement that stays GREEN keeps them).
2. If a transition is committed, only start its target movements as soon as SignalBank allows;
   release the commitment when all of them are GREEN.
3. Waiting = RED with a call. No waiting calls → rest in the current greens (even gapped out).
4. Hold: the waiting call with the longest wait ≥ 90 000 ms (ties: MOVES order).
5. Target: score every compatible group (fixed `GROUPS` order, members in MOVES order):
   S(G) = Σ P over waiting members + 2 per member serving (RED_YELLOW or GREEN) and not gapped
   out; P = Q̂ + wait/τ, compared exactly as Q̂·10 000 + wait_ms. With a hold, only groups
   containing it. Exact ties: more serving members, then the longest single wait, then
   GROUPS order.
6. Add compatible demand: start a waiting movement if SignalBank allows it and it is compatible
   with BOTH target members (target protection, literal study rule). Nothing is stopped.
7. Switch: end only the GREEN movements that conflict with the target's waiting members, and
   only when ALL of them are releasable at once (minimum green met and gap-out, max-out, or
   conflict with the hold), then commit.

**Fail-safe recall for faulted detectors (approved).** While any detector of a movement is
faulted (`SensorBank.faulted_detectors()`, however the fault arises), the movement has demand
with Q̂ = max(its real Q̂, 2), its call time is the later of the fault onset and its last
service end, and its gap-out is ignored (it ends by max-out or a hold). Consequence: a
movement with a disconnected real detector is served every time it can be, for up to its
maximum green while conflicting calls exist, whether or not traffic is present — the
efficiency of that movement falls to roughly fixed-time level, but it can never disappear or
lock up. A real detector that has not reported since HELLO (startup) is also on recall.

**Starvation bound.** The hold threshold is exactly 90 s. ≈ 120 s is the theoretical
single-hold time until GREEN under Signals V2 (≈ 10.5 s of a transition already committed,
≈ 18 s of the new greens' minimum green and clearance, 1 s red+yellow). It is not a guarantee:
the study algorithm does not bound the wait when several holds arise together (they are served
one after another) or when the hold's best group needs a green that is compatible with the
hold but conflicts with its partner (step 7 needs all blockers releasable at once; that green
must gap out or max out). Measured maximum waits at RED (study policy, FT-A comparison below,
seeds 1–3): S1 133–149 s, S3 122–138 s, S7a 139–149 s, S10 149–158 s. (Official campaign, seeds 1–10: see "W_MAX interpretation" in the Stage 14 section.) Every wait above 120 s
in those runs was classified: committed transition in progress, partner interaction, or
another hold first — none unexplained.

**Known limitation (study §15).** Q̂ saturates at 6: once both are LEVEL_2, a moderately and a
very heavily queued movement look the same. Combined with the unbounded waiting-time term,
split compatible groups such as NS+NL / SS+SL can outrank NS+SS, which serves only one straight
at a time and reduces straight capacity under balanced or heavy demand. This is reported, not
optimised around. Any later tuning follows the study: tune only on S1 with seeds 101–105, then
lock the parameters before evaluating the other scenarios.

**Exploratory validation (NOT the Stage 14 results).** Mean control delay (s) of arrivals in
[0, 3600) s, straight / left, seeds 1–3, dt = 0.1 s, same arrivals for both controllers;
last column: longest wait of a call at RED.

| Scenario | FT-A | Smart (study policy) | Smart longest RED wait |
|---|---|---|---|
| S1 balanced | 35.2 / 68.3 | 55.8 / 58.6 | 148.9 s |
| S3 one approach nearly empty | 34.7 / 68.1 | 37.0 / 47.3 | 137.5 s |
| S4 very low | 26.7 / 41.4 | 14.6 / 18.0 | 77.8 s |
| S7a heavy | 215.3 / 247.1 | 173.2 / 179.9 | 148.7 s |
| S10 long wait | 215.3 / 245.2 | 238.2 / 117.9 | 158.2 s |

FT-A is the single S1 design plan (so it is oversaturated in S7a/S10); FT-B was not run here.

## Statistics and shared runner (Stage 12, approved design 2026-09-27)

`runner.py` runs one simulation from a `RunConfig` (scenario, seed, controller FT-A / FT-B /
FT-TOD / Smart, `SmartParams`, dt = 100 ms, warm-up 300 s, measurement 3600 s, flush ≤ 900 s).
Everything except the controller is built identically: arrivals from
`get_or_create_arrivals` (one file per scenario and seed, R2) or an explicit schedule,
`VehicleModel` and `SignalBank` starting at −300 s, `VirtualDetectors` (or a source factory,
e.g. hybrid), `SensorBank`. Fixed controllers start their first cycle at −300 s.
`run_pair()` runs several controllers on the same arrivals and checks the checksum.

Step order (shared by both controllers): vehicles → sensors → `SignalBank.advance` →
`controller.update` → statistics → read-only observers (`StepView`, for Stage 13). Steps are
dt long, shortened only so that every 1 s sample instant, the measurement end and the fixed
controller's next command are step boundaries (never at dt = 100 ms). The run ends at the
first step ≥ 3600 s at which every measured vehicle has crossed, at 4500 s at the latest.

`stats.py` only observes (it calls no state-changing method; tests check this and compare
runs with and without statistics). Official metrics (study §11), per movement and
intersection-wide; V = vehicles scheduled in [0, 3600 s), C = those that crossed:

| Metric | Definition |
|---|---|
| Control delay | d = cross − (scheduled + 10.8 s) over C: mean, P95 (numpy linear), max, total veh·h |
| Unfinished | V not crossed at the run end (4500 s): excluded from the delay statistics, always counted; `delay_mean_lower_bound_s` includes them at d = end − free-flow crossing time |
| Vehicle waiting time | stopped time on the road + time held outside the 150 m entry, over C (not the detector-call wait) |
| `vehicles_control_delay_over_90s` | vehicles of C with d > 90 000 ms (study "waiting > 90 s", G7; not stopped time, not W_MAX) |
| Queue | stopped on the road + held outside the entry (both parts also stored), sampled exactly at t = 0, 1, …, 3599 s; per movement mean and max, headline = mean of the 8 movement means (G3), intersection-total mean and max; raw samples kept |
| Throughput | crossings of any vehicle (warm-up arrivals included) with cross in [0, 3600 s), veh/h; a measured vehicle crossing in the flush adds delay but no throughput |
| Served / % served | vehicles of V crossing before 3600 s |
| Green utilization | n × h / G: n = crossings in the hour, h = 1.9 / 2.0 s, G = displayed GREEN + FLASHING_GREEN seconds inside the hour. Under the standard policy a fully used green serves about G/h + 1 cars (first car at green onset), so the value can slightly exceed 1; under legacy-study (crossings in the first 2 s of YELLOW included, first car at t_g + 3.9 s) about G/h |
| Signal changes | GREEN onsets of the 8 signals in the hour, per hour |
| Jain's index | over movements with ≥ 1 crossed measured vehicle; 1.0 if all their means are 0; None for an empty run |
| Safety violations | steps with conflicting active movements (must be 0) |

Diagnostics (separate, may be None, never part of the official metrics): detector-call wait
maximum and calls over 120 s, time with both vs one opposing straight in service (split
groups), LEVEL_2 latched while the true queue exceeds 6 (Q̂ saturation), Smart terminations
(gap / max / hold) and hold decisions, faulted seconds and green while faulted.

Results: `RunResult` (metadata with scenario, seed, controller and parameters, arrival
checksum, dt, windows, step order; `IntersectionStats`; 8 `MovementStats`; diagnostics; 1 s
queue samples; signal log; `VehicleRecord`s) with deterministic `to_dict` / `to_json` and
`summary_row`, `movement_rows`, `vehicle_rows`, `queue_rows`. Across seeds: `mean_ci95`
(t interval; n = 1 → no interval; zero variance → width 0) and `paired_comparison` (paired
differences, 95 % CI, paired t-test only when n ≥ 2 and the differences vary).

Known properties: in this vehicle model (stop or free speed, no acceleration) a vehicle's
waiting time equals its control delay exactly; the metric is kept as the study defines it.
Queue averages come from 1 s samples as the study specifies (not a time integral). Stops
and travel time are not study metrics and are not computed (raw times are kept per vehicle).

## Visualization (Stage 13)

Presentation layer only; no core module was changed.

* **Modules.** `viz_geometry.py` (pure geometry, no pygame), `viz_session.py` (simulation
  thread, snapshots, pacing; no pygame), `viz_render.py` (pygame drawing of the scene and the
  panels), `viz.py` (application: window, events, controls, CLI, still frames).
* **Simulation.** `viz_session.VizSession` calls the Stage 12 `runner.run(config,
  observers=[...])` (and `source_factory=` with `hardware.build_hardware` in hardware mode)
  on a background thread. The observer is the last action of every step (after statistics):
  it updates the Stage 9 `SignalMirror` (hardware mode only; read-only on SignalBank), builds
  an immutable `Snapshot`, then sleeps to pace the run or blocks while paused. Sleeping or
  blocking between complete steps cannot change a step, so results are identical with and
  without the visualization, at any speed and with any pauses (tests compare `RunResult`
  JSON). Reset aborts the run from inside the observer and starts a new session.
* **Time.** Simulation time stays the runner's integer-ms steps (dt 100 ms). The display time
  follows the wall clock × speed; the simulation computes one step ahead so frames can
  interpolate vehicle positions between two snapshots (visual only). Paused: display time
  and simulation are frozen; on resume the display continues from the frozen time. At "Max"
  the run is unpaced and snapshots are taken at most every 30 ms of wall time. Hardware mode
  is locked to 1× because the ESP32 link maps device time to simulation time.
* **Signals.** Snapshot records each head's `SignalState` and state-entry time; lamps are
  `signals.lamps_for(state, t - since)` at the display time (so FLASHING_GREEN blinks exactly
  as SignalBank/firmware do). There is no signal logic in the visualization.
* **Geometry.** Right-hand traffic; per approach an inner LEFT lane (centre 2.4 m from the
  axis) and an outer STRAIGHT lane (6.0 m): 3.6 m lanes, a 1.2 m painted median, a 2.2 m
  cycle lane to the kerb; arms and box 20 m kerb to kerb (box = the straight BOX_LENGTH). Straight paths cross the box;
  left turns follow a smooth curve into the inner outbound lane of the arm on the left (the
  model's 25 m left box length is mapped onto the curve); compatible movements' paths stay
  ≥ 3.6 m apart (lane pitch); opposing left turns ≈ 6 m. Vehicle drawing position = the model's front-bumper position along that path.
  Exited vehicles are drawn for 6 s on the exit arm (visual only).
* **Detectors.** The 16 logical positions (D1 2 m, D2 32 m). The configured real detector's
  position is drawn with the "IR" style in place of the virtual one (hardware off: dimmed, it
  is simulated). Fault = red (Stage 9 `faulted_detectors`).
* **Smart panel.** Target group, starvation hold, and a reason derived from public controller
  and sensor state (`last_target`, `last_hold`, `committed()`, `decision_log`,
  `green_onset_ms`, `min_green_of`, `max_start_ms`, sensor gap timer). P shown = Q̂ + wait/τ
  recomputed from public SensorBank data (fault recall movements marked "R").
* **Live metrics.** Vehicles in system (road + box + held outside), queue (Stage 12
  definition), crossings since t = 0, flow over the last 5 min, mean control delay of measured
  vehicles that have crossed so far (equals the final `delay_mean_s` at the end of the run),
  longest current call. Final Stage 12 statistics are shown when the run completes.

## Experiments and tuning (Stage 14, approved design 2026-09-28)

**Rules (study §10, §15, §16 step 9).** Smart-controller parameters are tuned only on S1 with
seeds 101–105 and then frozen; all scenarios are evaluated on seeds 1–10. Only `tau_s` and
`w_max_s` are tuned (study Assumptions: "τ, Q̂ and W_MAX are our choices"); every other
`SmartParams` value stays at the study default (max green 45/30 s, min greens, extended initial,
green bonus, fault recall). Max green 60/40 s is a later sensitivity run, not a tuning option.
The study states no objective; the approved rule is pre-registered in `config/tuning_plan.json`
(frozen before any tuning run, SHA-256 recorded):

* primary: minimum mean control delay over the 5 tuning seeds;
* hard: safety violations 0, no simulation error (SignalBank refusal), signal log clean (legal
  transitions, exact RY / FG / Y / AR durations, GREEN + FLASHING_GREEN ≥ the standard minimum
  green), unfinished 0, one arrival checksum per seed for every run, determinism (the winner
  and Smart-study are re-run and must reproduce their result hashes);
* guardrails vs Smart-study (study defaults): mean P95 delay must not increase; the worst
  per-run maximum delay may increase by at most 10 %;
* ties: within 1 % of the best feasible mean delay prefer fewer changed parameters, then the
  smallest Σ|x − default| / default, then the grid order (τ, then W_MAX ascending).

Grid: τ ∈ {5, 10, 15, 20, 30} s × W_MAX ∈ {90, 105, 120, 150} s (20 candidates, the study
default (10, 90) included); references FT-A and FT-B (identical plans in S1).

**Disclosure.** Before tuning, exploratory Stage 11 work (the table above) looked at some
evaluation seeds (1–3 of S1, S3, S4, S7a, S10) with the untuned study controller. Those results
were not used for the grid, the rule or the selection; tuning used only S1 seeds 101–105.

**Infrastructure.** `experiments.py`: `RunSpec` (scenario, seed, controller, label, SmartParams,
study windows) → run key = SHA-256 of everything the simulation depends on (not the label; FT
runs ignore Smart parameters); the parent creates every arrival file serially, workers (spawn
process pool, Windows-safe, default min(4, CPUs)) only read them and verify the checksum, run the
unchanged Stage 12 runner and write `runs/<key>.json.gz` via temp file + atomic rename; resume
skips valid files and re-runs missing, corrupt or partial ones; summaries are rebuilt in a fixed
order, so sequential, parallel and resumed campaigns are byte-identical. A failed simulation is
recorded with its error, never dropped. `tuning.py`: plan validation, candidates, the selection
rule (pure, order-independent), the lock and the guarded official matrix.
`python -m smarttraffic.run_experiments tune [--workers N]` runs everything above.

**Lock.** `config/smart_params_locked.json` = "Smart-T1": full `SmartParams`, the tuned
parameters, the plan hash, the candidate-table hash and its own `lock_sha256`. `load_lock()`
refuses a missing or edited lock and any lock that changes an untuned parameter; the official
matrix (`official_specs`, `check_official`) uses only Smart-T1 = the lock and Smart-study =
`SmartParams()` (which stays the study default), the study windows, seeds 1–10 and the official
scenarios — no per-scenario or custom parameters.

**Selected and approved (2026-09-28): Smart-T1 = τ 15 s, W_MAX 90 s**, every other parameter at
the study default; lock_sha256 `3b3da0427f7f965f23a1c586a782fbeb3be8445608dc2eaf4135f5512e3ca84c`.
It is not retuned after seeing its S1 result (still worse than FT-A on the tuning seeds, where the
study expects fixed time to do well).

**Official campaign (`campaign.py`, `run_experiments official --lock-sha …`).** S1–S10 ×
{FT-A, FT-B, Smart-T1, Smart-study} × seeds 1–10, FT-TOD on S6, then the study extras Smart-T1
without W_MAX on S10 (G7) and Smart-T1 without aging on S1, S6, S9 (G12): 490 runs. The lock hash
must be stated; `check_official` refuses any other scenario, seed, window, label or parameter
set. Outputs in `results/official/campaign_T1/`: `runs.csv`, `movements.csv`, `aggregates.csv`
(per scenario × label × metric: n, mean, sd, 95 % t-interval), `paired.csv` (same-seed B − A with
95 % CI and paired t-test for Smart-T1 vs FT-A / FT-B / Smart-study, FT-TOD vs FT-B on S6, and
the extras vs Smart-T1; p-values unadjusted for multiple comparisons, as the study specifies no
correction), `manifest.json`. Green utilization of a run = mean over movements with green time.

**W_MAX interpretation (official results, accepted 2026-09-28).** W_MAX = 90 s is a
starvation-HOLD threshold, not a proven hard upper bound on detector-call waiting time. In the
official campaign (seeds 1-10) Smart-T1's maximum detector-call wait exceeded 90 s in every
scenario except S4 (131-156 s), and calls waiting more than 120 s occurred in 10 of 11
scenarios (up to about 92 per run in S7b). Waits can exceed 90 s because several movements can
become overdue together (holds are served one after another, longest wait first), the held
movement may wait behind another overdue movement, and minimum greens and the mandatory
clearance transitions (flashing green, yellow, all-red, red+yellow) are never skipped. In S10
the low-demand WL car waited 97.7 s on average and 157 s at worst under Smart-T1 (FT-A
44.8 / 82.7 s, FT-B 52.3 / 109.7 s). W_MAX is still useful: removing it (Smart-T1-noWMAX, S10)
made performance substantially worse (mean delay +140 s, max delay +333 s, throughput
-168 veh/h, worst WL wait about 209 s, all significant). The parameter and the controller were
not changed after these results.

**Outputs.** `results/tuning/S1_T1/`: `manifest.json` (plan hash, code fingerprint, commit if
available, environment, CSV hashes, lock hash), `runs.csv`, `candidates.csv` (ranked, with reject
reasons), `selection.json` (winner, guardrails, paired comparisons, determinism); `runs/` holds
the compressed full records and is not committed (.gitignore).

**Reproduction (verify-only).** The lock is written once and never overwritten. A reproduction
(for example on another PC) runs `tune --out <dir> --verify-lock <lock_sha256>`: it checks the
approved lock and the plan hash before any run, reuses completed runs in `<dir>/runs/`, and at the
end requires the same winner and a byte-identical `candidates.csv`; it never writes the lock
(`manifest.json` records `lock_mode: "verified (not written)"`). Without `--verify-lock`, `tune`
refuses to start when a lock already exists. `fingerprints.csv` holds per run a SHA-256 of the
simulated trajectory (vehicles, signal log, queue samples, metadata); `verify-tuning --ref
results/tuning/S1_T1 --new <dir> --lock-sha <sha>` requires identical candidates, fingerprints and
metric columns. Full-record hashes may differ between operating systems only in the last bit of
derived floating-point statistics (e.g. Jain's index via `pow`); the simulation itself is exact.

## Custom / Demo mode (Stage 14b)

**Separation.** `runner.RunConfig` has `mode` ("official" default, "custom") and `demand`
(`custom.CustomDemand`, custom only). Official mode refuses custom demand; custom keys are added
to the metadata only in custom mode. The official experiment path cannot express a custom run
(`experiments.RunSpec` has no mode/demand fields) and `run_campaign` refuses `results/custom/`.
Custom output (`custom.save_custom`) refuses `results/official`, `results/tuning`,
`data/arrivals` and `config`, and requires the result's `CUSTOM / DEMO` tag. Custom arrivals are
generated in memory with the same per-movement streams (seed, movement), so S1 demands + seed
reproduce S1's arrivals exactly. Custom runs use the standard vehicle policy (no per-run
option; the Stage 14b yellow-policy and start-up switches were replaced by the project-wide
policy below).

**Custom inputs.** 8 constant demands (number, finite, >= 0, < 2400 veh/h; UI cap 1800), seed
(int >= 0), FT-A or Smart (no FT-B custom planning), measurement 60–7200 s (UI: 5/10/30/60 min),
study warm-up 300 s, flush <= 900 s, dt 100 ms. Smart parameter set: Smart-T1 (the lock,
default) or custom (T1 with only tau, W_MAX and the two max greens changed,
within UI ranges; SmartParams' own checks apply). Load check: dual-ring critical flow ratio
Y = max(y_NL + y_SS, y_SL + y_NS) + max(y_EL + y_WS, y_WL + y_ES), y = q / (1895 veh/h
straight, 1800 left); X = Y / (1 - 23/120); X >= 0.85 near capacity, X >= 1 or any y >= 1
oversaturated (S1 0.69; S2, S7a, S10 0.88–0.94; S7b 1.22).

**Visualization.** "Custom…" after S10 in the scenario selector; the modal panel edits a
`viz_custom.CustomDraft` (no pygame) and Apply loads the validated spec paused. While the panel
is open only its own controls are clickable. Advanced settings are collapsed by default. Badge:
CUSTOM / DEMO (top bar pill, selector, ready/complete banners); official scenarios show
"OFFICIAL · LOCKED" and keep the locked settings. Ready banners and the Advanced section state
the traffic rules ("Stop on Yellow · first queued car starts at green").

**Smart in the UI = Smart-T1 (approved 2026-09-28).** Every official configuration the
visualization runs uses the locked Smart-T1 parameters (`viz.ui_config`: tau 15 s, W_MAX 90 s,
max greens 45 / 30 s and all other locked values); the top bar says "Smart actuated ·
Smart-T1". The Custom panel offers Smart-T1 (default) or custom tau / W_MAX / max greens only.
Smart-study is not a user-facing option; it remains internal (official campaign comparison,
historical tests, `custom.smart_params_for("study")`). No official result changed.

## Vehicle policy (project decision 2026-09-28)

**Standard behaviour everywhere.** `vehicles.VehiclePolicy` has exactly two named values:

| | `standard` (default) | `legacy-study` (compatibility only) |
|---|---|---|
| Yellow | Stop on Yellow: last permitted crossing t_y − 1 ms; a car due at t_y stops | permissive first 2.0 s, inclusive (D <= t_y + 2000 ms) |
| Discharge floor | D >= t_g (first queued car starts at green onset; from slot 1 it crosses 36 ms later) | D >= t_g + l1 + h (3.9 s straight / 4.0 s left) |
| Following cars | previous + h (1.9 / 2.0 s), one at a time | same |
| 10-car straight queue clears | 17.136 s after green | 21.0 s |
| Step with a GREEN onset inside | split at t_g (exact for any dt) | unchanged Stage 14 stepping (exact for dt <= 3.9 s) |

`VehicleModel(..., policy=...)`, `RunConfig.vehicle_policy`, `RunSpec.vehicle_policy`,
`tuning.tuning_specs / official_specs / extra_specs`, `campaign.campaign_specs / run_official`
and `run_experiments --policy` all default to `standard`. The visualization always runs
`standard` (`viz.ui_config` enforces it for official runs, Custom / Demo, screenshots, reset
and every scenario / controller / seed switch) and never offers `legacy-study`. The old
`VehicleOptions`, `yellow_extension_s`, `immediate_first_vehicle`, `--yellow` and
`--immediate-first` switches were removed.

Why this is consistent. Stop on Yellow loses the study's 2.0 s yellow extension; the immediate
start gains l1 + h = 3.9 / 4.0 s. Net, a green served to saturation discharges about one car
more than under the study rules (e.g. FT-A's 8 s left green: still 4 cars, at t_g + 0.036,
2.036, 4.036, 6.036 s; the 5th, due at t_y + 36 ms, stops). No car ever enters in YELLOW, so
the clearance (yellow + all-red) is pure clearance.

**Legacy compatibility.** `legacy-study` is exactly the Stage 14 model: run identities have no
policy field (same run keys), metadata has no policy keys, and the golden fingerprints of the
490 official runs (`tests/golden/official_T1_baseline.csv`) are reproduced (tests re-run S4
seed 1). Standard runs add `vehicle_policy` to the run identity (different keys, no resume from
historical records) and `vehicle_policy`, `stop_on_yellow`, `immediate_first_vehicle` to the
metadata. A campaign folder holds one policy (`vehicle_policy.json` marker; a folder with Stage
14 records and no marker is legacy-study) and refuses the other. Default output folders:
standard `results/tuning/S1_T1_standard`, `results/official/campaign_T1_standard`;
legacy-study `results/tuning/S1_T1`, `results/official/campaign_T1`.

**Historical results and the assumptions that are now historical.** The following were
produced with `legacy-study` and are kept unchanged, as the record of that study policy:

- `results/tuning/S1_T1` (and the PC reproduction `S1_T1_pc`): candidate ranking, reject
  reasons, guardrails and the selection of τ 15 s / W_MAX 90 s.
- `config/smart_params_locked.json`: Smart-T1 was *selected* under legacy-study
  (`tuning.lock_policy`); its parameters are kept for the standard policy without retuning,
  so the tuning evidence does not transfer automatically. `tune --verify-lock` under the
  standard policy is refused.
- `results/official/campaign_T1` (490 runs): every delay, queue, throughput, % served, green
  utilization, P95/max, delay > 90 s, detector-call-wait, fairness and paired-comparison
  figure, including the W_MAX findings (call waits up to about 156 s; removing W_MAX worsens
  S10) and the no-aging ablation.
- The Stage 10 FT-A validation (±20 % vs Webster / study figures) and the "inclusive yellow
  boundary" capacity argument (effective green = displayed green, l1 = e = 2.0 s).
- The FT-A / FT-B / FT-TOD plans (`config/fixed_plans.json`) are unchanged study / Webster
  plans (L = 23 s); their capacity arithmetic assumed the study start-up and yellow rules.
  Under the standard policy each saturated green serves about one car more.
- `custom.saturation_report` still uses L = 23 s, so its X is slightly conservative now.

Nothing above was rerun or overwritten. New standard-policy campaigns, if wanted later, go to
their own folders.

## Result figures (Stage 15)

`python -m smarttraffic.make_graphs` builds the report / presentation figures from committed data
only: the official campaign's `runs.csv`, `aggregates.csv`, `paired.csv` (each verified against the
SHA-256 in `manifest.json`), the tuning `selection.json` / `runs.csv` (historical Smart-T1 vs
Smart-study figure), and a committed S6 time-series extract. `make_graphs extract-s6` builds that
extract from the per-run records (or re-simulates the 40 runs) and accepts a run only if its run
key and simulated-trajectory fingerprint equal the official run's (`tests/golden/`).

Policy separation: a figure set is built from one vehicle policy. The campaign and tuning manifests
(missing field = legacy-study), the S6 extract and the output folder
(`results/figures/<campaign>__<policy>`) must all agree, otherwise `PolicyMixError`. The Stage 14
figures are therefore `results/figures/campaign_T1__legacy-study/`; every figure carries a
"LEGACY-STUDY VEHICLE POLICY (HISTORICAL RESULTS)" tag, a source line and PNG/PDF/SVG metadata.

Figures (study §12 numbering in brackets): F01 mean delay [G1], F02 P95 [G2], F03 max delay, F04
throughput + unfinished [G4], F05 mean queue [G3], F06 Jain fairness, F07a vehicles > 90 s, F07b S10
call waits vs W_MAX and the no-W_MAX extra [G7], F08a signal changes [G11], F08b green utilization,
F09 / F10 paired Smart-T1 − FT-A / FT-B forest plots (CI vs zero only, unadjusted), F11 historical
Smart-T1 vs Smart-study, F12 S6 queue over time and delay by arrival window [G5], F13 key scenarios.
Style: DejaVu Sans, one validated colour per controller (FT-A blue, FT-B orange, Smart-T1 aqua,
FT-TOD yellow; Smart-study violet and Smart-T1-noWMAX hatched magenta only where named), bar axes from
zero, small multiples with per-panel scales where magnitudes differ by up to 50×.
Outputs are deterministic: `figures_values.csv`, `figures_manifest.json` (no timestamps),
`README.md`, `.pdf` and `.svg` are byte-identical on re-run on one machine.
Not covered (no data): G6 demand sweep, G8 signal Gantt, G9 CDF, G10 per-movement, G12 ablations
beyond no-aging.

## Module status

| Module | Status |
|---|---|
| `conflicts.py` | Implemented and tested (study §3, §16 step 1) |
| `signals.py` | Implemented and tested (study §6, §7, §16 step 2; visible-sequence revision) |
| `traffic_gen.py` | Implemented and tested (study §5, §10, §16 step 3) |
| `vehicles.py` | Implemented and tested (study §4, §5, §16 step 4, R3); passive `watch_positions()` hook added in Stage 8; yellow window end made inclusive in the Stage 10 review; named vehicle policies (standard default, legacy-study compatibility) 2026-09-28 |
| `sensors.py` | Stage 8: virtual detectors + demand logic; Stage 9 added `detector_fault()` / `faulted_detectors()` |
| `esp_protocol.py`, `hardware.py`, `hybrid_sensors.py`, `hw_smoke.py`, firmware | Stage 9: implemented, tested with fake serial, physically verified (commit 3cf4d68) |
| `controller_fixed.py` + `config/fixed_plans.json` | Stage 10: implemented, tested, verified (commit 7e4ae52) |
| `controller_smart.py` | Stage 11: study §7 policy implemented, tested, verified (commit d98c4dd) |
| `stats.py`, `runner.py` | Stage 12: implemented, tested, verified (commit fde1505) |
| `viz.py`, `viz_render.py`, `viz_session.py`, `viz_geometry.py` | Stage 13: implemented, tested, verified (commit 6d2ca41) |
| `experiments.py`, `tuning.py`, `campaign.py`, `run_experiments.py`, `config/tuning_plan.json`, `config/smart_params_locked.json` | Stage 14: Smart-T1 approved and locked; official campaign run and accepted (490 runs) |
| `custom.py`, `viz_custom.py`, `tests/golden/official_T1_baseline.csv` | Stage 14b: Custom / Demo mode (committed a7da151); standard vehicle policy project-wide (awaiting review) |
| `make_graphs.py`, `results/figures/` | Stage 15: result figures from the Stage 14 (legacy-study) summaries (awaiting review) |
| everything else | Placeholder |
