# Stage 16 — Final integration and stress verification

Checkpoint verified: `main` = `8e68f4c7c2b7234c011b8d5a43d82616322eb596` (hardware update), full suite
1338 passed before Stage 16. Smart-T1 is the locked parameter set (τ 15 s, W_MAX 90 s, max green
45 s straight / 30 s left; lock_sha256 3b3da042…). Every run uses the standard vehicle policy
(Stop on Yellow; the first queued car starts at green onset). Nothing was retuned; the Stage 14
historical results and the Stage 15 figures were not touched.

## How it was verified

* **Independent audit** — `tests/stage16_checks.py`. A read-only runner observer checks every
  step, and after the run it rebuilds every signal interval from `SignalBank.log` and every
  stop-line crossing from the vehicles. It then checks the safety rules on those records
  **without** using the code that produced them. It is itself tested: injected illegal
  transitions, shortened intervals, conflicting greens, red-light crossings, short headways and a
  late first car are all detected (`tests/test_stage16_safety.py`).
* **pytest** — 112 new Stage 16 tests (short windows) in `tests/test_stage16_safety.py` (60),
  `test_stage16_sensor_smart.py` (14), `test_stage16_hardware_chain.py` (7), `test_stage16_ui.py` (31).
* **Full-length matrix and stress runs** — `python -m tests.stage16_stress` (25 min on 2 cores):
  130 jobs, 124 of them audited step by step (`results/stage16/stress_report.json`).

## 1. Traffic safety invariants (all audited runs)

| Invariant | Result |
|---|---|
| No conflicting movements in a blocking state at the same time (interval overlap, not only step ends) | **0 conflicts** in 229 751 transitions / 32 782 green onsets |
| Sequence RED → RED_YELLOW → GREEN → FLASHING_GREEN → YELLOW → ALL_RED → RED, exact durations (RY 1.0 s, FG 3.0 s, Y 3.5 / 3.0 s, AR 2.0 / 3.0 s) | 0 violations |
| No vehicle crosses in RED, RED_YELLOW, YELLOW or ALL_RED; none at or after yellow onset | 0 of 281 533 crossings |
| First queued car crosses exactly 36 ms after GREEN onset (starts at the onset) | 33 851 green starts checked, 0 deviations |
| Following headways ≥ 1.9 s straight / 2.0 s left, FIFO, no simultaneous release, no duplicate crossing | 0 violations |
| Smart partial transitions: every end_green had a valid reason (gap ≥ 2.5 s, max-out, or a conflicting hold ≥ W_MAX), never before the minimum green (12 s when LEVEL_2 at onset) | 11 633 end-of-green decisions checked (gap 5 356, hold 5 010, max 1 267), 0 violations |
| Virtual detectors equal the vehicle bodies over them; Q-hat 0 / 2 / 6; calls never in service | 0 violations (4 965 LEVEL_2 latches) |
| Result numbers finite, no time before the run start, no negative control delay | 0 violations |
| Logs grow only with events (≤ 4 detector edges per vehicle; ≤ 6 transitions per green) | holds in every run |

## 2. Software end-to-end matrix (full length: 300 s warm-up + 3600 s + ≤ 900 s flush, seeds 1–3)

These are verification runs under the standard vehicle policy, not an official campaign: the
delays below are for checking behaviour, not study results (the official Stage 14 results are
the historical legacy-study ones).

| Scenario | Controller | Vehicles audited (3 seeds) | Conflicts | Audit errors | Unfinished (seed 1 / 2 / 3) | Mean delay s (seed 1 / 2 / 3) | Longest call wait s |
|---|---|---|---|---|---|---|---|
| S1 | FT-A | 6744 | 0 | 0 | 0 / 0 / 0 | 38.4 / 41.8 / 40.0 | 83.9 |
| S1 | FT-B | 6744 | 0 | 0 | 0 / 0 / 0 | 38.4 / 41.8 / 40.0 | 83.9 |
| S1 | Smart-T1 | 6744 | 0 | 0 | 0 / 0 / 0 | 49.4 / 48.6 / 45.6 | 146.0 |
| S2 | FT-A | 8007 | 0 | 0 | 39 / 49 / 83 | 235.5 / 223.6 / 263.4 | 83.9 |
| S2 | FT-B | 8007 | 0 | 0 | 0 / 0 / 0 | 66.0 / 62.8 / 67.1 | 113.9 |
| S2 | Smart-T1 | 8007 | 0 | 0 | 0 / 0 / 0 | 142.5 / 137.7 / 160.7 | 166.1 |
| S3 | FT-A | 5161 | 0 | 0 | 0 / 0 / 0 | 38.0 / 41.8 / 39.6 | 83.9 |
| S3 | FT-B | 5161 | 0 | 0 | 0 / 0 / 0 | 38.0 / 41.8 / 39.6 | 83.9 |
| S3 | Smart-T1 | 5161 | 0 | 0 | 0 / 0 / 0 | 31.0 / 30.3 / 30.7 | 117.2 |
| S4 | FT-A | 1046 | 0 | 0 | 0 / 0 / 0 | 27.8 / 30.3 / 28.9 | 83.9 |
| S4 | FT-B | 1046 | 0 | 0 | 0 / 0 / 0 | 21.7 / 23.2 / 22.0 | 58.9 |
| S4 | Smart-T1 | 1046 | 0 | 0 | 0 / 0 / 0 | 10.1 / 9.2 / 10.5 | 45.4 |
| S5 | FT-A | 6840 | 0 | 0 | 0 / 0 / 0 | 42.0 / 46.5 / 44.3 | 83.9 |
| S5 | FT-B | 6840 | 0 | 0 | 0 / 0 / 0 | 42.0 / 46.5 / 44.3 | 83.9 |
| S5 | Smart-T1 | 6840 | 0 | 0 | 0 / 0 / 0 | 46.6 / 48.2 / 45.7 | 149.9 |
| S6 | FT-A | 6083 | 0 | 0 | 0 / 0 / 0 | 140.8 / 99.5 / 103.7 | 83.9 |
| S6 | FT-B | 6083 | 0 | 0 | 0 / 0 / 0 | 193.9 / 148.3 / 152.6 | 75.9 |
| S6 | Smart-T1 | 6083 | 0 | 0 | 0 / 0 / 0 | 32.2 / 36.3 / 31.7 | 135.9 |
| S6 | FT-TOD | 6083 | 0 | 0 | 0 / 0 / 0 | 29.5 / 30.1 / 29.0 | 100.9 |
| S7a | FT-A | 9190 | 0 | 0 | 0 / 0 / 0 | 136.1 / 139.4 / 117.0 | 83.9 |
| S7a | FT-B | 9190 | 0 | 0 | 0 / 0 / 0 | 61.8 / 70.6 / 63.8 | 110.9 |
| S7a | Smart-T1 | 9190 | 0 | 0 | 0 / 0 / 0 | 72.0 / 110.8 / 75.0 | 150.3 |
| S7b | FT-A | 11941 | 0 | 0 | 266 / 364 / 255 | 648.5 / 650.4 / 619.1 | 83.9 |
| S7b | FT-B | 11941 | 0 | 0 | 106 / 187 / 104 | 555.0 / 566.0 / 527.5 | 109.9 |
| S7b | Smart-T1 | 11941 | 0 | 0 | 0 / 70 / 5 | 464.2 / 550.8 / 464.2 | 149.1 |
| S8 | FT-A | 7517 | 0 | 0 | 559 / 652 / 647 | 601.6 / 664.2 / 662.1 | 83.9 |
| S8 | FT-B | 7517 | 0 | 0 | 0 / 0 / 0 | 51.6 / 50.4 / 50.8 | 90.9 |
| S8 | Smart-T1 | 7517 | 0 | 0 | 0 / 0 / 0 | 54.0 / 53.0 / 56.7 | 141.7 |
| S9 | FT-A | 5434 | 0 | 0 | 99 / 140 / 99 | 230.6 / 285.4 / 253.2 | 83.9 |
| S9 | FT-B | 5434 | 0 | 0 | 0 / 0 / 0 | 55.7 / 57.6 / 56.0 | 106.9 |
| S9 | Smart-T1 | 5434 | 0 | 0 | 0 / 0 / 0 | 59.9 / 67.6 / 79.6 | 124.4 |
| S10 | FT-A | 8721 | 0 | 0 | 0 / 0 / 0 | 131.4 / 127.6 / 114.1 | 83.9 |
| S10 | FT-B | 8721 | 0 | 0 | 0 / 0 / 0 | 59.6 / 67.7 / 61.2 | 110.9 |
| S10 | Smart-T1 | 8721 | 0 | 0 | 0 / 0 / 0 | 131.3 / 107.1 / 96.2 | 161.1 |

Unfinished vehicles appear only where demand exceeds the capacity of that controller under the
standard policy (FT-A: 14 straight cars per 25 s green ≈ 542 veh/h and 4 left cars per 8 s green
≈ 155 veh/h per 93 s cycle): S2 FT-A (NS 700), S8 FT-A (lefts 330), S9 FT-A (SL 250, WL 230), and
S7b for every controller (X = 1.22, oversaturated by design; FT-B/S7b ≈ 556 / 176 veh/h). Every
under-saturated run clears all measured vehicles.

## 3. Custom / Demo, long and hardware runs

| Run | Vehicles | Crossed / arrivals | Unfinished | Mean delay s | Load check | Smart end reasons | Audit errors |
|---|---|---|---|---|---|---|---|
| CUSTOM zero demand/FT-A | 0 | 0 / 0 | 0 | – | ok, X = 0.0 | – | 0 |
| CUSTOM zero demand/Smart-T1 | 0 | 0 / 0 | 0 | – | ok, X = 0.0 | – | 0 |
| CUSTOM one movement only (NS 600)/FT-A | 640 | 589 / 589 | 0 | 200.7 | ok, X = 0.392 | – | 0 |
| CUSTOM one movement only (NS 600)/Smart-T1 | 640 | 589 / 589 | 0 | 0.0 | ok, X = 0.392 | – | 0 |
| CUSTOM all movements (500/150)/FT-A | 2809 | 2575 / 2575 | 0 | 65.9 | near, X = 0.859 | – | 0 |
| CUSTOM all movements (500/150)/Smart-T1 | 2809 | 2575 / 2575 | 0 | 71.1 | near, X = 0.859 | gap 68, hold 132, max 30 | 0 |
| CUSTOM high valid (NS 1800 only)/FT-A | 1945 | 580 / 1797 | 1217 | 1995.5 | over, X = 1.175 | – | 0 |
| CUSTOM high valid (NS 1800 only)/Smart-T1 | 1945 | 1797 / 1797 | 0 | 0.9 | over, X = 1.175 | – | 0 |
| CUSTOM near capacity (560/165)/FT-A | 3140 | 2881 / 2881 | 0 | 165.8 | near, X = 0.958 | – | 0 |
| CUSTOM near capacity (560/165)/Smart-T1 | 3140 | 2881 / 2881 | 0 | 161.9 | near, X = 0.958 | gap 23, hold 161, max 48 | 0 |
| CUSTOM oversaturated (750/240)/FT-A | 4251 | 3356 / 3914 | 558 | 787.2 | over, X = 1.309 | – | 0 |
| CUSTOM oversaturated (750/240)/Smart-T1 | 4251 | 3725 / 3914 | 189 | 602.5 | over, X = 1.309 | gap 5, hold 191, max 68 | 0 |
| LONG zero demand 7200 s / Smart-T1 | 0 | 0 / 0 | 0 | – | – | – | 0 |
| LONG one movement only (WL 400) 7200 s / Smart-T1 | 834 | 804 / 804 | 0 | 0.0 | – | – | 0 |
| LONG virtual-only S1 7200 s / Smart-T1 | 4298 | 4113 / 4113 | 0 | 46.7 | – | gap 311, hold 165, max 27 | 0 |
| HEAVY S7b/Smart-T1/seed4 | 4017 | 3698 / 3705 | 7 | 492.0 | – | gap 8, hold 174, max 74 | 0 |
| HEAVY S7b/Smart-T1/seed5 | 3938 | 3656 / 3656 | 0 | 443.8 | – | gap 12, hold 184, max 64 | 0 |
| HEAVY S8/Smart-T1/seed4 | 2521 | 2311 / 2311 | 0 | 58.9 | – | gap 117, hold 116, max 30 | 0 |
| HEAVY S8/Smart-T1/seed5 | 2443 | 2269 / 2269 | 0 | 53.9 | – | gap 120, hold 111, max 24 | 0 |
| HEAVY S10/Smart-T1/seed4 | 2898 | 2670 / 2670 | 0 | 124.8 | – | gap 47, hold 138, max 48 | 0 |
| HEAVY S10/Smart-T1/seed5 | 2879 | 2691 / 2691 | 0 | 125.1 | – | gap 54, hold 146, max 39 | 0 |
| HARDWARE chain S1 1 h / Smart-T1 (disconnect 1200-1320 s) | 1695 | 1695 / 1695 | 0 | 44.1 | – | gap 178, hold 76, max 10 | 0 |

The Custom load check is a cycle-based heuristic: "NS 1800 only" is flagged X = 1.18, but a
single movement at 1800 veh/h (below its 1895 veh/h saturation flow) is fully served by Smart-T1,
which rests in green; FT-A, with 25 s of green per 93 s, cannot serve it.

Other stress results

* **Determinism**: S1 Smart-T1, S7b Smart-T1, S10 FT-A, S6 FT-TOD (seed 2) run twice give
  byte-identical RunResult JSON.
* **Max speed**: the visualization session at Max speed for S7b Smart-T1 (full length) gives the
  identical RunResult as `runner.run`, without errors.
* **Memory**: tracemalloc peak for S7a Smart-T1 is 7.4 MB for 1 h and 14.3 MB for 2 h (ratio 1.93,
  about 2.6 kB per vehicle): memory grows linearly with the number of vehicles, with no leak symptoms.
* **Zero demand for 2 h**: 75 001 steps, no signal change, no vehicle, no error.
* **One movement only (WL 400) for 2 h**: Smart-T1 rests in green (2 transitions in total), all
  804 measured vehicles served, mean delay 0.02 s.

## 4. Sensor / Smart-T1 integration (exact to the millisecond, `test_stage16_sensor_smart.py`)

* A D1 pulse places a call (LEVEL_1, Q-hat 2); Smart-T1 starts the movement at once when all red.
* D2 continuously occupied for 1.9 s / 1.999 s → LEVEL_1; 2.0 s / 2.6 s → LEVEL_2 (Q-hat 6),
  latched exactly 2000 ms after the rising edge; an interruption restarts the count.
* 2.5 s with both detectors clear in green → gap-out; an occupancy resets the timer; with a
  conflicting call the green ends at max(minimum, gap-out), without one it rests in green.
* LEVEL_2 at GREEN onset → 12 s extended minimum (green lasts exactly 12 s even though it gapped out).
* A faulted detector → fail-safe recall (served without any edge), gap-out ignored, ended by
  max-out 45 s after the conflicting call.
* W_MAX: in S10 holds occur and calls wait longer than 90 s (up to about 161 s in the full-length
  runs) while no interval is shortened: it is a selection threshold, not a hard bound.

## 5. Hardware end to end

**Automated** (`test_stage16_hardware_chain.py` and the 1 h stress run) on the executable firmware
model `esp_sim.SimulatedEsp32` (same pins, active-low decoding, 20 ms debounce, lamp tables,
3 s fail-safe), through the real host stack:
IR inputs → ESP32 logic → protocol v2 → EspLink → HardwareDetectorSource → Hybrid (2 real + 14
virtual) → SensorBank → Smart-T1 → SignalBank → simulator → SignalMirror → R/Y/G outputs.

| Check | Result |
|---|---|
| Connected start-up; D1 and D2 occupied / clear reach the SensorBank exactly | pass |
| Physical lamps equal the logical NS lamps at every connected step (1 h: 35 461 steps) | 0 mismatches |
| All six states shown; RED_YELLOW = red + yellow; FLASHING_GREEN follows `lamps_for` | pass |
| NS has no virtual vehicles; the other 7 movements stay virtual | 0 NS vehicles; 1 695 others served |
| Real demand drives Smart-T1: each NS green follows a real IR call (35 NS greens from 230 real edges in 1 h) | pass |
| Disconnect mid-run: no crash; NS:D1/D2 faulted and held; Smart recall; board blinks yellow only (1 Hz, 500 ms on) | pass |
| Reconnect: HELLO + full-state report re-syncs, lamps mirrored again | pass |
| Missing hardware at start-up: runs, NS served by recall, no commands sent | pass |
| Old firmware (protocol v1): PROTOCOL_MISMATCH, nothing sent | pass |

Observed and documented (not changed): the controller's very first decision (t = 0) happens
before the first serial read, so the two real detectors are still "not synced" (faulted) at that
instant and Smart-T1 gives NS one fail-safe recall service at start-up (its minimum green plus
clearance) even without a car. After the first HELLO (within 100 ms) NS is served only by real IR demand. This is the
existing, safe-side fail-safe rule; changing it would change the hardware fault semantics.

**Manual (done by the user on the physical board, 2026-09-28)** — not repeated or simulated here:

* D1 real sensor works
* D2 real sensor works
* physical traffic light works (light sequence)
* simulator + hardware work end to end; the simulator mirrors the physical light correctly
* NS has no virtual cars in hardware mode
* real sensor demand affects Smart-T1
* disconnect / reconnect worked without crashing

## 6. UI / presentation

* Rendered at 1920×1080, 1366×768 and 1100×680 (the smallest window the app allows) for Smart-T1,
  FT-A, FT-TOD, hardware connected / disconnected, the Custom panel and a Custom run: every text
  is on screen, sidebar texts stay in the sidebar, all 8 controller-table rows are drawn.
* Labels: "Smart actuated · Smart-T1"; Custom panel "Smart-T1 (locked)" and "Traffic rules (all
  runs): Stop on Yellow · first queued car starts at green"; hardware card "2 real + 14 virtual
  detectors", "NS: real demand only · no virtual cars", "Real IR NS:D1", "Real IR NS:D2",
  "R/Y/G head mirrors NS"; both IR sensors drawn in their own slots; terms match README.
* Controls: start, pause / resume, reset, all scenarios then Custom and back, every controller
  (FT-TOD only on S6; Smart always Smart-T1), seed ±, all six speeds, hardware on (speed locked
  to 1×) and off — no errors; Max speed equals a plain run; screenshot mode at the three sizes.

The clipping check is exact for the Custom / Demo panel too: that panel is a modal overlay
centred on the window (at 1100×680 it covers part of the sidebar), so its texts are checked
against the overlay's own rectangle, and every button label must lie inside its button. (The
first version applied the sidebar rule to overlay texts; on the Windows PC, whose fonts are a
few pixels narrower, the correctly placed "Zero" preset label crossed that rule's threshold.
The test was corrected, not relaxed; no layout change was needed.)

### Bug found and fixed

**Controller table cut off at 1100×680.** At the smallest allowed window the Smart panel (with a
two-line starvation-hold explanation) and the Fixed-time panel showed only 6 of the 8 signal
rows (WS and WL were missing), although the code intends all 8 rows to fit. Fix in
`smarttraffic/viz_render.py`: secondary explanation lines (Smart reasons 2–3, the fixed-time
"comparison only" note) are left out only when they would push rows off the panel, and the row
height may shrink to 16 × k instead of 20 × k. 1366×768 and 1920×1080 frames are pixel-identical
to before. Regression test: `test_stage16_ui.py::test_no_clipped_text_and_all_eight_rows`.

## 7. Freeze check

* Full suite: **1450 passed** (1338 + 112 Stage 16 tests; with the 40 official S6 run records
  present, as on the presentation PC; without them 1449 passed, 1 skipped).
* Smart controller, sensors, signals, vehicles, runner, hardware, firmware, config (lock) and all
  Stage 14 / Stage 15 result files are unchanged.
* Recommendation: the system is ready to freeze for the 4.10 presentation after the Stage 16
  commit.
