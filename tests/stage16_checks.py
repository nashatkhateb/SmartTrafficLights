"""Stage 16 -- independent safety / consistency audit of a complete simulation run.

Nothing here changes a run. `Audit` is a read-only runner observer that checks every step,
and `Audit.finish(sim)` re-derives every signal interval from SignalBank.log and every
stop-line crossing from the vehicles, then checks the study's safety rules against those
records WITHOUT using the code that produced them:

  signals   every transition is RED -> RED_YELLOW -> GREEN -> FLASHING_GREEN -> YELLOW ->
            ALL_RED -> RED; the timed states last exactly RED_YELLOW 1.0 s, FLASHING_GREEN
            3.0 s, YELLOW 3.5 / 3.0 s, ALL_RED 2.0 / 3.0 s (straight / left); no two
            conflicting movements are ever in a conflict-blocking state at the same time
            (interval overlap, not only at step boundaries); Smart greens last at least the
            minimum green (12 s when LEVEL_2 was latched at the GREEN onset).
  vehicles  every crossing happens while its own head is GREEN or FLASHING_GREEN (never RED,
            RED_YELLOW, ALL_RED, never at or after YELLOW onset); each vehicle crosses at most
            once, lanes cross in FIFO order, successive crossings in a lane are at least
            1.9 s (straight) / 2.0 s (left) apart, and a car queued at slot 1 when GREEN
            begins crosses exactly 36 ms after the onset (it starts moving at the onset).
  sensors   (virtual detectors) each detector is occupied exactly when some vehicle body
            covers it; Q-hat is 0 / 2 / 6 for no call / LEVEL_1 / LEVEL_2; LEVEL_2 only after
            >= 2.0 s of continuous D2 occupancy while waiting; gap timer only in service.
  smart     every end_green of a Smart run had a valid reason at that instant: gap-out
            (both detectors clear >= 2.5 s, not a faulted movement), max-out, or a
            starvation hold (a conflicting call waiting >= W_MAX); never before the minimum.
  numbers   all statistics finite; no time before the run start; logs grow only with events.
"""
import bisect
import json
import math
from collections import defaultdict
from typing import Dict, List, Optional

from smarttraffic import conflicts
from smarttraffic.conflicts import MOVES
from smarttraffic.sensors import DETECTOR_POSITION, D2_FILTER_MS, PASSAGE_TIME_MS, QHAT, DemandLevel
from smarttraffic.signals import (ALL_RED, ALL_RED_MS, CONFLICT_BLOCKING_STATES, FLASHING_GREEN,
                                  FLASHING_GREEN_MS, GREEN, RED, RED_YELLOW, RED_YELLOW_MS, YELLOW,
                                  YELLOW_MS, movement_type)
from smarttraffic.vehicles import FREE_SPEED, SLOT1_POSITION, VEHICLE_LENGTH, VehicleState

NEXT = {RED: RED_YELLOW, RED_YELLOW: GREEN, GREEN: FLASHING_GREEN, FLASHING_GREEN: YELLOW,
        YELLOW: ALL_RED, ALL_RED: RED}
HEADWAY_MS = {"straight": 1900, "left": 2000}
FIRST_CAR_MS = math.ceil(SLOT1_POSITION / (FREE_SPEED / 1000) - 1e-6)     # 36 ms from slot 1


class AuditError(AssertionError):
    pass


def fixed_duration_ms(m: str, s) -> Optional[int]:
    t = movement_type(m)
    return {RED_YELLOW: RED_YELLOW_MS, FLASHING_GREEN: FLASHING_GREEN_MS,
            YELLOW: YELLOW_MS[t], ALL_RED: ALL_RED_MS[t]}.get(s)


class Audit:
    """Read-only runner observer. virtual=True adds the vehicle<->detector cross-check
    (only valid when every detector is virtual)."""

    def __init__(self, virtual: bool = True, check_every: int = 1):
        self.virtual = virtual
        self.every = check_every
        self.steps = 0
        self.errors: List[str] = []
        self.max_queue = defaultdict(int)          # per movement (stopped on approach)
        self.max_outside = defaultdict(int)
        self.max_road = 0
        self.ext_at_onset: Dict[tuple, bool] = {}  # (m, onset ms) -> LEVEL_2 at GREEN onset
        self.end_checks = 0
        self.reasons = defaultdict(int)
        self.max_wait_ms = 0
        self.hold_seen = False
        self._cmd_idx = 0
        self._last_now = None
        self.level2_latches = 0
        self.gapouts_seen = 0

    def _err(self, msg):
        if len(self.errors) < 50:
            self.errors.append(msg)

    # ------------------------------------------------------------ per step
    def __call__(self, view) -> None:
        now, vm, sb, bank, ctrl = view.now_ms, view.vehicles, view.sensors, view.signals, view.controller
        if self._last_now is not None and now <= self._last_now:
            self._err(f"time did not advance: {self._last_now} -> {now}")
        self._last_now = now
        self.steps += 1
        try:
            bank.check_invariant()
            vm.check_invariants()
        except AssertionError as e:
            self._err(f"{now}: {e}")
        active = bank.active()
        if not conflicts.is_allowed_set(active):
            self._err(f"{now}: conflicting blocking set {active}")
        for m in MOVES:
            q = vm.queue_length(m)
            self.max_queue[m] = max(self.max_queue[m], q)
            self.max_outside[m] = max(self.max_outside[m], len(vm.waiting_outside(m)))
            self.max_road = max(self.max_road, len(vm.vehicles(m)))
            st = sb.status(m)
            if st.qhat != QHAT[st.demand_level]:
                self._err(f"{now} {m}: qhat {st.qhat} for {st.demand_level.name}")
            if st.has_call != (st.demand_level != DemandLevel.NONE):
                self._err(f"{now} {m}: call {st.has_call} but level {st.demand_level.name}")
            if st.level2_latched and (st.level2_since_ms is None):
                self._err(f"{now} {m}: LEVEL_2 without a latch time")
            if st.in_service != (bank.state(m) in (GREEN, FLASHING_GREEN)):
                self._err(f"{now} {m}: sensor mode {st.in_service} vs signal {bank.state(m).name}")
            if st.in_service and st.has_call:
                self._err(f"{now} {m}: call placed during service")
            if (st.gap_timer_ms is None) == st.in_service:
                self._err(f"{now} {m}: gap timer {st.gap_timer_ms} in_service {st.in_service}")
            if st.gap_out and not st.in_service:
                self._err(f"{now} {m}: gap-out outside service")
            if st.gap_out:
                self.gapouts_seen += 1
            if bank.state(m) is GREEN:
                key = (m, round(bank.since(m) * 1000))
                if key not in self.ext_at_onset:           # first step of this service
                    self.ext_at_onset[key] = sb.extended_initial_required(m)
            if st.has_call and bank.state(m) is RED:
                self.max_wait_ms = max(self.max_wait_ms, now - st.call_since_ms)
            if self.virtual and self.steps % self.every == 0:
                for d, p in DETECTOR_POSITION.items():
                    xs = [v.position for v in vm.vehicles(m) + vm.in_intersection(m) if v.position is not None]
                    if any(abs(x - p) < 1e-6 or abs(x - (p - VEHICLE_LENGTH)) < 1e-6 for x in xs):
                        continue                    # exactly on an edge instant: float rounding
                    cover = any(p - VEHICLE_LENGTH < x <= p for x in xs)
                    if cover != sb.occupied(m, d):
                        self._err(f"{now} {m}:{d} sensor {sb.occupied(m, d)} but vehicle cover {cover}")
        log = getattr(ctrl, "command_log", None)
        if log is not None and hasattr(ctrl, "decision_log"):
            for c in log[self._cmd_idx:]:
                if c.kind == "end_green":
                    self._check_end(ctrl, sb, bank, c)
            self._cmd_idx = len(log)
        for m in MOVES:
            if sb.level2_latched(m) and sb.status(m).level2_since_ms == now:
                self.level2_latches += 1

    def _check_end(self, ctrl, sb, bank, c) -> None:
        self.end_checks += 1
        m, now = c.movement, c.time_ms
        dec = [d for d in ctrl.decision_log if d.time_ms == now]
        reason = None
        for d in dec:
            for g, r in d.stopped:
                if g == m:
                    reason = r
        if reason is None:
            self._err(f"{now} {m}: end_green without a recorded reason")
            return
        self.reasons[reason] += 1
        onset = ctrl.green_onset_ms(m)
        mn = ctrl.min_green_of(m)
        p = ctrl.params
        if onset is None or mn is None:
            self._err(f"{now} {m}: end_green without onset/min")
            return
        ext = self.ext_at_onset.get((m, onset))
        want_min = p.min_green_ms(movement_type(m), bool(ext))
        if mn != want_min:
            self._err(f"{now} {m}: min green {mn} but LEVEL_2-at-onset={ext} requires {want_min}")
        if now + FLASHING_GREEN_MS < onset + want_min:
            self._err(f"{now} {m}: ended before its minimum green")
        if reason == "gap":
            if m in ctrl.faulted_movements() or sb.gap_timer_ms(m) < PASSAGE_TIME_MS:
                self._err(f"{now} {m}: gap-out reason but gap timer {sb.gap_timer_ms(m)}")
        elif reason == "max":
            ms = ctrl.max_start_ms(m)
            if ms is None or now < ms + p.max_green_ms(movement_type(m)) - FLASHING_GREEN_MS:
                self._err(f"{now} {m}: max-out reason too early (max_start {ms})")
        elif reason == "hold":
            self.hold_seen = True
            hold = [d.hold for d in dec if d.hold is not None]
            if not hold or conflicts.compatible(m, hold[0]):
                self._err(f"{now} {m}: hold reason without a conflicting hold")
            else:
                h = hold[0]
                call = sb.call_since_ms(h) if sb.has_call(h) else None
                if call is not None and now - call < p.w_max_ms:
                    self._err(f"{now} {m}: hold {h} waited only {now - call} ms")
        else:
            self._err(f"{now} {m}: unknown reason {reason}")

    # ------------------------------------------------------------ whole run
    def finish(self, sim, result=None) -> dict:
        bank, vm = sim.signals, sim.vehicles
        start = sim.config.window().start_ms
        end = sim.end_ms
        # 1. rebuild signal intervals from the transition log
        trans = defaultdict(list)                       # m -> [(t_ms, new_state)]
        cur = {m: (RED, start) for m in MOVES}
        intervals = defaultdict(list)                   # m -> [(t0, t1, state)]
        onsets = defaultdict(list)                      # m -> [GREEN onset ms]
        greens = []                                     # (m, onset, end of FG)
        for ev in bank.log:
            t = round(ev.time * 1000)
            m = ev.movement
            s0, t0 = cur[m]
            if ev.old is not s0:
                self._err(f"{t} {m}: log says {ev.old.name} but audit has {s0.name}")
            if NEXT[s0] is not ev.new:
                self._err(f"{t} {m}: illegal transition {s0.name} -> {ev.new.name}")
            fd = fixed_duration_ms(m, s0)
            if fd is not None and t - t0 != fd:
                self._err(f"{t} {m}: {s0.name} lasted {t - t0} ms, must be {fd}")
            if t < t0:
                self._err(f"{t} {m}: transition before previous one")
            intervals[m].append((t0, t, s0))
            trans[m].append((t, ev.new))
            if ev.new is GREEN:
                onsets[m].append(t)
            cur[m] = (ev.new, t)
        for m in MOVES:
            s, t0 = cur[m]
            intervals[m].append((t0, end, s))
            if s is not bank.state(m):
                self._err(f"{m}: final state {s.name} vs bank {bank.state(m).name}")
        # greens: GREEN+FG durations
        for m in MOVES:
            iv = intervals[m]
            for i, (a, b, s) in enumerate(iv):
                if s is GREEN and i + 1 < len(iv) and iv[i + 1][2] is FLASHING_GREEN:
                    greens.append((m, a, iv[i + 1][1]))
        # 2. conflicts: interval overlap of blocking states
        block = {m: [(a, b) for a, b, s in intervals[m] if s in CONFLICT_BLOCKING_STATES and b > a]
                 for m in MOVES}
        conflicts_found = 0
        for i, a in enumerate(MOVES):
            for b in MOVES[i + 1:]:
                if conflicts.compatible(a, b):
                    continue
                for x0, x1 in block[a]:
                    for y0, y1 in block[b]:
                        if x0 < y1 and y0 < x1:
                            conflicts_found += 1
                            self._err(f"conflict {a} [{x0},{x1}) with {b} [{y0},{y1})")
        # 3. crossings
        def state_at(m, t):
            tr = trans[m]
            i = bisect.bisect_right([x[0] for x in tr], t) - 1
            return RED if i < 0 else tr[i][1]
        crossings = 0
        ids = set()
        first_car_checked = 0
        for m in MOVES:
            h = HEADWAY_MS[movement_type(m)]
            crossed = vm.crossed(m)
            prev = None
            times = []
            for v in crossed:
                if v.id in ids:
                    self._err(f"vehicle {v.id} crossed twice")
                ids.add(v.id)
                crossings += 1
                t = v.cross_ms
                times.append(t)
                s = state_at(m, t)
                if s not in (GREEN, FLASHING_GREEN):
                    self._err(f"{m} vehicle {v.id} crossed at {t} in {s.name}")
                if t < start or t < v.scheduled_ms:
                    self._err(f"{m} vehicle {v.id} crossed at {t} before start/arrival")
                if v.cross_ms < v.free_flow_cross_ms:
                    self._err(f"{m} vehicle {v.id} negative control delay")
                if prev is not None:
                    if (v.scheduled_ms, v.seq) < (prev.scheduled_ms, prev.seq):
                        self._err(f"{m}: FIFO broken {prev.id} -> {v.id}")
                    if t - prev.cross_ms < h:
                        self._err(f"{m}: headway {t - prev.cross_ms} ms < {h} between {prev.id} and {v.id}")
                prev = v
            # first queued car at each GREEN onset
            pending = [v for v in vm.all_vehicles() if v.movement == m and v.entry_ms is not None]
            for tg in onsets[m]:
                queued = [v for v in pending
                          if v.entry_ms + 10800 - FIRST_CAR_MS <= tg and (v.cross_ms is None or v.cross_ms > tg)]
                if not queued:
                    continue
                front = min(queued, key=lambda v: (v.scheduled_ms, v.seq))
                if front.cross_ms is None:
                    if tg + FIRST_CAR_MS <= end and state_at(m, tg + FIRST_CAR_MS) in (GREEN,):
                        self._err(f"{m}: queued car {front.id} did not cross after onset {tg}")
                    continue
                first_car_checked += 1
                if front.cross_ms != tg + FIRST_CAR_MS:
                    self._err(f"{m}: first queued car {front.id} crossed at {front.cross_ms}, onset {tg}")
        for v in vm.all_vehicles():
            for x in (v.entry_ms, v.cross_ms, v.exit_ms):
                if x is not None and x < start:
                    self._err(f"vehicle {v.id}: time {x} before run start {start}")
            if v.cross_ms is None and v.state in (VehicleState.IN_INTERSECTION, VehicleState.EXITED):
                self._err(f"vehicle {v.id}: in box without crossing")
        # 4. Smart minimum green from the intervals
        ctrl = sim.controller
        if hasattr(ctrl, "params") and hasattr(ctrl, "decision_log"):
            for m, onset, fg_end in greens:
                ext = self.ext_at_onset.get((m, onset), False)
                mn = ctrl.params.min_green_ms(movement_type(m), ext)
                if fg_end - onset < mn:
                    self._err(f"{m}: green from {onset} lasted {fg_end - onset} ms < min {mn}")
        # 5. numbers
        if result is not None:
            txt = result.to_json()
            if "NaN" in txt or "Infinity" in txt:
                self._err("non-finite number in the result")
            d = json.loads(txt)
            def walk(o, path=""):
                if isinstance(o, float) and not math.isfinite(o):
                    self._err(f"non-finite {path}")
                elif isinstance(o, dict):
                    for k, x in o.items():
                        walk(x, path + "/" + k)
                elif isinstance(o, list):
                    for i, x in enumerate(o):
                        walk(x, f"{path}[{i}]")
            walk(d)
        n_veh = len(vm.all_vehicles())
        edges = len(sim.sensors.edge_log)
        if edges > 4 * crossings + 4 * n_veh + 64:
            self._err(f"detector log {edges} edges for {n_veh} vehicles")
        if len(bank.log) > 6 * sum(len(v) for v in onsets.values()) + 6 * len(MOVES):
            self._err(f"signal log {len(bank.log)} for {sum(len(v) for v in onsets.values())} greens")
        return {
            "steps": self.steps, "vehicles": n_veh, "crossings": crossings,
            "green_onsets": sum(len(v) for v in onsets.values()), "transitions": len(bank.log),
            "detector_edges": edges, "conflicts": conflicts_found,
            "first_car_checks": first_car_checked, "smart_end_checks": self.end_checks,
            "end_reasons": dict(self.reasons), "max_queue": max(self.max_queue.values(), default=0),
            "max_outside": max(self.max_outside.values(), default=0), "max_road": self.max_road,
            "max_wait_s": self.max_wait_ms / 1000, "hold_seen": self.hold_seen,
            "level2_latches": self.level2_latches, "errors": list(self.errors),
        }


def audited_run(config, *, arrivals=None, source_factory=None, observers=(), virtual=True,
                exclude_movements=(), check_every=1):
    """runner.run with an Audit observer. Returns (RunResult, report)."""
    from smarttraffic.runner import run as _run, simulate
    a = Audit(virtual=virtual, check_every=check_every)
    box = {}
    kw = {}
    if arrivals is not None:
        kw["arrivals"] = arrivals
    if source_factory is not None:
        kw["source_factory"] = source_factory
    if exclude_movements:
        kw["exclude_movements"] = tuple(exclude_movements)

    def grab(view):
        box["view"] = view
    res = _run(config, observers=[a, grab, *observers], **kw)
    # rebuild the Simulation handle from the last view (run() does not return it)
    sim = _SimView(config, box["view"], res)
    rep = a.finish(sim, res)
    return res, rep


class _SimView:
    def __init__(self, config, view, res):
        self.config = config
        self.vehicles, self.sensors, self.signals, self.controller = (view.vehicles, view.sensors,
                                                                      view.signals, view.controller)
        self.end_ms = view.now_ms


# ====================================================================== hardware chain
class HardwareChain:
    """The full hardware chain on the executable firmware model, driven by runner.simulate:

        SimulatedEsp32 inputs (GPIO27 / GPIO26, active-low) -> protocol v2 -> EspLink ->
        HardwareDetectorSource -> HybridDetectorSource (2 real + 14 virtual) -> SensorBank ->
        Smart-T1 -> SignalBank -> SignalMirror -> SIG -> SimulatedEsp32 R/Y/G outputs.

    `cars` is a list of (t_ms, channel, present) input changes; `script` maps t_ms -> callable(esp)
    for unplug / plug. Every step records (now, logical NS state, logical lamps, physical lamps,
    link state, NS faulted)."""

    def __init__(self, cars=(), script=None, esp=None):
        from smarttraffic import esp_sim
        from smarttraffic.hardware import load_hardware_config
        self.esp = esp or esp_sim.SimulatedEsp32()
        self.cfg = load_hardware_config()
        self.cars = sorted(cars)
        self.script = dict(script or {})
        self.trace = []
        self.hw = None

    def factory(self, vm):
        from smarttraffic.hardware import build_hardware
        self.hw = build_hardware(self.cfg, vm, lambda d: self.esp)
        return self.hw.detectors

    def observe(self, view):
        now = view.now_ms
        # the board runs in device time = simulation time; inputs change at their exact ms
        while self.cars and self.cars[0][0] <= now:
            t, ch, present = self.cars.pop(0)
            self.esp.advance_to(max(self.esp.ms, t))
            self.esp.car(ch, present)
        self.esp.advance_to(max(self.esp.ms, now))
        for t in sorted(k for k in self.script if k <= now):
            self.script.pop(t)(self.esp)
        self.hw.mirror.update(view.signals)
        self.ns_vehicles_seen = max(getattr(self, "ns_vehicles_seen", 0),
                                    len(view.vehicles.vehicles("NS")) + len(view.vehicles.crossed("NS"))
                                    + len(view.vehicles.waiting_outside("NS")))
        link = self.hw.hub.link("esp32_a").state.name
        self._sb = view.sensors
        self.ctrl = view.controller
        self.trace.append((now, view.signals.state("NS"), tuple(view.signals.lamp_state("NS")),
                           self.esp.lamps(0), link, ("NS", "D1") in view.sensors.faulted_detectors(),
                           self.esp.in_fault()))

    def summary(self):
        conn = [r for r in self.trace if r[4] == "CONNECTED" and not r[6]]
        mism = [r for r in conn if r[2] != r[3]]
        fault_rows = [r for r in self.trace if r[6]]
        blink_ok = all(not r[3][0] and not r[3][2] for r in fault_rows)
        blink_seen = {r[3][1] for r in fault_rows}
        return {"steps": len(self.trace), "connected_steps": len(conn), "lamp_mismatches": len(mism),
                "first_mismatch": mism[0][:4] if mism else None,
                "fault_steps": len(fault_rows), "fault_only_yellow": blink_ok,
                "fault_yellow_blinks": blink_seen == {True, False} if fault_rows else None,
                "states_seen": sorted({r[1].name for r in conn}),
                "esp_errors": list(self.esp.errors), "commands": len(self.esp.commands),
                "ns_virtual_vehicles_seen": getattr(self, "ns_vehicles_seen", 0),
                "ns_real_edges": len([e for e in self._sb.edge_log if e.movement == "NS"]) if hasattr(self, "_sb") else None,
                "ns_green_onsets": len([1 for a, b in zip(self.trace, self.trace[1:])
                                        if a[1].name != "GREEN" and b[1].name == "GREEN"]),
                "ns_faulted_steps": sum(1 for r in self.trace if r[5])}


def ns_car_script(start_ms, end_ms, period_ms=37_000, level2_every=4):
    """Deterministic IR events: a car passes D1 (0.4 s) every period; every `level2_every`-th
    time a queue also covers D2 for 3.0 s (-> LEVEL_2) before the D1 pulse."""
    ev, k, t = [], 0, start_ms
    while t < end_ms:
        if k % level2_every == 0:
            ev += [(t, 1, True), (t + 3000, 1, False)]
            t += 3500
        ev += [(t, 0, True), (t + 400, 0, False)]
        k += 1
        t += period_ms
    return ev
