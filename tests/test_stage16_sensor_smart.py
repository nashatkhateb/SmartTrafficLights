"""Stage 16 -- sensor <-> Smart-T1 integration with scripted detector edges.

The real SensorBank, SignalBank and SmartController (locked Smart-T1 parameters) run in the
runner's step order (sensors -> SignalBank.advance -> controller.update, 100 ms). The detector
source is a script of exact edges, so every rule can be checked at its exact millisecond:
D1 pulse -> call; D2 >= 2.0 s continuous while waiting -> LEVEL_2; Q-hat 2 / 6; gap-out after
2.5 s clear in green; LEVEL_2 at GREEN onset -> 12 s extended minimum; rest in green without a
conflicting call; fail-safe recall for a faulted detector; W_MAX is a selection threshold only.
"""
import pytest

from smarttraffic.controller_smart import END_GREEN, START_GREEN, SmartController
from smarttraffic.sensors import SENSOR_IDS, DemandLevel, DetectorEdge, SensorBank
from smarttraffic.signals import FLASHING_GREEN_MS, GREEN, RED, RED_YELLOW, SignalBank
from smarttraffic.viz import smart_t1_params

T1 = smart_t1_params()


class Scripted:
    """DetectorSource with exact scripted edges and optional fault windows."""
    sensor_ids = SENSOR_IDS

    def __init__(self, edges=(), faults=()):
        self._now = 0
        self.edges = sorted(edges)
        self.windows = list(faults)             # (t0, t1, (m, d))

    @property
    def now_ms(self):
        return self._now

    def initial_occupancy(self):
        return {sid: False for sid in SENSOR_IDS}

    def poll(self, until_ms):
        out = [DetectorEdge(t, m, d, occ) for t, m, d, occ in self.edges if self._now < t <= until_ms]
        self._now = until_ms
        return out

    def faults(self):
        return tuple(sid for t0, t1, sid in self.windows if t0 <= self._now < t1)


def pulse(t, m, d, dur):
    return [(t, m, d, True), (t + dur, m, d, False)]


class Loop:
    def __init__(self, edges=(), faults=(), params=T1):
        self.src = Scripted(edges, faults)
        self.sb = SensorBank(self.src)
        self.bank = SignalBank(0)
        self.ctrl = SmartController(self.sb, params)
        self.ctrl.update(self.bank)
        self.t = 0
        self.rows = {}

    def run_to(self, ms):
        while self.t < ms:
            self.sb.advance(0.1, self.bank)
            self.bank.advance(0.1)
            self.ctrl.update(self.bank)
            self.t += 100
            self.rows[self.t] = {m: self.sb.status(m) for m in ("NS", "ES")}
        return self

    def cmds(self, kind=None, m=None):
        return [(c.time_ms, c.kind, c.movement) for c in self.ctrl.command_log
                if (kind is None or c.kind == kind) and (m is None or c.movement == m)]

    def reasons(self):
        return [(d.time_ms, d.stopped) for d in self.ctrl.decision_log]


def test_smart_t1_parameters_are_the_lock():
    assert (T1.tau_s, T1.w_max_s, T1.max_green_straight_s, T1.max_green_left_s) == (15.0, 90.0, 45.0, 30.0)
    assert (T1.min_green_straight_s, T1.min_green_left_s, T1.extended_initial_s) == (7.0, 5.0, 12.0)


def test_d1_pulse_creates_demand_and_smart_serves_it():
    lp = Loop(pulse(5000, "ES", "D1", 200)).run_to(4900)
    assert not lp.sb.has_call("ES") and lp.cmds() == []
    lp.run_to(5000)
    st = lp.rows[5000]["ES"]
    assert st.has_call and st.call_since_ms == 5000 and st.demand_level is DemandLevel.LEVEL_1 and st.qhat == 2
    assert lp.cmds() == [(5000, START_GREEN, "ES")]           # all red: served at once
    lp.run_to(7000)
    assert lp.bank.state("ES") is GREEN and lp.bank.since("ES") == 6.0   # 1.0 s RED_YELLOW
    assert not lp.sb.has_call("ES")                           # the call is cleared at GREEN onset


@pytest.mark.parametrize("held, level", [(1900, DemandLevel.LEVEL_1), (1999, DemandLevel.LEVEL_1),
                                         (2000, DemandLevel.LEVEL_2), (2600, DemandLevel.LEVEL_2)])
def test_d2_continuous_occupancy_2_0_s_creates_level2(held, level):
    # NS is being served (call at 1000) so ES stays WAITING while its D2 is held
    edges = pulse(1000, "NS", "D1", 300) + [(3000, "NS", "D1", True)] + pulse(4000, "ES", "D2", held)
    lp = Loop(edges).run_to(4000 + held + 500)
    st = lp.sb.status("ES")
    assert lp.bank.state("ES") is RED and st.has_call
    assert st.demand_level is level and st.qhat == {DemandLevel.LEVEL_1: 2, DemandLevel.LEVEL_2: 6}[level]
    if level is DemandLevel.LEVEL_2:
        assert st.level2_since_ms == 6000                   # exactly 2.0 s after the rising edge
        assert lp.rows[5900]["ES"].demand_level is DemandLevel.LEVEL_1


def test_d2_interrupted_restarts_the_2_0_s_count():
    edges = pulse(1000, "NS", "D1", 300) + [(3000, "NS", "D1", True)]
    edges += pulse(4000, "ES", "D2", 1500) + pulse(5600, "ES", "D2", 1900)   # 1.5 s + gap + 1.9 s
    lp = Loop(edges).run_to(8000)
    assert lp.sb.demand_level("ES") is DemandLevel.LEVEL_1 and lp.sb.qhat("ES") == 2


def test_gap_out_after_2_5_s_clear_ends_green_only_with_conflicting_demand():
    # ES called at 5000 -> RED_YELLOW 5000, GREEN 6000. No ES detections afterwards: gap timer
    # counts from GREEN onset; NS (conflicting) calls at 8000. Minimum green 7 s: end_green
    # allowed from 6000 + 7000 - 3000 = 10000 -> ended at 10000 by gap-out.
    lp = Loop(pulse(5000, "ES", "D1", 200) + pulse(8000, "NS", "D1", 200)).run_to(12000)
    assert lp.rows[8500]["ES"].gap_timer_ms == 2500 and lp.rows[8500]["ES"].gap_out
    assert lp.rows[8400]["ES"].gap_out is False
    assert lp.cmds(END_GREEN, "ES") == [(10000, END_GREEN, "ES")]
    assert lp.reasons()[0] == (10000, (("ES", "gap"),))


def test_occupancy_in_green_resets_the_gap_timer():
    edges = pulse(5000, "ES", "D1", 200) + pulse(8000, "NS", "D1", 200) + pulse(9500, "ES", "D1", 300)
    lp = Loop(edges).run_to(14000)
    # clear again from 9800 -> gap-out at 12300 (> minimum 10000)
    assert lp.cmds(END_GREEN, "ES") == [(12300, END_GREEN, "ES")]
    assert lp.rows[12200]["ES"].gap_timer_ms == 2400


def test_rest_in_green_after_gap_out_without_conflicting_call():
    lp = Loop(pulse(5000, "ES", "D1", 200)).run_to(60_000)
    assert lp.bank.state("ES") is GREEN and lp.sb.gap_out("ES")
    assert lp.cmds(END_GREEN) == []


def test_level2_at_green_onset_gives_the_12_s_extended_minimum():
    # NS served first (call 1000: GREEN 2000). ES D2 held 3000-6000 while waiting -> LEVEL_2 at 5000.
    # NS gaps out (clear since onset) and is releasable at 2000 + 7000 - 3000 = 6000 -> NS ends 6000,
    # NS RED at 6000 + 3000 + 3500 + 2000 = 14500 -> ES GREEN 15500 with LEVEL_2 -> minimum 12 s.
    edges = pulse(1000, "NS", "D1", 200) + pulse(3000, "ES", "D2", 3000) + pulse(20_000, "NS", "D1", 200)
    lp = Loop(edges).run_to(20_000)
    assert lp.cmds(END_GREEN, "NS")[0] == (6000, END_GREEN, "NS")
    assert lp.rows[15500]["ES"].extended_initial_required and lp.rows[15500]["ES"].green_onset_ms == 15500
    assert lp.ctrl.min_green_of("ES") == 12_000
    lp.run_to(30_000)
    # ES gapped out long ago, NS calls at 20000, but ES may end only at 15500 + 12000 - 3000 = 24500
    assert lp.cmds(END_GREEN, "ES") == [(24500, END_GREEN, "ES")]
    assert (24500 + FLASHING_GREEN_MS) - 15500 == 12_000


def test_level1_at_onset_keeps_the_normal_minimum():
    edges = pulse(1000, "NS", "D1", 200) + pulse(3000, "ES", "D1", 200) + pulse(20_000, "NS", "D1", 200)
    lp = Loop(edges).run_to(30_000)
    assert not lp.rows[15500]["ES"].extended_initial_required
    assert lp.cmds(END_GREEN, "ES") == [(20_000, END_GREEN, "ES")]   # min 7 s already met, gapped out


def test_fail_safe_recall_serves_a_faulted_movement_and_ignores_gap_out():
    # ES:D1 faulted from 3000 s with no edges at all: ES must still be served (recall, Q-hat 2),
    # and while faulted its green ends only by max-out (45 s) when NS has a call.
    faults = [(3000, 10**9, ("ES", "D1"))]
    lp = Loop(pulse(10_000, "NS", "D1", 200), faults=faults).run_to(80_000)
    assert lp.cmds(START_GREEN, "ES")[0] == (3000, START_GREEN, "ES")
    assert "ES" in lp.ctrl.faulted_movements()
    ends = [r for r in lp.reasons() if any(g == "ES" for g, _ in r[1])]
    assert ends and ends[0][1] == (("ES", "max"),)
    # ES GREEN at 4000 (gap-out ignored while faulted); NS calls at 10000 -> max-out counted
    # from 10000: end_green at 10000 + 45000 - 3000 = 52000
    assert ends[0][0] == 52_000


def test_w_max_is_a_selection_threshold_not_a_hard_bound():
    """S10 (starvation scenario) with Smart-T1: holds occur, some calls wait longer than
    W_MAX = 90 s, and no signal interval is ever shortened (audited)."""
    from tests.stage16_checks import audited_run
    from smarttraffic.runner import RunConfig
    res, rep = audited_run(RunConfig("S10", 1, "Smart", warmup_s=120.0, measure_s=900.0, flush_s=900.0,
                                     smart_params=T1), check_every=5)
    assert rep["errors"] == []
    assert rep["hold_seen"] and rep["end_reasons"].get("hold", 0) > 0
    assert rep["max_wait_s"] > 90.0
