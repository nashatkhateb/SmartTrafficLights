"""Stage 16 -- automated traffic-safety invariants over the software matrix.

Every run is audited by tests/stage16_checks.Audit (independent re-derivation of all signal
intervals and crossings; see that module). The full-length (300 + 3600 + 900 s) matrix and the
stress runs are in tests/stage16_stress.py; here the same checks run on shorter windows so the
suite stays fast: S1-S10 x FT-A / FT-B / Smart-T1 (+ FT-TOD on S6), Custom/Demo edge cases,
several seeds on the hard scenarios. The audit itself is proven to catch injected violations.
"""
import pytest

from smarttraffic.runner import RunConfig, run, simulate
from smarttraffic.signals import (ALL_RED, FLASHING_GREEN, GREEN, RED, RED_YELLOW, YELLOW,
                                  TransitionEvent)
from smarttraffic.viz import smart_t1_params
from tests.stage16_checks import FIRST_CAR_MS, HEADWAY_MS, Audit, audited_run

SCENARIOS = ("S1", "S2", "S3", "S4", "S5", "S6", "S7a", "S7b", "S8", "S9", "S10")
SHORT = dict(warmup_s=120.0, measure_s=300.0, flush_s=600.0)
T1 = smart_t1_params()
MATRIX = [(sc, c) for sc in SCENARIOS for c in ("FT-A", "FT-B", "Smart")] + [("S6", "FT-TOD")]


def cfg(sc, ctrl, seed=1, **kw):
    return RunConfig(sc, seed, ctrl, smart_params=T1, **{**SHORT, **kw})


def _ok(res, rep):
    assert rep["errors"] == [], rep["errors"][:5]
    assert rep["conflicts"] == 0
    d = res.to_json()
    assert '"safety_violations":0' in d


def test_first_car_and_headway_constants():
    assert FIRST_CAR_MS == 36 and HEADWAY_MS == {"straight": 1900, "left": 2000}


@pytest.mark.parametrize("sc, ctrl", MATRIX)
def test_matrix_run_is_safe(sc, ctrl):
    res, rep = audited_run(cfg(sc, ctrl), check_every=4)
    _ok(res, rep)
    assert rep["crossings"] > 0 and rep["green_onsets"] > 0
    if ctrl == "Smart":
        assert rep["smart_end_checks"] > 0 and sum(rep["end_reasons"].values()) == rep["smart_end_checks"]


@pytest.mark.parametrize("sc, seed", [(sc, s) for sc in ("S7b", "S8", "S10") for s in (2, 3)])
def test_hard_scenarios_more_seeds(sc, seed):
    res, rep = audited_run(cfg(sc, "Smart", seed), check_every=4)
    _ok(res, rep)


def test_first_queued_car_starts_at_green_onset_and_headways_hold():
    # heavy, oversaturated: many queued starts and saturated discharges
    res, rep = audited_run(cfg("S7b", "FT-A"), check_every=4)
    _ok(res, rep)
    assert rep["first_car_checks"] >= 20


CUSTOM = {
    "zero": {},
    "one": {"NS": 600},
    "all": {m: (500 if m[1] == "S" else 150) for m in ("NS", "SS", "ES", "WS", "NL", "SL", "EL", "WL")},
    "high": {"NS": 1800},
    "near": {m: (560 if m[1] == "S" else 165) for m in ("NS", "SS", "ES", "WS", "NL", "SL", "EL", "WL")},
    "over": {m: (750 if m[1] == "S" else 240) for m in ("NS", "SS", "ES", "WS", "NL", "SL", "EL", "WL")},
}


@pytest.mark.parametrize("case", sorted(CUSTOM))
@pytest.mark.parametrize("ctrl", ["FT-A", "Smart"])
def test_custom_demo_edge_cases_are_safe(case, ctrl):
    from smarttraffic.conflicts import MOVES
    from smarttraffic.custom import CustomDemand, CustomSpec
    spec = CustomSpec(CustomDemand.from_mapping({m: CUSTOM[case].get(m, 0.0) for m in MOVES}),
                      controller=ctrl, measure_s=300)
    res, rep = audited_run(spec.config(), check_every=4)
    _ok(res, rep)
    if case == "zero":
        assert rep["vehicles"] == 0 and rep["crossings"] == 0
    if case in ("one", "high"):
        assert {v["movement"] for v in __import__("json").loads(res.to_json())["vehicles"]} == {"NS"}
    if case == "high" and ctrl == "Smart":
        assert '"unfinished":0' in res.to_json()          # one movement at 1800 veh/h: rest in green


def test_virtual_only_runs_are_deterministic():
    for sc, c in (("S7b", "Smart"), ("S6", "FT-TOD"), ("S10", "FT-B")):
        assert run(cfg(sc, c, 2)).to_json() == run(cfg(sc, c, 2)).to_json()


# ------------------------------------------------------------------ the audit catches violations
def _sim():
    a = Audit()
    sim = simulate(cfg("S1", "FT-A", measure_s=180.0, flush_s=300.0), observers=[a], collect_stats=False)
    assert a.finish(sim)["errors"] == []
    return a, sim


def _errors_after(mutate):
    a, sim = _sim()
    a2 = Audit()
    a2.steps = a.steps
    mutate(sim)
    return a2.finish(sim)["errors"]


def test_audit_detects_an_illegal_transition():
    def m(sim):
        ev = next(e for e in sim.signals.log if e.new is YELLOW)
        i = sim.signals.log.index(ev)
        sim.signals.log[i] = TransitionEvent(ev.time, ev.movement, ev.old, ALL_RED)
    assert any("illegal transition" in e for e in _errors_after(m))


def test_audit_detects_a_shortened_interval():
    def m(sim):
        i, ev = next((i, e) for i, e in enumerate(sim.signals.log) if e.new is ALL_RED)
        sim.signals.log[i] = TransitionEvent(ev.time - 0.1, ev.movement, ev.old, ev.new)
    assert any("lasted" in e for e in _errors_after(m))


def test_audit_detects_conflicting_greens():
    def m(sim):
        t = sim.end_ms / 1000 - 30
        # NS and ES conflict: both enter RED_YELLOW at the same time
        for mv in ("NS", "ES"):
            sim.signals.log.append(TransitionEvent(t, mv, RED, RED_YELLOW))
    errs = _errors_after(m)
    assert any(e.startswith("conflict") for e in errs)


def test_audit_detects_a_crossing_on_red_and_a_short_headway():
    def m(sim):
        vs = [v for v in sim.vehicles.crossed("NS")]
        vs[0]._cross_ms = sim.config.window().start_ms + 1       # everything is RED at the start
        if len(vs) > 2:
            vs[2]._cross_ms = vs[1]._cross_ms + 1000             # < 1.9 s after its leader
    errs = _errors_after(m)
    assert any("crossed at" in e and "RED" in e for e in errs)
    assert any("headway" in e for e in errs)


def test_audit_detects_a_late_first_car():
    def m(sim):
        for mv in ("NS", "ES", "SS", "WS"):
            for v in sim.vehicles.crossed(mv):
                if v.released_ms is not None and v.cross_ms - v.released_ms == FIRST_CAR_MS:
                    v._cross_ms += 100
                    return
    assert any("first queued car" in e for e in _errors_after(m))
