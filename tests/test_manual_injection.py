"""Custom / Demo manual vehicle injection (smarttraffic.manual).

Exact counts (+1, +5, typed), zero-rate demand, no overlap or duplicates, signals and headways
obeyed (audited), Smart-T1 reacts only through the detectors, hardware mode refuses NS (real-
demand-only) but allows the 7 virtual movements, official runs are refused and unchanged,
replay is deterministic, reset clears pending requests, and the UI card fits at 1920x1080,
1366x768 and 1100x680.
"""
import json
import time

import pytest

from smarttraffic.conflicts import MOVES
from smarttraffic.custom import CustomDemand, CustomSpec
from smarttraffic.manual import MANUAL_SPACING_MS, MAX_BATCH, ManualInjector, replay_schedule
from smarttraffic.runner import RunConfig, run, simulate
from smarttraffic.viz_session import VizSession


def custom(rates=None, ctrl="Smart", measure_s=300):
    d = CustomDemand.from_mapping({m: (rates or {}).get(m, 0.0) for m in MOVES})
    return CustomSpec(d, controller=ctrl, measure_s=measure_s).config()


def vehicles(res, source=None):
    vs = json.loads(res.to_json())["vehicles"]
    return [v for v in vs if source is None or v["source"] == source]


# ------------------------------------------------------------------ exact counts
@pytest.mark.parametrize("n", [1, 5, 13, MAX_BATCH])
def test_each_request_creates_exactly_n_vehicles(n):
    res = run(custom(), injector=ManualInjector(schedule=[(5000, "ES", n)]))
    man = vehicles(res, "manual")
    assert len(man) == n and {v["movement"] for v in man} == {"ES"}
    meta = json.loads(res.to_json())["metadata"]
    assert meta["manual_vehicles"] == n and meta["manual_injections"] == [[5000, "ES", n, 5000,
                                                                          5000 + (n - 1) * MANUAL_SPACING_MS]]


def test_invalid_counts_are_refused():
    inj = ManualInjector()
    for bad in (0, -1, MAX_BATCH + 1, 2.5, True):
        with pytest.raises(ValueError):
            inj.request("NS", bad)


def test_zero_rate_demand_plus_manual_vehicles_all_served():
    res = run(custom(), injector=ManualInjector(schedule=[(0, "NS", 5), (0, "NL", 2), (60_000, "ES", 3)]))
    d = json.loads(res.to_json())
    assert d["metadata"]["arrivals_count"] == 0                      # no generated traffic at all
    assert d["intersection"]["arrivals"] == 10 and d["intersection"]["crossed"] == 10
    assert all(v["source"] == "manual" for v in d["vehicles"])


def test_manual_vehicles_are_added_to_generated_traffic():
    cfg = custom({m: 300 for m in MOVES})
    base = vehicles(run(cfg))
    res = run(cfg, injector=ManualInjector(schedule=[(30_000, "WL", 4)]))
    assert len(vehicles(res)) == len(base) + 4 and len(vehicles(res, "manual")) == 4
    ids = [v["id"] for v in vehicles(res)]
    assert len(ids) == len(set(ids))                                 # no duplicate ids


def test_batches_are_spaced_and_never_overlap():
    inj = ManualInjector(schedule=[(10_000, "SS", 5), (10_000, "SS", 5), (11_000, "SS", 2)])
    a = _Audited(custom(), inj)
    t = sorted(v["scheduled_ms"] for v in vehicles(a.res, "manual"))
    assert len(t) == 12 and all(b - x == MANUAL_SPACING_MS for x, b in zip(t, t[1:]))
    assert t[0] == 10_000
    assert a.rep["errors"] == []                                     # vehicle model spacing invariants


class _Audited:
    def __init__(self, cfg, inj, **kw):
        from tests.stage16_checks import Audit
        from smarttraffic.runner import run as _run
        self.audit = Audit(check_every=1)
        box = {}
        self.res = _run(cfg, injector=inj, observers=[self.audit, lambda v: box.update(v=v)], **kw)
        from tests.stage16_checks import _SimView
        self.rep = self.audit.finish(_SimView(cfg, box["v"], self.res), self.res)


def test_injected_vehicles_obey_signals_headways_and_first_car_rule():
    # heavy manual batches on every movement, zero generated traffic, Smart-T1 and FT-A
    sched = [(t, m, 8) for t in (0, 40_000, 90_000) for m in MOVES]
    for ctrl in ("Smart", "FT-A"):
        a = _Audited(custom(ctrl=ctrl), ManualInjector(schedule=sched))
        assert a.rep["errors"] == [], a.rep["errors"][:3]
        assert a.rep["conflicts"] == 0 and a.rep["first_car_checks"] > 5
        assert len(vehicles(a.res, "manual")) == 3 * 8 * 8


def test_smart_reacts_only_through_the_detectors():
    """No command is issued when a batch is requested: the first start_green for ES comes
    after the first ES vehicle reaches D2 (32 m) / D1 (2 m), as a detector call."""
    cmds, calls = [], []

    def obs(view):
        if view.sensors.has_call("ES") and not calls:
            calls.append(view.now_ms)
    sim = simulate(custom(), injector=ManualInjector(schedule=[(20_000, "ES", 3)]), observers=[obs],
                   collect_stats=False)
    es_start = [c.time_ms for c in sim.controller.command_log if c.movement == "ES" and c.kind == "start_green"]
    edges = [e.time_ms for e in sim.sensors.edge_log if e.movement == "ES" and e.occupied]
    # 150 m at 50 km/h: D2 (32 m) is reached 8.5 s after entry
    assert edges and edges[0] == 20_000 + round((150 - 32) / (50 / 3.6) * 1000)
    assert calls and calls[0] >= edges[0] and es_start and es_start[0] >= edges[0]
    assert not [c for c in sim.controller.command_log if c.time_ms < edges[0]]


def test_official_runs_refuse_an_injector_and_stay_byte_identical():
    cfg = RunConfig("S1", 1, "Smart", warmup_s=60.0, measure_s=120.0, flush_s=120.0)
    ref = run(cfg).to_json()
    with pytest.raises(ValueError, match="Custom / Demo"):
        run(cfg, injector=ManualInjector())
    assert run(cfg).to_json() == ref and "manual" not in ref
    assert "manual_injections" not in run(custom()).to_json()        # custom without injector: unchanged


def test_hardware_mode_rejects_ns_but_allows_the_other_movements():
    from smarttraffic.hardware import load_hardware_config
    inj = ManualInjector(blocked=load_hardware_config().real_only_movements)
    assert "real-demand-only" in inj.request("NS", 5)
    assert inj.request("ES", 5) is None and inj.request("WL", 1) is None
    # scheduled (replayed) NS requests are rejected too, never injected
    inj2 = ManualInjector(schedule=[(1000, "NS", 3), (1000, "SS", 2)])
    res = run(custom(), injector=inj2, exclude_movements=("NS",))
    man = vehicles(res, "manual")
    assert {v["movement"] for v in man} == {"SS"} and len(man) == 2
    meta = json.loads(res.to_json())["metadata"]
    assert meta["manual_rejected"] == [[1000, "NS", 3, "real-demand-only movement"]]


def test_hardware_session_ns_rejected_others_injected():
    from tests.test_viz import LiveEsp
    from smarttraffic.hardware import load_hardware_config
    cfg = custom(measure_s=60)
    s = VizSession(cfg, hw_config=load_hardware_config(), transport_factory=lambda d: LiveEsp(),
                   running=True, clock=lambda: time.monotonic() * 200).start()
    try:
        assert "real-demand-only" in s.inject("NS", 1)
        assert s.inject("EL", 2) is None
        assert s.wait_finished(60) and s.error is None
        man = vehicles(s.result, "manual")
        assert [v["movement"] for v in man] == ["EL", "EL"]
    finally:
        s.stop()


def test_requests_after_the_measurement_window_are_rejected():
    inj = ManualInjector(schedule=[(299_000, "NS", 2), (299_000, "SS", 1)])
    res = run(custom(measure_s=300), injector=inj)
    meta = json.loads(res.to_json())["metadata"]
    assert [v["movement"] for v in vehicles(res, "manual")] == ["SS"]     # 299.0 s fits, 300.5 s does not
    assert meta["manual_rejected"][0][:3] == [299_000, "NS", 2]


def test_deterministic_replay_from_the_recorded_metadata():
    cfg = custom({m: 200 for m in MOVES})
    live = ManualInjector(schedule=[(12_300, "NS", 5), (12_300, "EL", 2), (80_000, "WS", 10)])
    a = run(cfg, injector=live)
    b = run(cfg, injector=ManualInjector(schedule=replay_schedule(json.loads(a.to_json())["metadata"])))
    assert a.to_json() == b.to_json()


def test_live_session_injection_is_recorded_and_replays_identically():
    cfg = custom({m: 150 for m in MOVES}, measure_s=120)
    s = VizSession(cfg, speed=None, running=False).start()
    try:
        assert s.inject("NS", 5) is None and s.inject("ES", 1) is None   # while paused
        s.set_running(True)
        assert s.wait_finished(60) and s.error is None
        meta = json.loads(s.result.to_json())["metadata"]
        assert meta["manual_vehicles"] == 6 and meta["mode"] == "custom"
        replay = run(cfg, injector=ManualInjector(schedule=replay_schedule(meta)))
        assert replay.to_json() == s.result.to_json()
    finally:
        s.stop()


def test_reset_clears_pending_manual_requests():
    from smarttraffic.viz import UIState
    st = UIState(custom(measure_s=120), session_factory=lambda cc, **k: VizSession(cc, **k))
    try:
        assert st.inject("NS", 5)                     # queued while paused (not yet applied)
        assert st.session.injector.pending() == 5
        old = st.session
        st.reset()
        assert st.session is not old and st.session.injector.pending() == 0
        assert st.manual_state()[0] == {m: 0 for m in MOVES} and st.manual_message is None
    finally:
        st.close()


def test_manual_injection_is_custom_only_in_the_ui():
    from smarttraffic.viz import UIState
    st = UIState(RunConfig("S1", 1, "Smart", warmup_s=0.0, measure_s=60.0, flush_s=60.0),
                 session_factory=lambda cc, **k: VizSession(cc, **k))
    try:
        assert st.manual_state() is None and st.session.injector is None
        assert not st.inject("NS", 1) and "Custom / Demo" in st.manual_message
    finally:
        st.close()


# ------------------------------------------------------------------ UI card
pygame = pytest.importorskip("pygame")


@pytest.mark.parametrize("size", [(1920, 1080), (1366, 768), (1100, 680)], ids=lambda s: f"{s[0]}x{s[1]}")
@pytest.mark.parametrize("hw", [False, True], ids=["virtual", "hardware"])
def test_manual_card_fits_and_all_buttons_are_live(size, hw):
    import os
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.display.init()
    pygame.font.init()
    pygame.display.set_mode((10, 10))
    from smarttraffic import viz_render as vr
    from smarttraffic.viz import Fonts, Renderer, _StillState, capture
    from smarttraffic.hardware import load_hardware_config
    from tests.test_viz import DeadPort
    cfg = custom({m: 300 for m in MOVES})
    kw = dict(hw_config=load_hardware_config(), transport_factory=lambda d: DeadPort()) if hw else {}
    a, b = capture(cfg, [60_000], **kw)[60_000]
    r = Renderer(Fonts())
    surf = pygame.Surface(size)
    rec, cards = [], []
    ot, og = vr.text, vr.glass

    def spy_text(s_, fonts, st, pos, *x, **k):
        rr = ot(s_, fonts, st, pos, *x, **k)
        rec.append((st, pygame.Rect(rr)))
        return rr

    def spy_glass(s_, rect, *x, **k):
        cards.append(pygame.Rect(rect))
        return og(s_, rect, *x, **k)
    vr.text, vr.glass = spy_text, spy_glass
    try:
        r.draw(surf, _StillState(cfg, hardware=hw), a, b, a.t_ms)
    finally:
        vr.text, vr.glass = ot, og
    k, top, canvas, side = r.layout(size)
    card = [c for c in cards if any(s == "MANUAL VEHICLES" and c.contains(rr) for s, rr in rec)]
    assert len(card) == 1
    card = card[0]
    assert canvas.contains(card)
    hw_card = [c for c in cards if any(s == "HYBRID HARDWARE" and c.contains(rr) for s, rr in rec)][0]
    assert not card.colliderect(hw_card)
    inside = [(s, rr) for s, rr in rec if card.colliderect(rr)]
    assert all(card.contains(rr) for s, rr in inside), [(s, tuple(rr)) for s, rr in inside if not card.contains(rr)]
    names = {n for _, _, n in r.buttons}
    for m in MOVES:
        want = {f"manual:{m}:1", f"manual:{m}:5", f"manual:{m}:n"}
        if hw and m == "NS":
            assert not want & names and any(s == "real sensors only" for s, _ in inside)
        else:
            assert want <= names
    for rect, _, n in r.buttons:                    # every label inside its button
        if n.startswith("manual:"):
            br = pygame.Rect(rect)
            for s, rr in inside:
                if br.collidepoint(rr.center):
                    assert br.contains(rr), (n, s)


def test_manual_card_buttons_inject_through_the_app():
    import os
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    from smarttraffic.viz import App, UIState
    st = UIState(custom(measure_s=120), speed=None, session_factory=lambda cc, **k: VizSession(cc, **k))
    app = App(st, size=(1366, 768))
    try:
        app.frame()
        b = {n: rect for rect, _, n in app.r.buttons}
        app.click(b["manual:WS:5"].center)
        app.click(b["manual:count+"].center)                   # exact count 10 -> 11
        app.frame()
        b = {n: rect for rect, _, n in app.r.buttons}
        app.click(b["manual:NL:n"].center)
        assert st.manual_count == 11 and st.session.injector.pending() == 16
        st.toggle_run()
        assert st.session.wait_finished(60) and st.error is None
        assert st.manual_state()[0]["WS"] == 5 and st.manual_state()[0]["NL"] == 11
    finally:
        app.close()


def test_unused_injector_leaves_a_custom_run_exactly_unchanged():
    cfg = custom({m: 250 for m in MOVES})
    assert run(cfg, injector=ManualInjector()).to_json() == run(cfg).to_json()
