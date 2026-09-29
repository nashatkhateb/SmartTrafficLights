"""Stage 16 -- UI / presentation readiness and the interactive control matrix.

Rendering: 1920x1080, 1366x768 and 1100x680 (the smallest window the app allows) for Smart-T1,
FT-A, FT-TOD, hardware connected / disconnected, the Custom panel and a Custom run. Every text
drawn on the screen is recorded: nothing may leave the screen, sidebar texts must stay inside
the sidebar, and all 8 controller-table rows must be drawn. Labels are checked verbatim.

Controls: UIState is driven through start, pause / resume, reset, every scenario (S1-S10, then
Custom), every controller (FT-TOD only on S6), seed switching, all six speeds, hardware on / off;
no errors, the configuration always matches the choice, and runs stay deterministic.
"""
import json
import time

import pytest

pygame = pytest.importorskip("pygame")

from smarttraffic.conflicts import MOVES                                    # noqa: E402
from smarttraffic.custom import VEHICLE_POLICY_LABEL, CustomDemand, CustomSpec   # noqa: E402
from smarttraffic.hardware import load_hardware_config                     # noqa: E402
from smarttraffic.runner import RunConfig, run                              # noqa: E402
from smarttraffic.traffic_gen import generate_arrivals                     # noqa: E402
from smarttraffic.viz_session import SPEEDS, VizSession                     # noqa: E402

SIZES = ((1920, 1080), (1366, 768), (1100, 680))


@pytest.fixture(scope="module", autouse=True)
def display():
    import os
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.display.init()
    pygame.font.init()
    pygame.display.set_mode((10, 10))
    yield


def _c(sc="S1", ctrl="Smart", **k):
    from smarttraffic.viz import ui_config
    return ui_config(RunConfig(sc, 1, ctrl, **{**dict(warmup_s=0.0, measure_s=300.0, flush_s=300.0), **k}))


def _cases():
    from tests.test_viz import DeadPort, LiveEsp
    from smarttraffic.viz_custom import CustomDraft
    hw = load_hardware_config()
    spec = CustomSpec(CustomDemand.uniform(500, 150), controller="Smart", measure_s=600)
    return {
        "smart": (_c(), 120_000, {}, None),
        "ft-a": (_c("S1", "FT-A"), 120_000, {}, None),
        "ft-tod": (_c("S6", "FT-TOD"), 120_000, {}, None),
        "hw-connected": (_c(), 6000, dict(hw_config=hw, transport_factory=lambda d: LiveEsp()), None),
        "hw-disconnected": (_c(), 10_000, dict(hw_config=hw, transport_factory=lambda d: DeadPort()), None),
        "custom-panel": (_c(), 60_000, {}, CustomDraft(advanced_open=True)),
        "custom-run": (spec.config(), 120_000, {}, None),
    }


def _draw(name, size):
    from smarttraffic import viz_render as vr
    from smarttraffic.viz import Fonts, Renderer, _StillState, _real_ids_hint, capture
    cfg, t, kw, draft = _cases()[name]
    a, b = capture(cfg, [t], **kw)[t]
    r = Renderer(Fonts())
    r.real_ids_hint = _real_ids_hint()
    surf = pygame.Surface(size)
    rec = []
    # The Custom / Demo panel is a modal overlay drawn last, centred over the whole window (it
    # covers part of the sidebar). Texts drawn inside it are recorded separately, with the
    # overlay's own rectangle (its first rounded rectangle), and checked against that.
    r.modal_rect, r.modal_texts = None, []
    orig_text, orig_rrect, orig_panel = vr.text, vr.rrect, vr.Renderer._custom_panel
    state = {"modal": False}

    def spy_text(s_, fonts, st, pos, *args, **k):
        rr = orig_text(s_, fonts, st, pos, *args, **k)
        if s_ is surf:
            (r.modal_texts if state["modal"] else rec).append((st, pygame.Rect(rr)))
        return rr

    def spy_rrect(s_, rect, *args, **k):
        if state["modal"] and r.modal_rect is None and s_ is surf:
            r.modal_rect = pygame.Rect(rect)
        return orig_rrect(s_, rect, *args, **k)

    def spy_panel(self, *args, **k):
        state["modal"] = True
        try:
            return orig_panel(self, *args, **k)
        finally:
            state["modal"] = False
    vr.text, vr.rrect, vr.Renderer._custom_panel = spy_text, spy_rrect, spy_panel
    try:
        r.draw(surf, _StillState(cfg, hardware="hw_config" in kw, custom_draft=draft), a, b, a.t_ms)
    finally:
        vr.text, vr.rrect, vr.Renderer._custom_panel = orig_text, orig_rrect, orig_panel
    return r, a, rec + r.modal_texts


@pytest.mark.parametrize("size", SIZES, ids=lambda s: f"{s[0]}x{s[1]}")
@pytest.mark.parametrize("name", ["smart", "ft-a", "ft-tod", "hw-connected", "hw-disconnected",
                                  "custom-panel", "custom-run"])
def test_no_clipped_text_and_all_eight_rows(name, size):
    r, a, rec = _draw(name, size)
    W, H = size
    k, top, canvas, side = r.layout(size)
    screen = pygame.Rect(0, 0, W, H)
    assert rec
    off = [(s, tuple(rr)) for s, rr in rec if not screen.contains(rr)]
    assert off == []
    under = rec[:len(rec) - len(r.modal_texts)]          # the normal UI (under any overlay)
    side_bad = [(s, tuple(rr)) for s, rr in under
                if rr.x >= side.x - 2 and not (side.contains(rr) or top.contains(rr))]
    assert side_bad == []
    rows = [m for m in MOVES if any(s == m and rr.x >= side.x for s, rr in under)]
    assert rows == list(MOVES)                           # all 8 rows of the controller table
    if name == "custom-panel":                           # the overlay: every text inside it
        assert r.modal_rect is not None and r.modal_texts
        assert screen.contains(r.modal_rect)
        modal_bad = [(s, tuple(rr)) for s, rr in r.modal_texts if not r.modal_rect.contains(rr)]
        assert modal_bad == []
    else:
        assert r.modal_rect is None and r.modal_texts == []
    # every button label lies completely inside its own button (while the overlay is open, only
    # its own buttons are live and only its own texts are on top)
    labels = r.modal_texts if name == "custom-panel" else under
    for brect, _, bname in r.buttons:
        br = pygame.Rect(brect)
        for s, rr in labels:
            if br.collidepoint(rr.center) and s.strip():
                assert br.contains(rr), (bname, s, tuple(rr), tuple(br))


def _texts(name, size=(1366, 768)):
    return [s for s, _ in _draw(name, size)[2]]


def test_header_shows_smart_t1_and_fixed_labels():
    assert any("Smart-T1" in s for s in _texts("smart"))
    assert any("FT-TOD" in s for s in _texts("ft-tod"))


def test_custom_panel_states_the_standard_policy_and_locked_t1():
    t = _texts("custom-panel")
    assert any(VEHICLE_POLICY_LABEL in s for s in t)
    assert VEHICLE_POLICY_LABEL == "Stop on Yellow · first queued car starts at green"
    assert any("Smart-T1 (locked)" in s for s in t)


def test_hardware_card_texts():
    on = _texts("hw-connected")
    for want in ("2 real + 14 virtual detectors", "NS: real demand only · no virtual cars",
                 "Real IR NS:D1", "Real IR NS:D2", "R/Y/G head mirrors NS", "Connected"):
        assert want in on, want
    off = _texts("hw-disconnected")
    assert "Disconnected" in off and off.count("Fault · held") == 2
    idle = " ".join(_texts("smart"))
    assert "Off · 16 simulated detectors" in idle and "2 real + 14 virtual" in idle


def test_both_real_detectors_are_drawn_in_their_own_slots(monkeypatch):
    from smarttraffic.viz_render import Renderer
    drawn = []
    orig = Renderer._ir_sensor

    def spy(self, surf, view, d, *a, **k):
        drawn.append((d.movement, d.detector))
        return orig(self, surf, view, d, *a, **k)
    monkeypatch.setattr(Renderer, "_ir_sensor", spy)
    r, a, rec = _draw("hw-connected", (1366, 768))
    assert a.hardware.real_ids == (("NS", "D1"), ("NS", "D2"))
    assert sorted(set(drawn)) == [("NS", "D1"), ("NS", "D2")]   # both IR sensors, in their own slots


def test_ui_terms_match_the_docs():
    from pathlib import Path
    readme = (Path(__file__).resolve().parent.parent / "README.md").read_text(encoding="utf-8")
    for term in ("Smart-T1", "FT-TOD", "Custom / Demo", "CUSTOM / DEMO", "Stop on Yellow",
                 "2 real + 14 virtual"):
        assert term in readme, term


# ------------------------------------------------------------------ interactive control matrix
def _state(c, **kw):
    from smarttraffic.viz import UIState

    def factory(cc, **k):
        arrivals = None if cc.mode == "custom" else generate_arrivals(cc.scenario, cc.seed, -cc.warmup_s, cc.measure_s)
        return VizSession(cc, arrivals=arrivals, **k)
    return UIState(c, session_factory=factory, **kw)


def test_ui_control_matrix_never_errors_and_config_follows_choices():
    from smarttraffic.viz import controllers_for
    c = _c("S1", "Smart", warmup_s=60.0, measure_s=120.0, flush_s=120.0)
    st = _state(c, speed=None)
    try:
        st.toggle_run()
        time.sleep(0.2)
        assert st.running and st.error is None
        st.toggle_run()                                     # pause
        assert not st.running
        st.toggle_run()                                     # resume
        assert st.running
        st.reset()
        assert not st.running and st.error is None
        seen = []
        for _ in range(11):                                 # S2 ... S10, then back to S1 via Custom
            st.next_scenario()
            if st.custom_open:
                st.cancel_custom()
                break
            seen.append(st.config.scenario)
            for ctrl in controllers_for(st.config.scenario):
                st.set_controller(ctrl)
                assert st.config.controller == ctrl and st.error is None
                if ctrl == "Smart":
                    assert st.config.smart_params.tau_s == 15.0      # always Smart-T1
            assert ("FT-TOD" in controllers_for(st.config.scenario)) == (st.config.scenario == "S6")
        assert seen == ["S2", "S3", "S4", "S5", "S6", "S7a", "S7b", "S8", "S9", "S10"]
        st.next_seed()
        st.next_seed()
        st.prev_seed()
        assert st.config.seed == 2
        for sp in SPEEDS:
            st.set_speed(sp)
            assert st.speed == sp and st.session.speed == sp
        st.toggle_run()
        time.sleep(0.2)
        assert st.error is None
    finally:
        st.close()


def test_custom_mode_from_the_ui_and_back():
    st = _state(_c("S10", "FT-A"), speed=None)
    try:
        st.next_scenario()                                  # after S10: Custom panel opens
        assert st.custom_open
        st.custom_draft.set_demand("NS", 900)
        assert st.apply_custom() and st.is_custom and st.config.mode == "custom"
        assert st.config.demand.rate("NS") == 900
        st.set_controller("FT-A")
        assert st.config.controller == "FT-A" and st.is_custom
        st.next_scenario()                                  # back to the official S1
        assert not st.is_custom and st.config.scenario == "S1" and st.error is None
    finally:
        st.close()


def test_hardware_toggle_locks_speed_and_back_to_virtual_only():
    from tests.test_viz import DeadPort
    st = _state(_c(), speed=5.0, transport_factory=lambda d: DeadPort())
    try:
        st.toggle_hardware()
        assert st.hardware and st.speed == 1.0 and st.session.hw_config is not None
        st.set_speed(None)                                  # ignored in hardware mode
        assert st.speed == 1.0
        st.toggle_hardware()
        assert not st.hardware and st.session.hw_config is None and st.error is None
    finally:
        st.close()


def test_max_speed_session_equals_a_plain_run():
    c = _c("S7b", "Smart", warmup_s=60.0, measure_s=240.0, flush_s=300.0)
    arr = generate_arrivals(c.scenario, c.seed, -c.warmup_s, c.measure_s)
    ref = run(c, arrivals=arr).to_json()
    s = VizSession(c, arrivals=arr, speed=None, running=True).start()
    assert s.wait_finished(120) and s.error is None
    assert s.result.to_json() == ref
    assert json.loads(ref)["intersection"]["safety_violations"] == 0


def test_screenshot_mode(tmp_path):
    from smarttraffic import viz
    for size in SIZES:
        out = tmp_path / f"shot_{size[0]}.png"
        viz.render_to_file(str(out), _c(), 60.0, size)
        assert out.exists() and out.stat().st_size > 10_000
