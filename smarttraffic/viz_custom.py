"""viz_custom -- the editable state of the Custom / Demo panel (Stage 14b). No pygame here.

The panel edits a CustomDraft; Apply turns it into a validated custom.CustomSpec (or shows
the validation error and changes nothing). The draft enforces the user-interface limits:
demands are capped at custom.UI_LIMIT_VEH_H (1800 veh/h) and are never negative; typed text
that is not a number is refused. Everything else is validated by CustomSpec itself.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from smarttraffic.conflicts import MOVES
from smarttraffic.custom import (CUSTOM_CONTROLLERS, CUSTOM_SMART_FIELDS, CustomDemand, CustomInputError,
                                 CustomSpec, SMART_PARAM_SETS, UI_LIMIT_VEH_H, YELLOW_POLICY_SECONDS,
                                 saturation_report, smart_params_for)

__all__ = ["CustomDraft", "TAB_ORDER", "MEASURE_CHOICES", "DEMAND_STEP", "SMART_STEP", "PRESETS", "MOVEMENT_NAMES",
           "SMART_FIELD_LABELS"]

MEASURE_CHOICES: Tuple[int, ...] = (300, 600, 1800, 3600)
TAB_ORDER: Tuple[str, ...] = ("NS", "SS", "ES", "WS", "NL", "SL", "EL", "WL")   # panel layout order
DEMAND_STEP = {"straight": 50.0, "left": 10.0}
SMART_STEP = {"tau_s": 1.0, "w_max_s": 5.0, "max_green_straight_s": 5.0, "max_green_left_s": 5.0}
SMART_FIELD_LABELS = {"tau_s": "τ (s)", "w_max_s": "W_MAX (s)", "max_green_straight_s": "Max G straight (s)",
                      "max_green_left_s": "Max G left (s)"}
MOVEMENT_NAMES = {"NS": "North · straight", "SS": "South · straight", "ES": "East · straight",
                  "WS": "West · straight", "NL": "North · left", "SL": "South · left", "EL": "East · left",
                  "WL": "West · left"}


def _preset(sid: str) -> Dict[str, float]:
    from smarttraffic.traffic_gen import load_scenarios
    seg = load_scenarios()[sid].segments[0]
    return {m: float(q) for m, q in seg.rates}


PRESETS: Tuple[Tuple[str, str], ...] = (("S1", "S1 balanced"), ("S7a", "S7a heavy"), ("S8", "S8 lefts"),
                                        ("ZERO", "Zero"))


@dataclass
class CustomDraft:
    demand: Dict[str, float] = field(default_factory=lambda: _preset("S1"))
    seed: int = 1
    controller: str = "Smart"
    measure_s: int = 600
    param_set: str = "T1"
    overrides: Dict[str, float] = field(default_factory=dict)
    yellow_policy: str = "study"
    immediate_first_vehicle: bool = False
    advanced_open: bool = False
    focus: Optional[str] = None           # "NS".."WL" or a Smart field name while typing
    text: str = ""                        # typed text of the focused field
    error: Optional[str] = None
    note: Optional[str] = None

    # ------------------------------------------------------------ construction
    @classmethod
    def from_spec(cls, spec: CustomSpec) -> "CustomDraft":
        return cls(demand=spec.demand.as_dict(), seed=spec.seed, controller=spec.controller,
                   measure_s=spec.measure_s, param_set=spec.param_set, overrides=dict(spec.smart_overrides),
                   yellow_policy=spec.yellow_policy, immediate_first_vehicle=spec.immediate_first_vehicle)

    # ------------------------------------------------------------ values
    def smart_value(self, name: str) -> float:
        if name in self.overrides:
            return self.overrides[name]
        base = smart_params_for("T1" if self.param_set != "study" else "study")
        return float(getattr(base, name))

    def set_demand(self, m: str, value: float) -> None:
        capped = min(float(UI_LIMIT_VEH_H), max(0.0, float(value)))
        self.note = (f"{m} capped at {UI_LIMIT_VEH_H} veh/h (interface limit)" if value > UI_LIMIT_VEH_H
                     else None)
        self.demand[m] = capped
        self.error = None

    def step_demand(self, m: str, sign: int) -> None:
        step = DEMAND_STEP["straight" if m[1] == "S" else "left"]
        self.set_demand(m, round(self.demand[m] / step) * step + sign * step)

    def set_smart(self, name: str, value: float) -> None:
        lo, hi = CUSTOM_SMART_FIELDS[name]
        self.param_set = "custom"
        self.overrides[name] = min(hi, max(lo, float(value)))
        self.error = None

    def step_smart(self, name: str, sign: int) -> None:
        self.set_smart(name, self.smart_value(name) + sign * SMART_STEP[name])

    def load_preset(self, pid: str) -> None:
        self.demand = {m: 0.0 for m in MOVES} if pid == "ZERO" else _preset(pid)
        self.error = self.note = None

    def set_param_set(self, ps: str) -> None:
        if ps not in ("T1", "custom"):              # Smart-study is not a user-facing option
            raise ValueError(ps)
        self.param_set = ps
        if ps != "custom":
            self.overrides = {}
        self.error = None

    def set_controller(self, c: str) -> None:
        if c in CUSTOM_CONTROLLERS:
            self.controller = c

    def set_measure(self, s: int) -> None:
        self.measure_s = int(s)

    def set_seed(self, s: int) -> None:
        self.seed = max(0, int(s))

    def set_yellow(self, name: str) -> None:
        if name in YELLOW_POLICY_SECONDS:
            self.yellow_policy = name

    def toggle_advanced(self) -> None:
        self.advanced_open = not self.advanced_open

    # ------------------------------------------------------------ typing
    def focus_field(self, name: Optional[str]) -> None:
        self.commit_text()
        self.focus = name
        self.text = ""

    def type_char(self, ch: str) -> None:
        if self.focus is None or not ch:
            return
        if ch.isdigit() or (ch == "." and "." not in self.text):
            if len(self.text) < 7:
                self.text += ch

    def backspace(self) -> None:
        self.text = self.text[:-1]

    def commit_text(self) -> None:
        """Apply the typed text to the focused field (empty text leaves the field unchanged)."""
        if self.focus is None or self.text == "":
            return
        try:
            v = float(self.text)
        except ValueError:
            self.error = f"{self.text!r} is not a number"
            self.text = ""
            return
        if self.focus in MOVES:
            self.set_demand(self.focus, v)
        else:
            self.set_smart(self.focus, v)
        self.text = ""

    def display(self, name: str) -> str:
        if self.focus == name and self.text:
            return self.text
        v = self.demand[name] if name in MOVES else self.smart_value(name)
        return f"{v:g}"

    # ------------------------------------------------------------ result
    def saturation(self):
        try:
            return saturation_report(CustomDemand.from_mapping(self.demand))
        except CustomInputError:
            return None

    def to_spec(self) -> Tuple[Optional[CustomSpec], Optional[str]]:
        self.commit_text()
        self.focus = None
        try:
            if self.controller == "Smart":
                ps = self.param_set
                ov = tuple(sorted(self.overrides.items())) if ps == "custom" else ()
            else:                                   # FT-A: Smart parameters do not apply
                ps, ov = "T1", ()
            spec = CustomSpec(CustomDemand.from_mapping(self.demand), self.seed, self.controller, self.measure_s,
                              ps, ov, self.yellow_policy, self.immediate_first_vehicle)
        except CustomInputError as e:
            self.error = str(e)
            return None, self.error
        self.error = None
        return spec, None
