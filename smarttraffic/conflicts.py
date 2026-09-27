"""conflicts -- conflict/compatibility model for the eight signalled movements.

Spec: docs/Eight-Movement_Intersection_Study.html, §3 (and §13 module table).
Responsibility: matrix, groups, compatible(), invariant check.

Movement names follow the approach the vehicle comes FROM:
    NS = enters from North, goes straight (exits South)
    NL = enters from North, turns left   (exits East)
    ... and likewise for S, E, W.

The study's matrix (§3) is transcribed below verbatim. Codes:
    "OK" = compatible (may be green together)
    "C"  = paths cross inside the junction
    "M"  = paths merge into the same exit leg (treated as a conflict, §2)
    "-"  = diagonal (a movement with itself)

Everything else in this module is derived from that table, and a
self-check at import time confirms the table agrees with the study's list
of eight approved compatible pairs. If the two ever disagree, importing
this module fails loudly rather than letting a controller run on a wrong
model.

This module has no knowledge of signals, vehicles, sensors or timing.
"""
from itertools import combinations
from typing import Dict, FrozenSet, Iterable, List, Optional, Tuple

# ---------------------------------------------------------------- movements
MOVES: Tuple[str, ...] = ("NS", "NL", "SS", "SL", "ES", "EL", "WS", "WL")

OK, CROSS, MERGE, SELF = "OK", "C", "M", "-"


class InvalidMovementError(ValueError):
    """Raised when a name is not one of the eight movements in MOVES."""


class SafetyViolation(AssertionError):
    """Raised when conflicting movements are active (non-RED) at the same time."""


# ------------------------------------------------- study §3 matrix, verbatim
#                 NS     NL     SS     SL     ES     EL     WS     WL
_MATRIX_ROWS = {
    "NS": ("-",   "OK",  "OK",  "C",   "C",   "M",   "C",   "C"),
    "NL": ("OK",  "-",   "C",   "OK",  "C",   "C",   "M",   "C"),
    "SS": ("OK",  "C",   "-",   "OK",  "C",   "C",   "C",   "M"),
    "SL": ("C",   "OK",  "OK",  "-",   "M",   "C",   "C",   "C"),
    "ES": ("C",   "C",   "C",   "M",   "-",   "OK",  "OK",  "C"),
    "EL": ("M",   "C",   "C",   "C",   "OK",  "-",   "C",   "OK"),
    "WS": ("C",   "M",   "C",   "C",   "OK",  "C",   "-",   "OK"),
    "WL": ("C",   "C",   "M",   "C",   "C",   "OK",  "OK",  "-"),
}

CONFLICT_MATRIX: Dict[str, Dict[str, str]] = {
    a: dict(zip(MOVES, _MATRIX_ROWS[a])) for a in MOVES
}

# ------------------------------------- study §3 approved maximal groups
GROUPS: Tuple[FrozenSet[str], ...] = (
    frozenset({"NS", "SS"}),  # N/S opposing straights
    frozenset({"NL", "SL"}),  # N/S dual protected lefts
    frozenset({"NS", "NL"}),  # North split
    frozenset({"SS", "SL"}),  # South split
    frozenset({"ES", "WS"}),  # E/W opposing straights
    frozenset({"EL", "WL"}),  # E/W dual protected lefts
    frozenset({"ES", "EL"}),  # East split
    frozenset({"WS", "WL"}),  # West split
)

# The set of unordered distinct pairs that may be green together.
COMPATIBLE_PAIRS: FrozenSet[FrozenSet[str]] = frozenset(GROUPS)


# ------------------------------------------------------------ validation
def validate_movement(m: object) -> str:
    """Return m if it is one of the eight movement names, else raise.

    Names are exact and case-sensitive ("NS", not "ns" or " NS").
    """
    if not isinstance(m, str):
        raise InvalidMovementError(
            f"movement must be a str, got {type(m).__name__}: {m!r}")
    if m not in CONFLICT_MATRIX:
        raise InvalidMovementError(
            f"unknown movement {m!r}; valid names are {', '.join(MOVES)}")
    return m


def _validate_all(moves: Iterable[object]) -> List[str]:
    if isinstance(moves, str):
        # A bare string like "NS" would otherwise iterate as "N", "S".
        raise InvalidMovementError(
            f"expected a collection of movement names, got the string {moves!r}")
    return [validate_movement(m) for m in moves]


# --------------------------------------------------------------- queries
def conflict_type(a: str, b: str) -> str:
    """Matrix code for the pair: "OK", "C" (cross), "M" (merge) or "-" (a == b)."""
    return CONFLICT_MATRIX[validate_movement(a)][validate_movement(b)]


def compatible(a: str, b: str) -> bool:
    """True if a and b may be non-RED at the same time.

    A movement is compatible with itself (§7 pseudocode: a == b).
    """
    validate_movement(a)
    validate_movement(b)
    return a == b or frozenset((a, b)) in COMPATIBLE_PAIRS


def conflicts_with(m: str) -> Tuple[str, ...]:
    """All movements that conflict with m (crossing or merging), in MOVES order."""
    validate_movement(m)
    return tuple(x for x in MOVES if not compatible(m, x))


def conflicting_pairs(moves: Iterable[str]) -> List[Tuple[str, str]]:
    """Every conflicting pair within `moves` (empty list if the set is safe)."""
    ms = sorted(set(_validate_all(moves)), key=MOVES.index)
    return [(a, b) for a, b in combinations(ms, 2) if not compatible(a, b)]


def is_allowed_set(moves: Iterable[str]) -> bool:
    """True if every movement in `moves` may be green at the same time.

    The empty set (all red) and any single movement are always allowed.
    """
    return not conflicting_pairs(moves)


def check_invariant(active: Iterable[str]) -> None:
    """Safety invariant (§7): no two non-RED movements may conflict.

    `active` is the collection of movements whose signal is GREEN, YELLOW
    or ALLRED. Raises SafetyViolation naming each conflicting pair.
    The simulator will call this every step (§7, §13).
    """
    bad = conflicting_pairs(active)
    if bad:
        detail = ", ".join(f"{a}-{b} ({conflict_type(a, b)})" for a, b in bad)
        raise SafetyViolation(f"conflicting movements active together: {detail}")


# -------------------------------------------- import-time self-consistency
def _self_check() -> None:
    assert tuple(CONFLICT_MATRIX) == MOVES, "matrix rows out of order"
    for a in MOVES:
        assert len(_MATRIX_ROWS[a]) == len(MOVES), f"row {a} has wrong length"
        for b in MOVES:
            code = CONFLICT_MATRIX[a][b]
            assert code in (OK, CROSS, MERGE, SELF), f"bad code {code!r} at {a},{b}"
            assert (code == SELF) == (a == b), f"diagonal error at {a},{b}"
            assert code == CONFLICT_MATRIX[b][a], f"matrix not symmetric at {a},{b}"
    ok_pairs = {frozenset((a, b)) for a, b in combinations(MOVES, 2)
                if CONFLICT_MATRIX[a][b] == OK}
    assert ok_pairs == set(GROUPS), (
        "matrix OK entries do not match the approved GROUPS")
    for g in GROUPS:
        assert len(g) == 2 and g <= set(MOVES), f"bad group {set(g)}"


_self_check()
