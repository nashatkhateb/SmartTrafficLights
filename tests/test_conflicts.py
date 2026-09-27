"""Tests for smarttraffic.conflicts against study §3.

The expected values are written here independently of the module's matrix:
they come from the study's list of 8 compatible pairs and its statement that
the only merge pairs are NS-EL, NL-WS, SS-WL, SL-ES. Every other distinct
pair crosses.
"""
from itertools import combinations, product

import pytest

from smarttraffic import conflicts as cf
from smarttraffic.conflicts import (InvalidMovementError, SafetyViolation,
                                    check_invariant, compatible,
                                    conflict_type, conflicting_pairs,
                                    conflicts_with, is_allowed_set)

MOVES = ["NS", "NL", "SS", "SL", "ES", "EL", "WS", "WL"]

APPROVED = [{"NS", "SS"}, {"NL", "SL"}, {"NS", "NL"}, {"SS", "SL"},
            {"ES", "WS"}, {"EL", "WL"}, {"ES", "EL"}, {"WS", "WL"}]
MERGES = [{"NS", "EL"}, {"NL", "WS"}, {"SS", "WL"}, {"SL", "ES"}]


def expected_code(a, b):
    if a == b:
        return "-"
    if {a, b} in APPROVED:
        return "OK"
    if {a, b} in MERGES:
        return "M"
    return "C"


ALL_64 = list(product(MOVES, MOVES))
DISTINCT_28 = list(combinations(MOVES, 2))


# ------------------------------------------------------------ basic shape
def test_exactly_eight_movements_in_study_order():
    assert list(cf.MOVES) == MOVES


def test_64_ordered_combinations_are_covered():
    assert len(ALL_64) == 64


# ----------------------------------------------- all 64 ordered pairs
@pytest.mark.parametrize("a,b", ALL_64, ids=[f"{a}-{b}" for a, b in ALL_64])
def test_matrix_code_matches_study(a, b):
    assert conflict_type(a, b) == expected_code(a, b)


@pytest.mark.parametrize("a,b", ALL_64, ids=[f"{a}-{b}" for a, b in ALL_64])
def test_compatible_matches_study(a, b):
    assert compatible(a, b) == (a == b or {a, b} in APPROVED)


# --------------------------------------------------------------- symmetry
@pytest.mark.parametrize("a,b", ALL_64, ids=[f"{a}-{b}" for a, b in ALL_64])
def test_symmetry(a, b):
    assert conflict_type(a, b) == conflict_type(b, a)
    assert compatible(a, b) == compatible(b, a)


# ------------------------------------------------------------- self pairs
@pytest.mark.parametrize("m", MOVES)
def test_movement_compatible_with_itself(m):
    assert compatible(m, m)
    assert conflict_type(m, m) == "-"


@pytest.mark.parametrize("m", MOVES)
def test_single_movement_always_allowed(m):
    assert is_allowed_set([m])
    check_invariant([m])  # must not raise


def test_all_red_is_allowed():
    assert is_allowed_set([])
    check_invariant([])


# ------------------------------------------------------- approved groups
def test_groups_constant_equals_approved_list():
    assert [set(g) for g in cf.GROUPS] == APPROVED


@pytest.mark.parametrize("g", APPROVED, ids=["+".join(sorted(g)) for g in APPROVED])
def test_approved_group_is_compatible_and_allowed(g):
    a, b = sorted(g)
    assert compatible(a, b)
    assert is_allowed_set(g)
    check_invariant(g)


def test_no_unapproved_pair_is_compatible():
    compat = [{a, b} for a, b in DISTINCT_28 if compatible(a, b)]
    assert sorted(map(sorted, compat)) == sorted(map(sorted, APPROVED))
    assert len(compat) == 8
    assert len(DISTINCT_28) - len(compat) == 20  # 20 conflicting pairs


def test_groups_are_maximal_no_three_can_be_green():
    for trio in combinations(MOVES, 3):
        assert not is_allowed_set(trio), trio


def test_each_movement_has_exactly_two_partners():
    for m in MOVES:
        partners = [x for x in MOVES if x != m and compatible(m, x)]
        assert len(partners) == 2, (m, partners)


# ------------------------------------- structural facts stated in §3 text
def test_cross_axis_pairs_12_cross_4_merge():
    ns = ["NS", "NL", "SS", "SL"]
    ew = ["ES", "EL", "WS", "WL"]
    codes = [conflict_type(a, b) for a in ns for b in ew]
    assert codes.count("C") == 12 and codes.count("M") == 4
    merges = [{a, b} for a in ns for b in ew if conflict_type(a, b) == "M"]
    assert sorted(map(sorted, merges)) == sorted(map(sorted, MERGES))


def test_within_axis_conflicts_are_straight_vs_opposing_left():
    within = [{a, b} for a, b in DISTINCT_28
              if a[0] in "NS" and b[0] in "NS" or a[0] in "EW" and b[0] in "EW"]
    conflicting = [p for p in within if not compatible(*sorted(p))]
    assert sorted(map(sorted, conflicting)) == sorted(map(sorted, [
        {"NS", "SL"}, {"SS", "NL"}, {"ES", "WL"}, {"WS", "EL"}]))
    assert all(conflict_type(*sorted(p)) == "C" for p in conflicting)


def test_brief_example_ss_sl_to_ss_ns():
    assert is_allowed_set({"SS", "SL"})
    assert is_allowed_set({"SS", "NS"})
    assert not compatible("NS", "SL")
    assert not is_allowed_set({"SS", "SL", "NS"})


def test_conflicts_with():
    assert conflicts_with("NS") == ("SL", "ES", "EL", "WS", "WL")
    for m in MOVES:
        assert len(conflicts_with(m)) == 5


# ----------------------------------------------------- safety invariant
def test_invariant_raises_on_conflict_and_names_pair():
    with pytest.raises(SafetyViolation, match=r"NS-SL \(C\)"):
        check_invariant(["NS", "SL"])
    with pytest.raises(SafetyViolation, match=r"NS-EL \(M\)"):
        check_invariant(["EL", "NS"])


def test_conflicting_pairs_lists_all():
    assert conflicting_pairs(["NS", "SS", "SL"]) == [("NS", "SL")]
    assert conflicting_pairs(["NS", "SS"]) == []


def test_safety_violation_is_assertion_error():
    assert issubclass(SafetyViolation, AssertionError)


# ---------------------------------------------------- invalid names
BAD_NAMES = ["XX", "ns", "", " NS", "NS ", "N", "NR", "NSX", None, 5, ("NS",)]


@pytest.mark.parametrize("bad", BAD_NAMES, ids=repr)
def test_invalid_name_raises_everywhere(bad):
    with pytest.raises(InvalidMovementError):
        compatible(bad, "NS")
    with pytest.raises(InvalidMovementError):
        compatible("NS", bad)
    with pytest.raises(InvalidMovementError):
        conflict_type(bad, "NS")
    with pytest.raises(InvalidMovementError):
        conflicts_with(bad)
    with pytest.raises(InvalidMovementError):
        is_allowed_set(["NS", bad])
    with pytest.raises(InvalidMovementError):
        check_invariant(["NS", bad])


def test_invalid_movement_error_is_value_error_with_clear_message():
    assert issubclass(InvalidMovementError, ValueError)
    with pytest.raises(InvalidMovementError, match="unknown movement 'XX'.*NS, NL"):
        compatible("XX", "NS")


def test_bare_string_rejected_as_collection():
    # "NS" as a collection would otherwise be read as movements "N" and "S".
    with pytest.raises(InvalidMovementError, match="collection"):
        is_allowed_set("NS")
    with pytest.raises(InvalidMovementError):
        check_invariant("NSSS")
