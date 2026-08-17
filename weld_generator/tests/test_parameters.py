"""ISO limit functions, checked against the tables transcribed in docs/PARAMETERS.md.

Values here are read off the standard, not off the implementation.
"""

from __future__ import annotations

import pytest

from weldgen.config import (
    angular_misalignment_limit,
    classify_quality,
    linear_misalignment_limit,
    root_gap_limit,
)


# ISO 5817:2023 Table 1, no. 5071 (PARAMETERS.md §2.1)
@pytest.mark.parametrize("t,level,expected", [
    (2.0, "D", 0.25 * 2.0 + 0.2),
    (2.0, "C", 0.15 * 2.0 + 0.2),
    (2.0, "B", 0.10 * 2.0 + 0.2),
    (3.0, "D", 0.25 * 3.0 + 0.2),     # boundary: the t <= 3 row still applies
    (8.4, "D", 0.25 * 8.4),
    (8.4, "C", 0.15 * 8.4),
    (8.4, "B", 0.10 * 8.4),
    (40.0, "D", 5.0),                 # caps
    (40.0, "C", 4.0),
    (40.0, "B", 3.0),
])
def test_linear_misalignment_5071(t, level, expected):
    assert linear_misalignment_limit(t, level) == pytest.approx(expected)


def test_linear_misalignment_is_monotone_in_level():
    """B is strictest, D loosest — at every thickness."""
    for t in (0.5, 1.0, 3.0, 3.1, 8.4, 12.0):
        b, c, d = (linear_misalignment_limit(t, lv) for lv in "BCD")
        assert b <= c <= d


# ISO 5817:2023 Table 1, no. 617 (PARAMETERS.md §2.4)
@pytest.mark.parametrize("t,throat,level,expected", [
    (2.0, 1.4, "D", 0.1 * 1.4 + 0.5),
    (2.0, 1.4, "C", 0.1 * 1.4 + 0.3),
    (2.0, 1.4, "B", 0.1 * 1.4 + 0.2),
    (8.4, 5.88, "D", 0.3 * 5.88 + 1.0),
    (8.4, 5.88, "C", 0.2 * 5.88 + 0.5),
    (8.4, 5.88, "B", 0.1 * 5.88 + 0.5),
    (50.0, 100.0, "D", 4.0),          # caps
    (50.0, 100.0, "C", 3.0),
    (50.0, 100.0, "B", 2.0),
])
def test_root_gap_617(t, throat, level, expected):
    assert root_gap_limit(t, throat, level) == pytest.approx(expected)


def test_angular_limits_are_our_convention():
    """Table 1 sets NO limit on beta — its numbering skips 3.3 entirely.

    2 deg / 1 deg come from Annex B Table B.1, whose columns are fatigue classes
    C 63 / B 90, not quality levels. D is unconstrained, so we double C and say so.
    """
    assert angular_misalignment_limit("C") == 2.0
    assert angular_misalignment_limit("B") == 1.0
    assert angular_misalignment_limit("D") == 4.0


# PARAMETERS.md §2.5 — the level is the WEAKEST of the strictest levels satisfied.
def test_quality_is_driven_by_the_worst_defect():
    t, throat = 8.4, 5.88
    # Everything comfortably inside B.
    assert classify_quality(t, 0.0, 0.0, 0.0, throat) == "B"
    # Misalignment alone drags an otherwise-B joint down to D.
    h_d = linear_misalignment_limit(t, "D")
    assert classify_quality(t, h_d, 0.0, 0.0, throat) == "D"
    # Angle alone does the same.
    assert classify_quality(t, 0.0, 4.0, 0.0, throat) == "D"
    # Gap alone does the same.
    assert classify_quality(t, 0.0, 0.0, root_gap_limit(t, throat, "D"), throat) == "D"


def test_below_D_is_reachable():
    t, throat = 8.4, 5.88
    assert classify_quality(t, 99.0, 0.0, 0.0, throat) == "below_D"
    assert classify_quality(t, 0.0, 99.0, 0.0, throat) == "below_D"
    assert classify_quality(t, 0.0, 0.0, 99.0, throat) == "below_D"


def test_classification_is_exactly_at_the_boundary():
    """A value exactly on a limit satisfies that level — limits are `<=`."""
    t, throat = 8.4, 5.88
    assert classify_quality(t, linear_misalignment_limit(t, "B"), 0.0, 0.0, throat) == "B"
    assert classify_quality(t, 0.0, angular_misalignment_limit("C"), 0.0, throat) == "C"


def test_realised_mix_is_roughly_balanced():
    """PARAMETERS.md §2.5 wants balanced support on the stratification axis.

    The realised mix is not the target mix — a scene is classified by its worst axis, so
    it can only ever land at or below what was aimed for. Assert the weaker property that
    actually matters: every class is well represented.
    """
    from collections import Counter

    from weldgen.config import load_config, sample_joint
    from weldgen.rng import Streams

    cfg = load_config()
    # Geometry test, not a curation test: the tier-1 omission policy would drop the
    # seeds whose camera happens to miss, and an xfail marker would then hide the
    # exception instead of reporting the geometry.
    cfg["require_visible_seam"] = False
    counts = Counter(sample_joint(cfg, Streams(s))[1] for s in range(1500))
    for level in ("B", "C", "D", "below_D"):
        frac = counts[level] / sum(counts.values())
        assert 0.10 < frac < 0.50, f"{level} at {frac:.3f} is not usable support"
