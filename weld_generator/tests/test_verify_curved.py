"""Phase 6b step 3 — D4 verification arms and weld frames for curved families.

Claims under test: the rediscovery arm recomputes the seam FROM THE PLACED PARTS to
machine precision (and detects a deliberately perturbed part — proving independence
from the drawn curve); the constructed families pass the consistency residual; the
per-point frames are exact and the dihedral genuinely varies along curved seams; and
the verdicts are derived — including the measured correction that an open bore CLEARS
the local cone, so the confinement gate is the semantic `bore_min_diameter_mm` rule.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.constructors import build_d29  # noqa: E402
from weldgen.curves import OffsetCurve, from_parametric  # noqa: E402
from weldgen.d29 import sample_d29_seam  # noqa: E402
from weldgen.verify_curved import (  # noqa: E402
    cone_clear_fractions, containment_residual, curve_distance, curved_seam_set,
    rediscover_seam, seam_verdict,
)


def _built(config, seed=None):
    rng = np.random.default_rng(config if seed is None else seed)
    return build_d29(sample_d29_seam(rng, config=config), rng)


# ------------------------------------------------------------- the rediscovery gate


@pytest.mark.parametrize("config", [2, 3, 4])
def test_rediscovery_matches_the_drawn_curve_to_machine_precision(config):
    for seed in (config, 30 + config):
        built = _built(config, seed)
        assert curve_distance(rediscover_seam(built), built["curve"]) < 1e-9


@pytest.mark.parametrize("config", [2, 3, 4])
def test_rediscovery_detects_a_perturbed_part(config):
    """The arm reads parts, not the record: nudge the stub 0.5 mm and the
    rediscovered curve must move away from the recorded one by about that much."""
    built = _built(config)
    stub = next(p for p in built["parts"] if p.id == "B")
    stub.T_world_part = stub.T_world_part.copy()
    stub.T_world_part[:3, 3] += np.array([0.5, 0.0, 0.0])
    d = curve_distance(rediscover_seam(built), built["curve"])
    assert 0.05 < d < 2.0, d


@pytest.mark.parametrize("config", [5, 6, 7])
def test_constructed_families_pass_the_consistency_residual(config):
    for seed in (config, 30 + config):
        assert containment_residual(_built(config, seed)) < 1e-9


# ------------------------------------------------------------------- weld frames


def test_frames_are_unit_and_dihedral_varies_on_tilted_and_saddle_seams():
    for config, lo_span in ((2, 0.0), (3, 20.0), (4, 40.0)):
        built = _built(config)
        weld = next(s for s in curved_seam_set(built) if s["role"] == "weld")
        for key in ("tangent", "nA", "nB", "approach"):
            assert np.abs(np.linalg.norm(weld[key], axis=1) - 1.0).max() < 1e-9
        # tangent orthogonal to both surface normals - it lies in both surfaces
        assert np.abs(np.einsum("ij,ij->i", weld["tangent"], weld["nA"])).max() < 1e-9
        assert np.abs(np.einsum("ij,ij->i", weld["tangent"],
                                weld["nB"])).max() < 1e-6
        span = float(weld["dihedral_deg"].max() - weld["dihedral_deg"].min())
        assert span >= lo_span, (config, span)
        if config == 2:
            assert np.abs(weld["dihedral_deg"] - 90.0).max() < 1e-6


def test_config6_welds_are_the_offsets_not_the_spine():
    built = _built(6)
    seams = curved_seam_set(built)
    assert [s["role"] for s in seams] == ["weld", "weld"]
    offs = sorted(float(s["curve"].offset_mm) for s in seams
                  if isinstance(s["curve"], OffsetCurve))
    band = built["parts"][1]
    assert offs == sorted([band.offset_lo_mm, band.offset_hi_mm])
    # and both are strictly off the spine - the sweep path is mid-material, not a weld
    assert all(abs(o) > 1.0 for o in offs)


def test_config7_centreline_is_butt_like_with_two_toes():
    built = _built(7)
    seams = curved_seam_set(built)
    roles = [s["role"] for s in seams]
    assert roles == ["weld", "toe", "toe"]
    weld = seams[0]
    assert np.abs(weld["dihedral_deg"] - 180.0).max() < 1e-9
    assert np.abs(weld["approach"] - np.array([0, 0, 1.0])).max() < 1e-9


def test_offset_curve_roundtrips_through_parametric_json():
    built = _built(6)
    seam = curved_seam_set(built)[0]
    d = seam["curve"].to_parametric()
    assert d["kind"] == "offset"
    c2 = from_parametric(d)
    ts = np.linspace(0, seam["curve"].t_period * 0.999, 40)
    assert np.allclose(seam["curve"].point(ts), c2.point(ts), atol=1e-9)


# --------------------------------------------------------------------- verdicts


def test_outer_welds_clear_and_verdicts_are_weldable():
    for config in (2, 5, 6):
        built = _built(config)
        for s in curved_seam_set(built, n=48):
            if s["role"] != "weld":
                continue
            ok, reason, frac = seam_verdict(s, built["parts"])
            assert ok and reason is None and frac == 1.0, (config, reason, frac)


def test_bore_verdict_is_the_cavity_gate_not_the_cone():
    """Measured and kept as the receipt: an open bore CLEARS the local cone - the
    torch-body confinement is what decides, per scene, via bore_min_diameter_mm."""
    seen_confined = seen_weldable = False
    for seed in range(12):
        built = _built(2, seed)
        bore = next(s for s in curved_seam_set(built, n=32) if s["role"] == "bore")
        frac = cone_clear_fractions(bore, built["parts"])
        assert frac > 0.9, "an open bore must clear the local cone"
        ok, reason, _ = seam_verdict(bore, built["parts"])
        if ok:
            seen_weldable = True
            assert bore["cavity_width_mm"] >= 80.0
        else:
            seen_confined = True
            assert reason == "confined_bore"
            assert bore["cavity_width_mm"] < 80.0
    assert seen_confined and seen_weldable, \
        "the gate must cut both ways across sampled radii"


def test_toe_verdict_matches_the_plate_butt_discipline():
    built = _built(7)
    toes = [s for s in curved_seam_set(built) if s["role"] == "toe"]
    for t in toes:
        ok, reason, _ = seam_verdict(t, built["parts"])
        assert not ok and reason == "toe_of_centreline"
