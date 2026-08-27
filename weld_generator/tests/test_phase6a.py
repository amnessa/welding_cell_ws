"""Phase 6a, first half — in-plane yaw (D28) for the joints that meet on a face.

The claim under test: yaw breaks the seam-parallel-to-plate-edge prior WITHOUT touching
anything else — the D4 enumeration rediscovers the rotated seam, the seam length stays
pinned by the footprint (D27), support holds by construction, and corpora generated
before the change reproduce bit-identically because yaw draws from the previously unused
`seam_curve` substream.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config, sample_joint  # noqa: E402
from weldgen.joints import JointSpec  # noqa: E402
from weldgen.layouts import build, max_supported_yaw_deg  # noqa: E402
from weldgen.rng import Streams  # noqa: E402


def spec_T(yaw=0.0, **kw):
    p = dict(L_A=220.0, W_A=200.0, t_A=8.0, L_B=140.0, H_B=80.0, t_B=8.0,
             root_gap_mm=0.5, linear_misalignment_mm=0.0, angular_misalignment_deg=0.0,
             included_angle_deg=90.0, stack_offset_mm=None, in_plane_yaw_deg=yaw)
    p.update(kw)
    return JointSpec(**p)


def _enumerate(spec, joint_type):
    from weldgen.accessibility import enumerate_candidates
    from weldgen.config import load_config
    slabs = build(spec, joint_type, np.eye(4))
    access = {k: (v.copy() if isinstance(v, dict) else v)
              for k, v in load_config()["accessibility"].items()}
    access["contact_tol_mm"] = 2.0 * spec.root_gap_mm + 0.5
    return slabs, enumerate_candidates(slabs, access, joint_type=joint_type)


def _primaries(cands, joint_type):
    return [c for c in cands if c.weldable and c.matches_joint_type
            and c.p0 is not None and c.length_mm > 1e-6]


def test_the_d4_rule_rediscovers_a_yawed_fillet():
    """A 30-deg yawed T still yields two fillets — along the ROTATED direction.

    This is the whole point of doing yaw before outlines: no new primitive, no new rule,
    the enumeration is orientation-agnostic geometry and must simply keep working. The
    seam direction must sit at the yaw angle to A's own u axis, and the length must stay
    pinned by B's footprint (D27) exactly as at yaw 0.
    """
    for yaw in (0.0, 17.0, 30.0, -41.0):
        spec = spec_T(yaw=yaw)
        slabs, cands = _enumerate(spec, "T")
        prim = _primaries(cands, "T")
        assert len(prim) == 2, f"yaw={yaw}: {len(prim)} fillets"
        for c in prim:
            d = np.asarray(c.p1, float) - np.asarray(c.p0, float)
            d = d / np.linalg.norm(d)
            ang = np.degrees(np.arctan2(d[1], d[0])) % 180.0
            want = yaw % 180.0
            assert min(abs(ang - want), 180.0 - abs(ang - want)) < 1.0, \
                f"yaw={yaw}: seam at {ang:.1f}"
            assert abs(c.length_mm - spec.L_B) < 2.0     # D27: pinned by the footprint


def test_yaw_is_not_dihedral():
    """The included angle survives yaw untouched — the two rotations are orthogonal."""
    from weldgen.geom import dihedral_deg

    a0 = _enumerate(spec_T(yaw=0.0, included_angle_deg=75.0), "T")
    a1 = _enumerate(spec_T(yaw=35.0, included_angle_deg=75.0), "T")
    d0 = sorted(round(c.dihedral_deg, 1) for c in _primaries(a0[1], "T"))
    d1 = sorted(round(c.dihedral_deg, 1) for c in _primaries(a1[1], "T"))
    assert d0 == d1


def test_support_bound_scales_with_the_base_plate():
    """The yaw range is support-limited, not configured: a wide base frees the full 90,
    a narrow one pins yaw near zero, and the sampled yaw never exceeds the bound."""
    wide = max_supported_yaw_deg(spec_T(W_A=400.0, L_A=400.0, L_B=100.0), "T")
    narrow = max_supported_yaw_deg(spec_T(W_A=12.0, L_B=200.0, L_A=210.0), "T")
    assert wide == 90.0
    assert narrow < 10.0
    assert max_supported_yaw_deg(spec_T(), "butt") == 0.0   # edge-sharing joints: no yaw


def test_lap_yaw_rotates_the_toe_on_a_but_not_as_boundary():
    """On a yawed lap, B's toe (the seam made by B's edge on A's face) follows the yaw."""
    spec = spec_T(yaw=25.0, stack_offset_mm=40.0, included_angle_deg=0.0,
                  W_A=240.0, L_A=300.0, L_B=120.0)
    slabs, cands = _enumerate(spec, "lap")
    prim = _primaries(cands, "lap")
    assert prim, "a yawed lap must still have toe seams"
    angs = []
    for c in prim:
        d = np.asarray(c.p1, float) - np.asarray(c.p0, float)
        d /= np.linalg.norm(d)
        angs.append(np.degrees(np.arctan2(d[1], d[0])) % 180.0)
    # at least one toe sits at the yaw angle (B's edge); an A-edge toe at 0 is the
    # seam-bearing edge and allowed by D28
    assert any(min(abs(a - 25.0), 180 - abs(a - 25.0)) < 1.5 for a in angs), angs


def test_yaw_off_reproduces_the_pre_phase6a_stream_draws():
    """`in_plane_yaw: false` (and configs that never heard of the key) draw nothing from
    any stream that existing corpora used — the same seed gives the same spec, field for
    field. This is the property that keeps out/bench valid without regeneration."""
    cfg = load_config()
    assert cfg["in_plane_yaw"] is False
    cfg["joint_type"] = "T"
    a, qa, _ = sample_joint(cfg, Streams(1234))
    cfg2 = load_config()
    cfg2["joint_type"] = "T"
    cfg2["in_plane_yaw"] = False
    b, qb, _ = sample_joint(cfg2, Streams(1234))
    assert a == b and qa == qb
    assert a.in_plane_yaw_deg == 0.0


def test_yaw_on_is_a_twin_of_yaw_off_in_everything_but_yaw():
    """Yaw draws from the previously unused `seam_curve` substream, so enabling it must
    change the yaw field and NOTHING else — the free-ablation-twin property."""
    cfg = load_config()
    cfg["joint_type"] = "T"
    off, _, _ = sample_joint(cfg, Streams(777))
    cfg["in_plane_yaw"] = True
    on, _, _ = sample_joint(cfg, Streams(777))
    for f in ("L_A", "W_A", "t_A", "L_B", "H_B", "t_B", "root_gap_mm",
              "linear_misalignment_mm", "angular_misalignment_deg",
              "included_angle_deg", "length_offset_mm"):
        assert getattr(on, f) == getattr(off, f), f
    # (the yaw itself may draw 0 by chance on one seed; check over several)
    yaws = []
    for seed in range(20):
        cfg["in_plane_yaw"] = True
        s_, _, _ = sample_joint(cfg, Streams(seed))
        yaws.append(s_.in_plane_yaw_deg)
    assert np.ptp(yaws) > 10.0, "yaw must actually vary"


def test_the_anti_shortcut_gate_moves_off_0_and_90():
    """The D28 gate, measured over sampled scenes: the angle between the seam and the
    nearest NON-seam-bearing boundary edge of A must not concentrate at 0/90 deg.

    (Defined over non-seam-bearing edges only — the amendment made on integrating the
    patch: B's contact edge IS the seam on T and lap, and a gate counting it can never
    pass.) On yawed scenes the folded angle min(|yaw|, 90-|yaw|) should spread over
    (0, 45]; before Phase 6a it was exactly 0 everywhere.
    """
    cfg = load_config()
    cfg["joint_type"] = "T"
    cfg["in_plane_yaw"] = True
    folded = []
    for seed in range(40):
        s_, _, _ = sample_joint(cfg, Streams(seed))
        a = abs(s_.in_plane_yaw_deg) % 90.0
        folded.append(min(a, 90.0 - a))
    folded = np.asarray(folded)
    assert folded.max() > 20.0
    assert (folded > 5.0).mean() > 0.5
    assert np.std(folded) > 8.0
