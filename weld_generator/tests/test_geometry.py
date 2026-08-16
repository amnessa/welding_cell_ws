"""Geometric correctness of the Phase 1 T-joint constructor.

These assert against *closed forms derived independently of the implementation*, which is
the only kind of geometry test worth having — comparing the code to itself proves nothing.
"""

from __future__ import annotations

import numpy as np
import pytest

from weldgen.joints import JointSpec, build_t_joint

REF = dict(L_A=232.0, W_A=120.0, t_A=8.4, L_B=232.0, H_B=90.0, t_B=8.4)


def spec(**kw) -> JointSpec:
    p = dict(REF, root_gap_mm=1.1, linear_misalignment_mm=0.0,
             angular_misalignment_deg=0.0, included_angle_deg=90.0)
    p.update(kw)
    return JointSpec(**p)


def test_two_fillets():
    """A T-joint has two fillets, one per side of the standing plate (D5)."""
    _, seams = build_t_joint(spec(), np.eye(4))
    assert len(seams) == 2
    assert {s.face_pair for s in seams} == {("A:+w", "B:-w"), ("A:+w", "B:+w")}
    assert all(s.weldable for s in seams)


def test_objects_watertight_union_is_not():
    """D21: each object watertight; the union is genuinely disjoint at g > 0."""
    slabs, _ = build_t_joint(spec(root_gap_mm=1.1), np.eye(4))
    for s in slabs:
        m = s.mesh()
        assert m.is_watertight and m.is_winding_consistent
    a, b = (s.mesh() for s in slabs)
    # Disjoint: B sits entirely above A's top face.
    assert b.bounds[0][2] >= a.bounds[1][2] - 1e-9


def test_seam_length_is_the_overlap():
    """The seam clips to where both faces have support along the seam direction."""
    _, seams = build_t_joint(spec(L_A=232.0, L_B=180.0), np.eye(4))
    assert all(s.length_mm == pytest.approx(180.0, abs=1e-6) for s in seams)


@pytest.mark.parametrize("g", [0.0, 0.5, 1.1, 3.0])
def test_d19_offsets_at_right_angle(g):
    """SCHEMA.md §1.3: at alpha = 90 the three curves are offset by 0, g/2, g."""
    _, seams = build_t_joint(spec(root_gap_mm=g), np.eye(4))
    for s in seams:
        assert np.linalg.norm(s.root_p0 - s.p0) == pytest.approx(g, abs=1e-9)
        assert np.linalg.norm(s.gapmid_p0 - s.p0) == pytest.approx(g / 2.0, abs=1e-9)


def test_d19_offsets_depend_on_alpha():
    """The 0 / g/2 / g closed form does NOT survive D18 — that is why root and gap_mid
    ship as derived arrays rather than as a published formula."""
    _, seams = build_t_joint(spec(included_angle_deg=120.0), np.eye(4))
    off = np.linalg.norm(seams[0].root_p0 - seams[0].p0)
    assert off > 1.1 + 1e-6, "tilting the plate must move the root off the g offset"


@pytest.mark.parametrize("alpha", [60.0, 70.0, 90.0, 120.0])
def test_fillet_dihedrals_are_supplementary(alpha):
    """D18: `included_angle_deg` is NOT `dihedral_deg`.

    A T-joint at included angle alpha has one fillet at alpha and one at 180 - alpha, so
    a single scene carries two different dihedrals for one included angle.
    """
    _, seams = build_t_joint(spec(included_angle_deg=alpha), np.eye(4))
    d = sorted(s.dihedral_deg for s in seams)
    assert d[0] + d[1] == pytest.approx(180.0, abs=1e-6)
    assert min(alpha, 180 - alpha) == pytest.approx(d[0], abs=1e-6)


@pytest.mark.parametrize("h,beta", [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (1.0, 1.5)])
def test_seam_position_matches_closed_form(h, beta):
    """Independent derivation: the B:+w fillet lies at y = h + g*tan(theta)."""
    g = 1.1
    s = spec(root_gap_mm=g, linear_misalignment_mm=h, angular_misalignment_deg=beta)
    _, seams = build_t_joint(s, np.eye(4))
    fillet = next(x for x in seams if x.face_pair == ("A:+w", "B:+w"))
    expected = h + g * np.tan(np.deg2rad(s.tilt_deg("T")))
    assert fillet.p0[1] == pytest.approx(expected, abs=1e-9)


def test_approach_escapes_the_material():
    """The bisector must leave the seam without re-entering either solid (D4)."""
    slabs, seams = build_t_joint(spec(), np.eye(4))
    meshes = [s.mesh() for s in slabs]
    for s in seams:
        probe = s.p0 + 0.5 * (s.p1 - s.p0) + s.approach * 25.0
        for m in meshes:
            assert not m.contains([probe])[0]


def test_world_pose_is_a_rigid_transform():
    """Placing the assembly must not change any intrinsic quantity."""
    from weldgen.geom import rot_z, translate
    T = translate(123.0, -45.0, 0.0) @ rot_z(37.0)
    _, at_origin = build_t_joint(spec(), np.eye(4))
    _, posed = build_t_joint(spec(), T)
    for a, b in zip(at_origin, posed):
        assert a.length_mm == pytest.approx(b.length_mm, abs=1e-6)
        assert a.dihedral_deg == pytest.approx(b.dihedral_deg, abs=1e-6)
        assert np.linalg.norm(a.root_p0 - a.p0) == pytest.approx(
            np.linalg.norm(b.root_p0 - b.p0), abs=1e-9)
