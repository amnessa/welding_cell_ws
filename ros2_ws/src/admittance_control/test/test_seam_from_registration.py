"""Mode A - the seam computed from registered poses. Plain pytest, no ROS.

Claims: a box mesh from the library yields a slab whose frame puts every CAD vertex on
the box; a synthetic T joint posed in METRES yields the two fillets the D4 rule
constructs, on the true intersection line, with unit approach axes off the standing
plate; the same joint at REGISTERED poses - the standing plate floating above the
base, tilted, or driven into it, as the bench's ICP actually returned it - still yields
both fillets on the plane-intersection line with the gap / penetration reported as
fit-up; a pose tolerance wider than the sheet does not pair a plate's own underside
with the standing plate; a lapping sheet thinner than the pose tolerance is reported
undecidable rather than guessed; the two saved bench assemblies yield their T fillets;
and a part without a registry entry is a RegistryError, not a silent skip.
"""

from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
WS = PKG.parents[2]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(WS / "weld_generator"))

from admittance_control import seam_from_registration as sfr  # noqa: E402
from admittance_control.weldgen_registry import derive_box, verify_entry  # noqa: E402

trimesh = pytest.importorskip("trimesh")


def _box(L, W, t):
    return trimesh.creation.box(extents=[L, W, t])


def test_library_plate_is_a_slab_with_its_corners_on_the_box():
    ply = PKG / "models" / "plate.ply"
    if not ply.exists():
        pytest.skip("library mesh not present")
    mesh = trimesh.load(str(ply), force="mesh")
    e = derive_box(mesh)
    assert e["primitive"] == "slab"
    assert sorted(e["dims_mm"], reverse=True) == pytest.approx([150.0, 100.0, 4.0])
    assert e["max_dev_mm"] < 0.01
    import weldgen
    assert verify_entry(e, mesh, weldgen) < 0.05      # the primitive IS the CAD


def test_a_bent_or_composite_mesh_is_unsupported_not_guessed():
    a = _box(100, 50, 4); b = _box(100, 50, 4); b.apply_translation([0, 27, 25])
    e = derive_box(trimesh.util.concatenate([a, b]))
    assert e["primitive"] is None and e["reason"]


def _t_joint(gap_mm=1.0, L=200.0, W=100.0, t=4.0):
    """Two slabs in a registry, posed in metres like assembly.json: A flat, B standing on
    A's top face with a root gap, its bottom edge along A's mid-line."""
    reg = {"parts": {"base": derive_box(_box(L, W, t)) | {"source": "base.ply"},
                     "web": derive_box(_box(L, W, t)) | {"source": "web.ply"}}}
    Rx = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)      # +90 deg about x
    T_b = np.eye(4); T_b[:3, :3] = Rx; T_b[:3, 3] = [0, 0, (t / 2 + gap_mm + W / 2) / 1000]
    return reg, [("base.ply", np.eye(4)), ("web.ply", T_b)]


def test_t_joint_yields_two_fillets_on_the_intersection_line():
    reg, objs = _t_joint()
    parts = sfr.posed_parts(objs, reg)
    seams = sfr.compute_seams(parts, sfr.runtime_access(parts, contact_tol_mm=4.0))
    weld = [s for s in seams if s["weldable"]]
    assert len(weld) == 2 and all(s["seam_class"] == "fillet" for s in weld)
    for s in weld:
        p = np.asarray(s["polyline_mm"])
        assert abs(s["length_mm"] - 200.0) < 1.0                 # the full shared edge
        assert np.allclose(p[:, 2], 2.0, atol=1e-6)               # on A's top face
        assert np.abs(p[:, 1]).max() < 2.5                        # at the web's faces (t/2)
        a = np.asarray(s["approach"]); assert abs(np.linalg.norm(a) - 1) < 1e-9
        assert a[2] > 0.5                                         # up and away from A
    assert any(not s["weldable"] for s in seams)                  # negatives are kept


def test_poses_in_metres_become_millimetres():
    reg, objs = _t_joint()
    parts = sfr.posed_parts(objs, reg)
    assert abs(parts[1].T_world_part[2, 3] - (2 + 1 + 50)) < 1e-9


def _t_joint_posed(dz_mm=0.0, tilt_deg=0.0, lean_deg=0.0, L=200.0, W=100.0, t=8.0):
    """The T with the standing plate displaced as a registration would: `dz_mm` along
    its own height (> 0 floats, < 0 penetrates), `tilt_deg` about the base normal's
    cross axis (one end lifted), `lean_deg` off vertical about the seam axis."""
    reg, objs = _t_joint(gap_mm=0.0, L=L, W=W, t=t)
    T_b = objs[1][1].copy()
    ry = np.deg2rad(tilt_deg); rx = np.deg2rad(lean_deg)
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    T_b[:3, :3] = Rx @ Ry @ T_b[:3, :3]
    T_b[:3, 3] = Rx @ Ry @ T_b[:3, 3] + np.array([0, 0, dz_mm / 1000.0])
    return reg, [objs[0], ("web.ply", T_b)]


@pytest.mark.parametrize("dz,tilt,lean,lo,hi", [
    (6.0, 0.0, 0.0, 6.0, 6.0),          # floating 6 mm
    (-9.0, 0.0, 0.0, -9.0, -9.0),       # driven 9 mm into the base
    (4.0, 1.5, 6.0, 1.0, 9.0),          # floating, one end lifted, leaning
])
def test_registered_t_joint_keeps_both_fillets_and_reports_fitup(dz, tilt, lean, lo, hi):
    reg, objs = _t_joint_posed(dz, tilt, lean)
    parts = sfr.posed_parts(objs, reg)
    seams = sfr.compute_seams(parts, sfr.runtime_access(parts, pose_tol_mm=10.0))
    weld = [s for s in seams if s["weldable"]]
    assert [s["seam_class"] for s in weld] == ["fillet", "fillet"], sfr.summarize(seams)
    for s in weld:
        assert s["length_mm"] > 190.0
        p = np.asarray(s["polyline_mm"])
        assert np.allclose(p[:, 2], 4.0, atol=1e-6)               # on A's top PLANE (t/2)
        f = sfr.abutting_fitup(s)
        assert f is not None and f[0] == "B"
        assert lo - 1.5 <= f[1] <= f[2] <= hi + 1.5
    # nothing on the underside (z = -t/2): the pose tolerance (10) exceeds the sheet (8)
    assert not any(s["weldable"] and np.asarray(s["polyline_mm"])[:, 2].mean() < 0
                   for s in seams if s["polyline_mm"])


def test_wide_pose_tolerance_does_not_pair_the_underside_on_thin_sheet():
    reg, objs = _t_joint(t=2.0)
    parts = sfr.posed_parts(objs, reg)
    access = sfr.runtime_access(parts, pose_tol_mm=10.0)
    assert access["pose_tol_mm"] == 10.0                          # not capped by the sheet
    seams = sfr.compute_seams(parts, access)
    weld = [s for s in seams if s["weldable"]]
    assert len(weld) == 2
    assert all(np.allclose(np.asarray(s["polyline_mm"])[:, 2], 1.0) for s in weld)   # top, z=+t/2


def test_lap_on_sheet_thinner_than_the_pose_tolerance_is_undecidable_not_guessed():
    t = 2.0
    reg = {"parts": {"base": derive_box(_box(200, 100, t)) | {"source": "base.ply"},
                     "top": derive_box(_box(200, 100, t)) | {"source": "top.ply"}}}
    T = np.eye(4); T[:3, 3] = [0, 0.060, t / 1000]                # lying on the base, 40 mm overlap
    parts = sfr.posed_parts([("base.ply", np.eye(4)), ("top.ply", T)], reg)
    seams = sfr.compute_seams(parts, sfr.runtime_access(parts, pose_tol_mm=10.0))
    toes = [s for s in seams if s["seam_class"] == "lap_toe" and s["length_mm"] > 100]
    assert toes and all(s["reject_reason"] == "member_within_pose_tol" for s in toes)
    # with the pose bound below the sheet the same lap is decided
    seams = sfr.compute_seams(parts, sfr.runtime_access(parts, pose_tol_mm=1.5))
    assert [s["seam_class"] for s in seams if s["weldable"]] == ["lap_toe", "lap_toe"]


@pytest.mark.parametrize("assembly", [
    "scripts/sam6d_results/assembly.json",
    "scripts/foundationpose_results/previous_assemblies/20260812-113902/assembly.json",
])
def test_bench_assemblies_yield_their_t_fillets(assembly):
    path = PKG / assembly
    reg_path = PKG / "models" / "weldgen_objects.json"
    if not path.exists() or not reg_path.exists():
        pytest.skip("bench assembly not present")
    from admittance_control.weldgen_registry import load_registry
    a = json.loads(path.read_text())
    objs = [(o["model"], np.asarray(o["pose_static"], float).reshape(4, 4)) for o in a["objects"]]
    parts = sfr.posed_parts(objs, load_registry(str(reg_path)))
    seams = sfr.compute_seams(parts, sfr.runtime_access(parts, pose_tol_mm=10.0))
    weld = [s for s in seams if s["weldable"]]
    assert len(weld) == 2 and all(s["seam_class"] == "fillet" and s["length_mm"] > 240 for s in weld)
    for s in weld:
        f = sfr.abutting_fitup(s)
        assert f is not None and abs(f[1]) <= 11.0                # the bench's 8-11 mm


def test_missing_registry_entry_is_an_error_naming_the_part():
    reg, objs = _t_joint()
    reg["parts"]["web"] = {"primitive": None, "reason": "bent plate"}
    with pytest.raises(sfr.RegistryError, match="web.*bent plate"):
        sfr.posed_parts(objs, reg)


def test_points_for_rviz_are_in_metres_and_coloured_per_seam():
    reg, objs = _t_joint()
    seams = sfr.compute_seams(sfr.posed_parts(objs, reg))
    xyz, rgb, idx = sfr.seams_points_m(seams)
    assert xyz.shape[1] == 3 and np.abs(xyz).max() < 0.2 and len(set(idx.tolist())) == 2
    assert json.dumps(seams)                                      # serialisable as written
