"""Registry entries for the curved parts (notes/curved_seams_plan.md step 1). Plain pytest.

Claims: a weldgen tube (flat, mitred or saddle-cut base) meshed at an arbitrary pose is
fitted back to its own radii, length and cut, and verifies both ways; a rounded-rect
tube comes back as the closed swept_slab of weldgen config 5; a band whose second side
is the first shifted sideways (a wall that changes along it) is refused with the range,
not approximated, and a constant-wall spline band is fitted back; the printed library
parts C1, E2, R2, S3, RR1 verify to well under the D34 budget and the redrawn SP3 comes
back as its drawn spline; and a tube's collision box wraps the tube.
(The seams of curved parts: test_curved_seams.py.)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
WS = PKG.parents[2]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(WS / "weld_generator"))

trimesh = pytest.importorskip("trimesh")
weldgen = pytest.importorskip("weldgen")

from weldgen.curves import rounded_rect_curve  # noqa: E402
from weldgen.geom import SweptSlab, Tube  # noqa: E402

from admittance_control import seam_from_registration as sfr  # noqa: E402
from admittance_control.weldgen_registry import (  # noqa: E402
    derive_entry, derive_extrusion, derive_tube, verify_both_ways)

BUDGET_MM = 0.25


def _pose(rx_deg, ry_deg, t):
    a, b = np.radians(rx_deg), np.radians(ry_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    Ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    T = np.eye(4); T[:3, :3] = Ry @ Rx; T[:3, 3] = t
    return T


def _tube_mesh(**kw):
    return Tube("B", "workpiece", 1, T_world_part=_pose(23.0, -41.0, [12.0, -30.0, 55.0]),
                **kw).mesh()


def _check_tube(mesh, r, wall, length=None):
    e = derive_tube(mesh)
    assert e["primitive"] == "tube", e.get("reason")
    p = e["params"]
    assert p["r_outer_mm"] == pytest.approx(r, abs=0.05)
    assert p["wall_mm"] == pytest.approx(wall, abs=0.05)
    if length is not None:
        assert p["length_mm"] == pytest.approx(length, abs=0.05)
    ver = verify_both_ways(e, mesh, weldgen)
    assert max(ver["cad_to_prim_mm"], ver["prim_to_cad_mm"]) < BUDGET_MM, ver
    assert ver["volume_rel_err"] < 0.01
    return e


def test_flat_tube_is_fitted_back():
    e = _check_tube(_tube_mesh(r_outer_mm=31.0, wall_mm=3.0, length_mm=100.0), 31, 3, 100)
    assert "base_cut" not in e["params"]


def test_mitred_tube_keeps_its_plane_cut():
    n = np.array([np.sin(np.radians(25)), 0.0, -np.cos(np.radians(25))])
    e = _check_tube(_tube_mesh(r_outer_mm=45.0, wall_mm=5.0, length_mm=110.0,
                               base_cut={"kind": "plane", "n_local": n.tolist(), "d": -20.0}),
                    45, 5)
    cut = e["params"]["base_cut"]
    assert cut["kind"] == "plane"
    # the tilt of the cut against the axis survives the re-framing
    assert abs(cut["n_local"][2]) == pytest.approx(np.cos(np.radians(25)), abs=1e-4)


def test_saddle_cut_tube_finds_the_other_pipe():
    # branch r 25 meeting a main pipe r 50 at 70 deg: the S3-on-R2 joint
    m = np.array([np.cos(np.radians(70)), 0.0, np.sin(np.radians(70))])
    cut = {"kind": "cylinder", "point_local": [0.0, 0.0, -52.0],
           "axis_local": [m[2], 0.0, -m[0]], "radius_mm": 50.0}
    e = _check_tube(_tube_mesh(r_outer_mm=25.0, wall_mm=4.0, length_mm=120.0, base_cut=cut),
                    25, 4)
    c = e["params"]["base_cut"]
    assert c["kind"] == "cylinder" and c["radius_mm"] == pytest.approx(50.0, abs=0.05)
    ax = np.asarray(c["axis_local"])
    # the angle between the two pipes' axes is frame-independent
    assert np.degrees(np.arccos(abs(ax[2]) / np.linalg.norm(ax))) == pytest.approx(70.0, abs=0.1)


def test_rounded_rect_tube_is_a_closed_swept_slab():
    spine = rounded_rect_curve([0, 0, 0], [1, 0, 0], [0, 1, 0], 80.0, 60.0, 10.0)
    mesh = SweptSlab("B", "workpiece", 1, spine, 0.0, 4.0, 0.0, 90.0,
                     _pose(-17.0, 33.0, [5.0, 40.0, -8.0])).mesh()
    e = derive_extrusion(mesh)
    assert e["primitive"] == "swept_slab", e.get("reason")
    assert sorted(e["rect_mm"]) == pytest.approx([60.0, 80.0], abs=0.05)
    assert e["corner_r_mm"] == pytest.approx(10.0, abs=0.1)
    assert e["params"]["offset_hi_mm"] == pytest.approx(4.0, abs=0.05)
    assert e["params"]["z1_mm"] == pytest.approx(90.0, abs=1e-6)
    ver = verify_both_ways(e, mesh, weldgen)
    assert max(ver["cad_to_prim_mm"], ver["prim_to_cad_mm"]) < BUDGET_MM, ver


def _strip_extrusion(left, right, height):
    """A watertight prism over the strip between two matched 2D polylines (x, z),
    extruded along -y like the printed SP3."""
    n = len(left)
    ring = np.vstack([left, right])                          # 0..n-1 left, n..2n-1 right
    v = np.vstack([np.column_stack([ring[:, 0], np.zeros(2 * n), ring[:, 1]]),
                   np.column_stack([ring[:, 0], -height * np.ones(2 * n), ring[:, 1]])])
    f = []
    for i in range(n - 1):
        a, b, c, d = i, i + 1, n + i, n + i + 1
        f += [[a, c, b], [b, c, d]]                          # top cap
        f += [[a + 2 * n, b + 2 * n, c + 2 * n], [b + 2 * n, d + 2 * n, c + 2 * n]]
    loop = list(range(n)) + list(range(2 * n - 1, n - 1, -1))
    for k in range(len(loop)):
        a, b = loop[k], loop[(k + 1) % len(loop)]
        f += [[a, a + 2 * n, b], [b, a + 2 * n, b + 2 * n]]
    m = trimesh.Trimesh(v, np.asarray(f), process=True)
    trimesh.repair.fix_normals(m)
    return m


def test_a_sideways_shifted_band_is_refused_with_its_wall_range():
    # SP3's construction: one S-curve and the same curve shifted 5 mm in x
    z = np.linspace(-110.0, 110.0, 111)
    x = 57.7 * np.sin(np.pi * z / 110.0)
    mesh = _strip_extrusion(np.column_stack([x, z]), np.column_stack([x + 5.0, z]), 80.0)
    assert mesh.is_watertight
    e = derive_entry(mesh)
    assert e["primitive"] is None
    lo, hi = e["wall_mm_range"]
    assert hi == pytest.approx(5.0, abs=0.1) and lo < 3.5


LIBRARY = {"C1": ("tube", None), "E2": ("tube", "plane"), "R2": ("tube", None),
           "S3": ("tube", "cylinder"), "RR1": ("swept_slab", None)}


@pytest.mark.parametrize("name", sorted(LIBRARY))
def test_printed_library_parts_verify(name):
    ply = PKG / "models" / f"{name}.ply"
    if not ply.exists():
        pytest.skip("library mesh not present")
    mesh = trimesh.load(str(ply), force="mesh")
    e = derive_entry(mesh)
    prim, cut = LIBRARY[name]
    assert e["primitive"] == prim, e.get("reason")
    if prim == "tube":
        assert (e["params"].get("base_cut") or {}).get("kind") == cut
    ver = verify_both_ways(e, mesh, weldgen)
    assert max(ver["cad_to_prim_mm"], ver["prim_to_cad_mm"]) < 0.1, ver


def test_printed_sp3_is_its_drawn_spline():
    """The redrawn SP3 (2026-10-09): a true 5 mm offset of the S-curve. The fit
    recovers the drawn cubic from the mesh to micrometres."""
    ply = PKG / "models" / "SP3.ply"
    if not ply.exists():
        pytest.skip("library mesh not present")
    mesh = trimesh.load(str(ply), force="mesh")
    e = derive_entry(mesh)
    assert e["primitive"] == "swept_slab" and e["shape"] == "open_band", e.get("reason")
    assert e["fit_resid_mm"] < 0.01
    p = e["params"]
    assert p["offset_hi_mm"] - p["offset_lo_mm"] == pytest.approx(5.0, abs=0.01)
    assert p["z1_mm"] - p["z0_mm"] == pytest.approx(80.0, abs=1e-6)
    ver = verify_both_ways(e, mesh, weldgen)
    assert max(ver["cad_to_prim_mm"], ver["prim_to_cad_mm"]) < 0.1, ver


def test_a_constant_wall_spline_band_is_fitted_back():
    # weldgen config 6 style: a cubic spine, band +/- 3 mm, 60 tall, skewed pose
    from weldgen.curves import BSplineCurve
    spine = BSplineCurve(np.array([[0.0, -100.0, 0.0], [-120.0, -30.0, 0.0],
                                   [120.0, 30.0, 0.0], [0.0, 100.0, 0.0]]))
    mesh = SweptSlab("B", "workpiece", 1, spine, -3.0, 3.0, 0.0, 60.0,
                     _pose(12.0, -20.0, [4.0, 9.0, -3.0])).mesh()
    e = derive_entry(mesh)
    assert e["primitive"] == "swept_slab", e.get("reason")
    assert e["params"]["offset_hi_mm"] - e["params"]["offset_lo_mm"] == pytest.approx(6.0, abs=0.05)
    ver = verify_both_ways(e, mesh, weldgen)
    assert max(ver["cad_to_prim_mm"], ver["prim_to_cad_mm"]) < BUDGET_MM, ver


def test_a_tube_collision_box_wraps_the_tube_not_its_base_frame():
    from admittance_control.collision import boxes_from_parts
    reg = {"parts": {"pipe": {"primitive": "tube", "T_cad_prim": np.eye(4).tolist(),
                              "params": {"r_outer_mm": 31.0, "wall_mm": 3.0,
                                         "length_mm": 100.0}}}}
    (b,) = boxes_from_parts(sfr.posed_parts([("pipe.ply", np.eye(4))], reg))
    assert b["centre"] == pytest.approx([0.0, 0.0, 0.05], abs=1e-6)       # metres
    assert b["half"] == pytest.approx([0.031, 0.031, 0.05], abs=1e-4)
