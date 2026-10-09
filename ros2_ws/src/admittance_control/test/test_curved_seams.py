"""Curved seams at registered poses (notes/curved_seams_plan.md step 2). Plain pytest.

Claims: a pipe on a plate yields ONE closed weld seam - the plate plane ∩ the outer
wall, a circle of 2πr - with the end-to-plate gap as fit-up, the approach at 45° to the
plate all round, and the bore kept as a confined negative; it is found on whichever end
the pipe stands; a mitred pipe sitting on its cut yields the ellipse with the dihedral
sweeping 65-115°; a pipe hanging off the plate edge is refused, one floating just beyond
the pose tolerance is reported rejected with its fit-up, one far beyond it has no seam; a branch on a run pipe yields the saddle with the gap
along the branch axis, also for the printed S3 on R2; a rounded-rect tube yields its
outer outline (weld) and inner one (bore); an open band yields both side fillets, with
per-point normals that stay matched to their points after the seam is re-oriented;
tacks on a closed seam are an even set, each with the approach at its own point; and a
pair with no rule is reported, not dropped.
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

from admittance_control import seam_from_registration as sfr  # noqa: E402

I4 = np.eye(4).tolist()
SN = np.sin(np.radians(25.0)); CS = np.cos(np.radians(25.0))
REG = {"parts": {
    "plate": {"primitive": "slab", "dims_mm": [256.0, 249.0, 8.0], "T_cad_prim": I4},
    "pipe": {"primitive": "tube", "T_cad_prim": I4,
             "params": {"r_outer_mm": 31.0, "wall_mm": 3.0, "length_mm": 100.0}},
    "mitre": {"primitive": "tube", "T_cad_prim": I4,
              "params": {"r_outer_mm": 45.0, "wall_mm": 5.0, "length_mm": 110.0,
                         "base_cut": {"kind": "plane", "n_local": [SN, 0.0, -CS], "d": 0.0}}},
    "run": {"primitive": "tube", "T_cad_prim": I4,
            "params": {"r_outer_mm": 50.0, "wall_mm": 6.0, "length_mm": 180.0}},
    # branch standing on "run" (axis along x through (0, 0, -60) in the branch frame)
    "branch": {"primitive": "tube", "T_cad_prim": I4,
               "params": {"r_outer_mm": 25.0, "wall_mm": 4.0, "length_mm": 120.0,
                          "base_cut": {"kind": "cylinder", "point_local": [0.0, 0.0, -60.0],
                                       "axis_local": [1.0, 0.0, 0.0], "radius_mm": 50.0}}},
    "rr": {"primitive": "swept_slab", "T_cad_prim": I4,
           "params": {"spine": rounded_rect_curve([0, 0, 0], [1, 0, 0], [0, 1, 0],
                                                  64.0, 64.0, 12.0).to_parametric(),
                      "offset_lo_mm": 0.0, "offset_hi_mm": 3.0, "z0_mm": 0.0, "z1_mm": 100.0}},
    # an arc stiffener, 5 mm thick, 80 tall: arc R 150 about (0, -150), 60..120 deg
    "band": {"primitive": "swept_slab", "T_cad_prim": I4,
             "params": {"spine": {"kind": "arc", "center_mm": [0.0, -150.0, 0.0],
                                  "u_dir": [1.0, 0.0, 0.0], "v_dir": [0.0, 1.0, 0.0],
                                  "radius_mm": 150.0, "t0": np.pi / 3, "t1": 2 * np.pi / 3},
                        "offset_lo_mm": -2.5, "offset_hi_mm": 2.5, "z0_mm": 0.0, "z1_mm": 80.0}},
}}


def _pose(t_mm, R=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = np.asarray(t_mm, float) / 1000.0
    return T


def _rx(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


PLATE = ("plate.ply", _pose([0.0, 0.0, -4.0]))          # top face at z = 0


def _seams(*objs, reg=REG):
    parts = sfr.posed_parts(list(objs), reg)
    return parts, sfr.compute_seams(parts)


def _by_role(seams, role):
    return [s for s in seams if s.get("role") == role]


def test_pipe_on_plate_is_one_closed_circle_with_its_gap():
    parts, seams = _seams(PLATE, ("pipe.ply", _pose([10.0, -5.0, 2.0], _ry(3.0))))
    (w,) = _by_role(seams, "weld")
    assert w["weldable"] and w["closed"] and w["member_end"] == "-w"
    assert w["length_mm"] == pytest.approx(2 * np.pi * 31.0, abs=0.5)
    lo, hi = w["fitup_mm"]["B"]                              # tilted 3 deg: 2 -/+ r sin 3
    assert lo == pytest.approx(2.0 - 31.0 * np.sin(np.radians(3.0)), abs=0.1)
    assert hi == pytest.approx(2.0 + 31.0 * np.sin(np.radians(3.0)), abs=0.1)
    a = np.asarray(w["approach_per_point"])
    tilt = np.degrees(np.arccos(a[:, 2]))                    # off the plate normal
    assert tilt.min() > 43.0 and tilt.max() < 47.0
    pts = np.asarray(w["polyline_mm"])
    assert np.abs(pts[:, 2]).max() < 1e-6                    # on the plate plane
    (b,) = _by_role(seams, "bore")
    assert not b["weldable"] and b["reject_reason"] == "confined_bore"
    assert "gap" in sfr.summarize(seams)


def test_pipe_standing_on_its_top_cap():
    _, seams = _seams(PLATE, ("pipe.ply", _pose([0.0, 0.0, 101.0], _rx(180.0))))
    (w,) = _by_role(seams, "weld")
    assert w["weldable"] and w["member_end"] == "+w"
    assert w["fitup_mm"]["B"] == pytest.approx([1.0, 1.0], abs=1e-6)


def test_mitred_pipe_on_its_cut_is_the_ellipse():
    # rotate the cut's normal (sin25, 0, -cos25) onto -z: the cut lies on the plate
    _, seams = _seams(PLATE, ("mitre.ply", _pose([0.0, 0.0, 1.0], _ry(25.0))))
    (w,) = _by_role(seams, "weld")
    a, b = 45.0 / CS, 45.0
    h = ((a - b) / (a + b)) ** 2
    ramanujan = np.pi * (a + b) * (1 + 3 * h / (10 + np.sqrt(4 - 3 * h)))
    assert w["weldable"] and w["length_mm"] == pytest.approx(ramanujan, abs=0.5)
    assert w["dihedral_deg_range"] == pytest.approx([65.0, 115.0], abs=0.1)
    assert w["fitup_mm"]["B"] == pytest.approx([1.0 / CS, 1.0 / CS], abs=1e-6)


def test_pipe_off_the_edge_is_refused_and_a_floating_one_has_no_seam():
    _, seams = _seams(PLATE, ("pipe.ply", _pose([120.0, 0.0, 0.5])))
    w = _by_role(seams, "weld")[0]
    assert not w["weldable"] and w["reject_reason"] == "off_plate_edge"
    _, seams = _seams(PLATE, ("pipe.ply", _pose([0.0, 0.0, 30.0])))
    assert seams == []
    # past the pose tolerance but within twice it: shown, rejected, with the fit-up
    _, seams = _seams(PLATE, ("pipe.ply", _pose([0.0, 0.0, 15.0])))
    w = _by_role(seams, "weld")[0]
    assert not w["weldable"] and w["reject_reason"].startswith("fitup_beyond_pose_tol")
    assert w["fitup_mm"]["B"] == pytest.approx([15.0, 15.0], abs=1e-6)


def test_branch_on_run_pipe_is_the_saddle_with_the_gap_along_the_branch():
    run = ("run.ply", _pose([-90.0, 0.0, -60.0], _ry(90.0)))     # axis along x
    branch = ("branch.ply", _pose([0.0, 0.0, 1.5]))             # 1.5 mm proud
    _, seams = _seams(run, branch)
    (w,) = _by_role(seams, "weld")
    assert w["weldable"] and w["closed"] and w["face_pair"] == ["A:lateral+", "B:lateral+"]
    assert w["fitup_mm"]["B"] == pytest.approx([1.5, 1.5], abs=1e-6)
    # 90 deg at the crown; at the flanks the run's normal tilts asin(r/R) = 30 deg away
    # from the branch on both sides, so the joint opens to 120 deg there
    assert w["dihedral_deg_range"] == pytest.approx([90.0, 120.0], abs=0.1)
    pts = np.asarray(w["polyline_mm"])                        # on both outer walls
    assert np.hypot(pts[:, 1], pts[:, 2] + 60.0) == pytest.approx(50.0, abs=1e-6)
    assert np.hypot(pts[:, 0], pts[:, 1]) == pytest.approx(25.0, abs=1e-6)


def test_printed_s3_on_r2_as_designed():
    from admittance_control.weldgen_registry import load_registry
    path = PKG / "models" / "weldgen_objects.json"
    reg = load_registry(path)
    if "S3" not in reg["parts"] or "R2" not in reg["parts"]:
        pytest.skip("S3 / R2 not in the registry")
    # R2's axis on the cut cylinder's axis (CAD z through (0, 1.193)), spanning the joint
    _, seams = _seams(("R2.ply", _pose([0.0, 1.193, -55.0])), ("S3.ply", np.eye(4)), reg=reg)
    (w,) = _by_role(seams, "weld")
    assert w["weldable"] and w["closed"]
    lo, hi = w["fitup_mm"]["B"]                # the cut is r 51.04 on a 50 mm pipe
    assert 1.0 < lo < hi < 1.4


def test_rounded_rect_tube_outer_outline_is_the_weld():
    _, seams = _seams(PLATE, ("rr.ply", _pose([0.0, 0.0, 1.5])))
    (w,) = _by_role(seams, "weld")
    (b,) = _by_role(seams, "bore")
    assert w["weldable"] and w["face_pair"][1] == "B:-w"
    assert w["length_mm"] == pytest.approx(4 * 64.0 - (8 - 2 * np.pi) * 12.0, abs=0.5)
    assert b["face_pair"][1] == "B:+w" and b["cavity_width_mm"] == pytest.approx(58.0, abs=0.1)
    assert not b["weldable"]


def test_open_band_has_two_fillets_with_frames_matched_to_points():
    _, seams = _seams(PLATE, ("band.ply", _pose([0.0, 0.0, 0.8])))
    w = _by_role(seams, "weld")
    assert len(w) == 2 and all(s["weldable"] and not s["closed"] for s in w)
    lengths = sorted(s["length_mm"] for s in w)
    assert lengths == pytest.approx([147.5 * np.pi / 3, 152.5 * np.pi / 3], abs=0.3)
    for s in w:
        pts = np.asarray(s["polyline_mm"]); nb = np.asarray(s["n_b_per_point"])
        radial = pts - [0.0, -150.0, 0.0]; radial[:, 2] = 0.0
        radial /= np.linalg.norm(radial, axis=1, keepdims=True)
        sign = np.sign(np.einsum("ij,ij->i", nb, radial))
        assert np.all(sign == sign[0])                       # every normal at its point
        r = np.linalg.norm(pts[:, :2] - [0.0, -150.0], axis=1)
        assert np.all(sign[0] * (r - 150.0) > 0)             # and pointing off the band


def test_printed_sp3_on_the_plate_has_two_fillets():
    from admittance_control.weldgen_registry import load_registry
    reg = load_registry(PKG / "models" / "weldgen_objects.json")
    if reg["parts"].get("SP3", {}).get("primitive") != "swept_slab":
        pytest.skip("SP3 not registered as a band")
    reg = {"parts": {**reg["parts"], "plate": REG["parts"]["plate"]}}
    # SP3 stands along CAD +y from y = 0: turn +y onto +z, 1 mm above the plate
    _, seams = _seams(PLATE, ("SP3.ply", _pose([0.0, 0.0, 1.0], _rx(90.0))), reg=reg)
    w = _by_role(seams, "weld")
    assert len(w) == 2 and all(s["weldable"] and not s["closed"] for s in w), sfr.summarize(seams)
    for s in w:
        assert s["fitup_mm"]["B"] == pytest.approx([1.0, 1.0], abs=1e-6)
        assert 300.0 < s["length_mm"] < 365.0


def test_tacks_on_a_closed_seam_carry_their_own_approach():
    parts, seams = _seams(PLATE, ("pipe.ply", _pose([0.0, 0.0, 1.0])))
    tacks = sfr.compute_tacks(parts, seams)["tacks"]
    assert len(tacks) >= 4 and len(tacks) % 2 == 0
    for t in tacks:
        p = np.asarray(t["point_mm"]); a = np.asarray(t["approach"])
        assert np.hypot(p[0], p[1]) == pytest.approx(31.0, abs=0.1)
        assert np.linalg.norm(a) == pytest.approx(1.0)
        radial = np.array([p[0], p[1], 0.0]) / np.hypot(p[0], p[1])
        assert a @ radial == pytest.approx(np.sqrt(0.5), abs=0.02)   # 45 deg, outward
    assert len({tuple(np.round(t["approach"], 3)) for t in tacks}) == len(tacks)


def test_a_pair_without_a_rule_is_reported():
    _, seams = _seams(("pipe.ply", _pose([0.0, 0.0, 0.0])), ("band.ply", _pose([0.0, 0.0, 0.0])))
    assert [s["seam_class"] for s in seams] == ["curved"]
    assert "no seam rule" in seams[0]["reject_reason"]
