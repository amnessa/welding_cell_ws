"""Tack reachability - milestone 3 of notes/pen_marking_plan.md. Plain pytest, no ROS.

Claims: /joint_states (alphabetical) is reordered to UR order and the scan home's branch
signature is the elbow-up one; IK on the branch never returns another branch; a T-joint
placed in front of the robot gets every tack an approach pose, a tack pose with the pen
touching (pen gap ~0) and a clear descent, one roll per seam, every solution on the
locked branch; the same joint far behind the robot is reported unreachable, not
guessed; and the report serialises.
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
from admittance_control import tack_reach as tr  # noqa: E402
from admittance_control.collision import CollisionModel, boxes_from_parts  # noqa: E402
from admittance_control.kinematics import ur5e_fk  # noqa: E402
from admittance_control.tool_model import load_tool_model  # noqa: E402
from admittance_control.weldgen_registry import derive_box  # noqa: E402

trimesh = pytest.importorskip("trimesh")

JS_NAMES = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
JS_POS = [-1.5552871227264404, -1.4443891805461426, -0.34471112886537725,
          -1.5529543210617085, 1.9432356357574463, -0.8597829977618616]


def test_joint_states_reorder_and_home_branch():
    q = tr.joint_state_to_ur_order(JS_NAMES, JS_POS)
    assert q[0] == pytest.approx(-0.3447, abs=1e-3) and q[2] == pytest.approx(-1.5553, abs=1e-3)
    cfg = tr.load_marking_config()
    assert np.allclose(cfg.home_q, q)
    assert cfg.branch_signature == (-1, -1, 1)               # lift -, elbow -, wrist_2 +


def _t_joint_in_front(centre_xy=(0.55, 0.10), z_top=0.02, L=200.0, W=100.0, t=4.0):
    """A T-joint (mode A parts, mm) on the bench in front of the robot: base plate flat
    with its top at `z_top`, web standing across x."""
    reg = {"parts": {"base": derive_box(trimesh.creation.box(extents=[L, W, t])) | {"source": "base.ply"},
                     "web": derive_box(trimesh.creation.box(extents=[L, W, t])) | {"source": "web.ply"}}}
    cx, cy = (v * 1000 for v in centre_xy)
    T_a = np.eye(4); T_a[:3, 3] = [cx, cy, z_top * 1000 - t / 2]
    Rx = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
    T_b = np.eye(4); T_b[:3, :3] = Rx; T_b[:3, 3] = [cx, cy, z_top * 1000 + W / 2]
    parts = sfr.posed_parts([("base.ply", T_a), ("web.ply", T_b)], reg, pose_units="mm")
    seams = sfr.compute_seams(parts, sfr.runtime_access(parts, pose_tol_mm=4.0))
    tacks = sfr.compute_tacks(parts, seams)["tacks"]
    return parts, seams, tacks


@pytest.fixture(scope="module")
def setup():
    tool, cfg = load_tool_model(), tr.load_marking_config()
    # the planning logic is under test, not the clearance policy: with the calibrated tip
    # (180.9 mm, 2026-09-28) the holder body clears the plates of a square T by
    # 60.9*sin45 - 42 = 1.1 mm at best, so the config's 3 mm makes every 90 deg fillet unreachable
    cfg.clearance_m = 0.0008
    return tool, cfg


def test_t_joint_in_front_is_reachable_on_the_branch_with_one_roll_per_seam(setup):
    tool, cfg = setup
    parts, seams, tacks = _t_joint_in_front()
    assert len(tacks) == 6
    model = CollisionModel(tool=tool, scene_boxes=boxes_from_parts(parts), table_z=None,
                           clearance=cfg.clearance_m)
    report = tr.plan_tacks(tacks, tool, model, cfg)
    assert report["all_ok"], tr.format_report(report)
    rolls = {s["seam_id"]: s["roll_deg"] for s in report["seams"]}
    sig = tuple(report["branch_signature"])
    for t in report["tacks"]:
        assert t["ok"] and t["roll_deg"] == rolls[t["seam_id"]]
        q_app, q_tack = np.asarray(t["q_app"]), np.asarray(t["q_tack"])
        assert tr.same_branch(q_app, sig) and tr.same_branch(q_tack, sig)
        # the tip really is on the tack point, the pen really is touching
        tip = tool.tip_in(ur5e_fk(q_tack))
        assert np.linalg.norm(tip - np.asarray(t["point_m"])) < 2e-3
        assert t["pen_gap_m"] < 2e-3
        assert t["clearance_app_m"] >= cfg.clearance_m and t["clearance_tack_m"] >= cfg.clearance_m
        assert np.degrees(t["joint_step_from_prev_rad"]) < np.degrees(cfg.max_joint_step_rad)
    assert json.dumps(report)
    assert "ALL REACHABLE" in tr.format_report(report)


def test_joint_far_behind_the_robot_is_reported_unreachable(setup):
    tool, cfg = setup
    parts, seams, tacks = _t_joint_in_front(centre_xy=(-1.4, 0.0))     # 1.4 m away: out of reach
    model = CollisionModel(tool=tool, scene_boxes=boxes_from_parts(parts), table_z=None,
                           clearance=cfg.clearance_m)
    report = tr.plan_tacks(tacks, tool, model, cfg)
    assert not report["all_ok"]
    assert all(not s["ok"] for s in report["seams"])
    assert "UNREACHABLE" in tr.format_report(report)


def test_solutions_on_the_other_branch_are_rejected(setup):
    tool, cfg = setup
    # a pose the other elbow branch reaches easily: seed only from an elbow-down guess
    T = tool.T_tool0_for_tip(np.array([0.55, 0.10, 0.05]), np.array([0, 0, -1.0]), 0.0)
    elbow_down = np.array([0.0, -1.0, 1.6, -2.2, -1.57, 0.0])
    q = tr.solve_on_branch(T, [elbow_down], cfg, n_random=0)
    assert q is None or tr.same_branch(q, cfg.branch_signature)


def test_wrist_3_stays_in_its_cable_window():
    """2026-10-16, SP3: tack-to-tack continuity wound wrist_3 to -202 and -287 deg from
    home, and the camera cable's pull read as a 1.5 N contact 84 mm above the part. Every
    pose has a 2*pi-equivalent within +-180 deg of home - the same tool pose."""
    cfg = tr.load_marking_config()
    home = cfg.home_q
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = home + rng.uniform(-2.5, 2.5, 6)
        q[5] = rng.uniform(-2 * np.pi, 2 * np.pi)
        ref = home + rng.uniform(-2.5, 2.5, 6)
        ref[5] = home[5] + rng.uniform(-np.pi, np.pi)
        out = tr.to_cable_window(q, ref, cfg)
        assert tr.in_cable_window(out, cfg)
        assert np.allclose(ur5e_fk(out), ur5e_fk(q), atol=1e-9)
    # the SP3 case: from -160 deg, plain continuity took tack 4 to -287; inside: +73
    q4, ref = home.copy(), home.copy()
    q4[5], ref[5] = home[5] + np.deg2rad(-287.0), home[5] + np.deg2rad(-160.0)
    assert np.degrees(tr.to_cable_window(q4, ref, cfg)[5] - home[5]) == pytest.approx(73.0)
    # a window over 180 deg keeps continuity where it can: -200 is inside +-210
    cfg.wrist3_window_deg = 210.0
    q4[5] = home[5] + np.deg2rad(-200.0)
    assert np.degrees(tr.to_cable_window(q4, ref, cfg)[5] - home[5]) == pytest.approx(-200.0)


def test_the_roll_choice_counts_wrist_3_as_the_transit_turns_it():
    """Wrist_3 stays in its window, so a transit turns it the long way inside rather than
    the short way across the window's edge: the cost has to say so."""
    a, b = np.zeros(6), np.zeros(6)
    a[5], b[5] = -3.0, 3.0                         # 6 rad apart, 0.28 the short way round
    assert tr._joint_cost(a, b) == pytest.approx(0.5 * 6.0)
    a[0], b[0], a[5], b[5] = -3.0, 3.0, 0.0, 0.0   # the other joints still go the short way
    assert tr._joint_cost(a, b) == pytest.approx(2.0 * (2 * np.pi - 6.0))
