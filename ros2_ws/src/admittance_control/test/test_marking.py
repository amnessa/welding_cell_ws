"""The marking motion - milestone 4 of notes/pen_marking_plan.md. Plain pytest, no ROS.

Claims: the visit order is seam by seam, tack_no ascending; the descent chain moves the
tip along the pen axis from the approach point to the overshoot in small steps on the
locked branch; the contact depth has the documented sign; joint-path timing respects the
speed cap; collision-aware shortcutting never produces an invalid edge; and a full plan
for the reachable T-joint in front of the robot has a transit, a descent and a way home
for every tack.
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

from admittance_control import marking as mk  # noqa: E402
from admittance_control import tack_reach as tr  # noqa: E402
from admittance_control.collision import CollisionModel, boxes_from_parts  # noqa: E402
from admittance_control.kinematics import _edge_valid, ur5e_fk  # noqa: E402
from admittance_control.tool_model import load_tool_model  # noqa: E402

trimesh = pytest.importorskip("trimesh")
from test_tack_reach import _t_joint_in_front  # noqa: E402


def test_visit_order_is_seam_then_tack_no():
    tacks = [{"seam_id": 1, "tack_no": 2, "ok": True}, {"seam_id": 0, "tack_no": 3, "ok": True},
             {"seam_id": 0, "tack_no": 1, "ok": True}, {"seam_id": 1, "tack_no": 1, "ok": False}]
    assert [(t["seam_id"], t["tack_no"]) for t in mk.visit_order(tacks)] == [(0, 1), (0, 3), (1, 2)]


def test_contact_depth_sign():
    point = np.array([0.5, 0.0, 0.02]); axis = np.array([0, 0, -1.0])       # pen going down
    assert mk.contact_depth_m(point, point + 0.004 * np.array([0, 0, 1.0]), axis) == pytest.approx(0.004)   # met 4 mm early
    assert mk.contact_depth_m(point, point - 0.002 * np.array([0, 0, 1.0]), axis) == pytest.approx(-0.002)  # went 2 mm past


def test_joint_path_timing_respects_the_speed_cap():
    path = [np.zeros(6), np.array([0.6, 0, 0, 0, 0, 0]), np.array([0.6, 0.03, 0, 0, 0, 0])]
    timed = mk.time_joint_path(path, v_max=0.3, t_min=0.3)
    assert [round(t, 3) for _, t in timed] == [0.0, 2.0, 2.3]


@pytest.fixture(scope="module")
def scene():
    tool, cfg = load_tool_model(), tr.load_marking_config()
    parts, seams, tacks = _t_joint_in_front()
    model = CollisionModel(tool=tool, scene_boxes=boxes_from_parts(parts), table_z=None,
                           clearance=cfg.clearance_m)
    report = tr.plan_tacks(tacks, tool, model, cfg)
    assert report["all_ok"]
    return tool, cfg, model, report


def test_descent_chain_moves_the_tip_along_the_axis(scene):
    tool, cfg, model, report = scene
    t = report["tacks"][0]
    point, axis = np.asarray(t["point_m"]), np.asarray(t["axis_m"])
    chain = mk.descent_chain(tool, point, axis, np.deg2rad(t["roll_deg"]), np.asarray(t["q_app"]), cfg,
                             overshoot_m=0.003, step_m=0.002)
    assert chain is not None and len(chain) >= 15
    tips = np.array([tool.tip_in(ur5e_fk(q)) for q in chain])
    along = (tips - point) @ axis
    assert along[0] == pytest.approx(-cfg.standoff_m, abs=1e-3)      # starts at the approach point
    assert along[-1] == pytest.approx(0.003, abs=1e-3)               # ends at the overshoot
    assert np.all(np.diff(along) > 0)                                  # monotone along the axis
    lateral = np.linalg.norm((tips - point) - np.outer(along, axis), axis=1)
    assert lateral.max() < 1e-3                                        # on the line
    sig = tuple(report["branch_signature"])
    assert all(tr.same_branch(q, sig) for q in chain)
    timed = mk.time_descent(chain, tool, v_tip=0.02)
    assert timed[-1][1] == pytest.approx((cfg.standoff_m + 0.003) / 0.02, rel=0.05)


def test_shortcut_keeps_every_edge_valid(scene):
    tool, cfg, model, report = scene
    q0 = np.asarray(report["tacks"][0]["q_app"]); q1 = cfg.home_q
    mid = 0.5 * (q0 + q1) + np.array([0.3, 0, 0, 0, 0, 0])
    path = mk.shortcut_path([q0, mid, q1], model.is_valid, resolution=0.01)
    for a, b in zip(path[:-1], path[1:]):
        assert _edge_valid(a, b, resolution=0.01, is_valid=model.is_valid)


def test_full_plan_for_the_t_joint(scene):
    tool, cfg, model, report = scene
    plan = mk.build_marking_plan(report, tool, model, cfg, cfg.home_q)
    assert plan.ok, plan.summary()
    assert len(plan.steps) == 6
    assert [(s.seam_id, s.tack_no) for s in plan.steps] == [(0, 1), (0, 2), (0, 3), (1, 1), (1, 2), (1, 3)]
    for s in plan.steps:
        assert s.transit_ok and s.descent_ok
        assert np.allclose(s.transit[-1], s.q_app)
        assert np.allclose(s.descent[0], s.q_app)
    assert plan.home_path is not None and np.allclose(plan.home_path[-1], cfg.home_q, atol=1e-9)
    d = mk.plan_to_dict(plan)
    import json
    assert json.dumps(d)
