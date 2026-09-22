"""The collision model - milestone 2 of notes/pen_marking_plan.md. Plain pytest, no ROS.

Claims: the primitive distances are right on cases with known answers; the arm's
capsules sit on the real tubes (tool0 at the end of the chain, upper arm displaced by
the shoulder offset); a pen standing 10 mm above a registered plate is clear while one
driven 5 mm into it is a collision naming the pen and the part; a wrist below the table
is a collision; and the RRT, given the model's validity test, plans around a box with
every waypoint clear.
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

from admittance_control import collision as col  # noqa: E402
from admittance_control.kinematics import ik_solve, rrt_connect, ur5e_fk, _edge_valid  # noqa: E402
from admittance_control.tool_model import load_tool_model  # noqa: E402


def _plate(name="part_A", centre=(0.5, 0.0, 0.10), half=(0.128, 0.125, 0.004), R=None):
    return col.box(name, centre, np.eye(3) if R is None else R, half)


# ---------------------------------------------------------------- distances ----------
def test_segment_segment_and_segment_box_distances():
    assert col._seg_seg_distance(np.zeros(3), np.array([1, 0, 0.]), np.array([0, 1, 0.]),
                                 np.array([1, 1, 0.])) == pytest.approx(1.0)
    assert col._seg_seg_distance(np.zeros(3), np.array([1, 0, 0.]), np.array([2, 0, 0.]),
                                 np.array([3, 0, 0.])) == pytest.approx(1.0)
    bx = col.box("b", [0, 0, 0], np.eye(3), [1, 1, 1])
    assert col._seg_box_distance(np.array([3, 0, 0.]), np.array([3, 5, 0.]), bx) == pytest.approx(2.0, abs=1e-3)
    assert col._seg_box_distance(np.array([-3, 0.5, 0.]), np.array([3, 0.5, 0.]), bx) == pytest.approx(0.0, abs=1e-3)
    cap = col.capsule("c", [0, 0, 1.5], [0, 0, 3.0], 0.25)
    assert col.primitive_distance(cap, bx) == pytest.approx(0.25, abs=1e-3)
    cap2 = col.capsule("c2", [0, 0, 1.1], [0, 0, 3.0], 0.25)
    assert col.primitive_distance(cap2, bx) == 0.0                       # overlapping


def test_box_box_separating_axis():
    a = col.box("a", [0, 0, 0], np.eye(3), [1, 1, 1])
    b = col.box("b", [2.5, 0, 0], np.eye(3), [1, 1, 1])
    assert not col._boxes_intersect(a, b) and col._boxes_intersect(a, b, inflate=0.6)
    c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    rot = col.box("r", [2.3, 0, 0], Rz, [1, 1, 1])                    # a rotated box's corner reaches 1.41
    assert col._boxes_intersect(a, rot)
    far = col.box("f", [2.5, 0, 0], Rz, [1, 1, 1])
    assert not col._boxes_intersect(a, far)


# ---------------------------------------------------------------- the arm ------------
def test_arm_capsules_follow_the_chain():
    q = np.array([0.4, -1.3, 1.5, -1.8, -1.5, 0.2])
    caps = {c["name"]: c for c in col.ur5e_capsules(q)}
    T = ur5e_fk(q)
    assert np.allclose(caps["wrist_2"]["p1"], T[:3, 3], atol=1e-9)     # ends at tool0
    d_upper = np.linalg.norm(caps["upper_arm"]["p1"] - caps["upper_arm"]["p0"])
    assert d_upper == pytest.approx(0.425, abs=1e-9)
    d_fore = np.linalg.norm(caps["forearm"]["p1"] - caps["forearm"]["p0"])
    assert d_fore == pytest.approx(np.hypot(0.3922, 0.1333), abs=1e-9)
    assert caps["base"]["p0"][2] == 0.0


# ---------------------------------------------------------------- the scene ----------
def _q_for_tip(tool, tip, axis, seed):
    T = tool.T_tool0_for_tip(np.asarray(tip), np.asarray(axis), 0.0)
    q = ik_solve(T, seed) if "q_init" not in ik_solve.__code__.co_varnames else ik_solve(T, q_init=seed)
    assert q is not None, "IK failed for the test pose"
    assert np.allclose(tool.tip_in(ur5e_fk(q)), tip, atol=2e-3)
    return q


@pytest.fixture(scope="module")
def tool():
    return load_tool_model()


SEED = np.array([0.0, -1.2, 1.6, -1.9, -1.57, 0.0])


def test_pen_above_plate_is_clear_and_pen_into_plate_is_a_collision(tool):
    plate = _plate()                                            # top face at z = 0.104
    model = col.CollisionModel(tool=tool, scene_boxes=[plate], table_z=-0.05, clearance=0.003)
    q_clear = _q_for_tip(tool, [0.5, 0.0, 0.104 + 0.010], [0, 0, -1], SEED)
    d, a, b = model.min_distance(q_clear)
    assert not model.in_collision(q_clear), model.report(q_clear)
    assert d == pytest.approx(0.010 - 0.0, abs=2e-3) and a == "pen" and b == "part_A"
    q_hit = _q_for_tip(tool, [0.5, 0.0, 0.104 - 0.005], [0, 0, -1], SEED)
    assert model.in_collision(q_hit)
    d, a, b = model.min_distance(q_hit)
    assert d == 0.0 and {a, b} == {"pen", "part_A"}
    assert "COLLISION" in model.report(q_hit)


def test_anything_below_the_table_plane_is_a_collision(tool):
    q = _q_for_tip(tool, [0.5, 0.0, 0.02], [0, 0, -1], SEED)  # pen down, tip at z = 20 mm
    # table plane 15 mm above the tip: the pen crosses it, the holder (48 mm up) does not
    high = col.CollisionModel(tool=tool, scene_boxes=[], table_z=0.035, clearance=0.005)
    assert high.in_collision(q)                                 # the pen crosses the plane
    d, a, b = high.min_distance(q)
    assert d == 0.0 and a == "pen" and b == "table"
    low = col.CollisionModel(tool=tool, scene_boxes=[], table_z=0.0, clearance=0.005)
    assert not low.in_collision(q)                              # 20 mm above it: clear
    assert low.min_distance(q)[0] == pytest.approx(0.020, abs=2e-3)


def test_rrt_with_the_model_plans_around_a_standing_plate(tool):
    # a plate standing across the straight joint-space line between two poses
    # the tip sweeps an arc of radius ~0.49 m about the base between the two poses;
    # the plate stands across it at x = 0.47, narrow enough to clear both end poses
    standing = col.box("part_B", [0.47, 0.0, 0.25], np.eye(3), [0.004, 0.10, 0.15])
    model = col.CollisionModel(tool=tool, scene_boxes=[standing], table_z=-0.05, clearance=0.005)
    q0 = _q_for_tip(tool, [0.45, -0.20, 0.30], [0, 0, -1], SEED)
    q1 = _q_for_tip(tool, [0.45, 0.20, 0.30], [0, 0, -1], SEED)
    assert model.is_valid(q0) and model.is_valid(q1)
    assert not _edge_valid(q0, q1, is_valid=model.is_valid)     # the straight line hits it
    rng = np.random.default_rng(0)
    np.random.seed(0)
    path = rrt_connect(q0, q1, step_size=0.15, max_iter=4000, is_valid=model.is_valid)
    assert path is not None and len(path) >= 2
    for a, b in zip(path[:-1], path[1:]):
        assert _edge_valid(a, b, resolution=0.02, is_valid=model.is_valid)


def test_boxes_from_mode_a_parts_are_in_metres():
    class P:  # a posed slab as mode A returns it (mm)
        id = "A"; dims_mm = (256.0, 249.0, 8.0)
        T_world_part = np.array([[1, 0, 0, -500.0], [0, 1, 0, 150.0], [0, 0, 1, -30.0], [0, 0, 0, 1]])
    bx = col.boxes_from_parts([P()])[0]
    assert bx["name"] == "part_A" and np.allclose(bx["centre"], [-0.5, 0.15, -0.03])
    assert np.allclose(bx["half"], [0.128, 0.1245, 0.004])
