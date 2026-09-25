"""The pen tool model - milestone 1 of notes/pen_marking_plan.md. Plain pytest, no ROS.

Claims: the package config loads with the pen tip where the user measured it and the
camera where the hand-eye calibration puts it (never typed by hand); primitives move
rigidly with tool0 under FK; and the tool0 pose solved for a tip target puts the tip
there with the pen along the requested axis, for any roll.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.tool_model import load_tool_model, transform_primitive  # noqa: E402
from admittance_control.kinematics import ur5e_fk  # noqa: E402


def test_package_config_loads_with_tip_and_calibrated_camera():
    tool = load_tool_model()
    assert tool.tip_tool0 == pytest.approx([0.00135, -0.00017, 0.18173])
    assert tool.touch_force_n == pytest.approx(1.5)
    names = [p["name"] for p in tool.primitives]
    assert {"pen", "holder_body", "flange_adapter", "camera_arm"} <= set(names)
    cam = next(p for p in tool.primitives if p["name"] == "camera_body")
    assert np.allclose(cam["centre"], [0.0, 0.09, 0.045]) and cam["half"][0] >= 0.045
    if (PKG / "notebooks" / "T_tcp_to_cam.npy").exists():
        assert tool.T_tool0_cam is not None
        # the calibrated optical origin must be on the tool, not across the room; how far
        # it sits from the bench-measured body centre is the open TCP-offset question
        # (2026-09-24: 62 mm, mostly along x), so that distance is reported, not asserted
        gap = np.linalg.norm(tool.T_tool0_cam[:3, 3] - cam["centre"])
        assert np.linalg.norm(tool.T_tool0_cam[:3, 3]) < 0.2, gap
        print(f"calibrated optical origin vs bench body centre: {gap * 1000:.0f} mm")
    assert "tip" in tool.describe()


def test_primitives_move_rigidly_with_tool0():
    tool = load_tool_model()
    q = np.array([0.3, -1.2, 1.6, -1.9, -1.5, 0.4])
    T = ur5e_fk(q)
    placed = tool.primitives_in(T)
    pen = next(p for p in placed if p["name"] == "pen")
    # the round cap of the pen capsule ends AT the tip: p1 is one radius short of it
    assert np.allclose(pen["p1"] + pen["radius"] * T[:3, 2], tool.tip_in(T), atol=1e-12)
    box = next(p for p in placed if p["type"] == "box")
    assert np.allclose(box["R"] @ box["R"].T, np.eye(3), atol=1e-9)


def test_tool0_pose_for_a_tip_target_puts_the_tip_there_along_the_axis():
    tool = load_tool_model()
    tip = np.array([0.45, -0.12, 0.08])
    axis = np.array([-0.7, 0.1, -0.7]); axis /= np.linalg.norm(axis)    # pen pointing down-and-in
    for roll in (0.0, 0.7, 2.9):
        T = tool.T_tool0_for_tip(tip, axis, roll)
        assert np.allclose(tool.tip_in(T), tip, atol=1e-12)
        assert np.allclose(T[:3, 2], axis, atol=1e-12)
        assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)
        assert np.linalg.det(T[:3, :3]) == pytest.approx(1.0)


def test_transform_primitive_rejects_unknown_types():
    with pytest.raises(ValueError):
        transform_primitive({"type": "sphere"}, np.eye(4))


def test_touch_off_recovers_an_off_axis_tip():
    from admittance_control.tool_model import solve_tip_offset
    d_true = np.array([0.012, -0.007, 0.191])                 # a pen 14 mm off the flange axis
    p_true = np.array([0.55, 0.10, 0.02])
    rng = np.random.default_rng(3)
    poses = []
    for _ in range(5):
        # a random orientation, the tool0 origin placed so that the tip is on p_true
        q = rng.normal(size=4); q /= np.linalg.norm(q); w, x, y, z = q
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p_true - R @ d_true + rng.normal(scale=2e-4, size=3)
        poses.append(T)
    d, p, rms = solve_tip_offset(poses)
    assert np.allclose(d, d_true, atol=1e-3) and np.allclose(p, p_true, atol=1e-3)
    assert rms < 1.0
    with pytest.raises(ValueError):
        solve_tip_offset(poses[:2])
