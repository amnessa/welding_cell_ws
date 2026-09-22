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
    assert tool.tip_tool0 == pytest.approx([0.0, 0.0, 0.19])
    assert tool.touch_force_n == pytest.approx(1.5)
    names = [p["name"] for p in tool.primitives]
    assert {"pen", "holder_body", "flange_adapter", "camera_arm"} <= set(names)
    if (PKG / "notebooks" / "T_tcp_to_cam.npy").exists():
        assert "camera" in names and tool.T_tool0_cam is not None
        cam = next(p for p in tool.primitives if p["name"] == "camera")
        assert np.allclose(cam["centre"], tool.T_tool0_cam[:3, 3])
        assert np.linalg.norm(cam["centre"]) < 0.2          # on the bracket, not across the room
    assert "tip" in tool.describe()


def test_primitives_move_rigidly_with_tool0():
    tool = load_tool_model()
    q = np.array([0.3, -1.2, 1.6, -1.9, -1.5, 0.4])
    T = ur5e_fk(q)
    placed = tool.primitives_in(T)
    pen = next(p for p in placed if p["name"] == "pen")
    assert np.allclose(pen["p1"], tool.tip_in(T))               # the pen ends at the tip
    assert np.linalg.norm(pen["p1"] - pen["p0"]) == pytest.approx(0.07)
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
