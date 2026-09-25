"""Robust pose mean for save_object. Plain pytest, no ROS."""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.pose_stats import chordal_mean, robust_pose_mean  # noqa: E402


def _rot(axis, deg):
    a = np.asarray(axis, float); a /= np.linalg.norm(a); th = np.deg2rad(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def test_mean_recovers_the_true_pose_and_discards_jumps():
    rng = np.random.default_rng(1)
    R_true = _rot([0.3, 1, 0.2], 40.0); t_true = np.array([0.5, 0.1, 0.03])
    poses = []
    for i in range(40):
        T = np.eye(4)
        T[:3, :3] = _rot(rng.normal(size=3), rng.normal(scale=0.3)) @ R_true      # 0.3 deg noise
        T[:3, 3] = t_true + rng.normal(scale=0.0015, size=3)                    # 1.5 mm noise
        if i % 10 == 0:                                                         # four jumps
            T[:3, 3] += np.array([0.02, -0.015, 0.0])
            T[:3, :3] = _rot([0, 0, 1], 6.0) @ T[:3, :3]
        poses.append(T)
    T_mean, st = robust_pose_mean(poses)
    assert np.linalg.norm(T_mean[:3, 3] - t_true) < 1e-3
    c = np.clip((np.trace(R_true.T @ T_mean[:3, :3]) - 1) / 2, -1, 1)
    assert np.degrees(np.arccos(c)) < 0.3
    assert st["n"] == 40 and st["n_used"] <= 36                                 # the jumps are out
    assert max(st["std_mm"]) < 2.5 and st["range_mm"][0] > 15.0                # spread reported honestly
    assert st["max_deg"] > 5.0


def test_single_pose_and_chordal_mean_are_sane():
    T = np.eye(4); T[:3, 3] = [1, 2, 3]
    T_mean, st = robust_pose_mean([T, T, T])
    assert np.allclose(T_mean, T) and st["n_used"] == 3
    R = chordal_mean([_rot([0, 0, 1], 10), _rot([0, 0, 1], -10)])
    assert np.allclose(R, np.eye(3), atol=1e-9)
    with pytest.raises(ValueError):
        robust_pose_mean([])
