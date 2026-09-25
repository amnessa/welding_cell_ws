"""Robust pose mean for save_object. Plain pytest, no ROS."""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.pose_stats import chordal_mean, robust_pose_mean, stationary_tail  # noqa: E402


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


def test_stationary_tail_excludes_the_place_the_part_came_from():
    rng = np.random.default_rng(2)
    def at(t, deg, n):
        out = []
        for _ in range(n):
            T = np.eye(4); T[:3, :3] = _rot([0, 0, 1], deg + rng.normal(scale=0.3))
            T[:3, 3] = np.asarray(t) + rng.normal(scale=0.0015, size=3); out.append(T)
        return out
    before = at([0.5, 0.1, 0.03], 0.0, 15)                      # first resting place
    moving = [np.eye(4) for _ in range(4)]
    for k, T in enumerate(moving):                               # sliding 6 cm in 4 ticks
        T[:3, 3] = [0.5 + 0.015 * (k + 1), 0.1, 0.03]
    after = at([0.56, 0.1, 0.03], 0.0, 12)                       # second resting place
    tail = stationary_tail(before + moving + after)
    assert 10 <= len(tail) <= 13                                 # the 12 at rest (+ maybe the last slide tick)
    T_mean, st = robust_pose_mean(tail)
    assert np.linalg.norm(T_mean[:3, 3] - [0.56, 0.1, 0.03]) < 1.5e-3
    # a single jump tick in the middle of a rest does not end the tail
    jump = at([0.56, 0.1, 0.03], 0.0, 1); jump[0][:3, 3] += [0.02, 0, 0]
    tail2 = stationary_tail(after[:6] + jump + after[6:])
    assert len(tail2) >= 12
    assert stationary_tail([]) == []
