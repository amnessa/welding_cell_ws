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


def test_remove_twist_keeps_the_axis_tilt_and_drops_the_spin():
    """A flat-ended pipe (C1, R2): ICP's spin about the axis is arbitrary (R2 gave 30.8
    deg on the bench); only the tilt of the axis is a correction."""
    from scipy.spatial.transform import Rotation
    from admittance_control.pose_stats import remove_twist
    a = np.array([0.0, 0.0, 1.0])
    R_ref = Rotation.from_euler("xyz", [10, -25, 40], degrees=True).as_matrix()
    R_d = (Rotation.from_rotvec(np.radians(30.8) * a)
           * Rotation.from_rotvec(np.radians(2.0) * np.array([0.6, 0.8, 0.0]))).as_matrix()
    R_new = R_ref @ R_d
    R_keep, swing, twist = remove_twist(R_ref, R_new, a)
    assert swing == pytest.approx(2.0, abs=1e-6) and twist == pytest.approx(30.8, abs=0.05)
    assert np.allclose(R_keep @ a, R_new @ a, atol=1e-9)          # the axis follows ICP
    assert np.allclose(R_keep.T @ R_keep, np.eye(3), atol=1e-12)
    # no spin at all: unchanged
    R_keep2, _, tw2 = remove_twist(R_ref, R_ref @ Rotation.from_rotvec([0.03, 0, 0]).as_matrix(), a)
    assert tw2 == pytest.approx(0.0, abs=1e-6)


def test_only_an_uncut_tube_has_a_symmetry_axis():
    from admittance_control.weldgen_registry import symmetry_axis
    T = np.eye(4).tolist()
    tube = {"primitive": "tube", "T_cad_prim": T, "params": {"r_outer_mm": 31.0}}
    assert np.allclose(symmetry_axis(tube), [0, 0, 1])
    cut = {**tube, "params": {"base_cut": {"kind": "plane"}}}
    assert symmetry_axis(cut) is None and symmetry_axis({"primitive": "slab"}) is None
    assert symmetry_axis(None) is None
