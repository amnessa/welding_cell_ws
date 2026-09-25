"""Robust mean of a set of rigid poses, and how spread they were. numpy only.

The ICP tracker's pose on a STATIONARY part wandered by 5 mm std / 24 mm range and up
to 6 deg (pose_jitter_probe, 2026-09-25): taking the last tracked pose at `save_object`
was a single draw from that distribution. `robust_pose_mean` gives the estimate the
window supports and, as importantly, its spread - which goes into `assembly.json` next
to the pose it qualifies.

    T_mean, stats = robust_pose_mean(list_of_4x4)
    stats = {"n": 30, "n_used": 27, "std_mm": [..3], "std_deg": 0.4, "range_mm": [..3],
             "max_deg": 1.1}

Method: per-axis median translation and a chordal-mean rotation (SVD of the summed
rotation matrices) over the INLIERS; inliers are the poses within `k` MADs of the
median translation and within `rot_gate_deg` of the chordal mean of all poses.
Jumps (an ICP tick that settled in another local minimum) are what the range shows
and the median discards; the std reported is over the inliers.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np


def chordal_mean(Rs: Sequence[np.ndarray]) -> np.ndarray:
    M = np.sum([np.asarray(R, float) for R in Rs], axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def rotation_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    c = np.clip((np.trace(Ra.T @ Rb) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def robust_pose_mean(poses: Sequence[np.ndarray], k_mad: float = 3.5,
                     rot_gate_deg: float = 3.0) -> tuple[np.ndarray, dict[str, Any]]:
    Ts = [np.asarray(T, float).reshape(4, 4) for T in poses]
    if not Ts:
        raise ValueError("no poses")
    t = np.array([T[:3, 3] for T in Ts])
    Rs = [T[:3, :3] for T in Ts]
    med = np.median(t, axis=0)
    mad = np.median(np.abs(t - med), axis=0) * 1.4826 + 1e-6
    R_all = chordal_mean(Rs)
    ang = np.array([rotation_angle_deg(R_all, R) for R in Rs])
    inlier = (np.abs(t - med) <= k_mad * mad).all(axis=1) & (ang <= rot_gate_deg)
    if inlier.sum() < max(3, len(Ts) // 3):
        inlier = np.ones(len(Ts), bool)                 # too few agree: keep everything
    t_in = t[inlier]
    R_in = [R for R, ok in zip(Rs, inlier) if ok]
    R_mean = chordal_mean(R_in)
    T_mean = np.eye(4)
    T_mean[:3, :3] = R_mean
    T_mean[:3, 3] = np.median(t_in, axis=0)
    ang_in = np.array([rotation_angle_deg(R_mean, R) for R in R_in])
    stats = {"n": int(len(Ts)), "n_used": int(inlier.sum()),
             "std_mm": [float(v) for v in t_in.std(axis=0) * 1000.0],
             "range_mm": [float(v) for v in (t.max(axis=0) - t.min(axis=0)) * 1000.0],
             "std_deg": float(ang_in.std()), "max_deg": float(ang.max())}
    return T_mean, stats
