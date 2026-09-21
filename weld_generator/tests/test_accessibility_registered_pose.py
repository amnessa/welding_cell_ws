"""A T-joint at an arbitrary (floating-precision) rigid pose keeps both fillets.

Regression for `_mutually_visible`: when the candidate line lies on the face that
generated it, the between-samples collapse onto the line itself, which at a non-exact
pose sits ~1e-7 mm inside the plate and was counted as material. The generator's exact
poses never triggered it; registered poses do on every run.
"""

from __future__ import annotations

import numpy as np

from weldgen.accessibility import enumerate_candidates
from weldgen.geom import Slab


def _t_joint(T_scene: np.ndarray, L=200.0, W=100.0, t=8.0, gap=0.0):
    Rx = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
    T_b = np.eye(4); T_b[:3, :3] = Rx; T_b[:3, 3] = [0, 0, t / 2 + gap + W / 2]
    A = Slab("A", "workpiece", 0, (L, W, t), T_scene @ np.eye(4))
    B = Slab("B", "workpiece", 1, (L, W, t), T_scene @ T_b)
    return [A, B]


def _random_rigid(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = rng.uniform(-800, 800, 3)
    return T


def test_t_joint_keeps_both_fillets_at_arbitrary_rigid_poses():
    for seed in range(12):
        parts = _t_joint(_random_rigid(seed))
        weld = [c for c in enumerate_candidates(parts) if c.weldable]
        assert len(weld) == 2, f"seed {seed}: {[(c.face_pair, c.reject_reason) for c in weld]}"
        assert all(c.seam_class == "fillet" and abs(c.length_mm - 200) < 1e-6 for c in weld)


def test_identity_pose_unchanged():
    weld = [c for c in enumerate_candidates(_t_joint(np.eye(4))) if c.weldable]
    assert len(weld) == 2
