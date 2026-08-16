"""Viewing helpers. Not part of the dataset contract — nothing here is stored.

Scenes are placed at a **sampled world pose** (SCHEMA.md §1.1): random yaw and
translation, deliberately, because a joint pinned to the world axes would leak the answer.
That is correct for the dataset and inconvenient for plotting: slicing on world X and
plotting world (y, z) is not a cross-section of anything.

`assembly_frame` gives the frame the joint was *constructed* in, which is the one worth
looking at.

Note this is NOT `T_world_joint`. That frame's +Z is the torch approach bisector, so a
90 degree fillet appears tilted 45 degrees in it. Fine for a tool pose, poor for a
drawing.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np


def assembly_frame(scene: Mapping[str, Any]) -> np.ndarray:
    """4x4 world -> assembly transform, for cross-sections and figures.

    In the returned frame:
        +X   along seam 0
        +Z   along part A's `+w` outward normal, so A's top face is the plane z = 0
        +Y   completes the right-handed set; part B stands at y >= 0
        origin at seam 0's start point

    So a T-joint always draws the same way regardless of how the scene was posed:
    the base plate fills ``z in [-t_A, 0]`` and the standing plate ``z >= g``.
    """
    A = next(o for o in scene["objects"] if o["id"] == "A")
    R_A = np.asarray(A["T_world_part"], dtype=float)[:3, :3]
    z = R_A @ np.array([0.0, 0.0, 1.0])          # A:+w outward normal

    s0 = scene["seams"][0]
    p0 = np.asarray(s0["parametric"]["p0_mm"], dtype=float)
    p1 = np.asarray(s0["parametric"]["p1_mm"], dtype=float)
    x = p1 - p0
    x /= np.linalg.norm(x)

    # Re-orthogonalise against z in case the seam is not exactly perpendicular to it
    # (it is for a slab T-joint, but Phase 6 curved parts will not be so tidy).
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)

    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z]).T     # world -> assembly is the transpose
    T[:3, 3] = -T[:3, :3] @ p0
    return T


def to_assembly(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a world -> assembly transform to an (N,3) array."""
    P = np.atleast_2d(np.asarray(points, dtype=float))
    return P @ T[:3, :3].T + T[:3, 3]


def cross_section(xyz_local: np.ndarray, half_width_mm: float = 3.0,
                  at_x: float | None = None) -> np.ndarray:
    """Boolean mask selecting a slab perpendicular to the seam.

    `at_x` defaults to the middle of the seam's extent.
    """
    x = xyz_local[:, 0]
    centre = float(np.median(x)) if at_x is None else float(at_x)
    return np.abs(x - centre) < float(half_width_mm)
