"""Camera pose sampling — PARAMETERS.md §4.1.

The camera looks at the assembly from a spherical shell: standoff, elevation, azimuth and
a roll about the view axis. Elevation is measured **from the world XY plane** (SCHEMA.md
§1.1), and its lower bound is the dataset's difficulty axis rather than a nuisance — below
about 20 deg the standing plate of a T-joint blocks its own seam, which is exactly the
condition the occlusion metric is there to measure. A sampler that politely avoids it
produces a dataset with no difficulty axis at all.

Poses are stored in the **OpenCV optical convention**: +X right, +Y down, +Z forward along
the view axis. Stated rather than implied, because ROS REP-103 body frames (+X forward,
+Z up) are one silent transpose away and the error is invisible until a figure is wrong.
"""

from __future__ import annotations

import numpy as np

WORLD_UP = np.array([0.0, 0.0, 1.0])


def look_at(eye: np.ndarray, target: np.ndarray, roll_deg: float = 0.0) -> np.ndarray:
    """`T_world_cam` for a camera at `eye` looking at `target`, OpenCV convention.

    The basis is built from `forward` and world up. In OpenCV, x cross y = z, so with
    `d` (down) and `f` (forward) known, `r = d x f` is the one choice that keeps the
    frame right-handed: (d x f) x d = f.
    """
    eye = np.asarray(eye, dtype=float)
    f = np.asarray(target, dtype=float) - eye
    f /= np.linalg.norm(f)

    up = WORLD_UP
    if abs(float(f @ up)) > 0.999:                    # looking straight down: pick another
        up = np.array([1.0, 0.0, 0.0])
    d = -(up - float(up @ f) * f)                     # world up, projected out of f, flipped
    d /= np.linalg.norm(d)
    r = np.cross(d, f)

    if roll_deg:
        a = np.deg2rad(roll_deg)
        r, d = np.cos(a) * r + np.sin(a) * d, -np.sin(a) * r + np.cos(a) * d

    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = r, d, f, eye
    return T


def sample_pose(target: np.ndarray, standoff_mm: float, elevation_deg: float,
                azimuth_deg: float, roll_deg: float) -> np.ndarray:
    """Place the camera on the shell of radius `standoff_mm` about `target`."""
    el, az = np.deg2rad(elevation_deg), np.deg2rad(azimuth_deg)
    offset = np.array([np.cos(el) * np.cos(az),
                       np.cos(el) * np.sin(az),
                       np.sin(el)]) * float(standoff_mm)
    return look_at(np.asarray(target, dtype=float) + offset, target, roll_deg)


def standoff_for_framing(extent_mm: float, framing_frac: float, focal_px: float,
                         width: int, height: int) -> float:
    """Range at which `extent_mm` projects to `framing_frac` of the SHORT image side.

    Sampling standoff uniformly over 300-1200 mm frames the assembly comfortably almost
    every time, so `in_frame` was near-constant and the only graded source of lost seam
    visibility never fired. Sampling the *framing fraction* instead puts the assembly
    partly out of frame on purpose, which is also what an eye-in-hand camera on a robot
    actually does - it frequently cannot get a whole 400 mm seam into one view.

    Measured against the short side, not the diagonal: a seam at an arbitrary orientation
    is clipped by whichever edge it reaches first, and the diagonal flatters the fit badly
    enough that `framing_frac > 1` would almost never clip anything.
    """
    short = float(min(int(width), int(height)))
    return float(focal_px) * float(extent_mm) / (max(float(framing_frac), 1e-6) * short)


def intrinsics(focal_px: float, width: int, height: int) -> list[list[float]]:
    """Pinhole `K` with the principal point at the image centre.

    No distortion and no jitter in tier 1 (PARAMETERS.md §4.1) - stated in `camera.model`
    rather than left for a reader to assume.
    """
    return [[float(focal_px), 0.0, width / 2.0],
            [0.0, float(focal_px), height / 2.0],
            [0.0, 0.0, 1.0]]


def project(xyz_world: np.ndarray, T_world_cam: np.ndarray, K: np.ndarray
            ) -> tuple[np.ndarray, np.ndarray]:
    """Project world points into the image. Returns `(uv, z_cam)` in px and mm.

    `z_cam` is the depth along the optical axis, which is what the stereo noise model is a
    function of - not the Euclidean range to the camera centre.
    """
    R, t = T_world_cam[:3, :3], T_world_cam[:3, 3]
    cam = (np.asarray(xyz_world, dtype=float) - t) @ R     # world -> cam is R transpose
    z = cam[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = (cam[:, :2] / z[:, None]) * np.array([K[0][0], K[1][1]]) \
            + np.array([K[0][2], K[1][2]])
    return uv, z


def in_frustum(uv: np.ndarray, z_cam: np.ndarray, width: int, height: int,
               min_z_mm: float) -> np.ndarray:
    """Inside the image, in front of the camera, and outside the sensor's blind zone.

    `min_z_mm` is a property of the sensor profile (D16), not a schema constant: below it
    a stereo pair has no overlap and the camera returns nothing at all.
    """
    return ((z_cam > float(min_z_mm))
            & (uv[:, 0] >= 0.0) & (uv[:, 0] < float(width))
            & (uv[:, 1] >= 0.0) & (uv[:, 1] < float(height)))
