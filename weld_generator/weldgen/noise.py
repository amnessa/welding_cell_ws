"""Analytic stereo depth noise — PARAMETERS.md §4.2.

Derived rather than tuned. Stereo depth is `z = f*b/d`, so disparity error propagates as

    sigma_z(z) = subpixel_px * z^2 / (f_px * b_mm)

which is the `sigma ~ z^2` shape the admittance repo already uses, but with coefficients
that come from the sensor's geometry instead of a guess. Two more effects ride along:
lateral blur, isotropic in the image plane, and dropout past a grazing incidence angle -
the one that actually removes the floor of a steeply-viewed fillet.

**The realisation is not stored** (SCHEMA.md §5.1). `scene.json` carries the parameters and
`cloud.npz` carries the clean points; the noisy cloud is whatever this function returns,
which makes the noise a citable convention anyone can re-run with different constants
rather than a baked artefact. It also halves the release size. The frozen Zenodo release is
the exception - it materialises one realisation so cross-paper numbers are bit-comparable
without depending on a library version.
"""

from __future__ import annotations

import numpy as np


def sigma_z_mm(z_mm: np.ndarray | float, noise_model: dict) -> np.ndarray:
    """Axial depth sigma at range `z`, in mm. The whole model in one line."""
    return (float(noise_model["subpixel_px"]) * np.asarray(z_mm, dtype=float) ** 2
            / (float(noise_model["focal_px"]) * float(noise_model["baseline_mm"])))


def apply(xyz: np.ndarray, normals: np.ndarray, T_world_cam: np.ndarray,
          noise_model: dict, K=None) -> tuple[np.ndarray, np.ndarray]:
    """Corrupt a clean cloud the way the sensor would. Returns `(xyz_noisy, valid)`.

    Deterministic in `noise_model["seed"]`, so the same scene gives the same realisation on
    any machine without storing it.

    `valid` is the *sensor's* dropout mask and is deliberately separate from
    `visible_from_cam`: a grazing surface is geometrically visible, the stereo matcher just
    fails on it. Keeping them apart is what lets `occluded_fraction` remain a statement
    about geometry that is comparable across sensor profiles.
    """
    xyz = np.asarray(xyz, dtype=float)
    R, t = np.asarray(T_world_cam, dtype=float)[:3, :3], np.asarray(T_world_cam)[:3, 3]
    cam = (xyz - t) @ R
    z = cam[:, 2]

    rng = np.random.default_rng(int(noise_model["seed"]))

    # Axial error is along the VIEW RAY, not along world z or the surface normal.
    ray = xyz - t
    ray /= np.maximum(np.linalg.norm(ray, axis=1, keepdims=True), 1e-12)
    axial = rng.normal(0.0, 1.0, len(xyz)) * sigma_z_mm(np.abs(z), noise_model)

    # Lateral blur is specified in pixels, so at range it is `sigma_px * z / f`: the same
    # blur costs more millimetres the further away the surface is.
    f = float(noise_model["focal_px"])
    lat_mm = float(noise_model["lateral_sigma_px"]) * np.abs(z) / f
    right, down = R[:, 0], R[:, 1]
    lateral = (rng.normal(0.0, 1.0, len(xyz))[:, None] * right[None, :]
               + rng.normal(0.0, 1.0, len(xyz))[:, None] * down[None, :]) * lat_mm[:, None]

    noisy = xyz + ray * axial[:, None] + lateral

    valid = np.ones(len(xyz), dtype=bool)
    if normals is not None:
        n = np.asarray(normals, dtype=float)
        cos_inc = np.abs(np.einsum("ij,ij->i", ray, n))
        valid &= cos_inc >= np.cos(np.deg2rad(float(noise_model["grazing_dropout_deg"])))
    valid &= np.abs(z) > float(noise_model["min_z_mm"])
    return noisy, valid
