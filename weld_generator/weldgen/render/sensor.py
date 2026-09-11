"""D16 on rendered depth - Phase 8 M4.

Tier 1 stores a CLEAN cloud and the noise-model parameters; the noisy realisation is
whatever `noise.apply` returns for the stored seed (SCHEMA.md §5.1). Tier 2 follows the same
rule on the rendered depth: `depth.png` is the clean ray cast (the twin), `depth_valid.png`
is the DETERMINISTIC part of the sensor model - grazing-incidence dropout on the rendered
normals and the profile's blind zone - and the random displacement is derived on demand by
`realise`, which calls the very same `noise.apply` so the two tiers share one convention.
"""

from __future__ import annotations

import numpy as np

from .. import noise
from .conventions import LABEL_ENV, LABEL_NONE, backproject


def sensor_validity(depth_mm: np.ndarray, valid: np.ndarray, normals_world: np.ndarray,
                    K, T_world_cam, noise_model: dict) -> np.ndarray:
    """Deterministic D16 validity per pixel: rendered pixel AND not grazing AND beyond min_z."""
    xyz, rows, cols = backproject(depth_mm, K, T_world_cam, valid)
    if len(xyz) == 0:
        return np.zeros_like(valid, dtype=bool)
    n = np.asarray(normals_world, dtype=float)[rows, cols]
    _, ok = noise.apply(xyz, n, T_world_cam, noise_model)
    out = np.zeros_like(valid, dtype=bool)
    out[rows, cols] = ok
    return out


def realise(depth_mm: np.ndarray, valid: np.ndarray, normals_world: np.ndarray, K, T_world_cam,
            noise_model: dict, seed: int | None = None) -> tuple[np.ndarray, np.ndarray]:
    """One noisy depth image (mm, 0 where invalid) and its validity, from the stored model.

    `seed` overrides `noise_model["seed"]` (views 1..N-1 use seed + view index so no two
    views share a realisation while view 0 shares tier 1's).
    """
    nm = dict(noise_model)
    if seed is not None:
        nm["seed"] = int(seed)
    xyz, rows, cols = backproject(depth_mm, K, T_world_cam, valid)
    out = np.zeros_like(np.asarray(depth_mm, dtype=float)); ok_img = np.zeros_like(valid, dtype=bool)
    if len(xyz) == 0:
        return out, ok_img
    n = np.asarray(normals_world, dtype=float)[rows, cols]
    noisy, ok = noise.apply(xyz, n, T_world_cam, nm)
    T = np.asarray(T_world_cam, dtype=float)
    z_noisy = ((noisy - T[:3, 3]) @ T[:3, :3])[:, 2]          # depth along the optical axis
    out[rows, cols] = np.where(ok, z_noisy, 0.0); ok_img[rows, cols] = ok
    return out, ok_img


def tier_comparison(scene: dict, cloud: dict, depth_mm: np.ndarray, valid: np.ndarray,
                    normals_world: np.ndarray, mask_object: np.ndarray, tol_mm: float = 1.0) -> dict:
    """The M4 check, POINT-WISE: for every tier-1 visible point that lands on a rendered
    workpiece pixel of the same depth, the sensor validity of the point (noise.apply on the
    cloud's own normal) must agree with the sensor validity of that pixel (noise.apply on the
    rendered normal). Fractions are not comparable across tiers - tier 1 samples surfaces by
    area, a render by projected area, and tier-1 visibility already excludes the blind zone.
    """
    from ..camera import project
    cam = scene["camera"]; K = np.asarray(cam["K"], float); T = np.asarray(cam["T_world_cam"], float)
    nm = scene["noise_model"]; H, W = np.asarray(depth_mm).shape
    t1 = np.asarray(cloud["visible_from_cam"], bool)
    xyz = np.asarray(cloud["xyz"], float)[t1]; nrm = np.asarray(cloud["normals"], float)[t1]
    _, ok_pt = noise.apply(xyz, nrm, T, nm)
    wp = valid & (mask_object != LABEL_NONE) & (mask_object != LABEL_ENV)
    ok_img = sensor_validity(depth_mm, wp, normals_world, K, T, nm)
    uv, z = project(xyz, T, K)
    j = np.clip(np.floor(uv[:, 0]).astype(int), 0, W - 1); i = np.clip(np.floor(uv[:, 1]).astype(int), 0, H - 1)
    on = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H) & wp[i, j] & (np.abs(np.asarray(depth_mm)[i, j] - z) < tol_mm)
    agree = ok_pt[on] == ok_img[i[on], j[on]]
    return {"profile": nm.get("profile"), "tier1_points": int(t1.sum()), "compared_points": int(on.sum()),
            "agreement": float(agree.mean()) if on.any() else float("nan"),
            "tier1_valid_fraction": float(ok_pt[on].mean()) if on.any() else float("nan"),
            "tier2_valid_fraction_at_those_pixels": float(ok_img[i[on], j[on]].mean()) if on.any() else float("nan"),
            "tier2_valid_fraction_all_workpiece_pixels": float(ok_img[wp].mean()) if wp.any() else float("nan"),
            "sigma_z_at_median_mm": float(noise.sigma_z_mm(np.median(z[on]), nm)) if on.any() else None}
