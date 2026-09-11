"""Conventions the renderer and its consumers share — pinned by the 2026-09-10 pilot.

1. Pixel centres: depth pixel (i, j) back-projects through (u, v) = (j + 0.5, i + 0.5) with the
   stored `K` (principal point at W/2, H/2). Using (j, i) puts a 0.26 mm bias in.
2. OpenCV -> USD camera: `T_usd = T_world_cam @ diag(1, -1, -1, 1)` (USD looks down -Z, +Y up),
   translation in metres. Gf matrices are row-vector, so set `T_usd.T`.
3. Intrinsics: focalLength = fx * horizontalAperture / W, verticalAperture = ha * H / W.
4. Units: the stage is metres, every weldgen number is millimetres; multiply by 0.001 in and
   by 1000 out. `distance_to_image_plane` is z_cam, the depth `camera.project` uses.
5. Depth on disk: uint16 PNG at 0.05 mm per unit (range 3.28 m), 0 = no return.
6. Object mask labels: object_id + 1 for workpieces, 255 fixture (object_id 254 + 1),
   254 environment (substrate plane), 0 nothing rendered.
"""

from __future__ import annotations

import numpy as np

MM_PER_M = 1000.0
M_PER_MM = 0.001
#: default USD horizontal aperture (mm) - any value works, only the focal/aperture ratio matters
USD_APERTURE_MM = 20.955
#: depth.png quantum, mm per uint16 unit
DEPTH_SCALE_MM = 0.05
DEPTH_MAX_MM = DEPTH_SCALE_MM * 65535
#: mask_object values
LABEL_NONE = 0
LABEL_ENV = 254
LABEL_FIXTURE = 255
#: `objects[].object_id` of the fixture (SCHEMA.md §2.1)
FIXTURE_OBJECT_ID = 255


def usd_camera_matrix(T_world_cam: np.ndarray) -> np.ndarray:
    """4x4 USD camera-to-world matrix (metres, -Z forward) from OpenCV `T_world_cam` (mm)."""
    T = np.asarray(T_world_cam, dtype=float).copy()
    T[:3, 3] *= M_PER_MM
    return T @ np.diag([1.0, -1.0, -1.0, 1.0])


def usd_intrinsics(K, width: int, height: int) -> tuple[float, float, float]:
    """`(focalLength, horizontalAperture, verticalAperture)` for a `UsdGeom.Camera`."""
    fx = float(np.asarray(K, dtype=float)[0, 0])
    ha = USD_APERTURE_MM
    return fx * ha / width, ha, ha * height / width


def object_label(object_id: int) -> int:
    """`mask_object` value for a part."""
    return LABEL_FIXTURE if int(object_id) == FIXTURE_OBJECT_ID else int(object_id) + 1


def pixel_rays(K, width: int, height: int) -> np.ndarray:
    """(H, W, 3) camera-frame directions through PIXEL CENTRES, z = 1."""
    K = np.asarray(K, dtype=float)
    u = (np.arange(width) + 0.5 - K[0, 2]) / K[0, 0]
    v = (np.arange(height) + 0.5 - K[1, 2]) / K[1, 1]
    uu, vv = np.meshgrid(u, v)
    return np.stack([uu, vv, np.ones_like(uu)], -1)


def backproject(depth_mm: np.ndarray, K, T_world_cam, valid=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """World-frame points (mm) of the valid depth pixels. Returns `(xyz, rows, cols)`."""
    d = np.asarray(depth_mm, dtype=float)
    H, W = d.shape
    if valid is None:
        valid = np.isfinite(d) & (d > 0)
    rows, cols = np.nonzero(valid)
    rays = pixel_rays(K, W, H)[rows, cols]
    cam = rays * d[rows, cols][:, None]
    T = np.asarray(T_world_cam, dtype=float)
    return cam @ T[:3, :3].T + T[:3, 3], rows, cols


def encode_depth(depth_mm: np.ndarray, valid=None) -> np.ndarray:
    """uint16 image at `DEPTH_SCALE_MM`; invalid or out-of-range pixels are 0."""
    d = np.asarray(depth_mm, dtype=float)
    ok = (np.isfinite(d) & (d > 0)) if valid is None else (np.asarray(valid, bool) & np.isfinite(d) & (d > 0))
    ok &= d <= DEPTH_MAX_MM
    out = np.zeros(d.shape, dtype=np.uint16)
    out[ok] = np.clip(np.round(d[ok] / DEPTH_SCALE_MM), 1, 65535).astype(np.uint16)
    return out


def decode_depth(png16: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """`(depth_mm float64, valid bool)` from the uint16 image."""
    a = np.asarray(png16)
    valid = a > 0
    return a.astype(np.float64) * DEPTH_SCALE_MM, valid
