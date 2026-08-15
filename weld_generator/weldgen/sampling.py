"""Surface sampling — `area_uniform` mode (SCHEMA.md §5.1).

Points are sampled per FACE rather than via `trimesh.sample`, for three reasons:

  * normals are exact (the face's own outward normal), not interpolated from triangles;
  * every point carries an exact `face_id`, giving free per-point face segmentation —
    a superset of the A/B/intersect masks K-Net-style methods consume;
  * the draw order is a deterministic function of (part order, face registry order),
    which the content-hash gate depends on.

`area_uniform` is NOT what a depth camera returns; `camera_raster` (Phase 3, D20) is.
The distinction is recorded per scene in `cloud.sampling_mode`.
"""

from __future__ import annotations

import numpy as np

from .geom import SLAB_FACES, Slab


def sample_slab_surface(
    slab: Slab,
    density_per_mm2: float,
    rng: np.random.Generator,
    face_id_base: int,
) -> dict[str, np.ndarray]:
    """Area-uniform sample over all six faces of one slab.

    Returns arrays keyed `xyz`, `normals`, `object_id`, `face_id`.
    """
    L, W, t = (float(v) for v in slab.dims_mm)
    half = np.array([L, W, t]) / 2.0

    xyz, normals, face_ids = [], [], []
    for local_id, name in enumerate(SLAB_FACES):
        area = slab.face_area(name)
        n_pts = int(round(density_per_mm2 * area))
        if n_pts <= 0:
            continue

        axis = "uvw".index(name[1])
        sign = 1.0 if name[0] == "+" else -1.0
        others = [i for i in range(3) if i != axis]

        # Uniform in the face's own 2D parametrisation.
        uv = rng.random((n_pts, 2))
        local = np.empty((n_pts, 3))
        local[:, axis] = sign * half[axis]
        local[:, others[0]] = (uv[:, 0] * 2.0 - 1.0) * half[others[0]]
        local[:, others[1]] = (uv[:, 1] * 2.0 - 1.0) * half[others[1]]

        world = (slab.T_world_part @ np.column_stack(
            [local, np.ones(n_pts)]).T)[:3].T
        xyz.append(world)
        normals.append(np.tile(slab.face_normal(name), (n_pts, 1)))
        face_ids.append(np.full(n_pts, face_id_base + local_id, dtype=np.uint8))

    if not xyz:  # pragma: no cover - a zero-area slab is rejected upstream
        empty = np.zeros((0, 3), dtype=np.float32)
        return {"xyz": empty, "normals": empty,
                "object_id": np.zeros(0, np.uint8), "face_id": np.zeros(0, np.uint8)}

    xyz = np.vstack(xyz).astype(np.float32)
    return {
        "xyz": xyz,
        "normals": np.vstack(normals).astype(np.float32),
        "object_id": np.full(len(xyz), slab.object_id, dtype=np.uint8),
        "face_id": np.concatenate(face_ids),
    }


def sample_scene_surface(
    slabs: list[Slab],
    density_per_mm2: float,
    rng: np.random.Generator,
) -> dict[str, np.ndarray]:
    """Concatenate per-slab samples in `objects[]` order.

    `visible_from_cam` is emitted all-True in Phase 1: there is no camera yet, so every
    point is "visible" by construction. Phase 3 replaces this with the ray-cast result.
    The field exists now so the array layout never changes (D6).
    """
    parts = [
        sample_slab_surface(s, density_per_mm2, rng, face_id_base=6 * i)
        for i, s in enumerate(slabs)
    ]
    out = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    out["visible_from_cam"] = np.ones(len(out["xyz"]), dtype=bool)
    return out


def sample_polyline(p0: np.ndarray, p1: np.ndarray, density_per_mm: float) -> np.ndarray:
    """Resample a straight seam at a requested density.

    This is the "10 dots per mm, or 50 if someone asks" property: the curve is stored
    parametrically, so any density is available without rerunning the generator.
    """
    length = float(np.linalg.norm(np.asarray(p1) - np.asarray(p0)))
    n = max(2, int(round(length * float(density_per_mm))) + 1)
    ts = np.linspace(0.0, 1.0, n)[:, None]
    return (np.asarray(p0)[None, :] * (1 - ts) + np.asarray(p1)[None, :] * ts)
