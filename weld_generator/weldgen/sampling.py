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

from .geom import SLAB_FACES, Prism, Slab


def sample_slab_surface(
    slab: Slab,
    density_per_mm2: float,
    rng: np.random.Generator,
    face_id_base: int,
) -> dict[str, np.ndarray]:
    """Area-uniform sample over all six faces of one slab.

    Returns arrays keyed `xyz`, `normals`, `object_id`, `face_id`.
    """
    if hasattr(slab, "sample_face"):
        # Phase 6b primitives (Tube / SweptSlab / PreparedSlab) sample their own
        # faces analytically - positions exactly on the true surfaces, D34's chord
        # error stays in the mesh. Same registry contract: per-face draw order,
        # cumulative face ids.
        xyz, normals, face_ids = [], [], []
        for local_id, name in enumerate(slab.face_names()):
            n_pts = int(round(density_per_mm2 * slab.face_area(name)))
            if n_pts <= 0:
                continue
            pts, nrm = slab.sample_face(name, n_pts, rng)
            xyz.append(pts)
            normals.append(nrm)
            face_ids.append(np.full(n_pts, face_id_base + local_id, dtype=np.uint8))
        xyz = np.vstack(xyz).astype(np.float32)
        return {"xyz": xyz,
                "normals": np.vstack(normals).astype(np.float32),
                "object_id": np.full(len(xyz), slab.object_id, dtype=np.uint8),
                "face_id": np.concatenate(face_ids)}

    is_slab = isinstance(slab, Slab)
    if is_slab:
        L, W, t = (float(v) for v in slab.dims_mm)
        half = np.array([L, W, t]) / 2.0

    xyz, normals, face_ids = [], [], []
    for local_id, name in enumerate(slab.face_names()):
        area = slab.face_area(name)
        n_pts = int(round(density_per_mm2 * area))
        if n_pts <= 0:
            continue

        if is_slab:
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
        else:
            world = _sample_prism_face(slab, name, n_pts, rng)
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


def _sample_prism_face(prism: Prism, name: str, n_pts: int,
                       rng: np.random.Generator) -> np.ndarray:
    """Area-uniform world-frame sample of one prism face.

    Caps: fan triangulation from vertex 0, triangle chosen area-weighted, then a uniform
    barycentric draw (the standard fold `u+v>1 -> 1-u, 1-v`). Sides: the rectangle draw.
    Draw order per face is fixed (one selection vector, one (n,2) uv block), so the
    content hash covers it exactly as it covers the slab path.
    """
    t2 = prism.thickness / 2.0
    o = prism.outline_uv
    if name in ("+w", "-w"):
        v0 = o[0]
        tri = np.array([[v0, o[i], o[i + 1]] for i in range(1, len(o) - 1)])
        e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
        areas = 0.5 * np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0])
        pick = rng.random(n_pts)
        idx = np.searchsorted(np.cumsum(areas) / areas.sum(), pick)
        idx = np.clip(idx, 0, len(tri) - 1)
        uv = rng.random((n_pts, 2))
        fold = uv.sum(axis=1) > 1.0
        uv[fold] = 1.0 - uv[fold]
        a, b, c = tri[idx, 0], tri[idx, 1], tri[idx, 2]
        p2 = a + uv[:, :1] * (b - a) + uv[:, 1:] * (c - a)
        w = t2 if name == "+w" else -t2
        local = np.column_stack([p2, np.full(n_pts, w)])
    else:
        k = int(name[1:])
        a2, b2 = o[k], o[(k + 1) % len(o)]
        uv = rng.random((n_pts, 2))
        p2 = a2 + uv[:, :1] * (b2 - a2)
        local = np.column_stack([p2, (uv[:, 1] * 2.0 - 1.0) * t2])
    T = prism.T_world_part
    return local @ T[:3, :3].T + T[:3, 3]


def sample_scene_surface(
    slabs: list[Slab],
    density_per_mm2: float,
    rng: np.random.Generator,
) -> dict[str, np.ndarray]:
    """Concatenate per-slab samples in `objects[]` order.

    `visible_from_cam` is emitted all-True here; `scene.py` overwrites it with the Phase 3
    ray-cast result. The field exists in both cases so the array layout never changes (D6).
    """
    parts = []
    base = 0
    for s in slabs:
        parts.append(sample_slab_surface(s, density_per_mm2, rng, face_id_base=base))
        base += len(s.face_names())    # == 6*i for all-slab scenes: hashes unchanged
    out = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    out["visible_from_cam"] = np.ones(len(out["xyz"]), dtype=bool)
    return out


def raster_density_per_mm2(z_mm: float, focal_px: float) -> float:
    """Surface density a pixel raster achieves at range `z`, in points per mm^2.

    One sample per pixel, and a pixel subtends `z / f_px` mm at range `z`, so the density
    goes as `(f_px / z)^2` - it falls off with the SQUARE of range. This is the first half
    of why `area_uniform` is not what a depth camera returns.
    """
    return float(focal_px / max(float(z_mm), 1e-6)) ** 2


def sample_scene_camera_raster(
    slabs: list[Slab],
    visible_fn,
    z_mean_mm: float,
    focal_px: float,
    rng: np.random.Generator,
) -> dict[str, np.ndarray]:
    """`camera_raster` mode — D20, SCHEMA.md §5.1.

    A raster only produces points where a ray hit, so there is no natural sample of the
    surface the camera *cannot* see. Sampling the visible surface alone would leave the
    mask all-True and collapse `occluded_fraction` to zero exactly when the scene is
    hardest - which is what D20 is about. So the hidden surface is sampled separately, at
    matched density, and flagged `visible_from_cam: false`:

        visible   points <- raster density (f_px / z)^2, then the D6 test
        invisible points <- the same draw's rejects, kept rather than discarded

    Drawing both from one dense area-uniform pass and splitting on the mask is what keeps
    the two rate-matched: the alternative, two independent draws, has to reconcile two
    densities that were never equal to begin with.

    The union is stored as one cloud, so D6's file layout is untouched and every consumer
    that filters on the mask keeps working. What changes is that **point density is no
    longer uniform across the mask boundary** - `density_per_mm2` becomes a nominal figure,
    not a guarantee, and any density-sensitive metric must be computed within a mask class
    rather than across it.
    """
    out = sample_scene_surface(slabs, raster_density_per_mm2(z_mean_mm, focal_px), rng)
    out["visible_from_cam"] = visible_fn(out["xyz"], out["normals"])
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
