"""Ray-cast visibility (D6) and HPR exteriority — Phase 3.

`trimesh.ray` is not used: its intersector needs `rtree`, and D9 requires the tier-1 core
to install on a clean machine. That is the same constraint that produced the analytic
`Slab.contains`, and the same answer applies — every part is a box, so ray-box intersection
is exact in closed form and vectorises over the whole cloud at once. Phase 6's swept and
revolved primitives are where a real ray engine (open3d's `RaycastingScene`) becomes
necessary, and by then it is a dependency that earns itself.

**What `visible_from_cam` means.** Geometry only: the point is in front of the camera,
inside the image, beyond the sensor's blind zone, facing the camera, and not occluded by
another part. Grazing-incidence dropout is *not* in the mask — a steeply-viewed surface is
visible, the stereo matcher just fails on it. That is a sensor effect and it lives in
`noise.apply`, which returns its own validity mask. Keeping the two apart is what lets
`occluded_fraction` stay a statement about geometry, comparable across sensor profiles.
"""

from __future__ import annotations

import numpy as np

from .camera import in_frustum, project
from .geom import Slab

#: Ray origins are lifted this far off the surface along the normal before casting, so a
#: point never occludes itself with its own face. Small next to any plate thickness, large
#: next to float32 storage error on a 400 mm plate.
SURFACE_EPS_MM = 1e-3


def ray_hits_slab(origins: np.ndarray, directions: np.ndarray, t_max: np.ndarray,
                  slab: Slab) -> np.ndarray:
    """Does each ray meet `slab` strictly before `t_max`? Exact, vectorised.

    The classic slab method, in the box's own frame: clip the ray against the three pairs
    of parallel planes and keep the interval that survives all three.
    """
    R = slab.T_world_part[:3, :3]
    c = slab.T_world_part[:3, 3]
    half = np.asarray(slab.dims_mm, dtype=float) / 2.0

    o = (np.asarray(origins, dtype=float) - c) @ R          # world -> box local
    d = np.asarray(directions, dtype=float) @ R

    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-half - o) / d
        t2 = (half - o) / d
    # A ray parallel to a slab pair either misses it entirely (origin outside) or is
    # unconstrained by it. inf/-inf express both without branching.
    parallel = np.abs(d) < 1e-12
    outside = parallel & (np.abs(o) > half)
    t1 = np.where(parallel, -np.inf, t1)
    t2 = np.where(parallel, np.inf, t2)

    t_near = np.max(np.minimum(t1, t2), axis=1)
    t_far = np.min(np.maximum(t1, t2), axis=1)
    return (t_near <= t_far) & (t_far > 0.0) & (t_near < np.asarray(t_max)) \
        & ~outside.any(axis=1)


def occluded(points: np.ndarray, cam_pos: np.ndarray, slabs, normals=None) -> np.ndarray:
    """True where the straight line from a point to the camera passes through a part."""
    p = np.asarray(points, dtype=float)
    if normals is not None:
        p = p + np.asarray(normals, dtype=float) * SURFACE_EPS_MM
    v = np.asarray(cam_pos, dtype=float)[None, :] - p
    dist = np.linalg.norm(v, axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        d = v / dist[:, None]

    hit = np.zeros(len(p), dtype=bool)
    for s in slabs:
        hit |= ray_hits_slab(p, d, dist, s)
    return hit


def visible_mask(points: np.ndarray, normals: np.ndarray, slabs,
                 T_world_cam: np.ndarray, K, width: int, height: int,
                 min_z_mm: float, face_test: bool = True) -> np.ndarray:
    """The full D6 test: framed, in range, front-facing and unoccluded.

    `face_test` is what separates a surface point from a seam point. A surface point
    genuinely cannot be seen from behind its own face. A seam point lies on the crease
    between two faces, on the boundary of both solids, and is visible from a far wider
    span than the 90 deg either side of its bisector - so for seams the ray-cast is the
    whole test and `normals` serves only to lift the origin off the surface.
    """
    p = np.asarray(points, dtype=float)
    cam_pos = np.asarray(T_world_cam, dtype=float)[:3, 3]

    uv, z_cam = project(p, T_world_cam, np.asarray(K, dtype=float))
    ok = in_frustum(uv, z_cam, width, height, min_z_mm)

    if face_test and normals is not None:
        # Facing away from the camera: no ray-cast needed and none would be correct, since
        # the surface's own solid is on the wrong side of it.
        to_cam = cam_pos[None, :] - p
        ok &= np.einsum("ij,ij->i", to_cam, np.asarray(normals, dtype=float)) > 0.0

    if ok.any():
        idx = np.flatnonzero(ok)
        n_sub = None if normals is None else np.asarray(normals, dtype=float)[idx]
        ok[idx] &= ~occluded(p[idx], cam_pos, slabs, n_sub)
    return ok


def seam_masks(seam_pts: np.ndarray, approach: np.ndarray, slabs,
               T_world_cam: np.ndarray, K, width: int, height: int,
               min_z_mm: float) -> dict[str, np.ndarray]:
    """Per-sample visibility along a seam, **decomposed**.

    Two different physics, kept apart because a consumer may want to filter on one and not
    the other:

      * `in_frame`   - inside the image and beyond the sensor's blind zone. A 400 mm seam
                       at 350 mm standoff does not fit the frame, and its near end can sit
                       inside the blind zone while its far end does not. Both cut the seam
                       *partway*, which is the only graded source of lost visibility that
                       two convex slabs and a straight seam can produce.
      * `unoccluded` - not hidden by another part. All-or-nothing for a straight seam under
                       a convex occluder, since the occluder spans the run the two parts
                       share to begin with.

    `visible` is their conjunction and is what gets stored: it is what the sensor returns.
    """
    p = np.asarray(seam_pts, dtype=float)
    n = np.asarray(approach, dtype=float)
    cam_pos = np.asarray(T_world_cam, dtype=float)[:3, 3]

    uv, z_cam = project(p, T_world_cam, np.asarray(K, dtype=float))
    in_frame = in_frustum(uv, z_cam, width, height, min_z_mm)
    # Cast along the whole seam, not only where it is framed: `occluded_fraction` has to
    # mean "hidden by another part" independently of how the camera happens to be framed,
    # or the two numbers are not separable after the fact.
    unoccluded = ~occluded(p, cam_pos, slabs, n)
    return {"in_frame": in_frame, "unoccluded": unoccluded,
            "visible": in_frame & unoccluded}


def seam_visibility(seam_pts: np.ndarray, approach: np.ndarray, slabs,
                    T_world_cam: np.ndarray, K, width: int, height: int,
                    min_z_mm: float) -> np.ndarray:
    """Per-sample visibility along one seam.

    The seam lies in the crease between two parts, so it has no single outward normal, and
    `approach_dir` must not be pressed into service as one. It is used only to lift the
    origin off the two faces that form the seam; whether the seam is *seen* is decided by
    the ray-cast alone. Testing the camera against the bisector as if it were a normal
    rejects a fillet the moment the camera crosses 90 deg from the torch direction, which
    on a T-joint is most of the hemisphere it is plainly visible from - every seam in the
    first scenes came back `occluded_fraction: 1.0`.
    """
    return visible_mask(seam_pts, np.asarray(approach, dtype=float), slabs,
                        T_world_cam, K, width, height, min_z_mm, face_test=False)


def hpr_exterior(points: np.ndarray, n_views: int = 30, radius_factor: float = 100.0
                 ) -> np.ndarray:
    """Hidden Point Removal exteriority (Katz, Tal & Basri 2007) — the CAD-free test.

    A point is **exterior** if some viewpoint on a surrounding sphere can see it. HPR gets
    that without building a surface: invert the cloud through a sphere centred on the
    viewpoint, so a point at distance `d` maps to radius `2R - d` and near points land
    outside far ones, then take the convex hull. What survives on the hull is visible.

    This is not how the ground truth decides exteriority - Phases 1-2 know the placement
    transforms and answer it analytically. It exists because the Phase 4 baselines do not:
    at runtime there is no CAD registration, so a point-cloud-only method needs this to
    reject the buried mid-lap interface that radius-PCA otherwise fires on.
    """
    from scipy.spatial import ConvexHull

    p = np.asarray(points, dtype=float)
    if len(p) < 4:
        return np.ones(len(p), dtype=bool)

    centre = p.mean(axis=0)
    scale = float(np.max(np.linalg.norm(p - centre, axis=1)))
    exterior = np.zeros(len(p), dtype=bool)

    for eye in _fibonacci_sphere(n_views) * (scale * 3.0) + centre:
        v = p - eye
        d = np.linalg.norm(v, axis=1)
        d = np.where(d < 1e-12, 1e-12, d)
        R = d.max() * radius_factor
        flipped = v * ((2.0 * (R - d) / d) + 1.0)[:, None]
        try:
            hull = ConvexHull(np.vstack([flipped, np.zeros((1, 3))]))
        except Exception:                              # degenerate view, skip it
            continue
        exterior[hull.vertices[hull.vertices < len(p)]] = True
    return exterior


def _fibonacci_sphere(n: int) -> np.ndarray:
    """`n` roughly equidistant directions — deterministic, no RNG draw.

    Deliberately not sampled: exteriority is a property of the geometry, so it must not
    move with a seed or consume a substream draw.
    """
    i = np.arange(n, dtype=float) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)
    theta = np.pi * (1.0 + 5.0 ** 0.5) * i
    return np.column_stack([np.cos(theta) * np.sin(phi),
                            np.sin(theta) * np.sin(phi),
                            np.cos(phi)])
