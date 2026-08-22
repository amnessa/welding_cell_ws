"""`lit-pcaslice` — PCA-determined slicing direction, per-slice centres, B-spline path.

A faithful reimplementation of the point-cloud half of:

    Tianqi Wang, Yifan Liang, Xinqi Liu, Zhigang Wang.
    "3D vision-based intersecting pipe welding path planning."
    *Welding in the World* (2026).  https://doi.org/10.1007/s40194-026-02509-9

Their pipeline, and which stages this dataset supplies:

    §3      YOLO11 boxes each weld INSTANCE; improved DeepLab V3+ segments the weld
            pixels inside the box; pixel mapping lifts them to a weld point cloud
                                    -> a PER-INSTANCE band ORACLE (see below)
    §4.1    Gaussian + statistical filtering            -> `statistical_filter`
    §4.2    PCA principal direction; project; slice uniformly; per-slice geometric
            centre                                      -> `pca_centerline`
    §4.3    cubic NURBS fit / interpolation             -> `bspline_path`
    §4.4    torch posture from the dihedral bisector, planes by MSAC + WTLSD
                                                        -> `msac_wtlsd_plane`, `torch_poses`

Published: welding accuracy **within 1 mm** on intersecting-pipe (saddle-curve) welds.
The slice width is never given a value; neither are the MSAC tolerance or the filter
constants. The tube radius for posture planes is **5 mm** ("empirically set").

The coarse stage is PER-INSTANCE, and that is not a detail
----------------------------------------------------------
YOLO boxes each weld separately, so §4.2 only ever sees ONE strip-like cloud at a time.
That assumption is structural: the per-slice *geometric centre* is the mechanism, and the
geometric centre of a slice containing TWO seams is the midpoint between them — the exact
mid-surface failure that made `ours` delete its line fit. Handing this method a merged
multi-seam mask therefore breaks it at the mechanism, not at a threshold; the honest L0
supplies one band per truth seam (`per_instance=True` in the harness adapter), and the L1
arm (whole cloud, one instance) measures the assumption's price directly.

Where the randomness lives
--------------------------
The PATH pipeline (§4.1-4.3) is deterministic. MSAC — the one sampling-based stage — feeds
only the torch POSTURE (§4.4), which the Phase 4 metrics do not currently score. The
registry therefore carries `randomised=False` for path scoring, with the seed threaded to
`torch_poses` so the flag flips cleanly if pose metrics land.

The plan's note that this method is "most interesting at Phase 6" stands: on straight
seams the PCA axis and the slice centres are near-trivial, and the adaptive-slicing claim
only starts working when the seam curves. Everything is **millimetres**.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:                                    # pragma: no cover
    cKDTree = None

TUBE_RADIUS_MM = 5.0          # §4.4, "empirically set to 5 mm"


def statistical_filter(pts: np.ndarray, k: int = 12, std_mul: float = 2.0) -> np.ndarray:
    """§4.1's statistical outlier removal: drop points whose mean k-NN distance is far.

    The Gaussian smoothing step is deliberately omitted on clean clouds — it exists to
    fight scanner speckle, and on the noiseless arm it would only blur the geometry the
    metrics measure. The noisy arm gets the same treatment their scan does.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) <= k:
        return pts
    d, _ = cKDTree(pts).query(pts, k=k + 1, workers=-1)
    m = d[:, 1:].mean(axis=1)
    return pts[m <= m.mean() + std_mul * m.std()]


def pca_centerline(pts: np.ndarray, slice_mm: float | None = None,
                   min_slice_pts: int = 3) -> np.ndarray:
    """§4.2 — eqs. 5-10 and the slicing: centres of uniform slices along the PCA axis.

    The slicing DIRECTION is the data's own principal direction — the "adaptive" in the
    paper's claim, against projection methods that need the direction chosen by hand.
    `slice_mm` is unpublished; the default is 3x the estimated point spacing, exposed.

    The mechanism to keep in view: each slice contributes its **geometric centre**. One
    strip-like cloud gives centres on the strip's spine; a slice that happens to contain
    two seams gives the midpoint *between* them, which lies on neither.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2 * min_slice_pts:
        return np.zeros((0, 3))
    c = pts.mean(axis=0)
    _, v = np.linalg.eigh(np.cov((pts - c).T))
    axis = v[:, -1]                                    # largest eigenvalue: the seam runs here
    t = (pts - c) @ axis
    if slice_mm is None:
        from .radius_pca import mean_spacing_mm
        slice_mm = max(3.0 * mean_spacing_mm(pts), 1.0)
    edges = np.arange(t.min(), t.max() + slice_mm, slice_mm)
    centers = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (t >= lo) & (t < hi)
        if int(m.sum()) >= min_slice_pts:
            centers.append(pts[m].mean(axis=0))
    return np.asarray(centers) if centers else np.zeros((0, 3))


def bspline_path(centers: np.ndarray, n_samples: int = 60, degree: int = 3
                 ) -> np.ndarray | None:
    """§4.3 — the smooth path through the slice centres.

    The paper fits cubic NURBS; with uniform weights a NURBS curve *is* a B-spline, and
    the paper gives no non-uniform weights, so `splprep` with k = 3 is the same curve
    family. Recorded as an equivalence, not a deviation. Falls back to the polyline of
    centres when there are too few for a cubic.
    """
    centers = np.asarray(centers, dtype=float)
    if len(centers) < 2:
        return None
    if len(centers) <= degree + 1:
        return centers
    from scipy.interpolate import splev, splprep
    tck, _ = splprep(centers.T, k=min(degree, len(centers) - 1),
                     s=len(centers) * 0.25)
    u = np.linspace(0.0, 1.0, int(n_samples))
    return np.column_stack(splev(u, tck))


def msac_wtlsd_plane(pts: np.ndarray, tol_mm: float = 1.0, iters: int = 60,
                     seed: int = 0) -> tuple[np.ndarray, float] | None:
    """§4.4's plane fit: MSAC consensus, then distance-weighted total least squares.

    MSAC scores a hypothesis by the truncated loss `sum(min(d^2, tol^2))` rather than by
    an inlier count — the M-estimator half of the name. WTLSD refits over the inliers
    with weights `1/(d + eps)`, i.e. a weighted PCA plane. This is the ONLY sampling-based
    stage in the whole method, and it feeds the torch posture, not the path.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3:
        return None
    rng = np.random.default_rng(seed)
    best, best_loss = None, np.inf
    for _ in range(iters):
        i = rng.choice(len(pts), 3, replace=False)
        n = np.cross(pts[i[1]] - pts[i[0]], pts[i[2]] - pts[i[0]])
        nn = np.linalg.norm(n)
        if nn < 1e-12:
            continue
        n = n / nn
        d = np.abs((pts - pts[i[0]]) @ n)
        loss = float(np.minimum(d, tol_mm).__pow__(2).sum())
        if loss < best_loss:
            best_loss, best = loss, (n, pts[i[0]])
    if best is None:
        return None
    n, p0 = best
    d = np.abs((pts - p0) @ n)
    inl = pts[d <= tol_mm]
    if len(inl) >= 3:
        w = 1.0 / ((np.abs((inl - inl.mean(0)) @ n)) + 1e-3)
        c = np.average(inl, axis=0, weights=w)
        A = (inl - c) * np.sqrt(w)[:, None]
        _, _, vt = np.linalg.svd(A, full_matrices=False)
        n = vt[-1] * np.sign(vt[-1] @ n)
        p0 = c
    return n, float(-n @ p0)


def torch_poses(path: np.ndarray, cloud: np.ndarray, radius_mm: float = TUBE_RADIUS_MM,
                seed: int = 0) -> list[np.ndarray]:
    """§4.4 — per path point: two local planes, dihedral bisector Z, tangent Y.

    The local ball (radius = the paper's 5 mm tube radius) is split into the two adjacent
    surfaces by sequential MSAC — fit one plane, remove its inliers, fit the second — and
    the torch axis is the bisector of their normals, signs aligned to point away from the
    material. Returns one 3x3 frame per path point (columns X, Y, Z).
    """
    path = np.asarray(path, dtype=float)
    cloud = np.asarray(cloud, dtype=float)
    tree = cKDTree(cloud)
    out = []
    for i, p in enumerate(path):
        tangent = path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)]
        tangent = tangent / max(np.linalg.norm(tangent), 1e-12)
        ids = tree.query_ball_point(p, radius_mm)
        ball = cloud[np.asarray(ids, dtype=int)] if len(ids) else np.zeros((0, 3))
        z = np.array([0.0, 0.0, 1.0])
        f1 = msac_wtlsd_plane(ball, seed=seed) if len(ball) >= 6 else None
        if f1 is not None:
            n1, d1 = f1
            rest = ball[np.abs(ball @ n1 + d1) > 1.0]
            f2 = msac_wtlsd_plane(rest, seed=seed + 1) if len(rest) >= 6 else None
            if f2 is not None:
                n2 = f2[0] * np.sign(f2[0] @ n1) if abs(f2[0] @ n1) > 1e-6 else f2[0]
                z = n1 + n2
                z = z / max(np.linalg.norm(z), 1e-12)
            else:
                z = n1
        x = np.cross(tangent, z)
        x = x / max(np.linalg.norm(x), 1e-12)
        out.append(np.column_stack([x, tangent, z]))
    return out


@dataclass
class PCASliceResult:
    """What `lit-pcaslice` returned, and enough context to know what it means."""

    seams: list[np.ndarray]               #: fitted B-spline paths, one per instance
    centers: list[np.ndarray]             #: the slice centres each path was fitted to
    poses: list[list[np.ndarray]] = field(default_factory=list)
    params: dict[str, Any] = field(default_factory=dict)
    #: True when per-instance masks were supplied - their YOLO+DeepLab stage. The method's
    #: single-strip assumption is STRUCTURAL (per-slice geometric centres), so this oracle
    #: is not a convenience; without it the mechanism itself is broken.
    used_instance_oracle: bool = False
    note: str = ""

    @property
    def polylines(self) -> list[np.ndarray]:
        return self.seams

    @property
    def n_seams(self) -> int:
        return len(self.seams)


def detect(pts: np.ndarray, instance_masks: list[np.ndarray] | None = None,
           slice_mm: float | None = None, filter_k: int = 12, filter_std: float = 2.0,
           n_samples: int = 60, min_centers: int = 3, plan_poses: bool = False,
           seed: int = 0) -> PCASliceResult:
    """Run `lit-pcaslice`. Lengths in **millimetres**.

    Args:
        instance_masks: one boolean mask per weld instance — the YOLO11 + DeepLab stage,
            supplied as an **oracle**. `None` is the L1 arm: the whole cloud as a single
            instance, which is exactly the input the mechanism cannot survive when the
            scene holds more than one seam.
        slice_mm: unpublished; default 3x point spacing.
        plan_poses: run §4.4 (MSAC + WTLSD dihedral frames). Off by default because the
            Phase 4 metrics do not score poses and MSAC is the one seeded stage.
    """
    pts = np.asarray(pts, dtype=float)
    used_oracle = instance_masks is not None
    groups = ([np.asarray(m, dtype=bool) for m in instance_masks]
              if used_oracle else [np.ones(len(pts), dtype=bool)])
    params = dict(slice_mm=slice_mm, filter_k=filter_k, filter_std=filter_std,
                  n_instances=len(groups), n_input=len(pts))

    seams, centers_all, poses = [], [], []
    for gi, m in enumerate(groups):
        sub = pts[m]
        if len(sub) < 2 * min_centers:
            continue
        sub = statistical_filter(sub, filter_k, filter_std)
        centers = pca_centerline(sub, slice_mm, )
        if len(centers) < min_centers:
            continue
        path = bspline_path(centers, n_samples)
        if path is None:
            continue
        seams.append(path)
        centers_all.append(centers)
        if plan_poses:
            poses.append(torch_poses(path, sub, seed=seed + gi))
    note = "" if seams else "no instance produced enough slice centres"
    return PCASliceResult(seams, centers_all, poses, params, used_oracle, note)
