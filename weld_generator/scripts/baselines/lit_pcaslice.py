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
    §4.3    stitch the four views' centre sets into the base frame, then cubic NURBS
            fit / interpolation                         -> `arc_centerline`, `bspline_path`
    §4.4    torch posture from the dihedral bisector, planes by MSAC + WTLSD
                                                        -> `msac_wtlsd_plane`, `torch_poses`

Published: welding accuracy **within 1 mm** on intersecting-pipe (saddle-curve) welds.
The slice width is never given a value; neither are the MSAC tolerance, the filter
constants, nor the local ball radius r of §4.4 (kept symbolic there).

Their §4.2 never sees a whole ring, and that is the second structural fact
-------------------------------------------------------------------------
The seam they slice is a CLOSED saddle curve, but no single camera pose ever sees it
closed: Figs. 7, 8c and 11 show one OPEN CRESCENT per view, and §4.3 stitches the four
views' results into the robot base frame before the single CLOSED cubic NURBS of
Fig. 13b is fitted. The slicing in §4.2 therefore always runs on an ARC, whose own PCA
direction is the arc's chord — a direction the arc is monotone along, so each slice cuts
it once.

Hand the same stage a closed ring in one piece and the mechanism inverts: the PCA axis
of a ring lies IN the ring's plane, every slice perpendicular to it cuts the ring TWICE,
and the slice's geometric centre — the midpoint of two opposite sides — lands in mid-air
inside the hole. Measured on this corpus that is F1 <= 0,07 with 25-30 mm median error on
the circle / ellipse / rounded_rect strata: the same mid-surface failure as two seams in
one mask, produced by one seam that closes.

`detect(..., ring_mode="arcs")` (the default) therefore SUBSTITUTES for their multi-view
capture: a closed band is split into `n_arcs=4` overlapping angular sectors about its
centroid in the band plane — the harness stand-in for their four camera poses — §4.2 runs
per arc, the centres are stitched back in angular order, and §4.3 is fitted PERIODIC so
Fig. 13b's closed curve comes out closed. It is a substitution, stated as one: the
sectors are geometric, not a rendered four-pose capture, so this arm measures their
slicing mechanism on arcs, not their hand-eye stitching error. `ring_mode="whole"` keeps
the ring-in-one-piece behaviour as a labelled ablation rung.

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

Deviations and substitutions, in one place
------------------------------------------
    * closed rings via 4 overlapping arcs  — SUBSTITUTION for §4.3's four-view capture and
      stitching (we own the whole cloud, so the sectors are cut geometrically). Default.
    * `ring_mode="whole"`                  — ABLATION: §4.2 on the ring in one piece, the
      input their pipeline never produces. Kept to price the assumption.
    * cubic B-spline for cubic NURBS       — EQUIVALENCE (uniform weights; none published).
    * WTLSD as a SINGLE weighted pass      — DEVIATION: their eq. (25) iterates until
      ||xi(i+1) - xi(i)|| < delta_0, and delta_0 is not published.
    * slice width, MSAC tolerance, filter constants, ball radius r — INVENTED defaults; the
      paper gives no value for any of them.

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

# INVENTED default for §4.4's local ball radius r. The paper keeps r symbolic ("select
# points within a radius r for planar fitting") and never gives it a value — 5 mm is ours,
# chosen to sit between the corpus's plate thicknesses and the seam curvature, and it is
# exposed as `radius_mm` so the choice stays visible.
TUBE_RADIUS_MM = 5.0


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


def band_plane(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """The band's own frame: `(centroid, e1, e2, normal)`, normal = least-variance axis.

    A weld band is a thin ribbon, so its two largest principal axes span the plane the
    seam lives in — exactly the plane a camera looking down the branch pipe sees the
    saddle curve in. Used for the closed/open test and for cutting angular sectors.
    """
    pts = np.asarray(pts, dtype=float)
    c = pts.mean(axis=0)
    _, v = np.linalg.eigh(np.cov((pts - c).T))
    return c, v[:, -1], v[:, -2], v[:, 0]


def band_angles(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Polar coordinates of the band in its own plane: `(theta, radius, centroid)`."""
    c, e1, e2, _ = band_plane(pts)
    d = np.asarray(pts, dtype=float) - c
    x, y = d @ e1, d @ e2
    return np.arctan2(y, x), np.hypot(x, y), c


def band_is_closed(pts: np.ndarray, gap_deg: float = 40.0, hole_frac: float = 0.45,
                   n_bins: int = 36) -> bool:
    """Does this band form a RING (a closed seam) rather than an open strip?

    Their pipeline never has to ask — a camera hands §4.2 an arc by construction — but a
    harness that owns the whole cloud does, and the answer decides whether §4.2 may be run
    on the band in one piece at all. Two conditions in the band's own plane, both needed:

        1. **no angular gap**: the points cover the full turn about the centroid, with no
           empty run wider than `gap_deg`. An open arc, or a ring seen from one side,
           leaves its complement empty.
        2. **a hole in the middle**: in each well-populated angular bin the band sits in
           an annulus that does not reach the centroid — the median over bins of
           r(10th pct) / r(90th pct) is at least `hole_frac`. An open band's centroid sits
           ON the band, so its bins run in to r ~ 0 and the ratio collapses.

    Both tests are per-bin ratios, so neither depends on the ring's aspect ratio, and the
    cut sits in a measured gap rather than on a tuned edge. On this corpus's 40 mm-wide L0
    bands (`seam_region_oracle`, much wider than the seam itself) the ratio reads 0,53-0,61
    for closed circle / ellipse / rounded_rect seams and 0,21-0,34 for open line and
    swept_path seams; a 6 mm-wide synthetic ring reads 0,93, and half a ring is caught by
    the gap test instead.

    It is still an INFERENCE — a ring whose band is nearly as wide as its own radius would
    defeat it. `detect(closed=...)` overrides it, and a caller holding the seam's closed
    flag should pass it rather than let it be re-derived from points.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3 * n_bins:
        return False
    th, r, _ = band_angles(pts)
    b = np.clip(((th + np.pi) / (2 * np.pi) * n_bins).astype(int), 0, n_bins - 1)
    counts = np.bincount(b, minlength=n_bins)
    if not counts.all():                               # widest empty run of bins, with wrap
        e = counts == 0
        run = best = 0
        for k in range(2 * n_bins):
            run = run + 1 if e[k % n_bins] else 0
            best = max(best, run)
        if best * 360.0 / n_bins > gap_deg:
            return False
    # A bin holding a handful of points cannot say anything about the band's radial
    # extent there, and on a straight strip those are exactly the bins that look empty in
    # the middle for the wrong reason.
    floor = max(5.0, 0.2 * float(counts[counts > 0].mean()))
    ratios = []
    for k in range(n_bins):
        if counts[k] < floor:
            continue
        lo, hi = np.percentile(r[b == k], [10, 90])
        ratios.append(float(lo) / max(float(hi), 1e-9))
    return bool(ratios) and float(np.median(ratios)) >= hole_frac


def arc_sectors(pts: np.ndarray, n_arcs: int = 4, overlap_deg: float = 15.0
                ) -> list[np.ndarray]:
    """Split a closed band into `n_arcs` OVERLAPPING angular sectors — our stand-in for
    their four camera poses (§4.3, Fig. 12).

    Each sector spans `360/n_arcs` degrees widened by `overlap_deg/2` at each end, so
    neighbours share `overlap_deg` of arc: the same overlap their stitching has to voxel
    away, and here it is what keeps the sector seams from tearing the centre sequence.
    Returns index arrays, in angular order.
    """
    th, _, _ = band_angles(pts)
    step = 2 * np.pi / int(n_arcs)
    half = np.radians(float(overlap_deg)) / 2.0
    out = []
    for k in range(int(n_arcs)):
        lo, hi = -np.pi + k * step - half, -np.pi + (k + 1) * step + half
        d = np.mod(th - lo, 2 * np.pi)                 # distance forward from the start
        out.append(np.flatnonzero(d <= (hi - lo)))
    return out


def arc_centerline(pts: np.ndarray, slice_mm: float | None = None, n_arcs: int = 4,
                   overlap_deg: float = 15.0, min_slice_pts: int = 3) -> np.ndarray:
    """§4.2 per ARC, then §4.3's stitch: centres of all arcs, in angular order.

    This is the closed-seam path their pipeline actually takes. Each sector is a crescent
    like Fig. 8c, so its own PCA direction is the crescent's chord — a direction the arc
    is monotone along for any sector below 180 degrees, which is what makes "one slice,
    one crossing, one centre on the seam" true again.

    The overlap is sliced but not kept: each arc contributes only the centres inside its
    OWN `360/n_arcs` core sector. The widened ends are there to condition the sector's PCA
    and to stop a slice from being cut short at the seam between sectors — the same
    duplicated material their §4.3 voxel-samples away once the views are in one frame.
    """
    pts = np.asarray(pts, dtype=float)
    if slice_mm is None:
        from .radius_pca import mean_spacing_mm
        slice_mm = max(3.0 * mean_spacing_mm(pts), 1.0)
    c0, e1, e2, _ = band_plane(pts)                # order in the BAND's frame, not an arc's
    step = 2 * np.pi / int(n_arcs)
    centers = []
    for k, idx in enumerate(arc_sectors(pts, n_arcs, overlap_deg)):
        if len(idx) < 2 * min_slice_pts:
            continue
        c = pca_centerline(pts[idx], slice_mm, min_slice_pts)
        if not len(c):
            continue
        d = c - c0
        core = np.mod(np.arctan2(d @ e2, d @ e1) - (-np.pi + k * step), 2 * np.pi) < step
        if core.any():
            centers.append(c[core])
    if not centers:
        return np.zeros((0, 3))
    centers = np.vstack(centers)
    d = centers - c0
    return centers[np.argsort(np.arctan2(d @ e2, d @ e1))]


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


def bspline_path(centers: np.ndarray, n_samples: int = 60, degree: int = 3,
                 closed: bool = False) -> np.ndarray | None:
    """§4.3 — the smooth path through the slice centres.

    The paper fits cubic NURBS; with uniform weights a NURBS curve *is* a B-spline, and
    the paper gives no non-uniform weights, so `splprep` with k = 3 is the same curve
    family. Recorded as an equivalence, not a deviation. Falls back to the polyline of
    centres when there are too few for a cubic.

    `closed=True` fits the PERIODIC spline (`per=1`) that Fig. 13b's single closed red
    curve requires: the centre sequence is wrapped, the knot vector is periodic, and the
    curve is sampled at `n_samples` distinct parameters over [0, 1) — the endpoint is
    excluded because it IS the start — with the first sample repeated once at the end so
    the returned polyline has no seam gap for arclength or distance use.
    """
    centers = np.asarray(centers, dtype=float)
    if len(centers) < 2:
        return None
    if len(centers) <= degree + 1:
        return np.vstack([centers, centers[:1]]) if closed else centers
    from scipy.interpolate import splev, splprep
    if closed:
        # splprep drops the last sample under per=1 (it is taken as the repeat of the
        # first), so hand it the wrapped sequence and lose nothing.
        wrapped = np.vstack([centers, centers[:1]])
        tck, _ = splprep(wrapped.T, k=min(degree, len(centers) - 1), per=1,
                         s=len(centers) * 0.25)
        u = np.linspace(0.0, 1.0, int(n_samples), endpoint=False)
        path = np.column_stack(splev(u, tck))
        return np.vstack([path, path[:1]])
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

    DEVIATION — one pass, not a loop. Their eq. (25) re-weights and re-solves until
    ||xi(i+1) - xi(i)|| < delta_0; this is the FIRST such pass only. delta_0 is not
    published, the poses are not scored at Phase 4, and the first pass carries the shape
    of the correction; a later pose metric should make this a loop before quoting it.
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

    The local ball (radius `radius_mm`, our invented default — the paper keeps r symbolic)
    is split into the two adjacent surfaces by sequential MSAC — fit one plane, remove its
    inliers, fit the second — and the torch axis is the bisector of their normals.

    Eq. (12) exactly: Y_W is the tangent O_Fi -> O_Fi+1, Z_W = **-(n_b + n_m)** (the
    NEGATIVE bisector, i.e. the torch points INTO the dihedral it welds rather than out of
    it), and X_W = **Z_W x Y_W**. Both signs were inverted here before 2026-09-14, which
    mirrored the frame; the poses are not scored, but a mirrored frame is still wrong.
    Returns one 3x3 frame per path point (columns X, Y, Z).
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
            near = np.abs(ball @ n1 + d1) <= 1.0
            rest = ball[~near]
            f2 = msac_wtlsd_plane(rest, seed=seed + 1) if len(rest) >= 6 else None
            if f2 is not None:
                n2, d2 = f2
                # Eq. (12) is written for the OUTWARD normals n_b, n_m; MSAC's come out
                # signed by whichever random triple seeded them, so the bisector would
                # otherwise flip at random from point to point. The joint is CONCAVE from
                # the torch's side — that is what makes it weldable — so each surface's
                # points lie on the OTHER surface's outward side, and that fixes both
                # signs with no viewpoint and no camera pose.
                if len(rest) and (rest.mean(axis=0) @ n1 + d1) < 0:
                    n1, d1 = -n1, -d1
                if near.any() and (ball[near].mean(axis=0) @ n2 + d2) < 0:
                    n2, d2 = -n2, -d2
                z = -(n1 + n2)                         # eq. (12): Z_W = -(n_b + n_m)
                z = z / max(np.linalg.norm(z), 1e-12)
            else:
                # One surface in the ball: the concavity rule has nothing to read, and the
                # paper never faces this case. INVENTED tie-break — take the normal that
                # points up, so the torch comes from above the table like every scan in
                # this corpus, and keep eq. (12)'s minus sign.
                z = -(n1 if n1[2] >= 0 else -n1)
        x = np.cross(z, tangent)                       # eq. (12): X_W = Z_W x Y_W
        x = x / max(np.linalg.norm(x), 1e-12)
        out.append(np.column_stack([x, tangent, z]))
    return out


@dataclass
class PCASliceResult:
    """What `lit-pcaslice` returned, and enough context to know what it means."""

    seams: list[np.ndarray]               #: fitted B-spline paths, one per instance
    centers: list[np.ndarray]             #: the slice centres each path was fitted to
    #: per kept instance: True where the band was treated as a closed ring (sliced per arc
    #: and fitted periodic), False where it was treated as an open strip.
    closed_flags: list[bool] = field(default_factory=list)
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
           seed: int = 0, closed: bool | list[bool] | None = None,
           ring_mode: str = "arcs", n_arcs: int = 4,
           arc_overlap_deg: float = 15.0) -> PCASliceResult:
    """Run `lit-pcaslice`. Lengths in **millimetres**.

    Args:
        instance_masks: one boolean mask per weld instance — the YOLO11 + DeepLab stage,
            supplied as an **oracle**. `None` is the L1 arm: the whole cloud as a single
            instance, which is exactly the input the mechanism cannot survive when the
            scene holds more than one seam.
        slice_mm: unpublished; default 3x point spacing.
        closed: is each instance's seam a closed ring? A single bool applies to every
            instance, a sequence is read parallel to `instance_masks`, and `None` (the
            default) asks `band_is_closed` to infer it. **The kwarg wins when given** — a
            caller holding the seam's own closed flag should pass it rather than let the
            band be re-inferred from points.
        ring_mode: what to do with a band judged closed. `"arcs"` (default) is their
            pipeline's own input: four overlapping sectors, §4.2 per sector, centres
            stitched, periodic §4.3 — see the module docstring. `"whole"` is the ABLATION
            that slices the ring in one piece, where the PCA axis lies in the ring's plane
            and every slice centre falls inside the hole.
        n_arcs, arc_overlap_deg: the sector split standing in for their four camera poses.
        plan_poses: run §4.4 (MSAC + WTLSD dihedral frames). Off by default because the
            Phase 4 metrics do not score poses and MSAC is the one seeded stage.
    """
    pts = np.asarray(pts, dtype=float)
    if ring_mode not in ("arcs", "whole"):
        raise ValueError(f"ring_mode must be 'arcs' or 'whole', got {ring_mode!r}")
    used_oracle = instance_masks is not None
    groups = ([np.asarray(m, dtype=bool) for m in instance_masks]
              if used_oracle else [np.ones(len(pts), dtype=bool)])
    if closed is None or isinstance(closed, (bool, np.bool_)):
        closed_in = [None if closed is None else bool(closed)] * len(groups)
    else:
        closed_in = [bool(c) for c in closed]
        if len(closed_in) != len(groups):
            raise ValueError(f"closed has {len(closed_in)} flags for {len(groups)} "
                             "instances")
    params = dict(slice_mm=slice_mm, filter_k=filter_k, filter_std=filter_std,
                  n_instances=len(groups), n_input=len(pts), ring_mode=ring_mode,
                  n_arcs=int(n_arcs), closed_given=closed is not None)

    seams, centers_all, poses, flags = [], [], [], []
    for gi, m in enumerate(groups):
        sub = pts[m]
        if len(sub) < 2 * min_centers:
            continue
        sub = statistical_filter(sub, filter_k, filter_std)
        is_closed = band_is_closed(sub) if closed_in[gi] is None else closed_in[gi]
        as_arcs = bool(is_closed) and ring_mode == "arcs"
        centers = (arc_centerline(sub, slice_mm, n_arcs, arc_overlap_deg) if as_arcs
                   else pca_centerline(sub, slice_mm))
        if len(centers) < min_centers:
            continue
        path = bspline_path(centers, n_samples, closed=as_arcs)
        if path is None:
            continue
        seams.append(path)
        centers_all.append(centers)
        flags.append(as_arcs)
        if plan_poses:
            poses.append(torch_poses(path, sub, seed=seed + gi))
    params["n_closed"] = int(sum(flags))
    note = "" if seams else "no instance produced enough slice centres"
    return PCASliceResult(seams, centers_all, flags, poses, params, used_oracle, note)
