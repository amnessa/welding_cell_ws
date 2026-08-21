"""`lit-ransac` — improved multi-plane RANSAC → plane intersections → weld path and pose.

A faithful reimplementation of the extractor in:

    Jinxin Yi, Xuan Kong, Hao Tang, Jie Zhang, Zhenming Chen, Lu Deng.
    "Weld seam extraction and path generation for robotic welding of steel structures
    based on 3D vision." *Automation in Construction* 183 (2026) 106792.
    https://doi.org/10.1016/j.autcon.2026.106792

Their pipeline has four stages (paper §2.2). Only the last two are a seam extractor; the
first two are how they *obtain* a clean cloud, and this dataset supplies that directly:

    §3  PointNet++ segmentation of the weld region   -> `seam_region_oracle` (an ORACLE)
    §4  multi-scale ICP registration of many views   -> `cloud_for(view="full")`
    §5  improved RANSAC multi-plane fitting          -> `multi_plane_fit`, `optimize_plane`
    §6  intersection lines, endpoints, torch pose    -> `seams_from_planes`

Replacing §3 with an oracle is the single most consequential choice here and it is why
`seam_region_oracle` is a separate, opt-in function rather than folded into `detect`. Their
network segments a ~40 mm-wide band around every seam at 95,07% mIoU, and the plane fitter
downstream is *only ever shown those points*. Run `detect` on a whole workpiece instead and
every plate's own top-face/edge-face pair is an orthogonal intersecting pair too, so the
method emits a seam along every plate border. That is not a bug in the reimplementation —
it is the L0->L1 delta the plan asks each method to report (`dataset_plan.md` §Phase 4,
"the oracle ladder"), and it is the same shape of dependency as `ours` on `object_id`.

The method is **randomised**. Every number it produces is a distribution, not a value; the
plan requires 100 repeats and a box plot for exactly this reason. `seed` is threaded
through so a repeat harness controls it.

Everything is **millimetres**, as everywhere else in `weld_generator`.

Published constants, used as defaults
-------------------------------------
    T_d2  = 2,0 mm    inlier distance threshold      (§5.2, sensitivity table 3)
    T_mpp = 0,025     minimum inlier occupancy ratio (§5.3, eq. 17)
    eta_0 = 0,7       RANSAC confidence level        (§5.3, eq. 18)
    ~40 mm            annotated weld-region width    (§3)

Deviations, and why each one exists
-----------------------------------
Each is behind a switch, defaults named, so a run can be made literal and the difference
measured rather than argued.

1. **Iteration count (eq. 18).** As printed, `(1 - (3/N_fp)^3)^K < 1 - eta_0`. Read
   literally, a 10 000-point subset needs K ~ 4e10 iterations, which contradicts the
   paper's own 0,187 s runtime in Table 6. The intended form is standard adaptive RANSAC
   with the inlier ratio `w = n_in / N_fp` in place of `3/N_fp`; that gives K ~ 44 at
   w = 0,3 and matches both the timing and the claim that iteration counts are
   "automatically determined". Implemented as `iteration_rule="adaptive"`;
   `iteration_rule="literal"` is available and will hit `max_iterations` every time.

2. **Plane refit (eq. 19).** §5.4 says: take the plane's centroid, query its KD-tree
   neighbourhood, pick **two random points** in it, and use one cross product as the
   refined normal. The *substance* of the idea is sound and is the reason it works — a
   neighbourhood of the centroid excludes the over-segmented points near the plate
   intersections, which are what contaminate the original fit. The *estimator* is not: one
   cross product from two random points is higher-variance than the RANSAC plane it
   replaces, and cannot yield the 0,011 deg residual reported in Fig. 16. Default
   `plane_refit="centroid_lstsq"` keeps the neighbourhood restriction and replaces the two
   points with a PCA fit over it. `"centroid_3pt"` is eq. 19 literally; `"none"` skips §5.4.

3. **Point on the intersection line (eq. 22)** fixes `z = 0` and solves a 2x2 system, which
   is singular whenever the seam runs parallel to the xy-plane — the commonest case there
   is. Implemented as eq. 22 but with the eliminated axis chosen by conditioning (z first,
   as printed). Same line, same parametrisation; only the reference point differs.

4. **Triple-point coordinate bound (§6.2)** rejects intersections outside
   `|coord| in [0.001, 1000]`. That test is frame-dependent: it discards a legitimate
   corner that happens to sit near this dataset's world origin, which is where the
   generator places joints. The guard's actual purpose — reject the runaway intersection
   of two near-parallel plates — is kept frame-independently, as a bound on distance
   outside the cloud's own bounding box. `coord_bounds_mm=(1e-3, 1e3)` restores the literal
   test.

5. **Pair support.** The paper never asks whether two fitted planes meet *within the data*,
   because after §3 every plane is a seam-local patch and they always do. On an
   unsegmented cloud two planes on opposite sides of a workpiece still define a line, in
   mid-air. `min_pair_support` points from each plane within `support_radius_mm` of the
   line is the minimum check that makes §6 well-posed off their segmented input.

6. **Torch frame handedness (eq. 24).** `X_t = X_s, Z_t = -Z_s, Y_t = X_t x Z_t` gives
   `det = -1` — a reflection, not a rotation. `right_handed=True` (default) flips `Y_t`;
   `False` returns the matrix exactly as printed. The seam *geometry* is unaffected either
   way, and the geometry is what the Phase 4 metrics currently score.

What the mechanism cannot express
---------------------------------
§6 opens "for each pair of orthogonal steel plates, the intersection line represents a
fillet weld seam". Two coplanar faces (butt) and two parallel faces (edge) have `n1 x n2`
of zero: no intersection line exists, so no amount of tuning produces a seam. That is the
coverage prediction in `dataset_plan.md` §4, and this module is built so the prediction is
*measured* — `LitRansacResult.note` records when a scene yielded no plane pair at all.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:                                    # pragma: no cover
    cKDTree = None

from .radius_pca import voxel_downsample

# Published values (§5.2, §5.3, §3). Named so a call site that changes one is visible.
T_D2_MM = 2.0
T_MPP = 0.025
ETA_0 = 0.7
SEAM_REGION_HALF_WIDTH_MM = 20.0


# --------------------------------------------------------------------------------
# §3 stand-in — the segmentation this method is normally handed
# --------------------------------------------------------------------------------

def seam_region_oracle(pts: np.ndarray, gt_polylines, half_width_mm: float = None,
                       end_margin_mm: float = 0.0) -> np.ndarray:
    """Boolean mask of the weld region around the truth seams. **An oracle.**

    Stands in for the trained PointNet++ of §3, which annotates "the weld seam
    neighborhood ... with the region width maintained at approximately 40 mm" and reaches
    95,07% mIoU. Passing this mask is the L0 arm: it gives `lit-ransac` the input its
    authors give it, and it is the only way the published numbers are reproducible.

    Withholding it is the L1 arm. The gap between the two is this method's dependency on a
    segmentation stage its plane-fitting contribution is never evaluated without — which is
    the whole point of measuring it rather than describing it.

    Args:
        half_width_mm: half of §3's annotated *width*, measured **across** the seam.
        end_margin_mm: how far the annotation runs past each seam **end**, along it.

    `end_margin_mm = 0` is not a detail. A plain "within 20 mm of the seam" test is a
    *capsule*, and a capsule reaches 20 mm beyond the last point of the seam — sweeping up
    base-plate material that no annotator would call weld region. §6.2 then reads the seam's
    extent off the points it was given, so it reads the capsule's extent instead of the
    plate's: measured on a T joint, the seam came back **38 mm too long, 19 mm at each end**,
    while its *line* was accurate to 0,6 mm. A round end on the mask is enough to destroy
    the length asymmetry between the two plates that §6.2 depends on, so the width and the
    extent are separate parameters here and the extent defaults to no margin at all.
    """
    from .metrics import distance_to_polylines
    hw = SEAM_REGION_HALF_WIDTH_MM if half_width_mm is None else float(half_width_mm)
    pts = np.asarray(pts, dtype=float)
    polys = [np.asarray(g, dtype=float) for g in gt_polylines if len(np.asarray(g)) >= 2]
    if len(pts) == 0 or not polys:
        return np.zeros(len(pts), dtype=bool)
    if cKDTree is None:                                # pragma: no cover
        return distance_to_polylines(pts, polys) <= hw

    # Exact point-to-polyline distance is the wrong tool here. `distance_to_polylines`
    # loops in Python over every segment, and the stored seams carry ~2000 vertices each,
    # so masking a 200 k-point cloud is 400 M segment evaluations - minutes per scene, which
    # is a corpus sweep that never finishes. A KD-tree over the truth curve resampled at
    # `step` costs one query and errs by at most `step/2`, against a 20 mm half-width. The
    # metrics keep the exact form, where the query set is a few hundred points.
    step = max(hw / 40.0, 0.5)
    samples, is_end, tangent = [], [], []
    for poly in polys:
        S = _resample(poly, step)
        samples.append(S)
        flag = np.zeros(len(S), dtype=bool)
        flag[0] = flag[-1] = True
        is_end.append(flag)
        t = np.zeros_like(S)
        t[0] = _unit(S[min(1, len(S) - 1)] - S[0])
        t[-1] = _unit(S[-1] - S[max(0, len(S) - 2)])
        tangent.append(t)
    S = np.vstack(samples)
    is_end = np.concatenate(is_end)
    tangent = np.vstack(tangent)

    dist, idx = cKDTree(S).query(pts, k=1, workers=-1)
    keep = dist <= hw
    # Points whose nearest truth sample is a seam END project past it. Their offset splits
    # into an across-seam part (already bounded by `hw`) and an along-seam part, which is
    # what `end_margin_mm` bounds.
    past = keep & is_end[idx]
    if past.any():
        axial = np.abs(np.einsum("ij,ij->i", pts[past] - S[idx[past]], tangent[idx[past]]))
        keep[past] = axial <= float(end_margin_mm)
    return keep


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros(3)


def _resample(poly: np.ndarray, step_mm: float) -> np.ndarray:
    """Polyline resampled at a fixed spacing, endpoints preserved."""
    from .metrics import _sample_polyline
    S = _sample_polyline(poly, step_mm)
    if len(S) == 0:
        return poly
    if np.linalg.norm(S[-1] - poly[-1]) > 1e-9:
        S = np.vstack([S, poly[-1]])
    return S


# --------------------------------------------------------------------------------
# §5.1 — the original RANSAC primitives, eqs. 13-16
# --------------------------------------------------------------------------------

def plane_from_three_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray
                            ) -> tuple[np.ndarray, float] | None:
    """`(n, d)` with `|n| = 1` and `n.x + d = 0`. Eqs. 13-14. `None` if collinear."""
    n = np.cross(np.asarray(p2, float) - p1, np.asarray(p3, float) - p1)
    norm = float(np.linalg.norm(n))
    if norm < 1e-12:
        return None
    n = n / norm
    return n, float(-n @ np.asarray(p1, float))


def point_plane_distance(pts: np.ndarray, n: np.ndarray, d: float) -> np.ndarray:
    """Eq. 16. `n` is expected unit-length, so this is `|n.p + d|`."""
    return np.abs(np.asarray(pts, float) @ np.asarray(n, float) + float(d))


def adaptive_iterations(n_inliers: int, n_points: int, confidence: float = ETA_0,
                        rule: str = "adaptive", cap: int = 100_000) -> int:
    """`K` from eq. 18. See deviation 1 in the module docstring for the two readings.

    `"adaptive"` substitutes the observed inlier ratio `w = n_in / N_fp` for the printed
    `3 / N_fp`, giving the standard `K = log(1-eta) / log(1 - w^3)`. `"literal"` uses the
    equation as typeset and returns something on the order of 1e10, which is why it exists
    only to be quoted, not run.
    """
    if n_points < 3:
        return 1
    w = (3.0 / n_points) if rule == "literal" else (n_inliers / n_points)
    w = float(np.clip(w, 0.0, 1.0))
    p = w ** 3
    if p <= 0.0:
        return cap
    if p >= 1.0:
        return 1
    k = np.log(max(1.0 - confidence, 1e-12)) / np.log(1.0 - p)
    return int(min(cap, max(1, np.ceil(k))))


def ransac_plane(pts: np.ndarray, dist_thresh_mm: float = T_D2_MM,
                 confidence: float = ETA_0, rng: np.random.Generator | None = None,
                 max_iterations: int = 1000, min_iterations: int = 10,
                 iteration_rule: str = "adaptive"
                 ) -> tuple[np.ndarray, float, np.ndarray, int]:
    """The dominant plane of `pts`: `(n, d, inlier_mask, iterations_run)`. §5.1 + eq. 18.

    Three points sampled uniformly, plane by cross product, inliers by eq. 16 under
    `T_d2`, keep the maximum-inlier hypothesis. The only addition over §5.1 is that the
    iteration budget is recomputed from the best-so-far inlier ratio each time the best
    improves, which is what §5.3 says the confidence level is for.
    """
    pts = np.asarray(pts, dtype=float)
    n_pts = len(pts)
    rng = np.random.default_rng() if rng is None else rng
    best_n = np.array([0.0, 0.0, 1.0])
    best_d = 0.0
    best_mask = np.zeros(n_pts, dtype=bool)
    if n_pts < 3:
        return best_n, best_d, best_mask, 0

    budget = max_iterations
    i = 0
    while i < min(budget, max_iterations):
        i += 1
        idx = rng.choice(n_pts, size=3, replace=False)
        plane = plane_from_three_points(pts[idx[0]], pts[idx[1]], pts[idx[2]])
        if plane is None:
            continue
        n, d = plane
        mask = point_plane_distance(pts, n, d) < dist_thresh_mm
        if int(mask.sum()) > int(best_mask.sum()):
            best_n, best_d, best_mask = n, d, mask
            budget = max(min_iterations,
                         adaptive_iterations(int(mask.sum()), n_pts, confidence,
                                             iteration_rule, max_iterations))
    return best_n, best_d, best_mask, i


# --------------------------------------------------------------------------------
# §5.3-5.4 — the improvement, and the centroid refit
# --------------------------------------------------------------------------------

@dataclass
class Plane:
    """One fitted plane: `n.x + d = 0`, plus the points that voted for it."""

    normal: np.ndarray                    #: unit normal, sign arbitrary
    d: float                              #: offset, `n.x + d = 0`
    inliers: np.ndarray                   #: indices into the cloud handed to `multi_plane_fit`
    centroid: np.ndarray                  #: `p_o` of §5.4
    n_inliers: int
    inlier_ratio: float                   #: against `N_seg`, the eq. 17 denominator
    iterations: int = 0
    refit: str = "none"

    def distance(self, pts: np.ndarray) -> np.ndarray:
        return point_plane_distance(pts, self.normal, self.d)


def optimize_plane(pts: np.ndarray, plane: Plane, mode: str = "centroid_lstsq",
                   k_neighbors: int = 200, rng: np.random.Generator | None = None
                   ) -> Plane:
    """§5.4 — refit a plane from a neighbourhood of its own centroid.

    Over-segmentation puts points from an adjacent plate into a plane's inlier set, and
    those points are all near the intersection, i.e. far from the centroid. Restricting the
    refit to the `k_neighbors` points nearest `p_o` therefore drops precisely the
    contaminated ones. That restriction is the method; see deviation 2 for why the default
    estimator over that neighbourhood is a PCA fit rather than eq. 19's two random points.
    """
    if mode == "none" or len(plane.inliers) < 3:
        return plane
    sub = np.asarray(pts, float)[plane.inliers]
    p_o = sub.mean(axis=0)

    k = int(min(k_neighbors, len(sub)))
    if cKDTree is not None:
        _, idx = cKDTree(sub).query(p_o, k=k)
        neigh = sub[np.atleast_1d(idx)]
    else:                                              # pragma: no cover
        order = np.argsort(np.linalg.norm(sub - p_o, axis=1))[:k]
        neigh = sub[order]

    if mode == "centroid_3pt":
        rng = np.random.default_rng() if rng is None else rng
        if len(neigh) < 2:
            return plane
        j = rng.choice(len(neigh), size=2, replace=False)
        n = np.cross(neigh[j[0]] - p_o, neigh[j[1]] - p_o)   # eq. 19
        if np.linalg.norm(n) < 1e-12:
            return plane
        n = n / np.linalg.norm(n)
        d = float(-n @ p_o)
    elif mode == "centroid_lstsq":
        if len(neigh) < 3:
            return plane
        c = neigh.mean(axis=0)
        _, _, vt = np.linalg.svd(neigh - c, full_matrices=False)
        n = vt[-1]
        d = float(-n @ c)
    else:
        raise ValueError(f"unknown plane_refit {mode!r}")

    if n @ plane.normal < 0:                           # keep the original orientation
        n, d = -n, -d
    return Plane(normal=n, d=d, inliers=plane.inliers, centroid=p_o,
                 n_inliers=plane.n_inliers, inlier_ratio=plane.inlier_ratio,
                 iterations=plane.iterations, refit=mode)


def multi_plane_fit(pts: np.ndarray, dist_thresh_mm: float = T_D2_MM,
                    min_inlier_ratio: float = T_MPP, confidence: float = ETA_0,
                    seed: int = 0, max_planes: int = 20, max_iterations: int = 1000,
                    iteration_rule: str = "adaptive",
                    plane_refit: str = "centroid_lstsq", refit_k: int = 200
                    ) -> list[Plane]:
    """§5.3, the flowchart of Fig. 14: fit, remove inliers, repeat until eq. 17 fails.

    `min_inlier_ratio` is tested against `N_seg`, the size of the **original** cloud, not
    the shrinking remainder — that is how eq. 17 is written, and it is what makes the
    termination meaningful: a plane must be 2,5% of the workpiece to be a plate rather
    than a slab of noise.
    """
    pts = np.asarray(pts, dtype=float)
    n_seg = len(pts)
    rng = np.random.default_rng(seed)
    remaining = np.arange(n_seg)
    planes: list[Plane] = []

    while len(remaining) >= 3 and len(planes) < max_planes:
        n, d, mask, iters = ransac_plane(pts[remaining], dist_thresh_mm, confidence, rng,
                                         max_iterations, iteration_rule=iteration_rule)
        n_in = int(mask.sum())
        if n_in < 3 or n_in / n_seg <= min_inlier_ratio:      # eq. 17
            break
        idx = remaining[mask]
        planes.append(optimize_plane(
            pts, Plane(normal=n, d=d, inliers=idx, centroid=pts[idx].mean(axis=0),
                       n_inliers=n_in, inlier_ratio=n_in / n_seg, iterations=iters),
            plane_refit, refit_k, rng))
        remaining = remaining[~mask]
    return planes


# --------------------------------------------------------------------------------
# §6.1 — intersection line, tool approach vector, torch pose
# --------------------------------------------------------------------------------

def intersection_line(a: Plane, b: Plane, parallel_tol: float = 1e-6
                      ) -> tuple[np.ndarray, np.ndarray] | None:
    """`(point, unit_direction)` of the line where two planes meet. Eqs. 20-22.

    `None` when `|n1 x n2|` is below `parallel_tol` — the coplanar and parallel cases, i.e.
    every butt and edge joint in the taxonomy. This return is the coverage prediction.
    """
    d_weld = np.cross(a.normal, b.normal)              # eq. 21
    mag = float(np.linalg.norm(d_weld))
    if mag < parallel_tol:
        return None
    d_weld = d_weld / mag

    # eq. 22 eliminates z. Do the same, but eliminate whichever axis leaves the
    # best-conditioned 2x2 - z first, so the printed choice wins whenever it is usable.
    A = np.stack([a.normal, b.normal])
    rhs = np.array([-a.d, -b.d])
    for drop in (2, 1, 0):
        keep = [i for i in range(3) if i != drop]
        M = A[:, keep]
        det = float(np.linalg.det(M))
        if abs(det) > 1e-9:
            sol = np.linalg.solve(M, rhs)
            p = np.zeros(3)
            p[keep] = sol
            return p, d_weld
    return None                                        # pragma: no cover


def torch_pose(d_weld: np.ndarray, p_line: np.ndarray, a: Plane, b: Plane,
               right_handed: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """`(approach_vector, R_torch)`. Eqs. 23-24.

    `v1` and `v2` are the in-plane directions perpendicular to the seam, taken from the
    line toward each plane's own centroid — that is Fig. 17(a)'s pair of vectors pointing
    out of the corner along each plate. Their sum `v_0` is the outward bisector, and the
    torch approaches along `-v_0`. See deviation 6 on the handedness of eq. 24.
    """
    def in_plane(pl: Plane) -> np.ndarray:
        v = pl.centroid - p_line
        v = v - (v @ d_weld) * d_weld
        norm = float(np.linalg.norm(v))
        return v / norm if norm > 1e-12 else np.zeros(3)

    v0 = in_plane(a) + in_plane(b)                     # eq. 23
    norm = float(np.linalg.norm(v0))
    v0 = v0 / norm if norm > 1e-12 else np.array([0.0, 0.0, 1.0])

    X_s, Z_s = d_weld, v0
    X_t = X_s
    Z_t = -Z_s
    Y_t = np.cross(X_t, Z_t)                           # eq. 24, as printed: det = -1
    if right_handed:
        Y_t = -Y_t
    return v0, np.column_stack([X_t, Y_t, Z_t])


def triple_intersection(a: Plane, b: Plane, c: Plane, bbox: tuple[np.ndarray, np.ndarray],
                        max_extrapolation_mm: float = 50.0,
                        coord_bounds_mm: tuple[float, float] | None = None
                        ) -> np.ndarray | None:
    """Common point of three planes. Eq. 25, with the §6.2 validity guard.

    The guard's stated purpose is to throw out the runaway intersection produced when two
    plates are *nearly* parallel. Default here is the frame-independent form of that test —
    the point must lie inside the cloud's bounding box, grown by `max_extrapolation_mm`.
    Pass `coord_bounds_mm=(1e-3, 1e3)` for §6.2 as printed; see deviation 4.
    """
    M = np.stack([a.normal, b.normal, c.normal])
    if abs(float(np.linalg.det(M))) < 1e-9:
        return None
    p = np.linalg.solve(M, np.array([-a.d, -b.d, -c.d]))   # eq. 25
    if not np.all(np.isfinite(p)):
        return None
    if coord_bounds_mm is not None:
        lo, hi = coord_bounds_mm
        if not np.all((np.abs(p) >= lo) & (np.abs(p) <= hi)):
            return None
    lo_b, hi_b = bbox
    if np.any(p < lo_b - max_extrapolation_mm) or np.any(p > hi_b + max_extrapolation_mm):
        return None
    return p


# --------------------------------------------------------------------------------
# §6.2 — endpoints
# --------------------------------------------------------------------------------

def _project_onto_line(pts: np.ndarray, origin: np.ndarray, direction: np.ndarray
                       ) -> np.ndarray:
    """`t_N` of eq. 27 for each point — the signed arclength along `v_ij`."""
    return (np.asarray(pts, float) - origin) @ direction


def seam_endpoints(pts: np.ndarray, a: Plane, b: Plane, p_line: np.ndarray,
                   d_weld: np.ndarray, corner: np.ndarray | None = None,
                   source: str = "smaller_plane", support_radius_mm: float = 3 * T_D2_MM
                   ) -> tuple[np.ndarray, np.ndarray] | None:
    """The two endpoints of one seam. Eqs. 26-28.

    §6.2 projects "the vertical point clouds" onto the seam direction. Operationally that
    is the plate that *terminates* at the seam — the one whose extent along `d_weld` is the
    shorter of the two, since the base plate always overruns. `source` selects that
    (`"smaller_plane"`, the default), both planes (`"both"`), or the overlap of the two
    projected ranges (`"overlap"`, the physically correct extent and a documented
    improvement rather than the paper's rule).

    With a valid `corner` from eq. 25 on this line, §6.2's rule applies: the corner is one
    endpoint and the farthest projection from it is the other. Without one, the endpoints
    are the two projections at maximum separation.
    """
    pts = np.asarray(pts, float)
    ts = []
    for pl in (a, b):
        sub = pts[pl.inliers]
        if len(sub) == 0:
            return None
        near = sub[np.linalg.norm(np.cross(sub - p_line, d_weld), axis=1)
                   <= support_radius_mm]
        t = _project_onto_line(near if len(near) else sub, p_line, d_weld)
        ts.append(t)

    if source == "both":
        t_all = np.concatenate(ts)
    elif source == "overlap":
        lo = max(ts[0].min(), ts[1].min())
        hi = min(ts[0].max(), ts[1].max())
        if hi <= lo:
            return None
        t_all = np.array([lo, hi])
    elif source == "smaller_plane":
        spans = [float(t.max() - t.min()) for t in ts]
        t_all = ts[int(np.argmin(spans))]
    else:
        raise ValueError(f"unknown endpoint source {source!r}")

    if len(t_all) < 2:
        return None
    t_lo, t_hi = float(t_all.min()), float(t_all.max())

    if corner is not None:
        t_c = float(_project_onto_line(corner[None, :], p_line, d_weld)[0])
        if t_lo - 1e-6 <= t_c <= t_hi + 1e-6:
            far = t_lo if abs(t_lo - t_c) > abs(t_hi - t_c) else t_hi
            return p_line + t_c * d_weld, p_line + far * d_weld

    if t_hi - t_lo < 1e-6:
        return None
    return p_line + t_lo * d_weld, p_line + t_hi * d_weld


# --------------------------------------------------------------------------------
# result and driver
# --------------------------------------------------------------------------------

@dataclass
class LitRansacSeam:
    """One extracted weld: geometry from §6.1-6.2, pose from eq. 24."""

    p0: np.ndarray
    p1: np.ndarray
    direction: np.ndarray                 #: `d_weld`, eq. 21
    approach: np.ndarray                  #: `v_0`, eq. 23 — the outward bisector
    R_torch: np.ndarray                   #: eq. 24
    plane_ids: tuple[int, int]
    #: Fold between the two faces. RANSAC normals have arbitrary sign, so only the
    #: unsigned inter-normal angle is recoverable; this is `180 - |angle(n1, n2)|`, i.e.
    #: 90 deg for a square fillet and 180 deg for coplanar faces, and it cannot tell a
    #: 60 deg fold from a 120 deg one.
    dihedral_deg: float
    from_corner: bool = False             #: an endpoint came from a triple point, eq. 25

    @property
    def polyline(self) -> np.ndarray:
        return np.stack([self.p0, self.p1])

    @property
    def length_mm(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))


@dataclass
class LitRansacResult:
    """What `lit-ransac` returned, and enough context to know what it means."""

    seams: list[LitRansacSeam]
    planes: list[Plane]
    #: Every plane pair §6 considered, and what became of it: `{i, j, fold_deg, status}`
    #: with status one of `seam`, `parallel`, `not_orthogonal`, `no_support`,
    #: `no_endpoints`, `too_short`. The coverage claim in `dataset_plan.md` §4 is a claim
    #: about *one* of these values, so it is recorded rather than described - an edge joint
    #: rejecting every pair as `parallel` is the mechanism limit, in data.
    pairs: list[dict[str, Any]] = field(default_factory=list)
    params: dict[str, Any] = field(default_factory=dict)
    #: True when `seam_region_oracle` (or any externally supplied mask) restricted the
    #: input. The published pipeline always has this; a number produced without it is a
    #: different experiment and must be labelled as one.
    used_segmentation_oracle: bool = False
    note: str = ""
    #: The cloud §5 actually fitted — after the segmentation mask and the voxel merge, so
    #: `Plane.inliers` indexes into THIS and not into what the caller passed. Kept because
    #: a plane is only interpretable next to the points that voted for it.
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))

    @property
    def polylines(self) -> list[np.ndarray]:
        """The prediction in the shape `metrics.evaluate` wants."""
        return [s.polyline for s in self.seams]

    @property
    def n_seams(self) -> int:
        return len(self.seams)


def detect(pts: np.ndarray,
           dist_thresh_mm: float = T_D2_MM,
           min_inlier_ratio: float = T_MPP,
           confidence: float = ETA_0,
           seed: int = 0,
           segmentation_mask: np.ndarray | None = None,
           voxel_mm: float | None = None,
           prefilter_density_per_mm2: float | None = None,
           max_planes: int = 20,
           max_iterations: int = 1000,
           iteration_rule: str = "adaptive",
           plane_refit: str = "centroid_lstsq",
           refit_k: int = 200,
           pair_rule: str = "orthogonal",
           orthogonal_tol_deg: float = 30.0,
           parallel_tol: float = 1e-6,
           support_radius_mm: float | None = None,
           min_pair_support: int = 10,
           endpoint_source: str = "smaller_plane",
           min_seam_length_mm: float = 5.0,
           max_extrapolation_mm: float = 50.0,
           coord_bounds_mm: tuple[float, float] | None = None,
           right_handed: bool = True) -> LitRansacResult:
    """Run `lit-ransac` end to end: §5 multi-plane fit, then §6 paths and poses.

    Args:
        pts: (N,3) cloud, millimetres. `cloud_for(view="full")` is the closest analogue of
            the paper's input, which is a multi-view registered reconstruction (§4).
        dist_thresh_mm: `T_d2`. The paper's 2,0 mm comes from a sensitivity sweep on a
            0,64 mm-resolution cloud with 6 mm plate, and §5.2 is explicit that it must
            exceed the surface deviation and stay below the plate thickness. **That is a
            validity window of the same shape as the one `ours` has**, on a different
            quantity — worth sweeping rather than accepting.
        min_inlier_ratio: `T_mpp`, eq. 17. Against the original cloud size.
        confidence: `eta_0`, eq. 18. See `adaptive_iterations` and deviation 1.
        segmentation_mask: boolean per-point mask restricting the input, standing in for
            §3's PointNet++. Use `seam_region_oracle` to build it. **An oracle** — recorded
            in the result and reported alongside every number produced with it.
        prefilter_density_per_mm2: voxel-downsample to about this density first. Set it for
            any cross-scene comparison, for the same reason `ours` needs it: the generator
            samples density over 0,25-4 pts/mm², and `T_mpp` is a *ratio*, so an
            uncontrolled density silently changes what counts as a plane.
        pair_rule: `"orthogonal"` (default) applies §6's own premise, "each pair of
            orthogonal steel plates"; `"intersecting"` accepts any non-parallel pair. The
            generous reading exists so the coverage result cannot be dismissed as a
            handicap — a V-groove at 60 deg is not orthogonal but is still a fillet-like
            fold this machinery can express.
        support_radius_mm: how close a plane's points must come to the intersection line
            for that line to count as a real edge. Default `3 x dist_thresh_mm`. See
            deviation 5 — off the segmented input this check is what stops mid-air seams.
        min_pair_support: points required from *each* plane within `support_radius_mm`.
        endpoint_source: see `seam_endpoints`.
        min_seam_length_mm: drop slivers. A pair of nearly-parallel planes that survives
            `parallel_tol` produces a line with almost no support along it.
    """
    pts = np.asarray(pts, dtype=float)
    used_oracle = segmentation_mask is not None
    if used_oracle:
        pts = pts[np.asarray(segmentation_mask, dtype=bool)]

    if prefilter_density_per_mm2 and voxel_mm is None:
        voxel_mm = 1.0 / np.sqrt(float(prefilter_density_per_mm2))
    if voxel_mm:
        pts = voxel_downsample(pts, float(voxel_mm))

    support_radius_mm = (3.0 * dist_thresh_mm if support_radius_mm is None
                         else float(support_radius_mm))
    params = dict(dist_thresh_mm=dist_thresh_mm, min_inlier_ratio=min_inlier_ratio,
                  confidence=confidence, seed=seed, iteration_rule=iteration_rule,
                  plane_refit=plane_refit, pair_rule=pair_rule,
                  orthogonal_tol_deg=orthogonal_tol_deg,
                  support_radius_mm=support_radius_mm, min_pair_support=min_pair_support,
                  endpoint_source=endpoint_source, voxel_mm=voxel_mm, n_input=len(pts))

    if len(pts) < 3:
        return LitRansacResult([], [], [], params, used_oracle,
                               "cloud too small to fit a plane", points=pts)

    planes = multi_plane_fit(pts, dist_thresh_mm, min_inlier_ratio, confidence, seed,
                             max_planes, max_iterations, iteration_rule, plane_refit,
                             refit_k)
    if len(planes) < 2:
        return LitRansacResult([], planes, [], params, used_oracle,
                               f"{len(planes)} plane(s) fitted; §6 needs a pair", points=pts)

    bbox = (pts.min(axis=0), pts.max(axis=0))
    # eq. 25 first: the triple points are shared across pairs, so compute them once.
    corners: list[np.ndarray] = []
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            for k in range(j + 1, len(planes)):
                c = triple_intersection(planes[i], planes[j], planes[k], bbox,
                                        max_extrapolation_mm, coord_bounds_mm)
                if c is not None:
                    corners.append(c)

    seams: list[LitRansacSeam] = []
    pairs: list[dict[str, Any]] = []

    def verdict(i, j, fold_deg, status):
        pairs.append({"i": i, "j": j, "fold_deg": fold_deg, "status": status})

    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            a, b = planes[i], planes[j]
            cos = abs(float(np.clip(a.normal @ b.normal, -1.0, 1.0)))
            fold_deg = float(np.degrees(np.arccos(cos)))          # 0 = parallel faces

            line = intersection_line(a, b, parallel_tol)
            if line is None:
                verdict(i, j, fold_deg, "parallel")
                continue
            p_line, d_weld = line

            if pair_rule not in ("orthogonal", "intersecting"):
                raise ValueError(f"unknown pair_rule {pair_rule!r}")
            if pair_rule == "orthogonal" and abs(fold_deg - 90.0) > orthogonal_tol_deg:
                verdict(i, j, fold_deg, "not_orthogonal")
                continue

            # deviation 5 — does this line touch both plates, or is it in mid-air?
            support_ok = True
            for pl in (a, b):
                sub = pts[pl.inliers]
                dist = np.linalg.norm(np.cross(sub - p_line, d_weld), axis=1)
                if int((dist <= support_radius_mm).sum()) < min_pair_support:
                    support_ok = False
                    break
            if not support_ok:
                verdict(i, j, fold_deg, "no_support")
                continue

            corner = None
            for c in corners:
                if float(np.linalg.norm(np.cross(c - p_line, d_weld))) <= support_radius_mm:
                    corner = c
                    break

            ends = seam_endpoints(pts, a, b, p_line, d_weld, corner, endpoint_source,
                                  support_radius_mm)
            if ends is None:
                verdict(i, j, fold_deg, "no_endpoints")
                continue
            p0, p1 = ends
            if float(np.linalg.norm(p1 - p0)) < min_seam_length_mm:
                verdict(i, j, fold_deg, "too_short")
                continue
            verdict(i, j, fold_deg, "seam")

            v0, R = torch_pose(d_weld, p_line, a, b, right_handed)
            seams.append(LitRansacSeam(p0=p0, p1=p1, direction=d_weld, approach=v0,
                                       R_torch=R, plane_ids=(i, j),
                                       dihedral_deg=180.0 - fold_deg,
                                       from_corner=corner is not None))

    note = ""
    if not seams:
        # Say which gate did the rejecting, not which one was expected to. Two faces that
        # are parallel *in the geometry* are rarely parallel in the FIT - RANSAC on sampled
        # points leaves a fraction of a degree between them, enough that `n1 x n2` is not
        # numerically zero and the line exists, in mid-air. It is the orthogonality gate
        # that then throws it out, at a fold angle near 0. Reporting "0 pairs parallel"
        # for a joint whose faces are all parallel is exactly the wrong summary.
        census = {}
        for pr in pairs:
            census[pr["status"]] = census.get(pr["status"], 0) + 1
        detail = ", ".join(f"{v} {k}" for k, v in sorted(census.items()))
        note = f"no seam from {len(planes)} planes; {len(pairs)} pair(s): {detail}"
    return LitRansacResult(seams, planes, pairs, params, used_oracle, note, points=pts)
