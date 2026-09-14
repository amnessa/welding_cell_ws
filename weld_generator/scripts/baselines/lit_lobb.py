"""`lit-lobb` — LOBB flatness, nonlinear activation, hierarchical K-means, polynomial fit.

Two papers by the same group, and both are needed. The **pipeline** is:

    Yuankai Zhang, Yusen Geng, Xincheng Tian, Yujie Sun, Xiaolong Xu.
    "A novel weld seam extraction method with semantic segmentation and point cloud
    feature for irregular structure workpieces."
    *Robotics and Computer-Integrated Manufacturing* 95 (2025) 102987.

and the **feature descriptor** it calls, its ref. [24], is:

    Yuankai Zhang, Yusen Geng, Xincheng Tian, Fuquan Zheng, Yong Jiang, Min Lai.
    "A Feature Extraction Approach Over Workpiece Point Clouds for Robotic Welding."
    *IEEE Transactions on Automation Science and Engineering* 22 (2025) 75-84.

Stages, and which of them this dataset supplies:

    RCIM §3.1  K-Net semantic segmentation of the 2D image into per-COMPONENT masks
               ("A" and "B"), 97,35% mIoU              -> `part_labels_oracle` (an ORACLE)
    RCIM §3.2  edge pixels = a neighbourhood holding two mask values (eq. 2), lifted to
               3D through the depth image              -> `part_boundary_points`
    RCIM §3.3  adaptive ROI by shape extension: edge cloud as seeds, k-NN expansion into
               the downsampled cloud, dedupe           -> `roi_by_extension`
    TASE §III  LOBB: PCA frame, bounding box (l, w, h), flatness / centre offset / corner
               entropy                                 -> `lobb_features`
    TASE §III-C nonlinear tanh activation, then the three-layer hierarchical binary K-means
               of Fig. 9 (boundary -> crease on the non-boundary subset -> corner on the
               edge cloud)          -> `activate`, `kmeans_1d_binary`, `corner_points`
    RCIM §3.4.2 Mean-Shift density filtering to fit key points   -> `mean_shift_keypoints`
    RCIM §3.4.3 polynomial fit along the dominant axis           -> `fit_dominant_axis`

Published numbers, the reproduction target: **max error < 1,2 mm, RMSE < 0,7 mm**, under
5 s. LOBB radius **2,5 mm**, chosen in TASE §III "according to the weld width".

The third distinct coarse oracle, and the most interesting one
----------------------------------------------------------------
Each of the three implemented literature methods has a different coarse stage, and treating
them as one mask - which this repo did until it was measured - flatters some and starves
others:

| method | coarse stage produces | equivalent here |
|---|---|---|
| `lit-ransac` | a **weld-seam band**, annotated at ~40 mm width | `seam_region_oracle` |
| `lit-regiongrow` | one mask per **surface** (FastSAM) | `surface_labels_oracle` (`face_id`) |
| `lit-lobb` | one mask per **component** (K-Net, labels "A"/"B") | `part_labels_oracle` (`object_id`) |

The third row is the one that matters for the thesis. RCIM eq. 2 declares a pixel an edge
when its neighbourhood holds two different component masks - which is **exactly `ours`'
cross-object gate**, computed in 2D by a trained network rather than read from a CAD stack.
`dataset_plan.md` records that `ours` loses F1 0,75 -> 0,03 when `object_id` is withheld, and
calls that a dependency on segmentation it does not publish about. This paper is the
literature answering the same question the same way, and being explicit about it. That makes
`lit-lobb` the strongest available evidence that the dependency is a property of the problem
rather than a weakness of `ours`.

Deviations, and why each one exists
-----------------------------------
1. **Stages §3.1-3.2 are 2D and this dataset is not an image pipeline.** Their edge test runs
   on image pixels and is lifted through the depth map; here it runs directly on the point
   cloud, which is the same test without the projection. The `n x n` pixel neighbourhood
   becomes a metric radius - `edge_radius_mm` - because a pixel window has no fixed size in
   millimetres and every number in this repo does. **It is a resolution-relative radius, and
   that is the whole point**: RCIM eq. 1-2 sets `n = 3` (§4.1) on a 1280 x 1024 sensor whose
   near field of view is 220 mm, so their window is ~0,5 mm across - three SAMPLES wide, not
   three millimetres. At this generator's ~1 pt/mm2 the scale-equivalent of a `3 x 3` window
   is a ball of radius `1,5 x` the point spacing, so `edge_radius_mm=None` (the default) now
   measures the spacing and uses that; `EDGE_RADIUS_MM_LEGACY = 3,0` mm is the fixed ball
   this module used before the audit, six times too wide in sample units, and is reachable
   by passing the number explicitly.

   **What the faithful reading costs, measured:** a ROOT GAP is a distance their window
   never has to cross. In the image the two masks abut in projection; in 3D they are
   physically apart, and at 1 pt/mm2 the resolution-relative ball is ~1,1 mm - smaller than
   a 1,5 mm root gap. On `bench_phase4` `butt/line_square` (gap 1,49 mm) it therefore finds
   **zero** boundary points and the method returns "ROI too small after the coarse stage",
   where the 3,0 mm ball returned a seam (a wrong one: F1 0,00 either way). That is a
   property of the published predicate meeting a gapped joint, not a bug to paper over, and
   it is why the failure returns a `note` instead of an exception. A joint whose parts are
   further apart than the sampling window needs `edge_radius_mm` set explicitly, at which
   point the number is a JOINT parameter and must be reported as one.
2. **The K-means is 1-D and binary at every level**, which TASE §III-C states outright
   ("all feature clustering problems in this paper are binary classification problems of
   one-dimensional data"). Implemented directly rather than through `sklearn`, seeded at the
   min and max, so it is deterministic - `lit-lobb` is one of the methods that should show
   **zero seed spread** in the Phase 4 box plots, and that is only true if nothing inside it
   is randomised.
3. **Mean-Shift is stated but not parameterised.** RCIM §3.4.2 gives the Gaussian kernel and
   the iteration, and no bandwidth. Defaulted to `2 x` the LOBB radius and exposed. Its
   stated justification is a *sensor* artefact - "the density of point cloud data obtained by
   the camera typically increases significantly at the creases of weld seams", because
   creases scatter more light - which **this generator does not simulate**: `area_uniform`
   sampling is uniform by construction. So the filter cannot do here what it does there, and
   `mean_shift=False` is the honest default with the faithful path kept behind the flag. Say
   this in the write-up rather than reporting a number the mechanism could not have produced.
4. **Multi-seam.** The paper fits one weld. Connected components before fitting, shared with
   `radius_pca.connected_components`, so a T joint's two fillets come back as two curves.
5. **The hierarchy is a hierarchy** (TASE Fig. 9, p. 79), which it was not until the audit:
   the boundary split runs on the centre offset over the ROI, and the crease split then runs
   on the flatness over the **non-boundary subset only**. Computing both splits over the
   whole ROI and taking `crease & ~boundary` is a different algorithm, because a 1-D K-means
   threshold is fitted to whatever population it is given - the rim points drag the flatness
   threshold down, and face points on the far side of it come back as creases. The old path
   is `kmeans="flat"` and stays runnable so the two can be compared rather than asserted.
6. **The third layer (corner points) is implemented but OFF by default, and here is the
   measured reason.** `corner_layer=True` recomputes the corner entropy over
   `boundary | crease` - it has to be recomputed, because over the edge cloud alone a line
   point's neighbourhood is a band and a corner's is not, a distinction that does not exist
   over the full ROI - and cuts the crease components at the corners before the fit. TASE
   §III-B2 says *"on the extracted edge LINE point cloud"*, and that precondition does not
   hold here: this crease layer returns a **band several millimetres wide**, not a
   one-point-wide curve, so at a 2,5 mm LOBB radius nearly every edge neighbourhood is
   isotropic and the low-entropy ("corner") cluster swallows the seam - on the first bench
   corner scene, 1228 of 1241 crease points came back as corners and the fit lost the seam
   entirely. Making it the default would therefore be a reimplementation error, not fidelity.
   TODO to close it properly: thin the edge cloud to a curve (skeletonise, or take the
   Mean-Shift key points of §3.4.2) BEFORE the entropy layer, which is the input TASE Fig. 9
   assumes; then re-measure whether the corner cut splits the closed T perimeter that
   deviation 4 and the author correspondence both flag.

Author correspondence (Yuankai Zhang, first author, 2026-08-27)
---------------------------------------------------------------
The three implementation choices above were put to the authors and answered in writing:

1. **The oracle masks are endorsed**: supplying per-component masks from generator ground
   truth "is equivalent to using the corresponding component masks obtained from a 2D image
   segmentation network for testing the subsequent modules. Therefore, this treatment is
   acceptable." So `part_labels_oracle` is an author-accepted stand-in, not a liberty.
2. **Mean-Shift off is accepted for density-uniform data**, with the caveat that the filter
   "is intended to further ensure the stability of weld reconstruction". That caveat is why
   the Phase 4 write-up should report the `mean_shift=True` arm alongside the default, not
   silently drop it: the mechanism it was built for is absent here, but its stabilising
   side-effect is a fair thing to measure.
3. **LOBB radius confirmed**: "The selection of the LOBB radius is not a free tuning
   parameter" - it is set by the weld width, exactly how `LOBB_RADIUS_MM` is treated.
4. **The closed contact perimeter is outside their §3.4.3 fit**: for the T-joint case where
   the fillets plus the plate-end contact runs close into a loop, they answer "you may
   consider using other curve-fitting methods, such as B-spline curves". So a B-spline
   variant of `fit_dominant_axis` is an author-suggested EXTENSION, to be reported as a
   separate arm - the polynomial fit stays the faithful reimplementation.

Everything is **millimetres**.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:                                    # pragma: no cover
    cKDTree = None

from .radius_pca import connected_components, voxel_downsample

LOBB_RADIUS_MM = 2.5          # TASE §III, "according to the weld width"
ROI_K = 30                    # RCIM §3.3 k-NN expansion; no value published
POLY_DEGREE = 3               # RCIM §3.4.3 `times`; no value published
N_SAMPLES = 80                # RCIM §4.1/§4.3, "uniformly interpolated to 80 points"
EDGE_RADIUS_SCALE = 1.5       # RCIM eq. 1-2's 3 x 3 pixel window, in units of point spacing
EDGE_RADIUS_MM_LEGACY = 3.0   # the pre-audit fixed metric ball; kept reachable, see dev. 1


# --------------------------------------------------------------------------------
# RCIM §3.1-3.2 — the coarse stage
# --------------------------------------------------------------------------------

def part_labels_oracle(object_id: np.ndarray) -> np.ndarray:
    """Per-point **component** labels. **An oracle** — K-Net's "A"/"B" masks, RCIM §3.1.

    Note what this is: `object_id`, the same input `ours` takes for its cross-object gate.
    The difference is only that this paper obtains it from a trained 2D segmenter at 97,35%
    mIoU instead of from a stack of registered CAD clouds. It is the same information.
    """
    return np.asarray(object_id).astype(np.int64)


def mean_point_spacing(pts: np.ndarray, sample: int = 20_000, seed: int = 0) -> float:
    """Mean nearest-neighbour distance — the cloud's **own resolution**, in millimetres.

    RCIM eq. 1-2's neighbourhood is `n x n` PIXELS with `n = 3` (§4.1) on a 1280 x 1024
    sensor whose near field of view is 220 mm, i.e. ~0,17 mm/px — so their window is three
    samples wide, not three millimetres. The scale-equivalent of a `3 x 3` window at sample
    spacing `s` is a ball of radius ~`1,5 s`, which is what `edge_radius_for` returns. This
    is the `s`.

    Subsampled above `sample` points and seeded, so it stays **deterministic**: `lit-lobb`
    is one of the methods whose zero seed spread is a reported finding.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return 0.0
    if cKDTree is None:                                # pragma: no cover
        raise ImportError("lit-lobb needs scipy.spatial.cKDTree")
    q = pts
    if len(pts) > int(sample):
        idx = np.random.default_rng(seed).choice(len(pts), int(sample), replace=False)
        q = pts[np.sort(idx)]
    d = cKDTree(pts).query(q, k=2, workers=-1)[0][:, 1]
    d = d[np.isfinite(d)]
    return float(d.mean()) if len(d) else 0.0


def edge_radius_for(pts: np.ndarray, edge_radius_mm: float | None = None,
                    scale: float = EDGE_RADIUS_SCALE,
                    spacing: float | None = None) -> float:
    """The RCIM eq. 1-2 window as a metric ball. `None` = resolution-relative, the default.

    `edge_radius_mm=None` means *"`scale` x the measured mean point spacing"* — the faithful
    reading, because the paper's window is a pixel count and this cloud has no pixels. An
    explicit value overrides it; `EDGE_RADIUS_MM_LEGACY` (3,0 mm) is the number this module
    used before the audit, kept reachable so the old rung stays runnable. Pass `spacing` to
    reuse a measurement already taken rather than paying for a second k-d tree.
    """
    if edge_radius_mm is not None:
        return float(edge_radius_mm)
    s = float(spacing) if spacing is not None else mean_point_spacing(pts)
    return float(scale) * s if s > 0 else EDGE_RADIUS_MM_LEGACY


def part_boundary_points(pts: np.ndarray, labels: np.ndarray,
                         edge_radius_mm: float | None = None) -> np.ndarray:
    """RCIM eq. 2 in 3D: a point is an edge point when its neighbourhood holds two masks.

    Their test is over an `n x n` pixel window on the segmented image; a pixel window has no
    fixed size in millimetres, so the window becomes a metric ball — but it is a ball with a
    **resolution-relative** radius, not a fixed one. `edge_radius_mm=None` (the default)
    takes `EDGE_RADIUS_SCALE x mean_point_spacing(pts)`, which reproduces the `3 x 3` window
    of RCIM §4.1 at whatever density the cloud happens to have. Pass a float for a fixed
    ball (3,0 mm is what this module used before the audit).

    Computed as "how far to the nearest point of a different component?", which is the same
    question and the form that finishes: one tree per component instead of a Python-level
    loop over half a million neighbour lists.
    """
    pts = np.asarray(pts, dtype=float)
    edge_radius_mm = edge_radius_for(pts, edge_radius_mm)
    labels = np.asarray(labels)
    out = np.zeros(len(pts), dtype=bool)
    if len(pts) == 0:
        return out
    if cKDTree is None:                                # pragma: no cover
        raise ImportError("lit-lobb needs scipy.spatial.cKDTree")
    for lab in np.unique(labels):
        mine = labels == lab
        others = np.flatnonzero(~mine)
        if len(others) == 0:
            continue
        d, _ = cKDTree(pts[others]).query(pts[mine], k=1, workers=-1)
        idx = np.flatnonzero(mine)
        out[idx[d <= float(edge_radius_mm)]] = True
    return out


def roi_by_extension(pts: np.ndarray, seeds: np.ndarray, k: int = ROI_K) -> np.ndarray:
    """RCIM §3.3 — grow the ROI from the edge cloud by k-NN expansion, then dedupe.

    *"Using the edge point cloud set as the seed point set and the original point cloud as
    the extension point set, expansion is performed using K Nearest Neighbors."* The dedupe
    the paper describes is what a boolean mask does for free.

    Their stated advantage is worth keeping in view when the numbers are read: this
    *"eliminates the cumbersome steps of removing background and irrelevant point clouds"* —
    that is, the ROI is small because the coarse stage already told it where to look.
    """
    pts = np.asarray(pts, dtype=float)
    seeds = np.asarray(seeds, dtype=bool)
    out = seeds.copy()
    if not seeds.any():
        return out
    _, idx = cKDTree(pts).query(pts[seeds], k=min(int(k), len(pts)), workers=-1)
    out[np.unique(np.atleast_2d(idx).ravel())] = True
    return out


# --------------------------------------------------------------------------------
# TASE §III — the LOBB descriptor
# --------------------------------------------------------------------------------

def lobb_features(pts: np.ndarray, radius_mm: float = LOBB_RADIUS_MM,
                  k: int | None = None, min_neighbors: int = 4, chunk: int = 20_000
                  ) -> dict[str, np.ndarray]:
    """`flatness`, `center_offset`, `corner_entropy` per point. TASE eqs. 3-8.

    The construction: PCA of the neighbourhood gives `{e1, e2, e3}` with `λ1 >= λ2 >= λ3`;
    the neighbourhood's **extents** along those axes are the box's `l`, `w`, `h`; the origin
    moves to the box corner. Then

        flatness       f = h / sqrt(l^2 + w^2)              (eq. 4)
        centre offset  d = |(x_Q - l/2, y_Q - w/2)|          (eq. 7)
        corner entropy s = std(l, w, h)                      (eq. 8)

    TASE §III-A is explicit that the box is *"an OBB with length l, width w, and height h
    for set P, where **l >= w >= h**"*. Eigen*value* order does not guarantee extent order —
    `λ1 >= λ2 >= λ3` orders variances, and a neighbourhood that is long-and-sparse along e1
    but wide-and-dense along e2 can come out with `w > l`. So the three extents are sorted
    descending and the axes permuted **with** them, which keeps `h` the thin direction in
    eq. 4 and `(l/2, w/2)` the box centre eq. 7 measures against. Without the sort, a
    swapped pair puts a thick direction in the numerator of `f` and the descriptor reports
    creases where there are none.

    **`f` is not `λ₃/Σλ`.** `ours` and `lit-regiongrow` both use a ratio of eigen*values* —
    variances, so an RMS over the neighbourhood. This is a ratio of bounding-box *extents*,
    which is a max-minus-min. Same geometric intent, a different statistic, and the
    difference is exactly outlier sensitivity: one point lifted off the surface moves `h`
    completely and `λ₃` hardly at all. Three methods in the comparison now share a family of
    features and differ in the statistic taken over it, which is a cleaner axis than any of
    them being "better".
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    out = {kk: np.zeros(n) for kk in ("flatness", "center_offset", "corner_entropy")}
    if n == 0:
        return out
    if cKDTree is None:                                # pragma: no cover
        raise ImportError("lit-lobb needs scipy.spatial.cKDTree")
    tree = cKDTree(pts)

    for start in range(0, n, chunk):
        sl = slice(start, min(start + chunk, n))
        if k is None:
            groups = tree.query_ball_point(pts[sl], float(radius_mm), workers=-1)
        else:
            groups = np.atleast_2d(tree.query(pts[sl], k=min(int(k), n), workers=-1)[1])
        for j, ids in enumerate(groups):
            ids = np.asarray(ids, dtype=int)
            i = start + j
            if len(ids) < min_neighbors:
                continue
            nb = pts[ids]
            c = nb - nb.mean(axis=0)
            w_, v_ = np.linalg.eigh(c.T @ c / len(nb))
            axes = v_[:, ::-1]                          # columns e1, e2, e3, lam desc
            loc = c @ axes                              # neighbourhood in the PCA frame
            lo = loc.min(axis=0)
            ext = loc.max(axis=0) - lo                  # box extents along e1, e2, e3
            order = np.argsort(-ext, kind="stable")     # TASE §III-A: l >= w >= h
            axes, lo, ext = axes[:, order], lo[order], ext[order]
            l_, w_box, h_ = ext
            denom = np.hypot(l_, w_box)
            out["flatness"][i] = h_ / denom if denom > 1e-12 else 0.0
            q = (pts[i] - nb.mean(axis=0)) @ axes - lo  # the query point, box-corner frame
            out["center_offset"][i] = float(np.hypot(q[0] - l_ / 2.0, q[1] - w_box / 2.0))
            out["corner_entropy"][i] = float(np.std([l_, w_box, h_]))
    return out


def activate(x: np.ndarray) -> np.ndarray:
    """TASE eq. 9 — normalise, project to [-2, 2], `tanh`, normalise again.

    *"Activated features tend to be more polarized, which reduces the influence of
    intermediate feature values."* The point is entirely about what K-means does next: a
    1-D binary split is driven by the mass sitting between the two modes, and `tanh` on a
    bilateral range pushes that mass outward. Without it, TASE Fig. 8(a) shows face points
    misclassified as creases; with it, Fig. 8(b) is clean. Worth keeping switchable, since
    it is a claim this dataset can check rather than accept.
    """
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        return x
    lo, hi = float(x.min()), float(x.max())
    if hi - lo < 1e-12:
        return np.zeros_like(x)
    t1 = 4.0 * (x - lo) / (hi - lo) - 2.0              # linear projection into [-2, 2]
    t2 = np.tanh(t1)
    return (t2 - t2.min()) / max(t2.max() - t2.min(), 1e-12)


def kmeans_1d_binary(x: np.ndarray, iters: int = 100) -> np.ndarray:
    """Binary K-means on 1-D data. `True` = the high-value cluster.

    TASE §III-C: *"all feature clustering problems in this paper are binary classification
    problems of one-dimensional data."* Implemented rather than imported, seeded at the min
    and max, so it is **deterministic** — `lit-lobb` should show zero spread across repeats
    in the Phase 4 box plots, and that is only a finding if nothing inside it is randomised.
    """
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        return np.zeros(0, dtype=bool)
    c0, c1 = float(x.min()), float(x.max())
    if c1 - c0 < 1e-12:
        return np.zeros(len(x), dtype=bool)
    hi = np.zeros(len(x), dtype=bool)
    for _ in range(iters):
        new = np.abs(x - c1) < np.abs(x - c0)
        if np.array_equal(new, hi):
            break
        hi = new
        if hi.any():
            c1 = float(x[hi].mean())
        if (~hi).any():
            c0 = float(x[~hi].mean())
    return hi


def corner_points(pts: np.ndarray, radius_mm: float = LOBB_RADIUS_MM,
                  k: int | None = None, nonlinear_activation: bool = True) -> np.ndarray:
    """TASE Fig. 9's **edge point cloud layer** — corner points among the edge points.

    The third level of the hierarchy, and the only one whose feature is recomputed rather
    than reused: *"On the extracted edge line point cloud, LOBB is established for all
    points respectively"* (§III-B2). That is the whole mechanism — over the **edge cloud
    alone** a line point's neighbourhood is a band, so `l >> w, h` and the side-length
    standard deviation `s` is large; a corner, where two edge lines meet, spreads in two
    directions, so `l`, `w`, `h` are *"relatively uniform"* and `s` is small. Computing `s`
    over the full ROI instead would destroy the distinction: there every neighbourhood is a
    disc.

    So the corner cluster is the **low**-`s` cluster, not the high one. Points with no LOBB
    (too few neighbours, `s == 0`) are excluded rather than swept into it.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    s_ = lobb_features(pts, radius_mm, k)["corner_entropy"]
    v = activate(s_) if nonlinear_activation else s_
    line = kmeans_1d_binary(v)                          # high entropy = a band = a line
    return (~line) & (s_ > 0)


# --------------------------------------------------------------------------------
# RCIM §3.4.2-3.4.3 — key points and the curve
# --------------------------------------------------------------------------------

def mean_shift_keypoints(pts: np.ndarray, bandwidth_mm: float, iters: int = 20,
                         tol_mm: float = 1e-3) -> np.ndarray:
    """RCIM eqs. 4-7 — shift each point up the local density gradient until it converges.

    **Read the justification before reading any number this produces.** §3.4.2 motivates it
    with a *sensor* effect: creases scatter more light, so a structured-light camera returns
    more points there. This generator samples `area_uniform` by construction, so that
    gradient does not exist here and the filter has nothing to climb. It is implemented
    faithfully and defaulted off; see deviation 3.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return pts
    h = float(bandwidth_mm)
    tree = cKDTree(pts)
    cur = pts.copy()
    for _ in range(iters):
        moved = 0.0
        nxt = cur.copy()
        for i, ids in enumerate(tree.query_ball_point(cur, 3.0 * h, workers=-1)):
            if not len(ids):
                continue
            nb = pts[np.asarray(ids, dtype=int)]
            w = np.exp(-np.sum((nb - cur[i]) ** 2, axis=1) / (h * h))
            s = w.sum()
            if s > 1e-12:
                nxt[i] = (w[:, None] * nb).sum(axis=0) / s
        moved = float(np.abs(nxt - cur).max())
        cur = nxt
        if moved < tol_mm:
            break
    return cur


def fit_dominant_axis(pts: np.ndarray, degree: int = POLY_DEGREE,
                      n_samples: int = N_SAMPLES) -> np.ndarray | None:
    """RCIM §3.4.3 / eq. 8 — polynomial fit against whichever axis the seam runs along.

    *"If the change magnitude along the x direction is greater than that along the y
    direction, fitting is performed in the x-y and x-z planes; conversely ... the y-x and
    y-z planes."* Two 1-D polynomials against the dominant axis, rather than one 3-D curve.

    Their axis choice is stated in the **camera** frame and justified by the camera being
    aimed roughly normal to the seam. This dataset is in world coordinates with the joint
    at an arbitrary yaw, so `x` and `y` carry no such guarantee; the dominant axis is taken
    from the data's own extent, which is what their rule computes and not what it assumes.

    `n_samples` is RCIM's own output sampling: §4.1 and §4.3 both interpolate the fitted
    curve to **80 points** before error analysis, so that is the default.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < degree + 2:
        return None
    span = np.ptp(pts, axis=0)
    u_ax = int(np.argmax(span[:2])) if span[0] != span[1] else 0
    if span[2] > span[:2].max():                       # a seam running up the world z axis
        u_ax = 2
    others = [a for a in range(3) if a != u_ax]
    u = pts[:, u_ax]
    if np.ptp(u) < 1e-9:
        return None
    deg = int(min(degree, max(1, len(pts) - 2)))
    coeffs = [np.polyfit(u, pts[:, a], deg) for a in others]
    t = np.linspace(u.min(), u.max(), int(n_samples))
    out = np.zeros((len(t), 3))
    out[:, u_ax] = t
    for a, c in zip(others, coeffs):
        out[:, a] = np.polyval(c, t)
    return out


# --------------------------------------------------------------------------------
# result and driver
# --------------------------------------------------------------------------------

@dataclass
class LobbResult:
    """What `lit-lobb` returned, and enough context to know what it means."""

    seams: list[np.ndarray]               #: fitted polylines, one per seam
    clusters: list[np.ndarray]            #: the crease points each curve was fitted to
    points: np.ndarray                    #: the cloud the method actually worked on
    roi: np.ndarray                       #: RCIM §3.3 mask over `points`
    features: dict[str, np.ndarray] = field(default_factory=dict)
    crease: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    boundary: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    #: TASE Fig. 9's edge-cloud layer — corner points among `boundary | crease`. Empty when
    #: `corner_layer=False`. The crease points are CUT at these before the curve fit.
    corner: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    params: dict[str, Any] = field(default_factory=dict)
    #: True when component labels were supplied. K-Net produces them at 97,35% mIoU; here
    #: they come from `object_id`, which is the SAME input `ours` calls an oracle.
    used_part_oracle: bool = False
    note: str = ""

    @property
    def polylines(self) -> list[np.ndarray]:
        return self.seams

    @property
    def n_seams(self) -> int:
        return len(self.seams)


def detect(pts: np.ndarray,
           object_id: np.ndarray | None = None,
           voxel_mm: float | None = 1.0,
           edge_radius_mm: float | None = None,
           roi_k: int = ROI_K,
           lobb_radius_mm: float = LOBB_RADIUS_MM,
           lobb_k: int | None = None,
           nonlinear_activation: bool = True,
           kmeans: str = "hierarchical",
           drop_boundary: bool = True,
           corner_layer: bool = False,
           mean_shift: bool = False,
           mean_shift_bandwidth_mm: float | None = None,
           link_mm: float | None = None,
           min_cluster_pts: int = 8,
           poly_degree: int = POLY_DEGREE,
           n_samples: int = N_SAMPLES,
           roi_mask: np.ndarray | None = None) -> LobbResult:
    """Run `lit-lobb` end to end. Lengths in **millimetres**.

    Args:
        object_id: per-point component labels — RCIM §3.1's K-Net masks, supplied as an
            **oracle**. Withhold them for the L1 arm, in which case the whole cloud is the
            ROI and §3.2's edge test never runs. That arm is the one to watch: this method's
            coarse stage is `ours`' cross-object gate obtained from a 2D network, so the two
            should fail together if the dependency is real.
        roi_mask: supply the ROI directly instead of deriving it from `object_id`, for
            comparing coarse stages across the three literature methods on equal footing.
        lobb_radius_mm: TASE's 2,5 mm, chosen "according to the weld width" — so it is a
            **weld-width** parameter, not a resolution one, and should not be swept as if it
            were free.
        edge_radius_mm: RCIM eq. 1-2's neighbourhood. `None` (the default) means
            `EDGE_RADIUS_SCALE x` the cloud's measured mean point spacing — the paper's
            window is `3 x 3` **pixels**, so it scales with resolution, not with millimetres.
            Pass `3.0` for the fixed ball this module used before the audit.
        nonlinear_activation: TASE eq. 9. On by default, because the paper shows the K-means
            split failing without it. Switchable because that is a claim worth checking.
        kmeans: `"hierarchical"` (default, TASE Fig. 9) runs the boundary split on the
            centre offset over the ROI and then the crease split on the flatness over the
            **non-boundary subset only** — *"the boundary layer is performed first, and then
            the crease layer is performed"*. `"flat"` is what this module did before the
            audit: both splits over the whole ROI, then `crease & ~boundary`. They differ
            because a K-means threshold depends on the population it is fitted to: with the
            rim points still in the pot the flatness split is pulled towards them, and face
            points on the far side land in the crease cluster. The activation still
            normalises over the whole ROI, which is where Fig. 9 forms `T_f` and `T_d`;
            only the clustering support is hierarchical.
        drop_boundary: TASE §III-C splits boundary from non-boundary by centre offset before
            splitting face from crease. A plate's own rim is a boundary, not a crease, so
            this is the step that is supposed to keep rims out of the answer — the same job
            `two_surface_edges` does in `lit-regiongrow`, by a different feature. With
            `drop_boundary=False` there is no boundary layer to subtract, so `kmeans` has
            nothing to be hierarchical about and both settings collapse to `"flat"`.
        corner_layer: TASE Fig. 9's third level — a binary K-means on the corner entropy
            recomputed over `boundary | crease`, cutting the crease components at the
            corners before the fit. **Off by default**: see deviation 6.
        n_samples: points on each returned polyline. RCIM interpolates to **80** (§4.1).
        mean_shift: RCIM §3.4.2. **Off by default** — its justification is a structured-light
            density artefact this generator does not simulate. See deviation 3.
    """
    if str(kmeans) not in ("hierarchical", "flat"):
        raise ValueError(f"kmeans must be 'hierarchical' or 'flat', got {kmeans!r}")
    pts = np.asarray(pts, dtype=float)
    P = voxel_downsample(pts, float(voxel_mm)) if voxel_mm else pts
    vox = float(voxel_mm) if voxel_mm else 1.0
    link_mm = float(link_mm) if link_mm else 3.0 * vox
    bw = float(mean_shift_bandwidth_mm) if mean_shift_bandwidth_mm else 2.0 * lobb_radius_mm
    spacing = mean_point_spacing(P) if len(P) > 1 else 0.0
    edge_r = edge_radius_for(P, edge_radius_mm, spacing=spacing)
    params = dict(voxel_mm=voxel_mm, edge_radius_mm=edge_r,
                  edge_radius_arg=edge_radius_mm, point_spacing_mm=spacing, roi_k=roi_k,
                  lobb_radius_mm=lobb_radius_mm, lobb_k=lobb_k,
                  nonlinear_activation=nonlinear_activation, kmeans=str(kmeans),
                  drop_boundary=drop_boundary, corner_layer=corner_layer,
                  mean_shift=mean_shift, link_mm=link_mm, poly_degree=poly_degree,
                  n_samples=n_samples, n_input=len(P))
    empty = np.zeros(len(P), dtype=bool)

    if len(P) < 8:
        return LobbResult([], [], P, empty, {}, empty, empty, empty, params, False,
                          "cloud too small")

    # --- RCIM §3.1-3.3, the coarse stage -----------------------------------------------
    used_oracle = False
    if roi_mask is not None:
        roi = np.asarray(roi_mask, dtype=bool)
        if len(roi) != len(P):                          # supplied against the input cloud
            _, idx = cKDTree(pts).query(P, k=1, workers=-1)
            roi = np.asarray(roi_mask, dtype=bool)[idx]
    elif object_id is not None:
        used_oracle = True
        _, idx = cKDTree(pts).query(P, k=1, workers=-1)
        labels = part_labels_oracle(np.asarray(object_id)[idx])
        roi = roi_by_extension(P, part_boundary_points(P, labels, edge_r), roi_k)
    else:
        roi = np.ones(len(P), dtype=bool)               # L1: no coarse stage at all

    sub = np.flatnonzero(roi)
    if len(sub) < 8:
        return LobbResult([], [], P, roi, {}, empty, empty, empty, params, used_oracle,
                          "ROI too small after the coarse stage")

    # --- TASE §III + Fig. 9, features then HIERARCHICAL K-means -------------------------
    # Fig. 9's initial point cloud layer: T_d and T_f are formed and activated over the
    # WHOLE ROI, the boundary layer is clustered first, and the crease layer is clustered
    # on what the boundary layer left behind. `kmeans="flat"` keeps the pre-audit path,
    # where both splits saw the whole ROI and the boundary mask was merely subtracted.
    feats = lobb_features(P[sub], lobb_radius_mm, lobb_k)
    act = {k: (activate(v) if nonlinear_activation else v) for k, v in feats.items()}

    boundary = kmeans_1d_binary(act["center_offset"])
    hier = str(kmeans) == "hierarchical" and drop_boundary
    if hier:
        crease = np.zeros(len(sub), dtype=bool)
        rest = np.flatnonzero(~boundary)                # the non-boundary subset
        if len(rest) >= 2:
            crease[rest] = kmeans_1d_binary(act["flatness"][rest])
    else:
        crease = kmeans_1d_binary(act["flatness"])
        if drop_boundary:
            crease = crease & ~boundary

    # Fig. 9's edge point cloud layer: corner entropy recomputed over `boundary | crease`.
    corner = np.zeros(len(sub), dtype=bool)
    if corner_layer:
        eidx = np.flatnonzero(boundary | crease)
        if len(eidx) >= min_cluster_pts:
            corner[eidx] = corner_points(P[sub[eidx]], lobb_radius_mm, lobb_k,
                                         nonlinear_activation)

    crease_full = np.zeros(len(P), dtype=bool)
    crease_full[sub[crease]] = True
    bound_full = np.zeros(len(P), dtype=bool)
    bound_full[sub[boundary]] = True
    corner_full = np.zeros(len(P), dtype=bool)
    corner_full[sub[corner]] = True
    feats_full = {k: np.zeros(len(P)) for k in feats}
    for k, v in feats.items():
        feats_full[k][sub] = v

    # The corners are a CUT, not a detection: a crease point that is also a corner is
    # dropped before the components are formed, so a run that turns through a corner comes
    # back as two components and gets one polynomial each.
    fit_mask = crease_full & ~corner_full if corner_layer else crease_full
    cpts = P[fit_mask]
    if len(cpts) < min_cluster_pts:
        return LobbResult([], [], P, roi, feats_full, crease_full, bound_full, corner_full,
                          params, used_oracle, f"{len(cpts)} crease point(s) — no seam")

    # --- RCIM §3.4.2-3.4.3, key points and the curve ------------------------------------
    if mean_shift:
        cpts = mean_shift_keypoints(cpts, bw)

    comp = connected_components(cpts, link_mm)
    seams, clusters = [], []
    for c in range(comp.max() + 1 if len(comp) else 0):
        m = comp == c
        if int(m.sum()) < min_cluster_pts:
            continue
        poly = fit_dominant_axis(cpts[m], poly_degree, n_samples)
        if poly is None:
            continue
        seams.append(poly)
        clusters.append(cpts[m])

    note = "" if seams else "crease points found but no cluster survived the fit"
    return LobbResult(seams, clusters, P, roi, feats_full, crease_full, bound_full,
                      corner_full, params, used_oracle, note)
