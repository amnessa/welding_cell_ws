"""`lit-regiongrow` — curvature-seeded region growing, then edges between two surfaces.

A faithful reimplementation of the extractor in:

    Pengkun Wei, Shuo Cheng, Dayou Li, Ran Song, Yipeng Zhang, Wei Zhang.
    "Coarse-to-Fine Detection of Multiple Seams for Robotic Welding."
    arXiv:2408.10710, 2024.

Their pipeline is coarse-to-fine, and only the fine half is a seam extractor:

    §III-C  FastSAM on the RGB image segments each workpiece SURFACE; the
            intersection of two surfaces is the approximate seam, and the cloud is
            cropped to it                              -> `seam_region_oracle` (an ORACLE)
    §III-D  pass-through filter + voxel downsample + KD-tree   -> `preprocess`
            region growing on curvature and normal angle       -> `region_grow`
            edge points that lie between TWO regions           -> `two_surface_edges`
            plane fit, project, fit (x,y), sample the path     -> `fit_seam`

The §III-C crop is the same shape of dependency `lit-ransac` has on its PointNet++, so it is
supplied the same way and reported the same way: `seam_region_oracle` is the L0 arm,
withholding it is L1.

Published numbers, which are the reproduction target:

    voxel grid 3 mm     chosen in §IV-A as the largest grid still under 1 mm of error
    RMSE 0,37 mm        2 linear seams
    RMSE 0,54 mm        10 linear seams
    RMSE 0,56 mm        1 curved seam
    "the maximum error of the welding path should not exceed 1 mm"   (§IV-A)

Why this method is in the seven
-------------------------------
Its curvature is **the same quantity `ours` uses**. Eq. 3 is

    M = (1/k) Σ (p_i - p_0)(p_i - p_0)ᵀ ,   δ = λ₀ / (λ₀ + λ₁ + λ₂)

which is `radius_pca.surface_variation` under another name. The methods then diverge on
three choices, and because the feature is shared, each one is measurable on its own:

1. **k-NN versus a radius ball.** This paper takes `k` nearest neighbours; `ours` takes a
   ball, and `README §8` argues that substitution is the single thing that makes the method
   work — the parts do not touch, so a k-NN ball on a point at part A's edge contains only
   part-A points and looks flat. Here that argument becomes an A/B test rather than a claim:
   `neighbourhood="knn"` is this paper, `"radius"` is `ours`' choice, same code path.
2. **Threshold versus region growing.** `ours` thresholds δ globally. This paper grows
   regions from the *smoothest* point and splits on the normal-angle jump, so the seam is
   found as a *boundary between surfaces* rather than as a set of high-curvature points.
3. **Oracle versus self-supplied segmentation.** `ours` needs `object_id` to know two points
   are on different parts. This method derives its own regions and then keeps the edge
   points whose neighbourhood spans two of them (§III-D). **That is the same test without
   the oracle**, and it is the most interesting thing in the paper for this project: it is
   a candidate answer to the L0→L1 collapse `ours` suffers.

Deviations, and why each one exists
-----------------------------------
1. **Alg. 1 as printed grows one region and never terminates cleanly.** It has no visited
   set, deletes the seed inside the curvature branch, and has no outer loop — yet §III-D
   says it segments "the workpiece and its individual surfaces", plural. Implemented as the
   standard region-growing formulation those pseudocode lines abbreviate (Rabbani et al.,
   which PCL's `RegionGrowing` follows): a visited set, seeds ordered by curvature, an outer
   loop that starts a new region at the lowest-curvature unvisited point, and a minimum
   region size. `smoothness_deg` is Threshold1 and `curvature_thresh` is Threshold2.
2. **Threshold1, Threshold2 and k are never given numerically.** Only the voxel grid (3 mm)
   is published. The defaults here are PCL's, which is the implementation the pseudocode
   describes: 30 neighbours, 3° smoothness — with the curvature threshold at 0,03, matching
   the value `ours` uses for the same quantity so the two are comparable by construction.
   **Every one of these is a guess and is exposed**; a sweep is the only honest treatment.
3. **"Points that satisfy both conditions" (§III-D) is not spelled out.** High curvature and
   "represent the intersection between two surfaces". Implemented as: an edge point whose
   `edge_radius_mm` neighbourhood contains points of at least two distinct regions, with the
   two dominant regions each holding `min_region_share` of it — so a point on the rim of one
   surface, with a handful of stray neighbours, is not an edge.
4. **The path fit is a plane fit, then a fit of (x, y) in that plane.** For a straight seam
   the in-plane fit is a line, which is what `curve="line"` does. `curve="poly"` fits a
   degree-`poly_degree` polynomial in the plane's dominant direction, for the curved seams
   of Phase 6. The paper's own experiments are three linear workpieces and one curved one.
5. **Multi-seam clustering is implied, not described.** "Extract all weld seams at once"
   requires the edge points to be split before fitting; Euclidean clustering, shared with
   `radius_pca.connected_components`. **This is where the method breaks on our geometry, and
   it breaks exactly where `ours` does.** A T joint's two fillets are one plate-thickness
   apart — 8 mm — and proximity is the only thing the clustering has to go on. Link short and
   the seam shatters into ten fragments of a dozen points each; link long enough to hold a
   seam together and the two fillets merge into one line sitting between them, at ~3 mm RMSE.
   `dataset_plan.md` already records the same conclusion for `ours`: *proximity alone cannot
   separate two parallel centrelines a plate-thickness apart; splitting components by
   direction as well would*. Two independent methods reaching the same wall is the more
   interesting version of that finding, and neither paper could have found it — their
   workpieces are large steel structures whose seams are nowhere near each other.

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

from .lit_ransac import seam_region_oracle  # noqa: F401  (re-exported: the §III-C stand-in)
from .radius_pca import connected_components, voxel_downsample

# §IV-A: "setting the edge length to 3 mm can ensure sub-millimeter accuracy while
# maximizing efficiency". The one published constant in the method.
VOXEL_MM = 3.0
# NOT the paper's - it publishes no value for any of these. PCL's RegionGrowing defaults
# (k = 30, smoothness 3 deg) are the obvious inheritance, since Alg. 1 is PCL's algorithm in
# pseudocode, and they **do not work here**: at a 3 mm grid, 30 neighbours span a ~9 mm ball,
# which is wider than an 8 mm plate, so the normal wraps around the plate's own two faces
# and every point in the scene comes back marked as an edge. That is `ours`' "the ball must
# not bridge a plate's own two faces" bound, arriving in a second method by a different
# route. These are chosen by sweep against constructed truth over the 12 T scenes in
# `out/phase3`; the sweep is in `notebooks/05_lit_regiongrow`. They are **tuned**, and any
# result that turns on them is a result about this reimplementation, not about the paper.
K_NEIGHBORS = 10
SMOOTHNESS_DEG = 20.0
CURVATURE_THRESH = 0.03


# --------------------------------------------------------------------------------
# §III-D step 1 — preprocessing, and the feature of eq. 3
# --------------------------------------------------------------------------------

def preprocess(pts: np.ndarray, voxel_mm: float | None = VOXEL_MM,
               z_range_mm: tuple[float, float] | None = None) -> np.ndarray:
    """Pass-through filter, then voxel downsample. §III-D, eq. 2.

    The pass-through filter is described as using "prior knowledge such as the optimal
    working range of the camera and the known height of the workbench" — scene furniture
    this dataset does not put in the cloud, so it is off unless a range is given.
    """
    pts = np.asarray(pts, dtype=float)
    if z_range_mm is not None:
        lo, hi = z_range_mm
        pts = pts[(pts[:, 2] >= lo) & (pts[:, 2] <= hi)]
    return voxel_downsample(pts, float(voxel_mm)) if voxel_mm else pts


def local_pca(pts: np.ndarray, k: int = K_NEIGHBORS, radius_mm: float | None = None,
              chunk: int = 20_000) -> tuple[np.ndarray, np.ndarray]:
    """`(normals, curvature)` from eq. 3. Curvature is `δ = λ₀/(λ₀+λ₁+λ₂)`, normal is `v₀`.

    Set `radius_mm` to swap the k-NN neighbourhood for a ball — the one substitution
    `README §8` says decides whether this feature can see across a joint gap at all. Same
    eigen-decomposition either way, so the comparison is clean.

    `δ` here is bit-identical in definition to `radius_pca.surface_variation`; that module
    returns only the ratio because it never needs the normal, and this one needs both.
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    normals = np.zeros((n, 3))
    curv = np.zeros(n)
    if n == 0:
        return normals, curv
    if cKDTree is None:                                # pragma: no cover
        raise ImportError("lit-regiongrow needs scipy.spatial.cKDTree")

    tree = cKDTree(pts)
    for start in range(0, n, chunk):
        sl = slice(start, min(start + chunk, n))
        if radius_mm is None:
            _, idx = tree.query(pts[sl], k=min(k, n), workers=-1)
            idx = np.atleast_2d(idx)
            nb = pts[idx]                                          # (m, k, 3)
            c = nb - nb.mean(axis=1, keepdims=True)
            cov = np.einsum("mki,mkj->mij", c, c) / max(idx.shape[1], 1)
        else:
            lists = tree.query_ball_point(pts[sl], radius_mm, workers=-1)
            cov = np.zeros((len(lists), 3, 3))
            for j, ids in enumerate(lists):
                if len(ids) < 3:
                    cov[j] = np.eye(3) * 1e-12
                    continue
                c = pts[ids] - pts[ids].mean(axis=0)
                cov[j] = c.T @ c / len(ids)
        w, v = np.linalg.eigh(cov)                                 # ascending
        tot = w.sum(axis=1)
        curv[sl] = np.where(tot > 0, w[:, 0] / np.where(tot > 0, tot, 1.0), 0.0)
        normals[sl] = v[:, :, 0]
    return normals, curv


# --------------------------------------------------------------------------------
# §III-D step 2 — Alg. 1
# --------------------------------------------------------------------------------

def region_grow(pts: np.ndarray, normals: np.ndarray, curvature: np.ndarray,
                k: int = K_NEIGHBORS, smoothness_deg: float = SMOOTHNESS_DEG,
                curvature_thresh: float = CURVATURE_THRESH,
                min_region_pts: int = 30, radius_mm: float | None = None
                ) -> tuple[np.ndarray, np.ndarray]:
    """Alg. 1. Returns `(labels, is_edge)`; label `-1` is unassigned.

    Seeds are ordered by curvature and the growth starts at the smoothest point, which is
    the paper's stated reason for sorting: *"the smoother the region in the point cloud, the
    less likely it is to be a weld region, and growing from the smoothest region could
    improve efficiency."*

    Two thresholds, both from Alg. 1 and neither given a value in the paper:

    * `smoothness_deg` (Threshold1) decides **surface or edge**. A neighbour whose normal
      turns by more than this is not part of this surface, and is marked as an edge point.
      This is the line that makes the method a *boundary* detector rather than a curvature
      detector, and it is what distinguishes it from `ours`.
    * `curvature_thresh` (Threshold2) decides **whether a neighbour may itself seed**, which
      stops growth from continuing through a fold.

    Normal signs are arbitrary out of an eigen-decomposition, so the angle test folds to
    `|cos|` — otherwise a flat surface splits in two wherever the sign happens to flip.
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    labels = np.full(n, -1, dtype=int)
    is_edge = np.zeros(n, dtype=bool)
    if n == 0:
        return labels, is_edge

    tree = cKDTree(pts)
    cos_thresh = np.cos(np.radians(float(smoothness_deg)))
    order = np.argsort(curvature)                      # smoothest first
    label = 0

    for start in order:
        if labels[start] != -1:
            continue
        seeds = [int(start)]
        labels[start] = label
        members = 1
        while seeds:
            s = seeds.pop()
            nb = (tree.query_ball_point(pts[s], radius_mm) if radius_mm is not None
                  else tree.query(pts[s], k=min(k, n))[1])
            for j in np.atleast_1d(nb):
                j = int(j)
                if j == s or labels[j] != -1:
                    continue
                if abs(float(normals[s] @ normals[j])) < cos_thresh:
                    is_edge[j] = True                  # Alg. 1 line 8
                    continue
                labels[j] = label                      # Alg. 1 line 10
                members += 1
                if curvature[j] < curvature_thresh:    # Alg. 1 line 12-13
                    seeds.append(j)
        if members < min_region_pts:                   # too small to be a surface
            labels[labels == label] = -1
        else:
            label += 1
    return labels, is_edge


def two_surface_edges(pts: np.ndarray, labels: np.ndarray, is_edge: np.ndarray,
                      edge_radius_mm: float, min_region_share: float = 0.15
                      ) -> np.ndarray:
    """§III-D: an edge point must also *"represent the intersection between two surfaces"*.

    High curvature alone is not enough and the paper says so — a plate's own outer rim is a
    boundary too. Kept here only if the point's neighbourhood spans **two** grown regions,
    each holding at least `min_region_share` of it, so a stray neighbour or two does not
    qualify a rim point.

    This is `ours`' cross-object gate with the oracle removed: the regions come from the
    method's own segmentation rather than from `object_id`. That substitution is the single
    most transferable idea in this paper for the rest of Phase 4.
    """
    pts = np.asarray(pts, dtype=float)
    cand = np.flatnonzero(is_edge)
    keep = np.zeros(len(pts), dtype=bool)
    if len(cand) == 0:
        return keep
    tree = cKDTree(pts)
    for i in cand:
        ids = tree.query_ball_point(pts[i], edge_radius_mm)
        lab = labels[ids]
        lab = lab[lab >= 0]
        if len(lab) == 0:
            continue
        counts = np.bincount(lab)
        top = np.sort(counts)[::-1]
        if len(top) < 2 or top[1] < min_region_share * len(lab):
            continue
        keep[i] = True
    return keep


# --------------------------------------------------------------------------------
# §III-D step 3 — the path
# --------------------------------------------------------------------------------

def fit_seam(pts: np.ndarray, curve: str = "line", poly_degree: int = 2,
             n_samples: int = 2) -> np.ndarray | None:
    """§III-D: fit the plane the edge points lie on, project, then fit `(x, y)` in it.

    The projection is the part that matters. Edge points form a *ribbon*, and a
    total-least-squares line through a ribbon lands in its middle — the failure that made
    `ours` drop its line fit entirely. Here the ribbon is first flattened onto its own
    plane, so the fit is 2D and the remaining spread is along one in-plane axis only.
    Whether that is enough is a question this dataset can answer and the paper cannot,
    because the paper has no exact truth to answer it against.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3:
        return None
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    e0, e1 = vt[0], vt[1]                              # in-plane axes; vt[2] is the normal
    u = (pts - c) @ e0
    v = (pts - c) @ e1

    if curve == "line":
        t = np.array([u.min(), u.max()])
        # Least squares v(u) in the plane. For a straight seam this is flat by construction;
        # fitting it anyway is what the paper does and it costs nothing.
        a, b = np.polyfit(u, v, 1) if np.ptp(u) > 1e-9 else (0.0, float(v.mean()))
        return c + t[:, None] * e0 + (a * t + b)[:, None] * e1
    if curve == "poly":
        if np.ptp(u) < 1e-9:
            return None
        coef = np.polyfit(u, v, int(poly_degree))
        t = np.linspace(u.min(), u.max(), max(int(n_samples), poly_degree + 2))
        return c + t[:, None] * e0 + np.polyval(coef, t)[:, None] * e1
    raise ValueError(f"unknown curve {curve!r}")


def torch_pose(pts: np.ndarray, normals: np.ndarray, idx: np.ndarray) -> np.ndarray:
    """§III-D: *"compute their mean vector direction as the welding pose"*.

    Mean normal over the seam's own edge points. Signs are arbitrary from the
    eigen-decomposition, so they are aligned to the first before averaging — without that
    the mean of a fold's two faces is noise rather than a bisector.
    """
    if len(idx) == 0:
        return np.array([0.0, 0.0, 1.0])
    nb = normals[idx]
    nb = nb * np.sign(nb @ nb[0] + 1e-12)[:, None]
    m = nb.mean(axis=0)
    norm = float(np.linalg.norm(m))
    return m / norm if norm > 1e-12 else np.array([0.0, 0.0, 1.0])


# --------------------------------------------------------------------------------
# result and driver
# --------------------------------------------------------------------------------

@dataclass
class RegionGrowResult:
    """What `lit-regiongrow` returned, and enough context to know what it means."""

    seams: list[np.ndarray]               #: fitted polylines, one per seam
    clusters: list[np.ndarray]            #: the edge points each seam was fitted to
    poses: list[np.ndarray]               #: mean-normal torch direction per seam
    points: np.ndarray                    #: the preprocessed cloud everything indexes into
    labels: np.ndarray                    #: region id per point, -1 unassigned
    curvature: np.ndarray                 #: `δ` of eq. 3 — the same feature `ours` uses
    is_edge: np.ndarray                   #: Alg. 1's Threshold1 verdict, before §III-D's pair test
    seam_mask: np.ndarray                 #: edge points that also span two regions
    params: dict[str, Any] = field(default_factory=dict)
    used_segmentation_oracle: bool = False
    note: str = ""

    @property
    def polylines(self) -> list[np.ndarray]:
        return self.seams

    @property
    def n_seams(self) -> int:
        return len(self.seams)

    @property
    def n_regions(self) -> int:
        return int(self.labels.max()) + 1 if len(self.labels) else 0


def detect(pts: np.ndarray,
           voxel_mm: float | None = VOXEL_MM,
           k: int = K_NEIGHBORS,
           smoothness_deg: float = SMOOTHNESS_DEG,
           curvature_thresh: float = CURVATURE_THRESH,
           neighbourhood: str = "knn",
           radius_mm: float | None = None,
           min_region_pts: int = 30,
           edge_radius_mm: float | None = None,
           min_region_share: float = 0.15,
           link_mm: float | None = None,
           min_cluster_pts: int = 8,
           curve: str = "line",
           poly_degree: int = 2,
           segmentation_mask: np.ndarray | None = None,
           z_range_mm: tuple[float, float] | None = None) -> RegionGrowResult:
    """Run `lit-regiongrow` end to end. Lengths in **millimetres**.

    Args:
        voxel_mm: §IV-A's grid, published as 3 mm. It is the accuracy/speed knob the paper
            actually sweeps (Fig. 5), and it bounds the achievable RMSE from below — a 3 mm
            grid moves every point by up to ~1,5 mm before the method sees it.
        k: neighbours for eq. 3 and for growth. **Not published**; PCL's default.
        smoothness_deg: Alg. 1's Threshold1, the normal-angle jump that calls a point an
            edge. **Not published.**
        curvature_thresh: Alg. 1's Threshold2, whether a neighbour may itself seed.
            **Not published.** Defaulted to `ours`' value for the same quantity, so the two
            methods can be compared on the threshold as well as on the feature.
        neighbourhood: `"knn"` (the paper) or `"radius"` (`ours`' choice). The A/B on the
            substitution `README §8` says decides whether this feature can cross a joint gap.
        radius_mm: ball radius when `neighbourhood="radius"`; defaults to `2 x voxel_mm`.
        edge_radius_mm: neighbourhood for §III-D's two-surface test. Defaults to
            `4 x voxel_mm`, and this is the most consequential unpublished parameter in the
            method. It has to reach **across the whole edge band into labelled surface
            points on both sides**: edge points are left unlabelled by Alg. 1, so a ball
            that only spans the band sees nothing but `-1` and the two-surface test starves.
            Measured on the T scenes, `1,5 x voxel` keeps 164 seam points and `4-5 x voxel`
            keeps over a thousand, for the same input.
        curve: `"line"` for the paper's three linear workpieces, `"poly"` for its curved one.
        segmentation_mask: the §III-C FastSAM crop, supplied as an **oracle** via
            `seam_region_oracle`. Withhold it for the L1 arm.
    """
    pts = np.asarray(pts, dtype=float)
    used_oracle = segmentation_mask is not None
    if used_oracle:
        pts = pts[np.asarray(segmentation_mask, dtype=bool)]

    P = preprocess(pts, voxel_mm, z_range_mm)
    vox = float(voxel_mm) if voxel_mm else 1.0
    radius_mm = float(radius_mm) if radius_mm else 2.0 * vox
    edge_radius_mm = float(edge_radius_mm) if edge_radius_mm else 4.0 * vox
    link_mm = float(link_mm) if link_mm else 2.0 * vox
    ball = radius_mm if neighbourhood == "radius" else None
    if neighbourhood not in ("knn", "radius"):
        raise ValueError(f"unknown neighbourhood {neighbourhood!r}")

    params = dict(voxel_mm=voxel_mm, k=k, smoothness_deg=smoothness_deg,
                  curvature_thresh=curvature_thresh, neighbourhood=neighbourhood,
                  radius_mm=radius_mm, edge_radius_mm=edge_radius_mm,
                  min_region_share=min_region_share, link_mm=link_mm, curve=curve,
                  n_input=len(P))
    empty = np.zeros((0, 3))
    if len(P) < max(k, 4):
        return RegionGrowResult([], [], [], P, np.zeros(len(P), int), np.zeros(len(P)),
                                np.zeros(len(P), bool), np.zeros(len(P), bool), params,
                                used_oracle, "cloud too small to grow a region")

    normals, curv = local_pca(P, k, ball)
    labels, is_edge = region_grow(P, normals, curv, k, smoothness_deg, curvature_thresh,
                                  min_region_pts, ball)
    seam_mask = two_surface_edges(P, labels, is_edge, edge_radius_mm, min_region_share)

    idx = np.flatnonzero(seam_mask)
    if len(idx) < min_cluster_pts:
        n_reg = int(labels.max()) + 1 if len(labels) else 0
        return RegionGrowResult([], [], [], P, labels, curv, is_edge, seam_mask, params,
                                used_oracle,
                                f"{n_reg} region(s), {int(is_edge.sum())} edge point(s), "
                                f"{len(idx)} spanning two regions — no seam")

    comp = connected_components(P[idx], link_mm)
    seams, clusters, poses = [], [], []
    for c in range(comp.max() + 1 if len(comp) else 0):
        sub = idx[comp == c]
        if len(sub) < min_cluster_pts:
            continue
        poly = fit_seam(P[sub], curve, poly_degree)
        if poly is None:
            continue
        seams.append(poly)
        clusters.append(P[sub])
        poses.append(torch_pose(P, normals, sub))

    note = "" if seams else "edge points found but no cluster survived the fit"
    return RegionGrowResult(seams, clusters, poses, P, labels, curv, is_edge, seam_mask,
                            params, used_oracle, note)
