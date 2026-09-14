"""`lit-regiongrow` — curvature-seeded region growing, then edges between two surfaces.

A faithful reimplementation of the extractor in:

    Pengkun Wei, Shuo Cheng, Dayou Li, Ran Song, Yipeng Zhang, Wei Zhang.
    "Coarse-to-Fine Detection of Multiple Seams for Robotic Welding."
    arXiv:2408.10710, 2024.

Their pipeline is coarse-to-fine, and only the fine half is a seam extractor:

    §III-C  a TRAINED WELD-KEYPOINT detector clicks each seam; those clicks prompt
            FastSAM, and the cloud is cropped to the resulting masks, keeping ~12,7 %
            of it (Table II, p. 6: 81 730 -> 10 356 points, "over 80% ... removed")
                                                       -> `keypoint_crop` (an ORACLE)
    §III-D  pass-through filter + voxel downsample + KD-tree   -> `preprocess`
            region growing on curvature and normal angle       -> `region_grow`
            edge points that lie between TWO regions           -> `two_surface_edges`
            plane fit, project, fit (x,y), sample the path     -> `fit_seam`

The §III-C crop is the same shape of dependency `lit-ransac` has on its PointNet++, so it is
supplied the same way and reported the same way: the crop is the L0 arm, withholding it is
L1. **Which** crop is the whole question, and getting it wrong was the method's headline
failure on this corpus — see Deviation 6. `keypoint_crop` is the one that matches §III-C;
`surface_intersection_crop` and `seam_region_oracle` are the older, far more generous
stand-ins and are kept only so the difference stays measurable.

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
6. **§III-C is a KEYPOINT crop, and it is severe.** It is not "segment every surface and
   intersect them all": §III-C is a *trained weld-keypoint detector* whose clicks prompt
   FastSAM — *"only 4 key points need to be labeled for an open square weldment"* (p. 3,
   right column). Table II (p. 6) prices what that leaves behind: 81 730 -> 10 356 points,
   **12,7 % kept**, *"over 80% of redundant point clouds"* removed. Cropping instead on the
   intersections of all 12 faces of a two-plate joint (`surface_intersection_crop`) keeps
   54-97 % of these scenes, i.e. most of the workpiece, and the fine stage then over-detects
   grossly: an audit of this corpus measured **9-31 % of all points marked as edges, of
   which only 10-15 % lie within 3 mm of a seam**. That is a failure of the coarse stage we
   supplied, not of Alg. 1. `keypoint_crop` is the stand-in that keeps the paper's fraction.
   It is an **ORACLE** — it reads the truth polylines, exactly as `seam_region_oracle` does
   for `lit-ransac` — and it is labelled as one everywhere it is reported.
7. **The harness runs a 1,5 mm voxel; §IV-A (p. 5) publishes 3 mm.** `VOXEL_MM` stays 3,0
   because that is the published value, but `harness._run_lit_regiongrow` passes
   `voxel_mm=1.5` and that is a **deviation**, measured: 3 mm is worse on this corpus. The
   paper's workpieces are metre-scale steel structures; ours are 8 mm plates, and a 3 mm
   grid displaces every point by up to ~1,5 mm — a fifth of the feature — before the method
   sees it. The published value is kept as the module default and the harness value is
   recorded in `params["voxel_mm"]` next to `params["voxel_mm_published"]`.
8. **§III-D's edge test is a conjunction and the first implementation dropped half of it.**
   *"[weld] seam edges not only need to have high curvature but also need to represent the
   intersection between two surfaces"* (p. 4, right column). `two_surface_edges` tested only
   the second clause; `edge_curv_thresh` restores the first. Likewise `_edge_from_labels`
   accepted a `labels` argument and never used it, so the supplied-surface arm ran no
   label-boundary test at all — it now requires the neighbourhood to span two surfaces as
   well as the normal to turn.

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

from .lit_ransac import (seam_region_oracle,  # noqa: F401  (re-exported)
                         surface_intersection_crop, surface_labels_oracle)
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
# Table II (p. 6): the coarse stage takes 81 730 points to 10 356, and §IV-B states it
# "can remove over 80% of redundant point clouds". That ratio is the only quantitative
# description of the §III-C crop the paper gives, so it is what `keypoint_crop` targets.
KEYPOINT_CROP_FRACTION = 10_356 / 81_730                       # 0,1267
# NOT the paper's. A floor and a ceiling on the crop radius so the search cannot degenerate:
# below ~5 mm the crop is thinner than a voxel band and holds no surface to grow, and above
# 60 mm it is wider than the plates in this corpus and crops nothing.
KEYPOINT_RADIUS_BOUNDS_MM = (5.0, 60.0)


# --------------------------------------------------------------------------------
# §III-C — the coarse stage, as an ORACLE
# --------------------------------------------------------------------------------

def keypoint_crop(pts: np.ndarray, seam_polylines,
                  radius_mm: float | None = None,
                  target_fraction: float = KEYPOINT_CROP_FRACTION,
                  radius_bounds_mm: tuple[float, float] = KEYPOINT_RADIUS_BOUNDS_MM,
                  ) -> tuple[np.ndarray, dict[str, Any]]:
    """§III-C's keypoint-prompted crop, supplied as an **ORACLE**. Returns `(mask, info)`.

    **This is an oracle and must be reported as one.** It reads the truth polylines, the
    same way `lit_ransac.seam_region_oracle` reads them for that paper's PointNet++ and
    `surface_labels_oracle` reads `face_id` for this paper's FastSAM. It stands in for a
    stage this project does not implement: a *trained weld-keypoint detector* that clicks
    each seam, whose clicks prompt FastSAM, whose masks crop the cloud (§III-C, p. 3-4).

    Why it exists at all. The previous stand-in — `surface_intersection_crop` over every
    face pair — kept 54-97 % of these scenes, while §III-C keeps **12,7 %** (Table II,
    p. 6: 81 730 -> 10 356 points; *"over 80% of redundant point clouds"* removed). Handing
    the fine stage most of a workpiece is what produced the over-detection this module was
    audited for: 9-31 % of all points returned as edges, 10-15 % of them within 3 mm of a
    seam. A keypoint detector does not find twelve faces, it finds the seams — so the
    honest stand-in keeps a band around the seams at the paper's own retained fraction.

    The radius is **searched, not assumed**. `target_fraction` is the published quantity;
    the radius that realises it depends on the scene's extent and sampling density, so it is
    found by bisection on the point-to-seam distances and clamped to `radius_bounds_mm`. The
    realised radius and the realised fraction both come back in `info` and belong in any
    table this crop appears in, because a scene whose geometry cannot reach 12,7 % inside
    the bounds is a scene where the crop is *not* the paper's.

    Args:
        pts: `(N, 3)` cloud, millimetres.
        seam_polylines: the truth seams — **the oracle input**.
        radius_mm: skip the search and use this radius. `info["searched"]` records which.
        target_fraction: fraction of the cloud to retain. Defaults to Table II's ratio.
        radius_bounds_mm: `(min, max)` clamp on the searched radius.

    Returns:
        `(mask, info)`, `mask` a boolean of length `N`; `info` carries `radius_mm`,
        `fraction`, `target_fraction`, `n_kept`, `n_total`, `searched` and `note`.
    """

    pts = np.asarray(pts, dtype=float)
    polys = [np.asarray(g, dtype=float) for g in (seam_polylines or [])
             if len(np.asarray(g)) >= 2]
    lo_b, hi_b = (float(radius_bounds_mm[0]), float(radius_bounds_mm[1]))
    info: dict[str, Any] = {"target_fraction": float(target_fraction),
                            "n_total": int(len(pts)), "searched": radius_mm is None,
                            "radius_bounds_mm": (lo_b, hi_b), "note": ""}
    if len(pts) == 0 or not polys:
        # No truth to crop against: keep everything and say so, rather than silently
        # returning an empty cloud that the fine stage would report as "no seam".
        mask = np.ones(len(pts), dtype=bool)
        info.update(radius_mm=float("inf"), fraction=1.0, n_kept=int(len(pts)),
                    note="no seam polylines — crop is a no-op and this is NOT §III-C")
        return mask, info

    # nearest polyline VERTEX via a KD-tree: the truth polylines are stored at 0,1 mm
    # spacing, so the vertex distance is within 0,05 mm of the segment distance, which is
    # nothing against a 5-60 mm radius - and 100x faster than the exact per-segment loop.
    from scipy.spatial import cKDTree
    from .metrics import _sample_polyline
    dense = np.vstack([_sample_polyline(pl, 0.5) for pl in polys])   # coarse polylines densified first
    d, _ = cKDTree(dense).query(pts, k=1, workers=-1)
    if radius_mm is not None:
        r = float(radius_mm)
    else:
        # Bisection on the radius, evaluated against the sorted distances so each step is a
        # binary search rather than a pass over the cloud.
        ds = np.sort(d)
        target_n = int(round(float(target_fraction) * len(pts)))
        lo, hi = lo_b, hi_b
        for _ in range(64):
            mid = 0.5 * (lo + hi)
            if int(np.searchsorted(ds, mid, side="right")) < target_n:
                lo = mid
            else:
                hi = mid
        r = float(min(max(0.5 * (lo + hi), lo_b), hi_b))
        # The distances are discrete — a regular scan puts whole rows of points at the same
        # distance — so the bisection lands on a jump and always overshoots it. Step back to
        # the radius just below the tie block if that lands closer to the target.
        k_hi = int(np.searchsorted(ds, r, side="right"))
        below = ds[ds < r]
        if len(below):
            r_lo = float(below[-1])
            k_lo = int(np.searchsorted(ds, r_lo, side="right"))
            if k_lo > 0 and r_lo >= lo_b and abs(k_lo - target_n) < abs(k_hi - target_n):
                r = r_lo
    mask = d <= r
    frac = float(mask.mean()) if len(mask) else 0.0
    info.update(radius_mm=r, fraction=frac, n_kept=int(mask.sum()))
    if abs(frac - float(target_fraction)) > 0.03:
        info["note"] = (f"realised {frac:.3f} of the cloud, not the paper's "
                        f"{float(target_fraction):.3f} — the radius hit a bound "
                        f"({lo_b:g}-{hi_b:g} mm)")
    return mask, info


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
                min_region_pts: int = 30, radius_mm: float | None = None,
                seed_edges: bool = True
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

    `seed_edges` is Alg. 1 lines 12-14 read literally, and it is the default because that is
    what the pseudocode says. Lines 7-10 sort the neighbour into `S_edges` or `S_surfaces`;
    lines 12-14 then test `Curvature < Threshold2` on **every** neighbour, the edge ones
    included, and the first implementation here `continue`d out of the loop body after
    marking an edge so that branch never ran. In practice the difference is small — an edge
    point is by construction a high-curvature point, so it rarely passes Threshold2 — but
    "small" is a measurement, not an assumption, and `seed_edges=False` restores the older
    behaviour so the two can be compared. Edge points are still never *labelled*: Alg. 1
    puts them in `S_edges`, not in a surface, and the two-surface test of §III-D needs them
    to stay unassigned so a ball around one can see labelled surface on both sides.

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
    # Edge points are never labelled, so `labels != -1` cannot stop one being re-queued.
    # This is the visited set Alg. 1 omits (Deviation 1), restricted to the edge branch so
    # that `seed_edges=False` reproduces the older behaviour exactly.
    queued_edge = np.zeros(n, dtype=bool)

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
                    is_edge[j] = True                  # Alg. 1 line 8: S_edges
                    if seed_edges and not queued_edge[j]:
                        queued_edge[j] = True
                        if curvature[j] < curvature_thresh:   # Alg. 1 lines 12-13, which
                            seeds.append(j)                   # apply to EVERY neighbour
                    continue
                labels[j] = label                      # Alg. 1 line 10: S_surfaces
                members += 1
                if curvature[j] < curvature_thresh:    # Alg. 1 lines 12-13
                    seeds.append(j)
        if members < min_region_pts:                   # too small to be a surface
            labels[labels == label] = -1
        else:
            label += 1
    return labels, is_edge


def _labels_for(src_pts: np.ndarray, src_labels: np.ndarray, dst_pts: np.ndarray
                ) -> np.ndarray:
    """Carry per-point labels through the voxel merge, by nearest neighbour.

    `preprocess` replaces each voxel with its centroid, so the label array the caller passed
    no longer indexes the cloud the rest of the method sees. Nearest-neighbour is exact
    wherever a voxel holds one surface and picks the majority side where it straddles two —
    which is the same ambiguity the voxel merge introduces for every other quantity.
    """
    if cKDTree is None:                                # pragma: no cover
        raise ImportError("lit-regiongrow needs scipy.spatial.cKDTree")
    _, idx = cKDTree(np.asarray(src_pts, float)).query(dst_pts, k=1, workers=-1)
    return np.asarray(src_labels).astype(np.int64)[idx]


def _edge_from_labels(pts: np.ndarray, labels: np.ndarray, normals: np.ndarray, k: int,
                      smoothness_deg: float, radius_mm: float | None,
                      label_radius_mm: float | None = None,
                      require_label_span: bool = True) -> np.ndarray:
    """Alg. 1's edge test, run against supplied surfaces instead of grown ones.

    Two clauses, and the first implementation ran only one of them — it took `labels` and
    never read it, so the supplied-surface arm had no label-boundary test at all and every
    normal wobble in the cloud came back as an edge. Both clauses now run, AND-combined:

    * **the normal turns** by more than `smoothness_deg` somewhere in the neighbourhood.
      This is Alg. 1 line 7 (Threshold1) verbatim.
    * **the neighbourhood spans two surfaces.** Alg. 1's growth sorts a neighbour into
      `S_edges` or `S_surfaces` *relative to the region it is growing*, so "edge" means
      "where this surface stops". With the surfaces handed over, the same statement is
      "where two of the supplied labels meet", tested at `label_radius_mm` — about one
      voxel, i.e. the smallest neighbourhood in which two labels can be adjacent at all.
      Widen it and the band thickens by the same amount on each side.

    Own label included, so a point sitting alone in an unlabelled pocket does not qualify;
    negative labels are unassigned and never count as a surface. `require_label_span=False`
    restores the older normal-only behaviour for an A/B.
    """
    tree = cKDTree(pts)
    cos_thresh = np.cos(np.radians(float(smoothness_deg)))
    if radius_mm is None:
        # The k-NN case is a rectangular (N, k) index array, so the whole test is one
        # einsum. The loop this replaces was 150 000 numpy calls per scene, and this runs
        # once per scene per arm across a 100-scene corpus.
        idx = np.atleast_2d(tree.query(pts, k=min(k, len(pts)), workers=-1)[1])
        cos = np.abs(np.einsum("nkj,nj->nk", normals[idx], normals))
        angle = cos.min(axis=1) < cos_thresh
    else:
        angle = np.zeros(len(pts), dtype=bool)
        for i, nb in enumerate(tree.query_ball_point(pts, radius_mm, workers=-1)):
            if len(nb) and np.min(np.abs(normals[np.asarray(nb, dtype=int)] @ normals[i])) \
                    < cos_thresh:
                angle[i] = True
    if not require_label_span:
        return angle
    return angle & _spans_two_labels(pts, labels, label_radius_mm, k, tree)


def _spans_two_labels(pts: np.ndarray, labels: np.ndarray, radius_mm: float | None,
                      k: int, tree=None) -> np.ndarray:
    """Does each point's ~1-voxel ball contain **two** distinct surfaces? §III-D's clause 2.

    A bounded k-NN query rather than `query_ball_point`, because this runs once per point
    per scene over a 100-scene corpus and the ball form is a Python loop. Anything past the
    radius comes back as `inf` and is dropped, so up to the neighbour count this is a true
    ball test. That count is floored at 12: at a one-voxel radius on a voxel grid only the
    point itself and its six face-adjacent voxels are inside, so 12 leaves headroom for the
    irregular spacing voxel *centroids* actually have.
    """
    pts = np.asarray(pts, dtype=float)
    labels = np.asarray(labels).astype(np.int64)
    n = len(pts)
    if n == 0:
        return np.zeros(0, dtype=bool)
    if radius_mm is None or not np.isfinite(radius_mm):
        return np.ones(n, dtype=bool)
    tree = tree if tree is not None else cKDTree(pts)
    kk = int(min(max(int(k), 12), n))
    dist, idx = tree.query(pts, k=kk, workers=-1,
                           distance_upper_bound=float(radius_mm))
    dist = np.atleast_2d(dist)
    idx = np.atleast_2d(idx)
    inside = np.isfinite(dist)
    lab = np.where(inside, labels[np.minimum(idx, n - 1)], -1)
    lab = np.where(lab >= 0, lab, -1)
    s = np.sort(lab, axis=1)                           # -1 (unassigned) sorts to the front
    distinct = ((s[:, 1:] != s[:, :-1]) & (s[:, 1:] >= 0)).sum(axis=1) + (s[:, 0] >= 0)
    return distinct >= 2


def two_surface_edges(pts: np.ndarray, labels: np.ndarray, is_edge: np.ndarray,
                      edge_radius_mm: float, min_region_share: float = 0.15,
                      curvature: np.ndarray | None = None,
                      edge_curv_thresh: float = CURVATURE_THRESH) -> np.ndarray:
    """§III-D's edge test, **both clauses**: high curvature AND between two surfaces.

    *"the welding seam edges not only need to have high curvature but also need to represent
    the intersection between two surfaces"* (p. 4, right column). This function tested only
    the second clause until the over-detection audit; `curvature` supplies the first.

    Clause 1, the curvature gate: `δ = λ₀/(λ₀+λ₁+λ₂)` of eq. 3 must be at least
    `edge_curv_thresh`. The default is `CURVATURE_THRESH` (0,03) — the same value Alg. 1
    uses for Threshold2 and the same value `ours` thresholds on, so the gate introduces no
    new tuned constant, and the three quantities stay comparable by construction. Passing
    `curvature=None` disables the gate and is the pre-audit behaviour, kept for the A/B.

    Clause 2, the two-surface test: high curvature alone is not enough and the paper says so
    — a plate's own outer rim is a boundary too. Kept only if the point's neighbourhood
    spans **two** regions, each holding at least `min_region_share` of it, so a stray
    neighbour or two does not qualify a rim point.

    This is `ours`' cross-object gate with the oracle removed: the regions come from the
    method's own segmentation rather than from `object_id`. That substitution is the single
    most transferable idea in this paper for the rest of Phase 4.
    """
    pts = np.asarray(pts, dtype=float)
    gate = np.asarray(is_edge, dtype=bool)
    if curvature is not None:                          # §III-D clause 1: "high curvature"
        gate = gate & (np.asarray(curvature, dtype=float) >= float(edge_curv_thresh))
    cand = np.flatnonzero(gate)
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
           edge_curvature_gate: bool = True,
           edge_curv_thresh: float | None = None,
           seed_edges: bool = True,
           label_span: bool = True,
           label_span_radius_mm: float | None = None,
           link_mm: float | None = None,
           min_cluster_pts: int = 8,
           curve: str = "line",
           poly_degree: int = 2,
           segmentation_mask: np.ndarray | None = None,
           region_labels: np.ndarray | None = None,
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
        edge_curvature_gate: §III-D clause 1, *"not only ... high curvature but also ..."*.
            **Default ON.** With it off (the pre-audit behaviour) any label or region
            boundary is an edge, however flat the surface is across it, and on this corpus
            that returned 9-31 % of the cloud as seam.
        edge_curv_thresh: the δ the gate compares against. Defaults to `curvature_thresh`,
            i.e. Alg. 1's own Threshold2 and `ours`' threshold for the same feature — so the
            gate adds **no new tuned constant**.
        seed_edges: Alg. 1 lines 12-14 applied to every neighbour, edges included, which is
            what the pseudocode says. Default ON; `False` is the older reading.
        label_span: with `region_labels` supplied, require the neighbourhood to span two
            surfaces as well as the normal to turn. Default ON — see `_edge_from_labels`.
        label_span_radius_mm: radius of that test. Defaults to **one voxel**: the smallest
            ball in which two labels can be adjacent at all. Widening it thickens the
            returned band symmetrically.
        curve: `"line"` for the paper's three linear workpieces, `"poly"` for its curved one.
        segmentation_mask: the §III-C crop, supplied as an **oracle**. Which crop is the
            question — `keypoint_crop` keeps the paper's 12,7 %, `surface_intersection_crop`
            keeps 54-97 % of these scenes, and the difference is the method's headline
            failure mode here. See Deviation 6 and `region_labels`.
        region_labels: per-point **surface** labels, supplied instead of grown. This is what
            §III-C's FastSAM actually returns: it is prompted at the centre of each workpiece
            *surface* and gives one mask per surface, and the weld region is then *derived*
            as the area where two surfaces meet. Supplying only a seam-band crop and making
            Alg. 1 re-grow the regions gives the method a band centred on the answer while
            withholding the labels its own pipeline already has — generous in one direction
            and stingy in the other. Pass `surface_labels_oracle(face_id)` for the rung the
            paper actually occupies. Points whose label is negative are treated as
            unassigned, exactly as Alg. 1 leaves its edge points.
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
    label_span_radius_mm = (float(label_span_radius_mm) if label_span_radius_mm
                            else 1.0 * vox)
    edge_curv_thresh = (float(edge_curv_thresh) if edge_curv_thresh is not None
                        else float(curvature_thresh))
    ball = radius_mm if neighbourhood == "radius" else None
    if neighbourhood not in ("knn", "radius"):
        raise ValueError(f"unknown neighbourhood {neighbourhood!r}")

    params: dict[str, Any] = dict(
        voxel_mm=voxel_mm, k=k, smoothness_deg=smoothness_deg,
        curvature_thresh=curvature_thresh, neighbourhood=neighbourhood,
        radius_mm=radius_mm, edge_radius_mm=edge_radius_mm,
        min_region_share=min_region_share, link_mm=link_mm, curve=curve,
        min_region_pts=min_region_pts, min_cluster_pts=min_cluster_pts,
        edge_curvature_gate=bool(edge_curvature_gate),
        edge_curv_thresh=edge_curv_thresh if edge_curvature_gate else None,
        seed_edges=bool(seed_edges), label_span=bool(label_span),
        label_span_radius_mm=label_span_radius_mm,
        # §IV-A publishes 3 mm; the harness passes 1,5 because 3 measures worse on 8 mm
        # plates (Deviation 7). Both values travel with the row so the deviation is never
        # read off as the paper's setting.
        voxel_mm_published=VOXEL_MM,
        # NOT the paper's — it publishes no value for any of these. Named here so that any
        # result turning on one of them is legible as a result about this reimplementation.
        # `k`, `smoothness_deg` and `curvature_thresh` are unpublished too (Deviation 2) and
        # are already above; these five are additionally *invented*, having no counterpart
        # in the paper at all.
        invented=("min_region_pts", "min_cluster_pts", "link_mm", "edge_radius_mm",
                  "min_region_share"),
        n_input=len(P))
    empty = np.zeros((0, 3))
    if len(P) < max(k, 4):
        return RegionGrowResult([], [], [], P, np.zeros(len(P), int), np.zeros(len(P)),
                                np.zeros(len(P), bool), np.zeros(len(P), bool), params,
                                used_oracle, "cloud too small to grow a region")

    normals, curv = local_pca(P, k, ball)
    if region_labels is None:
        labels, is_edge = region_grow(P, normals, curv, k, smoothness_deg, curvature_thresh,
                                      min_region_pts, ball, seed_edges)
    else:
        # Supplied surfaces replace Alg. 1's growth, not its edge test: §III-D still has to
        # decide which points sit ON a junction, and that is the normal-angle threshold.
        labels = _labels_for(pts, region_labels, P)
        is_edge = _edge_from_labels(P, labels, normals, k, smoothness_deg, ball,
                                    label_span_radius_mm, label_span)
        params["region_labels"] = "supplied"
    seam_mask = two_surface_edges(P, labels, is_edge, edge_radius_mm, min_region_share,
                                  curv if edge_curvature_gate else None, edge_curv_thresh)

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
