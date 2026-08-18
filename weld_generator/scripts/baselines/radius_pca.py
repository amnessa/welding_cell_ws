"""Baseline A (ours) — radius-PCA surface variation.

Migrated from `ros2_ws/src/admittance_control/scripts/icp_pose_refiner_node.py`
(`surface_variation`, `_welding_points`, `_cross_object_mask`) and the voxel merge in
`admittance_control/icp.py`. The method, unchanged in substance:

    C = (1/N) Σ (p - p̄)(p - p̄)ᵀ  over a RADIUS ball,   V = λ₃ / (λ₁ + λ₂ + λ₃)

`V` is the fraction of variance normal to the best-fit local plane: ~0 on a flat face, and
0.05–0.15 at a fold. **Radius search, not k-NN**, is the one substitution that makes it
work — the parts do not touch, and a k-NN ball on a point at part A's edge contains only
part-A points, so it looks flat. A radius ball wider than the gap reaches across it.

`R` is bounded on both sides, which is the repo's falsifiable prediction and the reason
this dataset exists:

    gap + point_spacing  <  R  <  part_thickness

Too small and the ball never crosses the gap; too large and it bridges a plate's *own two
faces*, at which point every point looks like an edge.

Four things changed in the migration, all of them things the number will now show:

1. **Metres → millimetres.** Every length is mm, as everywhere else in `weld_generator`.
2. **Object membership is an explicit oracle input, not a hidden assumption.** The ROS node
   recovers per-point object ids from the order objects were stacked into the SEPC, which
   exists only because the cloud was assembled from registered CAD. At runtime there is no
   such thing. It is kept, because it is what the published method does — but it is passed
   in by name and reported in the result, so a `cross_object=True` number can never be
   quoted as if it were a point-cloud-only result.
3. **Multi-seam.** The original assumes one seam and returns one band. Connected components
   on the surviving points, one line fit per component, so a T-joint's two fillets come back
   as two seams instead of one blob spanning both (`dataset_plan.md` Phase 4).
4. **Band → centreline.** README §8's honest caveat is that the published seam is a ~16 mm
   band, not a line, and that a true toolpath needs a line fit through it. That fit is here,
   so Chamfer is measured against a centreline rather than against a cloud whose width is
   most of the error.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:                                    # pragma: no cover
    cKDTree = None


# --------------------------------------------------------------------------------
# the validity window
# --------------------------------------------------------------------------------

def validity_window_mm(root_gap_mm: float, spacing_mm: float, thickness_mm: float
                       ) -> tuple[float, float]:
    """`(lo, hi)` bounds on the PCA radius. Empty (lo >= hi) means the method cannot work.

    Straight from README §8: the ball must cross the gap and must not bridge a plate's own
    two faces. On the reference T-joint (g = 1,1 mm, spacing = 2,7 mm, t = 8,4 mm) this is
    roughly 3,8 – 8,4 mm, and the repo's default R = 6 mm sits inside it.

    The point of returning it rather than asserting it: on thin sheet the window **closes**,
    and a baseline that has no valid radius at all is a result about the whole method class,
    not a configuration error to be tuned away.
    """
    return float(root_gap_mm) + float(spacing_mm), float(thickness_mm)


def mean_spacing_mm(pts: np.ndarray, sample: int = 2000, seed: int = 0) -> float:
    """Median nearest-neighbour distance — the `point_spacing` of the window above."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return 0.0
    rng = np.random.default_rng(seed)
    idx = (rng.choice(len(pts), size=sample, replace=False)
           if len(pts) > sample else np.arange(len(pts)))
    if cKDTree is None:                                # pragma: no cover
        d = np.linalg.norm(pts[idx][:, None, :] - pts[None, :, :], axis=-1)
        np.fill_diagonal(d[:, idx], np.inf) if len(idx) == len(pts) else None
        return float(np.median(np.sort(d, axis=1)[:, 1]))
    d, _ = cKDTree(pts).query(pts[idx], k=2, workers=-1)
    return float(np.median(d[:, 1]))


# --------------------------------------------------------------------------------
# the method
# --------------------------------------------------------------------------------

def surface_variation(pts: np.ndarray, radius_mm: float, min_neighbors: int = 5,
                      chunk: int = 20_000) -> np.ndarray:
    """Per-point `V = λ₃ / (λ₁ + λ₂ + λ₃)` over a radius ball. Same quantity, batched.

    The original loops in Python over one `np.cov` + `eigvalsh` per point, which is fine
    for a few-thousand-point SEPC and hopeless on a 400 k-point scene: sweeping a corpus
    never finished. The covariance is accumulated with `bincount` over the flattened
    neighbour pairs instead, and the eigenvalues come from one batched `eigvalsh` on an
    (N,3,3) stack. Identical arithmetic, ~100x faster.

    Chunked because the pair list is the memory cost: at 2,8 pts/mm² a 4 mm ball holds
    ~140 neighbours, so a 400 k-point cloud is 57 M pairs at once and `chunk` bounds it.

    Points with fewer than `min_neighbors` neighbours get `V = 0`: an isolated speck would
    otherwise produce a degenerate, high-variation covariance and read as a perfect edge.
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    var = np.zeros(n)
    if n == 0:
        return var
    if cKDTree is None:                                # pragma: no cover
        return _surface_variation_slow(pts, radius_mm, min_neighbors)

    tree = cKDTree(pts)
    for start in range(0, n, chunk):
        sl = slice(start, min(start + chunk, n))
        lists = tree.query_ball_point(pts[sl], radius_mm, workers=-1)
        counts = np.fromiter((len(x) for x in lists), dtype=np.int64, count=len(lists))
        if not counts.any():
            continue
        flat = np.concatenate([np.asarray(x, dtype=np.int64) for x in lists if x])
        owner = np.repeat(np.arange(len(lists)), counts)
        nb = pts[flat]
        m = len(lists)

        # E[x] and E[x xᵀ] per point, then cov = E[x xᵀ] - E[x]E[x]ᵀ. Only the six unique
        # components of the symmetric outer product are accumulated.
        cnt = np.maximum(counts, 1)
        mu = np.stack([np.bincount(owner, nb[:, c], minlength=m) / cnt
                       for c in range(3)], axis=1)
        cov = np.empty((m, 3, 3))
        for a in range(3):
            for b in range(a, 3):
                v = np.bincount(owner, nb[:, a] * nb[:, b], minlength=m) / cnt \
                    - mu[:, a] * mu[:, b]
                cov[:, a, b] = cov[:, b, a] = v

        lam = np.linalg.eigvalsh(cov)                  # ascending: λ₃ ≤ λ₂ ≤ λ₁
        total = lam.sum(axis=1)
        ok = (counts >= min_neighbors) & (total > 1e-18)
        v = np.zeros(m)
        v[ok] = np.maximum(lam[ok, 0], 0.0) / total[ok]
        var[sl] = v
    return var


def _surface_variation_slow(pts: np.ndarray, radius_mm: float, min_neighbors: int
                            ) -> np.ndarray:  # pragma: no cover
    """The original point-by-point form. Kept as the reference the fast path is checked
    against, and as the no-scipy fallback."""
    n = len(pts)
    var = np.zeros(n)
    d2 = np.sum((pts[:, None, :] - pts[None, :, :]) ** 2, axis=-1)
    for i in range(n):
        ids = np.flatnonzero(d2[i] <= radius_mm ** 2)
        if len(ids) < min_neighbors:
            continue
        nb = pts[ids]
        lam = np.linalg.eigvalsh(np.cov((nb - nb.mean(0)).T, bias=True))
        total = float(lam.sum())
        if total > 1e-18:
            var[i] = max(float(lam[0]), 0.0) / total
    return var


def cross_object_mask(pts: np.ndarray, object_id: np.ndarray, radius_mm: float,
                      candidates: np.ndarray) -> np.ndarray:
    """True where a candidate sees a point of a DIFFERENT object within `radius_mm`.

    Curvature alone is not enough: a plate's own outer border is a 90° fold too, and at
    R = 6 mm it accounted for half the high-`V` points in the original, turning the result
    into a 266×211×134 mm blob. This condition is true only along the joint.

    It is also an **oracle**: per-point object membership comes from the registered CAD
    assembly, and no sensor provides it. `detect(..., cross_object=False)` is the honest
    point-cloud-only arm; the difference between the two is a result worth reporting.
    """
    pts = np.asarray(pts, dtype=float)
    mask = np.zeros(len(pts), dtype=bool)
    idx = np.flatnonzero(candidates)
    if len(idx) == 0:
        return mask
    if cKDTree is not None:
        neighbours = cKDTree(pts).query_ball_point(pts[idx], radius_mm, workers=-1)
    else:                                              # pragma: no cover
        d2 = np.sum((pts[idx][:, None, :] - pts[None, :, :]) ** 2, axis=-1)
        neighbours = [np.flatnonzero(row <= radius_mm ** 2) for row in d2]
    for i, nb in zip(idx, neighbours):
        mask[i] = bool((np.asarray(object_id)[nb] != object_id[i]).any())
    return mask


def voxel_downsample(pts: np.ndarray, voxel_mm: float) -> np.ndarray:
    """Average points per voxel — the trick that merges the two parallel edge lines.

    Thresholding returns a line on each part, either side of the gap. A voxel coarser than
    the gap averages a straddling pair into one point in the middle of the joint.
    """
    pts = np.asarray(pts, dtype=float)
    if voxel_mm <= 0 or len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel_mm).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = (keys.max(axis=0) + 1).astype(object)
    if int(dims[0]) * int(dims[1]) * int(dims[2]) < (1 << 62):
        flat = (keys[:, 0] * int(dims[1]) + keys[:, 1]) * int(dims[2]) + keys[:, 2]
        _, inv = np.unique(flat, return_inverse=True)
    else:                                              # pragma: no cover
        _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    counts = np.bincount(inv)
    out = np.empty((len(counts), 3))
    for c in range(3):
        out[:, c] = np.bincount(inv, weights=pts[:, c]) / counts
    return out


def connected_components(pts: np.ndarray, link_mm: float) -> np.ndarray:
    """Component label per point, linking anything within `link_mm`.

    This is the multi-seam upgrade. Without it a T-joint's two fillets - genuinely two
    seams, on opposite sides of the standing plate - are returned as one point set, and a
    single line fit through both lands in the middle of neither.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return np.zeros(0, dtype=int)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components as _cc

    pairs = cKDTree(pts).query_pairs(link_mm, output_type="ndarray")
    if len(pairs) == 0:
        return np.arange(len(pts))
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                   shape=(len(pts), len(pts)))
    _, labels = _cc(g, directed=False)
    return labels


def fit_line(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Total-least-squares line through a band, returned as its two endpoints.

    README §8's caveat, answered: the thresholded result is a band ~16 mm wide, not a line,
    and "for a true single-line toolpath, fit a line/spline through the band and project
    onto it". Measuring Chamfer against the band instead charges the method for its own
    width, which is a property of the threshold rather than of where the seam is.
    """
    pts = np.asarray(pts, dtype=float)
    centre = pts.mean(axis=0)
    d = np.linalg.svd(pts - centre, full_matrices=False)[2][0]
    t = (pts - centre) @ d
    return centre + d * t.min(), centre + d * t.max()


@dataclass
class RadiusPCAResult:
    """What the baseline returned, and enough context to know what it means."""

    band: np.ndarray                      #: (N,3) surviving points, before the line fit
    seams: list[np.ndarray]               #: one (2,3) endpoint pair per component
    labels: np.ndarray                    #: component label per band point
    variation: np.ndarray                 #: V for every input point - the raw signal
    params: dict[str, Any] = field(default_factory=dict)
    #: True when object membership was used. An oracle: no sensor provides it, so a
    #: number produced with this set is not a point-cloud-only result.
    used_object_oracle: bool = False
    note: str = ""

    @property
    def n_seams(self) -> int:
        return len(self.seams)


def detect(pts: np.ndarray,
           radius_mm: float,
           curvature_thresh: float = 0.03,
           min_neighbors: int = 5,
           voxel_mm: float | None = None,
           object_id: np.ndarray | None = None,
           cross_object: bool = True,
           exterior: np.ndarray | None = None,
           min_component_pts: int = 8,
           link_mm: float | None = None,
           prefilter_density_per_mm2: float | None = None) -> RadiusPCAResult:
    """Run the baseline. Lengths in **millimetres**.

    Args:
        pts: (N,3) cloud. Pass the visible subset for the single-view arm, or all of it for
            the full-visibility arm; the plan wants both reported separately.
        radius_mm: the PCA ball. See `validity_window_mm` - outside that window the method
            is expected to fail, and that failure is a result.
        voxel_mm: merge voxel, default `2 x radius / 3`. Must exceed the gap or the two
            parallel edge lines survive as two seams.
        object_id: per-point object membership. **Oracle** - see `cross_object_mask`.
        exterior: optional boolean mask from `weldgen.visibility.hpr_exterior`, the
            CAD-free way to drop buried surface. This is the exteriority gate the plan
            asks for, and unlike `object_id` it IS available point-cloud-only.
        prefilter_density_per_mm2: voxel-downsample the cloud to about this density
            first. **Set it for any comparison across scenes.** `density_per_mm2` is a
            sampled axis (0,25 - 4 pts/mm²), so running at whatever a scene happened to
            draw conflates the density plot with every other plot - the method's own
            validity window has `point_spacing` in it, so density is not a nuisance
            parameter here, it is one of the four axes the window is a surface over.
            It also bounds the cost: a 400 k-point cloud at R = 5 mm is ~180 M neighbour
            pairs, which is where an unprefiltered corpus sweep runs out of memory.
        link_mm: component linking distance, default `radius_mm`. Bounded by the same
            argument as `R` itself: if the PCA ball must not bridge a plate's own two
            faces, neither must the thing that decides which points belong to one seam.
            The old `2 x voxel` default is `4R/3`, i.e. *wider* than the ball, and it
            merged a butt joint's top and bottom centrelines on a 6,6 mm plate.

            It remains the most sensitive knob in the method, and not in a way a default
            fixes - on one scene, `link = R` gave 1 seam where there were 2, `0,6 R` gave
            5, and `0,4 R` gave none. Proximity alone cannot separate two parallel
            centrelines a plate-thickness apart; splitting components by direction as well
            would, and that is an upgrade to make with a number in front of you.
    """
    pts = np.asarray(pts, dtype=float)
    voxel_mm = float(voxel_mm) if voxel_mm else 2.0 * radius_mm / 3.0
    link_mm = float(link_mm) if link_mm else float(radius_mm)
    params = dict(radius_mm=radius_mm, curvature_thresh=curvature_thresh,
                  min_neighbors=min_neighbors, voxel_mm=voxel_mm, link_mm=link_mm,
                  cross_object=cross_object, min_component_pts=min_component_pts,
                  n_input=len(pts))
    empty = np.zeros((0, 3))

    if len(pts) < min_neighbors:
        return RadiusPCAResult(empty, [], np.zeros(0, int), np.zeros(len(pts)),
                               params, False, "too few points")

    if prefilter_density_per_mm2:
        # Voxel side ~ mean spacing at the target density (spacing ~ 1/sqrt(rho)). Object
        # ids come along by nearest surviving point, since the voxel average is not one of
        # the originals.
        keep_pts = voxel_downsample(pts, 1.0 / np.sqrt(float(prefilter_density_per_mm2)))
        if object_id is not None and cKDTree is not None and len(keep_pts):
            object_id = np.asarray(object_id)[cKDTree(pts).query(keep_pts, k=1)[1]]
        pts = keep_pts
        params["n_prefiltered"] = len(pts)

    if exterior is not None:
        keep_idx = np.flatnonzero(np.asarray(exterior, dtype=bool))
        pts = pts[keep_idx]
        object_id = None if object_id is None else np.asarray(object_id)[keep_idx]
        params["n_exterior"] = len(pts)

    var = surface_variation(pts, radius_mm, min_neighbors)
    keep = var >= curvature_thresh
    params["n_above_threshold"] = int(keep.sum())
    if not keep.any():
        return RadiusPCAResult(
            empty, [], np.zeros(0, int), var, params, False,
            f"no point above V={curvature_thresh:g} (max {var.max():.4f}); the radius is "
            f"probably below the gap")

    used_oracle = False
    if cross_object and object_id is not None:
        keep &= cross_object_mask(pts, np.asarray(object_id), radius_mm, keep)
        used_oracle = True
        params["n_cross_object"] = int(keep.sum())
        if not keep.any():
            return RadiusPCAResult(
                empty, [], np.zeros(0, int), var, params, True,
                "high-curvature points exist but none lie within R of another object")

    band = voxel_downsample(pts[keep], voxel_mm)
    labels = connected_components(band, link_mm)

    seams: list[np.ndarray] = []
    kept_labels = np.full(len(band), -1)
    for lab in np.unique(labels):
        m = labels == lab
        if int(m.sum()) < min_component_pts:
            continue                                   # a speck, not a seam
        kept_labels[m] = len(seams)
        p0, p1 = fit_line(band[m])
        seams.append(np.stack([p0, p1]))

    params["n_components"] = int(len(np.unique(labels)))
    return RadiusPCAResult(band, seams, kept_labels, var, params, used_oracle,
                           f"{len(seams)} seam(s) from {int(keep.sum())} edge points")
