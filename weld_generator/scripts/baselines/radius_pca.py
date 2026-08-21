"""`ours` — radius-PCA surface variation. One of seven methods in the Phase 4 comparison.

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

def validity_window_mm(root_gap_mm: float, spacing_mm: float, thickness_mm: float,
                       upper: str = "half_thickness") -> tuple[float, float]:
    """`(lo, hi)` bounds on the PCA radius. Empty (lo >= hi) means the method cannot work.

    README §8 states the claim: the ball must cross the gap and must not bridge a plate's
    own two faces.

        lo = root_gap + point_spacing        the ball must reach across the gap
        hi = thickness / 2                   the BALL must not span the plate

    **The upper bound is a correction, measured 2026-08-20.** The claim was written as
    `R < thickness`, and the derivation does not support it: what bridges a plate's two
    faces is the ball's **diameter**, not its radius. At `R` just under `t` the ball is
    nearly `2t` across, so a point on the top face has the bottom face inside its
    neighbourhood and the covariance is computed over both — which is exactly the failure
    the bound exists to prevent. On this corpus a "valid" `R` under the old bound gave a ball
    spanning 1,2–1,9x the plate.

    Measured paired on 44 T / corner / butt / lap scenes where both bounds leave the window
    open, changing nothing else:

        F1          0,709 -> 0,897     better in 38 of 44 scenes
        precision   0,583 -> 0,839     better in 39 of 44
        band width  5,70 -> 3,31 mm    better in 44 of 44
        Chamfer     5,05 -> 3,68 mm    better in 42 of 44

    Pass `upper="thickness"` for the original claim; it is kept so the correction stays
    falsifiable rather than being quietly absorbed.

    The point of returning a window rather than asserting one: on thin sheet it **closes**,
    and a baseline with no valid radius at all is a result about the whole method class, not
    a configuration error to be tuned away. The corrected bound closes it more often — 44 of
    70 runs against 70 of 70 — and that is a cost of the correction, not an argument against
    it: those runs were previously "valid" only because the bound was wrong.
    """
    lo = float(root_gap_mm) + float(spacing_mm)
    if upper == "half_thickness":
        return lo, 0.5 * float(thickness_mm)
    if upper == "thickness":
        return lo, float(thickness_mm)
    raise ValueError(f"upper must be 'half_thickness' or 'thickness', got {upper!r}")


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


def density_anomaly(pts: np.ndarray, radius_mm: float, chunk: int = 20_000
                    ) -> np.ndarray:
    """Local point density, normalised by the set's own median. 1,0 = typical.

    Measured on the **raw** band, before any voxel step. That ordering is the whole point:
    `voxel_downsample` makes density uniform by construction, so computing this after it
    returns 1,0 everywhere and the signal is gone.

    Two surfaces converge at a fold, so a fold seam sits in a density *surplus*; a coplanar
    seam is a gap in a flat surface, so it sits in a *deficit*. Measured on the band,
    normalised, against distance from the true seam in units of R:

    | joint | 0-,25R | ,25-,5R | ,5-1R | 1-1,5R |
    |---|---|---|---|---|
    | corner | **1,26** | 1,10 | 0,85 | 0,47 |
    | lap | **1,16** | 1,09 | 0,88 | 0,97 |
    | butt | **1,14** | 1,03 | 0,85 | 0,61 |
    | T | **1,05** | 1,03 | 0,98 | 0,64 |
    | edge | **0,62** | 0,71 | 0,71 | 0,63 |

    So `|1 - density|` responds on every joint type and **the sign says which family the
    seam belongs to** - which is the same measurement the gap channel wanted for coplanar
    seams, read in the other direction.

    Weaker than `V` (a 1,05-1,26x surplus against `V`'s much sharper response), so it is a
    tie-breaker and a clustering weight rather than a detector in its own right.

    Normalised by the median rather than an absolute expectation, so it needs no density
    parameter and behaves the same at 0,25 and 4 pts/mm².
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0 or cKDTree is None:               # pragma: no cover
        return np.ones(len(pts))
    tree = cKDTree(pts)
    counts = np.empty(len(pts))
    for start in range(0, len(pts), chunk):
        sl = slice(start, min(start + chunk, len(pts)))
        counts[sl] = [len(x) for x in tree.query_ball_point(pts[sl], radius_mm,
                                                            workers=-1)]
    med = float(np.median(counts))
    return counts / med if med > 0 else np.ones(len(pts))


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


def dbscan_components(pts: np.ndarray, eps_mm: float, min_samples: int) -> np.ndarray:
    """Density-based clustering. Returns labels; **-1 marks noise**.

    The difference from `connected_components` is entirely in that noise label, and it is
    the point. Plain linking is DBSCAN with `min_samples = 1`: every point is a core point,
    so a thin trail of stragglers between two seams chains them into one cluster. That is
    the failure behind every wrong seam count - one component spanning both ground-truth
    seams, and a fitted polyline zigzagging between them by up to 65 degrees.

    With `min_samples > 1` the sparse in-between points are not core points, so they join a
    cluster only if they are reachable from one and never act as bridges themselves. The
    band supports this because the density really does dip between seams: normalised to the
    band median it runs ~1,05-1,26 at the seam and ~0,47-0,64 at 1-1,5 R out.

    **Must run on the raw band.** `voxel_downsample` makes density uniform by construction,
    so after it the dip DBSCAN needs is gone and it degenerates to plain linking.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return np.zeros(0, dtype=int)
    try:
        from sklearn.cluster import DBSCAN
    except ImportError:                                # pragma: no cover
        raise ImportError("cluster_method='dbscan' needs scikit-learn; "
                          "pip install scikit-learn") from None
    return DBSCAN(eps=float(eps_mm), min_samples=int(min_samples)).fit(pts).labels_


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


def local_tangent(pts: np.ndarray, radius_mm: float, min_neighbors: int = 4,
                  chunk: int = 20_000) -> np.ndarray:
    """Unit direction the seam runs in at each point — the top PCA axis of a local ball.

    On a seam band the points form a ribbon, so the largest-variance direction is *along*
    the seam. Sign is arbitrary out of an eigen-decomposition and is deliberately left that
    way: every consumer here compares `|t_i . t_j|`, because a curve has a direction but not
    an orientation and forcing a sign would split a straight seam wherever the flip landed.
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    out = np.zeros((n, 3))
    if n == 0:
        return out
    tree = cKDTree(pts)
    for start in range(0, n, chunk):
        sl = slice(start, min(start + chunk, n))
        for j, ids in enumerate(tree.query_ball_point(pts[sl], radius_mm, workers=-1)):
            ids = np.asarray(ids, dtype=int)
            i = start + j
            if len(ids) < min_neighbors:
                continue
            c = pts[ids] - pts[ids].mean(axis=0)
            _, v = np.linalg.eigh(c.T @ c)
            out[i] = v[:, -1]
    return out


def lateral_split(pts: np.ndarray, link_mm: float, min_pts: int = 4) -> np.ndarray:
    """Split one direction-consistent component into parallel runs. Label per point.

    The second half of the problem, and the half a direction test cannot touch: **two
    parallel seams have the same direction.** A T joint's two fillets differ by one
    plate-thickness of *lateral offset*, not by any angle, so no tangent comparison
    separates them and no link distance does either — they are proximity-connected across
    the plate.

    What does separate them is where they sit once the along-seam coordinate is removed.
    Project onto the plane perpendicular to the component's own principal axis and cluster
    *there*: a single seam collapses to one spot, two parallel seams to two spots a plate
    apart. That is the same trick §III-D of `lit_lobb` uses for fitting, applied to
    clustering instead.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < min_pts:
        return np.zeros(len(pts), dtype=int)
    c = pts - pts.mean(axis=0)
    _, v = np.linalg.eigh(c.T @ c)
    axis = v[:, -1]
    perp = c - np.outer(c @ axis, axis)                # drop the along-seam coordinate
    return connected_components(perp, link_mm)


def directional_components(pts: np.ndarray, link_mm: float, tangent_radius_mm: float,
                           max_turn_deg: float = 30.0, min_neighbors: int = 4,
                           lateral_link_mm: float | None = None) -> np.ndarray:
    """Components that link on proximity **and** direction. Component label per point.

    `connected_components` is the standard choice and it has one failure this dataset keeps
    producing: two seams that run parallel a plate-thickness apart are proximity-connected,
    and a T joint's contact perimeter is *closed* — its two fillets are bridged at the ends
    by short cross-runs D4 excludes from the label. No link distance separates either case,
    because the quantity that distinguishes them is **direction**, and proximity never
    consults it.

    Two points join only when they are within `link_mm` **and the step between them runs
    along both their local tangents** to within `max_turn_deg`. Testing the *edge direction*
    rather than the two tangents against each other is what makes it work on both failure
    modes: a step onto a cross-run is perpendicular to the fillet it leaves, and a step
    across to a parallel fillet is perpendicular to both — while a step along either seam is
    not. Comparing tangents to each other cuts neither, because a smoothed tangent blends
    through a corner and two parallel seams have the same tangent by construction.

    **This is an upgrade to `ours`, and it is deliberately not applied to any `lit-*`
    method.** Neither Wei et al. nor Zhang et al. specifies a clustering step at all
    (`lit_regiongrow` deviation 5, `lit_lobb` deviation 4), so replacing the standard choice
    with a better one there would make the reimplementation outperform the paper — the same
    fidelity failure as making it worse, in the flattering direction, and it would leave
    every number unattributable. Use `cluster_method="directional"` here; keep the
    literature on `connected_components`; and if the question is *"how much of each method's
    error is clustering?"*, run this as a clearly labelled diagnostic arm across all of them
    and never quote it as a paper's result.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return np.zeros(0, dtype=int)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components as _cc

    pairs = cKDTree(pts).query_pairs(link_mm, output_type="ndarray")
    if len(pairs) == 0:
        return np.arange(len(pts))

    t = local_tangent(pts, tangent_radius_mm, min_neighbors)

    # The test is on the EDGE direction, not on the two tangents against each other.
    #
    # Comparing tangents fails on both cases this exists for. At a sharp corner the tangent
    # is smoothed over its own ball, so a fillet point and a cross-run point a millimetre
    # apart have nearly the same blended tangent and the bridge survives - measured, it did.
    # And two PARALLEL seams have identical tangents by definition, so that test can never
    # separate them at all.
    #
    # Requiring the connecting edge to run ALONG both tangents fixes both at once: a step
    # from a fillet onto a cross-run is perpendicular to the fillet, and a step across to
    # the parallel fillet is perpendicular to both. A step along either seam is not.
    d = pts[pairs[:, 1]] - pts[pairs[:, 0]]
    n = np.linalg.norm(d, axis=1)
    e = d / np.where(n[:, None] > 1e-12, n[:, None], 1.0)
    # `|cos|`, not `cos`: the eigenvector sign is arbitrary, so only an unsigned comparison
    # means anything. A point with no tangent (too few neighbours) keeps its proximity links
    # rather than being cut loose.
    ok = np.cos(np.radians(float(max_turn_deg)))
    have0 = np.linalg.norm(t[pairs[:, 0]], axis=1) > 0.5
    have1 = np.linalg.norm(t[pairs[:, 1]], axis=1) > 0.5
    a0 = (~have0) | (np.abs(np.einsum("ij,ij->i", e, t[pairs[:, 0]])) >= ok)
    a1 = (~have1) | (np.abs(np.einsum("ij,ij->i", e, t[pairs[:, 1]])) >= ok)
    pairs = pairs[a0 & a1]
    if len(pairs) == 0:
        return np.arange(len(pts))

    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                   shape=(len(pts), len(pts)))
    _, labels = _cc(g, directed=False)

    # Stage two. Cutting the cross-runs leaves each fillet pair still joined along its
    # length, because they are PARALLEL - identical direction, one plate-thickness apart.
    # Measured before this was added: direction alone split 1 of 7 T and corner scenes.
    lat = float(lateral_link_mm) if lateral_link_mm else float(link_mm)
    out = np.full(len(pts), -1, dtype=int)
    nxt = 0
    for lab in np.unique(labels):
        m = np.flatnonzero(labels == lab)
        sub = lateral_split(pts[m], lat, min_neighbors)
        for s_lab in np.unique(sub):
            out[m[sub == s_lab]] = nxt
            nxt += 1
    return out


@dataclass
class RadiusPCAResult:
    """What the baseline returned, and enough context to know what it means."""

    band: np.ndarray                      #: (N,3) the detection - the whole band
    #: The band split into clusters, one point set each. NOT fitted lines: the line fit is
    #: gone on purpose. On these generated joints the band is a RECTANGLE and a
    #: total-least-squares line through it lands in the middle of that rectangle, which is
    #: not where the seam is - it is the mid-surface between two plates. The band is what
    #: the method actually knows; the line was an assumption laid on top of it.
    clusters: list[np.ndarray]
    labels: np.ndarray                    #: component label per band point
    variation: np.ndarray                 #: V for every input point - the raw signal
    params: dict[str, Any] = field(default_factory=dict)
    #: True when object membership was used. An oracle: no sensor provides it, so a
    #: number produced with this set is not a point-cloud-only result.
    used_object_oracle: bool = False
    note: str = ""
    #: The band BEFORE the voxel merge, and its local density normalised to the set median.
    #: Kept because `voxel_downsample` flattens density by construction, so after it the
    #: signal no longer exists to be measured. Exposed, not yet acted on.
    band_raw: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    band_density: np.ndarray = field(default_factory=lambda: np.zeros(0))

    @property
    def n_clusters(self) -> int:
        return len(self.clusters)


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
           prefilter_density_per_mm2: float | None = None,
           density_radius_mm: float | None = None,
           band_sampling: str = "voxel",
           cluster_method: str = "components",
           tangent_radius_mm: float | None = None,
           max_turn_deg: float = 30.0,
           lateral_link_mm: float | None = None,
           dbscan_eps_mm: float | None = None,
           dbscan_min_samples: int = 6,
           density_keep: float | None = None) -> RadiusPCAResult:
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
        density_radius_mm: ball for the density anomaly (`density_anomaly`), default
            1,5 x the point spacing. Small on purpose - it has to resolve a surplus that
            is only 1,05-1,26x, so a large ball averages it away.
        band_sampling: `"voxel"` (default, the original) merges the band on a voxel grid
            coarser than the gap, which averages the two parallel edge lines into one.
            `"raw"` keeps every band point. Voxelising **destroys the density signal**, so
            `"raw"` is required for `cluster_method="dbscan"` to do anything.
        cluster_method: `"components"` (default, the original), `"dbscan"`, or
            `"directional"`. See `directional_components` — proximity alone cannot separate
            two parallel centrelines a plate-thickness apart, nor cut a closed contact
            perimeter into the open runs that are actually welded, and both cases occur in
            every T joint in this dataset.
        tangent_radius_mm: ball for the local seam direction used by `"directional"`.
            Default `2 x radius_mm` — it has to span enough of the ribbon to see which way
            it runs, and stay short enough to turn with a curved seam.
        lateral_link_mm: link distance for the second stage of `"directional"`, applied
            *across* the seam after the along-seam coordinate is projected out. It must be
            **below the separation between two parallel seams** — a plate thickness — and
            above the band's own width, which is what makes it the sensitive knob. Defaults
            to `link_mm`.
        max_turn_deg: how far the step between two points may lie off their own tangents
            and still link them. A cross-run meets a
            fillet at ~90 deg, so anything well under that cuts the bridge; the default is
            set by how much a *curved* seam may turn within `link_mm`, not by the bridge.
        dbscan_eps_mm: neighbourhood for DBSCAN, default 2 x the point spacing. It must be
            **below the seam separation** or the two seams are density-reachable from each
            other and no `min_samples` can separate them.
        dbscan_min_samples: core-point threshold. This is the knob that decides what counts
            as a bridge rather than a seam.
        density_keep: if set, drop band points whose `density_anomaly` is below this before
            clustering. A direct filter on the same dip DBSCAN exploits - cruder, but it
            makes the effect visible on its own.
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
    spacing_hint = (1.0 / np.sqrt(float(prefilter_density_per_mm2))
                    if prefilter_density_per_mm2 else radius_mm / 4.0)
    voxel_mm = float(voxel_mm) if voxel_mm else 2.0 * radius_mm / 3.0
    link_mm = float(link_mm) if link_mm else float(radius_mm)
    params = dict(radius_mm=radius_mm, curvature_thresh=curvature_thresh,
                  min_neighbors=min_neighbors, voxel_mm=voxel_mm, link_mm=link_mm,
                  cross_object=cross_object, min_component_pts=min_component_pts,
                  n_input=len(pts))
    empty = np.zeros((0, 3))

    if len(pts) < min_neighbors:
        return RadiusPCAResult(empty, [], np.zeros(0, int), np.zeros(len(pts)),
                               params=params, note="too few points")

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
            empty, [], np.zeros(0, int), var, params=params,
            note=f"no point above V={curvature_thresh:g} (max {var.max():.4f}); the radius "
                 f"is probably below the gap")

    used_oracle = False
    if cross_object and object_id is not None:
        keep &= cross_object_mask(pts, np.asarray(object_id), radius_mm, keep)
        used_oracle = True
        params["n_cross_object"] = int(keep.sum())
        if not keep.any():
            return RadiusPCAResult(
                empty, [], np.zeros(0, int), var, params=params, used_object_oracle=True,
                note="high-curvature points exist but none lie within R of another object")

    # Density is measured on the RAW band and carried alongside: the voxel merge below
    # makes density uniform, so after it the signal no longer exists to be measured.
    band_raw = pts[keep]
    band_density = density_anomaly(band_raw, density_radius_mm or (1.5 * spacing_hint))
    params["density_radius_mm"] = float(density_radius_mm or (1.5 * spacing_hint))

    if density_keep is not None:
        surviving = band_density >= float(density_keep)
        params["n_density_kept"] = int(surviving.sum())
        if surviving.sum() >= min_component_pts:
            band_raw = band_raw[surviving]
            band_density = band_density[surviving]

    if band_sampling not in ("voxel", "raw"):
        raise ValueError(f"band_sampling must be 'voxel' or 'raw', got {band_sampling!r}")
    band = voxel_downsample(band_raw, voxel_mm) if band_sampling == "voxel" else band_raw
    params["band_sampling"] = band_sampling

    if cluster_method == "dbscan":
        eps = float(dbscan_eps_mm) if dbscan_eps_mm else 2.0 * spacing_hint
        params["dbscan_eps_mm"] = eps
        params["dbscan_min_samples"] = int(dbscan_min_samples)
        labels = dbscan_components(band, eps, dbscan_min_samples)
        params["n_noise"] = int((labels < 0).sum())
    elif cluster_method == "components":
        labels = connected_components(band, link_mm)
    elif cluster_method == "directional":
        tr = float(tangent_radius_mm) if tangent_radius_mm else 2.0 * radius_mm
        params["tangent_radius_mm"] = tr
        params["max_turn_deg"] = float(max_turn_deg)
        labels = directional_components(band, link_mm, tr, max_turn_deg)
    else:
        raise ValueError("cluster_method must be 'components', 'dbscan' or 'directional', "
                         f"got {cluster_method!r}")
    params["cluster_method"] = cluster_method

    clusters: list[np.ndarray] = []
    kept_labels = np.full(len(band), -1)
    for lab in np.unique(labels):
        if lab < 0:
            continue                                   # DBSCAN noise: the bridge points
        m = labels == lab
        if int(m.sum()) < min_component_pts:
            continue                                   # a speck, not a seam
        kept_labels[m] = len(clusters)
        clusters.append(band[m])

    params["n_components"] = int(len(np.unique(labels)))
    return RadiusPCAResult(band, clusters, kept_labels, var,
                           band_raw=band_raw, band_density=band_density,
                           params=params, used_object_oracle=used_oracle,
                           note=f"{len(clusters)} cluster(s), {len(band)} band points "
                                f"from {int(keep.sum())} above threshold")
