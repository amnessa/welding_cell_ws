"""`lit-quadric` — surface-shape recognition by normal statistics, least-squares plane /
quadric fits, and the seam as the intersection of two fitted welding surfaces.

A faithful reimplementation of:

    Dongmin Li, Yu Wang, Zhengyong Wang.
    "Automatic recognition on impeller shape and weld seam based on normal of point
    clouds and PCA." Journal of Mechanical Science and Technology 40 (3) (2026)
    2039-2046. DOI 10.1007/s12206-026-0243-8.

Their pipeline, section by section, and where each lands here:

    §2.1  Gaussian filter (denoise)                        -> `gaussian_smooth`
    §2.2  voxel-centroid filter (downsample)               -> `voxel_downsample`
    §2.3  region growing splits the cloud into the two     -> L0: the `surfaces` ORACLE
          welding surfaces (impeller hub / fan blade)         (per-face labels + part
                                                              membership); L1: grown here
                                                              with Wei et al.'s Alg. 1
    §3.1  PCA normals (eq. 1-4); a surface is FLAT when     -> `local_pca`, `surface_kind`
          the normal-deviation distribution is concentrated
          ("over 90 % within the standard deviation", eq. 5-6)
    §3.2  flat: plane by least squares (eq. 7-11);          -> `fit_plane`, `fit_quadric`
          curved: the general quadric
          Ax²+By²+Cz²+Dxy+Exz+Fyz+Gx+Hy+Iz+J = 0 (eq. 12)
    §3.3  the theoretical seam is the intersection of the   -> `seam_points`
          two fitted surfaces; the actual seam is the cloud
          points within a threshold of it
    §3.3  ordering: a graph walk finds the initial point,   -> `order_points`
          then bubble sort by distance from it (eq. 13)

Published number, the reproduction target: **position error < 1 mm on x, y, z** against a
taught path, on a flat and a curved fan blade (Fig. 6). Their curved blade is the reason
the method is here: it is the one entry in the comparison whose surface model is a
quadric, so a pipe standing on a plate (plane ∩ cylinder) and a pipe on a pipe
(cylinder ∩ cylinder) are inside its mechanism, where every plane-pair method is not.

Readings the paper leaves to the implementer — each one recorded, each one a knob
---------------------------------------------------------------------------------
1. **The flat/curved statistic.** §3.1 says a surface is flat when "the distribution
   probability of the normal vectors within the standard deviation range is over 90 %".
   Read literally that statistic is scale-free (for any unimodal distribution roughly
   two thirds of samples lie within one σ), and §4.2 applies it as an ANGLE test: "more
   than 30 % of the normal deviation angle is over 10°, thus curved". Implemented as the
   §4.2 reading — flat iff at least `flat_fraction` (0,9) of the normals deviate less than
   `flat_angle_deg` (10°) from the surface's mean normal — with the literal σ statistic
   computed and stored alongside (`surfaces[].within_sigma`) so the two can be compared.
2. **The plane fit as printed has no constraint.** Eq. 8-11 minimise Σ(ax+by+cz+d)² over
   (a, b, c, d) with no normalisation, whose minimiser is the zero vector; the intended
   estimator (their ref. 21, Shakarji & Srinivasan) is total least squares with ‖n‖ = 1.
   Implemented as TLS (centroid + smallest covariance eigenvector).
3. **The quadric fit** is eq. 12's algebraic least squares. Implemented as the homogeneous
   problem (smallest right singular vector of the monomial design matrix, ‖coef‖ = 1) on
   centred and scaled coordinates for conditioning — the paper does not say how the
   trivial solution is excluded or how the system is conditioned.
4. **"Within the threshold" of the theoretical seam** is unpublished. A point is near the
   intersection when it is near BOTH fitted surfaces (first-order distance |f|/‖∇f‖), so
   the band is the points of either surface within `seam_tol_mm` of the other's fit,
   and each is then projected onto the intersection by alternating orthogonal
   projections — the "ideal welding path" of §3.3 made explicit.
5. **The ordering is by Euclidean distance from the initial point** (eq. 13 finds the
   initial point; the path is "sorted in order from small to large" by distance). On a
   straight or gently curved open seam that is an arclength ordering; on a closed ring or
   a strongly curved seam it FOLDS — points on opposite sides of the ring at equal
   distance interleave. Implemented as published (`ordering="distance"`), with a
   nearest-neighbour chain from the same initial point as the corrected arm
   (`ordering="chain"`); the delta is reported, never silently applied.
6. **The initial point's walk starts "at a random point".** Kept as an arm
   (`start="random"`, seeded), but the default start is canonical — the point farthest
   from the seam's centroid — because the random start only changes the DIRECTION the
   seam is traversed in, and the harness requires a deterministic method's output to be
   a pure function of its input (two seeds otherwise differ at the 1e-6 of a resampled
   metric). On an open seam the walk's terminal point is one of the two ends either way.
   **The threshold must exceed the root gap.** A point qualifies only if it lies within
   `seam_tol_mm` of the other surface's fit AND its projection lands on that surface's
   patch; when the two parts are separated by more than the threshold (a T with a
   3 mm root gap at the 2,25 mm default) neither test passes and the joint is invisible
   to the method. The authors' blade sits in contact with the hub; on an ISO 5817
   no. 617 gap range this is a gap-conditioned coverage that the batch stratifies.
7. **Two parts, one surface each.** Their region growing yields the hub and the blade,
   and each is fitted as ONE surface because the camera sees one face per part. A plate
   in this dataset shows several faces, so the surface unit here is the FACE (the
   `surfaces` oracle at L0, grown regions at L1) and a seam candidate is a pair of
   surfaces from DIFFERENT parts at L0 (their two-part split), any adjacent pair at L1
   (no part membership without the oracle — the L0→L1 delta prices exactly that).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:                                    # pragma: no cover
    cKDTree = None

from .lit_regiongrow import local_pca, region_grow
from .radius_pca import voxel_downsample

VOXEL_MM = 1.5
K_NEIGHBORS = 20
FLAT_FRACTION = 0.90          # §3.1 / §4.2: "over 90 %"
FLAT_ANGLE_DEG = 10.0         # §4.2: "over 10°" is the curved verdict
MONOMIALS = ("x2", "y2", "z2", "xy", "xz", "yz", "x", "y", "z", "1")


# --------------------------------------------------------------------------------
# §2 pretreatment
# --------------------------------------------------------------------------------

def gaussian_smooth(pts: np.ndarray, radius_mm: float, sigma_mm: float | None = None,
                    chunk: int = 20_000) -> np.ndarray:
    """§2.1: each point replaced by the Gaussian-weighted mean of its neighbours."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0 or radius_mm <= 0:
        return pts
    sigma = float(sigma_mm) if sigma_mm else 0.5 * float(radius_mm)
    tree = cKDTree(pts)
    out = pts.copy()
    for start in range(0, len(pts), chunk):
        sl = slice(start, min(start + chunk, len(pts)))
        lists = tree.query_ball_point(pts[sl], float(radius_mm), workers=-1)
        for j, ids in enumerate(lists):
            if len(ids) < 2:
                continue
            nb = pts[ids]
            w = np.exp(-np.sum((nb - pts[start + j]) ** 2, axis=1) / (2 * sigma ** 2))
            out[start + j] = (w[:, None] * nb).sum(axis=0) / w.sum()
    return out


# --------------------------------------------------------------------------------
# §3.1 shape recognition
# --------------------------------------------------------------------------------

def normal_deviation_deg(normals: np.ndarray) -> np.ndarray:
    """Angle of every normal from the surface's mean normal, sign-folded (eigenvector
    signs are arbitrary), in degrees."""
    n = np.asarray(normals, dtype=float)
    if len(n) == 0:
        return np.zeros(0)
    ref = n[0]
    n = n * np.where(n @ ref < 0, -1.0, 1.0)[:, None]  # fold signs toward one hemisphere
    mu = n.mean(axis=0)
    mu /= max(np.linalg.norm(mu), 1e-12)
    c = np.clip(np.abs(n @ mu), -1.0, 1.0)
    return np.degrees(np.arccos(c))


def surface_kind(normals: np.ndarray, flat_fraction: float = FLAT_FRACTION,
                 flat_angle_deg: float = FLAT_ANGLE_DEG) -> dict[str, Any]:
    """§3.1's verdict on one surface, with both readings of the statistic (reading 1)."""
    dev = normal_deviation_deg(normals)
    if len(dev) == 0:
        return {"kind": "flat", "within_angle": 1.0, "within_sigma": 1.0, "sigma_deg": 0.0}
    within_angle = float((dev <= flat_angle_deg).mean())
    sigma = float(dev.std())
    within_sigma = float((dev <= sigma).mean()) if sigma > 0 else 1.0
    return {"kind": "flat" if within_angle >= flat_fraction else "curved",
            "within_angle": within_angle, "within_sigma": within_sigma, "sigma_deg": sigma,
            "mean_dev_deg": float(dev.mean())}


# --------------------------------------------------------------------------------
# §3.2 fitting
# --------------------------------------------------------------------------------

@dataclass
class ImplicitSurface:
    """`f(p) = 0` — a plane (`n·(p − c)`) or a quadric in centred, scaled coordinates."""

    kind: str                                  # "flat" | "curved"
    centre: np.ndarray
    scale: float
    normal: np.ndarray | None = None           # flat
    coef: np.ndarray | None = None             # curved: 10 monomial coefficients
    rms_residual_mm: float = 0.0

    def _local(self, p: np.ndarray) -> np.ndarray:
        return (np.atleast_2d(np.asarray(p, dtype=float)) - self.centre) / self.scale

    def f(self, p: np.ndarray) -> np.ndarray:
        if self.kind == "flat":
            return (np.atleast_2d(np.asarray(p, dtype=float)) - self.centre) @ self.normal
        q = self._local(p)
        return _design(q) @ self.coef

    def grad(self, p: np.ndarray) -> np.ndarray:
        if self.kind == "flat":
            return np.tile(self.normal, (len(np.atleast_2d(p)), 1))
        q = self._local(p)
        A, B, C, D, E, F, G, H, I, _ = self.coef
        x, y, z = q[:, 0], q[:, 1], q[:, 2]
        g = np.column_stack([2 * A * x + D * y + E * z + G,
                             2 * B * y + D * x + F * z + H,
                             2 * C * z + E * x + F * y + I])
        return g / self.scale                                  # chain rule of the scaling

    def distance(self, p: np.ndarray) -> np.ndarray:
        """First-order (Sampson) distance |f| / ‖∇f‖ — exact for a plane."""
        g = np.linalg.norm(self.grad(p), axis=1)
        return np.abs(self.f(p)) / np.maximum(g, 1e-12)

    def project(self, p: np.ndarray, iters: int = 3) -> np.ndarray:
        """Newton projection onto the surface (exact in one step for a plane)."""
        p = np.atleast_2d(np.asarray(p, dtype=float)).copy()
        for _ in range(1 if self.kind == "flat" else iters):
            g = self.grad(p)
            p -= (self.f(p) / np.maximum(np.sum(g * g, axis=1), 1e-12))[:, None] * g
        return p


def _design(q: np.ndarray) -> np.ndarray:
    x, y, z = q[:, 0], q[:, 1], q[:, 2]
    return np.column_stack([x * x, y * y, z * z, x * y, x * z, y * z, x, y, z,
                            np.ones(len(q))])


def fit_plane(pts: np.ndarray) -> ImplicitSurface:
    """Eq. 7-11 as their ref. 21 intends it: total least squares (reading 2)."""
    pts = np.asarray(pts, dtype=float)
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[-1]
    s = ImplicitSurface("flat", c, 1.0, normal=n)
    s.rms_residual_mm = float(np.sqrt(np.mean(s.f(pts) ** 2)))
    return s


def fit_quadric(pts: np.ndarray) -> ImplicitSurface:
    """Eq. 12 as a homogeneous least-squares problem on centred, scaled points (reading 3)."""
    pts = np.asarray(pts, dtype=float)
    c = pts.mean(axis=0)
    scale = float(np.sqrt(np.mean(np.sum((pts - c) ** 2, axis=1)))) or 1.0
    q = (pts - c) / scale
    _, _, vt = np.linalg.svd(_design(q), full_matrices=False)
    s = ImplicitSurface("curved", c, scale, coef=vt[-1])
    s.rms_residual_mm = float(np.sqrt(np.mean(s.distance(pts) ** 2)))
    return s


def fit_surface(pts: np.ndarray, kind: str) -> ImplicitSurface:
    return fit_plane(pts) if kind == "flat" else fit_quadric(pts)


# --------------------------------------------------------------------------------
# §3.3 the seam
# --------------------------------------------------------------------------------

def seam_points(pts_a: np.ndarray, sa: ImplicitSurface, pts_b: np.ndarray,
                sb: ImplicitSurface, tol_mm: float, iters: int = 12,
                patch_tol_mm: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Points of either surface within `tol_mm` of the OTHER's fit — and within
    `patch_tol_mm` of the other's actual POINTS — projected onto the intersection by
    alternating projections (reading 4). Returns `(raw, projected)`.

    The patch test is what makes the fitted surface finite: a plane fit is infinite,
    and a point of a far face that happens to lie near the *extended* plane of another
    surface is not near any seam. In the paper's two-part setting the surfaces are the
    parts themselves, so "near the intersection line" is implicitly "where both parts
    are"; here it must be said: the FOOT of the point's projection onto the other fit
    must fall within `patch_tol_mm` (default `tol_mm`) of the other surface's points.
    """
    patch = float(patch_tol_mm) if patch_tol_mm else float(tol_mm)
    near_a = near_b = np.zeros((0, 3))
    if len(pts_a) and len(pts_b):
        ta, tb = cKDTree(pts_a), cKDTree(pts_b)
        # near the other's FIT, and the foot of that projection near the other's POINTS
        ma = sb.distance(pts_a) <= tol_mm
        if ma.any():
            ma[ma] = tb.query(sb.project(pts_a[ma]), k=1, workers=-1)[0] <= patch
        mb = sa.distance(pts_b) <= tol_mm
        if mb.any():
            mb[mb] = ta.query(sa.project(pts_b[mb]), k=1, workers=-1)[0] <= patch
        near_a, near_b = pts_a[ma], pts_b[mb]
    raw = np.vstack([near_a, near_b])
    if len(raw) == 0:
        return raw, raw
    p = raw.copy()
    for _ in range(iters):
        p = sb.project(sa.project(p))
    ok = np.isfinite(p).all(axis=1) & (np.linalg.norm(p - raw, axis=1) <= 3.0 * tol_mm)
    return raw, np.where(ok[:, None], p, raw)


def _gradient_angle_deg(sa: ImplicitSurface, sb: ImplicitSurface, p: np.ndarray) -> float:
    ga, gb = sa.grad(p), sb.grad(p)
    ga /= np.maximum(np.linalg.norm(ga, axis=1, keepdims=True), 1e-12)
    gb /= np.maximum(np.linalg.norm(gb, axis=1, keepdims=True), 1e-12)
    return float(np.degrees(np.arccos(np.clip(np.median(np.abs(np.sum(ga * gb, axis=1))), 0, 1))))


def order_points(pts: np.ndarray, ordering: str = "distance", seed: int = 0,
                 start: str = "farthest") -> np.ndarray:
    """§3.3's path ordering (readings 5 and 6).

    The graph walk of eq. 13: from a source point, repeatedly move to the nearest
    unvisited point; the last point reached is the initial point of the weld. Then, as
    published, sort every point by its distance from that initial point (`"distance"`);
    or, the corrected arm, chain nearest neighbours from it (`"chain"`).

    `start`: the paper starts the walk "at a random point" (`"random"`, drawn from
    `seed`). Nothing downstream depends on which end the walk reaches, but the DIRECTION
    of the output does, and two seeds then give the same seam traversed opposite ways -
    identical up to the 1e-6 of a resampled metric, which is not the exact zero the
    harness demands of a deterministic method. Default `"farthest"`: the walk starts at
    the point farthest from the centroid, a canonical choice that makes the output a
    pure function of the points (deviation recorded; the random start is kept as an arm).
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    if n < 3:
        return pts
    tree = cKDTree(pts)
    visited = np.zeros(n, dtype=bool)
    if start == "random":
        cur = int(np.random.default_rng(seed).integers(n))
    else:
        cur = int(np.argmax(np.linalg.norm(pts - pts.mean(axis=0), axis=1)))
    visited[cur] = True
    for _ in range(n - 1):
        k = min(n, 32)
        while True:
            d, idx = tree.query(pts[cur], k=k)
            cand = [int(j) for j in np.atleast_1d(idx) if not visited[j]]
            if cand or k >= n:
                break
            k = min(n, k * 4)
        if not cand:
            break
        cur = cand[0]
        visited[cur] = True
    start = cur                                          # the last point reached
    if ordering == "distance":
        return pts[np.argsort(np.linalg.norm(pts - pts[start], axis=1), kind="stable")]
    # chain: nearest unvisited neighbour from the initial point
    visited[:] = False
    order = [start]
    visited[start] = True
    cur = start
    for _ in range(n - 1):
        k = min(n, 32)
        while True:
            d, idx = tree.query(pts[cur], k=k)
            cand = [int(j) for j in np.atleast_1d(idx) if not visited[j]]
            if cand or k >= n:
                break
            k = min(n, k * 4)
        if not cand:
            break
        cur = cand[0]
        visited[cur] = True
        order.append(cur)
    return pts[order]


# --------------------------------------------------------------------------------
# the result, and the pipeline
# --------------------------------------------------------------------------------

@dataclass
class QuadricResult:
    """What `lit-quadric` returned, and the verdicts that produced it."""

    seams: list[np.ndarray]                    #: ordered seam polylines, one per surface pair
    clusters: list[np.ndarray]                 #: the raw band points behind each seam
    surfaces: list[dict[str, Any]]             #: per surface: label, kind, statistics, fit
    #: every surface pair considered: {i, j, status, n_band, angle_deg}; status one of
    #: `seam`, `same_part`, `no_band`, `coplanar`, `tangent`, `too_few`, `too_short` — coverage as data
    pairs: list[dict[str, Any]] = field(default_factory=list)
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    labels: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=int))
    params: dict[str, Any] = field(default_factory=dict)
    used_segmentation_oracle: bool = False
    note: str = ""

    @property
    def n_seams(self) -> int:
        return len(self.seams)

    @property
    def polylines(self) -> list[np.ndarray]:
        return self.seams


def _labels_to(src: np.ndarray, src_labels: np.ndarray, dst: np.ndarray) -> np.ndarray:
    _, idx = cKDTree(src).query(dst, k=1, workers=-1)
    return np.asarray(src_labels)[idx]


def detect(pts: np.ndarray,
           voxel_mm: float | None = VOXEL_MM,
           gaussian_radius_mm: float | None = None,
           k: int = K_NEIGHBORS,
           flat_fraction: float = FLAT_FRACTION,
           flat_angle_deg: float = FLAT_ANGLE_DEG,
           seam_tol_mm: float | None = None,
           min_surface_pts: int = 40,
           min_seam_pts: int = 8,
           min_seam_length_mm: float = 10.0,
           coplanar_deg: float = 10.0,
           ordering: str = "distance",
           seed: int = 0,
           region_labels: np.ndarray | None = None,
           part_labels: np.ndarray | None = None,
           smoothness_deg: float = 20.0,
           curvature_thresh: float = 0.10) -> QuadricResult:
    """Run `lit-quadric` end to end. Lengths in **millimetres**.

    Args:
        voxel_mm: §2.2's voxel-centroid grid. Unpublished; 1,5 mm like the other methods.
        gaussian_radius_mm: §2.1's denoising radius. Unpublished, and aimed at scanner
            outliers this dataset's tier-1 clouds do not contain, so OFF by default;
            set it (e.g. `2 × voxel`) to reproduce the paper's pretreatment literally.
        k: neighbours for the PCA normals of eq. 3. Unpublished; PCL's default.
        flat_fraction, flat_angle_deg: §3.1's verdict as §4.2 applies it (reading 1).
        seam_tol_mm: §3.3's "within the threshold" (reading 4). Defaults to `1,5 × voxel`.
        min_seam_length_mm: a seam shorter than this is a sliver where one surface's
            extended fit clips the corner of another; unpublished, 10 mm like the
            other methods' floors.
        coplanar_deg: two surfaces closer than this in angle at the band have no usable
            intersection line — a butt centreline is outside the mechanism, and so is a
            standing plate's contact face against the plate it rests on; the pair census
            says so. 10° because no weldable dihedral is that shallow (D4's degenerate
            verdict), while a tilted contact face is often 5-9° off parallel.
        ordering: `"distance"` as published, `"chain"` the corrected arm (reading 5).
        region_labels: per-point SURFACE labels supplied instead of grown — the L0 arm
            (`surface_labels_oracle(face_id)`), reading 7. Negative = unassigned.
        part_labels: per-point PART membership, supplied with the L0 oracle so that a
            seam candidate is a surface pair from different parts (their two-part split).
            Without it every adjacent pair is a candidate, a plate's own rim included.
        smoothness_deg, curvature_thresh: the L1 region growing (their §2.3 cites a
            generic region-growing method and gives no thresholds). Wei et al.'s Alg. 1
            with its published 20° normal jump and a curvature seed threshold of 0,1 —
            the looser value is forced by THIN PLATES: a k-NN neighbourhood on a 1,5 mm
            grid reaches across a 1-3 mm plate to its far face, so the surface-variation
            curvature of a perfectly flat face reads 0,05-0,1 and a 0,03-0,05 threshold
            starves the growth (measured: 9 % of a T scene assigned at 0,05, 100 % at 0,1).
    """
    pts = np.asarray(pts, dtype=float)
    raw_pts = pts
    if gaussian_radius_mm:
        pts = gaussian_smooth(pts, gaussian_radius_mm)
    if voxel_mm:
        pts = voxel_downsample(pts, float(voxel_mm))
    tol = float(seam_tol_mm) if seam_tol_mm else 1.5 * float(voxel_mm or 1.0)
    params: dict[str, Any] = dict(voxel_mm=voxel_mm, gaussian_radius_mm=gaussian_radius_mm,
                                  k=k, flat_fraction=flat_fraction,
                                  flat_angle_deg=flat_angle_deg, seam_tol_mm=tol,
                                  ordering=ordering, seed=seed)
    empty = QuadricResult([], [], [], points=pts, params=params)
    if len(pts) < min_surface_pts:
        empty.note = "too few points"
        return empty

    normals, curv = local_pca(pts, k=k)
    if region_labels is not None:
        labels = _labels_to(raw_pts, np.asarray(region_labels), pts)
        parts = (_labels_to(raw_pts, np.asarray(part_labels), pts)
                 if part_labels is not None else None)
        used_oracle = True
    else:
        labels, _ = region_grow(pts, normals, curv, k=k, smoothness_deg=smoothness_deg,
                                curvature_thresh=curvature_thresh,
                                min_region_pts=min_surface_pts)
        parts = None
        used_oracle = False

    surfaces: list[dict[str, Any]] = []
    fits: dict[int, ImplicitSurface] = {}
    members: dict[int, np.ndarray] = {}
    for lab in np.unique(labels):
        if lab < 0:
            continue
        m = np.where(labels == lab)[0]
        if len(m) < min_surface_pts:
            continue
        # §3.1 computes the normals OF THE WELDING SURFACE, after segmentation: a
        # neighbourhood drawn from the whole cloud crosses the crease and tilts the
        # normals of every point within k neighbours of it, which votes a small
        # visible face "curved". Per-surface PCA is the paper's order of operations.
        n_surf, _ = local_pca(pts[m], k=min(k, len(m)))
        verdict = surface_kind(n_surf, flat_fraction, flat_angle_deg)
        surf = fit_surface(pts[m], verdict["kind"])
        fits[int(lab)] = surf
        members[int(lab)] = m
        surfaces.append({"label": int(lab), "n": int(len(m)), **verdict,
                         "rms_residual_mm": surf.rms_residual_mm,
                         "part": int(np.bincount(parts[m]).argmax()) if parts is not None else None})

    seams: list[np.ndarray] = []
    clusters: list[np.ndarray] = []
    pairs: list[dict[str, Any]] = []
    labs = sorted(fits)
    part_of = {s["label"]: s["part"] for s in surfaces}
    for a_i, la in enumerate(labs):
        for lb in labs[a_i + 1:]:
            rec: dict[str, Any] = {"i": la, "j": lb, "n_band": 0, "angle_deg": float("nan")}
            if parts is not None and part_of[la] == part_of[lb]:
                rec["status"] = "same_part"; pairs.append(rec); continue
            sa, sb = fits[la], fits[lb]
            pa, pb = pts[members[la]], pts[members[lb]]
            raw, proj = seam_points(pa, sa, pb, sb, tol)
            rec["n_band"] = int(len(raw))
            if len(raw) == 0:
                rec["status"] = "no_band"; pairs.append(rec); continue
            ang = _gradient_angle_deg(sa, sb, proj)
            rec["angle_deg"] = ang
            if sa.kind == "flat" and sb.kind == "flat" and ang < coplanar_deg:
                rec["status"] = "coplanar"; pairs.append(rec); continue
            if ang < coplanar_deg:
                rec["status"] = "tangent"; pairs.append(rec); continue
            if len(proj) < min_seam_pts:
                rec["status"] = "too_few"; pairs.append(rec); continue
            poly = order_points(proj, ordering=ordering, seed=seed)
            span = float(np.linalg.norm(proj - proj.mean(axis=0), axis=1).max()) * 2.0
            if span < min_seam_length_mm:                  # extent, not path length: the
                                                            # published ordering inflates paths
                rec["status"] = "too_short"; pairs.append(rec); continue
            rec["status"] = "seam"; pairs.append(rec)
            seams.append(poly)
            clusters.append(raw)

    return QuadricResult(seams, clusters, surfaces, pairs=pairs, points=pts, labels=labels,
                         params=params, used_segmentation_oracle=used_oracle,
                         note=(f"{len(seams)} seam(s) from {len(fits)} surfaces; "
                               f"pair census {dict(zip(*np.unique([p['status'] for p in pairs], return_counts=True))) if pairs else {}}"))
