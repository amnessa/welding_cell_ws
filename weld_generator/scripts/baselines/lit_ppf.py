"""`lit-ppf` — point-pair-feature coplanarity planes, orthogonal-pair Hough voting.

A faithful reimplementation of the point-cloud half of:

    Shengbo Wang, Zengxu Li, Guodong Chen, Yaobin Yue.
    "Weld seam object detection system based on the fusion of 2D images and 3D point
    clouds using interpretable neural networks."
    *Scientific Reports* 14 (2024) 21137.  https://doi.org/10.1038/s41598-024-71989-w

Their pipeline, and which stages this dataset supplies:

    §"Interpretable image-based…"  Faster R-CNN (ResNet50) finds a 2D box around each
                                   weld; the cloud is cropped to it   -> the `band` ORACLE
    §"Plane extraction"            25 mm distance sampling; per-sample planes grown by
                                   PPF coplanarity; leftover points assigned by
                                   point-to-plane distance             -> `ppf_planes`
    §"Improved orthogonal plane…"  OPP test from the descriptor's angle component; local
                                   Hough voting in (theta, rho), eqs. 21-23
                                                                       -> `opp_vote`
    experiments                    feature points = distance component under a threshold;
                                   DBSCAN refinement; the two FARTHEST feature points are
                                   the corner pair                     -> `detect`

Published numbers, the reproduction target: corner-distance error **2,17% average / 3,84 mm
max** on T welds and **2,4% / 6,82 mm** on V welds, against a 100 mm workpiece edge. Note
what the metric is: the distance between the two extracted corners against the workpiece
size — a **length** check, not a lateral-accuracy one. `metrics.path_error_mm` carries
`length_error_mm` for exactly this comparison.

Three things about this paper that the reimplementation has to be explicit about
--------------------------------------------------------------------------------
1. **The paper contradicts itself about RANSAC, and this module implements the prose.**
   The method section proposes PPF *instead of* RANSAC ("extraction speed is slow, and the
   distance threshold setting is more inconvenient"); the implementation section's
   Algorithm 1 then says planes were "extracted … using RANSAC". The PPF path is the
   stated contribution and the reason this method is in the seven, so that is what is
   implemented — and the contradiction is recorded rather than smoothed over.
2. **As published, the pipeline is DETERMINISTIC.** Distance-grid sampling, Hough voting,
   DBSCAN, farthest-pair corners: no stage draws a random number. `dataset_plan.md`
   grouped `lit-ppf` with `lit-ransac` as "randomised" on the strength of its
   RANSAC-alternative framing — that assumption is corrected by reading the paper, and the
   method registers `randomised=False`. Its zero seed-spread is *measured* by the harness
   like the other deterministic three.
3. **It consumes normals** — the first implemented method that does. The paper estimates
   them from the scan (PCL), so estimation is the faithful default here
   (`normals="estimate"`), and passing the generator's exact normals is the L-ladder's
   normal-oracle arm (`normals="exact"` + the `normals_xyz` argument). This is the L2 rung
   the plan said would become meaningful at `lit-ppf`.

The published constants: sampling interval **25 mm**; feature-distance threshold
**0,1 mm**. The second is stated for a 50 um-accuracy Photoneo scan; on clouds sampled at
~1 mm spacing nothing survives a 0,1 mm gate, so the default here scales with spacing and
`feature_tol_mm=0.1` reproduces the literal paper. Every other threshold is unpublished
and every default below says so.

The coverage prediction this method carries
-------------------------------------------
The mechanism is built on **orthogonal plane pairs** — the OPP test throws away any pair
that is not near 90 deg. A butt joint's two top faces are coplanar and an edge joint's
faces are parallel, so the prediction of `dataset_plan.md` §4 is that `lit-ppf` cannot
express them — with the same refinement `lit-ransac` measured: a butt joint's **root-gap
walls** are orthogonal to the faces, so a nonzero gap smuggles the seam back in.

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

from .lit_regiongrow import local_pca
from .radius_pca import dbscan_components, mean_spacing_mm, voxel_downsample

SAMPLE_INTERVAL_MM = 25.0     # published: "the sampling point interval is set to 25 mm"
FEATURE_TOL_PAPER_MM = 0.1    # published, for a 50 um scanner; see the module docstring


# --------------------------------------------------------------------------------
# the descriptor
# --------------------------------------------------------------------------------

def ppf(p1, n1, p2, n2) -> tuple[float, float, float, float]:
    """The classical four-component point-pair feature of the paper's ref. 29 (Drost et al.).

    `F = (|d|, angle(n1, d), angle(n2, d), angle(n1, n2))` with `d = p2 - p1`. The paper
    uses two readings of it: **coplanarity** (components 2 and 3 at 90 deg, component 4 at
    0 or 180 deg) grows the planes, and the **angle component** near 90 deg marks an
    orthogonal pair, whose eq. 23 replaces component 1 by the signed `rho = n2 . d`.
    """
    p1, n1 = np.asarray(p1, float), np.asarray(n1, float)
    p2, n2 = np.asarray(p2, float), np.asarray(n2, float)
    d = p2 - p1
    nd = float(np.linalg.norm(d))
    if nd < 1e-12:
        return 0.0, 0.0, 0.0, float(np.degrees(np.arccos(np.clip(abs(n1 @ n2), -1, 1))))
    u = d / nd
    a = lambda x: float(np.degrees(np.arccos(np.clip(abs(x), -1.0, 1.0))))  # noqa: E731
    return nd, a(n1 @ u), a(n2 @ u), a(n1 @ n2)


# --------------------------------------------------------------------------------
# stage 1 — plane extraction by PPF coplanarity
# --------------------------------------------------------------------------------

@dataclass
class PPFPlane:
    """One extracted plane: seeded at a sample point, grown by coplanarity."""

    seed: np.ndarray                      #: the sample point `b`
    normal: np.ndarray                    #: its (estimated) normal, unit, sign arbitrary
    inliers: np.ndarray                   #: indices into the cloud handed to `ppf_planes`
    centroid: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @property
    def n_inliers(self) -> int:
        return len(self.inliers)


def ppf_planes(pts: np.ndarray, normals: np.ndarray,
               sample_interval_mm: float = SAMPLE_INTERVAL_MM,
               coplanar_tol_mm: float | None = None, angle_tol_deg: float = 15.0,
               assign_order: str = "asc", min_plane_pts: int = 30) -> list[PPFPlane]:
    """The Fig. 5 pipeline: sample at 25 mm, grow a plane per sample, assign the rest.

    A point `c` is coplanar with sample `(b, n_b)` when every PPF reading agrees: the
    offset `c - b` is perpendicular to *both* normals (components 2 and 3) and the normals
    are parallel (component 4). Vectorised over the whole cloud per sample — the pairwise
    formulation is the descriptor's story, not a required implementation.

    Two faithful oddities, kept and flagged rather than fixed:

    * The paper orders the plane set *"from least to most according to the number of point
      clouds contained"* before assigning the remaining points first-match. Ascending order
      lets the smallest plane claim contested points — the reverse of the usual convention.
      `assign_order="desc"` is the conventional reading; the default is the printed one.
    * Duplicate planes are inevitable (two samples on one face grow the same plane) and the
      paper never mentions merging. Near-duplicates (parallel within `angle_tol_deg`,
      separated under `coplanar_tol_mm`) are merged here, because without it every
      downstream pair test runs on the same plane against itself.

    `coplanar_tol_mm` defaults to 2x the estimated point spacing; the paper publishes no
    value for it.
    """
    pts = np.asarray(pts, dtype=float)
    normals = np.asarray(normals, dtype=float)
    if len(pts) < min_plane_pts:
        return []
    if coplanar_tol_mm is None:
        # Measured nearest-neighbour spacing, not a bounding-box estimate: on thin
        # geometry cbrt(volume/n) overestimates spacing badly enough that the duplicate
        # merge swallowed two parallel sheets 8 mm apart.
        coplanar_tol_mm = max(2.0 * mean_spacing_mm(pts), 0.5)
    cos_tol = np.cos(np.radians(float(angle_tol_deg)))

    samples = voxel_downsample(pts, float(sample_interval_mm))
    _, sidx = cKDTree(pts).query(samples, k=1, workers=-1)
    seeds = list(np.unique(np.atleast_1d(sidx)))

    def grow(i: int) -> PPFPlane | None:
        b, nb = pts[i], normals[i]
        if np.linalg.norm(nb) < 0.5:
            return None
        off = pts - b
        perp_b = np.abs(off @ nb) <= coplanar_tol_mm            # component 2
        par = np.abs(normals @ nb) >= cos_tol                   # component 4
        # component 3, |n_c . d| small, RELATIVE to |d|: for points beyond a spacing of
        # the seed this is the same plane test from the other end; inside it is vacuous.
        nd = np.linalg.norm(off, axis=1)
        perp_c = np.abs(np.einsum("ij,ij->i", off, normals)) <= \
            np.maximum(coplanar_tol_mm, 0.05 * nd)
        m = perp_b & par & perp_c
        if int(m.sum()) < min_plane_pts:
            return None
        sub = pts[m]
        c = sub - sub.mean(axis=0)
        # Refit the normal from the grown inliers. The seed POINT's estimated normal is
        # one noisy sample; used as the plane normal it tilts the intersection line by
        # tan(tilt) x half the plate - measured, enough to push every feature point
        # outside the tolerance on an ideal fold under estimated normals.
        _, v = np.linalg.eigh(c.T @ c)
        n_fit = v[:, 0] if abs(v[:, 0] @ nb) > 0.5 else nb
        return PPFPlane(seed=b, normal=n_fit * np.sign(n_fit @ nb),
                        inliers=np.flatnonzero(m), centroid=sub.mean(axis=0))

    planes: list[PPFPlane] = []
    claimed = np.zeros(len(pts), dtype=bool)
    for i in seeds:
        pl = grow(int(i))
        if pl is not None:
            planes.append(pl)
            claimed[pl.inliers] = True

    # Deviation, and the reason it exists: distance sampling at 25 mm can never seed a
    # surface that lies WITHIN the interval of an already-sampled one - an edge joint's
    # second sheet sits 8 mm above the first and no 25 mm-spaced sample ever lands on it.
    # Without re-seeding, the coverage result for coplanar joints would be an artifact of
    # the sampling stage; with it, the failure lands where the mechanism says it must -
    # at the orthogonality gate. Re-seeds are deterministic (lowest unclaimed index).
    for _ in range(8):
        rest = np.flatnonzero(~claimed)
        if len(rest) < min_plane_pts:
            break
        pl = grow(int(rest[0]))
        if pl is None:
            claimed[rest[0]] = True                    # dead seed; move on
            continue
        planes.append(pl)
        claimed[pl.inliers] = True

    # merge near-duplicates (see the docstring)
    merged: list[PPFPlane] = []
    for pl in sorted(planes, key=lambda p: -p.n_inliers):
        dup = False
        for q in merged:
            if abs(float(pl.normal @ q.normal)) >= cos_tol and \
                    abs(float((pl.centroid - q.centroid) @ q.normal)) <= 2 * coplanar_tol_mm:
                dup = True
                break
        if not dup:
            merged.append(pl)

    # assignment pass: leftover points join the first plane within the distance threshold,
    # planes visited in the printed (ascending) order
    order = sorted(merged, key=lambda p: p.n_inliers,
                   reverse=(assign_order == "desc"))
    claimed[:] = False
    for pl in order:
        claimed[pl.inliers] = True
    rest = np.flatnonzero(~claimed)
    if len(rest):
        extra: dict[int, list[int]] = {}
        taken = np.zeros(len(rest), dtype=bool)
        for k, pl in enumerate(order):
            # Distance only, exactly as printed - no normal condition. This is not an
            # oversight to fix: the fold's own points carry ~45-deg blended normals and
            # fail any normal gate, and it is the assignment pass that puts them back on
            # the planes. Without them the feature stage finds an empty corridor around
            # the very line it just voted for.
            d = np.abs((pts[rest] - pl.centroid) @ pl.normal)
            near = ~taken & (d <= coplanar_tol_mm)
            if near.any():
                extra[k] = rest[near].tolist()
                taken |= near
        for k, ids in extra.items():
            order[k].inliers = np.union1d(order[k].inliers, ids)
    for pl in order:
        pl.centroid = pts[pl.inliers].mean(axis=0)
    return order


# --------------------------------------------------------------------------------
# stage 2 — OPP detection and the (theta, rho) vote, eqs. 21-23
# --------------------------------------------------------------------------------

def rotation_to_z(n: np.ndarray) -> np.ndarray:
    """Eq. 21's `R_z`: the rotation taking the reference normal onto the z axis."""
    n = np.asarray(n, dtype=float)
    n = n / max(np.linalg.norm(n), 1e-12)
    phi = np.arctan2(n[0], n[2])
    ry = np.array([[np.cos(phi), 0, -np.sin(phi)], [0, 1, 0],
                   [np.sin(phi), 0, np.cos(phi)]])
    r = ry @ n
    omega = np.arctan2(r[1], r[2])
    rx = np.array([[1, 0, 0], [0, np.cos(omega), -np.sin(omega)],
                   [0, np.sin(omega), np.cos(omega)]])
    return rx @ ry


def opp_vote(pts: np.ndarray, normals: np.ndarray, ref: PPFPlane, other: PPFPlane,
             ortho_tol_deg: float = 15.0, n_pairs: int = 400,
             theta_bins: int = 36, rho_bin_mm: float = 2.0
             ) -> tuple[int, float, float] | None:
    """The local Hough vote for one candidate pair. `(votes, theta, rho)` or `None`.

    Directed point pairs are drawn across the two planes; a pair is OPP when the
    descriptor's angle component sits within `ortho_tol_deg` of 90 deg. Each OPP pair
    votes in the space of eqs. 22-23 — `theta`, the partner normal's azimuth once the
    reference normal is rotated onto z, and `rho = n2 . (x1 - x2)`.

    **The vote is local, and the first implementation here got that wrong.** For a fixed
    reference point `x1`, `rho = n2 . x1 - n2 . x2` is constant over every partner on a
    true plane (the second term is the plane's offset) — so per reference point the votes
    land in ONE bin by construction, and *across* reference points `rho` varies with `x1`'s
    distance from the partner plane, also by construction. A global (theta, rho)
    accumulator therefore scatters a perfect fold over dozens of rho bins, which is
    exactly what it did. The statistic that is global for a genuine orthogonal *plane*
    pair is `theta` — one intersection direction — so the peak is taken over theta, and
    a curved or accidental pairing smears it. `rho`'s per-reference consistency is what
    eq. 23 contributes to *locating* the line; with the planes already parameterised the
    line is recovered in closed form, which coincides with the voted location for true
    planes. Everything here is deterministic: pairs are strided, not drawn.
    """
    cosd = abs(float(np.clip(ref.normal @ other.normal, -1, 1)))
    if cosd > np.cos(np.radians(90.0 - ortho_tol_deg)):
        return None                                    # not orthogonal: no vote at all
    Rz = rotation_to_z(ref.normal)

    a = ref.inliers[:: max(1, len(ref.inliers) // int(np.sqrt(n_pairs)))]
    b = other.inliers[:: max(1, len(other.inliers) // int(np.sqrt(n_pairs)))]
    if len(a) == 0 or len(b) == 0:
        return None
    x1, x2 = pts[a], pts[b]
    n2 = normals[b]
    # eq. 22, one theta per partner point (its normal is constant over a plane up to noise)
    rn = (Rz @ n2.T).T
    theta = np.arctan2(rn[:, 1], rn[:, 0])
    # eq. 23, rho for every (x1, x2) pair
    rho = np.einsum("bj,abj->ab", n2, x1[:, None, :] - x2[None, :, :])

    th = np.broadcast_to(theta[None, :], rho.shape).ravel()
    ti = ((th + np.pi) / (2 * np.pi) * theta_bins).astype(int) % theta_bins
    counts = np.bincount(ti, minlength=theta_bins)
    peak = int(counts.argmax())
    votes = int(counts[peak])
    sel = ti == peak
    return votes, float(np.median(th[sel])), float(np.median(rho.ravel()[sel]))


# --------------------------------------------------------------------------------
# result and driver
# --------------------------------------------------------------------------------

@dataclass
class PPFResult:
    """What `lit-ppf` returned, and enough context to know what it means."""

    seams: list[np.ndarray]               #: corner-pair polylines, one per accepted OPP
    clusters: list[np.ndarray]            #: the feature points behind each seam
    planes: list[PPFPlane]
    #: every plane pair considered: {i, j, angle_deg, votes, status} with status one of
    #: `seam`, `not_orthogonal`, `low_votes`, `no_features` - the coverage claim as data,
    #: same discipline as `LitRansacResult.pairs`
    pairs: list[dict[str, Any]] = field(default_factory=list)
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    normals: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    params: dict[str, Any] = field(default_factory=dict)
    used_segmentation_oracle: bool = False
    used_exact_normals: bool = False
    note: str = ""

    @property
    def polylines(self) -> list[np.ndarray]:
        return self.seams

    @property
    def n_seams(self) -> int:
        return len(self.seams)


def detect(pts: np.ndarray,
           normals: str = "estimate",
           normals_xyz: np.ndarray | None = None,
           voxel_mm: float | None = 1.5,
           normal_k: int = 20,
           sample_interval_mm: float = SAMPLE_INTERVAL_MM,
           coplanar_tol_mm: float | None = None,
           angle_tol_deg: float = 15.0,
           assign_order: str = "asc",
           min_plane_pts: int = 30,
           ortho_tol_deg: float = 15.0,
           min_votes: int = 30,
           feature_tol_mm: float | None = None,
           dbscan_eps_mm: float | None = None,
           dbscan_min_samples: int = 6,
           min_feature_pts: int = 10,
           min_seam_length_mm: float = 5.0,
           segmentation_mask: np.ndarray | None = None) -> PPFResult:
    """Run `lit-ppf` end to end. Lengths in **millimetres**.

    Args:
        normals: `"estimate"` (the paper: normals from the scan via local PCA — the L2
            condition) or `"exact"` with `normals_xyz` (the generator's analytic normals —
            the normal-oracle rung of the ladder). The delta between the two arms is the
            price of normal estimation, a number no paper in the seven reports.
        sample_interval_mm: published, 25 mm.
        feature_tol_mm: the experiment section's published 0,1 mm — for a 50 um scanner.
            Default scales as 1,5x the point spacing instead, or nothing survives on a
            ~1 mm-spacing cloud; pass 0.1 to reproduce the paper literally.
        min_votes: acceptance threshold on the Hough peak. Unpublished; this is the knob
            that separates a real orthogonal pair from an accidental one.
        segmentation_mask: the Faster R-CNN weld-box crop, supplied as an **oracle** via
            `seam_region_oracle` (a learned 2D weld-region detector, the same shape of
            stage as Yi et al.'s PointNet++). Withhold for the L1 arm.
    """
    pts = np.asarray(pts, dtype=float)
    used_oracle = segmentation_mask is not None
    if used_oracle:
        mask = np.asarray(segmentation_mask, dtype=bool)
        pts_in = pts[mask]
        nx_in = None if normals_xyz is None else np.asarray(normals_xyz)[mask]
    else:
        pts_in, nx_in = pts, normals_xyz

    P = voxel_downsample(pts_in, float(voxel_mm)) if voxel_mm else pts_in
    spacing = mean_spacing_mm(P) if len(P) > 1 else 1.0
    feature_tol_mm = float(feature_tol_mm) if feature_tol_mm else max(1.5 * spacing, 1.0)
    dbscan_eps_mm = float(dbscan_eps_mm) if dbscan_eps_mm else 3.0 * feature_tol_mm

    params = dict(normals=normals, voxel_mm=voxel_mm, normal_k=normal_k,
                  sample_interval_mm=sample_interval_mm, angle_tol_deg=angle_tol_deg,
                  assign_order=assign_order, ortho_tol_deg=ortho_tol_deg,
                  min_votes=min_votes, feature_tol_mm=feature_tol_mm,
                  dbscan_eps_mm=dbscan_eps_mm, n_input=len(P))
    empty = np.zeros((0, 3))

    if len(P) < max(min_plane_pts, normal_k):
        return PPFResult([], [], [], [], P, empty, params, used_oracle, False,
                         "cloud too small")

    if normals == "exact":
        if nx_in is None:
            raise ValueError('normals="exact" needs normals_xyz')
        _, idx = cKDTree(pts_in).query(P, k=1, workers=-1)
        N = np.asarray(nx_in, dtype=float)[idx]
        used_exact = True
    elif normals == "estimate":
        N, _ = local_pca(P, k=normal_k)
        used_exact = False
    else:
        raise ValueError(f'normals must be "estimate" or "exact", got {normals!r}')

    planes = ppf_planes(P, N, sample_interval_mm, coplanar_tol_mm, angle_tol_deg,
                        assign_order, min_plane_pts)
    if len(planes) < 2:
        return PPFResult([], [], planes, [], P, N, params, used_oracle, used_exact,
                         f"{len(planes)} plane(s); the OPP stage needs a pair")

    seams, clusters, pair_log = [], [], []
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            a, b = planes[i], planes[j]
            ang = float(np.degrees(np.arccos(np.clip(abs(a.normal @ b.normal), -1, 1))))
            vote = opp_vote(P, N, a, b, ortho_tol_deg)
            if vote is None:
                pair_log.append({"i": i, "j": j, "angle_deg": ang, "votes": 0,
                                 "status": "not_orthogonal"})
                continue
            votes, _, _ = vote
            if votes < min_votes:
                pair_log.append({"i": i, "j": j, "angle_deg": ang, "votes": votes,
                                 "status": "low_votes"})
                continue

            # the voted pair's intersection line, then the experiment section's rule:
            # feature points are those whose distance component is under the threshold
            d = np.cross(a.normal, b.normal)
            d /= max(np.linalg.norm(d), 1e-12)
            M = np.stack([a.normal, b.normal, d])
            rhs = np.array([a.normal @ a.centroid, b.normal @ b.centroid,
                            d @ (a.centroid + b.centroid) / 2.0])
            p0 = np.linalg.solve(M, rhs)
            # "The point pair features on the two groups of planes ... whose distance is
            # less than the threshold are feature points." Membership of the two planes is
            # taken by the paper's own criterion - point-to-plane distance - rather than by
            # this implementation's competitive inlier lists: with first-match assignment a
            # third plane can claim the crease strip outright, and the corridor around the
            # very line the pair just voted for comes back empty.
            on_a = np.abs((P - a.centroid) @ a.normal) <= 2.0 * feature_tol_mm
            on_b = np.abs((P - b.centroid) @ b.normal) <= 2.0 * feature_tol_mm
            dist = np.linalg.norm(np.cross(P - p0, d), axis=1)
            feats = P[on_a & on_b & (dist <= feature_tol_mm)]
            if len(feats) < min_feature_pts:
                pair_log.append({"i": i, "j": j, "angle_deg": ang, "votes": votes,
                                 "status": "no_features"})
                continue
            lab = dbscan_components(feats, dbscan_eps_mm, dbscan_min_samples)
            keep = lab >= 0
            feats = feats[keep] if keep.any() else feats

            # "the two furthest point clouds in the feature point cloud set are selected
            # as a pair of corner points" - the paper's own endpoint rule, verbatim
            t = (feats - p0) @ d
            c1 = p0 + t.min() * d
            c2 = p0 + t.max() * d
            if float(np.linalg.norm(c2 - c1)) < min_seam_length_mm:
                pair_log.append({"i": i, "j": j, "angle_deg": ang, "votes": votes,
                                 "status": "no_features"})
                continue
            seams.append(np.stack([c1, c2]))
            clusters.append(feats)
            pair_log.append({"i": i, "j": j, "angle_deg": ang, "votes": votes,
                             "status": "seam"})

    note = ""
    if not seams:
        cen: dict[str, int] = {}
        for pr in pair_log:
            cen[pr["status"]] = cen.get(pr["status"], 0) + 1
        note = (f"no seam from {len(planes)} planes; {len(pair_log)} pair(s): "
                + ", ".join(f"{v} {k}" for k, v in sorted(cen.items())))
    return PPFResult(seams, clusters, planes, pair_log, P, N, params, used_oracle,
                     used_exact, note)
