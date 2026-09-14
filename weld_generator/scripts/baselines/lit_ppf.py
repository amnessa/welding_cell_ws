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

Deviations from the printed algorithm, every one reachable as a kwarg
--------------------------------------------------------------------
Each of these is a place where the paper is silent, self-contradictory, or describes a
sensor this dataset does not have; each keeps the literal reading one kwarg away.

* **Duplicate planes are merged** (`ppf_planes`). Two 25 mm samples on one face grow the
  same plane and the paper never merges, which would run every downstream pair test on a
  plane against a copy of itself.
* **Re-seeding after the distance grid** (`ppf_planes`, 8 rounds, lowest unclaimed index
  so it stays deterministic). Sampling at 25 mm can never seed a surface that lies WITHIN
  the interval of an already-sampled one - an edge joint's second sheet sits 8 mm above
  the first - so without re-seeding the coverage result for coplanar joints would be an
  artifact of the sampling stage instead of a verdict on the orthogonality gate.
* **One seam per SCENE, not one per pair** (`dedup=True`, the default). Algorithm 1
  accumulates the feature points of *all* orthogonal plane pairs into a single set `F`
  and emits ONE corner pair for the scene. Emitting a seam per accepted pair instead
  turns every duplicate or spurious plane into a false positive - measured at recall
  >= 0,5 with precision <= 0,31 on curved strata. Seams whose intersection lines are
  collinear (`dedup_angle_deg`, `dedup_offset_mm`) are therefore merged into one seam
  over the UNION of their feature points. `dedup=False` restores the per-pair behaviour.
* **Outlier removal keeps the dominant cluster** (`keep_clusters="largest"`, the default).
  The paper refines the feature points with "DBSCAN clustering and outlier removal";
  keeping every non-noise label (`keep_clusters="all"`, the old behaviour) lets a distant
  blob stretch the farthest-pair corners clean across the band.
* **The orthogonality gate carries a tolerance** (`ortho_tol_deg=15`, `pair_rule`). The
  paper's OPP test is categorical - a pair is orthogonal or it is not - yet Table 2 puts
  V-shaped welds through the same pipeline, which no exact 90 deg test can accept. The
  15 deg is this module's invention and is named as such; `pair_rule="intersecting"`
  drops the 90 deg anchor entirely and takes any non-parallel pair (fold angle in
  `[fold_min_deg, 180 - fold_min_deg]`), so a sweep can price the gate rather than
  assume it.
* **theta is folded modulo pi before binning** (`opp_vote`). Estimated normals have
  arbitrary sign, so an unfolded histogram splits one intersection direction across two
  antipodal bins and halves the peak it is about to threshold.

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
             theta_bins: int = 36, pair_rule: str = "orthogonal",
             fold_min_deg: float = 20.0
             ) -> tuple[int, float, float, int] | None:
    """The local Hough vote for one candidate pair. `(votes, theta, rho, n_voted)` or `None`.

    Directed point pairs are drawn across the two planes; the pair is admitted when it
    passes `pair_rule`, and each admitted pair votes in the space of eqs. 22-23 - `theta`,
    the partner normal's azimuth once the reference normal is rotated onto z, and
    `rho = n2 . (x1 - x2)`.

    `pair_rule` is the gate the paper leaves categorical:

    * `"orthogonal"` (default, the printed test): the descriptor's angle component must sit
      within `ortho_tol_deg` of 90 deg. The tolerance is **not published** - the paper's
      OPP test is a yes/no - but a zero-tolerance test admits nothing on a sampled cloud,
      and the paper's own Table 2 runs V welds through this pipeline. 15 deg is kept as the
      default because it is what the rest of Phase 4 was measured with.
    * `"intersecting"`: any non-parallel pair, fold angle within
      `[fold_min_deg, 180 - fold_min_deg]`. Normal signs are arbitrary, so only the acute
      normal angle `ang = arccos|n1.n2|` is observable and a fold of `ang` is
      indistinguishable from one of `180 - ang`; the interval test reduces to
      `ang >= fold_min_deg`. This rung prices the orthogonality gate: it is what lets a
      60 deg V fold through a pipeline built around 90 deg corners.

    `n_voted`, the number of point pairs that voted, is returned so `detect` can read
    `min_votes` as a fraction of it: the strided pair draw gives ~`n_pairs` pairs, so the
    historical `min_votes=30` was always ~7,5 % of them, not an absolute quantity of
    evidence.

    **The vote is local, and the first implementation here got that wrong.** For a fixed
    reference point `x1`, `rho = n2 . x1 - n2 . x2` is constant over every partner on a
    true plane (the second term is the plane's offset) - so per reference point the votes
    land in ONE bin by construction, and *across* reference points `rho` varies with `x1`'s
    distance from the partner plane, also by construction. A global (theta, rho)
    accumulator therefore scatters a perfect fold over dozens of rho bins, which is
    exactly what it did. The statistic that is global for a genuine orthogonal *plane*
    pair is `theta` - one intersection direction - so the peak is taken over theta, and
    a curved or accidental pairing smears it. `rho`'s per-reference consistency is what
    eq. 23 contributes to *locating* the line; with the planes already parameterised the
    line is recovered in closed form, which coincides with the voted location for true
    planes. Everything here is deterministic: pairs are strided, not drawn.

    theta is folded **modulo pi** before binning. Normal orientation is arbitrary under
    PCA estimation, so `n2` and `-n2` are both plausible readings of one plane and their
    azimuths differ by pi; unfolded, one intersection direction lands in two antipodal
    bins and the peak that `min_votes` thresholds is halved by a sign convention. The
    `theta_bins` bins now span `[0, pi)`.
    """
    cosd = abs(float(np.clip(ref.normal @ other.normal, -1, 1)))
    ang = float(np.degrees(np.arccos(cosd)))         # acute angle between the normals
    if pair_rule == "orthogonal":
        if abs(ang - 90.0) > float(ortho_tol_deg):
            return None                                # not orthogonal: no vote at all
    elif pair_rule == "intersecting":
        if ang < float(fold_min_deg):
            return None                                # parallel (or nearly): no fold
    else:
        raise ValueError('pair_rule must be "orthogonal" or "intersecting", '
                         f'got {pair_rule!r}')
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

    th = np.mod(np.broadcast_to(theta[None, :], rho.shape).ravel(), np.pi)
    ti = (th / np.pi * theta_bins).astype(int) % theta_bins
    counts = np.bincount(ti, minlength=theta_bins)
    peak = int(counts.argmax())
    votes = int(counts[peak])
    sel = ti == peak
    return (votes, float(np.median(th[sel])), float(np.median(rho.ravel()[sel])),
            int(th.size))


# --------------------------------------------------------------------------------
# result and driver
# --------------------------------------------------------------------------------

@dataclass
class PPFResult:
    """What `lit-ppf` returned, and enough context to know what it means."""

    seams: list[np.ndarray]               #: corner-pair polylines, one per accepted OPP
    clusters: list[np.ndarray]            #: the feature points behind each seam
    planes: list[PPFPlane]
    #: every plane pair considered: {i, j, angle_deg, votes, vote_frac, n_pairs, status}
    #: with status one of `seam`, `merged_seam` (accepted, then absorbed into a collinear
    #: seam by `dedup` - carries `merged_into`, the index of the keeper's pair entry),
    #: `not_orthogonal` (failed `pair_rule`, whichever rule is in force), `low_votes`,
    #: `no_features` - the coverage claim as data, same discipline as
    #: `LitRansacResult.pairs`
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


def merge_collinear(cands: list[dict[str, Any]], angle_deg: float = 5.0,
                    offset_mm: float = 2.0) -> list[list[dict[str, Any]]]:
    """Group accepted pairs whose intersection LINES are the same line.

    This is Algorithm 1's accounting restored: the paper pools the feature points of every
    orthogonal pair into one set `F` and reports one corner pair, so two plane pairs that
    voted for the same crease are one seam, not two. Two lines are the same when their
    directions agree within `angle_deg` (sign-free: `d` and `-d` are one direction) and the
    perpendicular offset of one anchor from the other's line is under `offset_mm`.

    Greedy, highest-vote candidate first, so the keeper of each group is its best-supported
    pair; ties keep input order, which makes the grouping deterministic. Returns a list of
    groups, each group's keeper first.
    """
    cos_tol = np.cos(np.radians(float(angle_deg)))
    groups: list[list[dict[str, Any]]] = []
    order = sorted(range(len(cands)), key=lambda k: (-cands[k]["votes"], k))
    for k in order:
        c = cands[k]
        for g in groups:
            r = g[0]
            if abs(float(r["d"] @ c["d"])) < cos_tol:
                continue
            if float(np.linalg.norm(np.cross(c["p0"] - r["p0"], r["d"]))) > offset_mm:
                continue
            g.append(c)
            break
        else:
            groups.append([c])
    return groups


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
           pair_rule: str = "orthogonal",
           ortho_tol_deg: float = 15.0,
           fold_min_deg: float = 20.0,
           theta_bins: int = 36,
           min_votes: float = 30.0,
           feature_tol_mm: float | None = None,
           dbscan_eps_mm: float | None = None,
           dbscan_min_samples: int = 6,
           keep_clusters: str = "largest",
           min_feature_pts: int = 10,
           min_seam_length_mm: float = 5.0,
           dedup: bool = True,
           dedup_angle_deg: float = 5.0,
           dedup_offset_mm: float | None = None,
           segmentation_mask: np.ndarray | None = None) -> PPFResult:
    """Run `lit-ppf` end to end. Lengths in **millimetres**.

    Args:
        normals: `"estimate"` (the paper: normals from the scan via local PCA — the L2
            condition) or `"exact"` with `normals_xyz` (the generator's analytic normals —
            the normal-oracle rung of the ladder). The delta between the two arms is the
            price of normal estimation, a number no paper in the seven reports.
        sample_interval_mm: published, 25 mm.
        coplanar_tol_mm: plane-growth and point-assignment distance. Unpublished; defaults
            to 2x the measured point spacing and is logged in `params` as resolved.
        pair_rule: `"orthogonal"` (the printed OPP test, tolerance `ortho_tol_deg`) or
            `"intersecting"` (any non-parallel pair, fold angle within
            `[fold_min_deg, 180 - fold_min_deg]`). See `opp_vote`.
        ortho_tol_deg: half-width of the orthogonality gate, **unpublished** - the paper's
            test is categorical. Honoured end to end: it reaches `opp_vote` for every pair
            and is logged in `params`. Sweeping it (15 / 30 / 45) prices the invention.
        theta_bins: bins of the Hough peak, spanning `[0, pi)` after the modulo-pi fold.
        min_votes: acceptance threshold on the Hough peak. Unpublished; this is the knob
            that separates a real orthogonal pair from an accidental one. Read as an
            **absolute count** when `>= 1` and as a **fraction of the pairs that voted**
            when `< 1` - the honest reading of what it always was, since the strided draw
            gives ~400 pairs and 30 of them is ~7,5 %. `params` logs both.
        keep_clusters: `"largest"` (the paper's "outlier removal": the dominant DBSCAN
            cluster is the crease, everything else is an outlier) or `"all"` (every
            non-noise label, this module's earlier behaviour, which lets a distant blob
            stretch the farthest-pair corners across the band).
        feature_tol_mm: the experiment section's published 0,1 mm — for a 50 um scanner.
            Default scales as 1,5x the point spacing instead, or nothing survives on a
            ~1 mm-spacing cloud; pass 0.1 to reproduce the paper literally.
        dedup: merge seams that are the same line (Algorithm 1 pools all pairs into one
            feature set and emits ONE corner pair). `False` emits one seam per accepted
            pair, which is what this module did before and what makes duplicate planes
            read as false positives.
        dedup_angle_deg, dedup_offset_mm: the collinearity tolerances; the offset defaults
            to 2x `feature_tol_mm`, the width of the feature corridor itself.
        segmentation_mask: the Faster R-CNN weld-box crop, supplied as an **oracle** via
            `seam_region_oracle` (a learned 2D weld-region detector, the same shape of
            stage as Yi et al.'s PointNet++). Withhold for the L1 arm.
    """
    if keep_clusters not in ("largest", "all"):
        raise ValueError('keep_clusters must be "largest" or "all", '
                         f'got {keep_clusters!r}')
    if pair_rule not in ("orthogonal", "intersecting"):
        raise ValueError('pair_rule must be "orthogonal" or "intersecting", '
                         f'got {pair_rule!r}')
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
    dedup_offset_mm = float(dedup_offset_mm) if dedup_offset_mm is not None \
        else 2.0 * feature_tol_mm
    # resolved here rather than inside `ppf_planes` so the value can be logged
    if coplanar_tol_mm is None and len(P) > 1:
        coplanar_tol_mm = max(2.0 * mean_spacing_mm(P), 0.5)

    params = dict(normals=normals, voxel_mm=voxel_mm, normal_k=normal_k,
                  sample_interval_mm=sample_interval_mm, angle_tol_deg=angle_tol_deg,
                  assign_order=assign_order, coplanar_tol_mm=coplanar_tol_mm,
                  min_plane_pts=min_plane_pts, pair_rule=pair_rule,
                  ortho_tol_deg=ortho_tol_deg, fold_min_deg=fold_min_deg,
                  theta_bins=theta_bins, min_votes=min_votes,
                  min_votes_is_fraction=bool(min_votes < 1),
                  feature_tol_mm=feature_tol_mm, dbscan_eps_mm=dbscan_eps_mm,
                  dbscan_min_samples=dbscan_min_samples, keep_clusters=keep_clusters,
                  min_feature_pts=min_feature_pts, min_seam_length_mm=min_seam_length_mm,
                  dedup=dedup, dedup_angle_deg=dedup_angle_deg,
                  dedup_offset_mm=dedup_offset_mm, n_input=len(P))
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

    cands: list[dict[str, Any]] = []
    pair_log: list[dict[str, Any]] = []
    for i in range(len(planes)):
        for j in range(i + 1, len(planes)):
            a, b = planes[i], planes[j]
            ang = float(np.degrees(np.arccos(np.clip(abs(a.normal @ b.normal), -1, 1))))
            vote = opp_vote(P, N, a, b, ortho_tol_deg, theta_bins=theta_bins,
                            pair_rule=pair_rule, fold_min_deg=fold_min_deg)
            if vote is None:
                pair_log.append({"i": i, "j": j, "angle_deg": ang, "votes": 0,
                                 "vote_frac": 0.0, "n_pairs": 0,
                                 "status": "not_orthogonal"})
                continue
            votes, _, _, n_voted = vote
            frac = votes / max(n_voted, 1)
            rec = {"i": i, "j": j, "angle_deg": ang, "votes": votes, "vote_frac": frac,
                   "n_pairs": n_voted}
            # `min_votes < 1` is read as the fraction of voting pairs it always was
            need = float(min_votes) if min_votes >= 1 else float(min_votes) * n_voted
            if votes < need:
                pair_log.append({**rec, "status": "low_votes"})
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
                pair_log.append({**rec, "status": "no_features"})
                continue
            lab = dbscan_components(feats, dbscan_eps_mm, dbscan_min_samples)
            if (lab >= 0).any():
                if keep_clusters == "largest":
                    # "DBSCAN clustering and outlier removal": the crease is the dominant
                    # cluster and every other run of points is an outlier. Keeping all of
                    # them instead lets one distant blob set an endpoint.
                    vals, cnt = np.unique(lab[lab >= 0], return_counts=True)
                    feats = feats[lab == vals[int(cnt.argmax())]]
                else:
                    feats = feats[lab >= 0]
            if len(feats) < min_feature_pts:
                pair_log.append({**rec, "status": "no_features"})
                continue

            # "the two furthest point clouds in the feature point cloud set are selected
            # as a pair of corner points" - the paper's own endpoint rule, verbatim
            t = (feats - p0) @ d
            if float(t.max() - t.min()) < min_seam_length_mm:
                pair_log.append({**rec, "status": "no_features"})
                continue
            pair_log.append({**rec, "status": "seam"})
            cands.append({"p0": p0, "d": d, "feats": feats, "votes": votes,
                          "log": len(pair_log) - 1})

    groups = merge_collinear(cands, dedup_angle_deg, dedup_offset_mm) if dedup \
        else [[c] for c in cands]

    seams, clusters = [], []
    for g in groups:
        keep = g[0]
        # Algorithm 1's single feature set F: the union over the pairs that voted for this
        # line, corners taken from the union exactly as the paper takes them from F
        feats = np.unique(np.vstack([c["feats"] for c in g]), axis=0)
        p0, d = keep["p0"], keep["d"]
        t = (feats - p0) @ d
        c1, c2 = p0 + t.min() * d, p0 + t.max() * d
        if float(np.linalg.norm(c2 - c1)) < min_seam_length_mm:
            continue
        for c in g[1:]:
            pair_log[c["log"]]["status"] = "merged_seam"
            pair_log[c["log"]]["merged_into"] = keep["log"]
        seams.append(np.stack([c1, c2]))
        clusters.append(feats)

    note = ""
    if not seams:
        cen: dict[str, int] = {}
        for pr in pair_log:
            cen[pr["status"]] = cen.get(pr["status"], 0) + 1
        note = (f"no seam from {len(planes)} planes; {len(pair_log)} pair(s): "
                + ", ".join(f"{v} {k}" for k, v in sorted(cen.items())))
    elif dedup and len(cands) > len(seams):
        note = (f"{len(cands)} accepted pair(s) merged into {len(seams)} seam(s) "
                "(Algorithm 1 pools the feature points of every pair)")
    return PPFResult(seams, clusters, planes, pair_log, P, N, params, used_oracle,
                     used_exact, note)
