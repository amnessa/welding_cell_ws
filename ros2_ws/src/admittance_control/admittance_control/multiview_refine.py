"""Multi-view close-range pose refinement: the JOINT REFINEMENT of touching parts
(step 3 of notes/multiview_refine_plan.md, designs D5-D7). Pure numpy; no ROS.

Input: the saved parts (CAD surface samples with outward normals, their saved
`pose_static`) and K views of the same static scene, each a cloud in the static frame
with normals facing that view's camera. Output: refined poses, what each view alone
says (spread, extrinsic estimate) and an accept/reject verdict per part.

The four guards against touching parts drifting into each other (plan D5):

1. **Ownership.** Every scene point belongs to the part whose registered surface is
   nearest; points within `dead_band_m` (4 mm) of two parts' surfaces at once are
   dropped (the strip where parts meet pulls both ways and the root does not need it).
   Ownership is fixed inside a solve and recomputed between rounds. The ICP's normal
   gate separates parts that meet at a right angle, so for a T ownership changes nothing
   (tested); it is there for parallel faces. Capping `max_corr_dist` at half the
   parallel-face gap (`parallel_gap_rule`) is available but OFF: in the synthetic lap
   joint no swap happened without it, and it cost correction range.
2. **A prior on the correction.** Each part minimises the point-to-plane energy plus
   `xi^T Sigma^-1 xi` (xi: the correction from the saved pose, in se(3) about the part's
   own centre; Sigma from `prior_sigma_mm` / `prior_sigma_deg`), scaled by the point
   noise so the two terms are in the same units: a MAP estimate with the saved pose as
   the prior. Where the views measure a direction the data wins; where they do not
   (a plate seen from above sliding in its own plane) the part stays put. The data's
   information matrix at the solution shows which directions were measured
   (`weak_directions`).
3. **The overlap rule.** Points on a part's faces near a neighbour (`contact_samples`)
   may not sink more than `penetration_tol_m` into the neighbour's box: a one-sided
   point-to-plane penalty against the box face, active only while penetrating. A gap
   costs nothing - the fit-up gap is real information, mode A measures it.
4. **Equal weight per view.** Each scene point carries 1/(points in its view), so the
   closest or densest view does not dominate.

Parts take turns (`rounds`, 2 by default: the cell registers two parts): each refines
against the others held at their current poses, the best-supported part (most owned
points) first.

The solver is a small Gauss-Newton loop of its own rather than `icp.icp_point_to_plane`
because the prior must be expressed about the part's centre (a 1 deg turn about the
base_link origin, 0.5 m away, would move the part 9 mm) and the overlap rule is a
second residual type; it reuses the ICP's building blocks (NNIndex, the normal gate,
Welsch weights). The ICP itself is unchanged.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

import numpy as np

from .icp import NNIndex, _rodrigues, _welsch_weight
from .multiview import _points_box_distance


@dataclass
class RefineConfig:
    noise_m: float = 0.0015               # point-to-plane noise of one scene point (D435i, ~0.4 m)
    max_corr_m: float = 0.010
    min_corr_m: float = 0.003             # floor for the parallel-face rule
    # cap max_corr at half the smallest parallel-face gap between parts (D5 layer 1). OFF:
    # in the synthetic lap joint no swap happened even without ownership at 10 mm (the
    # lower plate's exposed top lies BESIDE the upper plate, never under its visible top),
    # while the cap cost correction range (a 6 mm error was corrected without it, stuck
    # with it). The gap is still measured and reported (AssemblyDiag.parallel_gap_m).
    parallel_gap_rule: bool = False
    normal_gate_deg: float = 60.0
    dead_band_m: float = 0.004
    own_max_m: float = 0.015              # scene points farther than this from every part: nobody's
    prior_sigma_mm: float = 3.0
    prior_sigma_deg: float = 1.0
    overlap_band_m: float = 0.008         # model points this close to a neighbour's box: contact samples
    contact_samples: int = 200
    # resting contact (2026-10-02): a part standing on another rests ON it - parts never
    # go into each other (the overlap rule) and do not float. Facing faces (outward normals
    # opposite within resting_facing_deg, e.g. the ear's foot on the base top) get a gentle
    # pull towards touching, weighted like resting_weight camera points per sample: where
    # the views measure the gap the data win and a real gap stays visible; where they
    # cannot see (a standing plate's rotation in its own plane, constrained only by 8 mm
    # edges) the pull decides. The gap that remains is measured and checked against the
    # ISO 5817:2023 no. 617 fillet root-gap limit at quality_level (weldgen's own
    # root_gap_limit): over it, a WARNING, not a rejection - a bad fit-up is information.
    # OFF by default (2026-10-02, the user's call): the gap must be MEASURED and warned
    # about - an operator who places a part too far off must be told. On the bench the
    # pull overrode the data (a synthetic 3 mm gap closed to < 1 mm, unwarned) and, with
    # the absolute observability rule, the views alone already determine the standing
    # plate (4 real runs, two registrations: consistent to 0.4 mm). Kept as an option for
    # scenes where resting is known and the views are poor.
    resting_contact: bool = False         # the faces that rest are the DOWN-facing ones (gravity)
    resting_weight: float = 1.0
    resting_facing_deg: float = 20.0
    quality_level: str = "C"
    penetration_tol_m: float = 0.0005
    overlap_weight: float = 10.0          # x the per-point data weight (1 / mean view size)
    rounds: int = 2
    max_iter: int = 30
    welsch_k: float = 3.0                 # nu starts at max(k * median |r|, p90 |r|), shrinks to noise_m
    observable_ratio: float = 0.05        # (kept for older callers; the criterion is absolute now)
    # a part direction counts as measured when its std (from the information of the data
    # AND the contacts, a point = noise_m) is at most this - absolute, as for d: a relative
    # rule called the ear's in-plane rotation "not measured" although its resting contact
    # (~50 samples) pinned it to a fraction of a mm, because the faces carry thousands
    observable_max_sigma_mm: float = 0.5
    # a direction of the extrinsic error d counts as determined when its standard deviation
    # (from the information, a point = noise_m) is at most this. Absolute, not relative: the
    # planned T views measure their weakest direction of d to 0.16 mm (actual errors
    # 0.2-0.33 mm over 4 noise draws) - a 5 % relative rule had thrown it away.
    extrinsic_max_sigma_mm: float = 0.5
    # online self-calibration: when this run's d (its determined part) exceeds
    # online_selfcal_min_mm, take R_cam d out of every view and refine again - the views
    # then agree and the parts land where they are (synthetic: within 0.5 mm, against a
    # 1.2 deg tilt the uncorrected joint fit had accepted). The ORIGINAL d is what the
    # persistent self-calibration records. Off: a d that large rejects every part instead.
    online_selfcal: bool = True
    online_selfcal_min_mm: float = 0.5
    converge_mm: float = 0.2
    converge_deg: float = 0.05
    # acceptance (plan D7)
    max_correction_mm: float = 10.0
    max_correction_deg: float = 4.0       # 3 until 2026-10-02 (the user raised it: an ear correction of 3.07 deg was refused)
    min_fitness: float = 0.2
    max_penetration_mm: float = 1.0
    max_relative_mm: float = 2.0
    max_relative_deg: float = 1.0


@dataclass
class Part:
    name: str
    pts: np.ndarray                       # model-frame surface samples (m)
    nrm: np.ndarray                       # outward normals, model frame
    verts: np.ndarray                     # model-frame vertices (m) - for the box
    T_saved: np.ndarray                   # 4x4 pose_static
    # the camera that registered it (assembly.json T_static_camera): its share of a camera
    # translation error, R_scan d, sits in T_saved too - the online self-calibration takes
    # it out of the prior as well as out of the views
    T_scan: Optional[np.ndarray] = None


@dataclass
class View:
    pts: np.ndarray                       # static frame (m)
    nrm: np.ndarray                       # facing that view's camera
    R_cam: Optional[np.ndarray] = None    # camera rotation in the static frame (for the extrinsic estimate)


@dataclass
class AssemblyDiag:
    """What the views say together (D6): the extrinsic translation error `d` in the camera
    frame (`estimate_extrinsic`), only the components the views determine, its 3x3
    information (1/mm^2) for combining runs, and the directions they cannot see."""
    max_corr_m: float = 0.0
    parallel_gap_m: float = float("inf")  # smallest same-facing parallel gap between parts (reported)
    rounds_run: int = 0
    extrinsic_d_mm: Optional[np.ndarray] = None      # its DETERMINED part only (camera frame)
    extrinsic_info: Optional[np.ndarray] = None      # 3x3 information of d, 1/mm^2 (camera frame)
    extrinsic_weak: list[str] = field(default_factory=list)   # directions of d the views cannot see
    extrinsic_sigma_mm: Optional[np.ndarray] = None  # std per eigen-direction of the information
    extrinsic_cond: float = float("nan")             # smallest / largest eigenvalue of the information
    extrinsic_rms_mm: float = float("nan")           # point-to-plane RMS with d taken out
    applied_d_mm: Optional[np.ndarray] = None        # online self-calibration: d taken out of the views
    residual_d_mm: Optional[np.ndarray] = None       # ... and what the corrected views still say


@dataclass
class PartResult:
    name: str
    T_saved: np.ndarray
    T: np.ndarray
    accepted: bool = True
    reason: str = ""
    n_owned: int = 0
    fitness: float = 0.0
    rmse_mm: float = float("nan")
    correction_mm: float = 0.0
    correction_deg: float = 0.0
    weak: list[str] = field(default_factory=list)
    weak_vectors: Optional[np.ndarray] = None   # (k, 6) unit, in (rho*w, t) coordinates
    max_penetration_mm: float = 0.0
    per_view: list[Optional[np.ndarray]] = field(default_factory=list)
    view_spread_mm: float = float("nan")
    view_spread_deg: float = float("nan")
    H: Optional[np.ndarray] = None        # data information at the joint pose, (rho*w, t)
    prior_shift_mm: float = 0.0           # online self-calibration: R_scan d taken out of the prior
    beyond_mm: float = 0.0                # ... and the correction beyond that (what the limits judge)
    beyond_deg: float = 0.0
    T_attempt: Optional[np.ndarray] = None   # the refined pose before judge/fallback (kept for the report)
    gaps_mm: dict = field(default_factory=dict)   # neighbour -> (min, max) gap of the faces resting on it
    gap_limit_mm: dict = field(default_factory=dict)  # neighbour -> ISO 5817 no. 617 limit used
    warnings: list = field(default_factory=list)


# ---------------------------------------------------------------- small helpers -------
def _T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def _rotvec(R: np.ndarray) -> np.ndarray:
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-12:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def pose_delta(Ta: np.ndarray, Tb: np.ndarray, about: Optional[np.ndarray] = None) -> tuple[float, float]:
    """(mm, deg) between two poses of one part: the rotation angle and how far the point
    `about` (default: the pose's origin) moves."""
    p = np.zeros(3) if about is None else np.asarray(about, float)
    pa = Ta[:3, :3] @ p + Ta[:3, 3]
    pb = Tb[:3, :3] @ p + Tb[:3, 3]
    return float(np.linalg.norm(pb - pa) * 1000), float(np.degrees(np.linalg.norm(_rotvec(Tb[:3, :3] @ Ta[:3, :3].T))))


def box_of(part: Part, T: np.ndarray) -> dict[str, Any]:
    lo, hi = part.verts.min(0), part.verts.max(0)
    return {"name": part.name, "type": "box", "centre": T[:3, :3] @ ((lo + hi) / 2) + T[:3, 3],
            "R": T[:3, :3].copy(), "half": (hi - lo) / 2, "group": "scene"}


def signed_distance_box(p: np.ndarray, bx: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Signed distance of points to a box (negative inside) and the outward normal of the
    face that distance is measured to (the nearest face from inside)."""
    local = (p - bx["centre"]) @ bx["R"]
    q = np.abs(local) - bx["half"]
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(q.max(axis=1), 0.0)
    sd = outside + inside
    k = np.argmax(q, axis=1)                                    # the face the point is nearest to
    n_local = np.zeros_like(local)
    n_local[np.arange(len(p)), k] = np.sign(local[np.arange(len(p)), k]) + (local[np.arange(len(p)), k] == 0)
    return sd, n_local @ bx["R"].T


def posed(part: Part, T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return part.pts @ T[:3, :3].T + T[:3, 3], part.nrm @ T[:3, :3].T


# ---------------------------------------------------------------- guard 1: ownership --
def ownership(scene: np.ndarray, parts: Sequence[Part], poses: Sequence[np.ndarray],
              cfg: RefineConfig) -> np.ndarray:
    """Owner index per scene point: the part with the nearest registered surface; -1 for
    points within `dead_band_m` of two parts' surfaces at once, or farther than
    `own_max_m` from every part."""
    d = np.stack([NNIndex(posed(p, T)[0]).query(scene)[1] for p, T in zip(parts, poses)], axis=1)
    order = np.sort(d, axis=1)
    owner = np.argmin(d, axis=1)
    owner = np.where(order[:, 0] > cfg.own_max_m, -1, owner)
    if d.shape[1] > 1:
        owner = np.where(order[:, 1] < cfg.dead_band_m, -1, owner)
    return owner


def parallel_face_gap(parts: Sequence[Part], poses: Sequence[np.ndarray], cfg: RefineConfig,
                      view_dirs: Optional[np.ndarray] = None, lateral_m: float = 0.005) -> float:
    """The smallest gap between same-facing parallel faces of DIFFERENT parts that sit one
    above the other (within `lateral_m` sideways), touching faces excluded - the case the
    normal gate cannot separate (a lap joint: the two top faces). Only faces turned
    towards at least one view count (`view_dirs`: the optical axes): the base's underside
    and the ear's foot face are 8.9 mm apart and both face down, and no camera above the
    table ever confuses them. inf when there is none."""
    best = np.inf
    surf = []
    for p, T in zip(parts, poses):
        pw, nw = posed(p, T)
        if view_dirs is not None and len(view_dirs):
            seen = (nw @ (-np.asarray(view_dirs)).T).max(axis=1) > 0.1
            pw, nw = pw[seen], nw[seen]
        surf.append((pw, nw))
    for i in range(len(parts)):
        for j in range(len(parts)):
            if i == j:
                continue
            (pi, ni), (pj, nj) = surf[i], surf[j]
            if len(pi) == 0 or len(pj) == 0:
                continue
            idx = NNIndex(pj)
            k = min(8, len(pj))
            nb, _ = idx.knn(pi, k)
            nb = np.atleast_2d(nb)
            for c in range(nb.shape[1]):
                q, nq = pj[nb[:, c]], nj[nb[:, c]]
                same = np.einsum("ij,ij->i", ni, nq) > 0.95
                v = q - pi
                along = np.abs(np.einsum("ij,ij->i", v, ni))
                lat = np.linalg.norm(v - np.einsum("ij,ij->i", v, ni)[:, None] * ni, axis=1)
                ok = same & (lat < lateral_m) & (along > 2 * cfg.penetration_tol_m + 0.001)
                if ok.any():
                    best = min(best, float(along[ok].min()))
    return best


def corr_distance(parts, poses, cfg: RefineConfig, view_dirs: Optional[np.ndarray] = None) -> float:
    """`max_corr_m`, or with `parallel_gap_rule` at most half the parallel-face gap."""
    if not cfg.parallel_gap_rule:
        return cfg.max_corr_m
    gap = parallel_face_gap(parts, poses, cfg, view_dirs)
    return float(np.clip(gap / 2, cfg.min_corr_m, cfg.max_corr_m)) if np.isfinite(gap) else cfg.max_corr_m


# ---------------------------------------------------------------- one part -------------
def contact_samples(part: Part, T: np.ndarray, neighbours: Sequence[dict[str, Any]], cfg: RefineConfig,
                    rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """(N, 6) model-frame points and outward normals of `part` within `overlap_band_m` of a
    neighbour's box at pose T: the faces the overlap rule and the resting contact watch."""
    if not neighbours:
        return np.zeros((0, 6))
    pw, _ = posed(part, T)
    d = np.min([_points_box_distance(pw, b) for b in neighbours], axis=0)
    sel = np.flatnonzero(d < cfg.overlap_band_m)
    if len(sel) > cfg.contact_samples:
        sel = (rng or np.random.default_rng(0)).choice(sel, cfg.contact_samples, replace=False)
    return np.hstack([part.pts[sel], part.nrm[sel]])


def refine_part(part: Part, T_start: np.ndarray, T_prior: np.ndarray, scene: np.ndarray,
                scene_n: np.ndarray, scene_w: np.ndarray, neighbours: Sequence[dict[str, Any]],
                contacts: np.ndarray, cfg: RefineConfig, max_corr: Optional[float] = None,
                use_prior: bool = True, translation_only: bool = False) -> tuple[np.ndarray, dict[str, Any]]:
    """One part's MAP pose against its owned scene points (guards 2-4).

    Works about the part's centre: everything is shifted by -c, so the rotation part of
    the correction (and of the prior) turns about the part itself. Returns (T, info) with
    fitness, rmse, the data information matrix H (6x6, in (rho*w, t) coordinates, rho the
    part's radius) and the largest penetration.

    `translation_only` (with `use_prior=False`) is the per-view diagnostic: a camera
    translation error shifts a view's cloud rigidly, so that view's pose must differ from
    the joint one by a translation only - letting it turn, or pulling it back with the
    prior, bent the per-view poses and biased the extrinsic estimate by up to 1.5 mm."""
    max_corr = cfg.max_corr_m if max_corr is None else max_corr
    c = T_prior[:3, :3] @ part.pts.mean(0) + T_prior[:3, 3]
    S = _T(np.eye(3), -c)
    T = S @ T_start
    T0 = S @ T_prior
    q_all, n_all = scene - c, scene_n
    rho = float(np.percentile(np.linalg.norm(part.pts - part.pts.mean(0), axis=1), 90))
    cos_gate = np.cos(np.radians(cfg.normal_gate_deg))
    w_mean = float(np.mean(scene_w)) if len(scene_w) else 1.0
    prior_L = np.concatenate([np.full(3, cfg.noise_m / np.radians(cfg.prior_sigma_deg)),
                              np.full(3, cfg.noise_m / (cfg.prior_sigma_mm / 1000))])
    nb_shift = [{**b, "centre": b["centre"] - c} for b in neighbours]
    info: dict[str, Any] = {"fitness": 0.0, "rmse": float("nan"), "H": np.zeros((6, 6)), "penetration": 0.0}
    if len(q_all) < 6:
        return T_start.copy(), info
    index = NNIndex(q_all)
    nu = None
    for it in range(cfg.max_iter):
        R, t = T[:3, :3], T[:3, 3]
        src = part.pts @ R.T + t
        srcn = part.nrm @ R.T
        idx, dist = index.query(src)
        qn = n_all[idx]
        r = np.einsum("ij,ij->i", src - q_all[idx], qn)
        ok = (dist < max_corr) & (np.einsum("ij,ij->i", srcn, qn) >= cos_gate)
        if ok.sum() < 6:
            break
        if nu is None:
            # from the 90th percentile too: when most matched points slide along their
            # faces (residual 0) the median says nothing, and the few faces that DO see
            # the error (a plate's end faces, 3 mm) were weighted away before the first step
            ra = np.abs(r[ok])
            nu = max(cfg.welsch_k * float(np.median(ra)), float(np.percentile(ra, 90)), cfg.noise_m)
        wv = scene_w[idx] / w_mean
        w = np.where(ok, _welsch_weight(np.abs(r), nu) * wv, 0.0)
        sw = np.sqrt(w)
        rows = [np.hstack((np.cross(src, qn), qn)) * sw[:, None]]
        rhs = [-r * sw]
        # overlap rule: contact samples may not sink deeper than penetration_tol into a neighbour
        pen = 0.0
        if len(contacts) and nb_shift:
            s = contacts[:, :3] @ R.T + t
            ns = contacts[:, 3:] @ R.T
            cos_face = np.cos(np.radians(cfg.resting_facing_deg))
            for b in nb_shift:
                sd, nb_n = signed_distance_box(s, b)
                act = sd < -cfg.penetration_tol_m
                pen = max(pen, float(-sd.min()))
                if act.any():
                    ow = np.sqrt(cfg.overlap_weight)
                    rows.append(np.hstack((np.cross(s[act], nb_n[act]), nb_n[act])) * ow)
                    rhs.append(-(sd[act] + cfg.penetration_tol_m) * ow)
                if cfg.resting_contact:
                    # resting: facing faces not penetrating are pulled gently towards touching
                    # only a face pointing DOWN rests on its neighbour (gravity): the part on
                    # top rests on the one below, never the other way - pulling the base's top
                    # up to the ear's foot tilted the base after the ear and made the two
                    # chase each other (bench, 2026-10-02: the base differed 2-3 mm run to run)
                    rest = (~act) & (np.einsum("ij,ij->i", ns, nb_n) < -cos_face) & (ns[:, 2] < -cos_face)
                    if rest.any():
                        rw = np.sqrt(cfg.resting_weight)
                        rows.append(np.hstack((np.cross(s[rest], nb_n[rest]), nb_n[rest])) * rw)
                        rhs.append(-sd[rest] * rw)
        # prior: xi = the left correction from the prior pose, (rotvec, translation) about c
        if use_prior:
            D = T @ np.linalg.inv(T0)
            xi = np.concatenate([_rotvec(D[:3, :3]), D[:3, 3]])
            rows.append(np.diag(prior_L))
            rhs.append(-prior_L * xi)
        A = np.vstack(rows)
        b = np.concatenate(rhs)
        if translation_only:
            xt, *_ = np.linalg.lstsq(A[:, 3:], b, rcond=None)
            x = np.concatenate([np.zeros(3), xt])
        else:
            x, *_ = np.linalg.lstsq(A, b, rcond=None)
        T = _T(_rodrigues(x[:3]), x[3:]) @ T
        nu = max(nu * 0.7, cfg.noise_m)
        if np.linalg.norm(x[3:]) < 1e-6 and np.linalg.norm(x[:3]) < 1e-6 and nu <= cfg.noise_m * 1.0001:
            break
    # final metrics and the data information matrix (rotation scaled by rho -> metres)
    R, t = T[:3, :3], T[:3, 3]
    src = part.pts @ R.T + t
    srcn = part.nrm @ R.T
    idx, dist = index.query(src)
    qn = n_all[idx]
    r = np.einsum("ij,ij->i", src - q_all[idx], qn)
    ok = (dist < max_corr) & (np.einsum("ij,ij->i", srcn, qn) >= cos_gate)
    J = np.hstack((np.cross(src[ok], qn[ok]) / rho, qn[ok]))
    wv = (scene_w[idx] / w_mean)[ok]
    info["H"] = (J * wv[:, None]).T @ J
    info["rho"] = rho
    info["fitness"] = float(ok.mean())
    info["rmse"] = float(np.sqrt(np.mean(r[ok] ** 2))) if ok.any() else float("nan")
    info["gaps"] = {}
    if len(contacts) and nb_shift:
        s = contacts[:, :3] @ R.T + t
        ns = contacts[:, 3:] @ R.T
        info["penetration"] = max(0.0, max(float(-signed_distance_box(s, b)[0].min()) for b in nb_shift))
        cos_face = np.cos(np.radians(cfg.resting_facing_deg))
        for b in nb_shift:
            sd, nb_n = signed_distance_box(s, b)
            facing = (np.einsum("ij,ij->i", ns, nb_n) < -cos_face) & (ns[:, 2] < -cos_face)
            if facing.sum() >= 3:                     # the gap along the faces resting on b (pointing down)
                info["gaps"][b["name"]] = (float(sd[facing].min()), float(sd[facing].max()))
                if cfg.resting_contact:
                    # the resting contact MEASURES directions too (a standing plate's height
                    # and its rotation in its own plane, which the views hardly see): its
                    # information joins the data's, so the observability report and the
                    # fit-up rule count those directions as determined
                    Jc = np.hstack((np.cross(s[facing], nb_n[facing]) / rho, nb_n[facing]))
                    info["H"] = info["H"] + cfg.resting_weight * (Jc.T @ Jc)
    return _T(np.eye(3), c) @ T, info


def weak_directions(H: np.ndarray, noise_m: float, max_sigma_mm: float) -> tuple[list[str], np.ndarray]:
    """Directions hardly measured: eigenvectors of H (rho*w, t; a point of weight 1 has
    std noise_m) whose std noise_m / sqrt(eigenvalue) exceeds max_sigma_mm. Described as
    a slide (translation-dominated) or a turn."""
    lam, V = np.linalg.eigh(H)
    if lam.max() <= 0:
        return ["nothing measured"], np.eye(6)
    lam_min = (noise_m * 1000 / max_sigma_mm) ** 2
    weak = [(l, V[:, k]) for k, l in enumerate(lam) if l < lam_min]
    out = []
    for l, v in weak:
        rot, tr = v[:3], v[3:]
        if np.linalg.norm(tr) >= np.linalg.norm(rot):
            a = tr / np.linalg.norm(tr)
            out.append(f"slide along {np.round(a, 2).tolist()}")
        else:
            a = rot / np.linalg.norm(rot)
            out.append(f"turn about {np.round(a, 2).tolist()}")
    return out, (np.array([v for _, v in weak]) if weak else np.zeros((0, 6)))


# ---------------------------------------------------------------- the assembly ---------
def _stack(views: Sequence[View]):
    pts = np.vstack([v.pts for v in views])
    nrm = np.vstack([v.nrm for v in views])
    lab = np.concatenate([np.full(len(v.pts), k) for k, v in enumerate(views)])
    w = np.concatenate([np.full(len(v.pts), 1.0 / max(len(v.pts), 1)) for v in views])
    return pts, nrm, w, lab


def refine_assembly(parts: Sequence[Part], views: Sequence[View], cfg: Optional[RefineConfig] = None,
                    use_ownership: bool = True, per_view: bool = True,
                    rng: Optional[np.random.Generator] = None) -> tuple[list[PartResult], AssemblyDiag]:
    """Refine every part against all views (D5), take the per-view diagnostics (D6) and
    judge the result (D7). `use_ownership=False` is for the tests: every part sees every
    point."""
    cfg = cfg or RefineConfig()
    rng = rng or np.random.default_rng(0)
    pts, nrm, w, lab = _stack(views)
    poses = [np.asarray(p.T_saved, float).copy() for p in parts]
    view_dirs = np.array([v.R_cam[:, 2] for v in views if v.R_cam is not None])
    max_corr = corr_distance(parts, poses, cfg, view_dirs)
    diag = AssemblyDiag(max_corr_m=max_corr, parallel_gap_m=parallel_face_gap(parts, poses, cfg, view_dirs))
    owner = ownership(pts, parts, poses, cfg) if use_ownership else np.zeros(len(pts), int)
    counts = [int((owner == i).sum()) if use_ownership else len(pts) for i in range(len(parts))]
    order = sorted(range(len(parts)), key=lambda i: -counts[i])
    infos: dict[int, dict] = {}
    for rnd in range(cfg.rounds):
        diag.rounds_run = rnd + 1
        moved = 0.0, 0.0
        for i in order:
            mine = (owner == i) if use_ownership else np.ones(len(pts), bool)
            nbrs = [box_of(parts[j], poses[j]) for j in range(len(parts)) if j != i]
            contacts = contact_samples(parts[i], poses[i], nbrs, cfg, rng)
            T_new, info = refine_part(parts[i], poses[i], parts[i].T_saved, pts[mine], nrm[mine], w[mine],
                                      nbrs, contacts, cfg, max_corr)
            dmm, ddeg = pose_delta(poses[i], T_new, parts[i].pts.mean(0))
            moved = max(moved[0], dmm), max(moved[1], ddeg)
            poses[i], infos[i] = T_new, info
        if use_ownership:
            owner = ownership(pts, parts, poses, cfg)
        if moved[0] < cfg.converge_mm and moved[1] < cfg.converge_deg:
            break

    results = []
    for i, p in enumerate(parts):
        info = infos.get(i, {"fitness": 0.0, "rmse": float("nan"), "H": np.zeros((6, 6)), "penetration": 0.0})
        weak, wv = weak_directions(info["H"], cfg.noise_m, cfg.observable_max_sigma_mm)
        cmm, cdeg = pose_delta(p.T_saved, poses[i], p.pts.mean(0))
        res = PartResult(name=p.name, T_saved=p.T_saved, T=poses[i], n_owned=int((owner == i).sum()),
                         fitness=info["fitness"], rmse_mm=info["rmse"] * 1000, correction_mm=cmm,
                         correction_deg=cdeg, weak=weak, weak_vectors=wv,
                         max_penetration_mm=info["penetration"] * 1000, H=info["H"],
                         gaps_mm={k: (v[0] * 1000, v[1] * 1000) for k, v in info.get("gaps", {}).items()})
        if per_view:
            _per_view(res, p, i, poses, parts, pts, nrm, w, lab, owner, views, cfg, max_corr, rng, use_ownership)
        results.append(res)
    for r in results:
        r.T_attempt = r.T.copy()
    if per_view:
        estimate_extrinsic(parts, poses, views, owner, cfg, max_corr, diag, use_ownership)
        big = diag.extrinsic_d_mm is not None and np.linalg.norm(diag.extrinsic_d_mm) > cfg.online_selfcal_min_mm
        if big and cfg.online_selfcal:
            # the views disagree by a camera translation: take it out and refine again
            d = diag.extrinsic_d_mm / 1000
            fixed = [View(v.pts - v.R_cam @ d, v.nrm, v.R_cam) for v in views]
            # ... and out of the PRIOR: each saved pose was registered through the same
            # camera, from its scan pose, so it carries R_scan d too. Without this, what the
            # views do not measure (a plate's slide) stayed at the uncorrected saved pose
            # while the measured rest moved - a fake 8 mm fit-up change on the bench
            # (2026-10-02) and both parts rejected.
            prior = []
            for p in parts:
                Tp = p.T_saved.copy()
                if p.T_scan is not None:
                    Tp[:3, 3] -= np.asarray(p.T_scan, float)[:3, :3] @ d
                prior.append(Part(p.name, p.pts, p.nrm, p.verts, Tp, p.T_scan))
            again = dataclasses.replace(cfg, online_selfcal=False)
            results2, diag2 = refine_assembly(prior, fixed, again, use_ownership, per_view, rng)
            for r, p, q in zip(results2, parts, prior):
                # judged against the corrected prior; reported against what was saved
                r.prior_shift_mm = float(np.linalg.norm(q.T_saved[:3, 3] - p.T_saved[:3, 3]) * 1000)
                r.beyond_mm, r.beyond_deg = pose_delta(q.T_saved, r.T_attempt if r.T_attempt is not None else r.T,
                                                       p.pts.mean(0))
                r.T_saved = p.T_saved
                if not r.accepted:
                    r.T = p.T_saved.copy()
                # the correction as ATTEMPTED (a rejected part's T is back at its saved pose)
                r.correction_mm, r.correction_deg = pose_delta(
                    p.T_saved, r.T_attempt if r.T_attempt is not None else r.T, p.pts.mean(0))
            diag2.residual_d_mm = diag2.extrinsic_d_mm
            for name in ("extrinsic_d_mm", "extrinsic_info", "extrinsic_weak", "extrinsic_sigma_mm",
                         "extrinsic_cond", "extrinsic_rms_mm"):
                setattr(diag2, name, getattr(diag, name))       # the history records the ORIGINAL d
            diag2.applied_d_mm = diag.extrinsic_d_mm
            return results2, diag2
        if big:
            for r in results:
                r.accepted = False
                r.reason = (f"the views disagree by an extrinsic translation error d = "
                            f"{np.round(diag.extrinsic_d_mm, 1).tolist()} mm (camera frame); refine with "
                            f"online_selfcal, or self-calibrate the extrinsic first")
                r.T = r.T_saved.copy()
            return results, diag
    judge(results, parts, cfg)
    _fallback(results, parts, pts, nrm, w, owner, use_ownership, order, cfg, max_corr, rng)
    fitup_warnings(results, parts, cfg)
    return results, diag


def _fallback(results, parts, pts, nrm, w, owner, use_ownership, order, cfg: RefineConfig,
              max_corr: float, rng) -> None:
    """D7 fallback: a rejected part keeps its saved pose; every accepted part is refined
    once more against it (so no accepted part was fitted next to a pose that is no longer
    there), and an accepted part whose pose relative to a held one still changed beyond
    `max_relative_*` is kept as saved too - the assembly stays consistent, refined
    together or not at all."""
    held = [i for i, r in enumerate(results) if not r.accepted]
    if not held:
        return
    for i in held:
        results[i].T = parts[i].T_saved.copy()
    for i in order:
        r = results[i]
        if not r.accepted:
            continue
        mine = (owner == i) if use_ownership else np.ones(len(pts), bool)
        nbrs = [box_of(parts[j], results[j].T) for j in range(len(parts)) if j != i]
        contacts = contact_samples(parts[i], r.T, nbrs, cfg, rng)
        r.T, info = refine_part(parts[i], r.T, parts[i].T_saved, pts[mine], nrm[mine], w[mine],
                                nbrs, contacts, cfg, max_corr)
        r.correction_mm, r.correction_deg = pose_delta(parts[i].T_saved, r.T, parts[i].pts.mean(0))
        r.max_penetration_mm = info["penetration"] * 1000
        r.gaps_mm = {k: (v[0] * 1000, v[1] * 1000) for k, v in info.get("gaps", {}).items()}
    changed = True
    while changed:
        changed = False
        for i, r in enumerate(results):
            if not r.accepted:
                continue
            for j in held:
                dmm, ddeg = pose_delta(np.linalg.inv(parts[j].T_saved) @ parts[i].T_saved,
                                       np.linalg.inv(parts[j].T_saved) @ r.T, parts[i].pts.mean(0))
                if dmm > cfg.max_relative_mm or ddeg > cfg.max_relative_deg:
                    r.accepted = False
                    r.reason = (f"kept as saved with {parts[j].name} (rejected): refined alone it would "
                                f"change their fit-up by {dmm:.1f} mm / {ddeg:.2f} deg")
                    r.T = parts[i].T_saved.copy()
                    held.append(i)
                    changed = True
                    break


def _per_view(res: PartResult, part: Part, i: int, poses, parts, pts, nrm, w, lab, owner, views,
              cfg: RefineConfig, max_corr: float, rng, use_ownership: bool) -> None:
    """D6: the part registered to each view alone (from the joint pose, same prior), the
    spread of those poses, and the extrinsic translation `d` (camera frame) from
    t_v = t0 + R_cam,v d."""
    nbrs = [box_of(parts[j], poses[j]) for j in range(len(parts)) if j != i]
    contacts = contact_samples(part, poses[i], nbrs, cfg, rng)
    c_local = part.pts.mean(0)
    per, centres, Rs = [], [], []
    for k, v in enumerate(views):
        mine = (lab == k) & ((owner == i) if use_ownership else True)
        if mine.sum() < 50:
            per.append(None)
            continue
        Tk, _ = refine_part(part, poses[i], poses[i], pts[mine], nrm[mine], w[mine], nbrs, contacts, cfg, max_corr,
                            use_prior=False, translation_only=True)
        per.append(Tk)
        centres.append(Tk[:3, :3] @ c_local + Tk[:3, 3])
        Rs.append(v.R_cam)
    res.per_view = per
    good = [T for T in per if T is not None]
    if len(good) >= 2:
        # only in the directions the part measures: a plate's unmeasured in-plane slide
        # wanders per view and says nothing about the camera
        C = np.array(centres)
        P = np.eye(3)
        if res.H is not None:
            lam, U = np.linalg.eigh(res.H[3:, 3:])
            Us = U[:, lam >= (cfg.noise_m * 1000 / cfg.observable_max_sigma_mm) ** 2]
            P = Us @ Us.T
        res.view_spread_mm = float(np.linalg.norm(((C - C.mean(0)) @ P).std(0)) * 1000)
        res.view_spread_deg = max(pose_delta(poses[i], T, c_local)[1] for T in good)


def estimate_extrinsic(parts: Sequence[Part], poses: Sequence[np.ndarray], views: Sequence[View],
                       owner: np.ndarray, cfg: RefineConfig, max_corr: float, diag: AssemblyDiag,
                       use_ownership: bool = True, iters: int = 20) -> None:
    """D6: the extrinsic translation error d (camera frame) together with every part's pose,
    in ONE least-squares problem over all owned points: a point of view v is where the
    camera saw it shifted by R_cam,v d, so its residual is n . (T_i p - q + R_cam,v d),
    linear in d and in the parts' corrections (each about its own centre, no prior).

    Not every direction of d is determined by the views: one that moves EVERY view's cloud
    the same way is indistinguishable from all parts sitting elsewhere (for views at one
    elevation, the combination of image-down and forward that is purely vertical). The
    information of d is therefore the Schur complement of the normal matrix (the parts'
    corrections eliminated); eigen-directions whose standard deviation exceeds
    `extrinsic_max_sigma_mm` are reported and their component of d is set to 0 - only
    what the views see is kept. (For the planned T views, 45/60 deg, the weakest is a
    purely vertical common shift, still measured to 0.16 mm through the 60 deg view.)

    (The first version fitted each part to each view alone and regressed the per-view
    positions on R_cam: from the TRUE poses it returned (7, 13, -15) mm for an injected
    (3, -2, 1). This one uses the points directly.)"""
    if not views or any(v.R_cam is None for v in views):
        return
    pts, nrm, w, lab = _stack(views)
    RV = np.stack([v.R_cam for v in views])
    n = len(parts)
    Ts = [np.asarray(T, float).copy() for T in poses]
    d = np.zeros(3)
    cos_gate = np.cos(np.radians(cfg.normal_gate_deg))
    w_mean = float(np.mean(w))
    nu = None

    def system(Ts, d, nu):
        q = pts - np.einsum("kij,j->ki", RV[lab], d)
        rows, rhs, r_ok = [], [], []
        cs = [T[:3, :3] @ p.pts.mean(0) + T[:3, 3] for p, T in zip(parts, Ts)]
        for i, (p, T) in enumerate(zip(parts, Ts)):
            mine = (owner == i) if use_ownership else np.ones(len(q), bool)
            if mine.sum() < 6:
                continue
            qi, ni, wi, li = q[mine], nrm[mine], w[mine] / w_mean, lab[mine]
            src = p.pts @ T[:3, :3].T + T[:3, 3]
            srcn = p.nrm @ T[:3, :3].T
            idx, dist = NNIndex(qi).query(src)
            qn = ni[idx]
            r = np.einsum("ij,ij->i", src - qi[idx], qn)
            ok = (dist < max_corr) & (np.einsum("ij,ij->i", srcn, qn) >= cos_gate)
            if nu is None:
                ra = np.abs(r[ok])
                nu_i = max(float(np.percentile(ra, 90)) if len(ra) else cfg.noise_m, 2 * cfg.noise_m)
            else:
                nu_i = nu
            ww = np.where(ok, _welsch_weight(np.abs(r), nu_i) * wi[idx], 0.0)
            sw = np.sqrt(ww)
            J = np.zeros((len(src), 6 * n + 3))
            J[:, 6 * i:6 * i + 3] = np.cross(src - cs[i], qn)
            J[:, 6 * i + 3:6 * i + 6] = qn
            J[:, 6 * n:] = np.einsum("kj,kji->ki", qn, RV[li[idx]])
            rows.append(J * sw[:, None])
            rhs.append(-r * sw)
            r_ok.append(r[ok])
        return rows, rhs, cs, r_ok

    for it in range(iters):
        rows, rhs, cs, r_ok = system(Ts, d, nu)
        if not rows:
            return
        if nu is None:
            ra = np.abs(np.concatenate(r_ok))
            nu = max(float(np.percentile(ra, 90)) if len(ra) else cfg.noise_m, 2 * cfg.noise_m)
        A, b = np.vstack(rows), np.concatenate(rhs)
        x, *_ = np.linalg.lstsq(A, b, rcond=1e-9)
        for i in range(n):
            xi = x[6 * i:6 * i + 6]
            S = _T(np.eye(3), cs[i]) @ _T(_rodrigues(xi[:3]), xi[3:]) @ _T(np.eye(3), -cs[i])
            Ts[i] = S @ Ts[i]
        d = d + x[6 * n:]
        nu = max(nu * 0.7, cfg.noise_m)
        if np.linalg.norm(x) < 1e-7 and nu <= cfg.noise_m * 1.0001:
            break
    rows, rhs, _, r_ok = system(Ts, d, nu)
    A = np.vstack(rows)
    N = A.T @ A
    k = 6 * n
    Sd = N[k:, k:] - N[k:, :k] @ np.linalg.pinv(N[:k, :k], rcond=1e-10) @ N[:k, k:]
    Sd = (Sd + Sd.T) / 2
    lam, U = np.linalg.eigh(Sd)
    lam = np.maximum(lam, 0.0)
    lam_mm = lam / (cfg.noise_m ** 2) / 1e6                  # information, 1/mm^2
    sigma = np.where(lam_mm > 0, 1.0 / np.sqrt(np.maximum(lam_mm, 1e-30)), np.inf)
    strong = sigma <= cfg.extrinsic_max_sigma_mm
    Us = U[:, strong]
    diag.extrinsic_d_mm = (Us @ Us.T @ d) * 1000
    diag.extrinsic_info = (U * lam_mm) @ U.T                 # all directions; selfcal judges them
    diag.extrinsic_sigma_mm = sigma
    diag.extrinsic_weak = [f"camera-frame direction {np.round(U[:, k2], 2).tolist()} "
                           f"(sigma {sigma[k2]:.2f} mm)" for k2 in np.flatnonzero(~strong)]
    diag.extrinsic_cond = float(max(lam.min(), 0.0) / lam.max()) if lam.max() > 0 else 0.0
    ra = np.concatenate(r_ok) if r_ok else np.zeros(0)
    diag.extrinsic_rms_mm = float(np.sqrt(np.mean(ra ** 2)) * 1000) if len(ra) else float("nan")


def _thickness_mm(part: Part) -> float:
    return float((part.verts.max(0) - part.verts.min(0)).min() * 1000)


def fitup_gap_limit_mm(t_a_mm: float, t_b_mm: float, level: str) -> Optional[float]:
    """ISO 5817:2023 Table 1 no. 617, incorrect root gap for FILLET welds, at `level`,
    computed by weldgen's own root_gap_limit (throat a = 0.7 t_min, as weldgen draws it).
    None when weldgen cannot be imported. Butt and edge joints take their gap from ISO
    9692-1 instead - this cell's parts meet as fillets (T, corner, lap)."""
    try:
        from .seam_from_registration import import_weldgen
        import_weldgen()
        from weldgen.config import root_gap_limit
    except Exception:  # noqa: BLE001
        return None
    t = min(t_a_mm, t_b_mm)
    return float(root_gap_limit(t, 0.7 * t, level))


def fitup_warnings(results: Sequence[PartResult], parts: Sequence[Part], cfg: RefineConfig) -> None:
    """A gap between faces resting on each other beyond the ISO fillet root-gap limit is
    WARNED about (never a rejection: a real bad fit-up is information to act on)."""
    by_name = {p.name: p for p in parts}
    for r, p in zip(results, parts):
        r.warnings = [w for w in r.warnings if not w.startswith("fit-up gap")]
        for nb, (gmin, gmax) in r.gaps_mm.items():
            other = by_name.get(nb)
            if other is None:
                continue
            lim = fitup_gap_limit_mm(_thickness_mm(p), _thickness_mm(other), cfg.quality_level)
            if lim is None:
                continue
            r.gap_limit_mm[nb] = lim
            if gmax > lim:
                r.warnings.append(f"fit-up gap to {nb} up to {gmax:.1f} mm exceeds the ISO 5817 no. 617 "
                                  f"level {cfg.quality_level} fillet limit {lim:.1f} mm (t "
                                  f"{min(_thickness_mm(p), _thickness_mm(other)):.0f} mm, a = 0.7 t)")


def judge(results: Sequence[PartResult], parts: Sequence[Part], cfg: RefineConfig) -> None:
    """D7: reject a part's refinement (keep its saved pose) when the correction is too
    large, the fit too thin, it overlaps a neighbour, or the pose between two touching
    parts changed beyond `max_relative_*` along a direction the views did not measure."""
    for r in results:
        why = []
        if r.correction_mm > cfg.max_correction_mm or r.correction_deg > cfg.max_correction_deg:
            why.append(f"correction {r.correction_mm:.1f} mm / {r.correction_deg:.2f} deg over "
                       f"{cfg.max_correction_mm:g} / {cfg.max_correction_deg:g}")
        if r.fitness < cfg.min_fitness:
            why.append(f"fitness {r.fitness:.2f} < {cfg.min_fitness:g}")
        if r.max_penetration_mm > cfg.max_penetration_mm:
            why.append(f"overlaps a neighbour by {r.max_penetration_mm:.1f} mm")
        if why:
            r.accepted, r.reason = False, "; ".join(why)
    # relative pose of touching parts: only measured directions may change it beyond the limit
    for a in range(len(results)):
        for b in range(a + 1, len(results)):
            ra, rb = results[a], results[b]
            rel0 = np.linalg.inv(ra.T_saved) @ rb.T_saved
            rel1 = np.linalg.inv(ra.T) @ rb.T
            dmm, ddeg = pose_delta(rel0, rel1, parts[b].pts.mean(0))
            if dmm <= cfg.max_relative_mm and ddeg <= cfg.max_relative_deg:
                continue
            # each part's own correction in its centred (rho*w, t) coordinates - the ones
            # its information matrix (and so its weak directions) is expressed in
            for r, other in ((rb, ra), (ra, rb)):
                if r.weak_vectors is None or not len(r.weak_vectors):
                    continue
                part = parts[results.index(r)]
                c_loc = part.pts.mean(0)
                rho = float(np.percentile(np.linalg.norm(part.pts - c_loc, axis=1), 90))
                c0 = r.T_saved[:3, :3] @ c_loc + r.T_saved[:3, 3]
                c1 = r.T[:3, :3] @ c_loc + r.T[:3, 3]
                xi = np.concatenate([_rotvec(r.T[:3, :3] @ r.T_saved[:3, :3].T) * rho, c1 - c0])
                weak_part = float(np.linalg.norm(r.weak_vectors @ xi)) * 1000
                if weak_part > cfg.max_relative_mm and r.accepted:
                    r.accepted = False
                    r.reason = (f"its pose relative to {other.name} changed {dmm:.1f} mm / {ddeg:.2f} deg, "
                                f"{weak_part:.1f} mm of it along directions the views did not measure")
