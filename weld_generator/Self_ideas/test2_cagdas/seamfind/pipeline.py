"""The seam finder end to end: two labelled clouds in, seams with toes, gap, angle, joint
class and torch feasibility out. Stages follow `seam_finder.md`; stage 6 (rolling-ball
spine) is not implemented (optional in the plan, M9)."""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from . import access, assemble, seeds, walk
from .config import Params
from .geom import PartSurface, estimate_spacing, remove_outliers, unit, voxel_downsample


@dataclass
class Result:
    seams: list
    seeds: dict
    h: float
    timings: dict = field(default_factory=dict)
    counts: dict = field(default_factory=dict)


def _prep_part(P, Nhint, h, prm, cam_pos):
    P, Nh = voxel_downsample(np.asarray(P, float), h, Nhint) if Nhint is not None \
        else (voxel_downsample(np.asarray(P, float), h)[0], None)
    if prm.remove_outliers:
        m = remove_outliers(P, prm.outlier_k, prm.outlier_std)
        P = P[m]
        Nh = Nh[m] if Nh is not None else None
    return PartSurface(P, h, k=prm.k_normals, orient=Nh, cam_pos=cam_pos,
                       r_mls_h=prm.r_mls_h, gate=prm.mls_normal_gate, support_h=prm.support_h)


def extract(A, B, params: Params | None = None, normals_A=None, normals_B=None,
            cam_pos=None, gap_hint_mm=None) -> Result:
    """A, B: (n,3) labelled clouds in one frame (mm). `normals_*`: optional per-point
    normals used ONLY for the sign of the PCA normals (the plan's synthetic orientation
    rule); without them the normals face `cam_pos` (the real-scan rule)."""
    prm = params or Params()
    T, tick = {}, [time.time()]

    def lap(name):
        now = time.time(); T[name] = round(now - tick[0], 3); tick[0] = now

    if min(len(A), len(B)) < max(prm.k_normals, 10):     # a part out of view: nothing to pair
        return Result([], {}, float("nan"), T, {"n_A": len(A), "n_B": len(B), "seeds": 0})
    hA = prm.h_mm or estimate_spacing(np.asarray(A, float))
    hB = prm.h_mm or estimate_spacing(np.asarray(B, float))
    h = max(hA, hB)
    SA = _prep_part(A, normals_A, hA, prm, cam_pos)
    SB = _prep_part(B, normals_B, hB, prm, cam_pos)
    lap("stage1")

    gap_max = prm.gap_max_mm if gap_hint_mm is None else float(gap_hint_mm)
    tau = gap_max + prm.tau_d_extra_h * h
    # every point that can be in a kept pair is within tau of the other part: filtering
    # those before matching is exactly filtering all points (stage 2a's job)
    dA = SB.tree.query(SA.P, distance_upper_bound=tau)[0]
    dB = SA.tree.query(SB.P, distance_upper_bound=tau)[0]
    candA, candB = np.flatnonzero(np.isfinite(dA)), np.flatnonzero(np.isfinite(dB))
    bA, bB = seeds.boundary_mask(SA), seeds.boundary_mask(SB)
    if prm.coplanar_pairs:
        dA2 = SB.tree.query(SA.P, distance_upper_bound=prm.coplanar_tau_mm)[0]
        dB2 = SA.tree.query(SB.P, distance_upper_bound=prm.coplanar_tau_mm)[0]
        copA = np.flatnonzero(np.isfinite(dA2) & bA)
        copB = np.flatnonzero(np.isfinite(dB2) & bB)
    else:
        copA = copB = np.zeros(0, int)
    lap("bands")

    keepA = np.ones(len(SA.P), bool); keepB = np.ones(len(SB.P), bool)
    if prm.prefilter:
        merged = np.vstack([SA.P, SB.P]); mt = cKDTree(merged)
        for S, keep, cand in ((SA, keepA, np.union1d(candA, copA)), (SB, keepB, np.union1d(candB, copB))):
            if len(cand):
                f = access.free_ray_fraction(S.P[cand], S.N[cand], mt, h, prm.n_rays,
                                             prm.cone_half_deg, prm.ray_start_h, prm.ray_hit_h,
                                             prm.ray_free_mm, prm.ray_max_steps, cloud=merged)
                keep[cand[f < prm.tau_vis]] = False
    candA, candB = candA[keepA[candA]], candB[keepB[candB]]
    copA, copB = copA[keepA[copA]], copB[keepB[copB]]
    lap("stage2a")

    if prm.seeding == "fps":
        sa = candA[seeds.fps(SA.P[candA], prm.n_fps)] if len(candA) else candA
        sb = candB[seeds.fps(SB.P[candB], prm.n_fps)] if len(candB) else candB
        i, j, _ = seeds.mutual_nn_matrix(SA.P[sa], SB.P[sb], tau) if len(sa) and len(sb) else ([], [], [])
        ia, jb = sa[np.asarray(i, int)], sb[np.asarray(j, int)]
    elif prm.seeding == "oneway":
        i, j, _ = seeds.oneway_nn(SA.P[candA], SB.P[candB], tau) if len(candA) and len(candB) \
            else (np.zeros(0, int), np.zeros(0, int), None)
        ia, jb = candA[i], candB[j]
    else:
        i, j, _ = seeds.mutual_nn(SA.P[candA], SB.P[candB], tau) if len(candA) and len(candB) \
            else (np.zeros(0, int), np.zeros(0, int), None)
        ia, jb = candA[i], candB[j]
    src = np.zeros(len(ia), int)
    if prm.coplanar_pairs and len(copA) and len(copB):
        ci, cj, _ = seeds.coplanar_pairs(SA, SB, copA, copB, prm.coplanar_tau_mm,
                                         prm.coplanar_deg, max(prm.coplanar_tol_mm, 0.5 * h))
        new = ~np.isin(ci, ia)
        ia, jb = np.r_[ia, ci[new]], np.r_[jb, cj[new]]
        src = np.r_[src, np.ones(new.sum(), int)]
    th = seeds.theta_n(SA.N[ia], SB.N[jb])
    plane_tol = max(prm.coplanar_tol_mm, 0.5 * h)
    off = np.abs(((SB.P[jb] - SA.P[ia]) * unit(SA.N[ia] + SB.N[jb])).sum(1))
    cls = seeds.joint_class(th, prm.coplanar_deg, prm.facing_deg, off, plane_tol)
    keep = (cls != "facing") & (cls != "parallel")
    ia, jb, th, cls, src = ia[keep], jb[keep], th[keep], cls[keep], src[keep]
    lap("stage3")
    counts = {"n_A": len(SA.P), "n_B": len(SB.P), "band_A": len(candA), "band_B": len(candB),
              "seeds": int(len(ia)), "seeds_coplanar_3b": int(src.sum())}
    if not len(ia):
        return Result([], {}, h, T, counts)

    eps, tol = prm.eps_h * h, prm.tol_h * h
    tA, nA, dfa, stA, cvA, HA = walk.walk(SA, SB, SA.P[ia], prm.alpha, eps, tol, prm.k_max)
    bdA = walk.walk.last_boundary.copy()
    tB, nB, dfb, stB, cvB, HB = walk.walk(SB, SA, SB.P[jb], prm.alpha, eps, tol, prm.k_max)
    bdB = walk.walk.last_boundary.copy()
    thr = np.nanmean(np.stack([walk.theta_r(HA, dfa, prm.alpha), walk.theta_r(HB, dfb, prm.alpha)]), 0)
    s_mid = 0.5 * (SA.P[ia] + SB.P[jb])
    gap = np.linalg.norm(tA - tB, axis=1)
    if prm.crease_free:
        R_fit = prm.crease_fit_h * h
        cA, fA, okA = walk.crease_free_planes(SA, SB, tA, gap, R_fit, h)
        cB, fB, okB = walk.crease_free_planes(SB, SA, tB, gap, R_fit, h)
        if prm.crease_free == "interior":
            # a toe at own's part edge (corner, lap, butt plates) sits where two of own's
            # faces meet - the walker's MLS plane there is the joint face's, the far-field
            # fit may pick the other one; refit only toes that stopped inside the patch
            okA &= ~bdA; okB &= ~bdB
        # the toe moves onto the fitted plane (its position is the walker's, its plane the fit's)
        nA = np.where(okA[:, None], fA, nA); nB = np.where(okB[:, None], fB, nB)
        tA = np.where(okA[:, None], tA - ((tA - cA) * fA).sum(1, keepdims=True) * fA, tA)
        tB = np.where(okB[:, None], tB - ((tB - cB) * fB).sum(1, keepdims=True) * fB, tB)
        th2 = seeds.theta_n(nA, nB)
        off2 = np.abs(((tB - tA) * unit(nA + nB)).sum(1))
        cls = np.where(cls == "coplanar", cls, seeds.joint_class(th2, prm.coplanar_deg, prm.facing_deg, off2, plane_tol))
    cop = cls == "coplanar"
    root = walk.root_point(tA, nA, tB, nB, s_mid, cop)
    th_w = seeds.theta_n(nA, nB)
    lap("stage4")

    # stage 5.1: convergence + angle agreement (theta_r is defined in [0, 90])
    folded = np.minimum(th_w, 180 - th_w)
    disagree = prm.angle_check & np.isfinite(thr) & ~cop & (np.abs(folded - thr) > prm.angle_disagree_deg)
    far = np.linalg.norm(root - s_mid, axis=1) > tau + 2 * h      # root ran off along the line
    ok = cvA & cvB & ~disagree & ~far & (cls != "facing") & (cls != "parallel")
    counts.update(seeds_converged=int((cvA & cvB).sum()), seeds_angle_drop=int(disagree.sum()),
                  seeds_kept=int(ok.sum()))
    sd = dict(root=root, toeA=tA, toeB=tB, nA=nA, nB=nB, gap=gap, theta_n=th_w, theta_r=thr,
              cls=cls, ok=ok, stepsA=stA, stepsB=stB, src=src)
    R, nA_, nB_, cl_ = root[ok], nA[ok], nB[ok], cls[ok]
    if len(R) < prm.dbscan_min:
        return Result([], sd, h, T, counts)
    # seeds come out ~one per 2-3 h along a seam (mutual pairs are sparser than samples),
    # so the plan's eps = 4h is tied to the measured seed spacing as well
    sp = float(np.median(cKDTree(R).query(R, k=2)[0][:, 1]))
    eps_c = max(prm.dbscan_eps_h * h, prm.dbscan_eps_spacing * sp)
    counts["seed_spacing_mm"] = round(sp, 3)
    lab = assemble.cluster(R, nA_, nB_, eps_c, prm.dbscan_min, prm.normal_eps)
    seams = []
    allP = np.vstack([SA.P, SB.P]); allT = cKDTree(allP)
    sel_idx = np.flatnonzero(ok)
    for c in np.unique(lab[lab >= 0]):
        m = lab == c
        P = R[m]
        P, groups = assemble.order_cluster(P, max(h, eps_c / 3.0))
        sid = sel_idx[m]
        ii = np.array([sid[g][0] for g in groups], int) if len(groups) else np.zeros(0, int)
        if len(P) < 4:
            continue
        length = np.linalg.norm(np.diff(P, axis=0), axis=1).sum()
        if length < prm.min_seam_mm:
            continue
        closed = np.linalg.norm(P[0] - P[-1]) < prm.close_h * h and length > 12 * h
        sig = max(assemble.smooth_sigma(P), 0.05 * h)
        try:
            pts, tan, L = assemble.fit_spline(P, closed, sig, prm.ds_mm)
        except Exception:
            continue
        _, nn = cKDTree(R[m]).query(pts)
        g = sel_idx[m][nn]
        bis = unit(nA[g] + nB[g])
        jc = "coplanar" if np.mean(cls[g] == "coplanar") > 0.5 else "fillet"
        dihedral = 180.0 - th_w[g]
        s = {"points": pts, "tangent": tan, "length": L, "closed": bool(closed),
             "toe_A": tA[g], "toe_B": tB[g], "gap": gap[g], "theta_n": th_w[g],
             "theta_r": thr[g], "dihedral": dihedral, "joint_class": jc,
             "torch_axis": -bis, "nA": nA[g], "nB": nB[g], "n_seeds": int(m.sum())}
        seams.append(s)
    if prm.cross_runs:
        n0 = len(seams)
        seams = assemble.drop_cross_runs(seams, prm.cross_run_tol_deg)
        counts["cross_runs_dropped"] = n0 - len(seams)
    if prm.suppress_toes:
        n0 = len(seams)
        seams = assemble.suppress_toes(seams, h)
        counts["toes_suppressed"] = n0 - len(seams)
    lap("stage5")
    if prm.torch_check:
        for s in seams:
            frac, _ = access.torch_feasibility(s["points"], s["tangent"], -s["torch_axis"], allT, allP, h,
                                               prm.torch_half_deg, prm.torch_standoff_mm, prm.max_work_deg,
                                               prm.work_step_deg, prm.travel_deg)
            dh = s["dihedral"]
            ok_d = (s["joint_class"] == "coplanar") | ((dh >= prm.dihedral_min_deg) & (dh <= prm.dihedral_max_deg))
            conf = access.confined(s["points"], (s["nA"], s["nB"]), allT, h, prm.bore_min_diameter_mm, cloud=allP) \
                if prm.bore_check else np.zeros(len(s["points"]), bool)
            s["torch_feasible_frac"] = frac
            s["confined"] = conf
            s["weldable"] = (frac > 0) & ok_d & ~conf
    lap("stage2c")
    seams.sort(key=lambda s: -s["length"])
    if prm.output == "weldable" and prm.torch_check:
        seams = weldable_spans(seams, prm.min_seam_mm)
    return Result(seams, sd, h, T, counts)


def weldable_spans(seams, min_len):
    """Split each seam into its weldable runs (plan Stage 5 step 7); runs shorter than the
    generator's minimum seam length are dropped."""
    out = []
    for s in seams:
        w = np.asarray(s["weldable"], bool)
        if w.all():
            out.append(s); continue
        n = len(w)
        edges = np.flatnonzero(np.diff(np.r_[0, w.astype(int), 0]))
        for a, b in zip(edges[::2], edges[1::2]):
            idx = np.arange(a, b)
            if len(idx) < 2:
                continue
            P = s["points"][idx]
            L = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
            if L < min_len:
                continue
            q = {k: (v[idx] if isinstance(v, np.ndarray) and len(v) == n else v) for k, v in s.items()}
            q["length"], q["closed"] = L, False
            out.append(q)
    return out
