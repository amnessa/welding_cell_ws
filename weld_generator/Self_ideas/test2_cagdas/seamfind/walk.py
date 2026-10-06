"""Stage 4 — sphere-tracing walkers, and the root point.

The walker is the plan's pseudocode, vectorised over all seeds at once. Three changes:
  * the step is alpha x the tangential component of the vector to B (alpha d |t|), not
    alpha d: at a root gap the full distance is longer than the in-plane distance to B's
    foot and the plan's step overshoots (the never-overshoot test fails without this);
  * the step is followed by a SUPPORT test (`PartSurface.project(..).on`): an MLS plane is
    infinite, so on a butt the walker would otherwise extrapolate A's top face straight
    across the gap onto B. A step whose foot leaves A's samples stops the walker at A's
    boundary - which is the toe.
  * theta_r from the step ratio assumes the walker converges to d = 0. With a root gap it
    converges to d = g, so the ratio is taken on (d_k - g_final); with fewer than two
    ratios theta_r is undefined (NaN) and the agreement test is skipped (a 90 deg joint
    converges in one or two steps at alpha = 0.9).
"""
from __future__ import annotations

import numpy as np

from .geom import unit


def walk(own, other, X0, alpha=0.9, eps=0.05, tol=0.01, k_max=50):
    """Walk points of `own` towards `other`. eps / tol in mm. Returns
    toe (N,3), d_final, n_steps, converged, d history; `walk.last_boundary` flags the
    walkers that stopped at own's patch boundary (an edge of the part)."""
    x, n, _ = own.project(X0)
    N = len(x)
    d_prev = np.full(N, np.inf)
    active = np.ones(N, bool)
    steps = np.zeros(N, int)
    hist = [np.full(N, np.nan) for _ in range(k_max + 1)]
    converged = np.zeros(N, bool)
    boundary = np.zeros(N, bool)          # stopped because the next step left own's samples
    d_last = np.full(N, np.nan)
    for k in range(k_max + 1):
        idx = np.flatnonzero(active)
        if not len(idx):
            break
        q, _, d = other.closest(x[idx])
        d_last[idx] = d
        hist[k][idx] = d
        stop = (d < eps) | (d_prev[idx] - d < tol)
        converged[idx[stop]] = True
        u = (q - x[idx]) / np.maximum(d, 1e-12)[:, None]
        nn = n[idx]
        t = u - (u * nn).sum(1, keepdims=True) * nn
        tl = np.linalg.norm(t, axis=1)
        at_toe = tl < 1e-3
        converged[idx[at_toe]] = True
        go = ~(stop | at_toe)
        if k == k_max:
            break
        gi = idx[go]
        if not len(gi):
            break
        # step alpha x the TANGENTIAL part of (q - x), i.e. alpha d |t| along t/|t|. The
        # plan's alpha d is safe only at zero gap: with B hovering g above A, d exceeds
        # the in-plane distance to B's foot and a 0.9 d step overshoots the toe.
        xn, nn_new, on = own.project(x[gi] + (alpha * d[go])[:, None] * t[go])
        # off the patch: the walker has reached own's boundary -> that IS the toe
        converged[gi[~on]] = True
        boundary[gi[~on]] = True
        moved = gi[on]
        x[moved], n[moved] = xn[on], nn_new[on]
        steps[moved] += 1
        d_prev[idx] = d
        active[:] = False
        active[moved] = True
    H = np.stack(hist, 1)
    walk.last_boundary = boundary
    return x, n, d_last, steps, converged, H


def theta_r(H, d_final, alpha):
    """Joint angle from the mean step ratio, gap-corrected; NaN with < 2 ratios."""
    E = H - d_final[:, None]
    with np.errstate(invalid="ignore", divide="ignore"):
        R = E[:, 1:] / E[:, :-1]
    ok = np.isfinite(R) & (E[:, :-1] > 1e-6) & (E[:, 1:] >= 0)
    rbar = np.where(ok.sum(1) >= 2, np.nanmean(np.where(ok, R, np.nan), 1), np.nan)
    return np.degrees(np.arcsin(np.clip((1 - rbar) / alpha, 0, 1)))


def root_point(toeA, nA, toeB, nB, s, coplanar):
    """Fillet: the point of (plane A at toe_A) ∩ (plane B at toe_B) closest to the seed
    midpoint s - SCHEMA D19's `nominal` curve, which is what the benchmark stores.
    Coplanar (butt / edge): the midpoint of the two toes - the gap centreline."""
    root = 0.5 * (toeA + toeB)
    f = ~coplanar
    if f.any():
        a, b = nA[f], nB[f]
        dvec = np.cross(a, b)
        dd = (dvec ** 2).sum(1)
        good = dd > 1e-6
        ha, hb = (a * toeA[f]).sum(1), (b * toeB[f]).sum(1)
        # point on the line: (ha (b x d) + hb (d x a)) / |d|^2
        p0 = (ha[:, None] * np.cross(b, dvec) + hb[:, None] * np.cross(dvec, a)) / np.maximum(dd, 1e-12)[:, None]
        dir_ = unit(dvec)
        p = p0 + ((s[f] - p0) * dir_).sum(1, keepdims=True) * dir_
        r = root[f]
        r[good] = p[good]
        root[f] = r
    return root


def crease_free_planes(own, other, toes, gap, radius, h):
    """The supporting plane of `own`'s joint face at each toe, fitted on own points within
    `radius` of the toe that lie farther than gap + 2h from `other`.

    The benchmark's truth is the intersection of the two EXTENDED supporting planes
    (SCHEMA D19 `nominal`), and the region next to the crease is exactly where the
    estimate is worst: at sub-millimetre gaps the tier-1 exterior scan keeps a strip of
    the terminating part's buried face (one or two samples wide, normals unrecoverable),
    and on thin plates the crease neighbourhood straddles three faces. Fitting away from
    the crease removes both; with too few points the walker's MLS plane is kept.
    Returns (centroid, normal, ok)."""
    nbrs = own.tree.query_ball_point(toes, radius)
    C = np.full((len(toes), 3), np.nan); Nn = np.full((len(toes), 3), np.nan)
    ok = np.zeros(len(toes), bool)
    for i, ids in enumerate(nbrs):
        if len(ids) < 8:
            continue
        ids = np.asarray(ids)
        far = other.tree.query(own.P[ids])[0] > gap[i] + 2.0 * h
        if far.sum() < 8:
            continue
        ids = ids[far]
        # one face only: the face of the far point nearest the toe (a thin plate puts both
        # of its broad faces, t apart, inside the ball - their joint fit is the mid-plane)
        ref = own.N[ids[np.argmin(np.linalg.norm(own.P[ids] - toes[i], axis=1))]]
        ids = ids[(own.N[ids] @ ref) > 0.5]
        if len(ids) < 8:
            continue
        Q = own.P[ids]
        c = Q.mean(0)
        _, V = np.linalg.eigh(np.cov((Q - c).T))
        n = V[:, 0]
        if n @ ref < 0:
            n = -n
        C[i], Nn[i], ok[i] = c, n, True
    return C, Nn, ok
