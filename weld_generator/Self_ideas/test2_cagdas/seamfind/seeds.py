"""Stage 3 — seeds: cross-part pairs that pick each other as nearest neighbour.

`seeding="fps"` is the plan as written: farthest-point samples (N = M = 1500) and the dense
N x M distance matrix. On the corpus that budget is too thin: a 200 x 266 mm base plate
at 1500 samples has ~10 mm between samples, more than tau_d, so most of a seam has no
pair at all. `seeding="dense"` (default) runs the SAME mutual-NN rule on every point of the
two downsampled clouds, with KD-trees in place of the matrix (argmin over a row of D is a
nearest-neighbour query, so the result is identical to an N x M matrix over all points).

Stage 3b (new, `coplanar_pairs`): coplanar boundary pairing for butt / edge joints. The
stored butt seam is the centreline between the two TOP faces (SCHEMA D19, generator
`_coplanar_candidate`). On an ISO 9692-1 groove those faces are up to ~25 mm apart, far
beyond tau_d, and their nearest neighbours are fusion-face points, so Stage 3 alone can
only ever find the groove root. 3b matches BOUNDARY points of A and B that are coplanar
(parallel normals, same plane within tol) mutually over `coplanar_tau_mm`.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from .geom import unit


def fps(P, n, seed=0):
    n = min(n, len(P))
    rng = np.random.default_rng(seed)
    sel = np.empty(n, int)
    sel[0] = rng.integers(len(P))
    d = np.linalg.norm(P - P[sel[0]], axis=1)
    for k in range(1, n):
        sel[k] = int(np.argmax(d))
        d = np.minimum(d, np.linalg.norm(P - P[sel[k]], axis=1))
    return sel


def mutual_nn(PA, PB, tau, tA=None, tB=None):
    """(i, j, D_ij) with j = argmin_j D[i, :], i = argmin_i D[:, j], D_ij < tau."""
    tA = tA or cKDTree(PA)
    tB = tB or cKDTree(PB)
    dAB, jB = tB.query(PA, distance_upper_bound=tau)
    ok = np.isfinite(dAB)
    i = np.flatnonzero(ok)
    j = jB[ok]
    _, iA = tA.query(PB[j])
    m = iA == i
    return i[m], j[m], dAB[ok][m]


def oneway_nn(PA, PB, tau, tA=None, tB=None):
    """Ablation (plan Stage 3): every A point within tau of B paired with its nearest B
    point, and every B point with its nearest A point - no reciprocity check."""
    tA = tA or cKDTree(PA)
    tB = tB or cKDTree(PB)
    dAB, jB = tB.query(PA, distance_upper_bound=tau)
    dBA, iA = tA.query(PB, distance_upper_bound=tau)
    a = np.flatnonzero(np.isfinite(dAB)); b = np.flatnonzero(np.isfinite(dBA))
    i = np.r_[a, iA[b]]; j = np.r_[jB[a], b]; d = np.r_[dAB[a], dBA[b]]
    key = np.unique(np.column_stack([i, j]), axis=0, return_index=True)[1]
    return i[key], j[key], d[key]


def mutual_nn_matrix(PA, PB, tau):
    """The plan's literal form: the dense N x M matrix (use only on FPS samples)."""
    D = np.linalg.norm(PA[:, None, :] - PB[None, :, :], axis=-1)
    j = D.argmin(1)
    i = D.argmin(0)
    a = np.arange(len(PA))
    m = (i[j] == a) & (D[a, j] < tau)
    return a[m], j[m], D[a[m], j[m]]


def boundary_mask(S, k=16, frac=0.35):
    """Patch-boundary points: the neighbourhood centroid is pushed off the point along the
    tangent plane by more than `frac` of the neighbourhood radius."""
    k = min(k, len(S.P))
    d, idx = S.tree.query(S.P, k=k)
    w = ((S.N[idx] * S.N[:, None, :]).sum(-1) > 0.5).astype(float)[..., None]   # same face only
    c = (S.P[idx] * w).sum(1) / np.maximum(w.sum(1), 1) - S.P
    c -= (c * S.N).sum(1, keepdims=True) * S.N
    return np.linalg.norm(c, axis=1) > frac * np.maximum(d[:, -1], 1e-9)


def coplanar_pairs(SA, SB, cand_A, cand_B, tau, cop_deg, plane_tol):
    """Mutual NN among coplanar boundary candidates (Stage 3b)."""
    if not len(cand_A) or not len(cand_B):
        return np.zeros(0, int), np.zeros(0, int), np.zeros(0)
    PA, PB = SA.P[cand_A], SB.P[cand_B]
    i, j, d = mutual_nn(PA, PB, tau)
    nA, nB = SA.N[cand_A][i], SB.N[cand_B][j]
    par = (nA * nB).sum(1) > np.cos(np.radians(cop_deg))
    off = np.abs(((PB[j] - PA[i]) * unit(nA + nB)).sum(1)) < plane_tol
    m = par & off
    return cand_A[i[m]], cand_B[j[m]], d[m]


def theta_n(nA, nB):
    return np.degrees(np.arccos(np.clip((nA * nB).sum(1), -1, 1)))


def joint_class(th, cop_deg=20.0, facing_deg=150.0, offset=None, plane_tol=None):
    """Per-seed class from the normal angle: 'coplanar' (butt / edge), 'fillet' (T, corner,
    lap toe - theta_n alone cannot tell them apart), 'facing' (dropped).

    With `offset` (|(b - a) . n_mean| per pair) a parallel pair counts as coplanar only
    when both points lie in ONE plane (generator `coplanar_tol_mm`); parallel faces offset
    by a plate thickness - the two bottom faces of a lap, both broad faces of a stack -
    are 'parallel' and dropped like facing ones: no seam lies between them."""
    out = np.full(len(th), "fillet", dtype=object)
    out[th < cop_deg] = "coplanar"
    if offset is not None:
        out[(th < cop_deg) & (offset > plane_tol)] = "parallel"
    out[th > facing_deg] = "facing"
    return out
