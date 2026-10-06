"""Stage 5 — seam assembly: filter, cluster, order (MST longest path), spline, frames.

Additions for the corpus: (a) toe suppression - a fillet cluster that runs along a
coplanar cluster within the gap distance is the TOE of that butt / edge weld, not a seam
of its own (ISO 17659 class rule, generator `_suppress_toes`; the benchmark keeps those
lines as negatives); (b) clusters shorter than the generator's `min_seam_length_mm`
(10 mm) are dropped; (c) the spline smoothing sigma is estimated from the ordered root
points themselves (residual to a moving average), the plan's "from noise sigma".
"""
from __future__ import annotations

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra, minimum_spanning_tree
from scipy.spatial import cKDTree

from .geom import unit


def cluster(root, nA, nB, eps, min_samples, normal_eps=0.35):
    """Plan step 2, literally: DBSCAN on the root POSITIONS, then split each cluster where
    the seed normals jump (the two sides of a T sit one plate thickness apart - 1 mm at
    the thin end of ISO 9692-1 - so position alone merges them; their n_B are opposite).
    The split is a second DBSCAN on the unit normal pair (eps 0.35 ~ 20 deg); seeds whose
    normals fit no group (thin-sheet crease points, see geom.py) become noise."""
    from sklearn.cluster import DBSCAN
    lab = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(root)
    out = np.full(len(root), -1)
    nxt = 0
    for c in np.unique(lab[lab >= 0]):
        m = np.flatnonzero(lab == c)
        sub = DBSCAN(eps=normal_eps, min_samples=min(min_samples, 3)).fit_predict(np.hstack([nA[m], nB[m]]))
        for k in np.unique(sub[sub >= 0]):
            mm = m[sub == k]
            # a normal group can be split along the seam by a gap: re-run positions
            pl = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(root[mm])
            for q in np.unique(pl[pl >= 0]):
                out[mm[pl == q]] = nxt
                nxt += 1
    return out


def longest_path_order(P, r):
    n = len(P)
    pairs = cKDTree(P).query_pairs(r, output_type="ndarray")
    if len(pairs) == 0:
        return np.arange(min(n, 1))
    w = np.linalg.norm(P[pairs[:, 0]] - P[pairs[:, 1]], axis=1) + 1e-9
    G = coo_matrix((w, (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    _, lab = connected_components(G, directed=False)
    big = np.argmax(np.bincount(lab))
    T = minimum_spanning_tree(G).tocsr()
    start = np.flatnonzero(lab == big)[0]
    d0 = dijkstra(T, directed=False, indices=start)
    a = int(np.argmax(np.where(np.isfinite(d0), d0, -1)))
    da, pred = dijkstra(T, directed=False, indices=a, return_predecessors=True)
    b = int(np.argmax(np.where(np.isfinite(da), da, -1)))
    path = [b]
    while path[-1] != a:
        path.append(pred[path[-1]])
    return np.array(path[::-1])


def order_cluster(P, h):
    """MST longest path through the cluster; every point is then projected onto the path
    (plan step 3) by assigning it to its nearest path vertex, and each vertex is replaced
    by the mean of the points assigned to it. One-way seeds form a band a few seeds wide;
    sorting the raw band by arclength would interleave its two edges into a zig-zag.
    Returns the ordered curve points and, per point, the indices of the seeds it averages."""
    path = longest_path_order(P, 3.0 * h)
    if len(path) < 2:
        return P[:0], []
    V = P[path]
    _, j = cKDTree(V).query(P)
    groups = [np.flatnonzero(j == k) for k in range(len(V))]
    keep = [k for k in range(len(V)) if len(groups[k])]
    C = np.array([P[groups[k]].mean(0) for k in keep])
    return C, [groups[k] for k in keep]


def smooth_sigma(P, k=5):
    if len(P) < 2 * k + 1:
        return 0.0
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    M = np.column_stack([np.convolve(P[:, j], ker, mode="valid") for j in range(3)])
    return float(np.sqrt(((P[k:-k] - M) ** 2).sum(1).mean()))


def fit_spline(P, closed, sigma, ds):
    keep = np.r_[True, np.linalg.norm(np.diff(P, axis=0), axis=1) > 1e-9]
    P = P[keep]
    if closed:
        P = np.vstack([P, P[:1]])
    m = len(P)
    k = 3 if m > 5 else 1
    tck, _ = splprep(P.T, s=m * sigma ** 2, per=1 if closed else 0, k=k)
    u = np.linspace(0, 1, max(20 * m, 400))
    dense = np.array(splev(u, tck)).T
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    n = max(int(round(s[-1] / ds)), 2)
    us = np.interp(np.linspace(0, s[-1], n, endpoint=not closed), s, u)
    return np.array(splev(us, tck)).T, unit(np.array(splev(us, tck, der=1)).T), float(s[-1])


def suppress_toes(seams, h, frac=0.6):
    """Drop fillet seams that run alongside a coplanar seam within its toe distance
    (half the coplanar seam's median gap + 2h): those are the toes that bound a butt /
    edge gap (ISO 17659; generator `toe_of_centreline`), not seams of their own."""
    cop = [s for s in seams if s["joint_class"] == "coplanar"]
    if not cop:
        return seams
    Ct = cKDTree(np.vstack([s["points"] for s in cop]))
    rad = float(np.nanmedian(np.concatenate([s["gap"] for s in cop]))) / 2 + 2 * h
    return [s for s in seams
            if not (s["joint_class"] == "fillet" and (Ct.query(s["points"])[0] < rad).mean() > frac)]


def drop_cross_runs(seams, tol_deg=45.0, straight=0.9, candidate=0.6):
    """Generator rule (SCHEMA 2.6.2, `_drop_cross_runs`): a joint has a direction - the
    principal axis of the straight seams' chords, each weighted by its length - and a
    roughly straight seam more than `tol_deg` off it is a run ACROSS the plate at an end,
    not a seam. Voters: chord / length > `straight`; candidates for dropping: > `candidate`.
    Closed rings and arcs below the candidate ratio are never dropped."""
    def chord(s):
        P = s["points"]
        c = P[-1] - P[0]
        L = float(np.linalg.norm(c))
        return c / max(L, 1e-9), L
    voters = [s for s in seams if not s["closed"] and chord(s)[1] > straight * s["length"]]
    if not voters:
        return seams
    T = np.array([chord(s)[0] for s in voters]); W = np.array([s["length"] for s in voters])
    dom = np.linalg.eigh((T * W[:, None]).T @ T)[1][:, -1]
    cos_t = np.cos(np.radians(tol_deg))
    # a short end run wobbles (chord / length ~0.7), so candidates are looser than voters
    cands = [s for s in seams if not s["closed"] and chord(s)[1] > candidate * s["length"]]
    drop = {id(s) for s in cands if abs(chord(s)[0] @ dom) < cos_t}
    return [s for s in seams if id(s) not in drop]
