#!/usr/bin/env python3
"""
weld_seam.py
============
Footprint-based weld seam extraction with rolling-ball path planning.

Inputs
  --base      point cloud of the base part alone (part A)
  --combined  point cloud of the assembly in weld position (A + B)

Pipeline
  1. load, voxel-downsample, optional outlier removal
  2. optional ICP registration of the base scan into the assembly
  3. segment the assembly into A / B   (B = what the base scan does not explain)
  4. distance fields: each part's bounded point-to-surface distance to the other
  5. footprints  F_A = {p in A : D_A(p) < g},  F_B likewise
  6. direction vectors = surface gradient of the distance field; every footprint
     point slides along -grad until the distance reaches the fit-up gap.
     (Footprint boundary points are a subset; use --boundary-only to restrict.)
  7. exposure filter (both parts visible nearby) + clustering into seams
  8. thin, order (MST longest path), smoothing spline, resample
  9. joint frame per path point from leg directions d_A, d_B -> angle, torch axis
 10. rolling ball in each cross-section -> toes, leg lengths, ball centre, bead area
 11. outputs: CSV / JSON / PLY + PNG plots (+ ground-truth metrics with --gt)

Dependencies: numpy, scipy, matplotlib (open3d optional, only for reading .pcd)
All thresholds default to multiples of the voxel size v, so they scale with data.
"""
import argparse
import io
import json
import os
import sys
import time

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra, minimum_spanning_tree
from scipy.spatial import cKDTree

# ----------------------------------------------------------------------------- IO
_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def read_ply(path):
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError(f"{path}: not a PLY file")
        fmt, elements = None, []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: truncated header")
            t = line.decode("ascii", "ignore").split()
            if not t:
                continue
            if t[0] == "format":
                fmt = t[1]
            elif t[0] == "element":
                elements.append([t[1], int(t[2]), []])
            elif t[0] == "property":
                if t[1] == "list":
                    elements[-1][2].append((t[4], None))
                else:
                    elements[-1][2].append((t[2], _PLY_TYPES[t[1]]))
            elif t[0] == "end_header":
                break
        name, n, props = elements[0]
        if name != "vertex" or any(p[1] is None for p in props):
            raise ValueError(f"{path}: vertex element must come first with scalar properties")
        names = [p[0] for p in props]
        if fmt == "ascii":
            txt = b"".join(f.readline() for _ in range(n)).decode()
            data = np.loadtxt(io.StringIO(txt), ndmin=2)
            cols = [data[:, names.index(c)] for c in ("x", "y", "z")]
        else:
            end = "<" if fmt == "binary_little_endian" else ">"
            dt = np.dtype([(p[0], end + p[1]) for p in props])
            arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
            cols = [arr[c] for c in ("x", "y", "z")]
    return np.column_stack(cols).astype(np.float64)


def write_ply(path, pts, colors=None):
    pts = np.asarray(pts, np.float32)
    n = len(pts)
    header = ["ply", "format binary_little_endian 1.0", f"element vertex {n}",
              "property float x", "property float y", "property float z"]
    if colors is not None:
        header += ["property uchar red", "property uchar green", "property uchar blue"]
    header.append("end_header")
    if colors is None:
        rec = pts
    else:
        dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                       ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec = np.empty(n, dt)
        rec["x"], rec["y"], rec["z"] = pts.T
        c = np.asarray(colors, np.uint8)
        rec["r"], rec["g"], rec["b"] = c.T
    with open(path, "wb") as f:
        f.write(("\n".join(header) + "\n").encode())
        f.write(rec.tobytes())


def load_points(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".ply":
        return read_ply(path)
    if ext == ".pcd":
        try:
            import open3d as o3d
        except ImportError:
            raise SystemExit("Reading .pcd needs open3d (pip install open3d) - or convert to .ply/.xyz")
        return np.asarray(o3d.io.read_point_cloud(path).points, np.float64)
    if ext == ".npy":
        return np.load(path)[:, :3].astype(np.float64)
    if ext == ".npz":
        z = np.load(path)
        return z[z.files[0]][:, :3].astype(np.float64)
    for delim in (None, ",", ";"):           # .xyz / .txt / .csv / .pts
        for skip in (0, 1):
            try:
                return np.loadtxt(path, delimiter=delim, skiprows=skip, ndmin=2)[:, :3]
            except ValueError:
                continue
    raise SystemExit(f"Could not parse point file {path}")


# ----------------------------------------------------------------- geometry utils
def unit(v, eps=1e-12):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, eps)


def estimate_spacing(P, n_sample=5000, seed=0):
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), min(n_sample, len(P)), replace=False)]
    d, _ = cKDTree(P).query(S, k=5)
    # 4th neighbour: robust to noise-induced near-duplicates (on a grid it equals the pitch)
    return float(np.median(d[:, 4]))


def voxel_downsample(P, v):
    keys = np.floor(P / v).astype(np.int64)
    _, inv, cnt = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    out = np.column_stack([np.bincount(inv, P[:, j]) for j in range(3)])
    return out / cnt[:, None]


def remove_outliers(P, k=12, n_std=3.0):
    d, _ = cKDTree(P).query(P, k=k + 1)
    m = d[:, 1:].mean(1)
    return P[m < m.mean() + n_std * m.std()]


def pca_normals(P, k=16):
    """Unoriented PCA normals. Returns normals, kNN index array and the tree."""
    tree = cKDTree(P)
    _, idx = tree.query(P, k=k)
    nb = P[idx] - P[idx].mean(1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / k
    _, V = np.linalg.eigh(cov)
    return V[:, :, 0], idx, tree


def radius_components(P, eps):
    n = len(P)
    if n == 0:
        return np.zeros(0, int)
    pairs = cKDTree(P).query_pairs(eps, output_type="ndarray")
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    return connected_components(g, directed=False)[1]


def bounded_distance(X, Q, nQ, treeQ, r_s, k=4):
    """Point-to-surface distance: point-to-plane inside the local support radius r_s
    of the surface sample, growing like point-to-point outside it. This prevents a
    coplanar neighbour (butt joint) from reporting zero distance everywhere."""
    k = min(k, len(Q))
    _, i = treeQ.query(X, k=k)
    i = i.reshape(len(X), k)
    diff = X[:, None, :] - Q[i]
    dp = np.abs(np.einsum("nkj,nkj->nk", diff, nQ[i]))
    lat = np.sqrt(np.maximum((diff ** 2).sum(-1) - dp ** 2, 0.0))
    return np.sqrt(dp ** 2 + np.maximum(lat - r_s, 0.0) ** 2).min(1)


def tangent_basis(n):
    a = np.where(np.abs(n[:, :1]) < 0.9, [[1.0, 0, 0]], [[0, 1.0, 0]])
    e1 = unit(np.cross(n, a))
    return e1, np.cross(n, e1)


def surface_gradient(P, normals, vals, idx, scale):
    """Least-squares gradient of a scalar field restricted to the local tangent plane."""
    nb = P[idx] - P[:, None, :]
    e1, e2 = tangent_basis(normals)
    u = np.einsum("nkj,nj->nk", nb, e1) / scale
    v = np.einsum("nkj,nj->nk", nb, e2) / scale
    M = np.stack([np.ones_like(u), u, v], -1)
    MtM = np.einsum("nki,nkj->nij", M, M) + 1e-6 * np.eye(3)
    Mty = np.einsum("nki,nk->ni", M, vals[idx])
    sol = np.linalg.solve(MtM, Mty[..., None])[..., 0] / scale
    return sol[:, 1:2] * e1 + sol[:, 2:3] * e2


def snap_to_surface(X, P, normals, tree):
    _, i = tree.query(X)
    d = ((X - P[i]) * normals[i]).sum(1, keepdims=True)
    return X - d * normals[i]


def rodrigues(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def icp_point_to_plane(src, src_n, dst, dst_n, max_dist, min_dist, iters=60):
    """Point-to-plane ICP from the identity with a shrinking correspondence radius,
    normal-compatibility gating and Huber weights. No percentile trimming: on flat
    plates the few side-face matches are exactly what constrains in-plane sliding.
    Needs a rough initial alignment; use a global method (e.g. FPFH+RANSAC) first
    on real data with large offsets."""
    tree = cKDTree(dst)
    T, cur, cur_n, rmse = np.eye(4), src.copy(), src_n.copy(), np.inf
    for it in range(iters):
        md = max(min_dist, max_dist * (1 - it / (0.6 * iters)))
        d, i = tree.query(cur, distance_upper_bound=md)
        m = np.isfinite(d)
        m[m] &= np.abs((cur_n[m] * dst_n[i[m]]).sum(1)) > 0.8
        if m.sum() < 30:
            break
        p, q, n = cur[m], dst[i[m]], dst_n[i[m]]
        r = ((p - q) * n).sum(1)
        delta = 0.5 * min_dist
        w = np.sqrt(np.minimum(1.0, delta / np.maximum(np.abs(r), 1e-12)))
        A = np.hstack([np.cross(p, n), n]) * w[:, None]
        # damped normal equations: directions the geometry cannot observe (sliding
        # along a pipe axis, in-plane on a featureless plate) stay put instead of drifting
        H = A.T @ A
        x = np.linalg.solve(H + 1e-4 * np.trace(H) / 6 * np.eye(6), A.T @ (-r * w))
        R, t = rodrigues(x[:3]), x[3:]
        cur = cur @ R.T + t
        cur_n = cur_n @ R.T
        step = np.eye(4)
        step[:3, :3], step[:3, 3] = R, t
        T = step @ T
        rmse = float(np.sqrt((r ** 2).mean()))
        if np.linalg.norm(x) < 1e-9 and md <= min_dist:
            break
    return T, cur, rmse


# ------------------------------------------------------------ pipeline pieces
def segment_assembly(C, A, nA, treeA, v, seg_dist, cos_thr, min_comp):
    """Label assembly points that the base scan explains as A; the rest is B."""
    nC, _, _ = pca_normals(C, k=12)
    d, i = treeA.query(C)
    agree = np.abs((nC * nA[i]).sum(1)) >= cos_thr
    isA = ((d < seg_dist) & agree) | (d < 0.5 * seg_dist)
    CB = C[~isA]
    lab = radius_components(CB, 2.5 * v)
    if len(lab):
        sizes = np.bincount(lab)
        CB = CB[sizes[lab] >= min_comp]
    return C[isA], CB


def project_to_root(P0, dirs, gmag, D0, target, other, own, r_s, max_step, tol, iters=5):
    """Slide points along -dirs (unit, tangent to own surface) until the distance to
    the other part equals `target`. Secant iterations, re-snapped to own surface."""
    Q, nQ, tQ = other
    Po, no, to = own
    f_prev = D0 - target
    t_prev = np.zeros(len(P0))
    t = np.clip(f_prev / np.maximum(gmag, 1e-3), -max_step, max_step)
    for _ in range(iters):
        X = snap_to_surface(P0 - t[:, None] * dirs, Po, no, to)
        f = bounded_distance(X, Q, nQ, tQ, r_s) - target
        den = f - f_prev
        ok = np.abs(den) > 1e-9
        t_new = np.where(ok, t - f * (t - t_prev) / np.where(ok, den, 1.0), t)
        t_prev, f_prev = t, f
        t = np.clip(t_new, -max_step, max_step)
    X = snap_to_surface(P0 - t[:, None] * dirs, Po, no, to)
    f = bounded_distance(X, Q, nQ, tQ, r_s) - target
    valid = (np.abs(t) < max_step) & (np.abs(f) < tol)
    return X, valid


def leg_direction(X, Q, treeQ, R, k=64):
    """Direction from X towards the centroid of the visible points of one part within
    radius R: the 'leg' of the joint on that part (footprint -> non-footprint)."""
    if len(Q) == 0:
        return np.zeros_like(X), np.zeros(len(X), bool)
    k = min(k, len(Q))
    d, i = treeQ.query(X, k=k, distance_upper_bound=R)
    d, i = d.reshape(len(X), k), i.reshape(len(X), k)
    valid = np.isfinite(d)
    w = valid.astype(float)
    pts = Q[np.where(valid, i, 0)]
    cnt = w.sum(1)
    cen = (pts * w[..., None]).sum(1) / np.maximum(cnt, 1)[:, None]
    vec = cen - X
    nrm = np.linalg.norm(vec, axis=1)
    ok = (cnt >= 5) & (nrm > 0.1 * R)
    return unit(vec), ok


def thin_curve(P, r, iters=3):
    """Contract a band of points onto its centre line, moving only perpendicular to the
    local principal direction (so seam ends do not shrink)."""
    for _ in range(iters):
        k = min(16, len(P))
        d, i = cKDTree(P).query(P, k=k, distance_upper_bound=r)
        valid = np.isfinite(d)
        w = valid.astype(float)[..., None]
        nb = P[np.where(valid, i, 0)]
        cnt = np.maximum(w.sum(1), 1)
        cen = (nb * w).sum(1) / cnt
        c = (nb - cen[:, None]) * w
        _, V = np.linalg.eigh(np.einsum("nki,nkj->nij", c, c))
        e = V[:, :, 2]
        disp = cen - P
        disp -= (disp * e).sum(1, keepdims=True) * e
        P = P + disp
    return P


def order_points(P, r):
    """Order an unordered curve sample via the longest path of its MST."""
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


def fit_spline(Pord, closed, sigma, ds):
    keep = np.r_[True, np.linalg.norm(np.diff(Pord, axis=0), axis=1) > 1e-9]
    Pord = Pord[keep]
    if closed:
        Pord = np.vstack([Pord, Pord[:1]])
    m = len(Pord)
    k = 3 if m > 5 else 1
    tck, _ = splprep(Pord.T, s=m * sigma ** 2, per=1 if closed else 0, k=k)
    u = np.linspace(0, 1, max(20 * m, 400))
    dense = np.array(splev(u, tck)).T
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    L = s[-1]
    n = max(int(round(L / ds)), 2)
    s_target = np.linspace(0, L, n, endpoint=not closed)
    us = np.interp(s_target, s, u)
    pts = np.array(splev(us, tck)).T
    tan = unit(np.array(splev(us, tck, der=1)).T)
    return pts, tan, L


def split_at_corners(tan, closed, w, thr_deg):
    """Index ranges (inclusive) between sharp corners of a resampled path."""
    n = len(tan)
    if n < 2 * w + 3:
        return [(0, n - 1, closed)]
    i = np.arange(n)
    if closed:
        a, b = tan[(i - w) % n], tan[(i + w) % n]
    else:
        a, b = tan[np.clip(i - w, 0, n - 1)], tan[np.clip(i + w, 0, n - 1)]
    turn = np.degrees(np.arccos(np.clip((a * b).sum(1), -1, 1)))
    if not closed:
        turn[:w] = turn[-w:] = 0
    corners = []
    for j in np.argsort(-turn):
        if turn[j] < thr_deg:
            break
        if all(min(abs(j - c), n - abs(j - c) if closed else n) > 1.5 * w for c in corners):
            corners.append(int(j))
    corners.sort()
    if not corners:
        return [(0, n - 1, closed)]
    if closed:
        if len(corners) == 1:
            return [(corners[0], corners[0] + n, False)]       # indices taken modulo n
        return [(c0, c1 if c1 > c0 else c1 + n, False)
                for c0, c1 in zip(corners, corners[1:] + [corners[0] + n])]
    cuts = [0] + corners + [n - 1]
    return [(c0, c1, False) for c0, c1 in zip(cuts[:-1], cuts[1:])]


def rolling_ball(path, b, theta, r, ok, A, tA, B, tB, n_samp=160):
    """Roll a ball of radius r along the joint bisector in each cross-section.
    Returns ball centre, toe (contact) points on A and B, and validity."""
    n = len(path)
    half = np.clip(theta / 2, np.radians(5), np.radians(85))
    hmax = 2.5 * r / np.sin(half)
    hs = np.linspace(0, 1, n_samp)[None, :] * hmax[:, None]
    Cs = path[:, None, :] + hs[..., None] * b[:, None, :]
    flat = Cs.reshape(-1, 3)
    md = np.minimum(tA.query(flat)[0], tB.query(flat)[0]).reshape(n, n_samp)
    reach = md >= r
    j = np.argmax(reach, axis=1)
    valid = ok & reach.any(1) & (j > 0)
    j = np.clip(j, 1, n_samp - 1)
    rows = np.arange(n)
    m0, m1 = md[rows, j - 1], md[rows, j]
    f = np.clip((r - m0) / np.maximum(m1 - m0, 1e-12), 0, 1)
    h = hs[rows, j - 1] + f * (hs[rows, j] - hs[rows, j - 1])
    centre = path + h[:, None] * b
    dA, iA = tA.query(centre)
    dB, iB = tB.query(centre)
    two_sided = (np.abs(dA - r) < 0.25 * r) & (np.abs(dB - r) < 0.25 * r)
    return centre, A[iA], B[iB], valid & two_sided


def bead_area(root, toeA, toeB, centre, r):
    """Cross-section area between the two legs and the concave ball face."""
    def tri(p, q, s):
        return 0.5 * np.linalg.norm(np.cross(q - p, s - p), axis=1)
    quad = tri(root, toeA, centre) + tri(root, centre, toeB)
    a, c = unit(toeA - centre), unit(toeB - centre)
    phi = np.arccos(np.clip((a * c).sum(1), -1, 1))
    return quad - 0.5 * r ** 2 * phi


def classify_joint(theta_deg, butt_angle):
    if theta_deg >= butt_angle:
        return "butt"
    if theta_deg >= 150:
        return "open_corner"
    if theta_deg >= 30:
        return "fillet"
    return "narrow"


# ------------------------------------------------------------------- evaluation
def densify(poly, step):
    poly = np.asarray(poly, float)
    out = []
    for p, q in zip(poly[:-1], poly[1:]):
        n = max(int(np.ceil(np.linalg.norm(q - p) / step)), 1)
        out.append(p + np.linspace(0, 1, n, endpoint=False)[:, None] * (q - p))
    out.append(poly[-1:])
    return np.vstack(out)


def evaluate(paths, gt_polys, v):
    G = np.vstack([densify(p, v / 4) for p in gt_polys])
    if not paths:
        return {"n_paths": 0, "coverage": 0.0}
    Pall = np.vstack([p["points"] for p in paths])
    d = cKDTree(G).query(Pall)[0]
    cov = cKDTree(Pall).query(G)[0] < 2.0 * v
    return {
        "path_to_gt_mean": float(d.mean()),
        "path_to_gt_p95": float(np.percentile(d, 95)),
        "path_to_gt_max": float(d.max()),
        "gt_coverage": float(cov.mean()),
    }


# ----------------------------------------------------------------------- plots
def make_plots(out, CA, CB, A, B, paths, r_ball, v):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rng = np.random.default_rng(0)

    def sub(P, n):
        return P if len(P) <= n else P[rng.choice(len(P), n, replace=False)]

    sa, sb = sub(CA, 9000), sub(CB, 7000)
    allp = np.vstack([sa, sb])
    span = np.ptp(allp, axis=0)
    fig = plt.figure(figsize=(15, 7))
    for k, (elev, azim) in enumerate([(28, -60), (60, 30)]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        ax.scatter(*sa.T, s=0.4, c="0.6", alpha=0.25, linewidths=0)
        ax.scatter(*sb.T, s=0.4, c="tab:blue", alpha=0.25, linewidths=0)
        for pi, p in enumerate(paths):
            P = p["points"]
            ax.plot(*P.T, c="red", lw=2.2)
            ax.text(*P[0], f" S{pi}", color="red", fontsize=9)
            step = max(len(P) // 12, 1)
            axis = p["torch_axis"]
            okm = np.isfinite(axis).all(1)
            for i in range(0, len(P), step):
                if okm[i]:
                    tip = P[i]
                    tail = tip - axis[i] * 4 * r_ball
                    ax.plot(*np.c_[tail, tip], c="darkgreen", lw=1.2)
        ax.set_box_aspect(np.maximum(span, 1e-6))
        ax.view_init(elev, azim)
        ax.set_title("assembly: grey = base A, blue = part B, red = seams, green = torch")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "overview.png"), dpi=130)
    plt.close(fig)

    # cross-sections
    samples = []
    for pi, p in enumerate(paths):
        valid = np.flatnonzero(p["ball_ok"])
        if len(valid):
            for i in valid[np.linspace(0, len(valid) - 1, min(3, len(valid))).astype(int)]:
                samples.append((pi, i))
    samples = samples[:6]
    if not samples:
        return
    cols = min(3, len(samples))
    rows = int(np.ceil(len(samples) / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows), squeeze=False)
    for ax, (pi, i) in zip(axs.flat, samples):
        p = paths[pi]
        P, t, b = p["points"][i], p["tangent"][i], -p["torch_axis"][i]
        e1, e2 = b, unit(np.cross(t, b))
        R = 5 * r_ball

        def slab(Q):
            d = Q - P
            m = (np.abs(d @ t) < 0.8 * v) & (np.linalg.norm(d, axis=1) < R)
            return np.c_[d[m] @ e2, d[m] @ e1]

        qa, qb = slab(A), slab(B)
        ax.scatter(*qa.T, s=4, c="0.45", label="A")
        ax.scatter(*qb.T, s=4, c="tab:blue", label="B")
        c = p["ball_centre"][i] - P
        ta, tb = p["toe_A"][i] - P, p["toe_B"][i] - P
        cc = np.array([c @ e2, c @ e1])
        ang = np.linspace(0, 2 * np.pi, 100)
        ax.plot(cc[0] + r_ball * np.cos(ang), cc[1] + r_ball * np.sin(ang), "g-", lw=1)
        poly = np.array([[0, 0], [ta @ e2, ta @ e1], cc, [tb @ e2, tb @ e1]])
        ax.fill(poly[:, 0], poly[:, 1], color="orange", alpha=0.35, label="bead (approx)")
        ax.plot(0, 0, "r*", ms=12, label="root")
        ax.plot(*poly[[1, 3]].T, "ko", ms=5, label="toes")
        ax.plot(*cc, "g+", ms=10)
        ax.set_aspect("equal")
        ax.set_xlim(-R, R)
        ax.set_ylim(-R, R)
        ax.set_title(f"S{pi} #{i}: {p['joint'][i]} {p['theta_deg'][i]:.0f} deg, "
                     f"legs {p['leg_A'][i]:.1f}/{p['leg_B'][i]:.1f}")
        ax.legend(fontsize=7, loc="lower right")
    for ax in axs.flat[len(samples):]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "cross_sections.png"), dpi=120)
    plt.close(fig)


# ------------------------------------------------------------------------ main
def _ticker(timings, verbose):
    t = [time.time()]

    def tick(name):
        now = time.time()
        timings[name] = round(now - t[0], 3)
        t[0] = now
        if verbose:
            print(f"  [{timings[name]:6.2f}s] {name}")
    return tick


def extract(A0, C0, args, timings=None):
    """Steps 1-10 in memory: base cloud `A0` and assembly cloud `C0` (N x 3 arrays) in,
    seam paths out. `args` is a `build_parser()` namespace (only the tuning fields are
    read). Returns a dict with the paths and the intermediates the outputs need."""
    timings = {} if timings is None else timings
    tick = _ticker(timings, getattr(args, "verbose", False))
    v = args.voxel or estimate_spacing(A0)
    A = voxel_downsample(A0, v)
    C = voxel_downsample(C0, v)
    if args.outliers:
        A, C = remove_outliers(A), remove_outliers(C)
    tick("downsample")

    # parameters (multiples of v unless given)
    g = args.footprint * v + args.gap
    r_s = 0.75 * v
    r_ball = args.ball_radius or 4.0 * v
    ds = args.step or 2.0 * v
    R_dir = 4.0 * v
    r_vis = 3.0 * v

    nA, idxA, tA = pca_normals(A, k=16)
    reg_info = None
    if args.register:
        nC_tmp, _, _ = pca_normals(C, k=16)
        T, A, rmse = icp_point_to_plane(A, nA, C, nC_tmp, max_dist=args.reg_max_dist or 6 * v,
                                        min_dist=1.5 * v)
        nA, idxA, tA = pca_normals(A, k=16)
        reg_info = {"transform": T.tolist(), "rmse": rmse}
        tick("registration")

    CA, CB = segment_assembly(C, A, nA, tA, v, 1.5 * v, np.cos(np.radians(45)), 30)
    if len(CB) < 20:
        raise SystemExit("Part B is empty after segmentation - check registration / units.")
    nB, idxB, tB = pca_normals(CB, k=16)
    tCA = cKDTree(CA)
    tick("segmentation")

    # distance fields (smoothed over each surface)
    DA_raw = bounded_distance(A, CB, nB, tB, r_s)
    DB_raw = bounded_distance(CB, A, nA, tA, r_s)
    DA, DB = DA_raw.copy(), DB_raw.copy()
    for _ in range(args.smooth):          # optional, for noisy scans
        DA = DA[idxA].mean(1)
        DB = DB[idxB].mean(1)
    FA, FB = DA < g, DB < g
    if args.boundary_only:   # footprint points with at least one non-footprint neighbour
        FA &= (~FA[idxA]).any(1)
        FB &= (~FB[idxB]).any(1)
    tick("distance fields + footprints")

    # direction vectors and projection onto the root
    cands, origin = [], []
    for name, P, nP, idxP, tP, D, Draw, F, other in (
        ("A", A, nA, idxA, tA, DA, DA_raw, FA, (CB, nB, tB)),
        ("B", CB, nB, idxB, tB, DB, DB_raw, FB, (A, nA, tA)),
    ):
        sel = np.flatnonzero(F)
        if len(sel) == 0:
            continue
        grad = surface_gradient(P, nP, D, idxP, v)[sel]
        gm = np.linalg.norm(grad, axis=1)
        good = gm > args.grad_min
        sel, grad, gm = sel[good], grad[good], gm[good]
        X, ok = project_to_root(P[sel], unit(grad), gm, Draw[sel], args.gap, other,
                                (P, nP, tP), r_s, 3 * g, 0.75 * v)
        cands.append(X[ok])
        origin.append(np.full(ok.sum(), name))
    X = np.vstack(cands) if cands else np.zeros((0, 3))
    origin = np.concatenate(origin) if origin else np.zeros(0, str)
    n_raw = len(X)
    tick("direction vectors + root projection")

    # exposure: a real seam has both parts visible nearby in the assembly scan
    exposed = (tCA.query(X)[0] < r_vis) & (tB.query(X)[0] < r_vis)
    X, origin = X[exposed], origin[exposed]

    # cluster on position + groove-opening direction (separates the two sides of a T)
    dA, okA = leg_direction(X, CA, tCA, R_dir)
    dB, okB = leg_direction(X, CB, tB, R_dir)
    # un-normalised: for butt joints (opposite legs) the term vanishes instead of
    # becoming random noise; for the two sides of a T it differs by ~1
    open_dir = 0.5 * (dA * okA[:, None] + dB * okB[:, None])
    lab = radius_components(np.hstack([X, 2.0 * v * open_dir]), 2.5 * v)
    tick("exposure + clustering")

    paths = []
    for c in np.unique(lab):
        Pc = X[lab == c]
        if len(Pc) < args.min_points:
            continue
        Pc = voxel_downsample(thin_curve(Pc, 2.5 * v), 0.5 * v)
        order = order_points(Pc, 2.0 * v)
        Po = Pc[order]
        if len(Po) < 4:
            continue
        length = np.linalg.norm(np.diff(Po, axis=0), axis=1).sum()
        if length < args.min_length * v:
            continue
        closed = np.linalg.norm(Po[0] - Po[-1]) < 3 * v and length > 12 * v
        pts, tan, L = fit_spline(Po, closed, 0.3 * v, ds)

        # joint frame from leg directions, made perpendicular to the seam
        la, oka = leg_direction(pts, CA, tCA, R_dir)
        lb, okb = leg_direction(pts, CB, tB, R_dir)
        la = unit(la - (la * tan).sum(1, keepdims=True) * tan)
        lb = unit(lb - (lb * tan).sum(1, keepdims=True) * tan)
        theta = np.arccos(np.clip((la * lb).sum(1), -1, 1))
        th_deg = np.degrees(theta)
        bis = unit(la + lb)
        butt = th_deg >= args.butt_angle
        if butt.any():   # flat joint: open towards the side without material
            _, inear = tCA.query(pts[butt])
            n_loc = nA[tA.query(CA[inear])[1]]
            dd, ii = tA.query(pts[butt], k=32, distance_upper_bound=R_dir)
            side = np.nansum(np.where(np.isfinite(dd)[..., None],
                                      A[np.minimum(ii, len(A) - 1)] - pts[butt][:, None], 0), axis=1)
            sgn = np.sign((side * n_loc).sum(1))
            sgn[sgn == 0] = 1
            n_loc = -sgn[:, None] * n_loc
            bis[butt] = unit(n_loc - (n_loc * tan[butt]).sum(1, keepdims=True) * tan[butt])
        frame_ok = oka & okb
        torch_axis = np.where(frame_ok[:, None], -bis, np.nan)

        ball_ok_in = frame_ok & ~butt
        centre, toeA, toeB, ball_ok = rolling_ball(pts, bis, theta, r_ball, ball_ok_in,
                                                    A, tA, CB, tB)
        legA = np.linalg.norm(toeA - pts, axis=1)
        legB = np.linalg.norm(toeB - pts, axis=1)
        area = bead_area(pts, toeA, toeB, centre, r_ball)
        nan = ~ball_ok
        centre[nan], toeA[nan], toeB[nan] = np.nan, np.nan, np.nan
        legA[nan], legB[nan], area[nan] = np.nan, np.nan, np.nan
        joints = np.array([classify_joint(t, args.butt_angle) for t in th_deg])
        full = {"points": pts, "tangent": tan, "torch_axis": torch_axis,
                "theta_deg": th_deg, "joint": joints, "ball_centre": centre,
                "toe_A": toeA, "toe_B": toeB, "leg_A": legA, "leg_B": legB,
                "bead_area": area, "ball_ok": ball_ok, "frame_ok": frame_ok}
        ranges = ([(0, len(pts) - 1, closed)] if args.no_corner_split else
                  split_at_corners(tan, closed, max(1, int(round(3 * v / ds))), args.corner_angle))
        for (i0, i1, seg_closed) in ranges:
            ii = np.arange(i0, i1 + 1) % len(pts)
            p = {k: val[ii] for k, val in full.items()}
            P = p["points"]
            seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
            Lseg = float(seg.sum() + (np.linalg.norm(P[-1] - P[0]) if seg_closed else 0))
            if Lseg < 3 * v:
                continue
            a_f = np.where(np.isfinite(p["bead_area"]), p["bead_area"], 0)
            volume = float((0.5 * (a_f[1:] + a_f[:-1]) * seg).sum())
            fo, bo = p["frame_ok"], p["ball_ok"]
            names, counts = np.unique(p["joint"][fo], return_counts=True)
            jt = str(names[np.argmax(counts)]) if len(names) else "unknown"
            if jt == "butt":      # rolling ball is not meaningful for flat joints
                bo = np.zeros_like(bo)
                volume = 0.0
            p["summary"] = {
                "length": Lseg, "closed": bool(seg_closed), "n_points": int(len(P)),
                "cluster": int(c), "cluster_candidates": int(len(Pc)),
                "joint_type": jt,
                "theta_median_deg": float(np.nanmedian(p["theta_deg"])),
                "leg_A_median": float(np.nanmedian(p["leg_A"])) if bo.any() else None,
                "leg_B_median": float(np.nanmedian(p["leg_B"])) if bo.any() else None,
                "bead_volume": volume,
                "frame_valid_frac": float(fo.mean()),
                "ball_valid_frac": float(bo.mean()),
            }
            paths.append(p)
    paths.sort(key=lambda p: -p["summary"]["length"])
    tick("paths + frames + rolling ball")
    return {"paths": paths, "A": A, "C": C, "CA": CA, "CB": CB, "FA": FA, "FB": FB,
            "X": X, "v": v, "g": g, "r_ball": r_ball, "ds": ds, "n_raw": n_raw,
            "reg_info": reg_info, "timings": timings}


def run(args):
    T0 = time.time()
    timings = {}
    os.makedirs(args.out, exist_ok=True)
    A0 = load_points(args.base)
    C0 = load_points(args.combined)
    tick = _ticker(timings, args.verbose)
    tick("load")
    r = extract(A0, C0, args, timings)
    tick = _ticker(timings, args.verbose)
    paths, A, C, CA, CB, FA, FB, X = (r[k] for k in ("paths", "A", "C", "CA", "CB", "FA", "FB", "X"))
    v, g, r_ball, ds, n_raw, reg_info = (r[k] for k in ("v", "g", "r_ball", "ds", "n_raw", "reg_info"))

    # ---- outputs
    rows = []
    for pi, p in enumerate(paths):
        for i in range(len(p["points"])):
            rows.append([pi, i, *p["points"][i], *p["tangent"][i], *p["torch_axis"][i],
                         p["theta_deg"][i], *p["ball_centre"][i], *p["toe_A"][i],
                         *p["toe_B"][i], p["leg_A"][i], p["leg_B"][i], p["bead_area"][i],
                         int(p["ball_ok"][i]), p["joint"][i]])
    hdr = ("path,idx,x,y,z,tx,ty,tz,ax,ay,az,theta_deg,ball_x,ball_y,ball_z,"
           "toeA_x,toeA_y,toeA_z,toeB_x,toeB_y,toeB_z,leg_A,leg_B,bead_area,ball_ok,joint")
    with open(os.path.join(args.out, "seam_paths.csv"), "w") as f:
        f.write(hdr + "\n")
        for r in rows:
            f.write(",".join(f"{x:.5f}" if isinstance(x, float) else str(x) for x in r) + "\n")

    dense = [densify(p["points"], 0.3 * v) for p in paths]
    parts = [(CA, (160, 160, 160)), (CB, (90, 150, 230)), (A[FA], (255, 150, 30)),
             (CB[FB], (250, 220, 40)), (X, (230, 30, 30))] + [(d, (20, 200, 60)) for d in dense]
    write_ply(os.path.join(args.out, "debug_points.ply"),
              np.vstack([q for q, _ in parts]),
              np.vstack([np.tile(c, (len(q), 1)) for q, c in parts]))

    summary = {
        "inputs": {"base": args.base, "combined": args.combined,
                   "n_base": int(len(A)), "n_combined": int(len(C)),
                   "n_A_in_assembly": int(len(CA)), "n_B": int(len(CB))},
        "params": {"voxel": v, "footprint_g": g, "ball_radius": r_ball, "step": ds,
                   "grad_min": args.grad_min, "boundary_only": args.boundary_only},
        "registration": reg_info,
        "candidates": {"raw": int(n_raw), "exposed": int(len(X)),
                       "footprint_A": int(FA.sum()), "footprint_B": int(FB.sum())},
        "seams": [p["summary"] for p in paths],
        "timings_s": timings, "total_s": round(time.time() - T0, 2),
    }
    if args.gt:
        with open(args.gt) as f:
            gt = json.load(f)
        summary["ground_truth_eval"] = evaluate(paths, gt["polylines"], v)
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    if not args.no_plots:
        make_plots(args.out, CA, CB, A, CB, paths, r_ball, v)
    tick("outputs")
    return summary


def print_summary(s):
    print(f"voxel={s['params']['voxel']:.3g}  g={s['params']['footprint_g']:.3g}  "
          f"ball r={s['params']['ball_radius']:.3g}  |  A={s['inputs']['n_base']} "
          f"B={s['inputs']['n_B']}  candidates {s['candidates']['raw']} -> "
          f"{s['candidates']['exposed']}  ({s['total_s']} s)")
    for i, p in enumerate(s["seams"]):
        legs = (f"legs {p['leg_A_median']:.2f}/{p['leg_B_median']:.2f}"
                if p["leg_A_median"] is not None else "legs -")
        print(f"  S{i}: {p['joint_type']:<11} L={p['length']:8.2f} "
              f"{'closed' if p['closed'] else 'open  '}  theta~{p['theta_median_deg']:6.1f}  "
              f"{legs}  vol={p['bead_volume']:.1f}  frames {p['frame_valid_frac']:.0%} "
              f"ball {p['ball_valid_frac']:.0%}")
    if "ground_truth_eval" in s:
        e = s["ground_truth_eval"]
        if "gt_coverage" in e:
            print(f"  GT: mean {e['path_to_gt_mean']:.3f}  p95 {e['path_to_gt_p95']:.3f}  "
                  f"max {e['path_to_gt_max']:.3f}  coverage {e['gt_coverage']:.1%}")
        else:
            print("  GT: no seams found")


def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--base", required=True, help="base part point cloud (ply/pcd/xyz/csv/npy)")
    p.add_argument("--combined", required=True, help="assembly point cloud in weld position")
    p.add_argument("--out", default="weld_out")
    p.add_argument("--voxel", type=float, default=None, help="voxel size v (default: point spacing)")
    p.add_argument("--footprint", type=float, default=3.0, help="footprint threshold g in voxels")
    p.add_argument("--gap", type=float, default=0.0, help="expected fit-up gap (data units)")
    p.add_argument("--ball-radius", type=float, default=None, help="rolling-ball radius (default 4v)")
    p.add_argument("--step", type=float, default=None, help="path resampling step (default 2v)")
    p.add_argument("--grad-min", type=float, default=0.25, help="min |grad D| for projection")
    p.add_argument("--smooth", type=int, default=0, help="distance-field smoothing iterations")
    p.add_argument("--butt-angle", type=float, default=155.0, help="leg angle treated as butt")
    p.add_argument("--corner-angle", type=float, default=50.0, help="split paths at turns sharper than this")
    p.add_argument("--no-corner-split", action="store_true")
    p.add_argument("--min-points", type=int, default=20)
    p.add_argument("--min-length", type=float, default=10.0, help="min seam length in voxels")
    p.add_argument("--boundary-only", action="store_true", help="project only footprint boundary points")
    p.add_argument("--register", action="store_true", help="ICP-align base into combined first")
    p.add_argument("--reg-max-dist", type=float, default=None,
                   help="initial ICP correspondence radius (default 6v); must exceed the misalignment")
    p.add_argument("--outliers", action="store_true", help="statistical outlier removal")
    p.add_argument("--gt", default=None, help="ground-truth JSON {'polylines': [...]} for metrics")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("-v", "--verbose", action="store_true")
    return p


if __name__ == "__main__":
    a = build_parser().parse_args()
    print_summary(run(a))
    print(f"outputs written to {a.out}/")
