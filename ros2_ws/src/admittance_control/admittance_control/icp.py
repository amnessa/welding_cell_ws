"""Pure-NumPy point-to-plane ICP and the geometry helpers it needs.

Runs with plain NumPy alone (no hard open3d / scipy / sklearn dependency, so it
works on the dependency-light robot side and is unit-testable without ROS). If
SciPy *is* importable, ``nearest_neighbor`` transparently uses a KD-tree
(``scipy.spatial.cKDTree``) instead of the brute-force O(M*N) search -- ~15x
faster for the cloud sizes here, which is the main ICP speedup for real-time
tracking. Everything else stays pure NumPy.

Pipeline the icp_pose_refiner_node builds on top of this:
  - load_ply_mesh + sample_mesh_surface : CAD .ply -> model point cloud
  - backproject_depth                   : depth image + K -> organized cloud
  - estimate_normals_organized          : organized cloud -> per-pixel normals
    (cross product of pixel-neighbour gradients; no KD-tree needed)
  - voxel_downsample                     : thin dense clouds before ICP
  - icp_point_to_plane                   : refine a model->scene transform

Frames/units: everything is metres. The SAM-6D pose (model->camera, t in mm) is
converted to metres and passed as ``init`` to icp_point_to_plane; the returned
4x4 is the refined model->camera transform.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:  # optional: KD-tree accelerates nearest_neighbor when available
    from scipy.spatial import cKDTree as _cKDTree
except Exception:  # noqa: BLE001 - no scipy -> brute-force fallback
    _cKDTree = None


# Dynamic-Welsch schedule constants, matching the reference Fast-Robust-ICP
# (Zhang et al.; yaoyx689/Fast-Robust-ICP, ICP::Parameters). nu is annealed from
# nu_begin_k * median(residual) down to nu_end_k * (local surface noise), halving
# each outer stage.
NU_BEGIN_K = 3.0
NU_END_K = 1.0 / (3.0 * np.sqrt(3.0))
NU_ALPHA = 0.5


# ─────────────────────────── PLY mesh loading ────────────────────────────
def load_ply_mesh(path: Path) -> Tuple[np.ndarray, List[Tuple[int, ...]]]:
    """Read vertices (N,3 float64) and polygon faces from an ascii/binary PLY.

    Only the first three float vertex properties (x,y,z) are used. Faces are
    read from a ``property list <count> <index> vertex_indices`` element. This
    is a minimal reader, sufficient for the small CAD models used here.
    """
    with open(path, 'rb') as f:
        header_lines: List[str] = []
        raw = b''
        while b'end_header' not in raw:
            raw = f.readline()
            if not raw:
                raise ValueError(f'{path}: no end_header')
            header_lines.append(raw.decode('ascii', 'replace').rstrip('\n'))

        fmt = next(l for l in header_lines if l.startswith('format'))
        is_ascii = 'ascii' in fmt
        n_vert = 0
        n_face = 0
        vert_props: List[str] = []
        in_vertex = in_face = False
        for l in header_lines:
            if l.startswith('element vertex'):
                n_vert = int(l.split()[-1]); in_vertex, in_face = True, False
            elif l.startswith('element face'):
                n_face = int(l.split()[-1]); in_vertex, in_face = False, True
            elif l.startswith('element'):
                in_vertex = in_face = False
            elif l.startswith('property') and in_vertex and 'list' not in l:
                vert_props.append(l.split()[-1])

        if is_ascii:
            verts = np.array([[float(x) for x in f.readline().split()[:3]]
                              for _ in range(n_vert)], dtype=np.float64)
            faces: List[Tuple[int, ...]] = []
            for _ in range(n_face):
                parts = f.readline().split()
                k = int(parts[0])
                faces.append(tuple(int(x) for x in parts[1:1 + k]))
        else:  # binary_little_endian, float32 vertex props
            stride = len(vert_props)
            data = np.frombuffer(f.read(n_vert * stride * 4),
                                 dtype='<f4').reshape(n_vert, stride)
            verts = data[:, :3].astype(np.float64)
            faces = []
            for _ in range(n_face):
                k = int(np.frombuffer(f.read(1), dtype=np.uint8)[0])
                idx = np.frombuffer(f.read(k * 4), dtype='<i4')
                faces.append(tuple(int(x) for x in idx))
    return verts, faces


def sample_mesh_surface(verts: np.ndarray, faces: List[Tuple[int, ...]],
                        n_points: int, rng: np.random.Generator) -> np.ndarray:
    """Area-weighted uniform sampling of ``n_points`` on the mesh surface.

    Polygons are fan-triangulated. Returns (n_points, 3) in the mesh's units.
    Falls back to returning the vertices themselves if there are no faces.
    """
    tris: List[Tuple[int, int, int]] = []
    for face in faces:
        for i in range(1, len(face) - 1):
            tris.append((face[0], face[i], face[i + 1]))
    if not tris:
        return verts.copy()

    tri = np.array(tris, dtype=np.int64)
    v0, v1, v2 = verts[tri[:, 0]], verts[tri[:, 1]], verts[tri[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    total = areas.sum()
    if total <= 0:
        return verts.copy()
    probs = areas / total
    choice = rng.choice(len(tri), size=n_points, p=probs)
    u = rng.random(n_points)
    w = rng.random(n_points)
    over = u + w > 1.0
    u[over], w[over] = 1.0 - u[over], 1.0 - w[over]
    a, b, c = v0[choice], v1[choice], v2[choice]
    return a + (b - a) * u[:, None] + (c - a) * w[:, None]


# ─────────────────────── depth back-projection / normals ──────────────────
def backproject_depth(depth_m: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Organized (H,W,3) camera-frame point cloud from a metric depth image.

    Invalid pixels (depth <= 0 or non-finite) become NaN so they can be masked
    out downstream. Uses the optical-frame convention (x right, y down, z fwd).
    """
    h, w = depth_m.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_m.astype(np.float64)
    with np.errstate(invalid='ignore'):
        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy
    xyz = np.dstack((x, y, z))
    invalid = ~np.isfinite(z) | (z <= 0.0)
    xyz[invalid] = np.nan
    return xyz


def estimate_normals_organized(xyz: np.ndarray) -> np.ndarray:
    """Per-pixel normals (H,W,3) from an organized cloud via neighbour gradients.

    Normal = normalize(cross(dX/du, dX/dv)) using central differences over the
    pixel grid, then flipped to face the camera (origin). Border pixels and any
    pixel touching an invalid neighbour get NaN. No KD-tree required -- this is
    the cheap, standard way to get normals off a depth image.
    """
    h, w, _ = xyz.shape
    normals = np.full_like(xyz, np.nan)
    du = xyz[1:-1, 2:, :] - xyz[1:-1, :-2, :]       # horizontal gradient
    dv = xyz[2:, 1:-1, :] - xyz[:-2, 1:-1, :]       # vertical gradient
    n = np.cross(du, dv)
    norm = np.linalg.norm(n, axis=2, keepdims=True)
    with np.errstate(invalid='ignore', divide='ignore'):
        n = n / norm
    center = xyz[1:-1, 1:-1, :]
    # Orient toward the camera at the origin: normal should point back at us.
    flip = np.sum(n * center, axis=2) > 0.0
    n[flip] = -n[flip]
    bad = (~np.isfinite(n).all(axis=2)) | (norm[..., 0] <= 0.0)
    n[bad] = np.nan
    normals[1:-1, 1:-1, :] = n
    return normals


# ────────────────────────────── downsample ────────────────────────────────
def voxel_downsample(points: np.ndarray, voxel: float,
                     extra: Optional[np.ndarray] = None
                     ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Average points (and optionally an ``extra`` per-point array) per voxel.

    If ``extra`` looks like unit normals it is renormalized after averaging.
    Returns (down_points, down_extra). ``voxel <= 0`` disables downsampling.
    """
    if voxel <= 0 or len(points) == 0:
        return points, extra
    keys = np.floor(points / voxel).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    m = inv.max() + 1
    counts = np.bincount(inv, minlength=m)
    sums = np.zeros((m, 3))
    for c in range(3):
        sums[:, c] = np.bincount(inv, weights=points[:, c], minlength=m)
    down = sums / counts[:, None]
    down_extra = None
    if extra is not None:
        es = np.zeros((m, extra.shape[1]))
        for c in range(extra.shape[1]):
            es[:, c] = np.bincount(inv, weights=extra[:, c], minlength=m)
        es /= counts[:, None]
        nrm = np.linalg.norm(es, axis=1, keepdims=True)
        # Renormalize only rows that are plausibly unit vectors (normals).
        if extra.shape[1] == 3 and np.nanmedian(np.linalg.norm(extra, axis=1)) > 0.5:
            with np.errstate(invalid='ignore', divide='ignore'):
                es = np.where(nrm > 0, es / nrm, es)
        down_extra = es
    return down, down_extra


# ──────────────────────────── oriented crop box ───────────────────────────
def crop_box_mask(points: np.ndarray, T_box: np.ndarray,
                  min_corner: np.ndarray, max_corner: np.ndarray) -> np.ndarray:
    """Boolean mask of ``points`` inside an oriented box.

    The box is an axis-aligned range ``[min_corner, max_corner]`` in the frame
    given by pose ``T_box`` (4x4, box->world). Points are expressed in the box
    frame via ``R^T (p - t)`` and range-tested. This is the "macro-filter" that
    replaces the SAM-6D mask in the tracking loop: transform the box to the
    current object pose and keep only the points that fall inside it.
    """
    R, t = T_box[:3, :3], T_box[:3, 3]
    local = (points - t) @ R            # row-wise R^T (p - t)
    return (np.all(local >= min_corner, axis=1)
            & np.all(local <= max_corner, axis=1))


# ──────────────────────────── nearest neighbour ───────────────────────────
def nearest_neighbor(src: np.ndarray, dst: np.ndarray,
                     chunk: int = 512) -> Tuple[np.ndarray, np.ndarray]:
    """NN index + distance from each src point to dst. Returns (idx, dist).

    Uses ``scipy.spatial.cKDTree`` when SciPy is importable (~15x faster for the
    cloud sizes here), otherwise falls back to a chunked brute-force O(M*N)
    search that bounds peak memory and needs no extra dependency.
    """
    if _cKDTree is not None and len(dst) > 0:
        dist, idx = _cKDTree(dst).query(src, workers=-1)
        return np.asarray(idx, dtype=np.int64), np.asarray(dist, dtype=np.float64)

    idx = np.empty(len(src), dtype=np.int64)
    dist = np.empty(len(src), dtype=np.float64)
    dst2 = np.einsum('ij,ij->i', dst, dst)
    for s in range(0, len(src), chunk):
        block = src[s:s + chunk]
        # ||a-b||^2 = ||a||^2 - 2 a.b + ||b||^2
        d2 = (np.einsum('ij,ij->i', block, block)[:, None]
              - 2.0 * block @ dst.T + dst2[None, :])
        j = np.argmin(d2, axis=1)
        idx[s:s + chunk] = j
        dist[s:s + chunk] = np.sqrt(np.maximum(d2[np.arange(len(block)), j], 0.0))
    return idx, dist


# ─────────────────────────── point-to-plane ICP ───────────────────────────
def _skew(w: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def _rodrigues(w: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    k = w / theta
    K = _skew(k)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


# ── SE(3) exp/log (Lie-algebra parameterization for Anderson acceleration) ──
def _se3_exp(xi: np.ndarray) -> np.ndarray:
    """Exponential map se(3) -> SE(3). xi = [omega(3), upsilon(3)]."""
    w, up = xi[:3], xi[3:]
    theta = float(np.linalg.norm(w))
    R = _rodrigues(w)
    W = _skew(w)
    if theta < 1e-9:
        V = np.eye(3) + 0.5 * W          # small-angle limit of the left Jacobian
    else:
        A = (1.0 - np.cos(theta)) / (theta * theta)
        B = (theta - np.sin(theta)) / (theta ** 3)
        V = np.eye(3) + A * W + B * (W @ W)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, V @ up
    return T


def _se3_log(T: np.ndarray) -> np.ndarray:
    """Logarithm map SE(3) -> se(3). Returns [omega(3), upsilon(3)]."""
    R, t = T[:3, :3], T[:3, 3]
    cos_t = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    theta = float(np.arccos(cos_t))
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if theta < 1e-9:
        return np.concatenate((0.5 * axis, t))       # V^-1 ~ I near identity
    w = (theta / (2.0 * np.sin(theta))) * axis
    W = _skew(w)
    half = 0.5 * theta
    # Left-Jacobian inverse: I - 1/2 W + (1/theta^2)(1 - (theta/2) cot(theta/2)) W^2
    coeff = (1.0 - half * np.cos(half) / np.sin(half)) / (theta * theta)
    Vinv = np.eye(3) - 0.5 * W + coeff * (W @ W)
    return np.concatenate((w, Vinv @ t))


def _p2plane_gn_step(source, target, target_normals, T, max_corr_dist):
    """One Gauss-Newton point-to-plane iteration from pose ``T``.

    Returns (T_next, energy, n_inl, rmse, incr) where ``energy`` is the mean
    squared point-to-plane residual *at* ``T`` (used for the Anderson safeguard),
    ``rmse`` the inlier RMSE at ``T`` and ``incr`` the update norm. Returns None
    if there are too few correspondences to solve.
    """
    R, t = T[:3, :3], T[:3, 3]
    src_t = source @ R.T + t
    idx, dist = nearest_neighbor(src_t, target)
    inl = dist < max_corr_dist
    n_inl = int(inl.sum())
    if n_inl < 6:
        return None
    p = src_t[inl]
    q = target[idx[inl]]
    nq = target_normals[idx[inl]]
    # A x = b, x = [wx,wy,wz, tx,ty,tz]; row_i = [p_i x n_i, n_i]
    A = np.hstack((np.cross(p, nq), nq))
    b = -np.einsum('ij,ij->i', p - q, nq)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    Td = np.eye(4)
    Td[:3, :3] = _rodrigues(x[:3])
    Td[:3, 3] = x[3:]
    T_next = Td @ T
    rmse = float(np.sqrt(np.mean(dist[inl] ** 2)))
    energy = float(np.mean(b ** 2))
    return T_next, energy, n_inl, rmse, float(np.linalg.norm(x))


# ───────────────────── Welsch robust kernel (Robust-ICP) ──────────────────
def _welsch_weight(r_abs: np.ndarray, nu: float) -> np.ndarray:
    """Welsch reweighting w_i = exp(-r_i^2 / (2 nu^2)) for |residual| r_abs."""
    nu = max(float(nu), 1e-12)
    return np.exp(-(r_abs * r_abs) / (2.0 * nu * nu))


def _welsch_energy(r_abs: np.ndarray, nu: float) -> float:
    """Welsch energy sum_i (1 - exp(-r_i^2 / (2 nu^2)))."""
    nu = max(float(nu), 1e-12)
    return float(np.sum(1.0 - np.exp(-(r_abs * r_abs) / (2.0 * nu * nu))))


def welsch_nu_end(target: np.ndarray, target_normals: np.ndarray,
                  k: int = 7) -> float:
    """Local surface-noise scale used for the Welsch nu floor (nu_end).

    Port of ``FindKnearestNormMed`` in the reference: for every target point,
    take its ``k`` nearest neighbours and measure how far they lie *along the
    point's own normal* (the point-to-plane thickness of the local surface),
    reduce with the median, then take the median over all points. Multiplying
    this by ``NU_END_K`` gives the smallest Welsch bandwidth we anneal down to --
    it stops the kernel from shrinking below the sensor's own noise floor.
    """
    n = len(target)
    if n < 2:
        return 0.0
    kk = min(int(k), n)
    if _cKDTree is not None:
        _, idx = _cKDTree(target).query(target, k=kk, workers=-1)
        idx = np.atleast_2d(idx)
        base = target[idx[:, 0]]                     # the point itself
        nrm = target_normals[idx[:, 0]]
        neigh = target[idx[:, 1:]]                    # (n, kk-1, 3)
        d = np.abs(np.einsum('nkj,nj->nk', neigh - base[:, None, :], nrm))
        per_point = np.median(d, axis=1)
        return float(np.median(per_point))
    # Brute-force fallback (subsample the target to bound the O(n^2) cost).
    if n > 2000:
        sel = np.random.default_rng(0).choice(n, 2000, replace=False)
        sub, sub_n = target[sel], target_normals[sel]
    else:
        sub, sub_n = target, target_normals
    per_point = np.empty(len(sub))
    for i in range(len(sub)):
        d2 = np.einsum('ij,ij->i', target - sub[i], target - sub[i])
        nn = np.argsort(d2)[1:kk]
        per_point[i] = np.median(np.abs((target[nn] - sub[i]) @ sub_n[i]))
    return float(np.median(per_point))


def _welsch_step(source, target, target_normals, T, nu):
    """One reweighted (Welsch) point-to-plane Gauss-Newton iteration from ``T``.

    Unlike the plain step there is **no hard correspondence cutoff**: every
    source point contributes, weighted by ``exp(-r^2/2nu^2)`` so a gripper or
    fixture that wanders into the crop box is smoothly ignored instead of
    dragging the fit. The weighted normal-equation is the MM surrogate that
    majorizes the Welsch energy, so minimizing it decreases that energy.

    Returns (T_next, energy_at_T, r_signed) or None if the weighted system is
    rank-deficient (too little effective support).
    """
    R, t = T[:3, :3], T[:3, 3]
    src_t = source @ R.T + t
    idx, _dist = nearest_neighbor(src_t, target)
    q = target[idx]
    nq = target_normals[idx]
    r = np.einsum('ij,ij->i', src_t - q, nq)         # signed point-to-plane
    r_abs = np.abs(r)
    w = _welsch_weight(r_abs, nu)
    if np.count_nonzero(w > 1e-6) < 6:
        return None
    energy = _welsch_energy(r_abs, nu)
    sw = np.sqrt(w)
    # Weighted GN: row_i = sqrt(w_i) [p_i x n_i, n_i], rhs_i = -sqrt(w_i) r_i
    A = np.hstack((np.cross(src_t, nq), nq)) * sw[:, None]
    b = -r * sw
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    Td = np.eye(4)
    Td[:3, :3] = _rodrigues(x[:3])
    Td[:3, 3] = x[3:]
    return Td @ T, energy, r


def _welsch_stage(source, target, target_normals, T_start, nu,
                  max_iter, anderson_depth):
    """Run reweighted point-to-plane at a *fixed* nu, Anderson-accelerated.

    Same fixed-point-in-se(3)-increment scheme as the Fast path, but the step is
    the Welsch-reweighted one and the safeguarded energy is the Welsch energy.
    Returns (T, iterations).
    """
    T0inv = np.eye(4)
    T0inv[:3, :3] = T_start[:3, :3].T
    T0inv[:3, 3] = -T_start[:3, :3].T @ T_start[:3, 3]

    def _fp(xi):
        T = _se3_exp(xi) @ T_start
        step = _welsch_step(source, target, target_normals, T, nu)
        if step is None:
            return None
        T_next, energy, _r = step
        return _se3_log(T_next @ T0inv), energy, T_next

    # Plain reweighted iteration (no Anderson).
    if anderson_depth <= 0:
        T = T_start
        for it in range(max_iter):
            step = _welsch_step(source, target, target_normals, T, nu)
            if step is None:
                return T, it
            T = step[0]
        return T, max_iter

    fp = _fp(np.zeros(6))
    if fp is None:
        return T_start, 0
    g, best_E, best_T = fp
    xs, gs = [np.zeros(6)], [g]
    for it in range(1, max_iter):
        m = min(len(xs), anderson_depth + 1)
        if m >= 2:
            F = np.array([gs[i] - xs[i] for i in range(len(xs) - m, len(xs))])
            G = np.array(gs[len(xs) - m:])
            dF = np.diff(F, axis=0).T
            dG = np.diff(G, axis=0).T
            theta, *_ = np.linalg.lstsq(dF, F[-1], rcond=None)
            cand = gs[-1] - dG @ theta
        else:
            cand = gs[-1]

        fp = _fp(cand)
        if fp is None:
            break
        g_c, E_c, T_c = fp
        if E_c <= best_E * (1.0 + 1e-9):             # accept extrapolated iterate
            xs.append(cand)
            gs.append(g_c)
            best_E, best_T = E_c, T_c
        else:                                        # reject: plain step, reset
            plain = gs[-1]
            fp2 = _fp(plain)
            if fp2 is None:
                break
            g2, E2, T2 = fp2
            xs, gs = [plain], [g2]
            best_E, best_T = E2, T2
        if len(xs) > anderson_depth + 1:
            xs, gs = xs[-(anderson_depth + 1):], gs[-(anderson_depth + 1):]
    return best_T, max_iter


def _final_metrics(source, target, target_normals, T, max_corr_dist):
    """Fitness / inlier-RMSE / correspondences of ``T`` (point-to-plane)."""
    R, t = T[:3, :3], T[:3, 3]
    src_t = source @ R.T + t
    idx, dist = nearest_neighbor(src_t, target)
    r = np.einsum('ij,ij->i', src_t - target[idx], target_normals[idx])
    inl = dist < max_corr_dist
    n_inl = int(inl.sum())
    rmse = float(np.sqrt(np.mean(r[inl] ** 2))) if n_inl else float('nan')
    return n_inl, rmse


def icp_point_to_plane(source: np.ndarray, target: np.ndarray,
                       target_normals: np.ndarray,
                       init: Optional[np.ndarray] = None,
                       max_corr_dist: float = 0.01,
                       max_iter: int = 40,
                       tol: float = 1e-6,
                       anderson_depth: int = 0,
                       robust: bool = False,
                       nu_begin_k: float = NU_BEGIN_K,
                       nu_end_k: float = NU_END_K,
                       nu_alpha: float = NU_ALPHA) -> Tuple[np.ndarray, Dict]:
    """Refine a source->target rigid transform minimizing point-to-plane error.

    Minimizes sum_i (((R p_i + t) - q_i) . n_i)^2 over correspondences, where
    q_i,n_i are the nearest target point and its normal. Each iteration is a
    small-angle Gauss-Newton step solved with numpy.lstsq.

    This is a NumPy port of Zhang et al., "Fast and Robust Iterative Closest
    Point" (yaoyx689/Fast-Robust-ICP) with three selectable levels:

    * **Plain** (``anderson_depth=0, robust=False``) -- baseline point-to-plane.
    * **Fast** (``anderson_depth>0, robust=False``) -- ICP as a fixed-point
      iteration on the pose, parameterized in se(3) (as an increment relative to
      ``init``, kept near the origin so the log map stays well-conditioned) and
      accelerated with Anderson Acceleration of the given history depth. A
      monotone safeguard accepts the extrapolated iterate only when the
      point-to-plane energy does not increase, else it falls back to a plain
      step and resets the history.
    * **Fast + Robust** (``robust=True``) -- the full method: every iteration is
      **Welsch-reweighted** (w_i = exp(-r_i^2 / 2nu^2)), so outliers such as a
      gripper/fixture entering the crop box are smoothly ignored. The bandwidth
      ``nu`` is *dynamically annealed*: it starts at ``nu_begin_k * median(r)``,
      is multiplied by ``nu_alpha`` after each stage, and stops at
      ``nu_end_k * welsch_nu_end(target)`` (the local sensor noise floor).
      Anderson acceleration is applied within each fixed-nu stage.

    Parameters
    ----------
    source          : (M,3) source points (e.g. CAD model in model frame)
    target          : (N,3) target points (e.g. segmented scene, camera frame)
    target_normals  : (N,3) unit normals aligned with ``target``
    init            : (4,4) initial source->target transform (e.g. SAM-6D pose)
    anderson_depth  : Anderson history size (0 = plain Gauss-Newton).
    robust          : enable the Welsch kernel + dynamic-nu annealing.
    nu_begin_k, nu_end_k, nu_alpha : Welsch schedule (see NU_* constants).

    Returns (T, info) with T the refined 4x4 and info holding fitness,
    inlier_rmse, iterations, correspondences and convergence flag.
    """
    T0 = np.eye(4) if init is None else init.astype(np.float64).copy()
    info: Dict = {'converged': False, 'iterations': 0,
                  'fitness': 0.0, 'inlier_rmse': float('nan'),
                  'correspondences': 0}

    def _update(n_inl, rmse, it):
        info['iterations'] = it
        info['correspondences'] = n_inl
        info['inlier_rmse'] = rmse
        info['fitness'] = n_inl / len(source) if len(source) else 0.0

    if len(source) == 0 or len(target) == 0:
        return T0, info

    # ---- Fast + Robust (Welsch, dynamic-nu annealing) ----
    if robust:
        # nu_begin from the spread of the initial residuals; nu_end from the
        # target's own local surface noise. nu is clamped to start >= floor.
        R, t = T0[:3, :3], T0[:3, 3]
        src_t = source @ R.T + t
        idx, _d = nearest_neighbor(src_t, target)
        r0 = np.abs(np.einsum('ij,ij->i', src_t - target[idx],
                              target_normals[idx]))
        med = float(np.median(r0)) if len(r0) else 0.0
        nu_end = nu_end_k * welsch_nu_end(target, target_normals)
        nu_end = nu_end if nu_end > 1e-9 else max(med * 1e-2, 1e-4)
        nu = max(nu_begin_k * med, nu_end)
        if not np.isfinite(nu) or nu <= 0:
            nu = nu_end

        T = T0
        total = 0
        inner = 6                                    # reference ramps 6 -> 10
        while total < max_iter:
            budget = min(inner, max_iter - total)
            T, iters = _welsch_stage(source, target, target_normals, T, nu,
                                     budget, anderson_depth)
            total += iters
            if abs(nu - nu_end) < 1e-9:
                info['converged'] = True
                break
            nu = max(nu * nu_alpha, nu_end)
            inner = min(inner + 1, 10)

        n_inl, rmse = _final_metrics(source, target, target_normals, T,
                                     max_corr_dist)
        _update(n_inl, rmse, total)
        return T, info

    # ---- Plain Gauss-Newton (baseline / fallback) ----
    if anderson_depth <= 0:
        T = T0
        prev_rmse = float('inf')
        for it in range(max_iter):
            step = _p2plane_gn_step(source, target, target_normals, T, max_corr_dist)
            if step is None:
                break
            T, _energy, n_inl, rmse, incr = step
            _update(n_inl, rmse, it + 1)
            if abs(prev_rmse - rmse) < tol and incr < tol:
                info['converged'] = True
                break
            prev_rmse = rmse
        return T, info

    # ---- Anderson-accelerated (Fast-ICP) ----
    T0inv = np.eye(4)
    T0inv[:3, :3] = T0[:3, :3].T
    T0inv[:3, 3] = -T0[:3, :3].T @ T0[:3, 3]

    def _fixed_point(xi):
        """One ICP step, in se(3)-increment coordinates around T0."""
        T = _se3_exp(xi) @ T0
        step = _p2plane_gn_step(source, target, target_normals, T, max_corr_dist)
        if step is None:
            return None
        T_next, energy, n_inl, rmse, _incr = step
        return _se3_log(T_next @ T0inv), energy, n_inl, rmse, T_next

    fp = _fixed_point(np.zeros(6))
    if fp is None:
        return T0, info
    g, best_E, n_inl, rmse, best_T = fp
    xs, gs = [np.zeros(6)], [g]                 # history of iterates and images
    _update(n_inl, rmse, 1)
    prev_rmse = rmse

    for it in range(1, max_iter):
        m = min(len(xs), anderson_depth + 1)
        if m >= 2:
            F = np.array([gs[i] - xs[i] for i in range(len(xs) - m, len(xs))])
            G = np.array(gs[len(xs) - m:])
            dF = np.diff(F, axis=0).T           # (6, m-1)
            dG = np.diff(G, axis=0).T
            theta, *_ = np.linalg.lstsq(dF, F[-1], rcond=None)
            cand = gs[-1] - dG @ theta
        else:
            cand = gs[-1]

        fp = _fixed_point(cand)
        if fp is None:
            break
        g_c, E_c, n_c, rmse_c, T_c = fp

        if E_c <= best_E * (1.0 + 1e-9):        # accept the extrapolated iterate
            xs.append(cand)
            gs.append(g_c)
            best_E, n_inl, rmse, best_T = E_c, n_c, rmse_c, T_c
        else:                                   # reject: plain step, reset history
            plain = gs[-1]
            fp2 = _fixed_point(plain)
            if fp2 is None:
                break
            g2, E2, n2, rmse2, T2 = fp2
            xs, gs = [plain], [g2]
            best_E, n_inl, rmse, best_T = E2, n2, rmse2, T2

        if len(xs) > anderson_depth + 1:
            xs, gs = xs[-(anderson_depth + 1):], gs[-(anderson_depth + 1):]

        _update(n_inl, rmse, it + 1)
        if abs(prev_rmse - rmse) < tol:
            info['converged'] = True
            break
        prev_rmse = rmse

    return best_T, info
