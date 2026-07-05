"""Pure-NumPy point-to-plane ICP and the geometry helpers it needs.

No open3d / scipy / sklearn dependency (they are not installed on the robot
side). Everything here is plain NumPy so it can run inside a rclpy node and be
unit-tested without ROS.

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


# ──────────────────────────── nearest neighbour ───────────────────────────
def nearest_neighbor(src: np.ndarray, dst: np.ndarray,
                     chunk: int = 512) -> Tuple[np.ndarray, np.ndarray]:
    """Brute-force NN from each src point to dst. Returns (idx, dist).

    Chunked over src to bound peak memory (clouds here are small after voxel
    downsampling, so O(M*N) is fine and needs no KD-tree).
    """
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


def icp_point_to_plane(source: np.ndarray, target: np.ndarray,
                       target_normals: np.ndarray,
                       init: Optional[np.ndarray] = None,
                       max_corr_dist: float = 0.01,
                       max_iter: int = 40,
                       tol: float = 1e-6) -> Tuple[np.ndarray, Dict]:
    """Refine a source->target rigid transform minimizing point-to-plane error.

    Minimizes sum_i (((R p_i + t) - q_i) . n_i)^2 over correspondences, where
    q_i,n_i are the nearest target point and its normal. Linearized (small-angle)
    Gauss-Newton, solved with numpy.lstsq each iteration.

    Parameters
    ----------
    source          : (M,3) source points (e.g. CAD model in model frame)
    target          : (N,3) target points (e.g. segmented scene, camera frame)
    target_normals  : (N,3) unit normals aligned with ``target``
    init            : (4,4) initial source->target transform (e.g. SAM-6D pose)

    Returns (T, info) with T the refined 4x4 and info holding fitness,
    inlier_rmse, iterations, correspondences and the applied delta.
    """
    T = np.eye(4) if init is None else init.astype(np.float64).copy()
    info: Dict = {'converged': False, 'iterations': 0,
                  'fitness': 0.0, 'inlier_rmse': float('nan'),
                  'correspondences': 0}
    prev_rmse = float('inf')
    for it in range(max_iter):
        R, t = T[:3, :3], T[:3, 3]
        src_t = source @ R.T + t
        idx, dist = nearest_neighbor(src_t, target)
        inl = dist < max_corr_dist
        n_inl = int(inl.sum())
        info['iterations'] = it + 1
        info['correspondences'] = n_inl
        if n_inl < 6:
            break

        p = src_t[inl]
        q = target[idx[inl]]
        nq = target_normals[idx[inl]]
        # A x = b, x = [wx,wy,wz, tx,ty,tz]; row_i = [p_i x n_i, n_i]
        A = np.hstack((np.cross(p, nq), nq))
        b = -np.einsum('ij,ij->i', p - q, nq)
        x, *_ = np.linalg.lstsq(A, b, rcond=None)
        w, dt = x[:3], x[3:]

        Td = np.eye(4)
        Td[:3, :3] = _rodrigues(w)
        Td[:3, 3] = dt
        T = Td @ T

        rmse = float(np.sqrt(np.mean(dist[inl] ** 2)))
        info['inlier_rmse'] = rmse
        info['fitness'] = n_inl / len(source)
        if abs(prev_rmse - rmse) < tol and float(np.linalg.norm(x)) < tol:
            info['converged'] = True
            break
        prev_rmse = rmse
    return T, info
