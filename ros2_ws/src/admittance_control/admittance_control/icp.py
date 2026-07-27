"""Pure-NumPy point-to-plane ICP and the geometry helpers it needs.

Runs with plain NumPy alone (no hard open3d / scipy / sklearn dependency, so it
works on the dependency-light robot side and is unit-testable without ROS).
Nearest-neighbour search picks the fastest available backend at runtime --
Open3D's ``core.nns``, then ``scipy.spatial.cKDTree``, then chunked brute force
-- via the ``NNIndex`` class. Everything else stays pure NumPy.

Pipeline the icp_pose_refiner_node builds on top of this:
  - load_ply_mesh + sample_mesh_surface : CAD .ply -> model point cloud
  - backproject_depth                   : depth image + K -> organized cloud
  - estimate_normals_organized          : organized cloud -> per-pixel normals
    (cross product of pixel-neighbour gradients; no KD-tree needed)
  - infer_pinhole_from_organized +
    box_image_roi                        : project the crop box into the image so
    only its pixel rectangle is unpacked (the tracking-loop front-end speedup)
  - voxel_downsample                     : thin dense clouds before ICP
  - icp_point_to_plane                   : refine a model->scene transform

Frames/units: everything is metres. The SAM-6D pose (model->camera, t in mm) is
converted to metres and passed as ``init`` to icp_point_to_plane; the returned
4x4 is the refined model->camera transform.

Performance notes (measured on a 640x480 organized cloud, 2500-point CAD, a
~2000-point cropped scene -- the real-time tracking case):
  * ICP is ~80% of a tracking tick, and ICP is almost entirely nearest-neighbour
    lookup: ~43 queries per call against the *same* target. Hence ``NNIndex``,
    built once per ICP call and reused by every iteration.
  * Open3D's nns is ~3x faster than cKDTree at these sizes (0.31 vs 1.02 ms) and
    returns identical indices; cKDTree's ``workers=-1`` is *slower* than a small
    fixed worker count because of thread-dispatch overhead.
  * Brute force is not competitive here (19+ ms) -- it stays only as the
    no-dependency fallback.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:  # optional: KD-tree accelerates nearest-neighbour search when available
    from scipy.spatial import cKDTree as _cKDTree
except Exception:  # noqa: BLE001 - no scipy -> brute-force fallback
    _cKDTree = None

try:  # optional: Open3D's nns is ~3x faster than cKDTree at tracking sizes
    import open3d as _o3d
    import open3d.core as _o3c
    _o3d_nns = _o3d.core.nns
except Exception:  # noqa: BLE001 - no open3d -> scipy / brute force
    _o3c = None
    _o3d_nns = None


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
    # Flatten the 3-D voxel key into a single int64 before np.unique. The obvious
    # np.unique(keys, axis=0) lexsorts a 2-D array, which is ~7x slower (4.2 ms
    # vs 0.6 ms on a 7.6k-point crop) for bit-identical output. Shifting by the
    # per-axis minimum keeps the indices non-negative so the mixed-radix encoding
    # below stays monotone and collision-free.
    keys -= keys.min(axis=0)
    dims = (keys.max(axis=0) + 1).astype(object)    # python ints: no wraparound
    if dims[0] * dims[1] * dims[2] < (1 << 62):
        d1, d2 = int(dims[1]), int(dims[2])
        flat = (keys[:, 0] * d1 + keys[:, 1]) * d2 + keys[:, 2]
        _, inv = np.unique(flat, return_inverse=True)
    else:
        # Absurd extent relative to the voxel (non-finite input would do this):
        # the flat key would overflow int64 and alias distinct voxels together,
        # so pay for the lexsort rather than silently merge them.
        _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)               # numpy>=2 keeps the input's shape
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


def infer_pinhole_from_organized(xyz: np.ndarray, stride: int = 8,
                                 max_resid_px: float = 0.5
                                 ) -> Optional[np.ndarray]:
    """Recover the 3x3 pinhole K of an organized, back-projected cloud, or None.

    An organized cloud out of ``depth_image_proc`` is exactly ``backproject_depth``
    run on the depth image, so pixel (u,v) and its point (x,y,z) satisfy
    ``u = fx*(x/z) + cx`` and ``v = fy*(y/z) + cy``. Both are straight lines, so
    two 1-D least-squares fits over a strided sample recover K without needing a
    CameraInfo subscription -- and, being read off the cloud itself, it can never
    disagree with the cloud actually being processed.

    The fit is verified against ``max_resid_px``: a cloud that is not a pinhole
    projection (or is too empty/degenerate to tell) returns None, and the caller
    falls back to the exhaustive path rather than cropping to a wrong rectangle.
    """
    if xyz.ndim != 3 or xyz.shape[2] != 3:
        return None
    h, w, _ = xyz.shape
    stride = max(int(stride), 1)
    sub = xyz[::stride, ::stride]
    us, vs = np.meshgrid(np.arange(0, w, stride, dtype=np.float64),
                         np.arange(0, h, stride, dtype=np.float64))
    z = sub[..., 2]
    with np.errstate(invalid='ignore'):
        good = np.isfinite(sub).all(axis=2) & (z > 1e-6)
    if int(good.sum()) < 50:
        return None
    a = (sub[..., 0][good] / z[good])            # x/z
    b = (sub[..., 1][good] / z[good])            # y/z
    u, v = us[good], vs[good]
    # Need real spread along both axes or the slope is unidentifiable.
    if np.ptp(a) < 1e-3 or np.ptp(b) < 1e-3:
        return None
    K = np.eye(3)
    for vals, pix, (f_i, f_j), (c_i, c_j) in (
            (a, u, (0, 0), (0, 2)), (b, v, (1, 1), (1, 2))):
        A = np.stack((vals, np.ones_like(vals)), axis=1)
        (slope, offset), *_ = np.linalg.lstsq(A, pix, rcond=None)
        if not np.isfinite(slope) or abs(slope) < 1e-6:
            return None
        resid = float(np.max(np.abs(A @ np.array([slope, offset]) - pix)))
        if resid > max_resid_px:
            return None
        K[f_i, f_j], K[c_i, c_j] = slope, offset
    return K


def box_image_roi(shape: Tuple[int, int], T_box: np.ndarray,
                  min_corner: np.ndarray, max_corner: np.ndarray,
                  K: np.ndarray, pad: int = 2
                  ) -> Optional[Tuple[int, int, int, int]]:
    """Pixel rectangle (v0, v1, u0, u1) containing the crop box's projection.

    The crop box is a bounded 3-D region, so a pinhole camera sees it inside a
    bounded *rectangle*. Slicing the organized cloud to that rectangle before
    unpacking, finiteness-testing and oriented-cropping it is exact -- the
    surviving point set is bit-identical -- but only touches the pixels that
    could possibly contribute, which is the tracking loop's front-end speedup
    (38 ms -> 5 ms measured on a 640x480 cloud).

    Returns None when the box is at or behind the image plane (its projection is
    then unbounded) or misses the image entirely, so the caller can fall back to
    scanning the whole cloud instead of guessing a rectangle.
    """
    h, w = int(shape[0]), int(shape[1])
    corners = np.array([[x, y, z] for x in (min_corner[0], max_corner[0])
                        for y in (min_corner[1], max_corner[1])
                        for z in (min_corner[2], max_corner[2])])
    cam = corners @ T_box[:3, :3].T + T_box[:3, 3]     # box frame -> camera
    zc = cam[:, 2]
    if not np.all(np.isfinite(cam)) or np.any(zc <= 1e-6):
        # Straddling the image plane: the projection is not a bounded rectangle.
        return None
    u = K[0, 0] * cam[:, 0] / zc + K[0, 2]
    v = K[1, 1] * cam[:, 1] / zc + K[1, 2]
    u0 = max(int(np.floor(u.min())) - pad, 0)
    u1 = min(int(np.ceil(u.max())) + pad + 1, w)
    v0 = max(int(np.floor(v.min())) - pad, 0)
    v1 = min(int(np.ceil(v.max())) + pad + 1, h)
    if u0 >= u1 or v0 >= v1:
        return None                                    # box is off-image
    return v0, v1, u0, u1


# ──────────────────────────── nearest neighbour ───────────────────────────
# cKDTree's workers=-1 spawns one thread per core, and for the few-thousand-point
# queries ICP issues the dispatch overhead costs more than the parallelism buys
# (1.02 ms at workers=-1 vs 0.57 ms at workers=4). A small fixed count wins.
NN_WORKERS = 4


def _brute_nn(src: np.ndarray, dst: np.ndarray, chunk: int = 512
              ) -> Tuple[np.ndarray, np.ndarray]:
    """Chunked brute-force O(M*N) NN. Fallback when neither backend is present."""
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


class NNIndex:
    """A nearest-neighbour acceleration structure built once over a fixed target.

    ICP queries the *same* target cloud once per iteration -- ~43 times per call
    with the default robust/Anderson settings -- so the structure is built once
    here and reused, instead of being rebuilt inside every query.

    Backend is chosen at construction, fastest first:
      * ``open3d.core.nns`` -- ~3x faster than cKDTree at tracking cloud sizes
        (0.31 ms vs 1.02 ms for 2500 queries into 2247 points) and returns
        identical indices. It works in float32 and reports *squared* distances,
        both of which are normalized away here.
      * ``scipy.spatial.cKDTree`` -- the previous default.
      * chunked brute force -- keeps the module dependency-free.

    ``backend`` names the one in use, for logging.
    """

    def __init__(self, dst: np.ndarray, workers: int = NN_WORKERS) -> None:
        self._dst = np.ascontiguousarray(dst, dtype=np.float64)
        self._workers = int(workers)
        self._impl = None
        self.backend = 'brute'
        if len(self._dst) == 0:
            return
        if _o3d_nns is not None:
            try:
                index = _o3d_nns.NearestNeighborSearch(
                    _o3c.Tensor(np.ascontiguousarray(dst, dtype=np.float32)))
                if index.knn_index():
                    self._impl, self.backend = index, 'open3d'
                    return
            except Exception:  # noqa: BLE001 - any o3d failure -> next backend
                self._impl = None
        if _cKDTree is not None:
            self._impl, self.backend = _cKDTree(self._dst), 'scipy'

    def __len__(self) -> int:
        return len(self._dst)

    def knn(self, src: np.ndarray, k: int = 1) -> Tuple[np.ndarray, np.ndarray]:
        """(idx, dist) of the ``k`` nearest target points, both (len(src), k)."""
        n = len(src)
        k = max(int(k), 1)
        if n == 0 or len(self._dst) == 0:
            return (np.empty((n, k), dtype=np.int64),
                    np.empty((n, k), dtype=np.float64))
        k = min(k, len(self._dst))
        if self.backend == 'open3d':
            idx, d2 = self._impl.knn_search(
                _o3c.Tensor(np.ascontiguousarray(src, dtype=np.float32)), k)
            idx = idx.numpy().reshape(n, k).astype(np.int64)
            # open3d reports SQUARED distances in float32
            dist = np.sqrt(np.maximum(
                d2.numpy().reshape(n, k).astype(np.float64), 0.0))
            return idx, dist
        if self.backend == 'scipy':
            dist, idx = self._impl.query(src, k=k, workers=self._workers)
            return (np.asarray(idx, dtype=np.int64).reshape(n, k),
                    np.asarray(dist, dtype=np.float64).reshape(n, k))
        if k == 1:
            idx, dist = _brute_nn(src, self._dst)
            return idx.reshape(n, 1), dist.reshape(n, 1)
        d2 = (np.einsum('ij,ij->i', src, src)[:, None]
              - 2.0 * src @ self._dst.T
              + np.einsum('ij,ij->i', self._dst, self._dst)[None, :])
        idx = np.argsort(d2, axis=1)[:, :k]
        return idx, np.sqrt(np.maximum(np.take_along_axis(d2, idx, 1), 0.0))

    def query(self, src: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(idx, dist) of the single nearest target point, both (len(src),)."""
        idx, dist = self.knn(src, 1)
        return idx[:, 0], dist[:, 0]


def nearest_neighbor(src: np.ndarray, dst: np.ndarray,
                     chunk: int = 512) -> Tuple[np.ndarray, np.ndarray]:
    """NN index + distance from each src point to dst. Returns (idx, dist).

    One-shot convenience wrapper: it builds an :class:`NNIndex` and throws it
    away. Fine for a single query (the node's background subtraction), but inside
    an iterative solver build the index once and call ``query`` on it instead.
    """
    return NNIndex(dst).query(src) if len(dst) else (
        np.empty(len(src), dtype=np.int64), np.empty(len(src), dtype=np.float64))


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


def _p2plane_gn_step(source, target, target_normals, T, max_corr_dist, index):
    """One Gauss-Newton point-to-plane iteration from pose ``T``.

    ``index`` is a prebuilt :class:`NNIndex` over ``target`` -- the caller owns it
    so the whole solve shares one structure instead of rebuilding it per step.

    Returns (T_next, energy, n_inl, rmse, incr) where ``energy`` is the mean
    squared point-to-plane residual *at* ``T`` (used for the Anderson safeguard),
    ``rmse`` the inlier RMSE at ``T`` and ``incr`` the update norm. Returns None
    if there are too few correspondences to solve.
    """
    R, t = T[:3, :3], T[:3, 3]
    src_t = source @ R.T + t
    idx, dist = index.query(src_t)
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
                  k: int = 7, index: Optional['NNIndex'] = None) -> float:
    """Local surface-noise scale used for the Welsch nu floor (nu_end).

    Port of ``FindKnearestNormMed`` in the reference: for every target point,
    take its ``k`` nearest neighbours and measure how far they lie *along the
    point's own normal* (the point-to-plane thickness of the local surface),
    reduce with the median, then take the median over all points. Multiplying
    this by ``NU_END_K`` gives the smallest Welsch bandwidth we anneal down to --
    it stops the kernel from shrinking below the sensor's own noise floor.

    This measures the *sensor's* noise, not the object's pose, so it barely moves
    from frame to frame -- see the tracking loop, which caches the result across
    ticks rather than paying ~2.5 ms for it every time.
    """
    n = len(target)
    if n < 2:
        return 0.0
    kk = min(int(k), n)
    if index is None and (_o3d_nns is not None or _cKDTree is not None):
        index = NNIndex(target)
    if index is not None and index.backend != 'brute':
        idx, _ = index.knn(target, kk)
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


def _welsch_step(source, target, target_normals, T, nu, index):
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
    idx, _dist = index.query(src_t)
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
                  max_iter, anderson_depth, index):
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
        step = _welsch_step(source, target, target_normals, T, nu, index)
        if step is None:
            return None
        T_next, energy, _r = step
        return _se3_log(T_next @ T0inv), energy, T_next

    # Plain reweighted iteration (no Anderson).
    if anderson_depth <= 0:
        T = T_start
        for it in range(max_iter):
            step = _welsch_step(source, target, target_normals, T, nu, index)
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


def _final_metrics(source, target, target_normals, T, max_corr_dist, index):
    """Fitness / inlier-RMSE / correspondences of ``T`` (point-to-plane)."""
    R, t = T[:3, :3], T[:3, 3]
    src_t = source @ R.T + t
    idx, dist = index.query(src_t)
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
                       nu_alpha: float = NU_ALPHA,
                       noise_floor: Optional[float] = None,
                       index: Optional['NNIndex'] = None
                       ) -> Tuple[np.ndarray, Dict]:
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
    noise_floor     : precomputed ``welsch_nu_end(target, target_normals)``. It
                      characterizes the *sensor*, not the pose, so a tracking
                      loop can measure it once and hand it back every frame
                      instead of paying ~2.5 ms per call to re-derive it.
    index           : prebuilt :class:`NNIndex` over ``target``. Built here when
                      omitted; pass one only if you already have it.

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

    # One acceleration structure for the whole solve. Every iteration queries the
    # same target, so rebuilding it per step (as this used to) was pure overhead.
    if index is None:
        index = NNIndex(target)

    # ---- Fast + Robust (Welsch, dynamic-nu annealing) ----
    if robust:
        # nu_begin from the spread of the initial residuals; nu_end from the
        # target's own local surface noise. nu is clamped to start >= floor.
        R, t = T0[:3, :3], T0[:3, 3]
        src_t = source @ R.T + t
        idx, _d = index.query(src_t)
        r0 = np.abs(np.einsum('ij,ij->i', src_t - target[idx],
                              target_normals[idx]))
        med = float(np.median(r0)) if len(r0) else 0.0
        floor = (float(noise_floor) if noise_floor is not None
                 else welsch_nu_end(target, target_normals, index=index))
        nu_end = nu_end_k * floor
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
                                     budget, anderson_depth, index)
            total += iters
            if abs(nu - nu_end) < 1e-9:
                info['converged'] = True
                break
            nu = max(nu * nu_alpha, nu_end)
            inner = min(inner + 1, 10)

        n_inl, rmse = _final_metrics(source, target, target_normals, T,
                                     max_corr_dist, index)
        _update(n_inl, rmse, total)
        return T, info

    # ---- Plain Gauss-Newton (baseline / fallback) ----
    if anderson_depth <= 0:
        T = T0
        prev_rmse = float('inf')
        for it in range(max_iter):
            step = _p2plane_gn_step(source, target, target_normals, T,
                                    max_corr_dist, index)
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
        step = _p2plane_gn_step(source, target, target_normals, T,
                                max_corr_dist, index)
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
