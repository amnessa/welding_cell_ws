"""`lit-modelreg` — a basic model registered onto the scan, the seam transferred with it.

A faithful reimplementation, within one documented substitution, of:

    Weihua Fang, Xincheng Tian.
    "A novel model-based welding trajectory planning method for identical structural
    workpieces."
    *Robotics and Computer-Integrated Manufacturing* 89 (2024) 102772.

Their pipeline (§3):

    §3.1  build the BASIC MODEL from the basic workpiece's 3D model: the seam trajectory
          P (extracted offline), the auxiliary trajectory P' (each point offset
          l_p = 15 mm along the local bisector Z_S), and the workpiece feature point set W
                                                            -> `build_model`
    §3.2  register the model point set Y = P u W onto the target's feature point set X by
          Bayesian coherent point drift: a similarity transform plus a coherent
          deformation field (eq. 3, x = sR(y + v) + t)      -> `cpd_similarity`,
                                                               `cpd_nonrigid`
    §3.3  the fitted mapping carries P (and P') onto the target: the target's seam is the
          TRANSFORMED MODEL SEAM, never something detected in the scan
                                                            -> `detect`
    §3.4  torch pose from P and P' (eq. 1)                  -> carried, not scored

Why this method anchors the ladder
----------------------------------
`lit-modelreg` never looks for a seam. It knows one, on a model, and every part of its
accuracy is the registration's. That makes it the one method whose error decomposes into
*registration residual + fit-up deviation between model and workpiece* — the "registration
error propagates into the seam" plot `dataset_plan.md` says only this method can produce —
and the natural top rung of the oracle ladder, held by a real published method rather than
a hypothetical one.

The plan's open item, resolved
------------------------------
*"Verify `lit-modelreg` is implementable without the original CAD assets; if not, it
becomes an L0 upper bound computed from generator transforms rather than a
reimplementation, and must be labelled as such."* Both halves turn out true at once:

* **Implementable — the generator is the CAD.** `scene.json` stores every part's
  `dims_mm` and `T_world_part` plus the assembly's `T_world_joint`, so the basic model is
  rebuilt exactly: slab surfaces sampled analytically in the JOINT frame, the stored truth
  seam mapped into that frame as the model's P. Registration then has real work to do —
  the scan sits at the scene's arbitrary world pose, the model at canonical.
* **And it is still CAD-by-construction, and labelled as such.** The model's seam IS the
  generator's truth (in model frame); nothing here is detected. Every number this method
  produces is an L0-with-CAD number and must be quoted with that label.

One substitution, documented: the paper registers **edge-feature** point sets, whose
extraction from a raw scan it never specifies (its targets carry model-derived features —
"identical workpieces"). Here both sides are voxel-subsampled DENSE surface clouds, which
is the only target representation a scan-only pipeline actually has. And their Bayesian
CPD is implemented as classical CPD (Myronenko) — similarity first, then a coherent
Gaussian-kernel deformation — the same eq.-3 family without the Bayesian priors; the
difference is a robustness refinement, not a different mechanism.

Deterministic: EM from a fixed PCA-based initialisation, no sampling anywhere.
Everything is **millimetres**.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .radius_pca import voxel_downsample

AUX_OFFSET_MM = 15.0          # §3.1, l_p "set to 15 mm based on experience"


# --------------------------------------------------------------------------------
# §3.1 — the basic model, from generator parameters
# --------------------------------------------------------------------------------

def slab_surface_points(dims_mm, T_world_part, step_mm: float = 6.0) -> np.ndarray:
    """Deterministic grid sample of a slab's six faces, transformed by `T_world_part`."""
    L, W, H = [float(d) for d in dims_mm]
    T = np.asarray(T_world_part, dtype=float)
    pts = []
    ax = [np.arange(-L / 2, L / 2 + 1e-9, step_mm),
          np.arange(-W / 2, W / 2 + 1e-9, step_mm),
          np.arange(-H / 2, H / 2 + 1e-9, step_mm)]
    for fixed_axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
        free = [a for a in range(3) if a != fixed_axis]
        u, v = np.meshgrid(ax[free[0]], ax[free[1]])
        face = np.zeros((u.size, 3))
        face[:, free[0]] = u.ravel()
        face[:, free[1]] = v.ravel()
        face[:, fixed_axis] = sign * [L, W, H][fixed_axis] / 2
        pts.append(face)
    P = np.vstack(pts)
    return P @ T[:3, :3].T + T[:3, 3]


def slab_edge_points(dims_mm, T_world_part, step_mm: float = 4.0) -> np.ndarray:
    """The 12 edges of a slab, sampled along their length, transformed by `T_world_part`.

    §3.1 is precise about this and the first implementation here ignored it: W is *"the
    point set generated by the EDGES of the basic workpiece"*, not its surface. The choice
    is load-bearing, measured the hard way — registering dense surfaces let a lap stack
    slide ~5 mm along its overlap direction (big faces dominate the correspondence mass
    and tolerate the slide; only the plate ends resist, and they are a vanishing minority
    of surface points). Edges ARE the ends; make them the whole of W and the slide costs
    what it should.
    """
    L, W, H = [float(d) for d in dims_mm]
    T = np.asarray(T_world_part, dtype=float)
    h = np.array([L, W, H]) / 2.0
    pts = []
    for run_axis in range(3):
        n = max(2, int(np.ceil(2 * h[run_axis] / step_mm)) + 1)
        u = np.linspace(-h[run_axis], h[run_axis], n)
        o1, o2 = [a for a in range(3) if a != run_axis]
        for s1 in (-1, 1):
            for s2 in (-1, 1):
                e = np.zeros((n, 3))
                e[:, run_axis] = u
                e[:, o1] = s1 * h[o1]
                e[:, o2] = s2 * h[o2]
                pts.append(e)
    P = np.vstack(pts)
    return P @ T[:3, :3].T + T[:3, 3]


def _poly_contains(o: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Point-in-convex-CCW-polygon, vectorised over query rows."""
    inside = np.ones(len(q), dtype=bool)
    k = len(o)
    cen = o.mean(axis=0)
    for i in range(k):
        a, b = o[i], o[(i + 1) % k]
        e = b - a
        n2 = np.array([e[1], -e[0]])
        if float((cen - a) @ n2) < 0:
            n2 = -n2
        inside &= (q - a) @ n2 >= -1e-9
    return inside


def prism_surface_points(outline_uv, thickness, T_world_part,
                         step_mm: float = 6.0) -> np.ndarray:
    """`slab_surface_points` for a Phase 6a prism: grid the caps inside the outline,
    grid each side rectangle. Rebuilt from the schema fields (`outline_uv`,
    `thickness_mm`), so the baseline still consumes nothing but scene.json."""
    o = np.asarray(outline_uv, dtype=float)
    t2 = float(thickness) / 2.0
    T = np.asarray(T_world_part, dtype=float)
    pts = []
    gu = np.arange(o[:, 0].min(), o[:, 0].max() + 1e-9, step_mm)
    gv = np.arange(o[:, 1].min(), o[:, 1].max() + 1e-9, step_mm)
    uu, vv = np.meshgrid(gu, gv)
    q = np.column_stack([uu.ravel(), vv.ravel()])
    q = q[_poly_contains(o, q)]
    for w in (t2, -t2):
        pts.append(np.column_stack([q, np.full(len(q), w)]))
    ws = np.arange(-t2, t2 + 1e-9, step_mm)
    if len(ws) < 2:
        ws = np.array([-t2, t2])
    k = len(o)
    for i in range(k):
        a, b = o[i], o[(i + 1) % k]
        L = float(np.linalg.norm(b - a))
        n = max(2, int(np.ceil(L / step_mm)) + 1)
        ts = np.linspace(0.0, 1.0, n)[:, None]
        seg = a + ts * (b - a)
        for w in ws:
            pts.append(np.column_stack([seg, np.full(n, w)]))
    P = np.vstack(pts)
    return P @ T[:3, :3].T + T[:3, 3]


def prism_edge_points(outline_uv, thickness, T_world_part,
                      step_mm: float = 4.0) -> np.ndarray:
    """`slab_edge_points` for a prism: both cap boundaries plus the vertical corners.
    Same reasoning as the slab version - the edges are what resists a slide."""
    o = np.asarray(outline_uv, dtype=float)
    t2 = float(thickness) / 2.0
    T = np.asarray(T_world_part, dtype=float)
    pts = []
    k = len(o)
    nv = max(2, int(np.ceil(2 * t2 / step_mm)) + 1)
    ws = np.linspace(-t2, t2, nv)[:, None]
    for i in range(k):
        a, b = o[i], o[(i + 1) % k]
        L = float(np.linalg.norm(b - a))
        n = max(2, int(np.ceil(L / step_mm)) + 1)
        ts = np.linspace(0.0, 1.0, n)[:, None]
        seg = a + ts * (b - a)
        for w in (-t2, t2):
            pts.append(np.column_stack([seg, np.full(n, w)]))
        pts.append(np.column_stack([np.repeat(a[None, :], nv, axis=0), ws]))
    P = np.vstack(pts)
    return P @ T[:3, :3].T + T[:3, 3]


def build_model(scene: dict, gt_world: list, step_mm: float = 6.0,
                edges_only: bool = True) -> tuple[np.ndarray, list, np.ndarray]:
    """The basic model in the JOINT frame: `(W_points, P_seams, T_world_joint)`.

    Parts are rebuilt from `dims_mm` at `T_world_part`, then pulled back through
    `T_world_joint` so the model sits at canonical pose while the scan sits wherever the
    scene put it — the "identical workpiece, new placement" scenario the paper is for.
    The model's seam trajectories P are the stored truth mapped into the same frame:
    CAD-by-construction, and the reason every number here carries the L0 label.
    """
    Twj = np.asarray(scene["T_world_joint"], dtype=float)
    inv = np.linalg.inv(Twj)
    clouds = []
    for o in scene["objects"]:
        if o.get("role") != "workpiece":
            continue
        Tp = inv @ np.asarray(o["T_world_part"], dtype=float)
        if o.get("primitive", "slab") == "prism":
            if edges_only:
                clouds.append(prism_edge_points(
                    o["outline_uv"], o["thickness_mm"], Tp, min(step_mm, 4.0)))
            else:
                clouds.append(prism_surface_points(
                    o["outline_uv"], o["thickness_mm"], Tp, step_mm))
        elif edges_only:
            clouds.append(slab_edge_points(o["dims_mm"], Tp, min(step_mm, 4.0)))
        else:
            clouds.append(slab_surface_points(o["dims_mm"], Tp, step_mm))
    W = np.vstack(clouds)
    P = [np.asarray(g, dtype=float) @ inv[:3, :3].T + inv[:3, 3] for g in gt_world]
    return W, P, Twj


# --------------------------------------------------------------------------------
# §3.2 — coherent point drift
# --------------------------------------------------------------------------------

def _pca_init(X: np.ndarray, Y: np.ndarray) -> list:
    """Deterministic coarse alignment: principal frames with third-moment sign fixing.

    CPD's EM converges from moderate misalignment, not from an arbitrary yaw, and the
    paper never specifies its initialisation (its workpieces arrive similarly posed).
    Principal axes align the frames; the sign of each axis is fixed by the third central
    moment along it, which is the skewness disambiguation `dataset_plan.md` already
    records as verified. Determinant is forced positive so the init is a rotation.
    """
    def frame(Z):
        c = Z.mean(axis=0)
        _, v = np.linalg.eigh(np.cov((Z - c).T))
        v = v[:, ::-1]
        for k in range(3):
            if np.sum(((Z - c) @ v[:, k]) ** 3) < 0:
                v[:, k] = -v[:, k]
        if np.linalg.det(v) < 0:
            v[:, 2] = -v[:, 2]
        return c, v
    cx, vx = frame(X)
    cy, vy = frame(Y)
    sx = np.sqrt(np.var(X - cx))
    sy = np.sqrt(np.var(Y - cy))
    s = float(sx / max(sy, 1e-12))

    # The third moment fixes each axis sign ONLY when the shape is skewed along it, and a
    # T assembly is near-symmetric along its seam axis - the moment sits at noise level
    # and the sign flips scene to scene, leaving EM in a 180-deg local minimum (measured:
    # 35-108 mm registration residual on a model that overlays the scan at 0,24 mm).
    # So: every proper-rotation sign combination is a candidate, scored by trimmed
    # nearest-neighbour distance, best one wins. Four candidates, deterministic.
    candidates = []
    for fx, fy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        F = np.diag([fx, fy, fx * fy])                 # det = +1 always
        R = vx @ F @ vy.T
        candidates.append((s, R, cx - s * R @ cy))
    return candidates


def cpd_similarity(X: np.ndarray, Y: np.ndarray, iters: int = 60, w: float = 0.1,
                   global_init: bool = True) -> tuple[float, np.ndarray, np.ndarray]:
    """Similarity-transform CPD (Myronenko): `(s, R, t)` with `x ~ s R y + t`.

    The GMM EM of the paper's eq. 4 family: E-step soft correspondences with a uniform
    outlier component of weight `w`; M-step the weighted Procrustes solution. Initialised
    by `_pca_init`, so the whole fit is deterministic.
    """
    X = np.asarray(X, dtype=float)
    Y = np.asarray(Y, dtype=float)
    N, M = len(X), len(Y)
    from scipy.spatial import cKDTree
    tree = cKDTree(X)

    # Every proper-rotation PCA init is refined briefly and the winner is judged on its
    # REFINED residual, not its initial one. A near-symmetric assembly (two stacked or
    # flush plates) has counterpart poses whose coarse NN score ties with the truth; only
    # after a few EM steps does the true pose pull ahead on the small asymmetries - the
    # dim difference between the plates, the fit-up defects. Picking on the coarse score
    # sent edge joints to the flipped pose, a plate-length of seam error (measured: 119 mm).
    def refine(s, R, t, n_it):
        TY = s * Y @ R.T + t
        # sigma2 from NEAREST-NEIGHBOUR residuals, not the all-pairs mean: the all-pairs
        # mean is ~object-size^2, the first E-step goes uniform, the centered
        # cross-covariance cancels, and the scale collapses to ~0,14 - measured, from an
        # init already within 2,7 degrees. Starting near the answer, sigma2 must say so.
        d0, _ = tree.query(TY, k=1, workers=-1)
        sigma2 = max(float(np.mean(d0 ** 2)), 1.0)
        for _ in range(n_it):
            s, R, t, sigma2, TY = _em_step(X, Y, s, R, t, sigma2, w)
        d, _ = tree.query(TY, k=1, workers=-1)
        return (s, R, t, sigma2), float(np.mean(d ** 2))

    if global_init:
        best, best_score = None, np.inf
        for cand in _pca_init(X, Y):
            ref, score = refine(*cand, n_it=max(15, iters // 3))
            if score < best_score:
                best_score, best = score, ref
        s, R, t, sigma2 = best
    else:
        (s, R, t, sigma2), _ = refine(1.0, np.eye(3), np.zeros(3), n_it=5)
    for _ in range(iters):
        s, R, t, sigma2, _ = _em_step(X, Y, s, R, t, sigma2, w)
    return s, R, t


def _em_step(X, Y, s, R, t, sigma2, w):
    """One CPD similarity EM step. Returns `(s, R, t, sigma2, TY)`."""
    N, M = len(X), len(Y)
    TY = s * Y @ R.T + t
    d2 = np.sum((X[:, None, :] - TY[None, :, :]) ** 2, axis=2)
    c = (2 * np.pi * sigma2) ** 1.5 * (w / max(1 - w, 1e-9)) * M / N
    P = np.exp(-d2 / (2 * sigma2))
    P = P / (P.sum(axis=1, keepdims=True) + c + 1e-300)
    Np = P.sum()
    if Np < 1e-9:
        return s, R, t, sigma2, TY
    px = P.sum(axis=1)
    py = P.sum(axis=0)
    mx = (px @ X) / Np
    my = (py @ Y) / Np
    Xh = X - mx
    Yh = Y - my
    A = Xh.T @ P @ Yh
    U, S_, Vt = np.linalg.svd(A)
    C = np.eye(3)
    C[2, 2] = np.linalg.det(U @ Vt)
    R = U @ C @ Vt
    denom = float(np.einsum("m,mj,mj->", py, Yh, Yh))
    s = float(np.trace(np.diag(S_) @ C)) / max(denom, 1e-12)
    t = mx - s * R @ my
    TY = s * Y @ R.T + t
    sigma2 = max(float((np.einsum("n,nj,nj->", px, Xh, Xh)
                        - s * np.trace(np.diag(S_) @ C)) / (3 * Np)), 1e-6)
    return s, R, t, sigma2, TY


def cpd_nonrigid(X: np.ndarray, Y: np.ndarray, beta_mm: float = 40.0, lam: float = 3.0,
                 iters: int = 30, w: float = 0.1
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Coherent (Gaussian-kernel) deformation on top of an aligned pair: `(TY, G_solve, Wc)`.

    Eq. 3's `v` term: `T(y) = y + G(y, Y) Wc` with `G = exp(-|y_i - y_j|^2 / (2 beta^2))`.
    `beta_mm` sets deformation smoothness at the physical scale of fit-up deviation —
    tens of millimetres of coherence, millimetres of amplitude — and `lam` weights the
    regulariser. The returned kernel machinery maps ANY point set through the fitted
    field, which is exactly how the seam P rides along (§3.3).
    """
    X = np.asarray(X, dtype=float)
    Y = np.asarray(Y, dtype=float)
    M = len(Y)
    G = np.exp(-np.sum((Y[:, None, :] - Y[None, :, :]) ** 2, axis=2) / (2 * beta_mm ** 2))
    Wc = np.zeros((M, 3))
    TY = Y.copy()
    from scipy.spatial import cKDTree
    d0, _ = cKDTree(X).query(TY, k=1, workers=-1)
    sigma2 = max(float(np.mean(d0 ** 2)), 1.0)         # same NN-residual init as above
    for _ in range(iters):
        d2 = np.sum((X[:, None, :] - TY[None, :, :]) ** 2, axis=2)
        c = (2 * np.pi * sigma2) ** 1.5 * (w / max(1 - w, 1e-9)) * M / len(X)
        P = np.exp(-d2 / (2 * sigma2))
        P = P / (P.sum(axis=1, keepdims=True) + c + 1e-300)
        dP = P.sum(axis=0)
        A = np.diag(dP) @ G + lam * sigma2 * np.eye(M)
        B = P.T @ X - np.diag(dP) @ Y
        Wc = np.linalg.solve(A, B)
        TY = Y + G @ Wc
        Np = P.sum()
        if Np < 1e-9:
            break
        sigma2 = max(float((np.einsum("nm,nj,nj->", P, X, X) / Np
                            - 2 * np.einsum("nm,nj,mj->", P, X, TY) / Np
                            + np.einsum("nm,mj,mj->", P, TY, TY) / Np) / 3), 1e-6)
    return TY, Y, Wc


def _apply_field(pts: np.ndarray, Y: np.ndarray, Wc: np.ndarray, beta_mm: float
                 ) -> np.ndarray:
    """Carry arbitrary points through the fitted deformation field (the seam transfer)."""
    G = np.exp(-np.sum((pts[:, None, :] - Y[None, :, :]) ** 2, axis=2)
               / (2 * beta_mm ** 2))
    return pts + G @ Wc


# --------------------------------------------------------------------------------
# result and driver
# --------------------------------------------------------------------------------

@dataclass
class ModelRegResult:
    """What `lit-modelreg` returned. Every number is L0-with-CAD; see the module header."""

    seams: list[np.ndarray]               #: the model's P, carried through the fit
    model_points: np.ndarray              #: Y after registration (for the figures)
    s: float = 1.0
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))
    params: dict[str, Any] = field(default_factory=dict)
    used_model_oracle: bool = True        #: constitutively true - the method IS the oracle
    note: str = ""

    @property
    def polylines(self) -> list[np.ndarray]:
        return self.seams

    @property
    def n_seams(self) -> int:
        return len(self.seams)


def _cap(pts: np.ndarray, n_max: int) -> np.ndarray:
    """Deterministic stride subsample. CPD's E-step is an N x M distance matrix per
    iteration; uncapped voxel grids left several thousand points a side and a single
    scene's registration did not finish. ~700 points a side registers a slab assembly to
    sub-millimetre in under a second."""
    if len(pts) <= n_max:
        return pts
    return pts[:: int(np.ceil(len(pts) / n_max))]


def _near_init_pose(scene: dict, rot_deg: float, trans_mm: float) -> np.ndarray:
    """A deterministic moderate offset composed onto the true pose — the paper's envelope.

    "Identical structural workpieces" arrive at a workstation roughly positioned; their
    BCPD refines a pose that is already close, and the paper never faces (or claims to
    face) an arbitrary-rotation registration. The same assumption is already in this
    plan's spine — D26's coarse positioning, *"önce bir kaynak bölgesine git"* — so the
    default arm perturbs the true pose by `rot_deg` / `trans_mm` (seeded from `scene_id`,
    reproducible) and lets registration close that gap. The `init="global"` arm removes
    the assumption and shows what breaks; see `detect`.
    """
    rng = np.random.default_rng(abs(hash(scene["scene_id"])) % (2 ** 32))
    ax = rng.normal(size=3)
    ax /= np.linalg.norm(ax)
    a = np.radians(rot_deg)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = rng.normal(size=3) / np.sqrt(3) * trans_mm
    return np.asarray(scene["T_world_joint"], dtype=float) @ np.linalg.inv(T)


def detect(pts: np.ndarray, scene: dict, gt_world: list,
           mode: str = "nonrigid", init: str = "near", target_features: str = "oracle",
           init_rot_deg: float = 10.0,
           init_trans_mm: float = 20.0, model_step_mm: float = 5.0,
           target_voxel_mm: float = 6.0, n_max: int = 900, beta_mm: float = 40.0,
           lam: float = 3.0, iters: int = 60, w: float = 0.1) -> ModelRegResult:
    """Register the basic model onto the scan and carry its seam across.

    Args:
        pts: the observed cloud, world frame.
        scene / gt_world: `scene.json` and the truth polylines — the CAD-by-construction
            inputs the model is built from. **This is what makes the method L0**: the seam
            is never detected, only transferred, so its world-frame truth enters through
            the model side and every output must be quoted with that label.
        mode: `"similarity"` (rigid + scale) or `"nonrigid"` (similarity, then the
            coherent field — the paper's eq. 3).
        init: `"near"` (default) poses the model at a deterministic moderate offset from
            the true pose — the paper's own operating envelope, and D26's coarse-
            positioning assumption. `"global"` starts from canonical pose with PCA
            candidate initialisation: **outside the paper's envelope**, and where the
            near-symmetry failure lives — a flush or stacked assembly registers onto its
            symmetric counterpart with a residual the surface can barely distinguish, and
            the seam lands a plate-length away. Report `"global"` numbers only as that
            finding, never as the method's accuracy.
    """
    pts = np.asarray(pts, dtype=float)
    Wm, Pm, _ = build_model(scene, gt_world, model_step_mm,
                            edges_only=(target_features == "oracle"))
    X = _cap(voxel_downsample(pts, float(target_voxel_mm)), n_max)
    Y = _cap(Wm, n_max) if target_features == "oracle" \
        else _cap(voxel_downsample(Wm, float(target_voxel_mm)), n_max)
    params = dict(mode=mode, model_step_mm=model_step_mm, target_voxel_mm=target_voxel_mm,
                  beta_mm=beta_mm, lam=lam, n_target=len(X), n_model=len(Y))

    if len(X) < 10 or len(Y) < 10:
        return ModelRegResult([], np.zeros((0, 3)), params=params, note="too few points")

    params["init"] = init
    params["target_features"] = target_features
    if target_features == "oracle":
        # The paper's X is the TARGET's feature (edge) point set - and its targets carry
        # model-derived features, because the workpieces are identical. A scan-only
        # pipeline has no such stage, so it is supplied the way every other method's
        # learned stage is supplied here: as an explicit oracle. Scan points within reach
        # of the true-posed model edges stand in for it, and every number produced this
        # way carries that label. `target_features="dense"` withholds the stage and
        # registers raw surfaces - which is where the slide failures live.
        Twj = np.asarray(scene["T_world_joint"], dtype=float)
        Ew = Wm @ Twj[:3, :3].T + Twj[:3, 3]
        from scipy.spatial import cKDTree
        d, _ = cKDTree(Ew).query(pts[:: max(1, len(pts) // 60000)], k=1, workers=-1)
        near = pts[:: max(1, len(pts) // 60000)][d <= 2.0 * float(target_voxel_mm)]
        if len(near) >= 30:
            X = _cap(voxel_downsample(near, float(target_voxel_mm) / 2.0), n_max)
    elif target_features != "dense":
        raise ValueError(f'target_features must be "oracle" or "dense", '
                         f'got {target_features!r}')
    if init == "near":
        T0 = _near_init_pose(scene, init_rot_deg, init_trans_mm)
        Y = Y @ T0[:3, :3].T + T0[:3, 3]
        Pm = [p @ T0[:3, :3].T + T0[:3, 3] for p in Pm]
        s, R, t = cpd_similarity(X, Y, iters, w, global_init=False)
    elif init == "global":
        s, R, t = cpd_similarity(X, Y, iters, w, global_init=True)
    else:
        raise ValueError(f'init must be "near" or "global", got {init!r}')
    TY = s * Y @ R.T + t
    seams = [s * p @ R.T + t for p in Pm]

    if mode == "nonrigid":
        TY2, Ybase, Wc = cpd_nonrigid(X, TY, beta_mm, lam, max(8, iters // 3), w)
        seams = [_apply_field(p, Ybase, Wc, beta_mm) for p in seams]
        TY = TY2
    elif mode != "similarity":
        raise ValueError(f'mode must be "similarity" or "nonrigid", got {mode!r}')

    return ModelRegResult(seams, TY, s, R, t, params)
