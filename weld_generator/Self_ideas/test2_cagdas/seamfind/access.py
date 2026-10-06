"""Stage 2 — accessibility. 2a: coarse ray prefilter; 2c: torch-cone check on seam points.

2a uses the plan's REAL-SCAN option (sphere tracing on the unsigned distance of the merged
cloud), not mesh ray casting: in the benchmark the meshes are the generator's ground
truth, and a method that ray-casts them is reading the answer. Two details the plan leaves
open, fixed so the test is not trivially "hit" at the joint:
  * a near point is a hit only when it lies AHEAD of the ray (the plan's p + eps n start
    sits inside the hit threshold of the point's own surface, and of the other part at
    the root - points behind the ray are the surfaces it is leaving);
  * a hit is distance < `ray_hit_h * h` to the merged cloud (density-tied, risk table).

2c uses the generator's torch model (scene.json `accessibility`): a clearance cone of
half-angle 30 deg reaching `standoff` mm from the seam point, tilted at most 45 deg off the
bisector in work angle, a few travel angles. A direction is feasible when no cloud point
lies inside its cone (beyond 2h of the seam point, which is the joint itself).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from .geom import unit


def _frame(n):
    a = np.where(np.abs(n[:, :1]) < 0.9, [[1.0, 0, 0]], [[0, 1.0, 0]])
    e1 = unit(np.cross(n, a))
    return e1, np.cross(n, e1)


def cone_rays(n, n_rays, half_deg, seed=0):
    """`n_rays` unit directions per normal: two rings (0.5 and 1.0 of the half-angle) plus
    the normal itself, fixed azimuth phase -> deterministic."""
    e1, e2 = _frame(n)
    k1 = max(1, (n_rays - 1) // 3)
    k2 = n_rays - 1 - k1
    dirs = [n]
    for frac, k in ((0.5, k1), (1.0, k2)):
        t = np.radians(half_deg * frac)
        for a in np.linspace(0, 2 * np.pi, k, endpoint=False) + 0.1:
            dirs.append(np.cos(t) * n + np.sin(t) * (np.cos(a) * e1 + np.sin(a) * e2))
    return np.stack(dirs, 1)          # (N, R, 3)


def free_ray_fraction(P, N, tree, h, n_rays=24, half_deg=60.0, start_h=0.05, hit_h=0.5,
                      free_mm=40.0, max_steps=60, cloud=None):
    """Fraction of cone rays around each point's normal that escape `free_mm` (sphere
    tracing on the merged cloud, step = distance to the nearest point).

    The ray starts at p + eps n (plan: eps = 0.05h). A near point counts as a hit only if
    it lies AHEAD of the ray ((q - x) . d > 0): the point's own surface and a wall the ray
    is leaving are behind it, so a root point is not "hit" by the joint it sits on, while
    a buried-face strip (facing the other part across a sub-mm gap) hits it at once."""
    D = cone_rays(N, n_rays, half_deg)
    n, R = D.shape[:2]
    O = np.repeat((P + start_h * h * N)[:, None, :], R, 1).reshape(-1, 3)
    D = D.reshape(-1, 3)
    t = np.zeros(len(O))
    alive = np.ones(len(O), bool)
    hit = np.zeros(len(O), bool)
    hit_thr, min_step = hit_h * h, 0.5 * h
    for _ in range(max_steps):
        idx = np.flatnonzero(alive)
        if not len(idx):
            break
        X = O[idx] + t[idx, None] * D[idx]
        d, j = tree.query(X)
        ahead = ((cloud[j] - X) * D[idx]).sum(1) > 0
        h_now = (d < hit_thr) & ahead
        hit[idx[h_now]] = True
        alive[idx[h_now]] = False
        go = idx[~h_now]
        t[go] += np.maximum(d[~h_now], min_step)
        alive[go[t[go] > free_mm]] = False
    return 1.0 - hit.reshape(n, R).mean(1)


def march(O, D, tree, max_dist, hit_thr, min_step, max_steps=80, t0=0.0, cloud=None):
    """Sphere tracing on the unsigned distance of a cloud. True where the ray hits; with
    `cloud`, a near point is a hit only when it lies ahead of the ray (see above)."""
    t = np.full(len(O), float(t0))
    alive = np.ones(len(O), bool)
    hit = np.zeros(len(O), bool)
    for _ in range(max_steps):
        idx = np.flatnonzero(alive)
        if not len(idx):
            break
        X = O[idx] + t[idx, None] * D[idx]
        d, j = tree.query(X)
        h_now = d < hit_thr
        if cloud is not None:
            h_now &= ((cloud[j] - X) * D[idx]).sum(1) > 0
        hit[idx[h_now]] = True
        alive[idx[h_now]] = False
        go = idx[~h_now]
        t[go] += np.maximum(d[~h_now], min_step)
        alive[go[t[go] > max_dist]] = False
    return hit


def confined(pts, axes, tree, h, bore_min_diameter=80.0, heights=(15.0, 35.0, 55.0), n_az=12, cloud=None):
    """The generator's bore rule (accessibility.torch_clearance.bore_min_diameter_mm): a seam
    is not weldable when the torch BODY cannot enter a cavity narrower than the minimum bore
    diameter. Around each joint-face normal in `axes` (n_A, n_B), at a few heights above the
    seam, a ring of rays parallel to that face is marched on the cloud; the point is confined
    when, for either face, some ring closes - every ray hits within half the diameter. Inside
    a bore the ring around the floor normal closes; outside a tube half of it escapes."""
    R = bore_min_diameter / 2.0
    out = np.zeros(len(pts), bool)
    az = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
    for axis in axes:
        e1, e2 = _frame(axis)
        for z in heights:
            C = pts + z * axis
            O = np.repeat(C[:, None, :], n_az, 1).reshape(-1, 3)
            D = (np.cos(az)[None, :, None] * e1[:, None, :] + np.sin(az)[None, :, None] * e2[:, None, :]).reshape(-1, 3)
            hit = march(O, D, tree, R, 0.5 * h, 0.5 * h, cloud=cloud).reshape(len(pts), n_az)
            out |= hit.all(1)
    return out


def torch_feasibility(pts, tan, bisector, cloud_tree, cloud, h, half_deg=30.0,
                      standoff=15.0, max_work_deg=45.0, work_step_deg=15.0,
                      travel_deg=(-15.0, 0.0, 15.0)):
    """Per seam point: the fraction of (work, travel) torch directions whose clearance cone
    is empty, and the best direction. `bisector` points from the seam INTO free space."""
    works = np.arange(-max_work_deg, max_work_deg + 1e-9, work_step_deg)
    side = unit(np.cross(tan, bisector))
    feas = np.zeros((len(pts), len(works), len(travel_deg)), bool)
    cos_c = np.cos(np.radians(half_deg))
    nb = cloud_tree.query_ball_point(pts, standoff)
    for i, ids in enumerate(nb):
        Q = cloud[ids] - pts[i] if ids else np.zeros((0, 3))
        r = np.linalg.norm(Q, axis=1)
        Q, r = Q[r > 2 * h], r[r > 2 * h]
        for a, w in enumerate(np.radians(works)):
            u0 = np.cos(w) * bisector[i] + np.sin(w) * side[i]
            for b, tr in enumerate(np.radians(travel_deg)):
                u = np.cos(tr) * u0 + np.sin(tr) * tan[i]
                feas[i, a, b] = not np.any((Q @ u) > cos_c * r) if len(Q) else True
    frac = feas.reshape(len(pts), -1).mean(1)
    return frac, feas
