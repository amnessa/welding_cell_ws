"""The twin gate — what makes a tier-2 view an honest twin of its tier-1 scene.

Three statements, each a number with a threshold (`dataset_plan.md` Phase 8, "Gates"):

* **residual**: workpiece pixels of view 0's clean depth back-project onto the exact tier-1
  primitives. p99 < 0.25 mm (the D34 budget); the pilot measured 0.001 mm max.
* **coverage**: a tier-1 cloud point that `visible_from_cam` calls visible lands on a depth
  pixel that agrees with it, and vice versa, after the sensor profile's blind zone and
  frustum are applied to the render the way tier 1 applies them to the cloud.
* **object ids**: the renderer's id buffer agrees with the nearest tier-1 point's object_id
  on workpiece pixels.

Pure numpy + trimesh + scipy; no renderer. Test with synthetic depth under plain pytest.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from ..camera import in_frustum, project
from .conventions import LABEL_ENV, LABEL_NONE, backproject, object_label

#: thresholds; a report passes when every one holds
THRESHOLDS = {"residual_p99_mm": 0.25, "coverage_agreement": 0.90, "object_agreement": 0.98}


def point_triangle_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """Exact point-to-triangle distance, vectorised over matching rows (Ericson, RTCD 5.1.5)."""
    ab, ac, ap = b - a, c - a, p - a
    d1 = np.einsum("ij,ij->i", ab, ap); d2 = np.einsum("ij,ij->i", ac, ap)
    bp = p - b; d3 = np.einsum("ij,ij->i", ab, bp); d4 = np.einsum("ij,ij->i", ac, bp)
    cp = p - c; d5 = np.einsum("ij,ij->i", ab, cp); d6 = np.einsum("ij,ij->i", ac, cp)
    vc, vb, va = d1 * d4 - d3 * d2, d5 * d2 - d1 * d6, d3 * d6 - d5 * d4
    q = np.empty_like(p)
    # vertex regions
    ra = (d1 <= 0) & (d2 <= 0); q[ra] = a[ra]
    rb = (d3 >= 0) & (d4 <= d3); q[rb] = b[rb]
    rc = (d6 >= 0) & (d5 <= d6); q[rc] = c[rc]
    done = ra | rb | rc
    # edge ab
    e = ~done & (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    v = np.zeros(len(p)); den = d1 - d3; v[e] = d1[e] / den[e]; q[e] = a[e] + v[e, None] * ab[e]; done |= e
    # edge ac
    e = ~done & (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    w = np.zeros(len(p)); den = d2 - d6; w[e] = d2[e] / den[e]; q[e] = a[e] + w[e, None] * ac[e]; done |= e
    # edge bc
    e = ~done & (va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0)
    den = (d4 - d3) + (d5 - d6); w2 = np.zeros(len(p)); w2[e] = (d4 - d3)[e] / den[e]
    q[e] = b[e] + w2[e, None] * (c - b)[e]; done |= e
    # face interior
    f = ~done
    denom = 1.0 / np.where(f, va + vb + vc, 1.0)
    v = vb * denom; w = vc * denom
    q[f] = a[f] + v[f, None] * ab[f] + w[f, None] * ac[f]
    return np.linalg.norm(p - q, axis=1)


def distance_to_mesh(xyz: np.ndarray, mesh, k: int = 32, exact_above_mm: float = 0.05,
                     brute_chunk: int = 1000) -> np.ndarray:
    """Distance to a mesh WITHOUT rtree (D9).

    Pass 1: exact distance to the `k` faces whose centroids are nearest each point. That can
    only OVER-estimate (a long thin triangle's centroid is far from its ends), so pass 2
    recomputes, brute force over every face, any point whose pass-1 distance exceeds
    `exact_above_mm`. In a passing render that is a handful of points; in a failing one it
    is the honest number.
    """
    xyz = np.asarray(xyz, float)
    tri = np.asarray(mesh.triangles, dtype=float)                 # (F, 3, 3)
    F = len(tri); k = min(k, F)
    _, idx = cKDTree(tri.mean(1)).query(xyz, k=k, workers=-1)
    idx = idx.reshape(len(xyz), k)
    P = np.repeat(xyz, k, axis=0); T = tri[idx.ravel()]
    d = point_triangle_distance(P, T[:, 0], T[:, 1], T[:, 2]).reshape(len(xyz), k).min(axis=1)
    redo = np.flatnonzero(d > exact_above_mm)
    for c0 in range(0, len(redo), brute_chunk):
        sel = redo[c0:c0 + brute_chunk]
        P = np.repeat(xyz[sel], F, axis=0); T = np.tile(tri, (len(sel), 1, 1))
        d[sel] = point_triangle_distance(P, T[:, 0], T[:, 1], T[:, 2]).reshape(len(sel), F).min(axis=1)
    return d


def residual_mm(xyz: np.ndarray, meshes, max_points: int = 60000, seed: int = 0,
                chunk: int = 20000) -> np.ndarray:
    """Distance from each point to the nearest surface among `meshes` (subsampled, chunked)."""
    xyz = np.asarray(xyz, dtype=float)
    if len(xyz) > max_points:
        xyz = xyz[np.random.default_rng(seed).choice(len(xyz), max_points, replace=False)]
    best = np.full(len(xyz), np.inf)
    for m in meshes:
        for c0 in range(0, len(xyz), chunk):
            best[c0:c0 + chunk] = np.minimum(best[c0:c0 + chunk], distance_to_mesh(xyz[c0:c0 + chunk], m))
    return best


def coverage(depth_mm: np.ndarray, valid: np.ndarray, cloud: dict, K, T_world_cam,
             min_z_mm: float, tol_mm: float = 1.0, mask_object: np.ndarray | None = None) -> dict:
    """Rendered visibility of every tier-1 point vs the stored `visible_from_cam`.

    A point is *rendered visible* when (1) it is framed and beyond the blind zone, exactly as
    tier 1 applies them; (2) its normal faces the camera - a render never shows a back face;
    (3) the id buffer at its pixel shows its own object, when a mask is given - a point hidden
    behind another part is not rendered; (4) its depth lies within the depth RANGE of the
    valid pixels among the four around its projection, widened by `tol_mm` - a point between
    pixel centres at grazing incidence sits inside that range, not at one pixel's value.
    (2) and (3) were added on 2026-09-12 when edge joints - two thin plates face to face, seen
    nearly edge-on - let hidden faying-surface points through the range test alone.
    """
    xyz = np.asarray(cloud["xyz"], dtype=float)
    H, W = depth_mm.shape
    T = np.asarray(T_world_cam, float); Kf = np.asarray(K, float)
    uv, z = project(xyz, T, Kf)
    framed = in_frustum(uv, z, W, H, min_z_mm)
    facing = np.ones(len(xyz), bool)
    if "normals" in cloud:
        to_cam = T[:3, 3][None, :] - xyz
        facing = np.einsum("ij,ij->i", to_cam, np.asarray(cloud["normals"], float)) > 0.0
    j0 = np.clip(np.floor(uv[:, 0] - 0.5).astype(int), 0, W - 1)
    i0 = np.clip(np.floor(uv[:, 1] - 0.5).astype(int), 0, H - 1)
    own = np.ones(len(xyz), bool)
    if mask_object is not None and "object_id" in cloud:
        want = np.array([object_label(o) for o in np.asarray(cloud["object_id"])])
        own = np.zeros(len(xyz), bool)
    lo = np.full(len(z), np.inf); hi = np.full(len(z), -np.inf)
    for dj, di in ((0, 0), (1, 0), (0, 1), (1, 1)):
        j = np.clip(j0 + dj, 0, W - 1); i = np.clip(i0 + di, 0, H - 1)
        ok = valid[i, j]; dd = depth_mm[i, j]
        lo = np.where(ok, np.minimum(lo, dd), lo); hi = np.where(ok, np.maximum(hi, dd), hi)
        if mask_object is not None and "object_id" in cloud:
            own |= ok & (mask_object[i, j] == want)
    agrees = (z >= lo - tol_mm) & (z <= hi + tol_mm)
    rendered_visible = framed & facing & own & agrees
    t1 = np.asarray(cloud["visible_from_cam"], bool)
    return {
        "n_points": int(len(xyz)),
        "tier1_visible": int(t1.sum()),
        "rendered_visible": int(rendered_visible.sum()),
        "agreement": float((rendered_visible == t1).mean()),
        "recall_of_tier1_visible": float(rendered_visible[t1].mean()) if t1.any() else 1.0,
        "precision_of_rendered": float(t1[rendered_visible].mean()) if rendered_visible.any() else 1.0,
    }


def object_agreement(mask_object: np.ndarray, depth_mm: np.ndarray, valid: np.ndarray,
                     cloud: dict, K, T_world_cam, max_points: int = 60000, seed: int = 0,
                     seam_points: np.ndarray | None = None, seam_band_mm: float = 1.0,
                     meshes=None, labels=None) -> dict:
    """Id-buffer label vs the object whose EXACT surface is nearest the back-projected pixel.

    Against meshes, not against the nearest tier-1 sample: the cloud is sampled at 0,25-4
    points/mm², so on a 2 mm plate the nearest sample to a pixel can lie on the plate
    behind it (edge joints, 2026-09-12). Pixels within `seam_band_mm` of a seam polyline
    are excluded - there two surfaces meet and the nearest one is ambiguous by construction.
    Falls back to the nearest tier-1 sample when no meshes are given (synthetic tests).
    """
    wp = valid & (mask_object != LABEL_NONE) & (mask_object != LABEL_ENV)
    xyz, rows, cols = backproject(depth_mm, K, T_world_cam, wp)
    if len(xyz) == 0:
        return {"n_pixels": 0, "agreement": 1.0, "excluded_seam_band": 0}
    if len(xyz) > max_points:
        pick = np.random.default_rng(seed).choice(len(xyz), max_points, replace=False)
        xyz, rows, cols = xyz[pick], rows[pick], cols[pick]
    keep = np.ones(len(xyz), bool)
    if seam_points is not None and len(seam_points):
        d, _ = cKDTree(np.asarray(seam_points, float)).query(xyz, workers=-1)
        keep = d > seam_band_mm
    P = xyz[keep]
    if meshes is not None and labels is not None and len(P):
        dist = np.stack([distance_to_mesh(P, m) for m in meshes], 1)          # (n, parts)
        want = np.asarray(labels)[np.argmin(dist, axis=1)]
    elif len(P):
        _, nn = cKDTree(np.asarray(cloud["xyz"], dtype=float)).query(P, workers=-1)
        want = np.array([object_label(o) for o in np.asarray(cloud["object_id"])[nn]])
    agree = float((mask_object[rows[keep], cols[keep]] == want).mean()) if len(P) else 1.0
    return {"n_pixels": int(wp.sum()), "agreement": agree, "excluded_seam_band": int((~keep).sum()),
            "seam_band_mm": seam_band_mm, "truth": "nearest_mesh" if meshes is not None else "nearest_sample"}


def seam_polyline_points(scene: dict, seams_npz) -> np.ndarray | None:
    """All stored seam polyline points of a scene, for the object statement's exclusion band."""
    if seams_npz is None:
        return None
    pts = [np.asarray(seams_npz[f"seam_{s['id']}"], float) for s in scene["seams"] if f"seam_{s['id']}" in seams_npz]
    return np.concatenate(pts) if pts else None


def twin_gate(scene: dict, cloud: dict, meshes, depth_mm: np.ndarray, valid: np.ndarray,
              mask_object: np.ndarray, seams_npz=None) -> dict:
    """Run all three statements on view 0 and say whether they pass."""
    cam = scene["camera"]
    K, T = np.asarray(cam["K"], float), np.asarray(cam["T_world_cam"], float)
    wp = valid & (mask_object != LABEL_NONE) & (mask_object != LABEL_ENV)
    xyz, _, _ = backproject(depth_mm, K, T, wp)
    r = residual_mm(xyz, meshes) if len(xyz) else np.zeros(0)
    report = {
        "workpiece_pixels": int(wp.sum()),
        "environment_pixels": int((valid & (mask_object == LABEL_ENV)).sum()),
        "residual_p50_mm": float(np.median(r)) if len(r) else 0.0,
        "residual_p99_mm": float(np.percentile(r, 99)) if len(r) else 0.0,
        "residual_max_mm": float(r.max()) if len(r) else 0.0,
        "coverage": coverage(depth_mm, valid, cloud, K, T, float(scene["noise_model"]["min_z_mm"]), mask_object=mask_object),
        "objects": object_agreement(mask_object, depth_mm, valid, cloud, K, T, seam_points=seam_polyline_points(scene, seams_npz),
                                    meshes=meshes if "objects" in scene else None,
                                    labels=[object_label(o["object_id"]) for o in scene.get("objects", [])] or None),
    }
    report["pass"] = bool(report["residual_p99_mm"] < THRESHOLDS["residual_p99_mm"]
                          and report["coverage"]["agreement"] >= THRESHOLDS["coverage_agreement"]
                          and report["objects"]["agreement"] >= THRESHOLDS["object_agreement"])
    return report


def format_report(report: dict) -> str:
    c, o = report["coverage"], report["objects"]
    return (f"twin gate: {'PASS' if report['pass'] else 'FAIL'}  "
            f"workpiece px={report['workpiece_pixels']} env px={report['environment_pixels']}  "
            f"residual p50={report['residual_p50_mm']:.4f} p99={report['residual_p99_mm']:.4f} "
            f"max={report['residual_max_mm']:.4f} mm  "
            f"coverage agree={c['agreement']:.3f} recall={c['recall_of_tier1_visible']:.3f} "
            f"prec={c['precision_of_rendered']:.3f} (t1 vis {c['tier1_visible']}, rendered {c['rendered_visible']})  "
            f"objects agree={o['agreement']:.3f}")
