"""Seam and tack masks - `maskrule-0.1` (dataset_plan.md Phase 8).

Constructed, never detected: the D19 nominal seam polyline from `seams.npz`, projected
through the view's camera, kept where the analytic ray cast says it is unoccluded (no
blind zone, no face test - the same test tier 1 uses for seams), drawn at a PHYSICAL
width projected at the point's own depth. Tacks are arclength intervals of the same
polyline from the `tacks` block (D38): `[s - L/2, s + L/2]`, wrapped on closed seams.

Values: `mask_seam` = seam id + 1 for weldable seams, `mask_tack` = tack index + 1, 0 = none.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from ..camera import project
from ..visibility import visible_mask


def stamp(mask: np.ndarray, uv: np.ndarray, z: np.ndarray, fx: float, width_mm: float,
          value: int, max_radius_px: int = 25) -> None:
    """Draw points with a physical width: radius_px = width/2 * f / z, per-radius dilation."""
    H, W = mask.shape
    if len(uv) == 0:
        return
    r_px = np.clip(np.round(0.5 * width_mm * fx / np.maximum(z, 1e-6)).astype(int), 1, max_radius_px)
    j = np.round(uv[:, 0] - 0.5).astype(int); i = np.round(uv[:, 1] - 0.5).astype(int)
    ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
    for r in np.unique(r_px[ok]):
        sel = ok & (r_px == r)
        base = np.zeros((H, W), bool); base[i[sel], j[sel]] = True
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disk = (xx ** 2 + yy ** 2) <= r ** 2
        mask[ndimage.binary_dilation(base, disk) & (mask == 0)] = value


def seam_and_tack_masks(scene: dict, seams_npz, parts, K, T_world_cam, width: int, height: int,
                        seam_width_mm: float = 2.0, tack_width_mm: float = 3.0) -> tuple[np.ndarray, np.ndarray, dict]:
    K = np.asarray(K, float); T = np.asarray(T_world_cam, float); fx = float(K[0, 0])
    mseam = np.zeros((height, width), np.uint8); mtack = np.zeros((height, width), np.uint8)
    tacks = scene.get("tacks") or {}
    t_seam = list(tacks.get("seam_id", [])); t_s = list(tacks.get("arclength_mm", [])); t_len = list(tacks.get("tack_length_mm", []))
    stats = {"seams_drawn": 0, "tacks_drawn": 0, "tacks_total": len(t_seam)}
    for s in scene["seams"]:
        if not s["weldable"]:
            continue
        pts = np.asarray(seams_npz[f"seam_{s['id']}"], float)
        ap = np.asarray(seams_npz[f"seam_{s['id']}_approach"], float)
        vis = visible_mask(pts, ap, parts, T, K, width, height, 0.0, face_test=False)
        if not vis.any():
            continue
        uv, z = project(pts, T, K)
        stamp(mseam, uv[vis], z[vis], fx, seam_width_mm, s["id"] + 1)
        stats["seams_drawn"] += 1
        sarr = np.asarray(seams_npz[f"seam_{s['id']}_s"], float); L = float(s["length_mm"])
        for ti, (sid, s0, tl) in enumerate(zip(t_seam, t_s, t_len)):
            if sid != s["id"]:
                continue
            ds = np.abs(sarr - float(s0))
            if s.get("closed", False):
                ds = np.minimum(ds, L - ds)
            sel = vis & (ds <= float(tl) / 2.0)
            if sel.any():
                stamp(mtack, uv[sel], z[sel], fx, tack_width_mm, ti + 1)
                stats["tacks_drawn"] += 1
    return mseam, mtack, stats
