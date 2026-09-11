"""Everything a render pass draws - appearance, environment, lights, drawn cameras.

All draws come from ONE generator seeded by sha256(scene_id, render_id) (the D39 pattern),
never from the tier-1 substreams, and are APPENDED in the order below. Adding a draw later
goes at the end; earlier draws must not move.

    1. alloy (one per scene)         2. surface condition (one per workpiece, in object order)
    3. substrate photo + span jitter + rotation + roughness
    4. dome panorama + intensity     5. key light intensity, elevation, azimuth
    6. drawn views 1..N-1, each: aim jitter (3), elevation, azimuth, roll, framing
       - redrawn until a primary seam is >= min_visible_fraction visible AND >= min_seam_px
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from ..camera import project, sample_pose, standoff_for_framing
from ..visibility import visible_mask


def render_rng(scene_id: str, render_id: str) -> np.random.Generator:
    h = hashlib.sha256(f"{scene_id}|{render_id}".encode()).digest()
    return np.random.default_rng(int.from_bytes(h[:8], "big"))


def _u(rng, lo_hi):
    return float(rng.uniform(float(lo_hi[0]), float(lo_hi[1])))


def draw_appearance(rng: np.random.Generator, cfg: dict, scene: dict, backgrounds: dict) -> dict:
    """Draws 1-5. Pure; returns a JSON-able dict that goes into render.json verbatim."""
    mats = cfg["materials"]; env = cfg["environment"]; lit = cfg["lighting"]
    alloy = str(rng.choice(mats["alloys"]))
    workpieces = [o["id"] for o in scene["objects"] if o["role"] == "workpiece"]
    surface = {oid: str(rng.choice(mats["surface_conditions"])) for oid in workpieces}
    photos = backgrounds["photos"]
    ph = photos[int(rng.integers(len(photos)))]
    span = float(ph["span_m"]) * (1.0 + _u(rng, (-env["span_jitter"], env["span_jitter"])))
    substrate = {"photo": ph["file"], "photo_sha256": ph["sha256"], "span_m": span,
                 "span_m_y": span * ph["height_px"] / ph["width_px"],
                 "rotation_deg": _u(rng, (0.0, 360.0)), "roughness": _u(rng, env["roughness"]),
                 "plane_half_m": float(env["plane_half_m"])}
    panos = backgrounds["_panoramas"]
    dome = {"panorama": Path(panos[int(rng.integers(len(panos)))]).name if panos else None,
            "intensity": _u(rng, lit["dome_intensity"])}
    key = {"intensity": _u(rng, lit["key_intensity"]), "elevation_deg": _u(rng, lit["key_elevation_deg"]),
           "azimuth_deg": _u(rng, lit["key_azimuth_deg"])}
    return {"alloy": alloy, "surface_condition": surface, "substrate": substrate, "dome": dome, "key_light": key}


def primary_seams(scene: dict, seams_npz) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    """`{seam id: (points, approach)}` for seams that are weldable AND match the joint type."""
    out = {}
    for s in scene["seams"]:
        if s["weldable"] and s["matches_joint_type"]:
            out[s["id"]] = (np.asarray(seams_npz[f"seam_{s['id']}"], float),
                            np.asarray(seams_npz[f"seam_{s['id']}_approach"], float))
    return out


def seam_visibility(T, K, W, H, parts, seams: dict) -> tuple[float, int]:
    """Best (visible fraction, distinct image pixels) over the primary seams for a camera."""
    best_f, best_px = 0.0, 0
    for pts, ap in seams.values():
        vis = visible_mask(pts, ap, parts, T, K, W, H, 0.0, face_test=False)
        if vis.any():
            uv, _ = project(pts[vis], T, K)
            n_px = len(np.unique(np.round(uv).astype(int), axis=0))
        else:
            n_px = 0
        best_f, best_px = max(best_f, float(vis.mean())), max(best_px, n_px)
    return best_f, best_px


def draw_views(rng: np.random.Generator, cfg: dict, scene: dict, seams: dict, parts,
               n_views: int | None = None) -> list[dict]:
    """View 0 = the tier-1 camera verbatim; views 1..N-1 drawn with the training ranges."""
    cam = scene["camera"]; K = np.asarray(cam["K"], float); W, H = int(cam["width"]), int(cam["height"])
    dv = cfg["drawn_views"]; n_views = int(cfg["views"] if n_views is None else n_views)
    f0, px0 = seam_visibility(np.asarray(cam["T_world_cam"], float), K, W, H, parts, seams)
    views = [{"view": 0, "view_kind": "tier1", "K": cam["K"], "T_world_cam": cam["T_world_cam"],
              "width": W, "height": H, "elevation_deg": cam.get("elevation_deg"),
              "standoff_mm": cam.get("standoff_mm"), "attempts": 0,
              "best_primary_visible": f0, "best_primary_seam_px": px0}]
    span = max(float(np.max(o["dims_mm"])) for o in scene["objects"] if o["role"] == "workpiece")
    centre = (np.mean([p.mean(0) for p, _ in seams.values()], 0) if seams
              else np.mean([np.asarray(o["T_world_part"], float)[:3, 3] for o in scene["objects"]], 0))
    fx = float(K[0, 0])
    for k in range(1, n_views):
        chosen = None
        for attempt in range(1, int(dv["max_attempts"]) + 1):
            aim = centre + rng.uniform(-1.0, 1.0, 3) * float(dv["aim_jitter_frac"]) * span
            el, az, roll, fr = _u(rng, dv["elevation_deg"]), _u(rng, (0.0, 360.0)), _u(rng, dv["roll_deg"]), _u(rng, dv["framing_frac"])
            standoff = float(np.clip(standoff_for_framing(span, fr, fx, W, H), *dv["standoff_mm"]))
            T = sample_pose(aim, standoff, el, az, roll)
            f, px = seam_visibility(T, K, W, H, parts, seams)
            if f >= float(dv["min_visible_fraction"]) and px >= int(dv["min_seam_px"]):
                chosen = {"view": k, "view_kind": "drawn", "K": cam["K"], "T_world_cam": T.tolist(),
                          "width": W, "height": H, "elevation_deg": el, "azimuth_deg": az, "roll_deg": roll,
                          "framing_frac": fr, "standoff_mm": standoff, "aim_mm": aim.tolist(),
                          "attempts": attempt, "best_primary_visible": f, "best_primary_seam_px": px}
                break
        if chosen is None:
            chosen = {"view": k, "view_kind": "undrawable", "attempts": int(dv["max_attempts"])}
        views.append(chosen)
    return views
