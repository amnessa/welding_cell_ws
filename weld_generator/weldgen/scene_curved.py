"""Scene assembly for the D29 curved families — Phase 6b step 3b.

Same contract as `scene.generate_scene`: `(scene_json, arrays)`, schema-valid, hashed
and written by the same writer — a curved scene is a scene, not a parallel format.
What differs is the spine of the pipeline, which is the D29 curve-first order:

    seam_curve stream   -> the curve family and its parameters (d29.sample_d29_seam)
    joint_config stream -> the realization's nuisance dims (wall, height, gap, plate)
    placement stream    -> the assembly pose (z-preserving: translate + rot_z;
                           the fixture is OFF for curved configurations, patch_phase6b)
    camera / surface_sample / noise -> exactly the plate pipeline's roles

Before anything is emitted the generation GATE runs: the D4 arm rediscovers the seam
from the placed parts (families #2–#4, machine precision) or the consistency residual
checks construction kept its promises (#5–#7); a failure raises, never emits. Frames,
verdicts and negatives come from `verify_curved.curved_seam_set` / `seam_verdict` —
per-point arrays because everything varies along a curved seam — and the recorded
`parametric` is the drawn curve in world coordinates (`transform_parametric`), so D1's
claim rides into the file unchanged.
"""

from __future__ import annotations

import datetime as _dt
import platform
from typing import Any

import numpy as np

from .accessibility import ALLOWED_CLASSES, DEFAULT_ACCESS
from .camera import intrinsics, sample_pose, standoff_for_framing
from . import GENERATOR_VERSION, SCHEMA_VERSION
from .config import SENSOR_PROFILES, classify_quality, geometry_config
from .constructors import build_d29
from .curves import transform_parametric
from .d29 import sample_d29_seam
from .geom import Slab, SweptSlab, Tube, rot_z, translate
from .hashing import config_id, twin_key
from .rng import Streams
from .sampling import sample_slab_surface
from .scene import NoVisibleSeams, SceneRejected, _git_commit, _iso_17659_term
from .verify_curved import (
    containment_residual, curve_distance, curved_seam_set, rediscover_seam,
    seam_verdict,
)
from .visibility import exterior_scan_subsampled, seam_masks, visible_mask


class CurvedGateFailed(SceneRejected):
    """The D4 rediscovery (or consistency residual) disagreed with the record.

    This is the Phase 2 gate for curved scenes: construction placed the parts from the
    drawn curve, verification recomputed the meeting line from the parts alone, and
    they did not match. Nothing is emitted past a failed gate.
    """


#: Rediscovery must match the record to this. Measured slack over machine precision
#: (observed 1e-13) purely for float accumulation across poses.
_GATE_MM = 1e-6

_SEAM_CLASS = {"weld": None, "bore": None, "toe": "lap_toe"}   # None -> by joint type


def _included_angle(built: dict[str, Any]) -> float:
    if built["config"] in (3, 4):
        return 90.0 - float(built["realization"].get("tilt_deg", 0.0))
    return 180.0 if built["config"] == 7 else 90.0


def _seam_shape(built: dict[str, Any]) -> str:
    if built["curve"].closed:
        return "closed"
    kind = built["curve"].to_parametric()["kind"]
    return "spline" if kind == "bspline" else "arc"


def _face_pair(built: dict[str, Any], role: str) -> list[str]:
    cfg = built["config"]
    if cfg in (2, 3):
        return ["A:+w", "B:lateral+" if role == "weld" else "B:lateral-"]
    if cfg == 4:
        return ["A:lateral+", "B:lateral+" if role == "weld" else "B:lateral-"]
    if cfg == 5:
        return ["A:+w", "B:-w" if role == "weld" else "B:+w"]
    if cfg == 6:
        return ["A:+w", "B:+w" if role == "weld" else "B:-w"]  # per-offset; cosmetic
    return ["A:-w", "B:+w"]                                    # 7: the two edge faces


def _objects_block(parts) -> list[dict[str, Any]]:
    out = []
    for p in parts:
        entry = {
            "id": p.id, "role": p.role, "object_id": p.object_id,
            "dims_mm": [float(v) for v in p.dims_mm],
            "thickness_mm": float(p.thickness_mm),
            "part_geometry_id": p.part_geometry_id,
            "mesh": None,
            "T_world_part": [[float(v) for v in row] for row in p.T_world_part],
        }
        if isinstance(p, Tube):
            entry["primitive"] = "tube"
            entry["params"] = {"r_outer_mm": float(p.r_outer_mm),
                               "wall_mm": float(p.wall_mm),
                               "length_mm": float(p.length_mm),
                               "base_cut": p.base_cut, "gap_mm": float(p.gap_mm)}
        elif isinstance(p, SweptSlab):
            entry["primitive"] = "swept_slab"
            entry["params"] = {"spine": p.spine.to_parametric(),   # part-local
                               "offset_lo_mm": float(p.offset_lo_mm),
                               "offset_hi_mm": float(p.offset_hi_mm),
                               "z0_mm": float(p.z0_mm), "z1_mm": float(p.z1_mm)}
        else:
            entry["primitive"] = "slab"
        out.append(entry)
    return out


def _faces_block(parts) -> list[dict[str, Any]]:
    out = []
    fid = 0
    for p in parts:
        for name in p.face_names():
            plane = p.face_plane(name) if hasattr(p, "face_plane") else None
            surface = p.surface_desc(name) if hasattr(p, "surface_desc") else None
            out.append({
                "face_id": fid,
                "ref": f"{p.id}:{name}", "object": p.id, "name": name,
                "plane": None if plane is None else
                {"n": [float(v) for v in plane.n], "d": float(plane.d)},
                "surface": surface,
                "area_mm2": float(p.face_area(name)),
            })
            fid += 1
    return out


def _sample_cloud(parts, density: float, rng) -> dict[str, np.ndarray]:
    xyz, nrm, oid, fid = [], [], [], []
    base = 0
    for p in parts:
        if isinstance(p, Slab):
            part_cloud = sample_slab_surface(p, density, rng, face_id_base=base)
            xyz.append(part_cloud["xyz"])
            nrm.append(part_cloud["normals"])
            oid.append(part_cloud["object_id"])
            fid.append(part_cloud["face_id"])
        else:
            for j, name in enumerate(p.face_names()):
                n_pts = int(round(density * p.face_area(name)))
                if n_pts <= 0:
                    continue
                pts, normals = p.sample_face(name, n_pts, rng)
                xyz.append(pts.astype(np.float32))
                nrm.append(normals.astype(np.float32))
                oid.append(np.full(n_pts, p.object_id, dtype=np.uint8))
                fid.append(np.full(n_pts, base + j, dtype=np.uint8))
        base += len(p.face_names())
    return {"xyz": np.vstack(xyz), "normals": np.vstack(nrm),
            "object_id": np.concatenate(oid), "face_id": np.concatenate(fid)}


def generate_curved_scene(cfg: dict[str, Any],
                          seed: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """One curved scene, D29 curve-first end to end. Same return contract as
    `scene.generate_scene`."""
    streams = Streams(seed)

    # --- the curve, then its realization (the D3 order, in the stream layout) -----
    fams = list(cfg.get("seam_families") or [2, 3, 4, 5, 6, 7])
    g_curve = streams["seam_curve"]
    fam = int(g_curve.choice(fams)) if len(fams) > 1 else int(fams[0])
    draw = sample_d29_seam(g_curve, config=fam)
    built = build_d29(draw, streams["joint_config"])

    # --- the generation gate (verification BEFORE pose, in the joint frame) ------
    if built["config"] in (2, 3, 4):
        gap = curve_distance(rediscover_seam(built), built["curve"])
        if gap > _GATE_MM:
            raise CurvedGateFailed(
                f"family {fam} seed {seed}: rediscovered seam departs the record "
                f"by {gap:.3e} mm")
    else:
        res = containment_residual(built)
        if res > _GATE_MM:
            raise CurvedGateFailed(
                f"family {fam} seed {seed}: consistency residual {res:.3e} mm")

    # --- verdicts (coarse) and emission frames (dense), both pre-pose -------------
    # runtime access: config overrides on the defaults, with the torch dict merged
    # NESTED so the bore gate's default survives a config that only sets the cone
    cfg_acc = cfg.get("accessibility", {})
    access = {k: (v.copy() if isinstance(v, dict) else v)
              for k, v in {**DEFAULT_ACCESS, **cfg_acc}.items()}
    access["torch_clearance"] = {**DEFAULT_ACCESS["torch_clearance"],
                                 **cfg_acc.get("torch_clearance", {})}
    gap_mm = float(built["gap_mm"])
    access["contact_tol_mm"] = float(np.clip(2.0 * gap_mm + 0.5, 0.5, 12.0))
    coarse = curved_seam_set(built, n=64)
    density_per_mm = 10.0
    dense = curved_seam_set(built, density_per_mm=density_per_mm)
    verdicts = [seam_verdict(s, built["parts"], access) for s in coarse]

    # --- pose (z-preserving; fixture off for curved configurations) ---------------
    g3 = streams["placement"]
    span = float(cfg["assembly_translation_mm"])
    T_pose = (translate(*(g3.uniform(-span, span, size=3) * np.array([1.0, 1.0, 0.0])))
              @ rot_z(float(g3.uniform(0.0, 360.0))))
    parts = built["parts"]
    for p in parts:
        p.T_world_part = T_pose @ p.T_world_part
    R_pose = T_pose[:3, :3]

    def w_pts(a):
        return (a @ R_pose.T + T_pose[:3, 3]).astype(np.float32)

    def w_vec(a):
        return (a @ R_pose.T).astype(np.float32)

    # --- camera (mirrors the plate pipeline's substream-5 draw order) -------------
    g5 = streams["camera"]
    profile = str(g5.choice(cfg["sensor_profiles"]))
    prof = SENSOR_PROFILES[profile]
    standoff = float(g5.uniform(max(prof["min_z_mm"], cfg["standoff_mm"][0]),
                                cfg["standoff_mm"][1]))
    elevation = float(g5.uniform(*cfg["elevation_deg"]))
    azimuth = float(g5.uniform(0.0, 360.0))
    roll = float(g5.uniform(*cfg["camera_roll_deg"]))
    aim_jitter = g5.uniform(-1.0, 1.0, size=3)
    framing = float(g5.uniform(*cfg["framing_frac"]))

    width, height = int(cfg["image_size"][0]), int(cfg["image_size"][1])
    K = intrinsics(prof["focal_px"], width, height)
    weld_pts = np.vstack([w_pts(s["points"]) for s in coarse
                          if s["role"] == "weld"])
    extent = float(np.ptp(np.vstack([p.mesh().vertices for p in parts]),
                          axis=0).max())
    target = (weld_pts.mean(axis=0)
              + aim_jitter * float(cfg["aim_jitter_frac"]) * extent)
    if cfg.get("frame_by_extent", True):
        standoff = float(np.clip(
            standoff_for_framing(extent, framing, prof["focal_px"], width, height),
            max(prof["min_z_mm"], cfg["standoff_mm"][0]), cfg["standoff_mm"][1]))
    T_cam = sample_pose(target, standoff, elevation, azimuth, roll)

    # --- cloud, substream 4 --------------------------------------------------------
    g4 = streams["surface_sample"]
    density = float(g4.uniform(*cfg["density_per_mm2"]))
    cloud = _sample_cloud(parts, density, g4)
    cloud["visible_from_cam"] = visible_mask(
        cloud["xyz"], cloud["normals"], parts, T_cam, K, width, height,
        prof["min_z_mm"])
    cloud["exterior"] = exterior_scan_subsampled(cloud["xyz"], cloud["normals"],
                                                 parts)

    # --- seam emission -------------------------------------------------------------
    arrays: dict[str, np.ndarray] = {}
    order = sorted(range(len(dense)), key=lambda i: not verdicts[i][0])
    seam_blocks = []
    any_visible = 0.0
    for out_i, i in enumerate(order):
        s = dense[i]
        weldable, reason, frac = verdicts[i]
        pts = w_pts(s["points"])
        appr = w_vec(s["approach"])
        n = len(pts)
        arrays[f"seams.npz:seam_{out_i}"] = pts
        arrays[f"seams.npz:seam_{out_i}_s"] = np.linspace(
            0.0, s["curve"].length_mm, n, endpoint=not s["curve"].closed
        ).astype(np.float32)
        arrays[f"seams.npz:seam_{out_i}_tangent"] = w_vec(s["tangent"])
        arrays[f"seams.npz:seam_{out_i}_approach"] = appr
        arrays[f"seams.npz:seam_{out_i}_nA"] = w_vec(s["nA"])
        arrays[f"seams.npz:seam_{out_i}_nB"] = w_vec(s["nB"])
        arrays[f"seams.npz:seam_{out_i}_dihedral"] = \
            s["dihedral_deg"].astype(np.float32)
        masks = seam_masks(pts, appr, parts, T_cam, K, width, height,
                           prof["min_z_mm"])
        arrays[f"seams.npz:seam_{out_i}_visible"] = masks["visible"]

        dmin, dmax = float(s["dihedral_deg"].min()), float(s["dihedral_deg"].max())
        dihedral = (dmin if dmax - dmin < 1e-6 else
                    {"min": dmin, "max": dmax,
                     "mean": float(s["dihedral_deg"].mean())})
        cls = _SEAM_CLASS[s["role"]] or \
            ("butt" if built["config"] == 7 else "fillet")
        vis_frac = float(masks["visible"].mean())
        if weldable:
            any_visible = max(any_visible, vis_frac)
        seam_blocks.append({
            "id": out_i,
            "weldable": bool(weldable),
            "reject_reason": reason,
            "clear_fraction": None if np.isnan(frac) else float(frac),
            "face_pair": _face_pair(built, s["role"]),
            "parametric": transform_parametric(s["curve"].to_parametric(), T_pose),
            "closed": bool(s["curve"].closed),
            "length_mm": float(s["curve"].length_mm),
            "dihedral_deg": dihedral,
            "seam_class": cls,
            "matches_joint_type": cls in ALLOWED_CLASSES[built["joint_type"]],
            "underside": None,
            "sampled": {"array": f"seam_{out_i}",
                        "density_per_mm": density_per_mm, "n": n},
            "occluded_fraction": float(1.0 - masks["unoccluded"].mean()),
            "in_frame_fraction": float(masks["in_frame"].mean()),
            "visible_fraction": vis_frac,
        })

    if any_visible < float(cfg.get("min_visible_fraction", 0.1)):
        raise NoVisibleSeams(
            f"family {fam} seed {seed}: best weldable visible fraction "
            f"{any_visible:.3f} < {cfg.get('min_visible_fraction', 0.1)}")

    for k, v in cloud.items():
        arrays[f"cloud.npz:{k}"] = v

    t_min = min(p.thickness_mm for p in parts if p.role == "workpiece")
    included = _included_angle(built)
    chord = max((p.max_chord_error_mm for p in parts
                 if hasattr(p, "max_chord_error_mm")), default=0.0)

    cid = config_id(cfg)
    scene = {
        "schema_version": SCHEMA_VERSION,
        "generator_version": GENERATOR_VERSION,
        "scene_id": f"{cid}-{seed:010d}",
        "config_id": cid,
        "seed": int(seed),
        "tier": 1,
        "twin_key": twin_key(geometry_config(cfg), seed),
        "units": {"length": "mm", "angle": "deg"},
        "joint": {
            "type": built["joint_type"],
            "seam_shape": _seam_shape(built),
            "seam_family": built["name"],                       # D29, curve-first
            "realization": built["realization"],
            "quality_level": classify_quality(t_min, 0.0, 0.0, gap_mm,
                                              0.7 * t_min),
            "contact_mode": "free",
            "included_angle_deg": included,
            "in_plane_yaw_deg": 0.0,
            "iso_17659_term": _iso_17659_term(built["joint_type"], included),
            # ISO 9692-1 Part 1 rows do not cover branch/pipe preparation (the 17659
            # review's open note); recorded honestly as absent.
            "iso_9692_ref": None,
            "ambiguous_with": None,
            "stack_offset_mm": None,
            "prep": "square",
        },
        "seam_definition": "nominal",
        # the full runtime block, schema-shaped, exactly as the plate pipeline stores
        # its own - including the bore gate parameter actually applied
        "accessibility": {k: access[k] for k in
                          ("torch_clearance", "max_work_angle_deg",
                           "dihedral_min_deg", "dihedral_max_deg",
                           "contact_tol_mm", "coplanar_tol_mm",
                           "cross_run_tol_deg", "min_seam_length_mm")},
        "objects": _objects_block(parts),
        "faces": _faces_block(parts),
        "fit": {"root_gap_mm": gap_mm, "linear_misalignment_mm": 0.0,
                "angular_misalignment_deg": 0.0,
                "throat_thickness_mm": 0.7 * t_min},
        "T_world_joint": [[float(v) for v in row] for row in T_pose],
        "seams": seam_blocks,
        "tacks": None,
        "camera": {"model": "pinhole", "K": K, "width": width, "height": height,
                   "T_world_cam": [[float(v) for v in row] for row in T_cam],
                   "standoff_mm": standoff, "elevation_deg": elevation},
        "noise_model": {"kind": "stereo_z2", "profile": profile,
                        "subpixel_px": prof["subpixel_px"],
                        "baseline_mm": prof["baseline_mm"],
                        "focal_px": prof["focal_px"], "min_z_mm": prof["min_z_mm"],
                        "lateral_sigma_px": 0.8, "grazing_dropout_deg": 75.0,
                        "seed": int(seed)},
        "cloud": {"file": "cloud.npz", "frame": "world",
                  "sampling_mode": "area_uniform",
                  "density_per_mm2": density,
                  "n_points": int(len(cloud["xyz"])),
                  "max_chord_error_mm": float(chord)},          # D34 (mesh-only)
        "rgb": None,
        "depth": None,
        "provenance": {
            "created_utc": _dt.datetime.now(_dt.timezone.utc)
            .strftime("%Y-%m-%dT%H:%M:%SZ"),
            "git_commit": _git_commit(),
            "python": platform.python_version(),
            "numpy": np.__version__,
        },
    }
    if chord > 0.25:
        raise CurvedGateFailed(
            f"family {fam} seed {seed}: chord error {chord:.3f} mm exceeds the "
            f"D34 gate")
    if cfg.get("emit_mps", False):        # D25, Phase 6c: derived, no random draw
        from .mps import mps_rule
        scene["mps"] = mps_rule(scene)
    return scene, arrays
