"""Scene assembly — produces a dict conforming to docs/scene.schema.json v2.0.0."""

from __future__ import annotations

import datetime as _dt
import platform
import subprocess
from typing import Any

import numpy as np

from . import GENERATOR_VERSION, SCHEMA_VERSION
from .config import SENSOR_PROFILES, geometry_config, sample_joint
from .geom import SLAB_FACES, Slab, rot_z, translate
from .hashing import config_id, twin_key
from .joints import build_t_joint
from .rng import Streams
from .sampling import sample_polyline, sample_scene_surface


def _git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL, text=True,
        ).strip()
    except Exception:  # pragma: no cover - git absent or not a repo
        return "unknown"


def _assert_watertight(slabs: list[Slab]) -> None:
    """D21: every object watertight and winding-consistent; the union is neither.

    On plain slabs this is trivially satisfied and costs nothing. It is here because it
    is what catches Phase 6, where swept and revolved primitives produce degenerate caps
    and duplicated seam vertices that a box never could.
    """
    for s in slabs:
        m = s.mesh()
        if not m.is_watertight:
            raise ValueError(f"object {s.id} is not watertight (D21)")
        if not m.is_winding_consistent:
            raise ValueError(f"object {s.id} has inconsistent winding (D21)")


def _faces_block(slabs: list[Slab]) -> list[dict[str, Any]]:
    """Flat scene-wide face registry. Index IS the per-point `face_id` (SCHEMA.md §2.3)."""
    out = []
    for i, s in enumerate(slabs):
        for j, name in enumerate(SLAB_FACES):
            plane = s.face_plane(name)
            out.append({
                "face_id": 6 * i + j,
                "ref": f"{s.id}:{name}",
                "object": s.id,
                "name": name,
                "plane": {"n": [float(v) for v in plane.n], "d": float(plane.d)},
                "surface": None,
                "area_mm2": float(s.face_area(name)),
            })
    return out


def generate_scene(cfg: dict[str, Any], seed: int) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """Generate one scene. Returns `(scene_json, arrays)`.

    `arrays` is keyed as the content hash expects: ``"cloud.npz:xyz"``, ``"seams.npz:seam_0"``.
    """
    streams = Streams(seed)
    spec, quality_level = sample_joint(cfg, streams)

    # --- placement, substream 3 -------------------------------------------------
    g3 = streams["placement"]
    span = float(cfg["assembly_translation_mm"])
    T_world_joint = (
        translate(*(g3.uniform(-span, span, size=3) * np.array([1.0, 1.0, 0.0])))
        @ rot_z(float(g3.uniform(0.0, 360.0)))
    )

    slabs, seams = build_t_joint(spec, T_world_joint)
    _assert_watertight(slabs)

    # --- surface sampling, substream 4 ------------------------------------------
    g4 = streams["surface_sample"]
    density = float(g4.uniform(*cfg["density_per_mm2"]))
    cloud = sample_scene_surface(slabs, density, g4)

    # --- sensor, substreams 5-6 --------------------------------------------------
    g5 = streams["camera"]
    profile = str(g5.choice(cfg["sensor_profiles"]))
    prof = SENSOR_PROFILES[profile]
    standoff = float(g5.uniform(max(prof["min_z_mm"], cfg["standoff_mm"][0]),
                                cfg["standoff_mm"][1]))
    elevation = float(g5.uniform(*cfg["elevation_deg"]))
    f = prof["focal_px"]
    K = [[f, 0.0, 640.0], [0.0, f, 360.0], [0.0, 0.0, 1.0]]

    # --- seams ------------------------------------------------------------------
    density_per_mm = 10.0
    arrays: dict[str, np.ndarray] = {}
    seam_blocks = []
    # Seam ids follow the documented sort (SCHEMA.md §2.7) so they are stable.
    ordered = sorted(
        seams,
        key=lambda s: (not s.weldable, s.face_pair[0], s.face_pair[1],
                       tuple(np.round(s.p0, 6)), tuple(np.round(s.p1, 6))),
    )
    for i, s in enumerate(ordered):
        pts = sample_polyline(s.p0, s.p1, density_per_mm)
        n = len(pts)
        arrays[f"seams.npz:seam_{i}"] = pts.astype(np.float32)
        arrays[f"seams.npz:seam_{i}_root"] = sample_polyline(
            s.root_p0, s.root_p1, density_per_mm).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_gapmid"] = sample_polyline(
            s.gapmid_p0, s.gapmid_p1, density_per_mm).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_s"] = np.linspace(
            0.0, s.length_mm, n).astype(np.float32)
        tangent = (s.p1 - s.p0) / max(s.length_mm, 1e-12)
        arrays[f"seams.npz:seam_{i}_tangent"] = np.tile(tangent, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_approach"] = np.tile(s.approach, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_nA"] = np.tile(s.n_a, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_nB"] = np.tile(s.n_b, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_visible"] = np.ones(n, dtype=bool)

        seam_blocks.append({
            "id": i,
            "weldable": bool(s.weldable),
            "reject_reason": s.reject_reason,
            "face_pair": list(s.face_pair),
            "parametric": {"kind": "line",
                           "p0_mm": [float(v) for v in s.p0],
                           "p1_mm": [float(v) for v in s.p1]},
            "length_mm": float(s.length_mm),
            "dihedral_deg": float(s.dihedral_deg),
            "sampled": {"array": f"seam_{i}", "density_per_mm": density_per_mm, "n": n},
            "occluded_fraction": 0.0,   # no camera in Phase 1; Phase 3 fills this in
        })

    for k, v in cloud.items():
        arrays[f"cloud.npz:{k}"] = v

    # --- joint frame: from seam 0 (SCHEMA.md §1.1) --------------------------------
    s0 = ordered[0]
    x_axis = (s0.p1 - s0.p0) / max(s0.length_mm, 1e-12)
    z_axis = s0.approach
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    T_joint = np.eye(4)
    T_joint[:3, 0], T_joint[:3, 1], T_joint[:3, 2] = x_axis, y_axis, z_axis
    T_joint[:3, 3] = s0.p0

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
            "type": cfg["joint_type"],
            "seam_shape": cfg["seam_shape"],
            "quality_level": quality_level,
            "contact_mode": "free",          # D12: Phase 1 is fixture-free
            "included_angle_deg": spec.included_angle_deg,
            "stack_offset_mm": None,         # T-joints carry no stack offset (D18)
            "prep": cfg["prep"],
        },
        "seam_definition": "nominal",        # D19
        "accessibility": cfg["accessibility"],
        "objects": [
            {
                "id": s.id, "role": s.role, "object_id": s.object_id,
                "primitive": "slab",
                "dims_mm": [float(v) for v in s.dims_mm],
                "thickness_mm": float(s.thickness_mm),
                "part_geometry_id": s.part_geometry_id,
                "mesh": None,
                "T_world_part": [[float(v) for v in row] for row in s.T_world_part],
            }
            for s in slabs
        ],
        "faces": _faces_block(slabs),
        "fit": {
            "root_gap_mm": spec.root_gap_mm,
            "linear_misalignment_mm": spec.linear_misalignment_mm,
            "angular_misalignment_deg": spec.angular_misalignment_deg,
            "throat_thickness_mm": spec.throat_thickness_mm,
        },
        "T_world_joint": [[float(v) for v in row] for row in T_joint],
        "seams": seam_blocks,
        "tacks": None,                        # Phase 7
        "camera": {
            "model": "pinhole", "K": K, "width": 1280, "height": 720,
            "T_world_cam": np.eye(4).tolist(),   # Phase 3 poses the camera properly
            "standoff_mm": standoff,
            "elevation_deg": elevation,
        },
        "noise_model": {
            "kind": "stereo_z2",
            "profile": profile,
            "subpixel_px": prof["subpixel_px"],
            "baseline_mm": prof["baseline_mm"],
            "focal_px": prof["focal_px"],
            "min_z_mm": prof["min_z_mm"],
            "lateral_sigma_px": 0.8,
            "grazing_dropout_deg": 75.0,
            "seed": int(seed),
        },
        "cloud": {
            "file": "cloud.npz", "frame": "world",
            "sampling_mode": "area_uniform",
            "density_per_mm2": density,
            "n_points": int(len(cloud["xyz"])),
        },
        "rgb": None,
        "depth": None,
        # EXCLUDED from the content hash (SCHEMA.md §6.2) - holds a timestamp.
        "provenance": {
            "created_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "git_commit": _git_commit(),
            "python": platform.python_version(),
            "numpy": np.__version__,
        },
    }
    return scene, arrays
