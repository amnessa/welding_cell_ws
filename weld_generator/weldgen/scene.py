"""Scene assembly — produces a dict conforming to docs/scene.schema.json v2.0.0."""

from __future__ import annotations

import datetime as _dt
import platform
import subprocess
from typing import Any

import numpy as np

from . import GENERATOR_VERSION, SCHEMA_VERSION
from .accessibility import d19_curves, enumerate_candidates
from .config import SENSOR_PROFILES, geometry_config, sample_joint
from .geom import SLAB_FACES, Slab, approach_dir, rot_x, rot_y, rot_z, translate
from .hashing import config_id, twin_key
from .layouts import build as build_layout
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
    spec, quality_level, joint_type = sample_joint(cfg, streams)

    # --- placement, substream 3 -------------------------------------------------
    # Fixture presence, pose and dims live here (not in substream 0) so that turning the
    # fixture on or off leaves workpiece geometry bit-identical - SCHEMA.md §6.1/§6.4.
    g3 = streams["placement"]
    span = float(cfg["assembly_translation_mm"])
    fixture_on = bool(cfg.get("fixture_present", False))
    tilt = float(cfg["fixture_tilt_deg"]) if fixture_on else 0.0

    T_world_joint = (
        translate(*(g3.uniform(-span, span, size=3) * np.array([1.0, 1.0, 0.0])))
        @ translate(0.0, 0.0, float(g3.uniform(*cfg["fixture_height_mm"])) if fixture_on else 0.0)
        @ rot_z(float(g3.uniform(0.0, 360.0)))
        @ rot_x(float(g3.uniform(-tilt, tilt)))
        @ rot_y(float(g3.uniform(-tilt, tilt)))
    )

    slabs = build_layout(spec, joint_type, T_world_joint)

    contact_mode = "free"
    if fixture_on:
        # The fixture and the assembly share T_world_joint, so they tilt together and
        # stay physically consistent: A rests flat on the working surface.
        fw, fl, ft = (float(g3.uniform(*cfg["fixture_dims_mm"][k]))
                      for k in ("length", "width", "thickness"))
        drop = float(np.min([np.min((s_.mesh().vertices - T_world_joint[:3, 3])
                                    @ T_world_joint[:3, 2]) for s_ in slabs]))
        slabs.append(Slab("F", "fixture", 255, (fw, fl, ft),
                          T_world_joint @ translate(0.0, 0.0, drop - ft / 2.0)))
        contact_mode = "flat"

    _assert_watertight(slabs)

    # --- seams: derived by the D4 rule, not declared by the constructor -----------
    # `contact_tol_mm` decides whether two faces are adjacent at all. A fixed constant
    # cannot serve both a 1 mm gap on 12 mm plate and a 5 mm gap on 2 mm sheet: too tight
    # and the below_D scenes lose their seams, too loose and a 2 mm edge joint admits
    # every face pair within 8 mm. So it tracks the gap it has to accommodate, and is
    # stored per scene so every rejection stays reproducible.
    access = {k: (v.copy() if isinstance(v, dict) else v)
              for k, v in cfg["accessibility"].items()}
    access["contact_tol_mm"] = float(
        np.clip(2.0 * spec.root_gap_mm + 1.0, 2.0, 12.0))
    cands = enumerate_candidates(slabs, access)

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
    by_id = {s_.id: s_ for s_ in slabs}
    # Facing pairs carry no line (p0 is None) and cannot be sampled; degenerate
    # zero-length runs are numerical debris. Neither belongs in `seams[]`.
    ordered = [c for c in cands
               if c.p0 is not None and c.length_mm > 1e-6]   # SCHEMA.md §2.7 order

    for i, c in enumerate(ordered):
        pts = sample_polyline(c.p0, c.p1, density_per_mm)
        n = len(pts)
        ida, fa = c.face_pair[0].split(":")
        idb, fb = c.face_pair[1].split(":")
        root, gapmid = d19_curves(by_id[ida], fa, by_id[idb], fb, c.p0, c.p1, n)

        arrays[f"seams.npz:seam_{i}"] = pts.astype(np.float32)
        arrays[f"seams.npz:seam_{i}_root"] = root.astype(np.float32)
        arrays[f"seams.npz:seam_{i}_gapmid"] = gapmid.astype(np.float32)
        arrays[f"seams.npz:seam_{i}_s"] = np.linspace(
            0.0, c.length_mm, n).astype(np.float32)
        tangent = (c.p1 - c.p0) / max(c.length_mm, 1e-12)
        appr = approach_dir(c.n_a, c.n_b)
        arrays[f"seams.npz:seam_{i}_tangent"] = np.tile(tangent, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_approach"] = np.tile(appr, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_nA"] = np.tile(c.n_a, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_nB"] = np.tile(c.n_b, (n, 1)).astype(np.float32)
        arrays[f"seams.npz:seam_{i}_visible"] = np.ones(n, dtype=bool)

        seam_blocks.append({
            "id": i,
            "weldable": bool(c.weldable),
            "reject_reason": c.reject_reason,
            "face_pair": list(c.face_pair),
            "parametric": {"kind": "line",
                           "p0_mm": [float(v) for v in c.p0],
                           "p1_mm": [float(v) for v in c.p1]},
            "length_mm": float(c.length_mm),
            "dihedral_deg": float(c.dihedral_deg),
            "sampled": {"array": f"seam_{i}", "density_per_mm": density_per_mm, "n": n},
            "occluded_fraction": 0.0,   # no camera yet; Phase 3 fills this in
        })

    for k, v in cloud.items():
        arrays[f"cloud.npz:{k}"] = v

    # --- joint frame: from seam 0 (SCHEMA.md §1.1) --------------------------------
    s0 = ordered[0]
    x_axis = (s0.p1 - s0.p0) / max(s0.length_mm, 1e-12)
    z_axis = approach_dir(s0.n_a, s0.n_b)
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
            "type": joint_type,
            "seam_shape": cfg["seam_shape"],
            "quality_level": quality_level,
            "contact_mode": contact_mode,    # D12: "free" iff no fixture
            "included_angle_deg": spec.included_angle_deg,
            "stack_offset_mm": spec.stack_offset_mm,
            "prep": cfg["prep"],
        },
        "seam_definition": "nominal",        # D19
        "accessibility": access,
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
