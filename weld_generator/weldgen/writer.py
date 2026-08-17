"""Scene serialisation — SCHEMA.md §3.1 and §5.

Layout per scene:

    <scene_id>/
      scene.json
      cloud.npz
      seams.npz
      scene.sha256     <- the content hash; it cannot live inside scene.json,
                          because a file cannot contain its own hash
      mesh_<id>.ply    <- only with emit_meshes=True (convenience, NOT hashed)
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .hashing import canonical_json, content_hash


def split_arrays(arrays: Mapping[str, np.ndarray]) -> dict[str, dict[str, np.ndarray]]:
    """Regroup ``"cloud.npz:xyz"`` keys into per-file bundles."""
    out: dict[str, dict[str, np.ndarray]] = {}
    for key, arr in arrays.items():
        fname, _, member = key.partition(":")
        out.setdefault(fname, {})[member] = arr
    return out


def write_scene(
    out_dir: str | Path,
    scene: Mapping[str, Any],
    arrays: Mapping[str, np.ndarray],
    emit_meshes: bool = False,
    slabs: list | None = None,
) -> Path:
    """Write one scene directory and return its path."""
    root = Path(out_dir) / scene["scene_id"]
    root.mkdir(parents=True, exist_ok=True)

    (root / "scene.json").write_text(json.dumps(scene, indent=2, sort_keys=True, allow_nan=False))

    for fname, members in split_arrays(arrays).items():
        np.savez_compressed(root / fname, **members)

    digest = content_hash(scene, arrays)
    (root / "scene.sha256").write_text(digest + "\n")

    if emit_meshes and slabs:
        for s in slabs:
            s.mesh().export(root / f"mesh_{s.id}.ply")

    return root


def index_row(scene: Mapping[str, Any], digest: str) -> dict[str, Any]:
    """One denormalised `index.jsonl` row — SCHEMA.md §5.3.

    Deliberately flat so the Phase 4 plots are a groupby on this one file:
    `df.groupby(["twin_key", "fixture_present"])` is the whole fixture ablation.
    """
    ts = [o["thickness_mm"] for o in scene["objects"] if o["role"] == "workpiece"]
    weldable = [s for s in scene["seams"] if s["weldable"]]
    primary = [s for s in weldable if s["matches_joint_type"]]
    return {
        "emitted": True,
        "scene_id": scene["scene_id"],
        "config_id": scene["config_id"],
        "seed": scene["seed"],
        "tier": scene["tier"],
        "twin_key": scene["twin_key"],
        "joint_type": scene["joint"]["type"],
        "seam_shape": scene["joint"]["seam_shape"],
        "quality_level": scene["joint"]["quality_level"],
        "contact_mode": scene["joint"]["contact_mode"],
        "included_angle_deg": scene["joint"]["included_angle_deg"],
        "fixture_present": any(o["role"] == "fixture" for o in scene["objects"]),
        "t_min_mm": min(ts),
        "t_max_mm": max(ts),
        "root_gap_mm": scene["fit"]["root_gap_mm"],
        "linear_misalignment_mm": scene["fit"]["linear_misalignment_mm"],
        "angular_misalignment_deg": scene["fit"]["angular_misalignment_deg"],
        "n_seams": len(scene["seams"]),
        # `n_weldable` is reachability; `n_primary` additionally requires the seam to be
        # one this joint type declares. The gap between them is how often a joint-type
        # label under-describes its own scene.
        "n_weldable": len(weldable),
        "n_primary": len(primary),
        "n_fixture_contact": sum(
            1 for s in scene["seams"] if s["reject_reason"] == "fixture_contact"),
        "total_seam_length_mm": float(sum(s["length_mm"] for s in weldable)),
        "mean_occluded_fraction": float(
            np.mean([s["occluded_fraction"] for s in scene["seams"]])),
        # Occlusion and framing are different physics (SCHEMA.md §5.2). Averaged over
        # PRIMARY seams, because those are the ones a metric is computed against.
        "mean_in_frame_fraction": float(
            np.mean([s["in_frame_fraction"] for s in primary])) if primary else None,
        "best_visible_fraction": float(
            max(s["visible_fraction"] for s in primary)) if primary else 0.0,
        "n_points": scene["cloud"]["n_points"],
        "density_per_mm2": scene["cloud"]["density_per_mm2"],
        "sampling_mode": scene["cloud"]["sampling_mode"],
        "sensor_profile": scene["noise_model"]["profile"],
        "standoff_mm": scene["camera"]["standoff_mm"],
        "elevation_deg": scene["camera"]["elevation_deg"],
        "part_geometry_ids": [o["part_geometry_id"] for o in scene["objects"]],
        "content_hash": digest,
    }


def skipped_row(seed: int, config_id_: str, reason: str, detail: str) -> dict[str, Any]:
    """One `index.jsonl` row for a seed that produced no emittable scene.

    Skipped seeds go in the index, not only to stdout. The omission policy conditions the
    dataset on the camera - and it does so unevenly across joint types - so the selection
    has to be *characterisable* rather than merely reported: `df[~df.emitted]` recovers
    exactly what was dropped and why. A yield percentage in a README is not a substitute,
    because it cannot be regrouped, plotted, or checked.

    `emitted` is what separates the two row kinds; every other field is null on a skip
    because there is no scene to read them from.
    """
    return {"emitted": False, "seed": int(seed), "config_id": config_id_,
            "skip_reason": reason, "skip_detail": detail}


def write_index(out_dir: str | Path, rows: list[dict[str, Any]]) -> Path:
    path = Path(out_dir) / "index.jsonl"
    path.write_text("".join(canonical_json(r) + "\n" for r in rows))
    return path
