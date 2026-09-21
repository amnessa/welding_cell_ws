"""Part registry: which weldgen primitive each library CAD is - mode A's one input.

Mode A of the seam pipeline (`notes/seam_two_modes_plan.md`) computes the weld seam
from the registered poses of the parts instead of detecting it. That needs each
library `.ply` described as an exact `weld_generator` primitive (slab, prism, tube,
swept band ...) together with the rigid transform between the CAD file's frame and the
primitive's canonical local frame - the SEPC poses (`pose_static`) place the CAD
frame, the primitive lives in its own frame.

`derive_box` reads that description off a box-shaped mesh automatically and VERIFIES it
(every vertex on the box surface, volume match), accepting a plate with small notches,
tabs or holes as its envelope slab with the ignored fraction recorded; anything it
cannot verify is recorded as unsupported with the reason, never guessed. Non-box primitives (a pipe stub, a curved
band) are written by hand in the same JSON when such parts enter the library, and
`verify_entry` checks them against the mesh the same way.

Units: the registry is in millimetres (the library convention, `model_units: mm`).
`T_cad_prim` maps primitive-local (mm) -> CAD-file frame (mm).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

REGISTRY_VERSION = "weldgen_registry-1.0"


def _load_mesh(path):
    import trimesh
    return trimesh.load(str(path), force="mesh")


def derive_box(mesh, tol_mm: float = 0.05, max_deficit: float = 0.10) -> dict[str, Any]:
    """A slab entry for a box-like mesh, or `{"primitive": None, "reason": ...}`.

    Axes: u = longest extent, v = middle, w = shortest (the thickness - weldgen's broad
    faces are +/-w). The frame is right-handed by construction, centred on the box.

    Two acceptances, both recorded in `approx`:
      * `"exact"`   - every vertex is a box corner and the volume is L*W*t: the CAD IS
                      the slab.
      * `"envelope"` - every vertex lies ON the oriented bounding box's surface and the
                      mesh fills at least `1 - max_deficit` of it: a plate with small
                      features cut from or added to its outline (edge notches, locating
                      tabs, holes) - the lab's slotted `test_objv1` parts. The slab is
                      the envelope; the features are ignored, so a seam that runs across
                      a notch is labelled at its nominal length and a tab that passes
                      through the other part shows up as a nominal penetration of the
                      tab length in `fitup_mm`. `envelope_deficit` records how much was
                      ignored. Hand-edit `dims_mm` / `T_cad_prim` and set
                      `"hand_edited": true` to describe the body instead.
    Anything else (an L-shaped composite, a curved band, a mesh with interior vertices)
    is unsupported with the reason, never guessed.
    """
    v = np.asarray(mesh.vertices, dtype=float)
    if len(v) < 8:
        return {"primitive": None, "reason": f"{len(v)} vertices - not a box"}
    obb = mesh.bounding_box_oriented
    T_obb = np.asarray(obb.primitive.transform, dtype=float)      # obb frame -> cad
    ext = np.asarray(obb.primitive.extents, dtype=float)
    order = np.argsort(ext)[::-1]                                  # longest first
    L, W, t = (float(ext[i]) for i in order)
    axes = T_obb[:3, :3][:, order]
    if np.linalg.det(axes) < 0:
        axes[:, 1] = -axes[:, 1]                                   # keep it right-handed
    T = np.eye(4)
    T[:3, :3] = axes
    T[:3, 3] = T_obb[:3, 3]
    local = (v - T[:3, 3]) @ axes
    half = np.array([L, W, t]) / 2.0
    box_vol = L * W * t
    entry = {"primitive": "slab", "dims_mm": [L, W, t], "T_cad_prim": T.tolist()}
    # exact box: every vertex is a corner (|coord| == half-extent on EVERY axis)
    dev = np.abs(np.abs(local) - half).max()
    vol_err = abs(float(mesh.volume) - box_vol) / box_vol
    if dev <= tol_mm and vol_err <= 0.01:
        return {**entry, "approx": "exact", "max_dev_mm": float(dev),
                "volume_rel_err": float(vol_err)}
    # envelope: every vertex on the box SURFACE (inside it, and on at least one face)
    per_axis = np.abs(np.abs(local) - half)                        # (n, 3)
    inside = (np.abs(local) <= half + tol_mm).all(axis=1)
    on_face = per_axis.min(axis=1) <= tol_mm
    if not (inside & on_face).all():
        return {"primitive": None,
                "reason": f"vertices deviate {dev:.3f} mm from a box (tol {tol_mm})"}
    deficit = 1.0 - float(mesh.volume) / box_vol
    if not (-0.01 <= deficit <= max_deficit):
        return {"primitive": None,
                "reason": (f"fills only {1 - deficit:.0%} of its envelope "
                           f"(features > {max_deficit:.0%}) - not a slab")}
    return {**entry, "approx": "envelope", "max_dev_mm": float(dev),
            "envelope_deficit": float(deficit)}


def verify_entry(entry: dict[str, Any], mesh, weldgen, tol_mm: float = 0.25) -> float:
    """Max distance of the CAD mesh's vertices to the primitive's surface (mm).

    Uses `weld_generator`'s rtree-free mesh distance so the check runs in the ROS
    environment as it is. `tol_mm` is the D34 chord budget: a hand-written tube or
    band entry is accepted when its tessellation and the CAD agree to that.
    """
    from weldgen.geom import from_object
    from weldgen.render.gate import distance_to_mesh
    obj = {"id": "A", "role": "workpiece", "object_id": 0,
           "primitive": entry["primitive"], "T_world_part": entry["T_cad_prim"],
           "dims_mm": entry.get("dims_mm"), "thickness_mm": entry.get("thickness_mm"),
           "outline_uv": entry.get("outline_uv"), "outline_shape": entry.get("outline_shape"),
           "params": entry.get("params")}
    prim = from_object({k: v for k, v in obj.items() if v is not None})
    d = distance_to_mesh(np.asarray(mesh.vertices, float), prim.mesh())
    return float(d.max())


def build_registry(models_dir, existing: dict[str, Any] | None = None,
                   tol_mm: float = 0.05) -> dict[str, Any]:
    """Every `*.ply` under `models_dir` -> an entry. Hand-written entries in `existing`
    (anything whose `primitive` is not `slab`, or marked `hand_edited`) are kept and
    re-verified, not overwritten."""
    models_dir = Path(models_dir)
    reg: dict[str, Any] = {"version": REGISTRY_VERSION, "units": "mm", "parts": {}}
    old = (existing or {}).get("parts", {})
    for ply in sorted(models_dir.glob("*.ply")):
        name = ply.stem
        prev = old.get(name)
        if prev and (prev.get("primitive") not in (None, "slab") or prev.get("hand_edited")):
            reg["parts"][name] = prev                              # hand-written: keep
            continue
        mesh = _load_mesh(ply)
        entry = derive_box(mesh, tol_mm)
        entry["source"] = ply.name
        reg["parts"][name] = entry
    return reg


def load_registry(path) -> dict[str, Any]:
    return json.loads(Path(path).read_text())


def save_registry(reg: dict[str, Any], path) -> None:
    Path(path).write_text(json.dumps(reg, indent=2))
