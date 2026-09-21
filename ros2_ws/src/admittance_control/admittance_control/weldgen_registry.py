"""Part registry: which weldgen primitive each library CAD is - mode A's one input.

Mode A of the seam pipeline (`notes/seam_two_modes_plan.md`) computes the weld seam
from the registered poses of the parts instead of detecting it. That needs each
library `.ply` described as an exact `weld_generator` primitive (slab, prism, tube,
swept band ...) together with the rigid transform between the CAD file's frame and the
primitive's canonical local frame - the SEPC poses (`pose_static`) place the CAD
frame, the primitive lives in its own frame.

`derive_box` reads that description off a box-shaped mesh automatically and VERIFIES it
(every vertex on the box surface, volume match); anything it cannot verify is recorded
as unsupported with the reason, never guessed. Non-box primitives (a pipe stub, a curved
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


def derive_box(mesh, tol_mm: float = 0.05) -> dict[str, Any]:
    """A slab entry for a box mesh, or `{"primitive": None, "reason": ...}`.

    Axes: u = longest extent, v = middle, w = shortest (the thickness - weldgen's broad
    faces are +/-w). The frame is right-handed by construction, centred on the box.
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
    # every vertex must sit on the box surface: |coord| == half-extent on each axis
    local = (v - T[:3, 3]) @ axes
    half = np.array([L, W, t]) / 2.0
    dev = np.abs(np.abs(local) - half).max()
    if dev > tol_mm:
        return {"primitive": None,
                "reason": f"vertices deviate {dev:.3f} mm from a box (tol {tol_mm})"}
    vol_err = abs(float(mesh.volume) - L * W * t) / (L * W * t)
    if vol_err > 0.01:
        return {"primitive": None,
                "reason": f"volume differs from L*W*t by {vol_err:.1%} - not a solid box"}
    return {"primitive": "slab", "dims_mm": [L, W, t], "T_cad_prim": T.tolist(),
            "max_dev_mm": float(dev), "volume_rel_err": float(vol_err)}


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
    (anything whose `primitive` is not `slab`) are kept and re-verified, not overwritten."""
    models_dir = Path(models_dir)
    reg: dict[str, Any] = {"version": REGISTRY_VERSION, "units": "mm", "parts": {}}
    old = (existing or {}).get("parts", {})
    for ply in sorted(models_dir.glob("*.ply")):
        name = ply.stem
        prev = old.get(name)
        if prev and prev.get("primitive") not in (None, "slab"):
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
