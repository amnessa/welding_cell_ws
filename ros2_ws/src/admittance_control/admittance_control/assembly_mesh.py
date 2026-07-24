"""Build a watertight CAD mesh of an assembled scene from the ICP node's manifest.

The SEPC point cloud (``static_env.ply``) has no faces, so it cannot go to the
FoundationPose host as a new PPF model -- the server builds a model by sampling a
*surface*. But every part in the SEPC was produced by sampling a known CAD at an
ICP-refined pose, and the ICP node records exactly that in ``assembly.json`` (each
object's CAD filename + its 4x4 ``pose_static``). So rather than reconstruct a mesh
from the points (lossy at 2500 pts, and it needs a Delaunay/graph-cut/manifold-repair
stack), we re-instantiate the *original CADs* at their stored poses and boolean-union
them. The result is CAD-exact, watertight by construction, and in the same millimetre
convention as the other library models.

Units. The ICP node samples each CAD scaled to metres (``mesh_scale``, mm->m) and
``pose_static`` maps those metre-scale points into the static frame. So here we scale
each CAD to metres, place it, union, then scale the *output* back to millimetres so the
host (which multiplies by ``MESH_SCALE=0.001`` on load) reads it at true size like any
other ``.ply``. The mesh is re-centred on its bounding-box centre to give the new part
a clean, origin-centred model frame (classification is translation-invariant, so this
only matters for later pose reporting).

Dependencies: ``trimesh`` as the boolean frontend and ``manifold3d`` as its backend
(``pip install trimesh manifold3d``). trimesh is used rather than calling manifold3d
directly because it abstracts the version-sensitive manifold API and gives the
watertight/volume checks for free.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from admittance_control.icp import load_ply_mesh


def _triangulate(faces: List[Tuple[int, ...]]) -> np.ndarray:
    """Fan-triangulate polygon faces into an (M,3) int array.

    The FreeCAD CADs here are already triangulated, but a quad or n-gon export
    would otherwise be dropped, so fan them rather than assume triangles.
    """
    tris: List[Tuple[int, int, int]] = []
    for f in faces:
        for i in range(1, len(f) - 1):
            tris.append((f[0], f[i], f[i + 1]))
    return np.asarray(tris, dtype=np.int64)


def load_assembly(assembly_json) -> Tuple[str, List[Tuple[str, np.ndarray]]]:
    """Read ``assembly.json`` -> (static_frame, [(cad_filename, pose4x4), ...])."""
    data = json.loads(Path(assembly_json).read_text())
    objects: List[Tuple[str, np.ndarray]] = []
    for o in data.get('objects', []):
        pose = np.asarray(o['pose_static'], dtype=np.float64).reshape(4, 4)
        objects.append((o['model'], pose))
    return data.get('static_frame', 'base_link'), objects


def _placed_trimesh(model_name: str, pose: np.ndarray, models_dir, mesh_scale: float):
    import trimesh
    verts, faces = load_ply_mesh(Path(models_dir) / model_name)
    tris = _triangulate(faces)
    if len(tris) == 0:
        raise ValueError(
            f"{model_name} has no faces -- it is a point cloud, not a CAD mesh; "
            "the assembly mesh can only be built from faced CAD models.")
    mesh = trimesh.Trimesh(vertices=verts * mesh_scale, faces=tris, process=False)
    mesh.apply_transform(pose)                       # model frame -> static (metres)
    return mesh


def build_assembly_mesh(objects: List[Tuple[str, np.ndarray]], models_dir,
                        mesh_scale: float = 0.001, recenter: bool = True,
                        to_millimetres: bool = True) -> Tuple[object, Dict]:
    """Re-instantiate each CAD at its pose and boolean-union them.

    Returns ``(trimesh.Trimesh, stats)`` where ``stats`` reports watertightness,
    volume and counts so the caller can refuse to upload a broken mesh.
    """
    try:
        import trimesh
    except ImportError as exc:  # pragma: no cover - env-dependent
        raise RuntimeError("assembly mesh needs trimesh (pip install trimesh "
                           "manifold3d)") from exc
    if not objects:
        raise ValueError("assembly has no objects; run ~/save_object first")

    parts = [_placed_trimesh(name, pose, models_dir, mesh_scale)
             for name, pose in objects]

    if len(parts) == 1:
        mesh = parts[0]
    else:
        # Union picks up manifold3d automatically when installed; a watertight
        # union of watertight solids is itself watertight (disjoint parts just
        # come back as multiple components in one mesh, which is fine).
        try:
            mesh = trimesh.boolean.union(parts)
        except Exception as exc:  # noqa: BLE001 - surface a backend-less env clearly
            raise RuntimeError(
                f"boolean union failed ({exc}). Is manifold3d installed? "
                "pip install manifold3d") from exc

    if recenter:
        mesh.apply_translation(-mesh.bounds.mean(axis=0))
    if to_millimetres:
        mesh.apply_scale(1000.0)                     # metres -> mm library convention

    stats = {
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "components": int(mesh.body_count),
        "watertight": bool(mesh.is_watertight),
        "volume_mm3": float(mesh.volume) if mesh.is_volume else 0.0,
        "extents_mm": np.round(mesh.extents, 2).tolist(),
    }
    return mesh, stats


def build_and_write(assembly_json, models_dir, out_path,
                    mesh_scale: float = 0.001) -> Dict:
    """End-to-end: read the manifest, build the union, write the .ply. Returns stats
    plus the output path, for a caller that just wants the file on disk."""
    _, objects = load_assembly(assembly_json)
    mesh, stats = build_assembly_mesh(objects, models_dir, mesh_scale=mesh_scale)
    out_path = Path(out_path)
    mesh.export(str(out_path))
    stats["path"] = str(out_path)
    return stats
