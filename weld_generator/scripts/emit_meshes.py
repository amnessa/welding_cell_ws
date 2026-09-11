"""Write `mesh_<id>.ply` next to every scene of a corpus — SCHEMA.md §3.1, Phase 8 M1.

Meshes are NOT part of a scene (parts are parametric, `scene.json` is the truth), so this
is a convenience artefact: excluded from `scene.sha256`, regenerable from the entry at
any time through `weldgen.geom.from_object`. Phase 5's CloudCompare work and the Phase 8
renderer's spot checks both want the surfaces on disk.

    python scripts/emit_meshes.py out/bench_phase4            # class dirs or scene dirs
    python scripts/emit_meshes.py out/bench_phase4 --check    # no writes: round-trip gate

`--check` is the M1 gate on a real corpus: for every object, serialise -> `from_object`
-> serialise must be the identity, and the rebuilt mesh must be watertight (D21).
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.geom import Slab, SweptSlab, Tube, from_object  # noqa: E402
from weldgen.hashing import canonical_json  # noqa: E402


def scene_dirs(root: pathlib.Path):
    if (root / "scene.json").exists():
        yield root
        return
    for d in sorted(root.rglob("scene.json")):
        yield d.parent


def reserialise(part) -> dict:
    """The serialiser that wrote this part: plate pipeline or curved pipeline."""
    if isinstance(part, (Tube, SweptSlab)):
        from weldgen.scene_curved import _objects_block
        return _objects_block([part])[0]
    if isinstance(part, Slab) and part.role == "workpiece":
        # both pipelines write slabs identically; use the plate one
        pass
    from weldgen.scene import _object_entry
    return _object_entry(part)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="corpus root, class dir, or one scene dir")
    ap.add_argument("--check", action="store_true", help="round-trip gate only, no writes")
    ap.add_argument("--force", action="store_true", help="rewrite existing .ply files")
    args = ap.parse_args()

    n_scenes = n_parts = n_written = n_bad = 0
    t0 = time.time()
    for d in scene_dirs(pathlib.Path(args.root)):
        scene = json.loads((d / "scene.json").read_text())
        n_scenes += 1
        for entry in scene["objects"]:
            n_parts += 1
            part = from_object(entry)
            if args.check:
                back = reserialise(part)
                same = canonical_json(back) == canonical_json(entry)
                mesh = part.mesh()
                ok = same and mesh.is_watertight and mesh.is_winding_consistent \
                    and part.part_geometry_id == entry["part_geometry_id"]
                if not ok:
                    n_bad += 1
                    print(f"BAD {d.name} {entry['id']} {entry['primitive']}: "
                          f"roundtrip={same} watertight={mesh.is_watertight} "
                          f"winding={mesh.is_winding_consistent}")
                continue
            out = d / f"mesh_{entry['id']}.ply"
            if out.exists() and not args.force:
                continue
            part.mesh().export(out)
            n_written += 1
    mode = "checked" if args.check else "wrote"
    print(f"{n_scenes} scenes, {n_parts} parts, {mode} {n_bad if args.check else n_written}"
          f"{' BAD' if args.check else ' meshes'} in {time.time() - t0:.0f}s")
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
