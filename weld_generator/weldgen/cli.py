"""Command line entry point.

    python -m weldgen generate --config configs/phase1.yaml --n 16 --out out/phase1
    python -m weldgen verify   --out out/phase1
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .config import load_config
from .hashing import content_hash
from .scene import generate_scene
from .writer import index_row, split_arrays, write_index, write_scene


def cmd_generate(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    rows = []
    for i in range(args.n):
        seed = args.seed0 + i
        scene, arrays = generate_scene(cfg, seed)
        write_scene(args.out, scene, arrays, emit_meshes=args.emit_meshes)
        digest = content_hash(scene, arrays)
        rows.append(index_row(scene, digest))
        if not args.quiet:
            print(f"{scene['scene_id']}  seeds={seed}  "
                  f"pts={scene['cloud']['n_points']:>7}  "
                  f"t={scene['fit']['root_gap_mm']:.2f}mm gap  "
                  f"{scene['joint']['quality_level']:>7}  {digest[:12]}")
    write_index(args.out, rows)
    print(f"\n{len(rows)} scenes -> {args.out}")
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    """Re-hash every scene on disk against its stored `scene.sha256`."""
    import numpy as np

    root = Path(args.out)
    bad = 0
    scene_dirs = sorted(p for p in root.iterdir() if (p / "scene.json").exists())
    for d in scene_dirs:
        scene = json.loads((d / "scene.json").read_text())
        arrays = {}
        for npz in d.glob("*.npz"):
            with np.load(npz) as z:
                for k in z.files:
                    arrays[f"{npz.name}:{k}"] = z[k]
        want = (d / "scene.sha256").read_text().strip()
        got = content_hash(scene, arrays)
        if want != got:
            bad += 1
            print(f"MISMATCH {d.name}\n  stored {want}\n  recomputed {got}")
    print(f"{len(scene_dirs) - bad}/{len(scene_dirs)} scenes verify")
    return 1 if bad else 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="weldgen")
    sub = p.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("generate", help="generate scenes")
    g.add_argument("--config", default=None)
    g.add_argument("--n", type=int, default=8)
    g.add_argument("--seed0", type=int, default=8412337)
    g.add_argument("--out", default="out/phase1")
    g.add_argument("--emit-meshes", action="store_true",
                   help="also write per-scene PLYs (convenience; not content-hashed)")
    g.add_argument("--quiet", action="store_true")
    g.set_defaults(func=cmd_generate)

    v = sub.add_parser("verify", help="re-hash scenes on disk")
    v.add_argument("--out", default="out/phase1")
    v.set_defaults(func=cmd_verify)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
