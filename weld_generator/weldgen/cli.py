"""Command line entry point.

    python -m weldgen generate --config configs/phase1.yaml --n 16 --out out/phase1
    python -m weldgen verify   --out out/phase1
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from collections import Counter

from .config import load_config
from .hashing import config_id
from .hashing import content_hash
from .scene import SceneRejected, generate_scene
from .writer import (index_row, skipped_row, split_arrays, write_index,
                     write_scene)


def cmd_generate(args: argparse.Namespace) -> int:
    """Write `n` scenes, skipping seeds that carry no usable ground truth.

    Two reasons a seed is skipped. A root gap comparable to the plate thickness leaves no
    face pair adjacent, so there is no seam to construct (`NoSeamsFound`); or every seam
    there is happens to be hidden from the camera (`NoVisibleSeams`). Neither is a hard
    example - both are empty ones - and `NoSeamsFound` used to abort the whole run, so a
    mixed-joint config could not be generated at all.

    Skipping stays deterministic: which seeds fail is a pure function of the config, so the
    same config and seed range give the same scenes.

    Seeds are NOT backfilled past `seed0 + n`. Both arms of an ablation twin must attempt
    the same seeds, and they do not fail on the same ones; backfilling would give them
    different seed sets and silently break the pairing. Note that visibility depends on the
    camera, so a sensor-profile or sampling-mode twin loses different seeds than its
    partner - `twin_key` pairs what survives in both and drops the singletons.
    """
    cfg = load_config(args.config)
    cid = config_id(cfg)
    gen = generate_scene
    if cfg.get("seam_families"):
        from .scene_curved import generate_curved_scene as gen   # Phase 6b, D29
    rows, skipped = [], []
    for i in range(args.n):
        seed = args.seed0 + i
        try:
            scene, arrays = gen(cfg, seed)
        except SceneRejected as e:
            # Recorded in the index, not just printed: the omission policy conditions the
            # dataset unevenly across joint types, so what it dropped has to stay queryable.
            skipped.append(skipped_row(seed, cid, type(e).__name__, str(e)))
            continue
        write_scene(args.out, scene, arrays, emit_meshes=args.emit_meshes)
        digest = content_hash(scene, arrays)
        rows.append(index_row(scene, digest))
        if not args.quiet:
            print(f"{scene['scene_id']}  seeds={seed}  "
                  f"pts={scene['cloud']['n_points']:>7}  "
                  f"t={scene['fit']['root_gap_mm']:.2f}mm gap  "
                  f"{scene['joint']['quality_level']:>7}  {digest[:12]}")
    write_index(args.out, rows + skipped)
    print(f"\n{len(rows)} scenes -> {args.out}"
          f"  ({len(rows)}/{args.n} seeds = {len(rows) / max(args.n, 1):.0%} yield)")
    if skipped:
        by_reason = Counter(r["skip_reason"] for r in skipped)
        print(f"{len(skipped)} seed(s) skipped, no usable ground truth: "
              f"{dict(by_reason)}")
        print("  recorded in index.jsonl with emitted=false; "
              "`df[~df.emitted]` recovers them")
        if not args.quiet:
            for r in skipped:
                print(f"  {r['seed']}  {r['skip_reason']}: "
                      f"{r['skip_detail'].split(' (', 1)[0]}")
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
