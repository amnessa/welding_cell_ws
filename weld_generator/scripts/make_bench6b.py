#!/usr/bin/env python3
"""Build `out/bench6b` — the Phase 6b balanced benchmark corpus (step 5).

Layout matches what `baselines.dataset.balanced_corpus` expects: one directory per
joint type, each with an `index.jsonl`. The Phase 6b point is WITHIN-class family
balance: a seam class that now has curved members gets them in equal measure.

    T       60 = 10 line + 10 circle + 10 ellipse + 10 saddle + 10 rounded_rect
                 + 10 swept_path                       (the 6 fillet-bearing families)
    butt    60 = 15 line square + 15 line grooved (D35 draw, square draws filtered)
                 + 30 arc (family 7)                   (line vs arc, 30/30 by family)
    corner  60   line only \
    lap     60   line only  >  these classes have a single family by definition
    edge    60   line only /

Seed interleaving, not blocks: `balanced_corpus` selects the `per_type` LOWEST seeds,
so contiguous per-source seed blocks would make any trim (e.g. Phase 4's 50) collapse
onto whichever source got the lowest block. Each source instead walks seeds congruent
to its own residue(s) modulo the class stride, from one shared base — every seed-sorted
prefix of the class then holds the sources in their intended proportions. The arc
source holds two of butt's four residues because it wants half the class.

Rejected seeds are recorded, never backfilled from another range (same policy as
`out/bench`); a grooved-arm draw that comes out square-prep is recorded as filtered,
not emitted — the square stratum is the `line_square` arm's job.

Deterministic: a pure function of the configs referenced below and BASE_SEED.

Usage:
    python scripts/make_bench6b.py [--out out/bench6b] [--quiet]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.config import load_config                     # noqa: E402
from weldgen.scene import SceneRejected, generate_scene    # noqa: E402
from weldgen.scene_curved import generate_curved_scene     # noqa: E402
from weldgen.writer import write_scene                     # noqa: E402

BASE_SEED = 2_000_000
ATTEMPT_CAP = 12          # per source, as a multiple of its target


def _plate(cfg_name):
    return load_config(str(ROOT / "configs" / cfg_name)), generate_scene


def _curved(family: int):
    cfg = load_config(str(ROOT / "configs" / "curved_smoke.yaml"))
    cfg["seam_families"] = [family]
    return cfg, generate_curved_scene


def _not_square(scene) -> bool:
    return scene["joint"]["prep"] != "square"


#: (source_name, config loader args, residues, target, keep predicate)
STRATA: dict[str, list] = {
    "T": [
        ("line",         ("plate", "bench6a_T.yaml"), [0], 10, None),
        ("circle",       ("curved", 2),               [1], 10, None),
        ("ellipse",      ("curved", 3),               [2], 10, None),
        ("saddle",       ("curved", 4),               [3], 10, None),
        ("rounded_rect", ("curved", 5),               [4], 10, None),
        ("swept_path",   ("curved", 6),               [5], 10, None),
    ],
    "butt": [
        ("line_square",  ("plate", "bench6a_butt.yaml"), [0], 15, None),
        ("line_grooved", ("plate", "grooved_butt.yaml"), [1], 15, _not_square),
        ("arc",          ("curved", 7),                  [2, 3], 30, None),
    ],
    "corner": [("line", ("plate", "bench6a_corner.yaml"), [0], 60, None)],
    "lap":    [("line", ("plate", "bench6a_lap.yaml"),    [0], 60, None)],
    "edge":   [("line", ("plate", "bench6a_edge.yaml"),   [0], 60, None)],
}


def _seeds(residues, stride):
    i = 0
    while True:
        for r in residues:
            yield BASE_SEED + r + stride * i
        i += 1


def build(out_root: Path, quiet: bool) -> dict:
    manifest = {"base_seed": BASE_SEED, "classes": {}}
    for jt, sources in STRATA.items():
        stride = sum(len(res) for _, _, res, _, _ in sources)
        class_dir = out_root / jt
        class_dir.mkdir(parents=True, exist_ok=True)
        index = open(class_dir / "index.jsonl", "w")
        cls_manifest = {"stride": stride, "sources": {}}
        for name, loader, residues, target, keep in sources:
            cfg, gen = (_plate(loader[1]) if loader[0] == "plate"
                        else _curved(loader[1]))
            emitted, attempts, t0 = 0, 0, time.time()
            for seed in _seeds(residues, stride):
                if emitted >= target or attempts >= ATTEMPT_CAP * target:
                    break
                attempts += 1
                row = {"seed": seed, "source": name, "emitted": False}
                try:
                    scene, arrays = gen(cfg, seed)
                except SceneRejected as e:
                    row["reason"] = type(e).__name__
                else:
                    if keep is not None and not keep(scene):
                        row["reason"] = "filtered_by_source_predicate"
                    else:
                        write_scene(class_dir, scene, arrays)
                        row.update(emitted=True, scene_id=scene["scene_id"])
                        emitted += 1
                index.write(json.dumps(row) + "\n")
                index.flush()
            cls_manifest["sources"][name] = {
                "config": list(loader), "residues": residues, "target": target,
                "emitted": emitted, "attempts": attempts,
                "seconds": round(time.time() - t0, 1)}
            status = "OK" if emitted >= target else "SHORT"
            if not quiet:
                print(f"[{jt}/{name}] {emitted}/{target} in {attempts} attempts "
                      f"({time.time() - t0:.0f}s) {status}", flush=True)
            if emitted < target:
                print(f"WARNING: {jt}/{name} short: {emitted}/{target}", flush=True)
        index.close()
        manifest["classes"][jt] = cls_manifest
    (out_root / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "out" / "bench6b"))
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    out_root = Path(args.out)
    if out_root.exists() and any(out_root.iterdir()):
        print(f"{out_root} exists and is not empty - refusing to mix corpora")
        return 1
    build(out_root, args.quiet)
    print("done:", out_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
