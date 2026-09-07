#!/usr/bin/env python3
"""Build the D12 fixture-twin corpus of a `make_bench6b.py` corpus: `<base>_fx`.

For every PLATE stratum of the base corpus (curved families keep the fixture off by
design, patch_phase6b), replay exactly the seeds the base EMITTED under the same config
with `fixture_present: true`. `twin_key` is computed over the geometry keys only, so
each twin has bit-identical workpiece geometry and seam truth and differs in the
fixture alone; `run_phase4_batch.py`'s fixture chunks pair the arms on it. Seeds the
fixture arm rejects (a fixture contact can close a seam, or occlude the last visible
one) are recorded as rejected, never backfilled - the pair count is a result.

Usage:
    python scripts/make_fixture_twins.py out/bench_phase4      # -> out/bench_phase4_fx
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from make_bench6b import STRATA, _plate  # noqa: E402
from weldgen.scene import SceneRejected  # noqa: E402
from weldgen.writer import write_scene  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    base = Path(args.base)
    out = Path(args.out) if args.out else base.parent / (base.name + "_fx")
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} exists and is not empty - refusing to mix corpora")
    manifest = {"twin_of": str(base), "fixture_present": True, "classes": {}}
    for jt, sources in STRATA.items():
        rows = [json.loads(ln)
                for ln in (base / jt / "index.jsonl").read_text().splitlines()]
        class_dir = out / jt
        class_dir.mkdir(parents=True, exist_ok=True)
        index = open(class_dir / "index.jsonl", "w")
        cls = {"sources": {}}
        for name, loader, residues, target, keep in sources:
            if loader[0] != "plate":
                continue                                  # curved: fixture off by design
            seeds = [r["seed"] for r in rows if r["source"] == name and r.get("emitted")]
            cfg, gen = _plate(loader[1])
            cfg["fixture_present"] = True
            emitted, t0 = 0, time.time()
            for seed in seeds:
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
                        row.update(emitted=True, scene_id=scene["scene_id"],
                                   twin_key=scene["twin_key"])
                        emitted += 1
                index.write(json.dumps(row) + "\n")
                index.flush()
            cls["sources"][name] = {"config": list(loader), "base_seeds": len(seeds),
                                    "emitted": emitted,
                                    "seconds": round(time.time() - t0, 1)}
            print(f"[{jt}/{name}] {emitted}/{len(seeds)} twins "
                  f"({time.time() - t0:.0f}s)", flush=True)
        index.close()
        manifest["classes"][jt] = cls
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print("done:", out)


if __name__ == "__main__":
    main()
