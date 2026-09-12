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

`--per-family N` overrides every source's target to N — the Phase 4 run corpus
(2026-09-03 ruling): homogeneous per FAMILY, not per class, so T becomes the largest
class (6 families) by design. Residues, stride and BASE_SEED are unchanged, so the
per-family corpus is a strict superset of `bench6b` within every stratum and every
seed-sorted prefix keeps the intended family proportions.

Usage:
    python scripts/make_bench6b.py [--out out/bench6b] [--quiet]
    python scripts/make_bench6b.py --per-family 60 --out out/bench_phase4
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


def _seeds(residues, stride, base_seed: int = BASE_SEED):
    i = 0
    while True:
        for r in residues:
            yield base_seed + r + stride * i
        i += 1


def build(out_root: Path, quiet: bool, per_family: int | None = None, base_seed: int = BASE_SEED,
          classes: list[str] | None = None, sources: list[str] | None = None,
          index_name: str = "index.jsonl") -> dict:
    """`classes` / `sources` restrict the run (Phase 8 M6: the 3600-scene `train_v1` corpus is
    built as one process per T source, each writing its own `index_<source>.jsonl`, merged by
    `merge_indexes` afterwards). The stride and residues are those of the FULL class, so a
    filtered run walks exactly the seeds the unfiltered run would."""
    manifest = {"base_seed": base_seed, "per_family": per_family, "classes": {}}
    for jt, all_sources in STRATA.items():
        if classes and jt not in classes:
            continue
        stride = sum(len(res) for _, _, res, _, _ in all_sources)
        class_dir = out_root / jt
        class_dir.mkdir(parents=True, exist_ok=True)
        index = open(class_dir / index_name, "w")
        cls_manifest = {"stride": stride, "sources": {}}
        for name, loader, residues, target, keep in all_sources:
            if sources and name not in sources:
                continue
            if per_family is not None:
                target = per_family
            cfg, gen = (_plate(loader[1]) if loader[0] == "plate"
                        else _curved(loader[1]))
            emitted, attempts, t0 = 0, 0, time.time()
            for seed in _seeds(residues, stride, base_seed):
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
    if index_name == "index.jsonl":
        (out_root / "manifest.json").write_text(json.dumps(manifest, indent=2))
    else:   # one part per (class, source) - three plate classes all have a source called "line"
        tag = "_".join(list(manifest["classes"]) + [index_name[len("index_"):-len(".jsonl")]])
        (out_root / f"manifest_{tag}.json").write_text(json.dumps(manifest, indent=2))
    return manifest


def merge_indexes(out_root: Path) -> None:
    """Merge per-source `index_<source>.jsonl` / `manifest_<source>.json` parts into the
    standard `index.jsonl` (seed-sorted) and `manifest.json`."""
    manifest = {"base_seed": None, "per_family": None, "classes": {}}
    for mpart in sorted(out_root.glob("manifest_*.json")):
        m = json.loads(mpart.read_text())
        manifest["base_seed"], manifest["per_family"] = m["base_seed"], m["per_family"]
        for jt, cm in m["classes"].items():
            manifest["classes"].setdefault(jt, {"stride": cm["stride"], "sources": {}})["sources"].update(cm["sources"])
    for class_dir in sorted(p for p in out_root.iterdir() if p.is_dir()):
        parts = sorted(class_dir.glob("index_*.jsonl"))
        if not parts:
            continue
        rows = [json.loads(l) for f in parts for l in open(f) if l.strip()]
        # a class whose manifest part was lost (pre-fix naming clash) is reconstructed from its index
        jt = class_dir.name
        cm = manifest["classes"].setdefault(jt, {"stride": sum(len(r) for _, _, r, _, _ in STRATA[jt]), "sources": {}})
        for name, loader, residues, target, _ in STRATA[jt]:
            srows = [r for r in rows if r["source"] == name]
            if srows and name not in cm["sources"]:
                cm["sources"][name] = {"config": list(loader), "residues": residues,
                                       "target": manifest["per_family"] or target,
                                       "emitted": sum(1 for r in srows if r.get("emitted")), "attempts": len(srows),
                                       "seconds": None, "reconstructed_from_index": True}
        rows.sort(key=lambda r: r["seed"])
        with open(class_dir / "index.jsonl", "w") as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")
    (out_root / "manifest.json").write_text(json.dumps(manifest, indent=2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "out" / "bench6b"))
    ap.add_argument("--per-family", type=int, default=None,
                    help="override every source's target to N (Phase 4 run corpus)")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--base-seed", type=int, default=BASE_SEED,
                    help="a DIFFERENT base seed keeps a training corpus disjoint from the benchmark (M6: 3000000)")
    ap.add_argument("--classes", default=None, help="comma list of joint types to build (default all)")
    ap.add_argument("--sources", default=None, help="comma list of source names to build (default all); writes index_<name>.jsonl")
    ap.add_argument("--merge", action="store_true", help="merge per-source index/manifest parts in --out and exit")
    args = ap.parse_args()
    out_root = Path(args.out)
    if args.merge:
        merge_indexes(out_root); print("merged:", out_root); return 0
    classes = args.classes.split(",") if args.classes else None
    sources = args.sources.split(",") if args.sources else None
    index_name = "index.jsonl" if not sources else f"index_{'_'.join(sources)}.jsonl"
    if out_root.exists() and any(out_root.iterdir()) and not (classes or sources):
        print(f"{out_root} exists and is not empty - refusing to mix corpora")
        return 1
    build(out_root, args.quiet, per_family=args.per_family, base_seed=args.base_seed,
          classes=classes, sources=sources, index_name=index_name)
    print("done:", out_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
