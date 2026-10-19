#!/usr/bin/env python3
"""Build the D12 fixture-twin corpus of a `make_bench6b.py` corpus: `<base>_fx`.

For every stratum of the base corpus, replay exactly the seeds the base EMITTED under the
same config with `fixture_present: true`. `twin_key` is computed over the geometry keys
only, so each twin has bit-identical workpiece geometry and seam truth and differs in
placement, camera and the fixture; `run_phase4_batch.py`'s fixture chunks pair the arms on
it. Seeds the fixture arm rejects (a fixture contact can close a seam, or occlude the last
visible one) are recorded as rejected, never backfilled - the pair count is a result.

All twelve strata since 2026-10-19 (user request). Until then only the six PLATE strata had
twins and the curved families kept the fixture off (patch_phase6b); `scene_curved` now
places the table under the lowest part - flat under a base plate or strip, along a line
under the saddle's main pipe (`contact_mode: "line"`, SCHEMA 2.4.0) - and keeps the pose
z-preserving (no tilt), which the offset curves of families 5 and 6 require.

Usage:
    python scripts/make_fixture_twins.py out/bench_phase4                 # all strata -> out/bench_phase4_fx
    python scripts/make_fixture_twins.py out/bench_phase4 --only curved --append
        # add the curved twins to an existing plate-only `_fx` corpus; plate twins untouched
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from make_bench6b import STRATA, _curved, _plate  # noqa: E402
from weldgen.scene import SceneRejected  # noqa: E402
from weldgen.writer import write_scene  # noqa: E402


def _replay(jt: str, name: str, loader: list, seeds: list[int], class_dir: str):
    """One source: replay its emitted seeds with the fixture on. Returns the index rows."""
    cfg, gen = _plate(loader[1]) if loader[0] == "plate" else _curved(loader[1])
    cfg["fixture_present"] = True
    keep = next(k for n, ld, *_r, k in STRATA[jt] if n == name)
    rows, t0 = [], time.time()
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
                write_scene(Path(class_dir), scene, arrays)
                row.update(emitted=True, scene_id=scene["scene_id"], twin_key=scene["twin_key"])
        rows.append(row)
    return jt, name, rows, round(time.time() - t0, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("base")
    ap.add_argument("--out", default=None)
    ap.add_argument("--only", choices=("all", "plate", "curved"), default="all",
                    help="which strata to replay (default all twelve)")
    ap.add_argument("--append", action="store_true",
                    help="add to an existing _fx corpus; sources already in its index are skipped")
    ap.add_argument("--workers", type=int, default=6, help="one source per worker")
    args = ap.parse_args()
    base = Path(args.base)
    out = Path(args.out) if args.out else base.parent / (base.name + "_fx")
    if out.exists() and any(out.iterdir()) and not args.append:
        sys.exit(f"{out} exists and is not empty - refusing to mix corpora (use --append)")
    man_path = out / "manifest.json"
    manifest = (json.loads(man_path.read_text()) if args.append and man_path.exists()
                else {"twin_of": str(base), "fixture_present": True, "classes": {}})

    jobs = []
    for jt, sources in STRATA.items():
        base_rows = [json.loads(ln) for ln in (base / jt / "index.jsonl").read_text().splitlines()]
        idx = out / jt / "index.jsonl"
        have = ({json.loads(ln)["source"] for ln in idx.read_text().splitlines()}
                if args.append and idx.exists() else set())
        (out / jt).mkdir(parents=True, exist_ok=True)
        for name, loader, *_ in sources:
            kind = loader[0]
            if args.only != "all" and kind != args.only:
                continue
            if name in have:
                print(f"[{jt}/{name}] already in {idx} - skipped")
                continue
            seeds = [r["seed"] for r in base_rows if r["source"] == name and r.get("emitted")]
            jobs.append((jt, name, list(loader), seeds, str(out / jt)))
    print(f"{len(jobs)} sources, {sum(len(j[3]) for j in jobs)} seeds, {args.workers} workers", flush=True)

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_replay, *j) for j in jobs]
        results = []
        for f in futs:                                    # submission order: deterministic index
            jt, name, rows, sec = f.result()
            results.append((jt, name, rows, sec))
            print(f"[{jt}/{name}] {sum(r['emitted'] for r in rows)}/{len(rows)} twins ({sec:.0f}s)", flush=True)

    loaders = {(jt, n): ld for jt, srcs in STRATA.items() for n, ld, *_ in srcs}
    for jt, name, rows, sec in results:
        with open(out / jt / "index.jsonl", "a") as index:
            for row in rows:
                index.write(json.dumps(row) + "\n")
        cls = manifest["classes"].setdefault(jt, {"sources": {}})
        cls["sources"][name] = {"config": list(loaders[(jt, name)]), "base_seeds": len(rows),
                                "emitted": sum(r["emitted"] for r in rows), "seconds": sec}
    manifest.setdefault("history", []).append(
        {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "only": args.only,
         "append": bool(args.append), "sources": [f"{jt}/{n}" for jt, n, *_ in results]})
    man_path.write_text(json.dumps(manifest, indent=2))
    print("done:", out)


if __name__ == "__main__":
    main()
