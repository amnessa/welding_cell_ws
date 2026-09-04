#!/usr/bin/env python
"""The Phase 4 batch job — every method, the whole balanced corpus, one dataframe.

This is the "remaining Phase 4 compute job" named in `dataset_plan.md`: after it runs,
every one of the nine first-results plots, the method coverage table, the repeat box
plots, the noise table, the fixture ablation AND the Task-2 short section (MPS selection
+ localization, the `task2` group) are `groupby`s over its output. Nothing in here
computes a new quantity — it is `harness.run_matrix` / `harness.run_task2` over an
explicit chunk list. Six literature methods; `ours` (radius-PCA) is excluded by the
2026-09-03 ruling (see METHODS_ALL), and the default corpus is the per-family
`out/bench_phase4` (60 scenes per family, classes unequal by design — report per
family/class, never pooled).

Usage
-----
    python scripts/run_phase4_batch.py --list            # show chunks + runtime estimate
    python scripts/run_phase4_batch.py                   # run everything (resumable)
    python scripts/run_phase4_batch.py --only coverage   # one chunk group
    python scripts/run_phase4_batch.py --per-type 5      # small dry run first
    nohup python scripts/run_phase4_batch.py > batch.log 2>&1 &   # the overnight form

Design decisions, so the numbers are interpretable later
--------------------------------------------------------
* **Resumable by construction.** One csv.gz per chunk in `--out`; a chunk whose file
  exists is skipped, so a crash or Ctrl+C costs only the chunk in flight. Delete a file to
  recompute it. `phase4_batch.csv.gz` is the concatenation, rewritten at the end of every
  run. (csv.gz, not parquet: this venv has no parquet engine and D9 says the core adds no
  optional native deps for a job pandas does natively.)
* **Memory-safe by streaming.** Scenes are prepared one at a time and freed (the pattern
  that runs the 250-scene corpus at ~0,5 GB; holding the corpus in memory needs ~19 GB and
  was the first thing to kill this job's notebook ancestor).
* **Each method gets its own paper's coarse stage at L0** — that is `run_matrix`'s
  contract, unchanged here. The L1 arm withholds it. `lit-modelreg` ignores the flag
  (constitutively L0-with-CAD) and its `target_features`/`init` arms are separate chunks.
* **Seeds follow the registry.** `lit-ransac` is the one randomised method and gets
  `--seeds` (default 30) everywhere it appears; the deterministic six get
  `verify_seeds = 2`, which keeps the measured-zero-spread guarantee at a quarter of the
  cost of 3+ repeats.
* **The noise axis is a multiplier on the derived sigma_z** (0 = clean, 1 = the stored
  profile, 2 = twice it), and the noise chunks run on the SINGLE view: sensor noise on a
  cloud no sensor returns would be a condition without an interpretation.
* **The fixture chunks run every method on the twin pairs** (`out/bench_fx`, matched on
  `twin_key`). The plan's specific prediction is a phantom plane-pair candidate from
  `lit-ransac` and `lit-ppf` along every part-fixture contact; running all seven prices
  the fixture for the whole table at ~50 extra scenes per arm.

Output columns are `run_matrix`'s (metrics + facts + aux) plus `chunk`, `condition`,
`fixture`. All lengths millimetres, as everywhere in this repo.
"""

from __future__ import annotations

import argparse
import gc
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

import pandas as pd  # noqa: E402

from baselines import balanced_corpus, prepare, run_matrix  # noqa: E402
from baselines.dataset import scene_dirs, scene_facts  # noqa: E402
from baselines.harness import run_task2  # noqa: E402

# `ours` (radius-PCA) EXCLUDED by ruling 2026-09-03: it brings no mechanism the lit
# methods lack and its coplanar-seam limit is already measured and reported
# (dataset_plan.md §10) - the run is a pure literature comparison, and the project's
# own contribution is the improvement built on whichever method wins it. Re-add with
# --include-ours if the floor-anchor appendix figure is wanted.
METHODS_ALL = ["lit-ransac", "lit-regiongrow", "lit-lobb", "lit-ppf",
               "lit-pcaslice", "lit-modelreg"]

# `lit-modelreg` rebuilds its CAD model from scene.json, and its samplers cover slab
# and prism primitives only (its papers register plate assemblies) - so its chunks run
# on the plate-primitive subset and the coverage restriction is printed, not hidden.
PLATE_PRIMS = {"slab", "prism"}

# measured seconds per scene per run (L0, clean, bench hardware) - for --list only
EST_SEC = {"ours": 1.5, "lit-ransac": 0.3, "lit-regiongrow": 1.0, "lit-lobb": 2.5,
           "lit-ppf": 0.6, "lit-pcaslice": 0.5, "lit-modelreg": 2.5}


def chunks(seeds_ransac: int):
    """The explicit chunk list. Comment out what you do not want; order is cheap-first."""
    out = []
    for cond in ("full_exterior", "single"):
        for m in METHODS_ALL:
            out.append(dict(group="coverage", name=f"coverage_{m}_{cond}",
                            methods=[m], view=cond, oracle=True, noise=0.0,
                            seeds=seeds_ransac if m == "lit-ransac" else 2))
    # the L1 arms (modelreg has none - the model IS its oracle)
    for m in [m for m in METHODS_ALL if m != "lit-modelreg"]:
        out.append(dict(group="l1", name=f"l1_{m}_full_exterior",
                        methods=[m], view="full_exterior", oracle=False, noise=0.0,
                        seeds=seeds_ransac if m == "lit-ransac" else 2))
    # the noise table: single view, sigma x {1, 2} (0 is the coverage chunk above)
    for ns in (1.0, 2.0):
        for m in METHODS_ALL:
            out.append(dict(group="noise", name=f"noise{ns:g}_{m}_single",
                            methods=[m], view="single", oracle=True, noise=ns,
                            seeds=seeds_ransac if m == "lit-ransac" else 2))
    # ladder extras: lit-ppf exact normals; lit-modelreg dense features + global init
    out.append(dict(group="ladder", name="ppf_exact_normals_full_exterior",
                    methods=["lit-ppf"], view="full_exterior", oracle=True, noise=0.0,
                    seeds=2, method_kw={"lit-ppf": {"normals": "exact"}}))
    out.append(dict(group="ladder", name="modelreg_dense_features",
                    methods=["lit-modelreg"], view="full_exterior", oracle=True,
                    noise=0.0, seeds=2,
                    method_kw={"lit-modelreg": {"target_features": "dense"}}))
    out.append(dict(group="ladder", name="modelreg_global_init",
                    methods=["lit-modelreg"], view="full_exterior", oracle=True,
                    noise=0.0, seeds=2, method_kw={"lit-modelreg": {"init": "global"}}))
    # the Task-2 chunks (6c(c)): MPS selection + localization, single view, clean.
    # Truth is the WELDABLE seam set (primary_only=False) because mps_rule-0.1 argmaxes
    # over weldable, not primary; D39's endpoint rule is inside run_task2.
    for m in METHODS_ALL:
        out.append(dict(group="task2", name=f"task2_{m}_single",
                        methods=[m], view="single", oracle=True, noise=0.0,
                        seeds=seeds_ransac if m == "lit-ransac" else 2, task2=True))
    # fixture twins: every method, both arms, paired seeds
    for m in METHODS_ALL:
        out.append(dict(group="fixture", name=f"fixture_{m}",
                        methods=[m], view="full_exterior", oracle=True, noise=0.0,
                        seeds=seeds_ransac if m == "lit-ransac" else 2, fixture=True))
    return out


def stream(dirs, spec, per_type_cap=None):
    """One chunk over its scene list, one scene in memory at a time."""
    rows = []
    t0 = time.time()
    for k, d in enumerate(dirs):
        prep = prepare([d], primary_only=not spec.get("task2"))
        if prep:
            if spec.get("task2"):
                df = run_task2(prep, methods=spec["methods"],
                               seeds=range(spec["seeds"]), verify_seeds=spec["seeds"],
                               noise_scale=spec["noise"],
                               method_kw=spec.get("method_kw", {}))
            else:
                df = run_matrix(prep, methods=spec["methods"],
                                seeds=range(spec["seeds"]), verify_seeds=spec["seeds"],
                                oracle=spec["oracle"], view=spec["view"],
                                noise_scale=spec["noise"],
                                method_kw=spec.get("method_kw", {}))
            rows.append(df)
        del prep
        gc.collect()
        if (k + 1) % 25 == 0:
            print(f"    {k + 1}/{len(dirs)} scenes  {time.time() - t0:.0f}s", flush=True)
    return pd.concat(rows, ignore_index=True) if rows else pd.DataFrame()


def fixture_pairs(bench_dirs, facts, per_type, corpus_root):
    fx_root = corpus_root.parent / (corpus_root.name + "_fx")
    fx_dirs = [d for jt in ("T", "corner", "butt", "lap", "edge")
               for d in scene_dirs(fx_root / jt)]
    fx_facts = [scene_facts(json.loads((d / "scene.json").read_text())) for d in fx_dirs]
    by_key = {f["twin_key"]: d for f, d in zip(fx_facts, fx_dirs)}
    off, on, per = [], [], {}
    for f, d in zip(facts, bench_dirs):
        if f["twin_key"] in by_key and per.get(f["joint_type"], 0) < per_type:
            off.append(d)
            on.append(by_key[f["twin_key"]])
            per[f["joint_type"]] = per.get(f["joint_type"], 0) + 1
    return off, on


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "out" / "phase4_batch"))
    ap.add_argument("--per-type", type=int, default=0,
                    help="0 (default) = ALL emitted scenes per class - the per-family "
                         "corpus is homogeneous per FAMILY, so classes differ in size "
                         "by design and a per-class trim would undo that; N > 0 = "
                         "balanced_corpus's lowest-N-seeds trim (bench/bench6b runs)")
    ap.add_argument("--seeds", type=int, default=30, help="seeds for lit-ransac")
    ap.add_argument("--only", default=None,
                    help="run only chunks whose group or name contains this")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--corpus", default=str(ROOT / "out" / "bench_phase4"),
                    help="benchmark corpus root (per-joint-type subdirs); the fixture "
                         "chunks read its sibling '<corpus>_fx' for the twins")
    args = ap.parse_args()

    corpus_root = Path(args.corpus)
    if args.per_type:
        corpus = balanced_corpus(corpus_root, per_type=args.per_type)
        dirs = [d for jt in ("T", "corner", "butt", "lap", "edge")
                for d in corpus[jt][: args.per_type]]
    else:
        dirs = [d for jt in ("T", "corner", "butt", "lap", "edge")
                for d in scene_dirs(corpus_root / jt)]
    facts = [scene_facts(json.loads((d / "scene.json").read_text())) for d in dirs]
    plate_dirs = [d for d, f in zip(dirs, facts)
                  if set(f["primitives"].split(",")) <= PLATE_PRIMS]

    plan = chunks(args.seeds)
    if args.only:
        plan = [c for c in plan
                if args.only in c["group"] or args.only in c["name"]]

    if args.list:
        total = 0.0
        for c in plan:
            if c.get("fixture"):
                n = min(8, args.per_type or 8) * 5
            elif "lit-modelreg" in c["methods"]:
                n = len(plate_dirs)
            else:
                n = len(dirs)
            reps = c["seeds"] if c["methods"] == ["lit-ransac"] else min(c["seeds"], 2)
            est = n * reps * EST_SEC[c["methods"][0]] * (2 if c.get("fixture") else 1)
            total += est
            print(f"{c['group']:9s} {c['name']:38s} ~{est / 60:6.1f} min")
        print(f"\ntotal ~{total / 3600:.1f} h on this hardware "
              f"({len(plan)} chunks, resumable per chunk; "
              f"{len(dirs)} scenes, {len(plate_dirs)} plate-primitive)")
        return

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    for c in plan:
        f = outdir / f"{c['name']}.csv.gz"
        if f.exists():
            print(f"[skip] {c['name']} (exists)", flush=True)
            continue
        print(f"[run ] {c['name']}", flush=True)
        tc = time.time()
        if c.get("fixture"):
            fx_root = corpus_root.parent / (corpus_root.name + "_fx")
            if not fx_root.exists():
                print(f"[skip] {c['name']} (no fixture-twin corpus at {fx_root} - "
                      f"the fixture table stays sourced from a corpus that has one)",
                      flush=True)
                continue
            off, on = fixture_pairs(dirs, facts, per_type=min(8, args.per_type or 8),
                                    corpus_root=corpus_root)
            a = stream(off, c)
            b = stream(on, c)
            a["fixture"], b["fixture"] = False, True
            df = pd.concat([a, b], ignore_index=True)
        else:
            cdirs = plate_dirs if "lit-modelreg" in c["methods"] else dirs
            if cdirs is plate_dirs:
                print(f"    lit-modelreg scope: {len(plate_dirs)}/{len(dirs)} scenes "
                      f"(slab/prism primitives only)", flush=True)
            df = stream(cdirs, c)
            df["fixture"] = False
        df["chunk"] = c["name"]
        df["condition"] = c["view"]
        df.to_csv(f, index=False)
        print(f"[done] {c['name']}  {len(df)} rows  {time.time() - tc:.0f}s", flush=True)

    parts = sorted(outdir.glob("*.csv.gz"))
    parts = [p for p in parts if p.name != "phase4_batch.csv.gz"]
    all_df = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True)
    all_df.to_csv(outdir / "phase4_batch.csv.gz", index=False)
    print(f"\n{len(all_df)} rows -> {outdir / 'phase4_batch.csv.gz'}  "
          f"({time.time() - t0:.0f}s this run)", flush=True)


if __name__ == "__main__":
    main()
