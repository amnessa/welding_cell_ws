#!/usr/bin/env python3
"""`self-footprint` over the tier-1 analytic benchmark (`out/bench_phase4`), every stratum.

The method of `Self_ideas/test1_ismail_hoca/weld_seam.py` through `baselines.self_footprint`,
scored by the Phase 4 harness exactly like the coverage chunks of `run_phase4_batch.py`:
L0 flag (ignored - the method consumes no oracle), clean clouds, `full_exterior` and
`single`, tolerance 3 mm. Two arms of its one process parameter: `gap="wps"` (the scene's
root gap, the method's declared input - headline) and `gap="zero"` (the CLI default).

Tier-1 only: no rendered data is read. Deterministic (zero seed spread is verified on one
scene per stratum in notebook 16), so one seed per cell.

    python scripts/run_self_footprint.py --smoke          # one scene per stratum
    python scripts/run_self_footprint.py --workers 8      # all 720 scenes, resumable

Per-scene results are written to `<out>/scenes/<scene_id>.csv` and concatenated into
`<out>/self_footprint[_smoke].csv.gz`; a rerun skips scenes that already have a file.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

VIEWS = ("full_exterior", "single")
GAPS = ("wps", "zero")


def strata(facts):
    fam = [r.seam_family if isinstance(r.seam_family, str)
           else ("line_grooved" if isinstance(r.prep, str) and r.prep != "square" else "line")
           for r in facts.itertuples()]
    return facts.joint_type + "/" + fam


def one_scene(scene_dir: str, out_csv: str) -> tuple[str, float, int]:
    warnings.filterwarnings("ignore")
    import pandas as pd
    from baselines import prepare, run_matrix
    from baselines import self_footprint as sf
    t0 = time.time()
    prep = prepare([scene_dir])
    rows = []
    for view in VIEWS:
        for gap in GAPS:
            df = run_matrix(prep, methods=[sf.spec()], seeds=[0], verify_seeds=1,
                            oracle=True, view=view, noise_scale=0.0, tol_mm=3.0,
                            method_kw={"self-footprint": {"gap": gap}})
            df["gap"], df["condition"] = gap, view
            rows.append(df)
    out = pd.concat(rows) if rows else pd.DataFrame()
    tmp = out_csv + ".tmp"
    out.to_csv(tmp, index=False)
    os.replace(tmp, out_csv)
    return Path(scene_dir).name, time.time() - t0, len(out)


def main():
    import pandas as pd
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--corpus", default=str(ROOT / "out" / "bench_phase4"))
    ap.add_argument("--out", default=str(ROOT / "out" / "self_footprint"))
    ap.add_argument("--smoke", action="store_true", help="first scene of every stratum only")
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()

    facts = pd.read_csv(Path(a.corpus) / "facts.csv")
    facts["stratum"] = strata(facts)
    if a.smoke:
        facts = facts.groupby("stratum", sort=True).head(1)
    sdir = Path(a.out) / "scenes"
    sdir.mkdir(parents=True, exist_ok=True)
    todo = [(str(Path(a.corpus) / r.joint_type / r.scene_id), str(sdir / f"{r.scene_id}.csv"))
            for r in facts.itertuples() if not (sdir / f"{r.scene_id}.csv").exists()]
    print(f"{len(facts)} scenes, {len(todo)} to run, {a.workers} workers", flush=True)

    os.environ.setdefault("OMP_NUM_THREADS", "1")
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        futs = [ex.submit(one_scene, d, o) for d, o in todo]
        for i, f in enumerate(as_completed(futs), 1):
            sid, sec, n = f.result()
            if i % 20 == 0 or i == len(futs):
                print(f"  {i}/{len(futs)}  last {sid} {sec:.0f}s  elapsed {time.time() - t0:.0f}s", flush=True)

    df = pd.concat([pd.read_csv(sdir / f"{s}.csv") for s in facts.scene_id], ignore_index=True)
    df = df.merge(facts[["scene_id", "stratum"]], on="scene_id", how="left")
    dst = Path(a.out) / ("self_footprint_smoke.csv.gz" if a.smoke else "self_footprint.csv.gz")
    df.to_csv(dst, index=False)
    print(f"{len(df)} rows -> {dst}")
    print(df.groupby(["condition", "gap", "stratum"]).f1.median().unstack(["condition", "gap"]).round(2).to_string())


if __name__ == "__main__":
    main()
