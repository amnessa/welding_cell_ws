#!/usr/bin/env python3
"""`self-seamfind` over the tier-1 analytic benchmark (`out/bench_phase4`), every stratum.

The seam finder of `Self_ideas/test2_cagdas/seam_finder.md` through
`baselines.self_seamfind`, scored like the coverage chunks of `run_phase4_batch.py`
(L0 = part labels from `object_id`, clean clouds, `full_exterior` and `single`, 3 mm).
Arms, one seed each (deterministic):

    default    the corpus-fitted pipeline (seamfind/config.py defaults)
    nocrease   supporting planes from the walker's MLS fit only (no crease-free refit)
    plan       the plan as written where it differs: mutual NN, theta_r agreement test,
               tau_vis = 0.3, no crease-free refit

Tier-1 only: no rendered data is read. Resumable: one CSV per scene under
`<out>/scenes/`, concatenated into `<out>/self_seamfind[_smoke].csv.gz`.
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

from run_self_footprint import strata  # noqa: E402

VIEWS = ("full_exterior", "single")
ARMS = {"default": {},
        "nocrease": {"crease_free": False},
        "plan": {"seeding": "dense", "angle_check": True, "tau_vis": 0.3, "crease_free": False}}


def one_scene(scene_dir: str, out_csv: str) -> tuple[str, float, int]:
    warnings.filterwarnings("ignore")
    import pandas as pd
    from baselines import prepare, run_matrix
    from baselines import self_seamfind as sk
    t0 = time.time()
    prep = prepare([scene_dir])
    rows = []
    for view in VIEWS:
        for arm, kw in ARMS.items():
            try:
                df = run_matrix(prep, methods=[sk.spec()], seeds=[0], verify_seeds=1, oracle=True,
                                view=view, noise_scale=0.0, tol_mm=3.0,
                                method_kw={"self-seamfind": kw})
                df["failure"] = ""
            except Exception as e:              # a crash is a row, never a lost scene
                df = pd.DataFrame([{"method": "self-seamfind", "scene_id": prep[0].facts["scene_id"] if prep else Path(scene_dir).name,
                                    "f1": 0.0, "precision": 0.0, "recall": 0.0,
                                    "failure": f"{type(e).__name__}: {e}"[:120]}])
            df["arm"], df["condition"] = arm, view
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
    ap.add_argument("--out", default=str(ROOT / "out" / "self_seamfind"))
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
    df = df.drop(columns=["stratum"], errors="ignore").merge(facts[["scene_id", "stratum"]], on="scene_id", how="left")
    dst = Path(a.out) / ("self_seamfind_smoke.csv.gz" if a.smoke else "self_seamfind.csv.gz")
    df.to_csv(dst, index=False)
    print(f"{len(df)} rows -> {dst}")
    print(df.groupby(["condition", "arm", "stratum"]).f1.median().unstack(["condition", "arm"]).round(2).to_string())


if __name__ == "__main__":
    main()
