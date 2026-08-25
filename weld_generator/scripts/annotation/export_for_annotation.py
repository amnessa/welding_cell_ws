#!/usr/bin/env python
"""Export scenes for hand annotation in CloudCompare — Phase 5, the label-noise floor.

The claim Phase 5 turns into a number: *existing datasets have estimated labels; ours are
exact*. LWSNet reports sub-millimetre accuracy against labels a human drew in CloudCompare,
and nobody has ever checked those labels against anything. Here the annotator draws in the
SAME tool the literature used — that is the point, not a convenience — and the analytic
truth scores the drawing afterwards.

What this script does
---------------------
* Picks a stratified, seeded sample of tier-1 scenes (default 4 per joint type = 20).
* Exports each cloud as an ASCII PLY under an **anonymous id** (`scene_07.ply`), points
  only, `full_exterior` condition — the annotator sees what a perfect scan sees, exactly
  the input Task 1 defines. **No truth of any kind ships in the export directory.**
* Writes `manifest.json` OUTSIDE the export directory (it maps anonymous ids back to scene
  paths and carries the joint type per scene). Give the annotator the export directory
  only; keep the manifest.
* Writes `BRIEFING.md` into the export directory — the fifteen-minute brief, with the
  per-scene joint type table (the advisor protocol: telling the annotator the joint type
  constrains SELECTION and isolates LOCALIZATION, the number Phase 5 most wants).

Contamination rule, encoded here because it decides what the numbers mean
-------------------------------------------------------------------------
The generator's author knows where the seams are; their clicks measure self-reproduction,
not annotation. So `score_annotations.py` distinguishes annotator roles:

    demo        the author's own pass - used to brief others, EXCLUDED from every headline
    briefed     the measurement (independent, 15-min brief, joint type given per scene)
    unbriefed   optional second arm: no brief, no joint types - the difference between
                briefed and unbriefed is the EXPERTISE term of the error

Annotator instructions (also in BRIEFING.md)
--------------------------------------------
CloudCompare, per scene:
  1. Open `scene_NN.ply`.
  2. `Tools > Point picking > Point list picking`.
  3. Click points ALONG one weld seam, in order, end to end (4-10 clicks; more where it
     curves). Save the list (disk icon) as `scene_NN_seam0.txt`.
  4. Clear the list, repeat for the next seam on the same scene (`_seam1`, `_seam2`, ...).
  5. A scene may have one or several seams; annotate every seam you believe is to be
     welded. That judgement is part of what is measured.
Files go in one folder per annotator: `annotations/<name>/scene_NN_seamK.txt`.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

BRIEFING = """\
# Weld seam annotation — 15-minute brief

Thank you! You will mark weld seams on {n} point-cloud scenes. About 3 minutes each.

## What a weld seam is, per joint type (reference figures: see `reference_figures/`)

* **T** — a plate standing on another: the seam runs along BOTH sides of the standing
  plate's foot (two seams).
* **corner** — two plates meeting in an L: the seam runs along the corner line (outside
  and/or inside — mark where you would weld).
* **butt** — two plates edge to edge in one plane: the seam is the line where they meet
  (top face; the underside counts as a second seam if you would weld it).
* **lap** — one plate overlapping another: the seam runs along each plate's overlapping
  EDGE where it lies on the other plate (two seams, on opposite sides of the overlap).
* **edge** — two plates stacked flush: the seam runs along the flush edge.

## How to click (CloudCompare)

1. Open the scene's `.ply`.
2. `Tools > Point picking > Point list picking`.
3. Click 4-10 points ALONG one seam, IN ORDER, end to end. Precision matters more than
   point count; add clicks where the line bends.
4. Save the list as `scene_NN_seam0.txt` (disk icon in the picking toolbar).
5. Clear, repeat for further seams on the same scene: `_seam1`, `_seam2`, ...
6. Mark every seam you believe is to be welded - that judgement is part of the study.

## Joint type per scene

{table}

Do not discuss scenes with other annotators until everyone is done.
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "out" / "annotation" / "export"))
    ap.add_argument("--manifest", default=str(ROOT / "out" / "annotation" / "manifest.json"))
    ap.add_argument("--per-type", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--corpus", default=str(ROOT / "out" / "bench"))
    ap.add_argument("--with-joint-types", action="store_true", default=True,
                    help="write the joint-type table into the briefing (briefed arm)")
    ap.add_argument("--unbriefed", action="store_true",
                    help="omit the joint-type table and the per-type reference text "
                         "(the unbriefed arm's export)")
    args = ap.parse_args()

    from baselines import balanced_corpus, cloud_for, ground_truth, load_scene

    corpus = balanced_corpus(args.corpus, per_type=50)
    rng = np.random.default_rng(args.seed)
    picked = []
    for jt in ("T", "corner", "butt", "lap", "edge"):
        dirs = list(corpus[jt])
        for i in rng.choice(len(dirs), size=args.per_type, replace=False):
            picked.append((jt, dirs[int(i)]))
    order = rng.permutation(len(picked))               # anonymise the ordering too

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    manifest = {}
    rows = []
    for k, oi in enumerate(order):
        jt, d = picked[int(oi)]
        scene, arrays = load_scene(d)
        gt = ground_truth(scene, arrays, primary_only=True)
        if not gt:
            continue
        c = cloud_for(scene, arrays, view="full_exterior")
        anon = f"scene_{k:02d}"
        _write_ply(out / f"{anon}.ply", c["xyz"])
        manifest[anon] = {"path": str(d), "joint_type": jt, "n_primary": len(gt)}
        rows.append(f"| {anon} | {jt} |")

    table = ("(not disclosed in this arm)" if args.unbriefed
             else "| scene | joint type |\n|---|---|\n" + "\n".join(rows))
    (out / "BRIEFING.md").write_text(BRIEFING.format(n=len(manifest), table=table))

    mpath = Path(args.manifest)
    mpath.parent.mkdir(parents=True, exist_ok=True)
    mpath.write_text(json.dumps(manifest, indent=2, sort_keys=True))
    print(f"{len(manifest)} scenes -> {out}")
    print(f"manifest (KEEP PRIVATE, not for the annotator) -> {mpath}")
    print("hand the annotator the export directory ONLY")


def _write_ply(path: Path, xyz: np.ndarray) -> None:
    """ASCII PLY, points only. No colour, no normals, no truth of any kind."""
    xyz = np.asarray(xyz, dtype=float)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n"
                f"element vertex {len(xyz)}\n"
                "property float x\nproperty float y\nproperty float z\n"
                "end_header\n")
        np.savetxt(f, xyz, fmt="%.3f")


if __name__ == "__main__":
    main()
