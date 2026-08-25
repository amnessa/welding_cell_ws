#!/usr/bin/env python
"""Score hand annotations against analytic truth — the Phase 5 numbers.

Reads `annotations/<annotator>/scene_NN_seamK.txt` (CloudCompare point-list exports) plus
the private `manifest.json`, and produces the three quantities Phase 5 exists for:

1. **The label-noise floor**: localization error of hand labels against exact truth —
   lateral RMSE / p95, per annotator, per joint type, and against all three D19 curves
   (a human clicks the visual crease, which may be the root or the gap-mid line rather
   than `nominal`; the curve that fits best is itself a finding).
2. **Selection vs localization, separated** (the advisor's sharp version): selection error
   is which seams were annotated at all — missed primaries, extra non-seams, off-class
   picks — and needs welding knowledge; localization is how precisely a correctly-selected
   line was placed, and does not. `matched_path_errors`' one-to-one matching draws the
   line between them, and `lateral_*` (distance to the supporting line) separates
   placement from endpoint judgement.
3. **The perturbation model** for the Phase 5 training experiment: per-annotator lateral
   sigma and endpoint sigma, written to `annotator_model.json`, so "train on exact labels
   vs labels perturbed by the measured annotator distribution" has a measured distribution
   to draw from.

Annotator roles (`--role`, recorded per annotator directory in the output):

    demo       the generator's author demonstrating the task - contaminated by
               construction (they know where the seams are) and EXCLUDED from every
               headline aggregate; kept in the output for the briefing record only
    briefed    the measurement: independent, 15-min brief, joint types given
    unbriefed  optional arm: no brief - (unbriefed - briefed) is the expertise term

Usage:
    python scripts/annotation/score_annotations.py \\
        --annotations out/annotation/annotations/anil --role briefed \\
        --annotations out/annotation/annotations/me   --role demo
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))


def parse_pointlist(path: Path) -> np.ndarray:
    """A CloudCompare point-list export: one point per line, index-prefixed or not.

    CloudCompare writes `index,x,y,z` (versions differ on the separator and on whether
    the index column exists), so: take the last three numeric columns of every line that
    has at least three.
    """
    pts = []
    for line in path.read_text().splitlines():
        nums = re.findall(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?", line)
        if len(nums) >= 3:
            pts.append([float(v) for v in nums[-3:]])
    return np.asarray(pts, dtype=float)


def load_annotations(folder: Path) -> dict[str, list[np.ndarray]]:
    out: dict[str, list[np.ndarray]] = {}
    for f in sorted(folder.glob("scene_*_seam*.txt")):
        m = re.match(r"(scene_\d+)_seam(\d+)", f.stem)
        if not m:
            continue
        poly = parse_pointlist(f)
        if len(poly) >= 2:
            out.setdefault(m.group(1), []).append(poly)
    return out


def score_annotator(folder: Path, manifest: dict, role: str,
                    match_tol_mm: float = 15.0) -> dict:
    from baselines import ground_truth, load_scene
    from baselines.metrics import matched_path_errors

    ann = load_annotations(folder)
    rows = []
    for anon, seams in sorted(ann.items()):
        if anon not in manifest:
            print(f"  [warn] {anon} not in manifest, skipped")
            continue
        scene, arrays = load_scene(manifest[anon]["path"])
        per_curve = {}
        for curve in ("nominal", "root", "gapmid"):
            gt = ground_truth(scene, arrays, curve=curve, primary_only=True)
            per_curve[curve] = [e for e in (matched_path_errors(seams, gt) if gt else [])
                                if e["matched"] and e["rmse"] <= match_tol_mm
                                or not e["matched"]]
        gt_n = ground_truth(scene, arrays, primary_only=True)
        errs = per_curve["nominal"]
        # A match only counts within `match_tol_mm`. The harness's one-to-one matching is
        # deliberately uncapped (a method's bad answer should be scored, not dropped), but
        # for SELECTION scoring an annotation 60 mm from any seam is a phantom, not a
        # selection - the fake-annotator test planted exactly that and the uncapped
        # matcher absorbed it, zeroing both the miss and the extra it should have counted.
        matched = [e for e in errs if e["matched"]]
        rows.append({
            "scene": anon, "joint_type": manifest[anon]["joint_type"],
            "n_gt": len(gt_n), "n_ann": len(seams),
            # SELECTION: what was picked at all
            "missed": len(gt_n) - len(matched),
            "extra": len(seams) - len(matched),
            # LOCALIZATION: how well the correctly-picked lines were placed
            "lat_rmse": float(np.median([e["lateral_rmse"] for e in matched]))
            if matched else np.nan,
            "lat_me": float(np.median([e["lateral_me"] for e in matched]))
            if matched else np.nan,
            "end_err": float(np.median([e["end_error_mm"] for e in matched]))
            if matched else np.nan,
            # which D19 curve the human actually clicked
            **{f"rmse_{c}": float(np.median([e["rmse"] for e in per_curve[c]
                                             if e["matched"]]))
               if any(e["matched"] for e in per_curve[c]) else np.nan
               for c in ("nominal", "root", "gapmid")},
            "match_tol_mm": match_tol_mm,
        })
    import pandas as pd
    df = pd.DataFrame(rows)
    df["annotator"] = folder.name
    df["role"] = role
    return {"df": df}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default=str(ROOT / "out" / "annotation" / "manifest.json"))
    ap.add_argument("--annotations", action="append", required=True,
                    help="one folder per annotator; repeatable")
    ap.add_argument("--role", action="append", required=True,
                    choices=["demo", "briefed", "unbriefed"],
                    help="one per --annotations, same order")
    ap.add_argument("--out", default=str(ROOT / "out" / "annotation"))
    args = ap.parse_args()
    if len(args.role) != len(args.annotations):
        ap.error("--role count must match --annotations count")

    import pandas as pd
    manifest = json.loads(Path(args.manifest).read_text())
    frames = []
    for folder, role in zip(args.annotations, args.role):
        print(f"scoring {folder} (role={role})")
        frames.append(score_annotator(Path(folder), manifest, role)["df"])
    df = pd.concat(frames, ignore_index=True)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    df.to_csv(out / "annotation_scores.csv", index=False)

    head = df[df.role != "demo"]                       # the contamination rule, enforced
    print("\n=== HEADLINE (demo excluded) — the label-noise floor ===")
    if len(head):
        g = head.groupby(["annotator", "role"]).agg(
            scenes=("scene", "nunique"),
            missed=("missed", "sum"), extra=("extra", "sum"),
            lat_rmse_med=("lat_rmse", "median"),
            lat_rmse_p95=("lat_rmse", lambda x: x.quantile(0.95)),
            end_err_med=("end_err", "median"))
        print(g.round(3).to_string())
        print("\nper joint type (localization, mm):")
        print(head.groupby("joint_type")[["lat_rmse", "lat_me", "end_err"]]
              .median().round(3).to_string())
        print("\nwhich D19 curve the annotator actually clicked (median RMSE, mm):")
        print(head[["rmse_nominal", "rmse_root", "rmse_gapmid"]].median()
              .round(3).to_string())
        print("\ncompare against the ~0,6 mm RMSE the literature reports against such labels.")

        # the perturbation model for the training experiment
        model = {}
        for ann, sub in head.groupby("annotator"):
            m = sub.dropna(subset=["lat_rmse"])
            model[ann] = {
                "role": sub.role.iloc[0],
                "lateral_sigma_mm": float(m.lat_rmse.median()) if len(m) else None,
                "endpoint_sigma_mm": float(m.end_err.median()) if len(m) else None,
                "miss_rate": float(sub.missed.sum() / max(sub.n_gt.sum(), 1)),
                "extra_rate": float(sub.extra.sum() / max(sub.n_gt.sum(), 1)),
                "per_joint_lateral_mm": {jt: float(v) for jt, v in
                                         m.groupby("joint_type").lat_rmse.median()
                                         .items()},
            }
        (out / "annotator_model.json").write_text(json.dumps(model, indent=2))
        print(f"\nperturbation model -> {out / 'annotator_model.json'}")
    else:
        print("only demo annotations present - no headline; recruit the briefed annotator")
    demo = df[df.role == "demo"]
    if len(demo):
        print(f"\n(demo pass: {demo.annotator.iloc[0]}, {demo.scene.nunique()} scenes, "
              f"lat_rmse med {demo.lat_rmse.median():.3f} mm - briefing record only, "
              f"excluded above)")


if __name__ == "__main__":
    main()
