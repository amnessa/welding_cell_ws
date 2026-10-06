#!/usr/bin/env python3
"""Generate every synthetic scenario, run the pipeline on it and print a table.

  python test_synthetic.py                  # all scenarios, default settings
  python test_synthetic.py --noise 0.2      # noisier scans
  python test_synthetic.py --perturb 2 --register   # misaligned base scan + ICP
"""
import argparse
import os

from make_synthetic import SCENARIOS, generate
from weld_seam import build_parser, print_summary, run

ap = argparse.ArgumentParser()
ap.add_argument("--scenarios", nargs="*", default=SCENARIOS)
ap.add_argument("--spacing", type=float, default=1.0)
ap.add_argument("--noise", type=float, default=0.05)
ap.add_argument("--perturb", type=float, default=0.0)
ap.add_argument("--register", action="store_true")
ap.add_argument("--out", default="test_out")
a, extra = ap.parse_known_args()   # unknown args are passed through to weld_seam

rows = []
for name in a.scenarios:
    p = generate(name, os.path.join(a.out, "data"), a.spacing, a.noise, a.perturb)
    argv = ["--base", p["base"], "--combined", p["combined"], "--gt", p["gt"],
            "--out", os.path.join(a.out, name)] + (["--register"] if a.register else []) + extra
    print(f"\n=== {name}")
    s = run(build_parser().parse_args(argv))
    print_summary(s)
    e = s.get("ground_truth_eval", {})
    rows.append((name, len(s["seams"]), e.get("path_to_gt_mean", float("nan")),
                 e.get("path_to_gt_p95", float("nan")), e.get("gt_coverage", 0.0),
                 ", ".join(f"{q['joint_type']}~{q['theta_median_deg']:.0f}" for q in s["seams"])))

print(f"\n{'scenario':15s} {'seams':>5s} {'mean err':>9s} {'p95 err':>8s} {'coverage':>9s}  joints")
for r in rows:
    print(f"{r[0]:15s} {r[1]:5d} {r[2]:9.3f} {r[3]:8.3f} {r[4]:9.1%}  {r[5]}")
