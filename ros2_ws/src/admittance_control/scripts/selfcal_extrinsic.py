#!/usr/bin/env python3
"""Self-calibrate the camera extrinsic's translation from refine_pose runs. No ROS.

    python3 scripts/selfcal_extrinsic.py            # the history and whether the runs agree
    python3 scripts/selfcal_extrinsic.py --write    # ... and write T_tcp_to_cam_selfcal.npy (+ .json)

Each multi-view refine_pose run appends its estimate of the extrinsic translation error
(camera frame, only the directions its views determine, with its information) to
notebooks/selfcal_history.json. Runs made with a different extrinsic file (sha1) are not
counted. The runs are combined by their information; with at least 3 that agree within
1 mm, `--write` puts the corrected extrinsic NEXT TO the one in use (directions no run
determines are left as they were, and named). Promote it by hand, as with the table refinement:

    cp notebooks/T_tcp_to_cam.npy notebooks/T_tcp_to_cam_before_selfcal.npy
    cp notebooks/T_tcp_to_cam_selfcal.npy notebooks/T_tcp_to_cam.npy

After promotion the history of the NEW file starts empty; the next runs should then
estimate d near 0 - that is the check.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.selfcal import (SelfcalConfig, evaluate, file_sha1, load_history,  # noqa: E402
                                        write_selfcal)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--history", default=str(PKG / "notebooks" / "selfcal_history.json"))
    ap.add_argument("--extrinsic", default=str(PKG / "notebooks" / "T_tcp_to_cam.npy"))
    ap.add_argument("--min-runs", type=int, default=SelfcalConfig.min_runs)
    ap.add_argument("--max-spread-mm", type=float, default=SelfcalConfig.max_spread_mm)
    ap.add_argument("--write", action="store_true", help="write T_tcp_to_cam_selfcal.npy when the runs agree")
    ap.add_argument("--kinematics", default="calibrated (config/ur5e_calibration.yaml)")
    args = ap.parse_args()

    cfg = SelfcalConfig(min_runs=args.min_runs, max_spread_mm=args.max_spread_mm)
    runs = load_history(args.history)
    sha = file_sha1(args.extrinsic)
    print(f"extrinsic {args.extrinsic} (sha1 {sha[:10]})\nhistory {args.history}: {len(runs)} run(s)")
    for k, r in enumerate(runs):
        same = "this extrinsic" if r.get("extrinsic_sha1") == sha else "OTHER extrinsic"
        I = r.get("info_per_mm2")
        det = 0 if I is None else int((np.linalg.eigvalsh(np.asarray(I)) >= 1.0 / cfg.max_sigma_mm ** 2).sum())
        print(f"  {k}: {r.get('time', '?')}  d = {np.round(r['d_cam_mm'], 2).tolist()} mm, "
              f"determines {det} of 3 directions, {same}")
    v = evaluate(runs, sha, cfg)
    if v.skipped:
        print("  not counted: " + ", ".join(f"{k} {n}" for k, n in v.skipped.items()))
    print(("READY: " if v.ready else "NOT READY: ") + v.message)
    if not args.write:
        return 0 if v.ready else 1
    if not v.ready:
        print("nothing written")
        return 1
    out = write_selfcal(args.extrinsic, v, runs, kinematics=args.kinematics)
    T0, T1 = np.load(args.extrinsic), np.load(out)
    print(f"wrote {out} (+ .json): translation in tool0 {np.round(T0[:3, 3] * 1000, 2).tolist()} -> "
          f"{np.round(T1[:3, 3] * 1000, 2).tolist()} mm. Promote by hand (see --help).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
