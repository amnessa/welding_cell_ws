#!/usr/bin/env python3
"""Refine the camera extrinsic's two tilt angles from table views. numpy only.

    python helper/calibration/refine_extrinsic_from_table.py                      # report only
    python helper/calibration/refine_extrinsic_from_table.py --write              # -> T_tcp_to_cam_refined.npy

Input: `notebooks/extrinsic_check.json` (written by scripts/extrinsic_check.py: per view
the camera rotation in base_link and the tilt of the table the camera saw against the
pen-measured plane) and the extrinsic that was IN USE during those captures
(`--extrinsic`, default notebooks/T_tcp_to_cam.npy).

The ChArUco hand-eye leaves ~0.3 deg of rotation uncertainty (1.8 deg per view over ~30
views); the table views pin the camera's two tilt angles far tighter. The fit
(`table_probe.refine_camera_rotation`) separates the camera-fixed tilt from the table's
own tilt where the camera looked, and returns the small rotation Rc to apply:

    T_tool0_cam_refined = T_tool0_cam_used @ [[Rc, 0], [0, 1]]

Rotation about the optical axis and the translation are NOT touched (a plane cannot see
them). Writes a NEW file; the file in use is never overwritten. To use it, launch with
`extrinsic_path:=.../T_tcp_to_cam_refined.npy`, run extrinsic_check again (wrist at four
yaws): the camera-fixed tilt should drop to ~0.1 deg or below.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PKG))

from admittance_control.table_probe import refine_camera_rotation, tilt_to_normal  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", default=str(PKG / "notebooks" / "extrinsic_check.json"))
    ap.add_argument("--extrinsic", default=str(PKG / "notebooks" / "T_tcp_to_cam.npy"),
                    help="the extrinsic that was IN USE while the views were captured")
    ap.add_argument("--plane", default=str(PKG / "notebooks" / "table_plane.json"))
    ap.add_argument("--write", action="store_true", help="write notebooks/T_tcp_to_cam_refined.npy (+ .json)")
    args = ap.parse_args()

    views = json.loads(Path(args.check).read_text())["views"]
    views = [v for v in views if "camera_R" in v]
    if len(views) < 3:
        print(f"need >= 3 views with the camera rotation recorded, have {len(views)}")
        return 1
    pl = json.loads(Path(args.plane).read_text())["plane"]
    n0 = np.asarray(pl["normal"], float)
    Rs = np.array([v["camera_R"] for v in views])
    Ns = np.array([tilt_to_normal(n0, v["tilt_deg"], v["tilt_azimuth_deg"]) for v in views])
    yaws = [v.get("camera_yaw_deg", 0.0) for v in views]
    out = refine_camera_rotation(Rs, Ns, n0)
    X = np.load(args.extrinsic)
    Rc = np.asarray(out["Rc"])
    Xr = X.copy(); Xr[:3, :3] = X[:3, :3] @ Rc
    w = out["w_camera_deg"]
    print(f"{out['n_views']} views, camera yaws {np.round(yaws, 0).tolist()}")
    print(f"camera tilt correction: {out['w_deg']:.3f} deg (about camera x {w[0]:+.3f}, y {w[1]:+.3f}; "
          f"about the optical axis: not observable, left as is)")
    print(f"table's own tilt there vs the pen plane: {out['world_tilt_deg']:.3f} deg")
    print(f"fit residual {out['residual_deg']:.3f} deg, conditioning {out['conditioning']:.2f} "
          f"({'ok' if out['conditioning'] > 0.3 else 'WEAK - use yaws spread over 180+ deg'})")
    print(f"at a 300 mm range this moves points by {300 * np.tan(np.deg2rad(out['w_deg'])):.1f} mm")
    if args.write:
        dst = Path(args.extrinsic).with_name("T_tcp_to_cam_refined.npy")
        np.save(dst, Xr)
        dst.with_suffix(".json").write_text(json.dumps({
            "written": time.strftime("%Y-%m-%d %H:%M:%S"), "base_extrinsic": str(Path(args.extrinsic).resolve()),
            "check": str(Path(args.check).resolve()), "plane": str(Path(args.plane).resolve()),
            "correction": out, "frame": "tool0",
            "note": "tilt refined from table views; translation and rotation about the optical axis unchanged"},
            indent=1))
        print(f"wrote {dst} (+ .json). Launch with extrinsic_path:={dst} and re-run extrinsic_check.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
