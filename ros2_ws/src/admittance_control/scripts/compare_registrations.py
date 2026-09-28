#!/usr/bin/env python3
"""Compare the registered poses of the same parts in two saved assemblies. No ROS.

    python scripts/compare_registrations.py <assembly A> <assembly B>
    # each argument: a save dir, an archived previous_assemblies/<ts>/ dir, or assembly.json
    python scripts/compare_registrations.py --last 2        # the two newest archives

The test it serves (2026-09-28): the table checks the camera's TILT, not its
horizontal position or its rotation about its own viewing axis - and two ChArUco
calibrations disagreed by 18 mm in exactly that position. Register the SAME, untouched
part twice from scan poses whose wrist yaw differs by 180 deg (save_object, then
reset_environment between them). A horizontal extrinsic translation error d appears in
base_link as R_tool d, so it flips with the wrist: the two registrations differ by ~2d
horizontally, while a correct extrinsic gives only the registration noise (the
pose_stats spread each save records). The part must not move between the two scans.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]


def _load(arg: str) -> dict:
    p = Path(arg)
    if p.is_dir():
        p = p / "assembly.json"
    return json.loads(p.read_text())


def _angle_deg(Ra, Rb) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", nargs="?")
    ap.add_argument("b", nargs="?")
    ap.add_argument("--last", type=int, default=0, help="compare the N newest archived assemblies (N=2)")
    args = ap.parse_args()
    if args.last:
        arch = sorted((PKG / "scripts" / "foundationpose_results" / "previous_assemblies").iterdir())
        arch = [d for d in arch if (d / "assembly.json").exists()][-args.last:]
        if len(arch) < 2:
            print("fewer than 2 archived assemblies"); return 1
        a, b = str(arch[-2]), str(arch[-1])
    else:
        if not (args.a and args.b):
            ap.error("give two assemblies or --last 2")
        a, b = args.a, args.b
    A, B = _load(a), _load(b)
    print(f"A: {a}\nB: {b}")
    by_model = {}
    for o in A["objects"]:
        by_model.setdefault(o["model"], []).append(o)
    found = False
    for ob in B["objects"]:
        cand = by_model.get(ob["model"])
        if not cand:
            continue
        oa = cand.pop(0)
        Ta = np.asarray(oa["pose_static"], float).reshape(4, 4)
        Tb = np.asarray(ob["pose_static"], float).reshape(4, 4)
        d = (Tb[:3, 3] - Ta[:3, 3]) * 1000.0
        ang = _angle_deg(Ta[:3, :3], Tb[:3, :3])
        sa, sb = oa.get("pose_stats") or {}, ob.get("pose_stats") or {}
        noise = [np.linalg.norm(s["std_mm"]) for s in (sa, sb) if s.get("std_mm")]
        found = True
        print(f"\n{ob['model']}: B - A = ({d[0]:+.1f}, {d[1]:+.1f}, {d[2]:+.1f}) mm in base_link, "
              f"horizontal {np.hypot(d[0], d[1]):.1f} mm, rotation {ang:.2f} deg")
        if noise:
            print(f"   registration spread recorded at save: {', '.join(f'{x:.1f}' for x in noise)} mm")
        print(f"   if the scans were 180 deg apart in wrist yaw: horizontal extrinsic error ~ "
              f"{np.hypot(d[0], d[1]) / 2:.1f} mm (half the difference)")
    if not found:
        print("no part appears in both assemblies")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
