#!/usr/bin/env python3
"""Replay a multi-view capture offline: everything refine_pose does after the robot has
moved, printed, nothing changed unless asked. No ROS. (Step 4 of
notes/multiview_refine_plan.md.)

    python3 scripts/multiview_refine_offline.py                      # the newest capture
    python3 scripts/multiview_refine_offline.py <capture dir>        # a given one
    python3 scripts/multiview_refine_offline.py <dir> --write        # + <dir>/assembly_refined.json
    python3 scripts/multiview_refine_offline.py <dir> --record       # + its d into selfcal_history.json
    python3 scripts/multiview_refine_offline.py --make-synthetic DIR [--d 3 -2 1]
        # write a synthetic capture of the bench T (4 planned views, known truth) and replay it

Captures live in <results>/multiview/<timestamp>/ (admittance_control/multiview_capture.py:
capture.json, view_<k>.npz with the organized cloud in the camera frame, assembly.json).
Tuning knobs of the refinement and the preprocessing can be overridden with --set
name=value (any RefineConfig / PreprocessConfig field), e.g. --set max_relative_mm=4.

--record only accepts a capture taken with the extrinsic file that is in use now (its
sha1 is in capture.json): an estimate under another extrinsic would mix calibrations.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control import multiview as mv  # noqa: E402
from admittance_control import multiview_capture as mc  # noqa: E402
from admittance_control import multiview_refine as mr  # noqa: E402
from admittance_control import selfcal as sc  # noqa: E402

RESULTS = PKG / "scripts" / "foundationpose_results"
# the bench T as registered on 2026-10-01 and the views plan_views picked for it
BASE = np.array([[-0.99432, 0.00976, 0.10594, -0.42685], [0.10569, -0.02313, 0.99413, -0.06161],
                 [0.01216, 0.99968, 0.02196, -0.05357], [0.0, 0.0, 0.0, 1.0]])
EAR = np.array([[0.66784, -0.74381, -0.02711, -0.60938], [-0.74351, -0.66837, 0.02196, 0.16042],
                [-0.03445, 0.00549, -0.99939, 0.056], [0.0, 0.0, 0.0, 1.0]])
T_VIEWS = [(60, 45, 180), (300, 60, 90), (0, 45, 180), (240, 45, 270)]


def _ortho(T):
    U, _, Vt = np.linalg.svd(T[:3, :3])
    T = T.copy()
    T[:3, :3] = U @ Vt
    return T


def make_synthetic(out: Path, d_cam_mm, seed: int = 0) -> Path:
    """The bench T at its true pose, saved 1-2 mm / 0.3 deg off, seen from the 4 planned
    views at 0.4 m, with an optional extrinsic translation error d (camera frame)."""
    models = PKG / "models"
    assembly = {"objects": [{"model": "test_objv2_base.ply", "pose_static": _ortho(BASE).tolist()},
                            {"model": "test_objv2_ear.ply", "pose_static": _ortho(EAR).tolist()}]}
    parts = mc.parts_from_assembly(assembly, models)
    truth = [p.T_saved for p in parts]
    R = mr._rodrigues
    saved = [mr._T(R(np.radians(0.3) * np.array([0.7, 0.7, 0])), np.array([0.0012, -0.0008, 0.0010])) @ truth[0],
             mr._T(R(np.radians(0.3) * np.array([0, 0.7, 0.7])), np.array([-0.0008, 0.0012, -0.0005])) @ truth[1]]
    allp = np.vstack([mr.posed(p, T)[0] for p, T in zip(parts, truth)])
    target = (allp.min(0) + allp.max(0)) / 2
    cams, metas = [], []
    for az, el, roll in T_VIEWS:
        e, a = np.radians(el), np.radians(az)
        cam = target + 0.4 * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        cams.append(mv.look_at(cam, target, np.radians(roll)))
        metas.append({"azimuth_deg": az, "elevation_deg": el, "roll_deg": roll})
    ext = PKG / "notebooks" / "T_tcp_to_cam.npy"
    extra = {"extrinsic_file": str(ext), "extrinsic_sha1": sc.file_sha1(ext) if ext.exists() else "",
             "kinematics": "synthetic", "ground_plane_file": ""}
    return mc.render_synthetic(out, parts, truth, saved, cams, d_cam_mm, view_meta=metas,
                               models_dir=models, extra_meta=extra, seed=seed)


def _apply_sets(sets, rcfg, pcfg):
    for s in sets or []:
        name, val = s.split("=", 1)
        for cfg in (rcfg, pcfg):
            if hasattr(cfg, name):
                typ = type(getattr(cfg, name))
                setattr(cfg, name, typ(json.loads(val)) if typ is not bool else val.lower() in ("1", "true", "yes"))
                break
        else:
            raise SystemExit(f"--set {name}: no such RefineConfig / PreprocessConfig field")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="?", help="capture dir (default: the newest in <results>/multiview)")
    ap.add_argument("--results", default=str(RESULTS))
    ap.add_argument("--models", default=str(PKG / "models"))
    ap.add_argument("--plane", default=None, help="table_plane.json (default: the one recorded in the capture)")
    ap.add_argument("--set", action="append", help="override a RefineConfig/PreprocessConfig field: name=value")
    ap.add_argument("--write", action="store_true", help="write <capture>/assembly_refined.json")
    ap.add_argument("--record", action="store_true", help="append d to notebooks/selfcal_history.json")
    ap.add_argument("--history", default=str(PKG / "notebooks" / "selfcal_history.json"))
    ap.add_argument("--make-synthetic", metavar="DIR", help="write a synthetic capture of the bench T there, then replay it")
    ap.add_argument("--d", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="synthetic: extrinsic error d (camera frame, mm)")
    args = ap.parse_args()

    if args.make_synthetic:
        path = make_synthetic(Path(args.make_synthetic), args.d)
        print(f"wrote synthetic capture {path}")
    else:
        path = Path(args.capture) if args.capture else mc.latest_capture(args.results)
        if path is None:
            print(f"no capture in {Path(args.results) / 'multiview'}; give a directory or --make-synthetic")
            return 1
    cap = mc.load_capture(path)
    rcfg, pcfg = mr.RefineConfig(), mc.PreprocessConfig()
    _apply_sets(args.set, rcfg, pcfg)
    rp = mc.refine_capture(cap, args.models, rcfg, pcfg, args.plane)
    print(mc.format_replay(rp))

    if args.write:
        out = {"static_frame": cap.assembly.get("static_frame", "base_link"), "objects": []}
        for o, r in zip(cap.assembly["objects"], rp.results):
            o2 = dict(o)
            o2["pose_static"] = np.asarray(r.T, float).tolist()
            o2["refine"] = {"accepted": r.accepted, "reason": r.reason, "pose_before": np.asarray(r.T_saved).tolist(),
                            "correction_mm": r.correction_mm, "correction_deg": r.correction_deg,
                            "not_measured": r.weak, "view_spread_mm": r.view_spread_mm,
                            "capture": str(path)}
            out["objects"].append(o2)
        (path / "assembly_refined.json").write_text(json.dumps(out, indent=1))
        print(f"wrote {path / 'assembly_refined.json'} (the results' assembly.json is NOT touched)")
    if args.record:
        d = rp.diag
        ext = cap.meta.get("extrinsic_file", "")
        if d.extrinsic_d_mm is None:
            print("nothing to record: no extrinsic estimate")
            return 1
        if not ext or not Path(ext).exists() or sc.file_sha1(ext) != cap.meta.get("extrinsic_sha1"):
            print(f"NOT recorded: the capture was taken with another extrinsic than {ext or '(none)'} now holds")
            return 1
        e = sc.record_run(args.history, d.extrinsic_d_mm, d.extrinsic_info, ext,
                          {"capture": str(path), "n_views": len(rp.views)})
        print(f"recorded d = {np.round(e['d_cam_mm'], 2).tolist()} mm into {args.history}; "
              f"see python3 scripts/selfcal_extrinsic.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
