#!/usr/bin/env python3
"""Reachability report for the tacks of a saved assembly - no ROS, no motion.

    python src/admittance_control/scripts/tack_reachability.py                       # foundationpose_results/
    python src/admittance_control/scripts/tack_reachability.py --save-dir src/admittance_control/scripts/foundationpose_results \
        --registry src/admittance_control/models/weldgen_objects.json --out tack_reach.json

Reads `welding_tacks.json` + `assembly.json` from the save dir (what the ICP node wrote
after `~/welding_points`), rebuilds the parts as collision boxes through mode A's
registry, loads the tool (`config/pen_tool.json`) and the marking settings
(`config/marking.json`: scan-home pose = branch lock), and prints per seam the roll
chosen and per tack the approach / tack / descent clearances and the joint step from
the previous tack. Writes `<save-dir>/tack_reach.json` for the marker node
(`tack_reach_marker_node.py`) and, later, the marking node.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG.parents[2] / "weld_generator"))

from admittance_control import seam_from_registration as sfr  # noqa: E402
from admittance_control.collision import CollisionModel, boxes_from_parts  # noqa: E402
from admittance_control.tack_reach import (format_report, load_marking_config,  # noqa: E402
                                           plan_tacks)
from admittance_control.tool_model import load_tool_model  # noqa: E402
from admittance_control.weldgen_registry import load_registry  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--save-dir", default=str(PKG / "scripts" / "foundationpose_results"))
    ap.add_argument("--registry", default=str(PKG / "models" / "weldgen_objects.json"))
    ap.add_argument("--tool", default=None, help="pen_tool.json (default: config/)")
    ap.add_argument("--marking", default=None, help="marking.json (default: config/)")
    ap.add_argument("--extrinsic", default=None, help="T_tcp_to_cam.npy for the camera reference")
    ap.add_argument("--table-z", type=float, default=None, help="table plane (m); overrides marking.json")
    ap.add_argument("--clearance", type=float, default=None,
                    help="required gap (m); overrides marking.json - e.g. 0.003 when the envelope is known conservative")
    ap.add_argument("--out", default=None, help="report JSON (default: <save-dir>/tack_reach.json)")
    args = ap.parse_args()

    save_dir = Path(args.save_dir)
    tacks = json.loads((save_dir / "welding_tacks.json").read_text())["tacks"]
    assembly = json.loads((save_dir / "assembly.json").read_text())
    objects = [(o["model"], np.asarray(o["pose_static"], float).reshape(4, 4))
               for o in assembly["objects"]]
    parts = sfr.posed_parts(objects, load_registry(args.registry))

    cfg = load_marking_config(args.marking)
    if args.table_z is not None:
        cfg.table_z_m = args.table_z
    if args.clearance is not None:
        cfg.clearance_m = args.clearance
    tool = load_tool_model(args.tool, args.extrinsic)
    model = CollisionModel(tool=tool, scene_boxes=boxes_from_parts(parts),
                           table_z=cfg.table_z_m, clearance=cfg.clearance_m)
    print(f"{len(tacks)} tacks, {len(parts)} parts, table_z={cfg.table_z_m}, "
          f"clearance {cfg.clearance_m * 1000:g} mm, standoff {cfg.standoff_m * 1000:g} mm")
    report = plan_tacks(tacks, tool, model, cfg)
    print(format_report(report))
    out = Path(args.out) if args.out else save_dir / "tack_reach.json"
    report["source"] = {"tacks": str(save_dir / "welding_tacks.json"),
                        "assembly": str(save_dir / "assembly.json"), "frame": assembly.get("static_frame", "base_link")}
    out.write_text(json.dumps(report, indent=1))
    print(f"wrote {out}")
    return 0 if report["all_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
