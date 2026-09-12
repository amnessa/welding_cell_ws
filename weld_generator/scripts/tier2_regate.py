"""Recompute the twin gate of rendered scenes from their stored view-0 files (no renderer).

    python scripts/tier2_regate.py <scene_dir>...   [--json out.json]

Reads `views/0/depth.png` (0,05 mm quantised, so residuals carry up to 0,025 mm of that),
`mask_object.png`, the tier-1 cloud, rebuilds the meshes, runs `render.gate.twin_gate` and
rewrites `twin_gate` in `render.json` (`render.sha256` covers files, not the report, so it is
unchanged). For gate-definition changes after a render, and for audits.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from weldgen.geom import from_object  # noqa: E402
from weldgen.render.gate import format_report, twin_gate  # noqa: E402
from weldgen.render.writer import read_view  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+"); ap.add_argument("--json", default=None); ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    out, bad = {}, 0
    for sd in args.scenes:
        sd = pathlib.Path(sd)
        if not (sd / "render.json").exists():
            continue
        scene = json.loads((sd / "scene.json").read_text()); cloud = dict(np.load(sd / "cloud.npz"))
        meshes = [from_object(o).mesh() for o in scene["objects"]]
        v = read_view(sd / "views" / "0")
        rep = twin_gate(scene, cloud, meshes, v["depth_mm"], v["valid"], v["mask_object"], dict(np.load(sd / "seams.npz")))
        r = json.loads((sd / "render.json").read_text())
        rep["sensor"] = (r.get("twin_gate") or {}).get("sensor"); rep["regated"] = True
        r["twin_gate"] = rep; (sd / "render.json").write_text(json.dumps(r, indent=1, sort_keys=True))
        out[sd.name] = rep; bad += not rep["pass"]
        if not args.quiet:
            print(f"{sd.parent.name}/{sd.name}: {format_report(rep)}", flush=True)
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(out, indent=1))
    print(f"{len(out) - bad}/{len(out)} pass", flush=True)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
