"""Twin gate on view 0 of one or more scenes — Phase 8 M2.

    cd /tmp && PYTHONUNBUFFERED=1 /isaac-sim/python.sh scripts/tier2_gate.py <scene_dir>... [--json out.json] [--no-substrate]

Renders each scene from its tier-1 camera in one Isaac app (clearing /World between scenes),
runs `weldgen.render.gate.twin_gate`, prints one line per scene and exits non-zero if any fails.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from weldgen.geom import from_object  # noqa: E402
from weldgen.render.gate import format_report, twin_gate  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+")
    ap.add_argument("--json", default=None)
    ap.add_argument("--no-substrate", action="store_true")
    ap.add_argument("--subframes", type=int, default=16)
    ap.add_argument("--dump", default=None, help="directory to save <scene>.npz with depth/valid/mask/normals")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    from weldgen.render.replicator import Renderer
    from weldgen.render.usd_stage import CAMERA_PATH, build_stage, clear_stage

    reports, renderer, bad = {}, None, 0
    for sd in args.scenes:
        sd = pathlib.Path(sd); t0 = time.time()
        scene = json.loads((sd / "scene.json").read_text())
        cloud = dict(np.load(sd / "cloud.npz"))
        parts = [from_object(o) for o in scene["objects"]]; meshes = [p.mesh() for p in parts]
        cam = scene["camera"]
        clear_stage()
        h = build_stage(parts, meshes, substrate=None if args.no_substrate else {})
        h.set_camera(cam["K"], cam["T_world_cam"], cam["width"], cam["height"])
        if renderer is None or (renderer.width, renderer.height) != (cam["width"], cam["height"]):
            renderer = Renderer(CAMERA_PATH, cam["width"], cam["height"], args.subframes)
        out = renderer.render(h.label_by_path)
        if args.dump:
            pathlib.Path(args.dump).mkdir(parents=True, exist_ok=True)
            np.savez_compressed(pathlib.Path(args.dump) / f"{sd.name}.npz", depth_mm=out["depth_mm"].astype(np.float32),
                                valid=out["valid"], mask_object=out["mask_object"], normals=out["normals"], rgb=out["rgb"])
        rep = twin_gate(scene, cloud, meshes, out["depth_mm"], out["valid"], out["mask_object"], dict(np.load(sd / "seams.npz")))
        rep["seconds"] = round(time.time() - t0, 1); reports[sd.name] = rep
        bad += not rep["pass"]
        print(f"{sd.parent.name}/{sd.name}: {format_report(rep)}  [{rep['seconds']}s]", flush=True)
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(reports, indent=1))
    print(f"{len(reports) - bad}/{len(reports)} scenes pass the twin gate", flush=True)
    app.close()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
