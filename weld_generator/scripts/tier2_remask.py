"""Rebuild every view's seam and tack masks of rendered scenes from the stored cameras -
no renderer. Masks are CONSTRUCTED (seams.npz + the analytic visibility), so a visibility
fix after a render is applied here: `mask_seam.png`, `mask_tack.png`, the per-view hashes
in `view.json`, and `render.json` / `render.sha256` are rewritten; rgb, depth, validity and
the object mask are untouched.

    python scripts/tier2_remask.py <scene_dir>... [--quiet]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from weldgen.geom import from_object  # noqa: E402
from weldgen.render.masks import seam_and_tack_masks  # noqa: E402
from weldgen.render.writer import render_hash  # noqa: E402


def _sha(arr: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(arr).tobytes()).hexdigest()


def remask_scene(sd: pathlib.Path, cfg_masks: dict) -> tuple[int, int]:
    scene = json.loads((sd / "scene.json").read_text()); seams_npz = dict(np.load(sd / "seams.npz"))
    parts = [from_object(o) for o in scene["objects"]]
    r = json.loads((sd / "render.json").read_text())
    changed = 0
    for e in r["views"]:
        if e.get("view_kind") == "undrawable":
            continue
        vd = sd / "views" / str(e["view"])
        ms, mt, stats = seam_and_tack_masks(scene, seams_npz, parts, e["K"], e["T_world_cam"], e["width"], e["height"],
                                            cfg_masks["seam_width_mm"], cfg_masks["tack_width_mm"])
        new = {"mask_seam": _sha(ms), "mask_tack": _sha(mt)}
        if new != {k: e["hashed"][k] for k in new}:
            Image.fromarray(ms).save(vd / "mask_seam.png", compress_level=6); Image.fromarray(mt).save(vd / "mask_tack.png", compress_level=6)
            e["hashed"].update(new); e["masks"] = {**e.get("masks", {}), **stats}; changed += 1
        vj = json.loads((vd / "view.json").read_text()); vj["hashed"] = e["hashed"]; vj["masks"] = e["masks"]
        (vd / "view.json").write_text(json.dumps(vj, indent=1, sort_keys=True))
    digest = render_hash(r["views"], r["config"])
    r["remasked"] = True
    (sd / "render.json").write_text(json.dumps(r, indent=1, sort_keys=True)); (sd / "render.sha256").write_text(digest + "\n")
    return changed, len(r["views"])


def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("scenes", nargs="+"); ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    tot_c = tot_v = n = 0
    for s in args.scenes:
        sd = pathlib.Path(s)
        if not (sd / "render.json").exists():
            continue
        cfg_masks = json.loads((sd / "render.json").read_text())["config"]["masks"]
        c, v = remask_scene(sd, cfg_masks); tot_c += c; tot_v += v; n += 1
        if not args.quiet:
            print(f"{sd.parent.name}/{sd.name}: {c}/{v} views re-masked", flush=True)
    print(f"{n} scenes: {tot_c}/{tot_v} views changed", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
