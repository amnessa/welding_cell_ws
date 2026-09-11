"""Render the tier-2 layer of scenes - Phase 8 M3 (batching in M6 builds on this).

    cd /tmp && PYTHONUNBUFFERED=1 /isaac-sim/python.sh scripts/render_tier2.py \
        --config configs/render/lab_v1.yaml <scene_dir>... [--views N] [--resume] [--no-gate-abort]

Per scene, in one Isaac app: draws (appearance, environment, cameras) from
sha256(scene_id, render_id); stage; view 0 from the tier-1 camera, twin-gated; views 1..N-1
drawn; masks from the truth; `views/<k>/...`, `render.json`, `render.sha256`. Tier-1 files are
never opened for writing. A scene whose view-0 gate fails is written with `twin_gate.pass =
false` and the run aborts unless `--no-gate-abort`.
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
from weldgen.render.config import hashable_config, load_backgrounds, load_render_assets, load_render_config, render_id  # noqa: E402
from weldgen.render.draws import draw_appearance, draw_views, primary_seams, render_rng  # noqa: E402
from weldgen.render.gate import format_report, twin_gate  # noqa: E402
from weldgen.render.masks import seam_and_tack_masks  # noqa: E402
from weldgen.render.hdr import dome_intensity_for  # noqa: E402
from weldgen.render.materials import recipe  # noqa: E402
from weldgen.render.sensor import sensor_validity, tier_comparison  # noqa: E402
from weldgen.render.writer import write_render, write_view  # noqa: E402


def material_spec(part, draws: dict, tex_by_id: dict, assets: dict | None):
    """materials-1.0 recipe for a workpiece when the surface set is available; None -> default."""
    if part.role != "workpiece":
        return None
    td = draws.get("textures", {}).get(part.id)
    if not td or assets is None or td["asset_id"] not in tex_by_id:
        return None
    rec = recipe(draws["alloy"], draws["surface_condition"].get(part.id, "ground"), tex_by_id[td["asset_id"]], td["roughness_jitter"])
    return {**rec, "assets_dir": assets["_dir"], "uv_rotation_deg": td["uv_rotation_deg"], "uv_offset": tuple(td["uv_offset"])}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes", nargs="+")
    ap.add_argument("--config", default=str(ROOT / "configs" / "render" / "lab_v1.yaml"))
    ap.add_argument("--views", type=int, default=None, help="override the config's view count")
    ap.add_argument("--resume", action="store_true", help="skip scenes that already have render.sha256")
    ap.add_argument("--no-gate-abort", action="store_true")
    args = ap.parse_args()

    cfg = load_render_config(args.config)
    bgs = load_backgrounds(cfg, ROOT); assets = load_render_assets(cfg, ROOT)
    rid = render_id(cfg, bgs, assets); hcfg = hashable_config(cfg, bgs, assets)
    asset_note = "none" if assets is None else f"{len(assets['hdris'])} HDRIs / {len(assets['textures'])} surface sets ({assets['set_hash']})"
    print(f"render_id {rid} ({cfg['_source']}), backgrounds {len(bgs['photos'])} photos / {len(bgs['_panoramas'])} panoramas, "
          f"assets: {asset_note}", flush=True)
    tex_by_id = {t["asset_id"]: t for t in (assets or {}).get("textures", [])}
    hdri_by_name = {h["name"]: h for h in (assets or {}).get("hdris", [])}

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import omni.kit.app
    kit_ver = omni.kit.app.get_app().get_build_version()
    from weldgen.render.replicator import Renderer
    from weldgen.render.usd_stage import CAMERA_PATH, build_stage, clear_stage

    renderer, n_fail = None, 0
    for sd in args.scenes:
        sd = pathlib.Path(sd); t0 = time.time()
        if args.resume and (sd / "render.sha256").exists():
            print(f"{sd.name}: exists, skipped", flush=True); continue
        scene = json.loads((sd / "scene.json").read_text())
        cloud = dict(np.load(sd / "cloud.npz")); seams_npz = dict(np.load(sd / "seams.npz"))
        parts = [from_object(o) for o in scene["objects"]]; meshes = [p.mesh() for p in parts]
        rng = render_rng(scene["scene_id"], rid)
        draws = draw_appearance(rng, cfg, scene, bgs, assets)
        views = draw_views(rng, cfg, scene, primary_seams(scene, seams_npz), parts, args.views)
        sub = draws["substrate"]; dome = draws["dome"]; key = draws["key_light"]
        clear_stage()
        h = build_stage(parts, meshes,
                        substrate={"photo": str(pathlib.Path(bgs["_dir"]) / sub["photo"]),
                                   "span_m": (sub["span_m"], sub["span_m_y"]), "rotation_rad": np.radians(sub["rotation_deg"]),
                                   "roughness": sub["roughness"], "half_m": sub["plane_half_m"]},
                        dome={"texture": (str(pathlib.Path(assets["_dir"]) / dome["file"]) if dome["kind"] == "hdri" else dome["file"]),
                              "intensity": dome_intensity_for(dome, cfg["lighting"], hdri_by_name),
                              "rotation_deg": dome.get("rotation_deg", 0.0)},
                        key_light={"intensity": key["intensity"],
                                   "rotation_xyz_deg": (-key["elevation_deg"], key["azimuth_deg"], 0.0)},
                        material_for=lambda p: material_spec(p, draws, tex_by_id, assets))
        cam = scene["camera"]; W, H = cam["width"], cam["height"]
        if renderer is None or (renderer.width, renderer.height) != (W, H):
            renderer = Renderer(CAMERA_PATH, W, H, int(cfg["rt_subframes"]))
        entries, gate = [], None
        for v in views:
            if v["view_kind"] == "undrawable":
                entries.append(v); continue
            h.set_camera(v["K"], v["T_world_cam"], W, H)
            out = renderer.render(h.label_by_path)
            ms, mt, mstats = seam_and_tack_masks(scene, seams_npz, parts, v["K"], v["T_world_cam"], W, H,
                                                 cfg["masks"]["seam_width_mm"], cfg["masks"]["tack_width_mm"])
            sv = sensor_validity(out["depth_mm"], out["valid"], out["normals"], v["K"], v["T_world_cam"], scene["noise_model"])
            if v["view"] == 0:
                gate = twin_gate(scene, cloud, meshes, out["depth_mm"], out["valid"], out["mask_object"])
                gate["sensor"] = tier_comparison(scene, cloud, out["depth_mm"], out["valid"], out["normals"], out["mask_object"])
            entries.append(write_view(sd / "views" / str(v["view"]), out["rgb"], out["depth_mm"], out["valid"], ms, mt,
                                      out["mask_object"], {**v, "masks": {"rule": cfg["masks"]["rule"], **mstats},
                                                            "sensor": {"profile": scene["noise_model"].get("profile"), "rule": "D16 deterministic validity; realisation via render.sensor.realise",
                                                                       "realisation_seed": int(scene["noise_model"]["seed"]) + int(v["view"])}},
                                      sensor_valid=sv))
        meta = {"render_id": rid, "scene_id": scene["scene_id"], "backend": {"name": "isaac_replicator", "kit": kit_ver},
                "draws": draws, "twin_gate": gate,
                "provenance": {"created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "config_source": cfg["_source"]}}
        digest = write_render(sd, meta, entries, hcfg)
        n_ok = sum(e.get("view_kind") != "undrawable" for e in entries)
        print(f"{sd.parent.name}/{sd.name}: {n_ok}/{len(views)} views, alloy {draws['alloy']}, {sub['photo']}, "
              f"{format_report(gate)}  sensor agree={gate['sensor']['agreement']:.3f} (t1 {gate['sensor']['tier1_valid_fraction']:.3f} vs t2 {gate['sensor']['tier2_valid_fraction_at_those_pixels']:.3f} on {gate['sensor']['compared_points']} pts)  "
              f"render.sha256 {digest[:12]}  [{time.time() - t0:.1f}s]", flush=True)
        if not gate["pass"]:
            n_fail += 1
            if not args.no_gate_abort:
                print("twin gate FAILED - aborting (use --no-gate-abort to continue)", flush=True); break
    app.close()
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
