"""PILOT (2026-09-11) — "look before render": RGB + depth + seam/tack/object masks for one
scene from the tier-1 camera (view 0) and one drawn view. Not the M2/M3 code; a prototype
to inspect by eye. Isaac Python only:

    cd /tmp && PYTHONUNBUFFERED=1 /isaac-sim/python.sh scripts/tier2_pilot_proto.py <scene_dir> <out_dir> [alloy]
"""
import sys, json, time, pathlib
sys.path.insert(0, "/workspaces/welding_cell_ws/weld_generator")
import numpy as np
from PIL import Image
from scipy import ndimage
from weldgen.geom import from_object
from weldgen.camera import project, sample_pose

SCENE = pathlib.Path(sys.argv[1]); OUT = pathlib.Path(sys.argv[2]); ALLOY = sys.argv[3] if len(sys.argv) > 3 else "mild_steel"
OUT.mkdir(parents=True, exist_ok=True)
scene = json.load(open(SCENE / "scene.json")); seams_npz = np.load(SCENE / "seams.npz"); cloud = np.load(SCENE / "cloud.npz")
parts = [from_object(o) for o in scene["objects"]]; meshes = [p.mesh() for p in parts]
cam = scene["camera"]; K = np.array(cam["K"]); W, H = cam["width"], cam["height"]
T0 = np.array(cam["T_world_cam"]); MM = 0.001

# six-alloy pilot palette: (base colour, metallic, roughness) — placeholder values for the look, not final
ALLOYS = {"mild_steel": ((0.42, 0.42, 0.44), 1.0, 0.45), "stainless": ((0.75, 0.76, 0.78), 1.0, 0.25),
          "aluminium": ((0.85, 0.86, 0.88), 1.0, 0.35), "cast_iron": ((0.28, 0.28, 0.29), 0.9, 0.7),
          "brass": ((0.85, 0.65, 0.25), 1.0, 0.3), "bronze": ((0.60, 0.40, 0.22), 1.0, 0.4)}
base, metallic, rough = ALLOYS[ALLOY]

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.replicator.core as rep, omni.usd
from pxr import UsdGeom, Gf, UsdLux, UsdShade, Sdf
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
dome = UsdLux.DomeLight.Define(stage, "/World/dome"); dome.GetIntensityAttr().Set(800)
key = UsdLux.DistantLight.Define(stage, "/World/key"); key.GetIntensityAttr().Set(2500); key.GetAngleAttr().Set(2.0)
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-40, 25, 0))
mat = UsdShade.Material.Define(stage, "/World/mat"); sh = UsdShade.Shader.Define(stage, "/World/mat/pbr")
sh.CreateIdAttr("UsdPreviewSurface"); sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*base))
sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic); sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
for p, m in zip(parts, meshes):
    um = UsdGeom.Mesh.Define(stage, f"/World/part_{p.id}")
    um.GetPointsAttr().Set([Gf.Vec3f(*(v * MM)) for v in m.vertices]); um.GetFaceVertexCountsAttr().Set([3] * len(m.faces))
    um.GetFaceVertexIndicesAttr().Set(m.faces.ravel().tolist()); um.GetSubdivisionSchemeAttr().Set("none")
    UsdShade.MaterialBindingAPI.Apply(um.GetPrim()).Bind(mat)
ucam = UsdGeom.Camera.Define(stage, "/World/cam"); ha = 20.955
ucam.GetFocalLengthAttr().Set(K[0, 0] * ha / W); ucam.GetHorizontalApertureAttr().Set(ha); ucam.GetVerticalApertureAttr().Set(ha * H / W)
ucam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 50.0)); xf = UsdGeom.Xformable(ucam); xf.ClearXformOpOrder(); op = xf.AddTransformOp()
rp = rep.create.render_product("/World/cam", (W, H))
ann = {n: rep.AnnotatorRegistry.get_annotator(n) for n in ("rgb", "distance_to_image_plane")}
for a in ann.values(): a.attach(rp)

def set_cam(T):
    Tu = T.copy(); Tu[:3, 3] *= MM; Tu = Tu @ np.diag([1, -1, -1, 1]); op.Set(Gf.Matrix4d(Tu.T.tolist()))

def stamp(mask_pts_uv, z, f, width_mm, value, out):
    """Draw points with a physical width: radius_px = width/2 * f / z, per-radius dilation."""
    r_px = np.clip(np.round(0.5 * width_mm * f / np.maximum(z, 1e-6)).astype(int), 1, 25)
    j = np.round(mask_pts_uv[:, 0] - 0.5).astype(int); i = np.round(mask_pts_uv[:, 1] - 0.5).astype(int)
    ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
    for r in np.unique(r_px[ok]):
        sel = ok & (r_px == r); base_m = np.zeros((H, W), bool); base_m[i[sel], j[sel]] = True
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]; disk = (xx ** 2 + yy ** 2) <= r ** 2
        out[ndimage.binary_dilation(base_m, disk) & (out == 0)] = value

def render_view(T, name):
    set_cam(T)
    for _ in range(2): rep.orchestrator.step(rt_subframes=32)
    rgb = ann["rgb"].get_data()[..., :3]; d = ann["distance_to_image_plane"].get_data().astype(np.float64) / MM
    valid = np.isfinite(d) & (d > 0)
    R, t = T[:3, :3], T[:3, 3]; f = K[0, 0]
    # masks: seams (weldable) and tacks, depth-agreement within 1.5 mm, physical width 2 mm
    mseam = np.zeros((H, W), np.uint8); mtack = np.zeros((H, W), np.uint8)
    for s in scene["seams"]:
        if not s["weldable"]: continue
        P = seams_npz[f"seam_{s['id']}"].astype(np.float64); uv, z = project(P, T, K)
        j = np.clip(np.round(uv[:, 0] - 0.5).astype(int), 0, W - 1); i = np.clip(np.round(uv[:, 1] - 0.5).astype(int), 0, H - 1)
        inimg = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H) & (z > 0)
        vis = inimg & valid[i, j] & (np.abs(d[i, j] - z) < 1.5)
        stamp(uv[vis], z[vis], f, 2.0, s["id"] + 1, mseam)
        sarr = seams_npz[f"seam_{s['id']}_s"].astype(np.float64); L = float(s["length_mm"])
        for ti, (sid, s0, tl) in enumerate(zip(scene["tacks"]["seam_id"], scene["tacks"]["arclength_mm"], scene["tacks"]["tack_length_mm"])):
            if sid != s["id"]: continue
            ds = np.abs(sarr - s0)
            if s["closed"]: ds = np.minimum(ds, L - ds)
            sel = vis & (ds <= tl / 2)
            stamp(uv[sel], z[sel], f, 3.0, ti + 1, mtack)
    # object mask by nearest primitive of back-projected pixel
    mobj = np.zeros((H, W), np.uint8); vv, uu = np.nonzero(valid); zc = d[vv, uu]
    pw = np.stack([(uu + 0.5 - K[0, 2]) / f * zc, (vv + 0.5 - K[1, 2]) / K[1, 1] * zc, zc], 1) @ R.T + t
    import trimesh
    best = np.full(len(pw), np.inf); lab = np.zeros(len(pw), np.uint8)
    for p, m in zip(parts, meshes):
        _, dist, _ = trimesh.proximity.closest_point(m, pw); better = dist < best; best[better] = dist[better]; lab[better] = p.object_id + 1
    mobj[vv, uu] = lab
    # write
    v = OUT / name; v.mkdir(exist_ok=True)
    Image.fromarray(rgb.astype(np.uint8)).save(v / "rgb.png")
    Image.fromarray(np.where(valid, np.round(d / 0.05), 0).astype(np.uint16)).save(v / "depth.png")
    Image.fromarray(mseam * 60).save(v / "mask_seam.png"); Image.fromarray(mtack * 25).save(v / "mask_tack.png"); Image.fromarray(mobj * 80).save(v / "mask_object.png")
    # review composite: rgb | depth (grey) | rgb with seam (green) + tacks (red) overlay
    dn = np.zeros((H, W), np.uint8)
    if valid.any(): lo, hi = d[valid].min(), d[valid].max(); dn[valid] = (255 - 200 * (d[valid] - lo) / max(hi - lo, 1e-6)).astype(np.uint8)
    ov = rgb.astype(np.float64).copy(); ov[mseam > 0] = 0.4 * ov[mseam > 0] + 0.6 * np.array([0, 255, 0]); ov[mtack > 0] = np.array([255, 0, 0])
    comp = np.concatenate([rgb, np.stack([dn] * 3, -1), ov.astype(np.uint8)], 1)
    Image.fromarray(comp).resize((comp.shape[1] // 2, comp.shape[0] // 2)).save(v / "review.png")
    json.dump({"K": K.tolist(), "T_world_cam": T.tolist(), "width": W, "height": H}, open(v / "view.json", "w"))
    print(f"VIEW {name}: valid={valid.mean():.3f} seam_px={int((mseam>0).sum())} tack_px={int((mtack>0).sum())} "
          f"tacks_drawn={len(np.unique(mtack))-1}/{len(scene['tacks']['seam_id'])} z={d[valid].min():.0f}..{d[valid].max():.0f}mm", flush=True)

t0 = time.time()
render_view(T0, "0")
# a drawn view: same standoff, elevation 45, azimuth +70 deg from view 0's, aimed at the tier-1 aim point (approx: cloud centroid)
aim = cloud["xyz"].astype(np.float64).mean(0); az0 = np.degrees(np.arctan2(T0[1, 3] - aim[1], T0[0, 3] - aim[0]))
render_view(sample_pose(aim, cam["standoff_mm"], 45.0, az0 + 70.0, 0.0), "1")
render_view(sample_pose(aim, cam["standoff_mm"] * 0.8, 30.0, az0 - 110.0, 5.0), "2")
print("TOTAL", round(time.time() - t0, 1), "s", flush=True)
app.close()
