"""PROTOTYPE (2026-09-10, notes/phase8_plan.md §1) - milestone (a) feasibility check.

Run under Isaac Python only:
    cd /tmp && PYTHONUNBUFFERED=1 /isaac-sim/python.sh scripts/tier2_gate_proto.py <scene_dir> [out.npz]

To be replaced by weldgen/render/ + scripts/tier2_gate.py in M2. Not imported by anything.
"""
import sys, time, json; t=time.time()
sys.path.insert(0, "/workspaces/welding_cell_ws/weld_generator")
import numpy as np
from weldgen.geom import Slab
from weldgen.camera import project
SCENE = sys.argv[1] if len(sys.argv) > 1 else "/workspaces/welding_cell_ws/weld_generator/out/bench_phase4/T/1ce3c6d2-0002000008"
scene = json.load(open(f"{SCENE}/scene.json")); z = np.load(f"{SCENE}/cloud.npz")
import dataclasses, trimesh
from weldgen.geom import Tube, Prism, PreparedPrism, SweptSlab
CLS = {"slab": Slab, "tube": Tube, "prism": Prism, "prepared_prism": PreparedPrism, "swept_slab": SweptSlab}
def from_object(o):
    cls = CLS[o["primitive"]]; names = {f.name for f in dataclasses.fields(cls)}
    kw = {"id": o["id"], "role": o["role"], "object_id": o["object_id"], "T_world_part": np.array(o["T_world_part"])}
    if cls is Slab: kw["dims_mm"] = tuple(o["dims_mm"])
    if "outline_uv" in o: kw["outline_uv"] = np.array(o["outline_uv"]); kw["shape"] = o.get("outline_shape", "polygon")
    if cls is Prism: kw["thickness"] = o["thickness_mm"]
    if cls is PreparedPrism: kw["t_mm"] = o["thickness_mm"]; kw["prep"] = o["params"]
    for k, v in (o.get("params") or {}).items():
        if k in names: kw[k] = v
    print("FACTORY", o["primitive"], sorted(kw), "missing:", sorted(names - set(kw)), flush=True)
    return cls(**{k: v for k, v in kw.items() if k in names})
slabs = [from_object(o) for o in scene["objects"]]
meshes = [s.mesh() for s in slabs]
print("MESHES", [(m.vertices.shape[0], m.faces.shape[0], m.is_watertight) for m in meshes], flush=True)
K = np.array(scene["camera"]["K"]); T = np.array(scene["camera"]["T_world_cam"]); W, H = scene["camera"]["width"], scene["camera"]["height"]
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.replicator.core as rep, omni.usd
from pxr import UsdGeom, Gf, UsdLux, Sdf
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
UsdLux.DistantLight.Define(stage, "/World/light").GetIntensityAttr().Set(3000)
MM = 0.001
for s, m in zip(slabs, meshes):
    um = UsdGeom.Mesh.Define(stage, f"/World/part_{s.id}")
    um.GetPointsAttr().Set([Gf.Vec3f(*(p*MM)) for p in m.vertices])
    um.GetFaceVertexCountsAttr().Set([3]*len(m.faces)); um.GetFaceVertexIndicesAttr().Set(m.faces.ravel().tolist())
    um.GetSubdivisionSchemeAttr().Set("none")
    um.GetPrim().CreateAttribute("primvars:weldgen_object_id", Sdf.ValueTypeNames.Int).Set(int(s.object_id))
cam = UsdGeom.Camera.Define(stage, "/World/cam"); ha = 20.955
cam.GetFocalLengthAttr().Set(K[0,0]*ha/W); cam.GetHorizontalApertureAttr().Set(ha); cam.GetVerticalApertureAttr().Set(ha*H/W)
cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 50.0))
T_usd = T.copy(); T_usd[:3,3] *= MM; T_usd = T_usd @ np.diag([1,-1,-1,1])
xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(Gf.Matrix4d(T_usd.T.tolist()))
rp = rep.create.render_product("/World/cam", (W, H))
dep = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"); dep.attach(rp)
rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach(rp)
for i in range(3):
    t0 = time.time(); rep.orchestrator.step(rt_subframes=16); dt = time.time()-t0
d = dep.get_data().astype(np.float64) / MM            # mm along optical axis
valid = np.isfinite(d) & (d > 0)
print(f"RENDER {W}x{H} frame_s={dt:.3f} valid_frac={valid.mean():.4f} z_range={d[valid].min():.2f}..{d[valid].max():.2f} mm", flush=True)

union = trimesh.util.concatenate(meshes)
def box_surface_dist(p):
    _, dist, _ = trimesh.proximity.closest_point(union, p)
    return dist

R, tcam = T[:3,:3], T[:3,3]
vv, uu = np.nonzero(valid)
for name, off in (("pixel_centre+0.5", 0.5), ("pixel_corner+0.0", 0.0)):
    u = uu + off; v = vv + off; zc = d[vv, uu]
    pc = np.stack([(u-K[0,2])/K[0,0]*zc, (v-K[1,2])/K[1,1]*zc, zc], 1)
    pw = pc @ R.T + tcam
    r = box_surface_dist(pw)
    print(f"RESIDUAL[{name}] n={len(r)} median={np.median(r):.4f} p95={np.percentile(r,95):.4f} p99={np.percentile(r,99):.4f} max={r.max():.3f} mm  frac>0.25mm={(r>0.25).mean():.4f}", flush=True)

# coverage: tier-1 visible points should land on valid pixels with matching depth
xyz = z["xyz"].astype(np.float64); vis = z["visible_from_cam"]
uv, zp = project(xyz, T, K)
inimg = (uv[:,0]>=0)&(uv[:,0]<W)&(uv[:,1]>=0)&(uv[:,1]<H)&(zp>0)
j = np.clip(uv[:,0].astype(int),0,W-1); i = np.clip(uv[:,1].astype(int),0,H-1)
zr = d[i, j]; ok = inimg & np.isfinite(zr)
dz = zr - zp
agree = ok & (np.abs(dz) < 1.0)
print(f"COVERAGE tier1 visible={vis.sum()} in_image={(inimg&vis).sum()}  visible&depth_agrees(<1mm)={(agree&vis).sum()/max((inimg&vis).sum(),1):.4f}  invisible_in_image={(inimg&~vis).sum()} invisible&depth_agrees={(agree&inimg&~vis).sum()/max((inimg&~vis).sum(),1):.4f}", flush=True)
print(f"DZ visible: median={np.median(dz[ok&vis]):.4f} p95|dz|={np.percentile(np.abs(dz[ok&vis]),95):.4f}", flush=True)
np.savez(sys.argv[2] if len(sys.argv)>2 else "/tmp/gate_proto.npz", depth_mm=d.astype(np.float32), rgb=rgb.get_data())
print("TOTAL", round(time.time()-t,1), flush=True)
app.close()
