"""Build the USD scene for one weldgen scene — `pxr` is imported inside functions only.

The stage is metres. Workpieces are the exact `geom` meshes (`subdivisionScheme = none`,
so the renderer's ray cast hits the tier-1 surface and nothing else). The environment layer
(`dataset_plan.md` Phase 8) is a substrate plane at the lowest workpiece vertex - tier 1
rests every assembly flat on the working surface, so the plane never occludes a joint -
and a dome light. Materials are a hook: M2 binds one neutral PBR to every part; M5 draws
alloys, surface conditions, lights and the substrate photo from the render substream.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .conventions import (LABEL_ENV, M_PER_MM, object_label, usd_camera_matrix,
                          usd_intrinsics)

WORLD = "/World"
CAMERA_PATH = "/World/cam"
SUBSTRATE_PATH = "/World/substrate"


@dataclass
class StageHandles:
    """What the renderer needs after the build: label per prim path, the camera op, plane."""
    label_by_path: dict = field(default_factory=dict)      # prim path -> mask_object value
    z_table_mm: float = 0.0
    substrate_half_m: float = 2.0
    _cam_op: object = None

    def set_camera(self, K, T_world_cam, width: int, height: int) -> None:
        from pxr import Gf
        from pxr import UsdGeom  # noqa: F401  (import guard: pxr must be live)
        cam = self._cam
        f, ha, va = usd_intrinsics(K, width, height)
        cam.GetFocalLengthAttr().Set(f)
        cam.GetHorizontalApertureAttr().Set(ha)
        cam.GetVerticalApertureAttr().Set(va)
        self._cam_op.Set(Gf.Matrix4d(usd_camera_matrix(T_world_cam).T.tolist()))


def planar_st(mesh, T_world_part, tile_mm: float) -> np.ndarray:
    """faceVarying texture coordinates: each triangle is projected along the dominant axis of
    its normal in the PART's local frame, in units of `tile_mm` per texture repeat. Boxes,
    tubes and bands all get a seam-free-enough mapping for a 1 m surface set; the draw adds a
    rotation and an offset so no two parts share the same patch of the map."""
    T = np.asarray(T_world_part, float); R, t = T[:3, :3], T[:3, 3]
    tri = (np.asarray(mesh.triangles, float) - t) @ R                    # (F, 3, 3) local
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); ax = np.argmax(np.abs(n), axis=1)
    others = np.array([[1, 2], [0, 2], [0, 1]])[ax]                        # (F, 2)
    st = np.take_along_axis(tri, others[:, None, :].repeat(3, 1), axis=2)  # (F, 3, 2)
    return (st / float(tile_mm)).reshape(-1, 2)


def _textured_pbr(stage, path: str, rec: dict, assets_dir: str, uv_rotation_deg: float = 0.0,
                  uv_offset=(0.0, 0.0)):
    """UsdPreviewSurface driven by a CC0 surface set (colour x F0 blend, roughness x scale + bias,
    normal map) - `materials-1.0`, see render/materials.py."""
    from pxr import Gf, Sdf, UsdShade
    d = Path(assets_dir)
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/pbr"); sh.CreateIdAttr("UsdPreviewSurface")
    rd = UsdShade.Shader.Define(stage, path + "/st"); rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    xf = UsdShade.Shader.Define(stage, path + "/uv"); xf.CreateIdAttr("UsdTransform2d")
    xf.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
    xf.CreateInput("rotation", Sdf.ValueTypeNames.Float).Set(float(uv_rotation_deg))
    xf.CreateInput("translation", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*[float(v) for v in uv_offset]))
    def tex(name, file, colorspace, scale4, bias4):
        t = UsdShade.Shader.Define(stage, f"{path}/{name}"); t.CreateIdAttr("UsdUVTexture")
        t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(d / file))
        t.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(colorspace)
        t.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat"); t.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(xf.ConnectableAPI(), "result")
        t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*scale4)); t.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*bias4))
        t.CreateOutput("rgb", Sdf.ValueTypeNames.Float3); t.CreateOutput("r", Sdf.ValueTypeNames.Float)
        return t
    f0 = rec["f0"]; b = float(rec["albedo_blend"])
    files = rec["files"]
    if "color" in files:   # colour = f0 * ((1 - b) + b * map)   ->  scale = f0*b, bias = f0*(1-b)
        tc = tex("color", files["color"], "sRGB", [c * b for c in f0] + [1.0], [c * (1 - b) for c in f0] + [0.0])
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tc.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*f0))
    if "roughness" in files:
        rs, rb = float(rec["roughness_scale"]), float(rec["roughness_bias"])
        tr = tex("rough", files["roughness"], "raw", [rs, rs, rs, 1.0], [rb, rb, rb, 0.0])
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(tr.ConnectableAPI(), "r")
    else:
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    if "normalgl" in files:
        tn = tex("normal", files["normalgl"], "raw", [2.0, 2.0, 2.0, 1.0], [-1.0, -1.0, -1.0, 0.0])
        sh.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(tn.ConnectableAPI(), "rgb")
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(rec["metallic"]))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _pbr(stage, path: str, base=(0.45, 0.45, 0.47), metallic=1.0, roughness=0.45, texture: str | None = None,
         st_scale: tuple[float, float] | None = None):
    from pxr import Gf, Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/pbr")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    if texture is None:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*base))
    else:
        rd = UsdShade.Shader.Define(stage, path + "/st"); rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, path + "/tex"); tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture))
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("mirror")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("mirror")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _mesh_prim(stage, path: str, mesh, label: int, material, st: np.ndarray | None = None):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    um = UsdGeom.Mesh.Define(stage, path)
    if st is not None:
        pv = UsdGeom.PrimvarsAPI(um.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        pv.Set([Gf.Vec2f(float(a), float(b)) for a, b in st])
    um.GetPointsAttr().Set([Gf.Vec3f(*(v * M_PER_MM)) for v in mesh.vertices])
    um.GetFaceVertexCountsAttr().Set([3] * len(mesh.faces))
    um.GetFaceVertexIndicesAttr().Set(mesh.faces.ravel().tolist())
    um.GetSubdivisionSchemeAttr().Set("none")
    um.GetPrim().CreateAttribute("weldgen:label", Sdf.ValueTypeNames.Int).Set(int(label))
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(um.GetPrim()).Bind(material)
    return um


def build_stage(parts, meshes, *, substrate: dict | None = None, dome: dict | None = None,
                key_light: dict | None = None, material_for=None) -> StageHandles:
    """Populate the live stage. `parts`/`meshes` from `geom.from_object` / `.mesh()`.

    substrate: {"photo": path | None, "span_m": (Lx, Ly), "rotation_rad": a, "roughness": r,
                "half_m": 2.0} or None for no plane (the void - never for a dataset view).
    dome: {"texture": path | None, "intensity": float}; key_light: {"intensity", "angle_deg",
    "rotation_xyz_deg"}. material_for(part) -> dict(base, metallic, roughness) or None.
    """
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdLux

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, WORLD)
    h = StageHandles()

    dome = {"texture": None, "intensity": 350.0, "rotation_deg": 0.0, **(dome or {})}
    dl = UsdLux.DomeLight.Define(stage, WORLD + "/dome")
    dl.GetIntensityAttr().Set(float(dome["intensity"]))
    if dome["texture"]:
        dl.GetTextureFileAttr().Set(str(dome["texture"]))
    if dome["rotation_deg"]:
        UsdGeom.Xformable(dl).AddRotateZOp().Set(float(dome["rotation_deg"]))
    key = {"intensity": 1200.0, "angle_deg": 2.0, "rotation_xyz_deg": (-40.0, 25.0, 0.0), **(key_light or {})}
    kl = UsdLux.DistantLight.Define(stage, WORLD + "/key")
    kl.GetIntensityAttr().Set(float(key["intensity"])); kl.GetAngleAttr().Set(float(key["angle_deg"]))
    UsdGeom.Xformable(kl).AddRotateXYZOp().Set(Gf.Vec3f(*key["rotation_xyz_deg"]))

    default_mat = _pbr(stage, WORLD + "/mat_default")
    for p, m in zip(parts, meshes):
        spec = material_for(p) if material_for else None
        st = None
        if spec and "rule" in spec:                          # materials-1.0 recipe with a surface set
            mat = _textured_pbr(stage, f"{WORLD}/mat_{p.id}", spec, spec["assets_dir"],
                                spec.get("uv_rotation_deg", 0.0), spec.get("uv_offset", (0.0, 0.0)))
            st = planar_st(m, p.T_world_part, spec["tile_mm"])
        elif spec:
            mat = _pbr(stage, f"{WORLD}/mat_{p.id}", **spec)
        else:
            mat = default_mat
        path = f"{WORLD}/part_{p.id}"
        _mesh_prim(stage, path, m, object_label(p.object_id), mat, st)
        h.label_by_path[path] = object_label(p.object_id)

    h.z_table_mm = float(min(m.vertices[:, 2].min() for m in meshes))
    if substrate is not None:
        sub = {"photo": None, "span_m": (1.5, 1.0), "rotation_rad": 0.0, "roughness": 0.7, "half_m": 2.0, **substrate}
        half = float(sub["half_m"]); h.substrate_half_m = half
        cxy = np.mean([m.vertices[:, :2].mean(0) for m in meshes], 0) * M_PER_MM
        zt = h.z_table_mm * M_PER_MM
        pl = UsdGeom.Mesh.Define(stage, SUBSTRATE_PATH)
        corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
        pl.GetPointsAttr().Set([Gf.Vec3f(cxy[0] + x, cxy[1] + y, zt) for x, y in corners])
        pl.GetFaceVertexCountsAttr().Set([4]); pl.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
        pl.GetSubdivisionSchemeAttr().Set("none")
        Lx, Ly = sub["span_m"]; a = float(sub["rotation_rad"]); ca, sa = np.cos(a), np.sin(a)
        st = UsdGeom.PrimvarsAPI(pl.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set([Gf.Vec2f((ca * x - sa * y) / Lx + 0.5, (sa * x + ca * y) / Ly + 0.5) for x, y in corners])
        mat = _pbr(stage, WORLD + "/mat_substrate", base=(0.5, 0.45, 0.4), metallic=0.0,
                   roughness=sub["roughness"], texture=sub["photo"])
        from pxr import UsdShade
        UsdShade.MaterialBindingAPI.Apply(pl.GetPrim()).Bind(mat)
        h.label_by_path[SUBSTRATE_PATH] = LABEL_ENV

    cam = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 50.0))
    xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder()
    h._cam = cam; h._cam_op = xf.AddTransformOp()
    return h


def clear_stage() -> None:
    """Remove /World so the next scene starts clean in the same app."""
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(WORLD):
        stage.RemovePrim(WORLD)
