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


def _mesh_prim(stage, path: str, mesh, label: int, material):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    um = UsdGeom.Mesh.Define(stage, path)
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

    dome = {"texture": None, "intensity": 350.0, **(dome or {})}
    dl = UsdLux.DomeLight.Define(stage, WORLD + "/dome")
    dl.GetIntensityAttr().Set(float(dome["intensity"]))
    if dome["texture"]:
        dl.GetTextureFileAttr().Set(str(dome["texture"]))
    key = {"intensity": 1200.0, "angle_deg": 2.0, "rotation_xyz_deg": (-40.0, 25.0, 0.0), **(key_light or {})}
    kl = UsdLux.DistantLight.Define(stage, WORLD + "/key")
    kl.GetIntensityAttr().Set(float(key["intensity"])); kl.GetAngleAttr().Set(float(key["angle_deg"]))
    UsdGeom.Xformable(kl).AddRotateXYZOp().Set(Gf.Vec3f(*key["rotation_xyz_deg"]))

    default_mat = _pbr(stage, WORLD + "/mat_default")
    for p, m in zip(parts, meshes):
        spec = material_for(p) if material_for else None
        mat = _pbr(stage, f"{WORLD}/mat_{p.id}", **spec) if spec else default_mat
        path = f"{WORLD}/part_{p.id}"
        _mesh_prim(stage, path, m, object_label(p.object_id), mat)
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
