"""The pen tool on the flange, as geometry: tip, collision envelope, camera.

One description (`config/pen_tool.json`, frame tool0, metres) read by everything that
needs the tool - the marking node (where is the tip, how far to stand off, when is it
touching) and the collision model (what must not hit the parts). Nothing here imports
ROS; the marker node wraps it for RViz.

    tool = load_tool_model()                 # the package config
    tool.tip_tool0                           # (3,) pen tip in tool0
    tool.primitives_tool0()                  # capsules + boxes in tool0, camera included
    tool.primitives_in(T_base_tool0)         # the same, placed by FK
    tool.tip_in(T_base_tool0)

The camera body is a box in tool0 measured on the bench like the rest of the envelope
(deriving it from the calibrated sensor origin put it in the wrong place: where the body
sits around the colour sensor is not something the calibration knows). The hand-eye
calibration (`T_tcp_to_cam.npy`, tool0 -> camera_color_optical_frame) is loaded as
`T_tool0_cam` for reference only: the marker node draws its origin, and it has to sit on
the lens face.

Primitives are plain dicts so the collision module can consume them without a class
hierarchy: `{"type": "capsule", "p0", "p1", "radius"}` and
`{"type": "box", "centre", "R" (3x3), "half"}`, all in the frame stated.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "pen_tool.json"


def _as_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def transform_primitive(prim: dict[str, Any], T: np.ndarray) -> dict[str, Any]:
    """The primitive expressed in the frame `T` maps its current frame into."""
    R, t = T[:3, :3], T[:3, 3]
    out = dict(prim)
    if prim["type"] == "capsule":
        out["p0"] = (R @ np.asarray(prim["p0"], float) + t)
        out["p1"] = (R @ np.asarray(prim["p1"], float) + t)
    elif prim["type"] == "box":
        out["centre"] = (R @ np.asarray(prim["centre"], float) + t)
        out["R"] = R @ np.asarray(prim.get("R", np.eye(3)), float)
    else:
        raise ValueError(f"unknown primitive type {prim['type']!r}")
    return out


@dataclass
class ToolModel:
    version: str
    parent_frame: str
    tip_tool0: np.ndarray
    touch_force_n: float
    standoff_m: float
    primitives: list[dict[str, Any]]                 # in tool0, camera body included
    T_tool0_cam: np.ndarray | None                   # tool0 -> camera optical frame (reference)
    camera_frame: str | None
    source: dict[str, Any] = field(default_factory=dict)

    # -- geometry ------------------------------------------------------------------
    def primitives_tool0(self) -> list[dict[str, Any]]:
        return [dict(p) for p in self.primitives]

    def primitives_in(self, T_frame_tool0: np.ndarray) -> list[dict[str, Any]]:
        return [transform_primitive(p, T_frame_tool0) for p in self.primitives]

    def tip_in(self, T_frame_tool0: np.ndarray) -> np.ndarray:
        return T_frame_tool0[:3, :3] @ self.tip_tool0 + T_frame_tool0[:3, 3]

    def T_tool0_for_tip(self, tip_frame: np.ndarray, axis_frame: np.ndarray,
                        roll_rad: float = 0.0) -> np.ndarray:
        """The tool0 pose that puts the pen tip at `tip_frame` with the pen (tool0 +Z)
        pointing along `axis_frame` (the direction the tip travels INTO the joint, i.e.
        minus the seam's approach axis), rolled by `roll_rad` about that axis.

        The free roll is the redundancy of an axisymmetric tool: this is the parameter
        the reachability search spends on elbow-up and joint-limit distance.
        """
        z = np.asarray(axis_frame, float)
        z = z / np.linalg.norm(z)
        ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x = np.cross(ref, z); x /= np.linalg.norm(x)
        y = np.cross(z, x)
        c, s = np.cos(roll_rad), np.sin(roll_rad)
        R = np.column_stack([c * x + s * y, -s * x + c * y, z])
        origin = np.asarray(tip_frame, float) - R @ self.tip_tool0
        return _as_T(R, origin)

    # -- reporting -----------------------------------------------------------------
    def describe(self) -> str:
        lines = [f"{self.version} in {self.parent_frame}: tip at {np.round(self.tip_tool0, 4)} m, "
                 f"touch {self.touch_force_n:g} N, standoff {self.standoff_m * 1000:g} mm"]
        for p in self.primitives:
            if p["type"] == "capsule":
                lines.append(f"  capsule {p['name']:<14} {np.round(p['p0'], 3)} -> "
                             f"{np.round(p['p1'], 3)} r={p['radius'] * 1000:g} mm")
            else:
                lines.append(f"  box     {p['name']:<14} centre {np.round(p['centre'], 3)} "
                             f"half {np.round(p['half'], 3)}")
        if self.T_tool0_cam is not None:
            lines.append(f"  camera  {self.camera_frame} origin {np.round(self.T_tool0_cam[:3, 3], 4)}"
                         f" optical axis {np.round(self.T_tool0_cam[:3, 2], 3)} (tool0)")
        return "\n".join(lines)


def load_tool_model(path: str | Path | None = None,
                    extrinsic_path: str | Path | None = None) -> ToolModel:
    """Read `pen_tool.json` (+ the calibration it names) into a `ToolModel`.

    `extrinsic_path` overrides the config's `camera.extrinsic_path` (the launch file
    resolves the calibration the same way for the TF publisher). A missing calibration
    file is not an error - the camera is then simply absent from the envelope, and
    `describe()` says so - because the tool must load on a machine without the
    notebooks folder; the marking node refuses to move without it.
    """
    cfg_path = Path(path) if path else DEFAULT_CONFIG
    cfg = json.loads(cfg_path.read_text())
    prims: list[dict[str, Any]] = []
    for p in cfg["primitives"]:
        q = {"name": p["name"], "type": p["type"]}
        if p["type"] == "capsule":
            q.update(p0=np.asarray(p["p0"], float), p1=np.asarray(p["p1"], float),
                     radius=float(p["radius"]))
        elif p["type"] == "box":
            q.update(centre=np.asarray(p["centre"], float), half=np.asarray(p["half"], float),
                     R=np.asarray(p.get("R", np.eye(3).tolist()), float))
        else:
            raise ValueError(f"{cfg_path}: unknown primitive type {p['type']!r}")
        prims.append(q)

    T_cam = None
    cam_frame = None
    cam = cfg.get("camera")
    if cam:
        cam_frame = cam.get("frame", "camera_color_optical_frame")
        ep = Path(extrinsic_path) if extrinsic_path else Path(cam["extrinsic_path"])
        if not ep.is_absolute():
            ep = (cfg_path.parent.parent / ep)
        if ep.exists():
            T_cam = np.asarray(np.load(ep), float).reshape(4, 4)
    return ToolModel(version=str(cfg.get("version", "pen_tool")),
                     parent_frame=str(cfg.get("parent_frame", "tool0")),
                     tip_tool0=np.asarray(cfg["pen_tip_m"], float),
                     touch_force_n=float(cfg.get("touch_force_n", 1.5)),
                     standoff_m=float(cfg.get("standoff_m", 0.035)),
                     primitives=prims, T_tool0_cam=T_cam, camera_frame=cam_frame,
                     source={"config": str(cfg_path),
                             "extrinsic": None if T_cam is None else str(ep)})
