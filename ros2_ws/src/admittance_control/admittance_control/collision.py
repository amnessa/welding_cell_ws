"""Collision model for the pen-marking motion: arm + tool against parts + table.

Milestone 2 of notes/pen_marking_plan.md (option A1). Everything is closed-form
geometry on primitives, so it runs inside the RRT's validity check at planning rates
and needs nothing beyond numpy:

    robot   six capsules hung on the UR5e joint frames (`kinematics.ur5e_link_frames`),
            the upper-arm and forearm tubes displaced off the joint line by the URDF's
            shoulder/elbow offsets (0.138 / 0.007 m), radii with margin
    tool    the primitives of `tool_model.ToolModel` (capsules and boxes) placed by FK
    scene   the registered parts as boxes (mode A's slabs at their poses, mm -> m) and
            the table as a horizontal plane below which nothing may go
    query   `in_collision(q)`, `min_distance(q)` (with the pair that sets it),
            `is_valid(q)` for the RRT (joint limits AND clearance)

Distances: capsule-capsule (segment-segment, closed form), capsule-box (point-to-box
is convex along a segment, so a golden-section search on the segment parameter is
exact to the tolerance), box-box (separating-axis test after inflating the scene box by
the clearance; an intersection test, not a distance, so `min_distance` reports 0 or
"clear" for that pair), primitive-plane (lowest point).

What it does NOT do, stated: full self-collision. The branch lock keeps the arm
elbow-up, and the tool primitives are checked against the base column and the upper
arm, which is where a wrist-mounted pen can meet the robot itself. Fixtures are
absent (magnets); add a box to `scene_boxes` the day one appears.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

from .kinematics import _in_limits, ur5e_link_frames

# UR5e link radii (m), tube diameters ~90 / 75 mm plus margin
UR5E_RADII = {"base": 0.075, "shoulder": 0.07, "upper_arm": 0.06, "forearm": 0.05,
              "wrist": 0.05}
SHOULDER_OFFSET = 0.138           # upper-arm tube off the joint line, along the lift axis
ELBOW_OFFSET = 0.007              # forearm tube off the joint line, along the elbow axis


# ---------------------------------------------------------------- primitives ----------
def capsule(name: str, p0, p1, radius: float, group: str = "robot") -> dict[str, Any]:
    return {"name": name, "type": "capsule", "p0": np.asarray(p0, float),
            "p1": np.asarray(p1, float), "radius": float(radius), "group": group}


def box(name: str, centre, R, half, group: str = "scene") -> dict[str, Any]:
    return {"name": name, "type": "box", "centre": np.asarray(centre, float),
            "R": np.asarray(R, float), "half": np.asarray(half, float), "group": group}


def boxes_from_parts(parts: Sequence[Any], scale: float = 1e-3) -> list[dict[str, Any]]:
    """Mode A's posed slabs (mm, `dims_mm`, `T_world_part`) as scene boxes in metres."""
    out = []
    for p in parts:
        T = np.asarray(p.T_world_part, float)
        out.append(box(f"part_{p.id}", T[:3, 3] * scale, T[:3, :3],
                       np.asarray(p.dims_mm, float) * 0.5 * scale))
    return out


# ---------------------------------------------------------------- distances -----------
def _seg_seg_distance(p0, p1, q0, q1) -> float:
    """Closest distance between segments p0p1 and q0q1 (Ericson, Real-Time CD 5.1.9)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(p0 - q0))
    if a <= eps:
        s, t = 0.0, float(np.clip(f / e, 0.0, 1.0))
    else:
        c = d1 @ r
        if e <= eps:
            t, s = 0.0, float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = float(np.clip((b * f - c * e) / denom, 0.0, 1.0)) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t, s = 1.0, float(np.clip((b - c) / a, 0.0, 1.0))
    return float(np.linalg.norm((p0 + d1 * s) - (q0 + d2 * t)))


def _point_box_distance(p, bx) -> float:
    local = bx["R"].T @ (p - bx["centre"])
    d = np.maximum(np.abs(local) - bx["half"], 0.0)
    return float(np.linalg.norm(d))


def _seg_box_distance(p0, p1, bx, tol: float = 1e-4) -> float:
    """Exact to `tol` in the segment parameter: distance to a convex set is convex
    along a line, so golden-section search finds its minimum."""
    f = lambda t: _point_box_distance(p0 + (p1 - p0) * t, bx)
    lo, hi = 0.0, 1.0
    g = (np.sqrt(5.0) - 1.0) / 2.0
    c, d = hi - g * (hi - lo), lo + g * (hi - lo)
    fc, fd = f(c), f(d)
    while hi - lo > tol:
        if fc < fd:
            hi, d, fd = d, c, fc
            c = hi - g * (hi - lo); fc = f(c)
        else:
            lo, c, fc = c, d, fd
            d = lo + g * (hi - lo); fd = f(d)
    return min(fc, fd, f(0.0), f(1.0))


def _boxes_intersect(a, b, inflate: float = 0.0) -> bool:
    """Separating-axis test between two oriented boxes (`b` grown by `inflate`)."""
    Ra, Rb = a["R"], b["R"]
    ha, hb = a["half"], b["half"] + inflate
    t = Rb.T @ (a["centre"] - b["centre"])          # a's centre in b's frame
    R = Rb.T @ Ra                                   # a's axes in b's frame
    absR = np.abs(R) + 1e-9
    axes = []
    for i in range(3):                              # b's axes
        ra = ha @ absR[i, :]; rb = hb[i]
        if abs(t[i]) > ra + rb:
            return False
    for j in range(3):                              # a's axes
        ra = ha[j]; rb = hb @ absR[:, j]
        if abs(t @ R[:, j]) > ra + rb:
            return False
    for i in range(3):                              # cross products
        for j in range(3):
            axis = np.cross(np.eye(3)[i], R[:, j])
            n = np.linalg.norm(axis)
            if n < 1e-9:
                continue
            axis /= n
            ra = ha @ np.abs(R.T @ axis)                # a's extent along the axis (b frame)
            rb = hb @ np.abs(axis)
            if abs(t @ axis) > ra + rb:
                return False
    return True


def _box_corners(bx) -> np.ndarray:
    s = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    return (s * bx["half"]) @ bx["R"].T + bx["centre"]


def primitive_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    """Surface distance between two primitives (0 when they overlap).
    Box-box is an intersection test: 0 if they touch, else +inf ("clear")."""
    if a["type"] == "capsule" and b["type"] == "capsule":
        return max(0.0, _seg_seg_distance(a["p0"], a["p1"], b["p0"], b["p1"])
                   - a["radius"] - b["radius"])
    if a["type"] == "capsule" and b["type"] == "box":
        return max(0.0, _seg_box_distance(a["p0"], a["p1"], b) - a["radius"])
    if a["type"] == "box" and b["type"] == "capsule":
        return primitive_distance(b, a)
    return 0.0 if _boxes_intersect(a, b) else np.inf


def lowest_point_z(prim: dict[str, Any]) -> float:
    if prim["type"] == "capsule":
        return float(min(prim["p0"][2], prim["p1"][2]) - prim["radius"])
    return float(_box_corners(prim)[:, 2].min())


# ---------------------------------------------------------------- the model -----------
def ur5e_capsules(q: np.ndarray, radii: dict[str, float] | None = None) -> list[dict[str, Any]]:
    """The arm as capsules in base_link at configuration `q`."""
    r = {**UR5E_RADII, **(radii or {})}
    F = ur5e_link_frames(q)
    o = {k: T[:3, 3] for k, T in F.items()}
    z_lift = F["lift"][:3, 2]                 # shoulder_lift axis
    z_elbow = F["elbow"][:3, 2]               # elbow axis
    up0, up1 = o["lift"] + SHOULDER_OFFSET * z_lift, o["elbow"] + SHOULDER_OFFSET * z_lift
    fa0, fa1 = o["elbow"] + ELBOW_OFFSET * z_elbow, o["wrist_1"] + ELBOW_OFFSET * z_elbow
    return [
        capsule("base", [0, 0, 0.0], [0, 0, 0.1625], r["base"]),
        capsule("shoulder", o["shoulder"], o["lift"] + SHOULDER_OFFSET * z_lift, r["shoulder"]),
        capsule("upper_arm", up0, up1, r["upper_arm"]),
        capsule("forearm", fa0, fa1, r["forearm"]),
        capsule("wrist_1", o["wrist_1"], o["wrist_2"], r["wrist"]),
        capsule("wrist_2", o["wrist_2"], o["wrist_3"], r["wrist"]),
    ]


@dataclass
class CollisionModel:
    """Arm + tool against the scene, with a clearance margin.

    `tool` is a `tool_model.ToolModel` (or None for the bare arm); `scene_boxes` are
    boxes in base_link (metres); `table_z` is the plane below which nothing may go
    (None to skip); `clearance` is the required gap in metres. `self_pairs` lists the
    robot capsules the tool is checked against.
    """
    tool: Any = None
    scene_boxes: list[dict[str, Any]] = field(default_factory=list)
    table_z: float | None = None
    clearance: float = 0.01
    radii: dict[str, float] | None = None
    self_pairs: tuple[str, ...] = ("base", "shoulder", "upper_arm")
    table_exempt: tuple[str, ...] = ("base", "shoulder")      # they stand on it

    def bodies(self, q: np.ndarray) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        arm = ur5e_capsules(q, self.radii)
        tool = []
        if self.tool is not None:
            T = ur5e_link_frames(q)["tool0"]
            for p in self.tool.primitives_in(T):
                tool.append({**p, "group": "tool"})
        return arm, tool

    def pair_distances(self, q: np.ndarray) -> list[tuple[float, str, str]]:
        """Every checked pair as (distance, name_a, name_b), unsorted."""
        arm, tool = self.bodies(q)
        out: list[tuple[float, str, str]] = []
        for a in arm + tool:
            for s in self.scene_boxes:
                out.append((primitive_distance(a, s), a["name"], s["name"]))
            if self.table_z is not None and a["name"] not in self.table_exempt:
                out.append((max(0.0, lowest_point_z(a) - self.table_z), a["name"], "table"))
        arm_by_name = {a["name"]: a for a in arm}
        for t in tool:
            for n in self.self_pairs:
                out.append((primitive_distance(t, arm_by_name[n]), t["name"], n))
        return out

    def min_distance(self, q: np.ndarray) -> tuple[float, str, str]:
        pairs = self.pair_distances(q)
        if not pairs:
            return np.inf, "", ""
        return min(pairs, key=lambda x: x[0])

    def in_collision(self, q: np.ndarray) -> bool:
        return self.min_distance(q)[0] < self.clearance

    def is_valid(self, q: np.ndarray) -> bool:
        """The RRT's configuration test: joint limits AND clearance."""
        return _in_limits(np.asarray(q, float)) and not self.in_collision(q)

    def validity_fn(self) -> Callable[[np.ndarray], bool]:
        return self.is_valid

    def report(self, q: np.ndarray, worst: int = 5) -> str:
        pairs = sorted(self.pair_distances(q), key=lambda x: x[0])[:worst]
        d, a, b = pairs[0] if pairs else (np.inf, "", "")
        head = (f"{'COLLISION' if d < self.clearance else 'clear'}: min {d * 1000:.1f} mm "
                f"({a} x {b}), clearance {self.clearance * 1000:g} mm")
        return head + "".join(f"\n  {x * 1000:7.1f} mm  {n1} x {n2}" for x, n1, n2 in pairs)
