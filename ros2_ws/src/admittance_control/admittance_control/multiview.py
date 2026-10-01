"""Multi-view close-range pose refinement: the VIEW PLANNER (step 2 of
notes/multiview_refine_plan.md, design D2). Pure numpy plus the package's IK, collision
model and transit planner; no ROS.

After the parts are registered and saved, the camera looks again at the centre of the
assembly from K close views (0.4 m instead of ~0.5 m from the scan home, from several
directions), so that the per-view camera errors - the extrinsic's translation, which
turns with the wrist, and the depth bias, along each view's own line of sight - average
out in a joint refinement (step 3). This module decides WHERE to look from:

1. the seam region: surface points of each part 1-30 mm from ANOTHER part's box - the
   faces either side of every place two parts meet, i.e. where the weld roots will be
   (the seams themselves are computed later, by welding_points). Other surface points
   count a little (`background_weight`), the touching faces themselves not at all;
2. candidate camera poses: look-at poses at `distance_m` from the target (the centre of
   the parts' combined box), every `azimuth_step_deg`, at each of `elevations_deg`, at
   each roll about the optical axis (`roll_step_deg`);
3. visibility of the targets from each candidate - depth range (D435i minimum 0.28 m),
   field of view, incidence under `max_incidence_deg`, no part box in the way - first,
   because it is cheap; only candidates that see something get the IK;
4. feasibility: IK on the locked elbow-up branch (`solve_on_branch`), wrist_2 and the
   elbow away from their singularities (the elbow straight is the workspace edge: a
   far-side view at 0.78 m from the base came out at elbow -0.16 rad), and the tool
   (camera body included) clear of the parts by the transit clearance (20 mm). Of the
   rolls of a chosen direction, the one with the least joint travel from home is taken;
5. a greedy pick of `n_views`: a target point's 1st, 2nd, 3rd sighting are worth
   `repeat_weights` (1, 0.5, 0.25), so the second view of a face from another side still
   pays, and picks closer than `min_separation_deg` in viewing direction are excluded;
6. the order (nearest neighbour in joint space from the current joints) and the
   collision-free transits between the views, ending with the way home (unwinding the
   wrist, `transit_path(..., unwrap=False)`).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

import numpy as np

from .collision import CollisionModel
from .icp import load_ply_mesh, sample_mesh_surface
from .kinematics import JOINT_LIMITS
from .marking import transit_model, transit_path
from .tack_reach import MarkingConfig, _unwrap_to, solve_on_branch


@dataclass
class ViewConfig:
    distance_m: float = 0.40              # camera to target (D435i minimum range 0.28 m)
    elevations_deg: tuple[float, ...] = (45.0, 60.0)   # above the table plane
    azimuth_step_deg: float = 30.0
    roll_step_deg: float = 90.0           # about the optical axis
    n_views: int = 4
    min_views: int = 2                    # fewer feasible views -> the plan fails
    min_separation_deg: float = 30.0      # between the viewing directions of two picks
    max_incidence_deg: float = 60.0       # surface normal vs the ray to the camera
    min_depth_m: float = 0.28
    max_depth_m: float = 0.60
    hfov_deg: float = 69.0                # D435i colour at 1280x720 (depth is aligned to it)
    vfov_deg: float = 42.0
    seam_band_m: float = 0.03             # a part's points this close to another part's box
    contact_gap_m: float = 0.001          # ... but farther than this (touching faces: hidden)
    background_weight: float = 0.1        # every other surface point
    repeat_weights: tuple[float, ...] = (1.0, 0.5, 0.25)
    occlusion_margin_m: float = 0.002     # the ray stops this short of its own surface
    n_surface_points: int = 6000          # per part, for the targets


# ---------------------------------------------------------------- geometry ------------
def look_at(cam_pos: np.ndarray, target: np.ndarray, roll_rad: float = 0.0) -> np.ndarray:
    """base <- camera optical frame (z forward, x right, y down) at `cam_pos` looking at
    `target`; roll 0 has the image 'down' along world -Z, roll turns about the optical axis."""
    cam_pos = np.asarray(cam_pos, float)
    z = np.asarray(target, float) - cam_pos
    z /= np.linalg.norm(z)
    down = np.array([0.0, 0.0, -1.0])
    y0 = down - (down @ z) * z
    if np.linalg.norm(y0) < 1e-6:                       # looking straight down: any 'down'
        y0 = np.array([1.0, 0.0, 0.0]) - z[0] * z
    y0 /= np.linalg.norm(y0)
    x0 = np.cross(y0, z)
    c, s = np.cos(roll_rad), np.sin(roll_rad)
    x, y = c * x0 + s * y0, -s * x0 + c * y0
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = cam_pos
    return T


def box_from_model(verts_m: np.ndarray, T: np.ndarray, name: str) -> dict[str, Any]:
    """The posed model's bounding box (in its own frame) as a collision `box` - exact for
    the plate parts."""
    lo, hi = verts_m.min(0), verts_m.max(0)
    return {"name": name, "type": "box", "centre": T[:3, :3] @ ((lo + hi) / 2) + T[:3, 3],
            "R": T[:3, :3].copy(), "half": (hi - lo) / 2, "group": "scene"}


def surfaces_from_models(objects: Sequence[tuple[str, np.ndarray]], models_dir,
                         n_points: int = 6000, rng: Optional[np.random.Generator] = None):
    """[(points, outward normals)] in the static frame and the matching boxes, one per
    saved object (model file name, 4x4 pose_static in metres)."""
    from pathlib import Path
    rng = rng or np.random.default_rng(0)
    surfaces, boxes = [], []
    for i, (name, T) in enumerate(objects):
        T = np.asarray(T, float)
        v, f = load_ply_mesh(Path(models_dir) / name)
        scale = 1e-3 if np.abs(v).max() > 5 else 1.0
        p, n = sample_mesh_surface(v, f, n_points, rng, return_normals=True)
        surfaces.append((p * scale @ T[:3, :3].T + T[:3, 3], n @ T[:3, :3].T))
        boxes.append(box_from_model(v * scale, T, f"part_{chr(ord('A') + i)}"))
    return surfaces, boxes


def _points_box_distance(pts: np.ndarray, bx: dict[str, Any]) -> np.ndarray:
    local = (pts - bx["centre"]) @ bx["R"]
    return np.linalg.norm(np.maximum(np.abs(local) - bx["half"], 0.0), axis=1)


def _segments_hit_box(origin: np.ndarray, ends: np.ndarray, bx: dict[str, Any]) -> np.ndarray:
    """Does the segment origin -> end[i] pass through the box? (slab test, vectorised)"""
    o = bx["R"].T @ (origin - bx["centre"])
    e = (ends - bx["centre"]) @ bx["R"]
    d = e - o
    t0 = np.zeros(len(ends))
    t1 = np.ones(len(ends))
    for k in range(3):
        dk = d[:, k]
        par = np.abs(dk) < 1e-12
        outside = par & (np.abs(o[k]) > bx["half"][k])
        with np.errstate(divide="ignore", invalid="ignore"):
            ta = (-bx["half"][k] - o[k]) / dk
            tb = (bx["half"][k] - o[k]) / dk
        lo = np.where(par, -np.inf, np.minimum(ta, tb))
        hi = np.where(par, np.inf, np.maximum(ta, tb))
        t0 = np.maximum(t0, lo)
        t1 = np.minimum(t1, hi)
        t1 = np.where(outside, -1.0, t1)
    return t0 <= t1


def seam_targets(surfaces, boxes, cfg: ViewConfig):
    """Target points, normals, weights and owning part index: weight 1 in the seam region
    (1-30 mm from another part's box), `background_weight` elsewhere; points on touching
    faces (within `contact_gap_m` of another part) are dropped."""
    pts, nrm, w, own = [], [], [], []
    for i, (p, n) in enumerate(surfaces):
        others = [b for j, b in enumerate(boxes) if j != i]
        d = (np.min([_points_box_distance(p, b) for b in others], axis=0)
             if others else np.full(len(p), np.inf))
        wi = np.where((d > cfg.contact_gap_m) & (d < cfg.seam_band_m), 1.0, cfg.background_weight)
        keep = d > cfg.contact_gap_m
        pts.append(p[keep]); nrm.append(n[keep]); w.append(wi[keep]); own.append(np.full(keep.sum(), i))
    return np.vstack(pts), np.vstack(nrm), np.concatenate(w), np.concatenate(own)


def visible(T_cam: np.ndarray, pts: np.ndarray, nrm: np.ndarray, boxes, cfg: ViewConfig
            ) -> np.ndarray:
    """Which points the camera at `T_cam` (base <- optical) sees: in depth range and in the
    field of view, facing it within `max_incidence_deg`, and with no part box in the way."""
    R, c = T_cam[:3, :3], T_cam[:3, 3]
    pc = (pts - c) @ R
    z = pc[:, 2]
    ok = (z > cfg.min_depth_m) & (z < cfg.max_depth_m)
    with np.errstate(divide="ignore", invalid="ignore"):
        ok &= np.abs(pc[:, 0] / z) <= np.tan(np.radians(cfg.hfov_deg) / 2)
        ok &= np.abs(pc[:, 1] / z) <= np.tan(np.radians(cfg.vfov_deg) / 2)
    ray = c - pts
    dist = np.linalg.norm(ray, axis=1)
    ok &= np.einsum("ij,ij->i", nrm, ray) >= np.cos(np.radians(cfg.max_incidence_deg)) * dist
    idx = np.flatnonzero(ok)
    if len(idx):
        ends = pts[idx] + cfg.occlusion_margin_m * ray[idx] / dist[idx, None]
        hit = np.zeros(len(idx), bool)
        for b in boxes:
            hit |= _segments_hit_box(c, ends, b)
        ok[idx[hit]] = False
    return ok


# ---------------------------------------------------------------- the plan ------------
@dataclass
class View:
    azimuth_deg: float
    elevation_deg: float
    roll_deg: float
    T_cam: np.ndarray                     # base <- camera optical frame
    T_tool0: np.ndarray
    q: Optional[np.ndarray] = None
    seen: Optional[np.ndarray] = None     # bool over the targets
    clearance_m: float = np.inf
    gain: float = 0.0                     # what it added in the greedy pick

    @property
    def direction(self) -> np.ndarray:
        return self.T_cam[:3, 2]


@dataclass
class ViewPlan:
    ok: bool
    reason: str
    target_m: np.ndarray
    views: list[View] = field(default_factory=list)          # in visiting order
    paths: list[list[np.ndarray]] = field(default_factory=list)   # q_start -> v1 -> ... -> vK
    home_path: Optional[list[np.ndarray]] = None
    n_candidates: int = 0
    n_feasible: int = 0                   # of the candidates the lazy pick checked
    rejected: dict[str, int] = field(default_factory=dict)
    coverage: dict[str, float] = field(default_factory=dict)

    def summary(self) -> str:
        lines = [f"{'VIEWS OK' if self.ok else 'VIEWS FAILED: ' + self.reason}: {len(self.views)} views "
                 f"of {self.n_feasible} feasible / {self.n_candidates} candidates, target "
                 f"{np.round(self.target_m * 1000).astype(int).tolist()} mm"]
        for k, v in enumerate(self.views):
            lines.append(f"  view {k + 1}: azimuth {v.azimuth_deg:5.0f}, elevation {v.elevation_deg:3.0f}, "
                         f"roll {v.roll_deg:4.0f} deg; sees {int(v.seen.sum()) if v.seen is not None else 0} "
                         f"targets, gain {v.gain:.0f}; clearance {v.clearance_m * 1000:.0f} mm")
        if self.coverage:
            lines.append("  of the seam region any candidate can see ({:.0%} of it): >=1 view {:.0%}, "
                         ">=2 views {:.0%}".format(self.coverage.get("seeable", 0.0),
                                                  self.coverage.get("seam_1", 0.0),
                                                  self.coverage.get("seam_2", 0.0)))
        if self.rejected:
            lines.append("  rejected: " + ", ".join(f"{k} {n}" for k, n in sorted(self.rejected.items())))
        return "\n".join(lines)


def _reject(rejected: dict[str, int], why: str) -> None:
    rejected[why] = rejected.get(why, 0) + 1


def nearest_in_limits(q: np.ndarray, ref: np.ndarray, margin: float = 0.1) -> np.ndarray:
    """Per joint, the 2*pi-equivalent of `q` nearest `ref` that is inside the joint
    limits (with `margin`): continuity without winding past them - a plain unwrap to the
    previous view put wrist_3 at -7.97 rad, beyond the UR's +-2*pi (2026-10-01)."""
    q = np.asarray(q, float).copy()
    for k, (lo, hi) in enumerate(JOINT_LIMITS):
        opts = [q[k] + 2 * np.pi * n for n in range(-3, 4)]
        opts = [x for x in opts if lo + margin <= x <= hi - margin]
        if opts:
            q[k] = min(opts, key=lambda x: abs(x - ref[k]))
    return q


def plan_views(surfaces, boxes, tool, model: CollisionModel, mcfg: MarkingConfig,
               q_start: np.ndarray, vcfg: Optional[ViewConfig] = None, plan_paths: bool = True,
               rng: Optional[np.random.Generator] = None) -> ViewPlan:
    """Choose, order and connect the views (module doc). `surfaces`/`boxes`: one per saved
    part (`surfaces_from_models`); `model`: the collision model with those parts in it."""
    vcfg = vcfg or ViewConfig()
    rng = rng or np.random.default_rng(0)
    q_start = np.asarray(q_start, float)
    allp = np.vstack([p for p, _ in surfaces])
    target = (allp.min(0) + allp.max(0)) / 2
    plan = ViewPlan(ok=False, reason="", target_m=target)
    if tool.T_tool0_cam is None:
        plan.reason = "the tool model has no camera extrinsic"
        return plan
    pts, nrm, w, _ = seam_targets(surfaces, boxes, vcfg)
    seam = w >= 1.0
    T_cam_tool0 = np.linalg.inv(tool.T_tool0_cam)
    free = dataclasses.replace(model, clearance=max(model.clearance, mcfg.transit_clearance_m))
    sing = np.sin(mcfg.wrist_singularity_margin_rad)

    # 1. every candidate's visibility (cheap); the IK only for those the pick wants
    cands: list[View] = []
    for el in vcfg.elevations_deg:
        for az in np.arange(0.0, 360.0, vcfg.azimuth_step_deg):
            e, a = np.radians(el), np.radians(az)
            cam = target + vcfg.distance_m * np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
            for roll in np.arange(0.0, 360.0, vcfg.roll_step_deg):
                plan.n_candidates += 1
                T_cam = look_at(cam, target, np.radians(roll))
                seen = visible(T_cam, pts, nrm, boxes, vcfg)
                if not (seen & seam).any():
                    _reject(plan.rejected, "sees no seam region")
                    continue
                cands.append(View(float(az), float(el), float(roll), T_cam, T_cam @ T_cam_tool0, None, seen))
    seeable = np.any([v.seen for v in cands], axis=0) if cands else np.zeros(len(pts), bool)

    feasibility: dict[int, bool] = {}

    def feasible(k: int) -> bool:
        if k not in feasibility:
            v = cands[k]
            ok = False
            q = solve_on_branch(v.T_tool0, [q_start, mcfg.home_q], mcfg, rng)
            if q is None:
                _reject(plan.rejected, "no IK on the elbow-up branch")
            elif abs(np.sin(q[4])) < sing:
                _reject(plan.rejected, "wrist_2 near its singularity")
            elif abs(np.sin(q[2])) < sing:
                _reject(plan.rejected, "elbow nearly straight (workspace edge)")
            else:
                d, a_name, b_name = free.min_distance(q)
                if d < free.clearance:
                    _reject(plan.rejected, f"collision ({a_name} x {b_name})")
                else:
                    v.q, v.clearance_m, ok = q, d, True
            feasibility[k] = ok
        return feasibility[k]

    # 2. lazy greedy pick: a point's repeated sightings are worth less (repeat_weights),
    # directions closer than min_separation_deg to a pick are out; the best remaining
    # candidate by gain is checked for feasibility, the first feasible one is taken
    counts = np.zeros(len(pts), int)
    rw = np.asarray(vcfg.repeat_weights, float)
    value = lambda c: np.where(c < len(rw), rw[np.minimum(c, len(rw) - 1)], 0.0)
    picked: list[View] = []
    cos_sep = np.cos(np.radians(vcfg.min_separation_deg))
    for _ in range(vcfg.n_views):
        vals = w * value(counts)
        gains = [(float((vals * v.seen).sum()), k) for k, v in enumerate(cands)
                 if feasibility.get(k, True)
                 and not any(v.direction @ p.direction > cos_sep for p in picked)]
        gains.sort(key=lambda g: -g[0])
        chosen = next(((g, k) for g, k in gains if g > 0.0 and feasible(k)), None)
        if chosen is None:
            break
        # the same direction at another roll sees (almost) the same: of those within 2 %
        # of the gain, take the feasible one with the least joint travel from home - the
        # first one checked may be a stretched arm near a limit
        c = cands[chosen[1]]
        sibs = [(g, k) for g, k in gains if k != chosen[1] and g >= 0.98 * chosen[0]
                and cands[k].azimuth_deg == c.azimuth_deg and cands[k].elevation_deg == c.elevation_deg]
        travel = lambda k: float(np.abs(nearest_in_limits(cands[k].q, mcfg.home_q) - mcfg.home_q).sum())
        best = min([chosen] + [sk for sk in sibs if feasible(sk[1])], key=lambda gk: travel(gk[1]))
        chosen = best
        cands[chosen[1]].gain = chosen[0]
        picked.append(cands[chosen[1]])
        counts += cands[chosen[1]].seen
    plan.n_feasible = sum(feasibility.values())

    # 3. visiting order: nearest neighbour in joint space, each q the 2*pi-equivalent
    # nearest the previous one inside the joint limits
    order: list[View] = []
    q_prev = q_start
    left = list(picked)
    while left:
        k = int(np.argmin([np.abs(nearest_in_limits(v.q, q_prev) - q_prev).sum() for v in left]))
        v = left.pop(k)
        v.q = nearest_in_limits(v.q, q_prev)
        order.append(v)
        q_prev = v.q

    if plan_paths:
        kept: list[View] = []
        q_prev = q_start
        for v in order:
            path = transit_path(q_prev, v.q, transit_model(model, mcfg, q_prev, v.q), unwrap=False)
            if path is None:
                _reject(plan.rejected, "no collision-free transit")
                continue
            v.q = path[-1]
            plan.paths.append(path)
            kept.append(v)
            q_prev = v.q
        order = kept
        if order:
            plan.home_path = transit_path(q_prev, mcfg.home_q,
                                          transit_model(model, mcfg, q_prev, mcfg.home_q), unwrap=False)

    plan.views = order
    counts = np.sum([v.seen for v in order], axis=0) if order else np.zeros(len(pts), int)
    # coverage of the seam region that ANY candidate could see (the underside of a
    # plate is in the seam band too, and no camera above the table ever sees it)
    reach = seam & seeable
    if reach.any():
        plan.coverage = {"seam_1": float((counts[reach] >= 1).mean()),
                         "seam_2": float((counts[reach] >= 2).mean()),
                         "seeable": float(reach.sum() / max(seam.sum(), 1))}
    if len(order) < vcfg.min_views:
        plan.reason = (f"only {len(order)} usable view(s), need {vcfg.min_views} "
                       f"({plan.n_feasible} feasible of {plan.n_candidates})")
    elif plan_paths and plan.home_path is None:
        plan.reason = "no collision-free way home from the last view"
    else:
        plan.ok = True
    return plan
