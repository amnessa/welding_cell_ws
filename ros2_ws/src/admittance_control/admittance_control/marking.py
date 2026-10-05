"""The pen-marking motion, computed (milestone 4 of notes/pen_marking_plan.md).

Everything the marking node needs that is not ROS: the visit order, the transit path
(collision-checked RRT + shortcutting), the straight descent as an IK chain, the timing
of joint paths, and the contact bookkeeping. The node (`tack_marking_node.py`) only
sends what this module computes and watches the wrench.

    plan  = build_marking_plan(report, tool, model, cfg, q_now)   # from tack_reach.json
    for step in plan.steps:                                     # transit, descend, retract
        ...

Per tack the four phases are:
    transit   joint path q_now -> q_app: the straight joint-space edge if it is clear,
              else RRT-Connect with the collision model (edge resolution 0.01 rad) and
              collision-aware shortcutting; timed by a joint speed limit
    descend   the pen tip along its axis from the approach point to the tack point plus
              `overshoot_m`, as an IK chain seeded from the previous step, timed by the
              tip speed - the node cancels it at |F| > touch force
    dwell     the mark (dot), or
    stroke    the tack segment / the seam polyline, contact-referenced: the measured
              contact depth plus a press, drawn in short chunks with the depth
              corrected between chunks from the force along the pen (`stroke_chain`)
    retract   along the pen axis from wherever the pen stopped

Contact depth: `(point - tip_contact) . axis` with `axis` the pen direction into the
joint. Positive = the pen met the surface BEFORE the registered point (the real surface
is nearer than the model says), negative = it went past it. That number, per tack, is
the placement check the plan is validated by.
"""

from __future__ import annotations

import dataclasses

import random
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

from .collision import CollisionModel
from .kinematics import _edge_valid, bezier_smooth_path, rrt_connect, ur5e_fk
from .tack_reach import MarkingConfig, _unwrap_to, solve_on_branch


# ---------------------------------------------------------------- paths ---------------
def shortcut_path(path: Sequence[np.ndarray], is_valid: Callable[[np.ndarray], bool],
                  resolution: float = 0.01, iterations: int = 100, seed: int = 0
                  ) -> list[np.ndarray]:
    """Random shortcutting that keeps every new edge valid under the SAME validity test
    the planner used (the package's `smooth_path` shortcuts with joint limits only)."""
    rng = random.Random(seed)
    path = [np.asarray(q, float) for q in path]
    for _ in range(iterations):
        if len(path) < 3:
            break
        i = rng.randint(0, len(path) - 3)
        j = rng.randint(i + 2, len(path) - 1)
        if _edge_valid(path[i], path[j], resolution=resolution, is_valid=is_valid):
            path = path[:i + 1] + path[j:]
    return path


def transit_model(model: CollisionModel, cfg: MarkingConfig, *qs: np.ndarray) -> CollisionModel:
    """The collision model for a FREE move: `cfg.transit_clearance_m` instead of the tack
    clearance, so the path keeps real room from the parts (2026-09-29: with the 3 mm tack
    clearance a transit to the far side of a standing plate skimmed it and, the parts
    sitting ~10 mm off their model, hit it at 9 N). Where an end pose itself sits closer
    (an approach pose beside a plate) the bound drops to 90 % of that pose's clearance,
    never below the tack clearance, so the move stays plannable."""
    need = float(cfg.transit_clearance_m)
    for q in qs:
        need = min(need, 0.9 * model.min_distance(np.asarray(q, float))[0])
    return dataclasses.replace(model, clearance=max(model.clearance, need))


def smooth_transit(path: Sequence[np.ndarray], is_valid: Callable[[np.ndarray], bool],
                   resolution: float = 0.01, max_step: float = 0.02) -> list[np.ndarray] | None:
    """Round the corners of a shortcut path the way the drawing server does: a centripetal
    Catmull-Rom spline through its waypoints (`kinematics.bezier_smooth_path`, continuous
    joint velocity), sampled every ~`max_step` rad. The spline leaves the straight edges
    that were collision-checked, so every edge between samples is checked again with the
    planner's own `is_valid`. Returns the dense path, or None when any of it leaves free
    space (the caller keeps the straight-segment path). Fewer than 3 points: nothing to round.
    """
    path = [np.asarray(q, float) for q in path]
    if len(path) < 3:
        return path
    dense = bezier_smooth_path(path, max_step=max_step)
    if np.abs(dense[0] - path[0]).max() > 1e-6 or np.abs(dense[-1] - path[-1]).max() > 1e-6:
        return None
    dense[0], dense[-1] = path[0].copy(), path[-1].copy()
    for a, b in zip(dense[:-1], dense[1:]):
        if not _edge_valid(a, b, resolution=resolution, is_valid=is_valid):
            return None
    return dense


def transit_path(q_from: np.ndarray, q_to: np.ndarray, model: CollisionModel,
                 edge_resolution: float = 0.01, max_iter: int = 8000, seed: int = 0,
                 unwrap: bool = True, smooth: bool = False) -> list[np.ndarray] | None:
    """Collision-free joint path: the straight edge when it is clear, else RRT + shortcut,
    and with ``smooth`` the shortcut path's corners rounded (`smooth_transit`; kept as it
    is when the rounded one is not clear). Before 2026-10-05 the marking node drove the
    straight segments of the first RRT-Connect path, corners and all - drastic swings
    between tacks on opposite sides of a plate.

    ``unwrap``: go to the 2*pi-equivalent of ``q_to`` nearest ``q_from`` (between tacks:
    the shortest wrist turn). False = to ``q_to`` exactly - the way home, so a wrist that
    wound up over the tacks turns back instead of arriving one turn off (2026-09-29:
    wrist_3 came home at 5.42 rad instead of -0.86, a turn closer to its +-2*pi limit).
    """
    q_from = np.asarray(q_from, float); q_to = np.asarray(q_to, float)
    if unwrap:
        q_to = _unwrap_to(q_to, q_from)
    if _edge_valid(q_from, q_to, resolution=edge_resolution, is_valid=model.is_valid):
        return [q_from, q_to]
    for attempt in range(3):
        random.seed(seed + attempt)
        raw = rrt_connect(q_from, q_to, step_size=0.15, max_iter=max_iter,
                          is_valid=model.is_valid, edge_resolution=edge_resolution)
        if raw is not None:
            short = shortcut_path(raw, model.is_valid, edge_resolution, seed=seed + attempt)
            if smooth:
                dense = smooth_transit(short, model.is_valid, edge_resolution)
                if dense is not None:
                    return dense
            return short
    return None


def _densify(path: Sequence[np.ndarray], max_step: float) -> list[np.ndarray]:
    out = [np.asarray(path[0], float)]
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.ceil(float(np.abs(b - a).max()) / max_step)))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return out


def time_trapezoid(path: Sequence[np.ndarray], v_max: float = 0.3, a_max: float = 0.5,
                   max_step: float = 0.02) -> list[tuple[np.ndarray, float]]:
    """(q, t) along the path with a trapezoidal speed profile: accelerate at `a_max`,
    cruise at `v_max`, decelerate to a stop, all in the arc length s = sum |dq|_inf, so no
    joint exceeds `v_max`. The path is first sampled every `max_step` rad, so the ramps
    happen inside straight segments too (the controller interpolates linearly between
    points). The fallback when TOTG is unavailable (`TrajectoryExecutor.time_path`)."""
    pts = _densify(path, max_step)
    keep = [pts[0]]
    for q in pts[1:]:
        if np.abs(q - keep[-1]).max() > 1e-9:          # the controller wants rising times
            keep.append(q)
    if len(keep) == 1:
        return [(keep[0], 0.0)]
    s = np.concatenate([[0.0], np.cumsum([np.abs(b - a).max() for a, b in zip(keep[:-1], keep[1:])])])
    S = float(s[-1])
    if S <= v_max * v_max / a_max:                      # triangular: never reaches v_max
        vp = float(np.sqrt(S * a_max))
        s_acc = S / 2.0
    else:
        vp = v_max
        s_acc = v_max * v_max / (2.0 * a_max)
    t_acc = vp / a_max
    T = 2.0 * t_acc + (S - 2.0 * s_acc) / vp

    def t_of(x: float) -> float:
        if x <= s_acc:
            return float(np.sqrt(2.0 * x / a_max))
        if x <= S - s_acc:
            return t_acc + (x - s_acc) / vp
        return T - float(np.sqrt(max(0.0, 2.0 * (S - x) / a_max)))

    return [(q, t_of(float(x))) for q, x in zip(keep, s)]


def time_joint_path(path: Sequence[np.ndarray], v_max: float = 0.3, t_min: float = 0.3
                    ) -> list[tuple[np.ndarray, float]]:
    """(q, t) with each segment taking max(|dq|_inf / v_max, t_min) - a speed cap, not a
    time-optimal profile; the scaled controller blends between the points."""
    out = [(np.asarray(path[0], float), 0.0)]
    t = 0.0
    for a, b in zip(path[:-1], path[1:]):
        dt = max(float(np.abs(np.asarray(b) - np.asarray(a)).max()) / v_max, t_min)
        t += dt
        out.append((np.asarray(b, float), t))
    return out


def descent_chain(tool, point: np.ndarray, axis: np.ndarray, roll_rad: float,
                  q_app: np.ndarray, cfg: MarkingConfig, overshoot_m: float = 0.003,
                  step_m: float = 0.002) -> list[np.ndarray] | None:
    """IK chain of the tip moving along `axis` from the approach point to the tack point
    plus `overshoot_m`, `step_m` apart, each solve seeded from the previous (continuity;
    the branch lock still applies). None if the chain breaks."""
    point = np.asarray(point, float); axis = np.asarray(axis, float) / np.linalg.norm(axis)
    total = cfg.standoff_m + overshoot_m
    n = max(2, int(np.ceil(total / step_m)))
    chain = [np.asarray(q_app, float)]
    for k in range(1, n + 1):
        tip = point - (cfg.standoff_m - k * total / n) * axis
        q = solve_on_branch(tool.T_tool0_for_tip(tip, axis, roll_rad), [chain[-1]], cfg, n_random=0)
        if q is None:
            return None
        q = _unwrap_to(q, chain[-1])
        if np.abs(q - chain[-1]).max() > 0.3:
            return None
        chain.append(q)
    return chain


def time_descent(chain: Sequence[np.ndarray], tool, v_tip: float = 0.02,
                 v_slow: float | None = None, slow_zone_m: float = 0.0,
                 point: np.ndarray | None = None, axis: np.ndarray | None = None
                 ) -> list[tuple[np.ndarray, float]]:
    """(q, t) along a descent chain at a constant TIP speed - or two speeds: `v_tip`
    until the tip is within `slow_zone_m` of `point` along `axis`, then `v_slow`.

    Why two: the pen is not sprung, and on a rigid part the force past the 1.5 N
    threshold grows as speed x stopping time x stiffness (the table read ~10 N at
    10 mm/s). The slow final approach keeps the contact force near the threshold and
    the recorded contact point accurate, at a few seconds per tack.
    """
    two = v_slow is not None and slow_zone_m > 0.0 and point is not None and axis is not None
    if two:
        point = np.asarray(point, float); axis = np.asarray(axis, float) / np.linalg.norm(axis)
    out = [(np.asarray(chain[0], float), 0.0)]
    t = 0.0
    tip_prev = tool.tip_in(ur5e_fk(chain[0]))
    for q in chain[1:]:
        tip = tool.tip_in(ur5e_fk(q))
        v = v_tip
        if two and float((tip - point) @ axis) > -slow_zone_m:
            v = v_slow
        t += max(float(np.linalg.norm(tip - tip_prev)) / v, 0.02)
        out.append((np.asarray(q, float), t))
        tip_prev = tip
    return out


def resample_polyline(points: np.ndarray, step_m: float = 0.002) -> np.ndarray:
    """Points along a polyline `step_m` apart (endpoints kept)."""
    P = np.asarray(points, float).reshape(-1, 3)
    if len(P) < 2:
        return P.copy()
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(np.ceil(cum[-1] / step_m)) + 1)
    s = np.linspace(0.0, cum[-1], n)
    return np.column_stack([np.interp(s, cum, P[:, k]) for k in range(3)])


def line_chain(tool, tips: np.ndarray, axis: np.ndarray, roll_rad: float, q_seed: np.ndarray,
               cfg: MarkingConfig, max_step_rad: float = 0.3) -> list[np.ndarray] | None:
    """IK chain putting the tip on each of `tips` in turn with the pen along `axis`,
    seeded from the previous solution (branch-locked); None if it breaks."""
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    chain = [np.asarray(q_seed, float)]
    for tip in np.asarray(tips, float).reshape(-1, 3):
        q = solve_on_branch(tool.T_tool0_for_tip(tip, axis, roll_rad), [chain[-1]], cfg, n_random=0)
        if q is None:
            return None
        q = _unwrap_to(q, chain[-1])
        if np.abs(q - chain[-1]).max() > max_step_rad:
            return None
        chain.append(q)
    return chain[1:]


def stroke_chain(tool, points_m: np.ndarray, axis: np.ndarray, roll_rad: float,
                 q_seed: np.ndarray, cfg: MarkingConfig, depth_m: float,
                 step_m: float = 0.002) -> list[np.ndarray] | None:
    """The stroke: the tip along the polyline, pressed `depth_m` along the pen axis
    (positive = into the surface, relative to the registered polyline)."""
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    tips = resample_polyline(points_m, step_m) + depth_m * axis
    return line_chain(tool, tips, axis, roll_rad, q_seed, cfg)


def contact_depth_m(point: np.ndarray, tip_contact: np.ndarray, axis: np.ndarray) -> float:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return float((np.asarray(point, float) - np.asarray(tip_contact, float)) @ axis)


# ---------------------------------------------------------------- the plan ------------
@dataclass
class TackStep:
    tack_id: int
    seam_id: int
    tack_no: int
    point_m: np.ndarray
    axis_m: np.ndarray
    roll_deg: float
    tilt_deg: float
    q_app: np.ndarray
    q_tack: np.ndarray
    transit: list[np.ndarray] = field(default_factory=list)      # q_prev -> q_app
    descent: list[np.ndarray] = field(default_factory=list)      # q_app -> past the point
    transit_ok: bool = False
    descent_ok: bool = False
    reason: str = ""
    #: the stroke to draw after contact (metres, base_link), empty for a dot; the descent
    #: then targets its first point, and `tack_point_m` keeps the tack centre for the record
    stroke_points_m: np.ndarray | None = None
    stroke_mode: str = "dot"
    tack_point_m: np.ndarray | None = None


@dataclass
class MarkingPlan:
    steps: list[TackStep]
    q_start: np.ndarray
    home_path: list[np.ndarray] | None = None                     # last q_app -> home
    ok: bool = False

    def summary(self) -> str:
        lines = []
        for s in self.steps:
            n_t = len(s.transit); n_d = len(s.descent)
            lines.append(f"tack {s.tack_id} (seam {s.seam_id} #{s.tack_no}): "
                         f"transit {'ok' if s.transit_ok else 'FAIL'} ({n_t} wp), "
                         f"descent {'ok' if s.descent_ok else 'FAIL'} ({n_d} wp)"
                         + (f" - {s.reason}" if s.reason else ""))
        lines.append(f"home path: {'ok' if self.home_path else 'FAIL'}")
        lines.append("PLAN OK" if self.ok else "PLAN INCOMPLETE")
        return "\n".join(lines)


def visit_order(report_tacks: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    """Seam by seam, `tack_no` ascending - the pen's order (no heat to balance)."""
    return sorted([t for t in report_tacks if t.get("ok")],
                  key=lambda t: (int(t["seam_id"]), int(t["tack_no"])))


def stroke_targets(mode: str, report_tacks: Sequence[dict[str, Any]],
                   tacks_json: Sequence[dict[str, Any]] | None = None,
                   seams_json: Sequence[dict[str, Any]] | None = None) -> dict[int, np.ndarray]:
    """What each tack draws, by tack id, in metres: nothing (`dot`), its own tack segment
    p0->p1 from welding_tacks.json (`tack`), or its seam's whole weldable polyline from
    welding_seams.json (`seam`; every tack of that seam draws it, so the visit order
    gives one stroke per tack - the node skips repeats of a seam already drawn)."""
    out: dict[int, np.ndarray] = {}
    if mode == "dot":
        return out
    by_id = {int(t["id"]): t for t in (tacks_json or [])}
    seams = {int(s["id"]): s for s in (seams_json or [])}
    for t in report_tacks:
        tid = int(t["tack_id"])
        if mode == "tack" and tid in by_id:
            out[tid] = np.array([by_id[tid]["p0_mm"], by_id[tid]["p1_mm"]], float) / 1000.0
        elif mode == "seam" and int(t["seam_id"]) in seams:
            out[tid] = np.asarray(seams[int(t["seam_id"])]["polyline_mm"], float) / 1000.0
    return out


def build_marking_plan(report: dict[str, Any], tool, model: CollisionModel,
                       cfg: MarkingConfig, q_now: np.ndarray, overshoot_m: float = 0.003,
                       edge_resolution: float = 0.01, strokes: dict[int, np.ndarray] | None = None,
                       stroke_mode: str = "dot", smooth_transits: bool = False) -> MarkingPlan:
    """Transit + descent for every reachable tack of a `tack_reach.json` report, in visit
    order, starting from the robot's current joints and ending with a path home. With
    `strokes` (from `stroke_targets`) the descent aims at the stroke's first point and the
    step carries the polyline to draw after contact. `smooth_transits`: round the transits'
    corners (`transit_path(..., smooth=True)`)."""
    q_now = np.asarray(q_now, float)
    steps: list[TackStep] = []
    q_prev = q_now
    all_ok = True
    strokes = strokes or {}
    for t in visit_order(report["tacks"]):
        step = TackStep(tack_id=int(t["tack_id"]), seam_id=int(t["seam_id"]),
                        tack_no=int(t["tack_no"]), point_m=np.asarray(t["point_m"], float),
                        axis_m=np.asarray(t["axis_m"], float), roll_deg=float(t["roll_deg"]),
                        tilt_deg=float(t.get("tilt_deg", 0.0)),
                        q_app=np.asarray(t["q_app"], float), q_tack=np.asarray(t["q_tack"], float))
        step.tack_point_m = step.point_m.copy()
        if step.tack_id in strokes and len(strokes[step.tack_id]) >= 2:
            step.stroke_points_m = np.asarray(strokes[step.tack_id], float)
            step.stroke_mode = stroke_mode
            # descend at the stroke's start; its approach pose must exist on the branch
            step.point_m = step.stroke_points_m[0].copy()
            T_app = tool.T_tool0_for_tip(step.point_m - cfg.standoff_m * step.axis_m, step.axis_m,
                                         np.deg2rad(step.roll_deg))
            q_app = solve_on_branch(T_app, [step.q_app, cfg.home_q], cfg)
            if q_app is None:
                step.reason = "no approach pose over the stroke start"
                all_ok = False
                steps.append(step)
                continue
            step.q_app = _unwrap_to(q_app, step.q_app)
        path = transit_path(q_prev, step.q_app, transit_model(model, cfg, q_prev, step.q_app),
                            edge_resolution, smooth=smooth_transits)
        if path is None:
            step.reason = "no collision-free transit"
            all_ok = False
        else:
            step.transit, step.transit_ok = path, True
            step.q_app = path[-1]                             # unwrapped to the previous q
        chain = descent_chain(tool, step.point_m, step.axis_m, np.deg2rad(step.roll_deg),
                              step.q_app, cfg, overshoot_m)
        if chain is None:
            step.reason = (step.reason + "; " if step.reason else "") + "descent IK chain broke"
            all_ok = False
        else:
            # The chain must stay clear (except for the pen meeting the part) up to the
            # tack point. Beyond it lies the overshoot, which the pen only reaches when
            # the real surface is farther than registered - and then the plates are
            # farther too - so there the bound is the clearance the tack pose had minus
            # the distance travelled past it (moving `s` along the pen changes any
            # distance by at most `s`), never a fixed number: what the model shows as an
            # overlap there is the registration's, not the motion's.
            d_ref, past_ref = np.inf, 0.0        # clearance at the last point before the tack
            for q in chain:
                tip = tool.tip_in(ur5e_fk(q))
                past = float((tip - step.point_m) @ step.axis_m)
                d = min((p[0] for p in model.pair_distances(q)
                         if not (p[1] == "pen" and p[2].startswith("part_"))), default=np.inf)
                if past <= 1e-4:
                    need, d_ref, past_ref = cfg.clearance_m, d, past
                else:
                    need = max(0.0, d_ref - (past - past_ref))
                if d < need:
                    step.reason = (step.reason + "; " if step.reason else "") + \
                        f"descent clearance {d * 1000:.1f} mm{' (overshoot)' if past > 1e-4 else ''}"
                    all_ok = False
                    break
            else:
                step.descent, step.descent_ok = chain, True
        steps.append(step)
        if step.transit_ok:
            q_prev = step.q_app
    home = (transit_path(q_prev, cfg.home_q, transit_model(model, cfg, q_prev, cfg.home_q),
                         edge_resolution, unwrap=False, smooth=smooth_transits) if steps else [q_now])
    if home is None:
        all_ok = False
    return MarkingPlan(steps=steps, q_start=q_now, home_path=home, ok=all_ok and bool(steps))


def plan_to_dict(plan: MarkingPlan) -> dict[str, Any]:
    return {"ok": plan.ok, "q_start": plan.q_start.tolist(),
            "home_path": None if plan.home_path is None else [q.tolist() for q in plan.home_path],
            "steps": [{k: (v.tolist() if isinstance(v, np.ndarray) else
                           [q.tolist() for q in v] if isinstance(v, list) else v)
                       for k, v in s.__dict__.items()} for s in plan.steps]}
