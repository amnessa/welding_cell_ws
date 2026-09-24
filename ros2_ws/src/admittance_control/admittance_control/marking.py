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
    dwell     the mark
    retract   the descent chain reversed, from wherever the pen stopped

Contact depth: `(point - tip_contact) . axis` with `axis` the pen direction into the
joint. Positive = the pen met the surface BEFORE the registered point (the real surface
is nearer than the model says), negative = it went past it. That number, per tack, is
the placement check the plan is validated by.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

from .collision import CollisionModel
from .kinematics import _edge_valid, rrt_connect, ur5e_fk
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


def transit_path(q_from: np.ndarray, q_to: np.ndarray, model: CollisionModel,
                 edge_resolution: float = 0.01, max_iter: int = 8000, seed: int = 0
                 ) -> list[np.ndarray] | None:
    """Collision-free joint path: the straight edge when it is clear, else RRT + shortcut."""
    q_from = np.asarray(q_from, float); q_to = _unwrap_to(np.asarray(q_to, float), q_from)
    if _edge_valid(q_from, q_to, resolution=edge_resolution, is_valid=model.is_valid):
        return [q_from, q_to]
    for attempt in range(3):
        random.seed(seed + attempt)
        raw = rrt_connect(q_from, q_to, step_size=0.15, max_iter=max_iter,
                          is_valid=model.is_valid, edge_resolution=edge_resolution)
        if raw is not None:
            return shortcut_path(raw, model.is_valid, edge_resolution, seed=seed + attempt)
    return None


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


def time_descent(chain: Sequence[np.ndarray], tool, v_tip: float = 0.02
                 ) -> list[tuple[np.ndarray, float]]:
    """(q, t) along a descent chain at a constant TIP speed."""
    out = [(np.asarray(chain[0], float), 0.0)]
    t = 0.0
    tip_prev = tool.tip_in(ur5e_fk(chain[0]))
    for q in chain[1:]:
        tip = tool.tip_in(ur5e_fk(q))
        t += max(float(np.linalg.norm(tip - tip_prev)) / v_tip, 0.02)
        out.append((np.asarray(q, float), t))
        tip_prev = tip
    return out


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


def build_marking_plan(report: dict[str, Any], tool, model: CollisionModel,
                       cfg: MarkingConfig, q_now: np.ndarray, overshoot_m: float = 0.003,
                       edge_resolution: float = 0.01) -> MarkingPlan:
    """Transit + descent for every reachable tack of a `tack_reach.json` report, in visit
    order, starting from the robot's current joints and ending with a path home."""
    q_now = np.asarray(q_now, float)
    steps: list[TackStep] = []
    q_prev = q_now
    all_ok = True
    for t in visit_order(report["tacks"]):
        step = TackStep(tack_id=int(t["tack_id"]), seam_id=int(t["seam_id"]),
                        tack_no=int(t["tack_no"]), point_m=np.asarray(t["point_m"], float),
                        axis_m=np.asarray(t["axis_m"], float), roll_deg=float(t["roll_deg"]),
                        tilt_deg=float(t.get("tilt_deg", 0.0)),
                        q_app=np.asarray(t["q_app"], float), q_tack=np.asarray(t["q_tack"], float))
        path = transit_path(q_prev, step.q_app, model, edge_resolution)
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
            # the whole chain must stay clear except for the pen meeting the part
            for q in chain:
                d = min((p[0] for p in model.pair_distances(q)
                         if not (p[1] == "pen" and p[2].startswith("part_"))), default=np.inf)
                if d < cfg.clearance_m:
                    step.reason = (step.reason + "; " if step.reason else "") + \
                        f"descent clearance {d * 1000:.1f} mm"
                    all_ok = False
                    break
            else:
                step.descent, step.descent_ok = chain, True
        steps.append(step)
        if step.transit_ok:
            q_prev = step.q_app
    home = transit_path(q_prev, cfg.home_q, model, edge_resolution) if steps else [q_now]
    if home is None:
        all_ok = False
    return MarkingPlan(steps=steps, q_start=q_now, home_path=home, ok=all_ok and bool(steps))


def plan_to_dict(plan: MarkingPlan) -> dict[str, Any]:
    return {"ok": plan.ok, "q_start": plan.q_start.tolist(),
            "home_path": None if plan.home_path is None else [q.tolist() for q in plan.home_path],
            "steps": [{k: (v.tolist() if isinstance(v, np.ndarray) else
                           [q.tolist() for q in v] if isinstance(v, list) else v)
                       for k, v in s.__dict__.items()} for s in plan.steps]}
