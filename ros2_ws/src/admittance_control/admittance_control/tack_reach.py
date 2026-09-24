"""Reachability of the tacks, before anything moves (milestone 3 of the pen plan).

For every tack of `welding_tacks.json` this answers: can the pen tip get there, on the
elbow-up branch, with the approach point above it and the straight descent between the
two clear of the parts and the table - and at what roll about the pen axis, chosen once
per seam so the wrist does not twist between neighbouring tacks.

    cfg    = load_marking_config()                    # config/marking.json (home, lock, ...)
    tool   = load_tool_model()
    model  = CollisionModel(tool, boxes_from_parts(parts), table_z, clearance)
    report = plan_tacks(tacks_mm, tool, model, cfg)   # tacks as mode A wrote them (mm)
    print(format_report(report))

The pen axis (tool0 +Z) points INTO the joint, i.e. along minus the tack's `approach`
(the cleared torch axis, which points out of the dihedral). The tip target is the tack
point; the approach point is `standoff_m` further out along `approach`.

Branch lock: the signs of the joints in `branch_joints` (lift, elbow, wrist_2 - the
elbow-up / wrist-not-flipped signature of `home_q`) must match at every solution;
solutions on another branch are discarded, never used. IK is the package's damped
least squares, seeded from the previous solution (continuity) and from the home pose.

Orientation choice per seam: every (work-angle tilt, roll) on the grids is tried for all
tacks of the seam (ascending `tack_no`). The tilt rotates the pen off the bisector about
the seam tangent within the admissible cone (`work_angles_deg`); the roll is the free
rotation about the pen. A combination is admissible when every tack has a branch-locked,
collision-free approach pose and tack pose and a clear LIN descent (along the pen axis),
and no single joint moves more than `max_joint_step_rad` between consecutive tacks.
Among admissible combinations the one with the largest minimum clearance wins, then the
least joint travel (with a penalty for wrist_2 near its singularity). Pose-only checks of
the tool envelope run before any IK, so hopeless combinations cost nothing. Seams with
no admissible combination are reported per tack with the reason of the best attempt.

At the tack pose the pen is SUPPOSED to touch the part, so its own distance to the parts
is excluded from that pose's clearance and reported separately as `pen_gap_m` (~0).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .collision import CollisionModel
from .kinematics import ik_solve, ur5e_fk

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MARKING_CONFIG = PACKAGE_ROOT / "config" / "marking.json"
UR_ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


# ---------------------------------------------------------------- config --------------
@dataclass
class MarkingConfig:
    home_q: np.ndarray
    branch_joints: tuple[int, ...] = (1, 2, 4)
    standoff_m: float = 0.035
    clearance_m: float = 0.01
    roll_step_deg: float = 15.0
    lin_steps: int = 6
    max_joint_step_rad: float = 1.2
    wrist_singularity_margin_rad: float = 0.35
    work_angles_deg: tuple[float, ...] = (0.0, -10.0, 10.0, -20.0, 20.0)
    table_z_m: float | None = None
    source: str = ""
    #: when set, ONLY these rolls (deg) are tried - the attribution experiment: the same
    #: tack at rolls 90 deg apart moves a TOOL error with the wrist and leaves a WORLD
    #: error (camera, registration) where it is
    roll_list_deg: tuple[float, ...] | None = None

    @property
    def branch_signature(self) -> tuple[int, ...]:
        return branch_signature(self.home_q, self.branch_joints)


def load_marking_config(path: str | Path | None = None) -> MarkingConfig:
    p = Path(path) if path else DEFAULT_MARKING_CONFIG
    cfg = json.loads(p.read_text())
    return MarkingConfig(home_q=np.asarray(cfg["home_q"], float),
                         branch_joints=tuple(cfg.get("branch_joints", (1, 2, 4))),
                         standoff_m=float(cfg.get("standoff_m", 0.035)),
                         clearance_m=float(cfg.get("clearance_m", 0.01)),
                         roll_step_deg=float(cfg.get("roll_step_deg", 15.0)),
                         lin_steps=int(cfg.get("lin_steps", 6)),
                         max_joint_step_rad=float(cfg.get("max_joint_step_rad", 1.2)),
                         wrist_singularity_margin_rad=float(cfg.get("wrist_singularity_margin_rad", 0.35)),
                         work_angles_deg=tuple(cfg.get("work_angles_deg", (0.0, -10.0, 10.0, -20.0, 20.0))),
                         table_z_m=cfg.get("table_z_m"), source=str(p))


def joint_state_to_ur_order(names: Sequence[str], positions: Sequence[float]) -> np.ndarray:
    """`/joint_states` lists joints alphabetically; the kinematics want UR order."""
    lookup = dict(zip(names, positions))
    return np.array([float(lookup[n]) for n in UR_ORDER])


def branch_signature(q: np.ndarray, joints: Sequence[int] = (1, 2, 4)) -> tuple[int, ...]:
    return tuple(int(np.sign(q[j])) for j in joints)


def same_branch(q: np.ndarray, signature: tuple[int, ...], joints: Sequence[int] = (1, 2, 4)) -> bool:
    return branch_signature(q, joints) == tuple(signature)


# ---------------------------------------------------------------- IK on the branch ----
def solve_on_branch(T_tool0: np.ndarray, seeds: Sequence[np.ndarray], cfg: MarkingConfig,
                    rng: np.random.Generator | None = None, n_random: int = 6
                    ) -> np.ndarray | None:
    """Damped-LS IK from several seeds; the first solution on the locked branch wins.
    Random seeds are drawn AROUND the home pose (so they start on its branch)."""
    rng = rng or np.random.default_rng(0)
    sig = cfg.branch_signature
    tried = list(seeds)
    for _ in range(n_random):
        tried.append(cfg.home_q + rng.normal(scale=0.4, size=6))
    for s in tried:
        q = ik_solve(T_tool0, np.asarray(s, float))
        if q is None:
            continue
        q = _unwrap_to(q, tried[0])
        if same_branch(q, sig, cfg.branch_joints):
            return q
    return None


def _unwrap_to(q: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Shift each joint by 2*pi multiples to lie nearest `ref` (continuity)."""
    q = np.asarray(q, float).copy()
    ref = np.asarray(ref, float)
    for i in range(6):
        while q[i] - ref[i] > np.pi:
            q[i] -= 2 * np.pi
        while q[i] - ref[i] < -np.pi:
            q[i] += 2 * np.pi
    return q


# ---------------------------------------------------------------- the plan ------------
@dataclass
class TackPlan:
    tack_id: int
    seam_id: int
    tack_no: int
    ok: bool
    reason: str = ""
    roll_deg: float = 0.0
    tilt_deg: float = 0.0
    q_app: np.ndarray | None = None
    q_tack: np.ndarray | None = None
    clearance_app_m: float = np.inf
    clearance_tack_m: float = np.inf
    worst_pair_app: str = ""
    worst_pair_tack: str = ""
    pen_gap_m: float = np.inf
    lin_min_clearance_m: float = np.inf
    joint_step_from_prev_rad: float = 0.0
    point_m: np.ndarray | None = None
    axis_m: np.ndarray | None = None          # pen axis (into the joint)

    def as_dict(self) -> dict[str, Any]:
        d = dict(self.__dict__)
        for k, v in d.items():
            if isinstance(v, np.ndarray):
                d[k] = v.tolist()
            elif isinstance(v, float) and not np.isfinite(v):
                d[k] = None
        return d


def _rotate(v: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    return (v * np.cos(angle) + np.cross(axis, v) * np.sin(angle)
            + axis * (axis @ v) * (1 - np.cos(angle)))


def pen_axis_for(tack: dict[str, Any], tilt_rad: float) -> np.ndarray:
    """The pen direction (into the joint): minus the tack's approach, tilted by
    `tilt_rad` about the seam tangent (the work angle off the bisector)."""
    approach = np.asarray(tack["approach"], float)
    approach /= np.linalg.norm(approach)
    tangent = np.asarray(tack["p1_mm"], float) - np.asarray(tack["p0_mm"], float)
    if np.linalg.norm(tangent) < 1e-9:
        return -approach
    return _rotate(-approach, tangent, tilt_rad)


def tool_only_clearance(model: CollisionModel, T_tool0: np.ndarray, exclude_pen: bool
                        ) -> tuple[float, str]:
    """Clearance of the TOOL envelope alone at a tool0 pose - no IK needed, so a pose
    that cannot work is dropped before any solve."""
    from .collision import lowest_point_z, primitive_distance
    best = (np.inf, "")
    for prim in model.tool.primitives_in(T_tool0):
        if exclude_pen and prim["name"] == "pen":
            continue
        for s in model.scene_boxes:
            d = primitive_distance(prim, s)
            if d < best[0]:
                best = (d, f"{prim['name']} x {s['name']}")
        if model.table_z is not None:
            d = max(0.0, lowest_point_z(prim) - model.table_z)
            if d < best[0]:
                best = (d, f"{prim['name']} x table")
    return best


def _clearance_excluding_pen(model: CollisionModel, q: np.ndarray) -> tuple[float, str, float]:
    """(min distance over pairs that are not pen-vs-part, its pair, the pen-vs-part gap)."""
    pairs = model.pair_distances(q)
    others = [p for p in pairs if not (p[1] == "pen" and p[2].startswith("part_"))]
    pen = [p for p in pairs if p[1] == "pen" and p[2].startswith("part_")]
    d, a, b = min(others, key=lambda x: x[0]) if others else (np.inf, "", "")
    return d, f"{a} x {b}", (min(p[0] for p in pen) if pen else np.inf)


def _try_tack(tack: dict[str, Any], roll: float, tilt: float, seeds: Sequence[np.ndarray],
              tool, model, cfg: MarkingConfig, rng) -> TackPlan:
    point = np.asarray(tack["point_mm"], float) / 1000.0
    axis = pen_axis_for(tack, tilt)                    # pen points into the joint
    approach = -axis                                   # the LIN descent runs along the pen
    plan = TackPlan(tack_id=int(tack["id"]), seam_id=int(tack["seam_id"]),
                    tack_no=int(tack["tack_no"]), ok=False, roll_deg=float(np.degrees(roll)),
                    tilt_deg=float(np.degrees(tilt)), point_m=point, axis_m=axis)
    # pose-only prechecks (no IK): the tool envelope at the tack and approach poses
    d_tool, pair_tool = tool_only_clearance(model, tool.T_tool0_for_tip(point, axis, roll), True)
    if d_tool < cfg.clearance_m:
        plan.clearance_tack_m, plan.worst_pair_tack = d_tool, pair_tool
        plan.reason = f"tack pose in collision ({pair_tool}, {d_tool * 1000:.1f} mm)"
        return plan
    d_tool, pair_tool = tool_only_clearance(
        model, tool.T_tool0_for_tip(point + cfg.standoff_m * approach, axis, roll), False)
    if d_tool < cfg.clearance_m:
        plan.clearance_app_m, plan.worst_pair_app = d_tool, pair_tool
        plan.reason = f"approach pose in collision ({pair_tool}, {d_tool * 1000:.1f} mm)"
        return plan
    T_app = tool.T_tool0_for_tip(point + cfg.standoff_m * approach, axis, roll)
    q_app = solve_on_branch(T_app, seeds, cfg, rng)
    if q_app is None:
        plan.reason = "approach pose: no IK on the elbow-up branch"
        return plan
    plan.q_app = q_app
    d, pair, _ = _clearance_excluding_pen(model, q_app)
    pen_app = min((p[0] for p in model.pair_distances(q_app) if p[1] == "pen"), default=np.inf)
    plan.clearance_app_m, plan.worst_pair_app = min(d, pen_app), pair
    if min(d, pen_app) < cfg.clearance_m:
        plan.reason = f"approach pose in collision ({pair}, {min(d, pen_app) * 1000:.1f} mm)"
        return plan
    T_tack = tool.T_tool0_for_tip(point, axis, roll)
    q_tack = solve_on_branch(T_tack, [q_app] + list(seeds), cfg, rng)
    if q_tack is None:
        plan.reason = "tack pose: no IK on the elbow-up branch"
        return plan
    q_tack = _unwrap_to(q_tack, q_app)
    plan.q_tack = q_tack
    d, pair, pen_gap = _clearance_excluding_pen(model, q_tack)
    plan.clearance_tack_m, plan.worst_pair_tack, plan.pen_gap_m = d, pair, pen_gap
    if d < cfg.clearance_m:
        plan.reason = f"tack pose in collision ({pair}, {d * 1000:.1f} mm)"
        return plan
    if np.abs(q_tack - q_app).max() > cfg.max_joint_step_rad:
        plan.reason = "descent needs a large joint move (branch change or singularity)"
        return plan
    # the LIN descent: tip along the line, IK seeded from the previous step
    lin_min = np.inf
    q_prev = q_app
    for k in range(1, cfg.lin_steps):
        f = k / cfg.lin_steps
        tip = point + (1 - f) * cfg.standoff_m * approach
        q_k = solve_on_branch(tool.T_tool0_for_tip(tip, axis, roll), [q_prev], cfg, rng, n_random=0)
        if q_k is None or np.abs(q_k - q_prev).max() > 0.5:
            plan.reason = f"descent step {k}: IK lost the branch"
            return plan
        d_k, _, _ = _clearance_excluding_pen(model, q_k)
        lin_min = min(lin_min, d_k)
        q_prev = q_k
    plan.lin_min_clearance_m = lin_min
    if lin_min < cfg.clearance_m:
        plan.reason = f"descent path in collision ({lin_min * 1000:.1f} mm)"
        return plan
    plan.ok = True
    plan.reason = "ok"
    return plan


def _score(plans: Sequence[TackPlan], q_from: np.ndarray, cfg: MarkingConfig) -> float:
    travel = 0.0
    q_prev = q_from
    sing = 0.0
    for p in plans:
        travel += float(np.abs(p.q_app - q_prev).sum())
        q_prev = p.q_app
        margin = abs(np.sin(p.q_app[4]))
        if margin < np.sin(cfg.wrist_singularity_margin_rad):
            sing += 1.0
    return travel + 3.0 * sing


def plan_tacks(tacks: Sequence[dict[str, Any]], tool, model: CollisionModel,
               cfg: MarkingConfig, seed: int = 0) -> dict[str, Any]:
    """One roll per seam, every tack judged; returns the report as a dict."""
    rng = np.random.default_rng(seed)
    rolls = np.deg2rad(np.asarray(cfg.roll_list_deg, float) if cfg.roll_list_deg
                       else np.arange(0.0, 360.0, cfg.roll_step_deg))
    tilts = np.deg2rad(np.asarray(cfg.work_angles_deg, float))
    by_seam: dict[int, list[dict[str, Any]]] = {}
    for t_ in tacks:
        by_seam.setdefault(int(t_["seam_id"]), []).append(t_)
    seams_out = []
    all_plans: list[TackPlan] = []
    q_from = cfg.home_q
    for sid in sorted(by_seam):
        seam_tacks = sorted(by_seam[sid], key=lambda t_: int(t_["tack_no"]))
        best: tuple[tuple[float, float], list[TackPlan]] | None = None
        best_failed: list[TackPlan] | None = None
        best_clr: tuple[float, str] | None = None        # the best tack-pose clearance seen
        for tilt in tilts:
            for roll in rolls:
                plans = []
                seeds = [q_from, cfg.home_q]
                ok = True
                for t_ in seam_tacks:
                    p = _try_tack(t_, roll, tilt, seeds, tool, model, cfg, rng)
                    if plans and p.q_app is not None and plans[-1].q_app is not None:
                        p.joint_step_from_prev_rad = float(np.abs(p.q_app - plans[-1].q_app).max())
                        if p.ok and p.joint_step_from_prev_rad > cfg.max_joint_step_rad:
                            p.ok = False
                            p.reason = (f"joint step from previous tack "
                                        f"{np.degrees(p.joint_step_from_prev_rad):.0f} deg > "
                                        f"{np.degrees(cfg.max_joint_step_rad):.0f} deg")
                    plans.append(p)
                    if np.isfinite(p.clearance_tack_m) and (best_clr is None or p.clearance_tack_m > best_clr[0]):
                        best_clr = (p.clearance_tack_m, p.worst_pair_tack)
                    if not p.ok:
                        ok = False
                        break
                    seeds = [p.q_app, cfg.home_q]
                if ok:
                    # largest minimum clearance first, then least travel
                    clr = min(min(p.clearance_app_m, p.clearance_tack_m, p.lin_min_clearance_m)
                              for p in plans)
                    key = (-round(clr, 4), _score(plans, q_from, cfg))
                    if best is None or key < best[0]:
                        best = (key, plans)
                else:
                    n_ok = sum(1 for p in plans if p.ok)
                    if best_failed is None or n_ok > sum(1 for p in best_failed if p.ok):
                        best_failed = plans
        if best is not None:
            chosen = best[1]
            q_from = chosen[-1].q_app
            seams_out.append({"seam_id": sid, "ok": True, "roll_deg": chosen[0].roll_deg,
                              "tilt_deg": chosen[0].tilt_deg, "min_clearance_m": -best[0][0],
                              "score": best[0][1], "n_tacks": len(chosen)})
        else:
            chosen = best_failed or []
            reason = chosen[-1].reason if chosen else "no tacks"
            if chosen and chosen[-1].clearance_tack_m < cfg.clearance_m and best_clr is not None:
                reason = (f"{reason}; best over all tilts/rolls {best_clr[0] * 1000:.1f} mm "
                          f"({best_clr[1]}) - a longer pen reach or a slimmer holder is what "
                          f"raises it, not the orientation")
            seams_out.append({"seam_id": sid, "ok": False, "roll_deg": None, "tilt_deg": None,
                              "min_clearance_m": None, "score": None, "n_tacks": len(seam_tacks),
                              "reason": reason})
        all_plans.extend(chosen)
    return {"config": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                       for k, v in cfg.__dict__.items()},
            "branch_signature": list(cfg.branch_signature),
            "seams": seams_out,
            "tacks": [p.as_dict() for p in all_plans],
            "all_ok": all(s["ok"] for s in seams_out) and bool(seams_out)}


def format_report(report: dict[str, Any]) -> str:
    lines = [f"branch lock (lift, elbow, wrist_2 signs): {report['branch_signature']}"]
    for s in report["seams"]:
        if s["ok"]:
            lines.append(f"seam {s['seam_id']}: OK, tilt {s['tilt_deg']:+.0f} deg off the bisector, "
                         f"roll {s['roll_deg']:.0f} deg, {s['n_tacks']} tacks, min clearance "
                         f"{s['min_clearance_m'] * 1000:.1f} mm, travel score {s['score']:.2f}")
        else:
            lines.append(f"seam {s['seam_id']}: UNREACHABLE - {s.get('reason', '')}")
    lines.append(f"{'tack':>6} {'seam.no':>8} {'ok':>3} {'app clr':>8} {'tack clr':>9} "
                 f"{'lin clr':>8} {'pen gap':>8} {'step':>6}  reason")
    for t in report["tacks"]:
        f = lambda v: "   -  " if v is None else f"{v * 1000:6.1f}"
        step = "" if t["q_app"] is None else f"{np.degrees(t['joint_step_from_prev_rad']):5.0f}°"
        lines.append(f"{t['tack_id']:>6} {t['seam_id']:>4}.{t['tack_no']:<3} {'yes' if t['ok'] else 'NO':>3} "
                     f"{f(t['clearance_app_m']):>8} {f(t['clearance_tack_m']):>9} "
                     f"{f(t['lin_min_clearance_m']):>8} {f(t['pen_gap_m']):>8} {step:>6}  {t['reason']}")
    lines.append("ALL REACHABLE" if report["all_ok"] else "NOT ALL REACHABLE")
    return "\n".join(lines)
