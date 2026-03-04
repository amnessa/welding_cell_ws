#!/usr/bin/env python3
"""
UR5e RRT-Connect Planner — Configuration-space motion planning.

Implements:
  - Forward kinematics for UR5e (base_link → tool0) using URDF transforms
  - Damped-least-squares iterative IK
  - RRT-Connect bi-directional planner in joint space
  - Shortcut-based path smoothing
"""

import math
import random
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# UR5e Kinematic Constants (from URDF)
# ---------------------------------------------------------------------------
# Joint limits [rad]  (from URDF)
JOINT_LIMITS = [
    (-6.2832, 6.2832),   # shoulder_pan
    (-6.2832, 6.2832),   # shoulder_lift
    (-3.1416, 3.1416),   # elbow
    (-6.2832, 6.2832),   # wrist_1
    (-6.2832, 6.2832),   # wrist_2
    (-6.2832, 6.2832),   # wrist_3
]

NUM_JOINTS = 6


# ---------------------------------------------------------------------------
# Homogeneous transform helpers
# ---------------------------------------------------------------------------

def _rotx(a: float) -> np.ndarray:
    """4×4 rotation about X."""
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1, 0,  0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1]], dtype=float)

def _roty(a: float) -> np.ndarray:
    """4×4 rotation about Y."""
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]], dtype=float)

def _rotz(a: float) -> np.ndarray:
    """4×4 rotation about Z."""
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]], dtype=float)

def _trans(x: float, y: float, z: float) -> np.ndarray:
    """4×4 translation."""
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T

def _rpy(r: float, p: float, y: float) -> np.ndarray:
    """RPY (roll-pitch-yaw = Rx*Ry*Rz) → 4×4 rotation matrix."""
    return _rotx(r) @ _roty(p) @ _rotz(y)


# ---------------------------------------------------------------------------
# UR5e Forward Kinematics (matches URDF chain: base_link → tool0)
# ---------------------------------------------------------------------------

def ur5e_fk(q: np.ndarray) -> np.ndarray:
    """
    Compute the 4×4 homogeneous transform from base_link to tool0.

    Parameters
    ----------
    q : array-like, shape (6,)
        Joint angles [shoulder_pan, shoulder_lift, elbow,
                      wrist_1, wrist_2, wrist_3] in radians.

    Returns
    -------
    T : np.ndarray, shape (4, 4)
        Pose of tool0 in base_link frame.

    The chain follows the URDF exactly:
      base_link
        → Rz(π)                         (base_link_inertia)
        → Tz(0.1625) Rz(q1)            (shoulder_pan)
        → Rx(π/2) Rz(q2)               (shoulder_lift)
        → Tx(-0.425) Rz(q3)            (elbow)
        → Tx(-0.3922) Tz(0.1333) Rz(q4)(wrist_1)
        → Rx(π/2) Ty(-0.0997) Rz(q5)  (wrist_2)
        → Rx(π/2)Ry(π)Rz(π) Ty(0.0996) Rz(q6) (wrist_3)
        → Ry(-π/2)Rz(-π/2)            (flange)
        → Rx(π/2)Rz(π/2)              (tool0)
    """
    PI = math.pi
    q = np.asarray(q, dtype=float)

    # base_link → base_link_inertia
    T = _rotz(PI)

    # shoulder_pan_joint
    T = T @ _trans(0, 0, 0.1625) @ _rotz(q[0])

    # shoulder_lift_joint
    T = T @ _rpy(PI/2, 0, 0) @ _rotz(q[1])

    # elbow_joint
    T = T @ _trans(-0.425, 0, 0) @ _rotz(q[2])

    # wrist_1_joint
    T = T @ _trans(-0.3922, 0, 0.1333) @ _rotz(q[3])

    # wrist_2_joint
    T = T @ _rpy(PI/2, 0, 0) @ _trans(0, -0.0997, 0) @ _rotz(q[4])

    # wrist_3_joint
    T = T @ _rpy(PI/2, PI, PI) @ _trans(0, 0.0996, 0) @ _rotz(q[5])

    # wrist_3_link → flange (fixed)
    T = T @ _rpy(0, -PI/2, -PI/2)

    # flange → tool0 (fixed)
    T = T @ _rpy(PI/2, 0, PI/2)

    return T


def ur5e_jacobian(q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """
    Numerical Jacobian (6×6) via central finite differences.

    Rows 0-2: linear velocity (x, y, z)
    Rows 3-5: angular velocity (wx, wy, wz)
    """
    q = np.asarray(q, dtype=float)
    J = np.zeros((6, NUM_JOINTS))
    T0 = ur5e_fk(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]

    for i in range(NUM_JOINTS):
        qp = q.copy(); qp[i] += eps
        qm = q.copy(); qm[i] -= eps
        Tp = ur5e_fk(qp)
        Tm = ur5e_fk(qm)

        # Linear part
        J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * eps)

        # Angular part (from rotation difference)
        dR = Tp[:3, :3] @ Tm[:3, :3].T
        # Extract angle-axis from dR
        J[3, i] = (dR[2, 1] - dR[1, 2]) / (2 * eps)
        J[4, i] = (dR[0, 2] - dR[2, 0]) / (2 * eps)
        J[5, i] = (dR[1, 0] - dR[0, 1]) / (2 * eps)

    return J


# ---------------------------------------------------------------------------
# Pose error helpers
# ---------------------------------------------------------------------------

def _rotmat_to_angle_axis(R: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → angle-axis vector (3,)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    cos_a = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_a)
    if abs(angle) < 1e-7:
        return np.zeros(3)
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-10:
        return np.zeros(3)
    return (angle / n) * axis


def pose_error(T_current: np.ndarray, T_target: np.ndarray) -> np.ndarray:
    """
    6-vector pose error: [position_error(3), orientation_error(3)].
    """
    pos_err = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3] @ T_current[:3, :3].T
    orient_err = _rotmat_to_angle_axis(R_err)
    return np.concatenate([pos_err, orient_err])


# ---------------------------------------------------------------------------
# Inverse Kinematics (damped least-squares iterative)
# ---------------------------------------------------------------------------

def ik_solve(
    T_target: np.ndarray,
    q_seed: np.ndarray,
    max_iter: int = 200,
    pos_tol: float = 1e-4,
    orient_tol: float = 1e-3,
    damping: float = 0.05,
) -> Optional[np.ndarray]:
    """
    Iterative IK using damped-least-squares (Levenberg-Marquardt style).

    Returns the joint configuration or None if no solution found.
    """
    q = np.array(q_seed, dtype=float)
    for _ in range(max_iter):
        T_cur = ur5e_fk(q)
        err = pose_error(T_cur, T_target)

        pos_err_mag = np.linalg.norm(err[:3])
        orient_err_mag = np.linalg.norm(err[3:])

        if pos_err_mag < pos_tol and orient_err_mag < orient_tol:
            # Wrap into joint limits
            q = _wrap_joints(q)
            if _in_limits(q):
                return q
            else:
                return None

        J = ur5e_jacobian(q)
        JJt = J @ J.T
        I6 = np.eye(6)
        dq = J.T @ np.linalg.solve(JJt + damping**2 * I6, err)

        # Limit step size
        step = np.linalg.norm(dq)
        if step > 0.3:
            dq *= 0.3 / step

        q += dq

    return None  # did not converge


def _wrap_joints(q: np.ndarray) -> np.ndarray:
    """Wrap joint angles into their limit ranges."""
    q = q.copy()
    for i in range(NUM_JOINTS):
        lo, hi = JOINT_LIMITS[i]
        # Wrap to [-π, π] first, then check limits
        while q[i] > math.pi:
            q[i] -= 2 * math.pi
        while q[i] < -math.pi:
            q[i] += 2 * math.pi
    return q


def _in_limits(q: np.ndarray) -> bool:
    """Check if all joints are within limits."""
    for i in range(NUM_JOINTS):
        lo, hi = JOINT_LIMITS[i]
        if q[i] < lo or q[i] > hi:
            return False
    return True


# ---------------------------------------------------------------------------
# RRT-Connect Planner
# ---------------------------------------------------------------------------

class _TreeNode:
    """Node in the RRT tree."""
    __slots__ = ('q', 'parent')
    def __init__(self, q: np.ndarray, parent: Optional['_TreeNode'] = None):
        self.q = q
        self.parent = parent


def _joint_distance(q1: np.ndarray, q2: np.ndarray) -> float:
    """Weighted L2 distance in joint space."""
    # Weight shoulder joints more (larger workspace effect)
    w = np.array([1.0, 1.0, 0.8, 0.5, 0.5, 0.3])
    return float(np.linalg.norm(w * (q1 - q2)))


def _random_config() -> np.ndarray:
    """Sample a random configuration within joint limits."""
    q = np.zeros(NUM_JOINTS)
    for i in range(NUM_JOINTS):
        lo, hi = JOINT_LIMITS[i]
        q[i] = random.uniform(lo, hi)
    return q


def _nearest(tree: List[_TreeNode], q: np.ndarray) -> _TreeNode:
    """Find the nearest node in the tree to q."""
    best = tree[0]
    best_dist = _joint_distance(best.q, q)
    for node in tree[1:]:
        d = _joint_distance(node.q, q)
        if d < best_dist:
            best = node
            best_dist = d
    return best


def _steer(q_from: np.ndarray, q_to: np.ndarray, step_size: float) -> np.ndarray:
    """Move from q_from towards q_to by at most step_size."""
    diff = q_to - q_from
    dist = _joint_distance(q_from, q_to)
    if dist <= step_size:
        return q_to.copy()
    return q_from + (step_size / dist) * diff


def _is_valid(q: np.ndarray) -> bool:
    """
    Check if a configuration is valid.
    Currently: joint limits only (no collision model).
    """
    return _in_limits(q)


def _edge_valid(q1: np.ndarray, q2: np.ndarray, resolution: float = 0.05) -> bool:
    """Check if a straight-line path in C-space is valid (discretized)."""
    dist = _joint_distance(q1, q2)
    n_steps = max(int(math.ceil(dist / resolution)), 1)
    for i in range(1, n_steps + 1):
        alpha = i / n_steps
        q_interp = q1 + alpha * (q2 - q1)
        if not _is_valid(q_interp):
            return False
    return True


_REACHED = 'reached'
_ADVANCED = 'advanced'
_TRAPPED = 'trapped'


def _extend(tree: List[_TreeNode], q_target: np.ndarray,
            step_size: float) -> Tuple[str, _TreeNode]:
    """Extend the tree towards q_target by one step."""
    near = _nearest(tree, q_target)
    q_new = _steer(near.q, q_target, step_size)

    if not _is_valid(q_new):
        return _TRAPPED, near

    if not _edge_valid(near.q, q_new):
        return _TRAPPED, near

    node = _TreeNode(q_new, near)
    tree.append(node)

    if _joint_distance(q_new, q_target) < 1e-3:
        return _REACHED, node
    return _ADVANCED, node


def _connect(tree: List[_TreeNode], q_target: np.ndarray,
             step_size: float) -> Tuple[str, _TreeNode]:
    """Greedily extend the tree towards q_target until REACHED or TRAPPED."""
    status = _ADVANCED
    node = tree[0]
    while status == _ADVANCED:
        status, node = _extend(tree, q_target, step_size)
    return status, node


def _extract_path(node: _TreeNode) -> List[np.ndarray]:
    """Walk parent pointers to extract the path from root to node."""
    path = []
    while node is not None:
        path.append(node.q.copy())
        node = node.parent
    path.reverse()
    return path


def rrt_connect(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    step_size: float = 0.2,
    max_iter: int = 5000,
    goal_bias: float = 0.1,
) -> Optional[List[np.ndarray]]:
    """
    RRT-Connect bi-directional planner in joint space.

    Parameters
    ----------
    q_start : Start configuration (6,)
    q_goal  : Goal configuration (6,)
    step_size : Max step in joint space per extend
    max_iter  : Max iterations
    goal_bias : Probability of sampling the goal instead of random

    Returns
    -------
    path : List of configurations from start to goal, or None if failed.
    """
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)

    if not _is_valid(q_start) or not _is_valid(q_goal):
        return None

    tree_a = [_TreeNode(q_start)]
    tree_b = [_TreeNode(q_goal)]

    for iteration in range(max_iter):
        # Sample random config (with goal bias)
        if random.random() < goal_bias:
            q_rand = q_goal if len(tree_a) >= len(tree_b) else q_start
        else:
            q_rand = _random_config()

        # Extend tree_a toward q_rand
        status_a, node_a = _extend(tree_a, q_rand, step_size)
        if status_a != _TRAPPED:
            # Try to connect tree_b to the new node
            status_b, node_b = _connect(tree_b, node_a.q, step_size)
            if status_b == _REACHED:
                # Trees connected — extract path
                path_a = _extract_path(node_a)
                path_b = _extract_path(node_b)
                path_b.reverse()  # b goes from connection point back to b-root

                # tree_a grows from start, tree_b from goal
                # But we swap trees, so check which is which
                if tree_a[0].q is q_start or np.allclose(tree_a[0].q, q_start):
                    return path_a + path_b
                else:
                    return path_b + path_a

        # Swap trees for balanced growth
        tree_a, tree_b = tree_b, tree_a

    return None  # failed


# ---------------------------------------------------------------------------
# Path smoothing (shortcutting)
# ---------------------------------------------------------------------------

def smooth_path(
    path: List[np.ndarray],
    max_attempts: int = 150,
) -> List[np.ndarray]:
    """
    Shortcutting-based path smoothing.
    Randomly pick two waypoints; if the direct edge is valid, remove
    the intermediate waypoints.
    """
    if len(path) <= 2:
        return path

    path = [q.copy() for q in path]  # deep copy
    for _ in range(max_attempts):
        if len(path) <= 2:
            break
        i = random.randint(0, len(path) - 3)
        j = random.randint(i + 2, len(path) - 1)
        if _edge_valid(path[i], path[j], resolution=0.02):
            path = path[:i+1] + path[j:]

    return path


# ---------------------------------------------------------------------------
# Path interpolation for execution
# ---------------------------------------------------------------------------

def interpolate_path(
    path: List[np.ndarray],
    max_step: float = 0.05,
) -> List[np.ndarray]:
    """
    Densely interpolate a path so each consecutive pair is at most
    max_step apart in joint space.
    """
    if not path:
        return path

    dense = [path[0].copy()]
    for i in range(len(path) - 1):
        q1 = path[i]
        q2 = path[i + 1]
        dist = _joint_distance(q1, q2)
        n = max(int(math.ceil(dist / max_step)), 1)
        for j in range(1, n + 1):
            alpha = j / n
            dense.append(q1 + alpha * (q2 - q1))
    return dense


# ---------------------------------------------------------------------------
# High-level: plan from current joints to a target Cartesian pose
# ---------------------------------------------------------------------------

def plan_to_pose(
    q_current: np.ndarray,
    T_target: np.ndarray,
    ik_seeds: Optional[List[np.ndarray]] = None,
    rrt_step: float = 0.2,
    rrt_max_iter: int = 5000,
    smooth_attempts: int = 150,
    interp_step: float = 0.05,
) -> Optional[List[np.ndarray]]:
    """
    Plan a joint-space path from q_current to a target Cartesian pose.

    1. Solve IK for T_target (tries multiple seeds)
    2. Run RRT-Connect from q_current to q_goal
    3. Smooth the path
    4. Interpolate densely

    Returns the dense interpolated path or None.
    """
    q_current = np.asarray(q_current, dtype=float)

    # Try IK with multiple seeds
    seeds = [q_current]
    if ik_seeds:
        seeds.extend(ik_seeds)
    # Also try some random seeds
    for _ in range(10):
        seeds.append(_random_config())

    q_goal = None
    for seed in seeds:
        result = ik_solve(T_target, seed)
        if result is not None:
            q_goal = result
            break

    if q_goal is None:
        return None  # IK failed

    # RRT-Connect
    raw_path = rrt_connect(q_current, q_goal, step_size=rrt_step,
                           max_iter=rrt_max_iter)
    if raw_path is None:
        return None  # planning failed

    # Smooth
    smoothed = smooth_path(raw_path, max_attempts=smooth_attempts)

    # Interpolate
    dense = interpolate_path(smoothed, max_step=interp_step)

    return dense
