#!/usr/bin/env python3
"""
PD Auto-Tuner — Coordinate Descent optimiser for Kp / Kd gains.

Repeatedly launches the sand_drawer velocity pipeline (trajectory mode with
a line trajectory), monitors EE tracking error + command jiggle, grades the
run, then tweaks Kp / Kd using coordinate descent.

Usage
-----
  cd ~/…/ros2_ws && source install/setup.bash
  python3 src/sand_drawer/scripts/pd_auto_tuner.py          # defaults
  python3 src/sand_drawer/scripts/pd_auto_tuner.py \
      --kp_init 1.5 --kd_init 0.0 --step_kp 0.3 --step_kd 0.05 \
      --max_iters 40 --jiggle_weight 0.5 --timeout 120

The tuner is a *standalone* Python script (not a ROS node).  It launches
the full sand_drawer.launch.py via subprocess, monitors /end_effector_velocity
and the actual EE TF, grades the run, kills the process, and repeats.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

# ---- ROS 2 imports (we spin a lightweight context for subscribers) ----
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np


# ======================================================================
# Data containers
# ======================================================================
@dataclass
class RunRecord:
    """Stores samples collected during a single trial run."""
    ee_positions: List[np.ndarray] = field(default_factory=list)
    target_positions: List[np.ndarray] = field(default_factory=list)
    twist_cmds: List[np.ndarray] = field(default_factory=list)  # 6-vec
    timestamps: List[float] = field(default_factory=list)
    tracking_errors: List[float] = field(default_factory=list)
    phase: str = "UNKNOWN"
    trajectory_done: bool = False
    boundary_violation: bool = False
    abort: bool = False


@dataclass
class TrialResult:
    kp: float
    kd: float
    cost: float
    rmse: float
    jiggle: float
    aborted: bool = False


# ======================================================================
# Monitor Node (spun inside the tuner process)
# ======================================================================
class TunerMonitorNode(Node):
    """
    Lightweight ROS 2 node that subscribes to the topics published during a
    sand_drawer run and collects data into a RunRecord.
    """

    def __init__(self, record: RunRecord, plane_json: str):
        super().__init__('pd_auto_tuner_monitor')
        self.record = record

        # Parse plane geometry from the JSON
        import json
        with open(plane_json, 'r') as f:
            data = json.load(f)
        plane = data['plane']
        self.plane_origin = np.array(plane['origin'], dtype=float)
        self.plane_x = np.array(plane['x_axis'], dtype=float)
        self.plane_y = np.array(plane['y_axis'], dtype=float)
        corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
        pxs, pys = [], []
        for c in corners:
            rel = c - self.plane_origin
            pxs.append(float(np.dot(rel, self.plane_x)))
            pys.append(float(np.dot(rel, self.plane_y)))
        self.bounds = dict(x_min=min(pxs), x_max=max(pxs),
                           y_min=min(pys), y_max=max(pys))
        self.boundary_margin = 0.02  # m

        # TF2
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        # Subscribers
        self.create_subscription(Twist, '/end_effector_velocity',
                                 self._twist_cb, 10)

        # We also watch the servo controller's log via /rosout — but it's
        # simpler to just detect phase transitions by watching whether
        # twist commands are being published.
        self._servo_started = False
        self._servo_start_time: Optional[float] = None
        self._last_twist_time: Optional[float] = None
        self._zero_twist_streak = 0

        # 20 Hz sampling timer
        self.create_timer(0.05, self._sample)

    # ---- callbacks ----
    def _twist_cb(self, msg: Twist):
        v = np.array([msg.linear.x, msg.linear.y, msg.linear.z,
                       msg.angular.x, msg.angular.y, msg.angular.z])
        self.record.twist_cmds.append(v)
        self.record.timestamps.append(time.monotonic())
        self._last_twist_time = time.monotonic()

        # Detect servo phase: non-trivial twist
        lin_mag = np.linalg.norm(v[:3])
        if lin_mag > 0.001:
            if not self._servo_started:
                self._servo_started = True
                self._servo_start_time = time.monotonic()
            self._zero_twist_streak = 0
        else:
            if self._servo_started:
                self._zero_twist_streak += 1
                # If we've had 20 consecutive zero twists (~2 s at 10 Hz publish)
                # after servo started, assume trajectory is done
                if self._zero_twist_streak >= 20:
                    self.record.trajectory_done = True

    def _sample(self):
        """Periodic: record EE position and check boundaries."""
        try:
            t = self.tf_buf.lookup_transform('base_link', 'tool0',
                                              rclpy.time.Time())
            pos = np.array([t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z])
            self.record.ee_positions.append(pos)

            # Check plane boundary violation
            rel = pos - self.plane_origin
            px = float(np.dot(rel, self.plane_x))
            py = float(np.dot(rel, self.plane_y))
            m = self.boundary_margin
            if (px < self.bounds['x_min'] - m or
                px > self.bounds['x_max'] + m or
                py < self.bounds['y_min'] - m or
                    py > self.bounds['y_max'] + m):
                # Only flag violation when the robot is actually servoing on-plane
                if self._servo_started:
                    self.record.boundary_violation = True

        except Exception:
            pass  # TF not ready yet


# ======================================================================
# Launch helper
# ======================================================================
def launch_trial(kp: float, kd: float, kp_ang: float, kd_ang: float,
                 extra_args: dict) -> subprocess.Popen:
    """Start sand_drawer.launch.py as a subprocess with given gains."""
    cmd = [
        'ros2', 'launch', 'sand_drawer', 'sand_drawer.launch.py',
        'mode:=trajectory',
        f'trajectory_key:=line',
        f'kp_linear:={kp:.6f}',
        f'kd_linear:={kd:.6f}',
        f'kp_angular:={kp_ang:.6f}',
        f'kd_angular:={kd_ang:.6f}',
        f'loop:=false',
    ]
    for k, v in extra_args.items():
        cmd.append(f'{k}:={v}')

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,  # New process group so we can kill the tree
    )
    return proc


def kill_trial(proc: subprocess.Popen):
    """Kill the entire process group (launch + children)."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=5)


# ======================================================================
# Cost calculation
# ======================================================================
def compute_cost(record: RunRecord, jiggle_weight: float) -> TrialResult:
    """
    Cost = RMSE(tracking error) + jiggle_weight * acceleration_penalty
    """
    kp = kd = 0.0  # caller fills these in

    if record.abort or record.boundary_violation:
        return TrialResult(kp=0, kd=0, cost=99999.0, rmse=99.0,
                           jiggle=99.0, aborted=True)

    # ---- Tracking RMSE ----
    errs = record.tracking_errors
    if not errs:
        # fallback: can't compute tracking without waypoint info — use twist
        # magnitude as proxy (lower = less movement needed)
        errs = [0.0]
    rmse = math.sqrt(sum(e * e for e in errs) / max(len(errs), 1))

    # ---- Jiggle (command acceleration) penalty ----
    twists = record.twist_cmds
    jiggle = 0.0
    if len(twists) >= 3:
        for i in range(1, len(twists) - 1):
            # Finite-difference second derivative (acceleration)
            accel = twists[i + 1] - 2 * twists[i] + twists[i - 1]
            jiggle += float(np.dot(accel, accel))
        jiggle /= (len(twists) - 2)

    cost = rmse + jiggle_weight * jiggle
    return TrialResult(kp=kp, kd=kd, cost=cost, rmse=rmse, jiggle=jiggle)


# ======================================================================
# Tracking error collection (from logged twist + EE position)
# ======================================================================
def collect_tracking_errors(record: RunRecord, plane_json: str,
                            line_u_start: float, line_v_start: float,
                            line_u_end: float, line_v_end: float):
    """
    After a run finishes, estimate the tracking error at each EE sample.

    We interpolate the nominal line endpoint positions along the timing
    of when the servo started / finished.
    """
    import json
    with open(plane_json, 'r') as f:
        data = json.load(f)
    corners = [np.array(c, dtype=float) for c in data['rectangle_corners']]
    rect_origin = corners[0]
    rect_w = corners[1] - corners[0]
    rect_h = corners[3] - corners[0]

    p_start = rect_origin + line_u_start * rect_w + line_v_start * rect_h
    p_end   = rect_origin + line_u_end   * rect_w + line_v_end   * rect_h

    # We approximate: the servo controller is active for len(waypoints) steps
    # Each EE position sample during servo is compared to the nearest point
    # on the line segment.
    positions = record.ee_positions
    if not positions:
        return

    line_vec = p_end - p_start
    line_len = np.linalg.norm(line_vec)
    if line_len < 1e-6:
        return

    for p in positions:
        # Project onto line
        t = float(np.dot(p - p_start, line_vec)) / (line_len * line_len)
        t = max(0.0, min(1.0, t))
        closest = p_start + t * line_vec
        err = float(np.linalg.norm(p - closest))
        record.tracking_errors.append(err)


# ======================================================================
# Coordinate Descent
# ======================================================================
def coordinate_descent(args):
    """
    Main loop: iterate over [Kp, Kd], perturb each by ±step, keep the
    gain that lowers cost, shrink step if no improvement.
    """
    print("=" * 70)
    print("  PD Auto-Tuner — Coordinate Descent")
    print("=" * 70)

    # ---- Gains being tuned ----
    gains = [args.kp_init, args.kd_init]
    steps = [args.step_kp, args.step_kd]
    labels = ['Kp', 'Kd']

    # Fixed angular gains (tuned separately or left as-is)
    kp_ang = args.kp_angular
    kd_ang = args.kd_angular

    # Clamp ranges
    gain_min = [0.1, 0.0]
    gain_max = [10.0, 5.0]

    min_step = [0.01, 0.005]

    # Extra launch args
    extra = {}
    if args.line_u_start is not None:
        extra['line_u_start'] = f'{args.line_u_start}'
    if args.line_v_start is not None:
        extra['line_v_start'] = f'{args.line_v_start}'
    if args.line_u_end is not None:
        extra['line_u_end'] = f'{args.line_u_end}'
    if args.line_v_end is not None:
        extra['line_v_end'] = f'{args.line_v_end}'

    line_u_start = float(extra.get('line_u_start', '0.5'))
    line_v_start = float(extra.get('line_v_start', '0.3'))
    line_u_end   = float(extra.get('line_u_end', '0.5'))
    line_v_end   = float(extra.get('line_v_end', '0.7'))

    plane_json = args.plane_json

    # CSV log
    log_path = args.log_file
    log_f = open(log_path, 'w', newline='')
    log_w = csv.writer(log_f)
    log_w.writerow(['iter', 'param', 'direction', 'Kp', 'Kd', 'cost',
                     'rmse', 'jiggle', 'aborted', 'accepted'])

    def run_trial(kp, kd) -> TrialResult:
        """Execute one full simulation trial and return its cost."""
        print(f"  Launching trial: Kp={kp:.4f}  Kd={kd:.4f} …")

        # 1. Init ROS context + monitor node
        rclpy.init()
        record = RunRecord()
        monitor = TunerMonitorNode(record, plane_json)
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)

        # Spin in background thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 2. Launch the simulation
        proc = launch_trial(kp, kd, kp_ang, kd_ang, extra)

        # 3. Wait for completion or timeout
        t0 = time.monotonic()
        timeout = args.timeout
        try:
            while time.monotonic() - t0 < timeout:
                time.sleep(0.5)

                # Abort conditions
                if record.boundary_violation:
                    print("    !! Boundary violation → aborting run")
                    record.abort = True
                    break
                if record.trajectory_done:
                    print("    Trajectory complete, collecting final data…")
                    time.sleep(2.0)  # gather a bit more
                    break
            else:
                print(f"    Timeout ({timeout}s) → ending run")
        finally:
            # 4. Kill launch
            kill_trial(proc)
            time.sleep(1.0)

            # 5. Shutdown ROS
            try:
                executor.shutdown()
                monitor.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

        # 6. Compute tracking errors from EE positions
        collect_tracking_errors(record, plane_json,
                                line_u_start, line_v_start,
                                line_u_end, line_v_end)

        # 7. Compute cost
        result = compute_cost(record, args.jiggle_weight)
        result.kp = kp
        result.kd = kd
        return result

    # ---- Evaluate baseline ----
    print(f"\n{'='*50}")
    print(f"  Baseline: Kp={gains[0]:.4f}  Kd={gains[1]:.4f}")
    print(f"{'='*50}")
    best = run_trial(gains[0], gains[1])
    best_cost = best.cost
    print(f"  Baseline cost = {best_cost:.6f}  "
          f"RMSE={best.rmse:.6f}  jiggle={best.jiggle:.6f}  "
          f"aborted={best.aborted}")
    log_w.writerow([0, 'baseline', '-', gains[0], gains[1],
                     best.cost, best.rmse, best.jiggle, best.aborted, True])
    log_f.flush()

    history: List[TrialResult] = [best]

    # ---- Coordinate descent iterations ----
    for iteration in range(1, args.max_iters + 1):
        improved_any = False

        for dim in range(len(gains)):
            label = labels[dim]
            step = steps[dim]

            if step < min_step[dim]:
                print(f"  [{label}] step below minimum ({step:.4f} < "
                      f"{min_step[dim]:.4f}), skipping")
                continue

            for direction in [+1, -1]:
                trial_gains = list(gains)
                trial_gains[dim] = np.clip(
                    trial_gains[dim] + direction * step,
                    gain_min[dim], gain_max[dim])

                # Skip if it didn't actually change
                if abs(trial_gains[dim] - gains[dim]) < 1e-8:
                    continue

                dir_label = '+' if direction > 0 else '-'
                print(f"\n--- Iter {iteration}  {label}{dir_label}  "
                      f"Kp={trial_gains[0]:.4f}  Kd={trial_gains[1]:.4f} ---")

                result = run_trial(trial_gains[0], trial_gains[1])
                print(f"  cost={result.cost:.6f}  RMSE={result.rmse:.6f}  "
                      f"jiggle={result.jiggle:.6f}  aborted={result.aborted}")

                accepted = False
                if result.cost < best_cost:
                    print(f"  ** IMPROVED ** ({best_cost:.6f} → "
                          f"{result.cost:.6f})")
                    best_cost = result.cost
                    gains = trial_gains
                    improved_any = True
                    accepted = True
                else:
                    print(f"  No improvement (best={best_cost:.6f})")

                log_w.writerow([iteration, label, dir_label,
                                 trial_gains[0], trial_gains[1],
                                 result.cost, result.rmse, result.jiggle,
                                 result.aborted, accepted])
                log_f.flush()
                history.append(result)

                if accepted:
                    break  # accept first improvement for this dim

        if not improved_any:
            # Shrink all step sizes
            for dim in range(len(steps)):
                steps[dim] *= 0.5
            print(f"\n  No improvement this round — shrinking steps: "
                  f"Kp_step={steps[0]:.4f}  Kd_step={steps[1]:.4f}")
            if all(s < m for s, m in zip(steps, min_step)):
                print("  All steps below minimum — CONVERGED.")
                break

    # ---- Final report ----
    log_f.close()
    print("\n" + "=" * 70)
    print("  TUNING COMPLETE")
    print(f"  Best gains:  Kp = {gains[0]:.4f}   Kd = {gains[1]:.4f}")
    print(f"  Best cost:   {best_cost:.6f}")
    print(f"  Log saved:   {log_path}")
    print("=" * 70)
    print(f"\n  Use these gains:\n")
    print(f"    ros2 launch sand_drawer sand_drawer.launch.py "
          f"mode:=trajectory trajectory_key:=line "
          f"kp_linear:={gains[0]:.4f} kd_linear:={gains[1]:.4f}")
    print()

    return gains


# ======================================================================
# Entry point
# ======================================================================
def main():
    parser = argparse.ArgumentParser(
        description='PD Auto-Tuner for sand_drawer velocity controller')
    parser.add_argument('--kp_init', type=float, default=1.5,
                        help='Initial proportional gain')
    parser.add_argument('--kd_init', type=float, default=0.0,
                        help='Initial derivative gain')
    parser.add_argument('--step_kp', type=float, default=0.3,
                        help='Initial step size for Kp')
    parser.add_argument('--step_kd', type=float, default=0.05,
                        help='Initial step size for Kd')
    parser.add_argument('--kp_angular', type=float, default=1.5,
                        help='Fixed angular P gain')
    parser.add_argument('--kd_angular', type=float, default=0.0,
                        help='Fixed angular D gain')
    parser.add_argument('--max_iters', type=int, default=30,
                        help='Maximum coordinate descent iterations')
    parser.add_argument('--jiggle_weight', type=float, default=0.5,
                        help='Weight for jiggle penalty in cost function')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='Max seconds per trial run')
    parser.add_argument('--plane_json', type=str, default=None,
                        help='Path to plane JSON file')
    parser.add_argument('--log_file', type=str, default='pd_tuner_log.csv',
                        help='CSV log output file')
    # Line UV
    parser.add_argument('--line_u_start', type=float, default=0.5)
    parser.add_argument('--line_v_start', type=float, default=0.3)
    parser.add_argument('--line_u_end', type=float, default=0.5)
    parser.add_argument('--line_v_end', type=float, default=0.7)

    args = parser.parse_args()

    # Default plane JSON path
    if args.plane_json is None:
        script_dir = os.path.dirname(os.path.realpath(__file__))
        args.plane_json = os.path.join(
            script_dir, '..', 'generated_planes', 'sand_drawer_plane.json')
    args.plane_json = os.path.realpath(args.plane_json)

    if not os.path.exists(args.plane_json):
        print(f"ERROR: Plane JSON not found: {args.plane_json}")
        sys.exit(1)

    coordinate_descent(args)


if __name__ == '__main__':
    main()
