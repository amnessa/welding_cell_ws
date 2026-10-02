#!/usr/bin/env python3
"""The ground-truth distance for the RealSense Tare calibration, from the robot.

    ros2 run admittance_control tare_distance.py        # or: python3 scripts/tare_distance.py
    # the camera looking straight down at the bare table, over the pen-touched patch

Tare (realsense-viewer -> More -> Calibration -> Tare) asks for the true distance from the
camera to a flat target. A tape measure gives it from the front glass, but the D435/D435i
depth origin sits 4.2 mm behind the glass: a few mm wrong at 0.3-0.6 m is a 1 % depth-scale
error (2026-10-02: after a Tare, the camera read the table +1.7 / +3.3 / +5.2 mm high at
0.29 / 0.42 / 0.60 m - linear, 11 mm per metre).

This computes it in the robot's own frame instead: the camera pose from /joint_states
through the CALIBRATED kinematics and the extrinsic file (no TF needed - the perception
launch must be stopped anyway, realsense-viewer cannot open the camera while the ROS node
holds it), and the table from notebooks/table_plane.json (pen touches). It prints the
depth along the optical axis to the table (enter that), the tilt of the camera to the
table (Tare wants the camera parallel to the target: keep it under ~1 deg), and how far
the optical axis lands from the touched patch (the plane is exact there, extrapolated
elsewhere).

The depth is to the colour optical frame's origin, which is what the extrinsic and so the
whole pipeline use; the D435i's depth origin (the left imager) lies in the same plane,
15 mm to the side, so the distance along the axis is the same to well under 0.5 mm.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.kinematics import active_kinematics, ur5e_fk, use_kinematics  # noqa: E402


def read_joints(timeout: float = 5.0) -> np.ndarray:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from admittance_control.tack_reach import UR_ORDER, joint_state_to_ur_order
    rclpy.init()
    node = Node("tare_distance")
    got = {}

    def cb(m):
        if all(n in m.name for n in UR_ORDER):
            got["q"] = joint_state_to_ur_order(m.name, m.position)
    node.create_subscription(JointState, "/joint_states", cb, qos_profile_sensor_data)
    t0 = time.time()
    while "q" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if "q" not in got:
        raise SystemExit("no /joint_states (is the UR driver running?)")
    return got["q"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--extrinsic", default=str(PKG / "notebooks" / "T_tcp_to_cam.npy"))
    ap.add_argument("--plane", default=str(PKG / "notebooks" / "table_plane.json"))
    ap.add_argument("--kinematics", default=str(PKG / "config" / "ur5e_calibration.yaml"))
    args = ap.parse_args()

    use_kinematics(args.kinematics)
    q = read_joints()
    T_cam = ur5e_fk(q) @ np.load(args.extrinsic)
    c, z = T_cam[:3, 3], T_cam[:3, 2]
    pl = json.loads(Path(args.plane).read_text())
    n = np.asarray(pl["plane"]["normal"], float)
    n = n / np.linalg.norm(n) * np.sign(n[2])
    p0 = np.asarray(pl["plane"]["centroid_m"], float)
    cos = float(-(z @ n))                                 # the optical axis points down at the table
    if cos <= 0.1:
        print("the camera is not looking at the table")
        return 1
    perp = float((c - p0) @ n)                            # camera height above the plane
    depth = perp / cos                                    # along the optical axis
    hit = c + depth * z
    tilt = float(np.degrees(np.arccos(min(1.0, cos))))
    off_patch = float(np.linalg.norm((hit - p0) - ((hit - p0) @ n) * n))
    print(f"kinematics: {active_kinematics()}\nextrinsic:  {args.extrinsic}\nplane:      {args.plane} "
          f"(written {pl.get('written', '?')})")
    print(f"camera {np.round(c * 1000, 1).tolist()} mm, optical axis {np.round(z, 3).tolist()}")
    print(f"\n  GROUND TRUTH FOR TARE: {depth * 1000:.1f} mm   (depth along the optical axis to the table)")
    print(f"  camera tilt to the table: {tilt:.2f} deg"
          + ("" if tilt < 1.0 else "   <- over 1 deg: turn the camera to look straight down first"))
    print(f"  the optical axis meets the table {off_patch * 1000:.0f} mm from the touched patch"
          + ("" if off_patch < 0.08 else "   <- far from it: the plane is extrapolated there, move over the patch"))
    print("\nKeep the arm still from here: stop the perception launch, open realsense-viewer, run Tare"
          " with that distance, close the viewer, restart the launch.")
    return 0 if tilt < 1.0 and off_patch < 0.08 else 2


if __name__ == "__main__":
    sys.exit(main())
