#!/usr/bin/env python3
"""Where is the pen tip right now, against the table plane and the REGISTERED parts?

    ros2 run admittance_control touch_probe.py        # or: python3 scripts/touch_probe.py
    # freedrive the pen tip LIGHTLY onto a surface, let go (the arm holds), run it

The tip is the controller's own TCP (TF base_link -> tool0_controller, the pendant TCP
= the pen tip). It prints:
  * the height above notebooks/table_plane.json (measured by pen touches): touching the
    bare table, this is the pen-length check - ~0 if the tip is where the TCP says;
  * the signed distance to the nearest face of every registered part in assembly.json
    (+ = the real surface, where the tip is, lies OUTSIDE the registered face along its
    normal): touching a part face, this is the registration + camera-chain error in the
    robot's frame at that spot, the number the camera alone cannot see;
  * for every tack, the tip minus the planned centre and its distance off the tack.

A few touches separate the two causes of early contact (2026-09-29): the table touch
reads the pen length; touches on the base plate's top and the standing plate's faces
read the part offset along each face normal.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.geometry import quat_to_rotmat  # noqa: E402
from admittance_control.icp import load_ply_mesh, sample_mesh_surface  # noqa: E402


def _tip(node: Node) -> tuple[np.ndarray, np.ndarray]:
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if buf.can_transform('base_link', 'tool0_controller', rclpy.time.Time()):
            break
    tr = buf.lookup_transform('base_link', 'tool0_controller', rclpy.time.Time()).transform
    q = tr.rotation
    return (np.array([tr.translation.x, tr.translation.y, tr.translation.z]),
            quat_to_rotmat([q.x, q.y, q.z, q.w])[:, 2])


def main() -> int:
    rclpy.init()
    node = Node('touch_probe')
    node.declare_parameter('results_dir', str(PKG / 'scripts' / 'foundationpose_results'))
    node.declare_parameter('models_dir', str(PKG / 'models'))
    node.declare_parameter('table_plane', str(PKG / 'notebooks' / 'table_plane.json'))
    res = Path(str(node.get_parameter('results_dir').value))
    models = Path(str(node.get_parameter('models_dir').value))
    plane_file = Path(str(node.get_parameter('table_plane').value))
    try:
        tip, axis = _tip(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'pen tip (controller TCP) in base_link: {np.round(tip * 1000, 1).tolist()} mm, '
          f'pen axis {np.round(axis, 2).tolist()} ({np.degrees(np.arccos(abs(axis[2]))):.0f} deg off vertical)')

    if plane_file.exists():
        pl = json.loads(plane_file.read_text())['plane']
        h = tip[2] - (pl['a'] * tip[0] + pl['b'] * tip[1] + pl['c'])
        print(f'height above the pen-measured table plane: {h * 1000:+.1f} mm '
              f'(touching the bare table: + = the tip sits higher than the TCP says, i.e. the pen is longer)')

    asm = res / 'assembly.json'
    if asm.exists():
        rng = np.random.default_rng(0)
        print('nearest face of each registered part (+ = the touched surface is OUTSIDE the registered face):')
        for o in json.loads(asm.read_text())['objects']:
            T = np.asarray(o['pose_static'], float)
            v, f = load_ply_mesh(models / o['model'])
            p, n = sample_mesh_surface(v, f, 40000, rng, return_normals=True)
            p = p * (1e-3 if np.abs(v).max() > 5 else 1.0)
            pw, nw = p @ T[:3, :3].T + T[:3, 3], n @ T[:3, :3].T
            i = int(np.argmin(np.linalg.norm(pw - tip, axis=1)))
            d = tip - pw[i]
            off = float(d @ nw[i])
            lat = float(np.linalg.norm(d - off * nw[i]))
            print(f"   {o['model']:<22} face normal {np.round(nw[i], 2).tolist()}: {off * 1000:+6.1f} mm along it"
                  f"{'' if lat < 0.003 else f'  (tip {lat * 1000:.0f} mm beside that face - near an edge, read with care)'}")

    tk = res / 'welding_tacks.json'
    if tk.exists():
        for t in json.loads(tk.read_text())['tacks']:
            c = np.asarray(t['point_mm'], float) / 1000.0
            a, b = np.asarray(t['p0_mm'], float) / 1000.0, np.asarray(t['p1_mm'], float) / 1000.0
            L = np.linalg.norm(b - a)
            u = (b - a) / L
            s = float((tip - a) @ u)
            off = tip - (a + np.clip(s, 0, L) * u)
            d = tip - c
            if np.linalg.norm(d) < 0.08:
                print(f"tack {t['id']} (seam {t['seam_id']} #{t['tack_no']}): tip - planned centre "
                      f"{np.round(d * 1000, 1).tolist()} mm; off the tack line by {np.linalg.norm(off) * 1000:.1f} mm "
                      f"{np.round(off * 1000, 1).tolist()}, {s * 1000:.0f} of {L * 1000:.0f} mm along it")
    return 0


if __name__ == '__main__':
    sys.exit(main())
