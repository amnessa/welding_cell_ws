#!/usr/bin/env python3
"""Check the saved registrations against the LIVE camera cloud, face by face.

    ros2 run admittance_control check_registration.py
    # robot at (or near) the scan pose, parts untouched since save_object

For every part in foundationpose_results/assembly.json: sample its CAD faces, keep the
ones turned towards the camera now, and measure how far the live cloud lies from each
face along its normal (sensor minus model, + = the real surface is in front of the
registered one). Per face it prints the median offset and its DRIFT across the face
(the end-to-end rise of a line through the medians of 10 mm strips, along both
in-plane axes). A good fit stays under 3 mm on both (D435i noise + bias at ~0.5 m).
A plate registered straddling its own two faces (2026-09-29: the ear, 6.5 deg lean,
root 10 mm off) ramps from ~0 at one end to ~one plate thickness at the other: that
case read drift 6.3 mm here, the gated re-fit of the same cloud 1.2 mm.

Both the cloud and the registration pass through the same TF and extrinsic, so this
tests the registration only; a hand-eye error is invisible here (use the pen).
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.geometry import quat_to_rotmat  # noqa: E402
from admittance_control.icp import NNIndex, load_ply_mesh, sample_mesh_surface  # noqa: E402

GOOD_MM = 3.0          # |offset| and drift under this: D435i noise + bias at ~0.5 m;
                       # a straddled plate drifts by ~its thickness (8 mm)


def _grab(node: Node, topic: str, timeout: float = 8.0):
    msgs = []
    node.create_subscription(PointCloud2, topic, msgs.append, qos_profile_sensor_data)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    t0 = time.time()
    while time.time() - t0 < timeout and not (msgs and time.time() - t0 > 2.0):
        rclpy.spin_once(node, timeout_sec=0.05)
    if not msgs:
        raise RuntimeError(f'no cloud on {topic}')
    m = msgs[-1]
    tr = buf.lookup_transform('base_link', m.header.frame_id, rclpy.time.Time()).transform
    T = np.eye(4)
    q = tr.rotation
    T[:3, :3] = quat_to_rotmat([q.x, q.y, q.z, q.w])
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    xyz = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(float)
    xyz = xyz[np.isfinite(xyz).all(1)]
    return xyz @ T[:3, :3].T + T[:3, 3], T[:3, 3]


def main() -> int:
    rclpy.init()
    node = Node('check_registration')
    node.declare_parameter('cloud_topic', '/camera/depth/color/points')
    node.declare_parameter('results_dir', str(PKG / 'scripts' / 'foundationpose_results'))
    node.declare_parameter('models_dir', str(PKG / 'models'))
    node.declare_parameter('max_dist_m', 0.015)
    res = Path(str(node.get_parameter('results_dir').value))
    models = Path(str(node.get_parameter('models_dir').value))
    max_d = float(node.get_parameter('max_dist_m').value)
    try:
        cloud, cam = _grab(node, str(node.get_parameter('cloud_topic').value))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    assembly = json.loads((res / 'assembly.json').read_text())
    rng = np.random.default_rng(0)
    print(f'live cloud {len(cloud)} pts, camera at {np.round(cam * 1000).astype(int).tolist()} mm (base_link)')
    parts = []
    for o in assembly['objects']:
        T = np.asarray(o['pose_static'], float)
        v, f = load_ply_mesh(models / o['model'])
        p, n = sample_mesh_surface(v, f, 20000, rng, return_normals=True)
        p = p * (1e-3 if np.abs(v).max() > 5 else 1.0)
        parts.append((o['model'], p @ T[:3, :3].T + T[:3, 3], n @ T[:3, :3].T))
    worst = 0.0
    for i, (name, pw, nw) in enumerate(parts):
        facing = np.einsum('ij,ij->i', cam - pw, nw) > 0
        # live points near the part, each matched to its nearest visible model point
        lo, hi = pw.min(0) - max_d, pw.max(0) + max_d
        c = cloud[((cloud > lo) & (cloud < hi)).all(1)]
        print(f"\n{name}")
        if len(c) < 50 or facing.sum() < 50:
            print('   too few live points near it (moved, or out of view)')
            continue
        idx, d = NNIndex(pw[facing]).query(c)
        # a point that lies on ANOTHER registered part (the base under a standing
        # plate's root) belongs to that one, not to this face
        for j, (_, pj, _) in enumerate(parts):
            if j != i:
                _, dj = NNIndex(pj).query(c)
                d = np.where(dj + 0.002 < d, np.inf, d)
        ok = d < max_d
        c, idx = c[ok], idx[ok]
        vp, vn = pw[facing][idx], nw[facing][idx]
        r = c - vp
        off = np.einsum('ij,ij->i', r, vn)
        # only points lying OVER the face (their offset mostly along its normal); points
        # beside an edge - the fixture, a neighbouring part - match the edge sideways
        over = np.linalg.norm(r - off[:, None] * vn, axis=1) < 0.003
        c, vp, vn, off = c[over], vp[over], vn[over], off[over] * 1000   # mm, + = real in front
        # faces = clusters of the model normal (plates: 6 directions)
        keys = np.round(vn, 1)
        for key in np.unique(keys, axis=0):
            k = (keys == key).all(1)
            if k.sum() < 100:
                continue
            # the face's two in-plane axes (from the MODEL points it matched, so sensor
            # noise does not bend them). Drift = the end-to-end rise of the median
            # offset in 10 mm strips along each axis: a straddled plate ramps across its
            # HEIGHT by ~its thickness, a good fit stays within the sensor noise
            m = vp[k]
            _, _, vt = np.linalg.svd(m - m.mean(0), full_matrices=False)
            s = (m - m.mean(0)) @ vt[:2].T * 1000                    # mm, (n, 2)
            ext = s.max(0) - s.min(0)
            if ext.min() < 20:                                        # a plate edge
                continue
            drift = 0.0
            for ax in range(2):
                strip = np.floor((s[:, ax] - s[:, ax].min()) / 10).astype(int)
                # the outermost strip on each side is dropped: mixed pixels at the
                # edges and beside a neighbouring part read several mm off
                inner = [b for b in np.unique(strip)[1:-1] if (strip == b).sum() >= 50]
                if len(inner) >= 3:
                    # a straddle is a RAMP: the rise of a line through the strip medians
                    # (one odd strip - mixed pixels at a neighbour's foot - barely moves it)
                    meds = [np.median(off[k][strip == b]) for b in inner]
                    slope = np.polyfit(np.asarray(inner, float) * 10, meds, 1)[0]
                    drift = max(drift, abs(float(slope)) * 10 * (inner[-1] - inner[0]))
            med = float(np.median(off[k]))
            worst = max(worst, abs(med), drift)
            flag = 'ok' if abs(med) < GOOD_MM and drift < GOOD_MM else 'CHECK'
            print(f'   face normal {str(np.round(key, 1).tolist()):<22} {k.sum():6d} pts '
                  f'({ext[0]:.0f} x {ext[1]:.0f} mm seen): offset {med:+5.1f} mm, drift across it {drift:4.1f} mm  {flag}')
    print(f"\n{'OK' if worst < GOOD_MM else 'CHECK'}: largest face offset/drift {worst:.1f} mm "
          f"(pass under {GOOD_MM:.0f} mm; a straddled plate drifts by ~its thickness)")
    return 0 if worst < GOOD_MM else 1


if __name__ == '__main__':
    sys.exit(main())
