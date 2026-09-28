#!/usr/bin/env python3
"""Check the camera extrinsic against the table the PEN measured - no ChArUco.

    ros2 run admittance_control extrinsic_check.py
    # empty table; move the arm to a view that sees the touched patch (scan home works)
    ros2 service call /extrinsic_check/capture std_srvs/srv/Trigger   # 3-5 views, different angles
    ros2 service call /extrinsic_check/report  std_srvs/srv/Trigger
    ros2 service call /extrinsic_check/clear   std_srvs/srv/Trigger

To tell a CAMERA error from the TABLE: capture one spot with the wrist turned 0/90/180/270
deg about the vertical. A tilt from the extrinsic turns with the camera; the table's own
does not. The report splits the two (`table_probe.separate_tilt`) once the yaws span 90+.

The ChArUco residual says whether a calibration fits ITS OWN samples; it cannot say
whether the camera is where the file says (two calibrations that each fit to 3 mm
disagreed by 20 mm on 2026-09-28). The table plane measured by pen touches
(`notebooks/table_plane.json`, RMSE 0.2 mm) is an independent truth: the camera's
depth points of that same table, taken to base_link through TF (i.e. through the
extrinsic in use), must land on it.

Per capture: the DOMINANT flat surface in view within `search_m` (25 cm) of the pen
plane - found by a height histogram, so a large extrinsic error is measured rather than
filtered out - anywhere on the table (`radius_m` > 0 restricts it to the touched patch),
a robust plane fit, and its difference to the pen plane extended to the same spot:
HEIGHT offset (mm) and TILT between the normals (deg, with its direction); the distance
from the touched patch is reported, since far from it the pen plane is extrapolated.
What they mean:
    tilt that changes with the view  -> extrinsic ROTATION error (it tips the camera's
                                        picture of the table differently per view)
    tilt the same in every view      -> the table plane file or a depth-sensor tilt
    height offset, same in all views -> depth bias or extrinsic translation along the
                                        view axis; changing with the view -> rotation
A good extrinsic: tilt < 0.3 deg and offsets within ~2 mm, in every view. Note the
in-plane (yaw) rotation and horizontal translation are invisible to a plane; the pen
marks are the check for those.
"""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from admittance_control.geometry import quat_to_rotmat  # noqa: E402
from admittance_control.table_probe import separate_tilt, surface_vs_plane  # noqa: E402


class ExtrinsicCheck(Node):
    def __init__(self) -> None:
        super().__init__('extrinsic_check')
        self.declare_parameter('cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('plane_file', str(PKG / 'notebooks' / 'table_plane.json'))
        self.declare_parameter('radius_m', 0.0)         # 0 = anywhere; > 0 = only near the touched patch
        self.declare_parameter('search_m', 0.25)        # look for the table within +-this of the pen plane
        self.declare_parameter('band_m', 0.008)         # points kept around the table found
        # D435i minimum depth: ~0.28 m at 1280x720 (the camera node's default), ~0.20 m
        # at 848x480. Closer, the depth is sparse and scattered (seen 2026-09-28).
        self.declare_parameter('min_range_m', 0.28)
        self.declare_parameter('out', str(PKG / 'notebooks' / 'extrinsic_check.json'))
        pl = json.loads(Path(str(self.get_parameter('plane_file').value)).read_text())['plane']
        self._n = np.asarray(pl['normal'], float); self._n /= np.linalg.norm(self._n)
        self._p0 = np.asarray(pl['centroid_m'], float); self._p0[2] = pl['z_at_centroid_m']
        self._cloud = None
        self._lock = threading.Lock()
        self._rows = []
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self.create_subscription(PointCloud2, str(self.get_parameter('cloud_topic').value),
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_service(Trigger, '~/capture', self._capture)
        self.create_service(Trigger, '~/report', self._report)
        self.create_service(Trigger, '~/clear', self._clear)
        self.get_logger().info(f'truth: table plane at z {self._p0[2] * 1000:.2f} mm around '
                               f'({self._p0[0]:.3f}, {self._p0[1]:.3f}); ~/capture from several views')

    def _on_cloud(self, msg):
        with self._lock:
            self._cloud = msg

    def _capture(self, request, response):
        with self._lock:
            msg = self._cloud
        if msg is None:
            response.success, response.message = False, 'no cloud yet'
            return response
        try:
            tf = self._tf.lookup_transform(str(self.get_parameter('base_frame').value),
                                           msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            try:
                tf = self._tf.lookup_transform(str(self.get_parameter('base_frame').value),
                                               msg.header.frame_id, Time())
            except TransformException as exc:
                response.success, response.message = False, f'no TF: {exc}'
                return response
        q, t = tf.transform.rotation, tf.transform.translation
        R = quat_to_rotmat([q.x, q.y, q.z, q.w]); tv = np.array([t.x, t.y, t.z])
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.asarray(pts, float).reshape(-1, 3) @ R.T + tv
        out = surface_vs_plane(pts, self._n, self._p0,
                               search_m=float(self.get_parameter('search_m').value),
                               band_m=float(self.get_parameter('band_m').value),
                               radius_m=float(self.get_parameter('radius_m').value))
        if 'error' in out:
            response.success = False
            response.message = out['error'] + ' - is the table in view? if the extrinsic is very wrong, raise -p search_m'
            return response
        rng = float(np.linalg.norm(np.asarray(out['surface_centre_m']) - tv))
        if rng < float(self.get_parameter('min_range_m').value):
            response.success = False
            response.message = (f'camera is {rng * 1000:.0f} mm from the table: below the D435i minimum depth '
                                f"({float(self.get_parameter('min_range_m').value) * 1000:.0f} mm at 1280x720), "
                                f'the depth there is scattered - move back to 300-500 mm and capture again')
            return response
        offset, tilt, az = out['height_offset_mm'], out['tilt_deg'], out['tilt_azimuth_deg']
        centre, extrap = np.asarray(out['surface_centre_m']), out['extrapolation_m']
        # camera yaw: its optical x axis projected on the table (turns with the wrist)
        xp = R[:, 0] - (R[:, 0] @ self._n) * self._n
        yaw = float(np.degrees(np.arctan2(xp[1], xp[0])))
        row = {'time': time.time(), **out, 'camera_position_m': tv.tolist(), 'view_axis': R[:, 2].tolist(),
               'range_m': rng, 'camera_yaw_deg': yaw, 'camera_R': R.tolist()}
        self._rows.append(row)
        response.success = True
        response.message = (f"view {len(self._rows)}: camera sees the table {offset:+.2f} mm "
                            f"{'above' if offset > 0 else 'below'} the pen-measured plane, tilted {tilt:.2f} deg "
                            f"(toward az {az:.0f}), {out['n_points']} pts, fit RMSE {out['plane_rmse_mm']:.2f} mm, at "
                            f"({centre[0]:.3f}, {centre[1]:.3f}) = {extrap * 1000:.0f} mm from the touched patch, range {rng * 1000:.0f} mm, camera yaw {yaw:.0f} deg"
                            + (' (far: the pen plane is extrapolated, trust the tilt more than the offset)'
                               if extrap > 0.3 else ''))
        self.get_logger().info(response.message)
        return response

    def _clear(self, request, response):
        self._rows = []
        response.success, response.message = True, 'captures cleared'
        return response

    def _report(self, request, response):
        if not self._rows:
            response.success, response.message = False, 'no captures'
            return response
        off = np.array([r['height_offset_mm'] for r in self._rows])
        tilt = np.array([r['tilt_deg'] for r in self._rows])
        Path(str(self.get_parameter('out').value)).write_text(json.dumps(
            {'plane_file': str(self.get_parameter('plane_file').value), 'views': self._rows}, indent=1))
        sep = separate_tilt(tilt, [r['tilt_azimuth_deg'] for r in self._rows],
                            [r['camera_yaw_deg'] for r in self._rows])
        # judge the CAMERA: when the yaws allow the split, only the camera-fixed tilt says
        # anything about the extrinsic; the world-fixed part is the table where it looked
        good = (sep['camera_tilt_deg'] < 0.15) if sep['separable'] else (tilt.max() < 0.3 and np.abs(off).max() < 2.0)
        if sep['separable']:
            split = (f" Tilt split: CAMERA-fixed {sep['camera_tilt_deg']:.2f} deg (extrinsic rotation / depth "
                     f"sensor), WORLD-fixed {sep['world_tilt_deg']:.2f} deg (the table there vs the pen plane), "
                     f"residual {sep['residual_deg']:.2f} deg over a {sep['yaw_spread_deg']:.0f} deg yaw spread.")
        else:
            split = (f" Camera vs table NOT separable: the camera yaw only spans {sep['yaw_spread_deg']:.0f} deg - "
                     f"capture the SAME spot with the wrist turned 90 and 180 deg (and 270) about the vertical.")
        response.success = True
        response.message = (f"{len(off)} views: height offset {off.mean():+.2f} mm mean, "
                            f"{np.ptp(off):.2f} mm spread; tilt {tilt.mean():.2f} deg mean, max {tilt.max():.2f}. "
                            + (('EXTRINSIC ROTATION OK (camera-fixed tilt < 0.15 deg).' if good else
                                'EXTRINSIC ROTATION OFF (camera-fixed tilt >= 0.15 deg): refine it with '
                                'helper/calibration/refine_extrinsic_from_table.py.') if sep['separable'] else
                               ('CONSISTENT WITH THE PEN.' if good else 'NOT within 0.3 deg / 2 mm (single yaw: '
                                'camera and table not separated).'))
                            + split + f" Wrote {self.get_parameter('out').value}")
        self.get_logger().info(response.message)
        return response


def main() -> None:
    rclpy.init()
    node = ExtrinsicCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
