#!/usr/bin/env python3
"""ICP pose refinement of a SAM-6D detection against the segmented scene cloud.

Idea: take the SAM-6D segmentation mask (detection_ism.npz) to cut the object
region out of the camera pointcloud, place the CAD .ply at the SAM-6D 6D pose,
and run point-to-plane ICP between the segmented scene cloud and the placed
model. Publishes both clouds + the refined pose for RViz and logs ICP metrics.

Trigger
-------
    ros2 service call /icp_pose_refiner/run_icp std_srvs/srv/Trigger

Inputs (all from the same SAM-6D capture, read fresh on each trigger)
--------------------------------------------------------------------
  <results_dir>/detection_pem.json   6D poses (R, t[mm], score, category)
  <results_dir>/detection_ism.npz    per-detection masks: segmentation (K,H,W)
  scene cloud, chosen by ``scene_from``:
    'pointcloud' : latest <pointcloud_topic> (organized sensor_msgs/PointCloud2)
    'depth_png'  : <transfer_dir>/depth.png + camera.json (the exact frame SAM-6D
                   saw -> guaranteed pixel alignment with the mask)

Publishes
---------
  <scene_cloud_topic>  sensor_msgs/PointCloud2  segmented scene (white)
  <model_cloud_topic>  sensor_msgs/PointCloud2  CAD model at the refined pose (green)
  <refined_pose_topic> geometry_msgs/PoseStamped
  TF: <camera_frame> -> <object_frame>          refined model->camera transform

The point-to-plane ICP and geometry live in admittance_control/icp.py (pure
NumPy). Metrics (fitness, inlier RMSE, correspondences, refinement delta) are
logged to the terminal on every run.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from admittance_control.geometry import rotmat_to_quat
from admittance_control.icp import (
    backproject_depth,
    estimate_normals_organized,
    icp_point_to_plane,
    load_ply_mesh,
    sample_mesh_surface,
    voxel_downsample,
)
from admittance_control.sam6d_io import resolve_transfer_dir


def make_xyzrgb_cloud(header, pts: np.ndarray, colors: np.ndarray) -> PointCloud2:
    """PointCloud2 (XYZRGB) from (N,3) float points and (N,3) uint8 colors."""
    n = len(pts)
    data = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                              ('rgb', '<f4')])
    data['x'], data['y'], data['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
    r, g, b = colors[:, 0].astype(np.uint32), colors[:, 1].astype(np.uint32), \
        colors[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    data['rgb'] = packed.view(np.float32)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


class IcpPoseRefinerNode(Node):
    def __init__(self) -> None:
        super().__init__('icp_pose_refiner')

        self.declare_parameter('results_dir', '')
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_units', 'mm')
        self.declare_parameter('scene_from', 'pointcloud')  # or 'depth_png'
        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('object_frame', 'sam6d_object')
        self.declare_parameter('scene_cloud_topic', '/perception/icp/scene_cloud')
        self.declare_parameter('model_cloud_topic', '/perception/icp/model_cloud')
        self.declare_parameter('refined_pose_topic', '/perception/icp/refined_pose')
        self.declare_parameter('detection_index', -1)   # -1 = best PEM score
        # ICP / sampling knobs.
        self.declare_parameter('n_model_points', 2500)
        self.declare_parameter('voxel_size_m', 0.006)
        self.declare_parameter('max_target_points', 15000)
        self.declare_parameter('max_corr_dist_m', 0.02)
        self.declare_parameter('max_iter', 30)

        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._object_frame = str(self.get_parameter('object_frame').value)
        self._scene_from = str(self.get_parameter('scene_from').value)
        self._det_index = int(self.get_parameter('detection_index').value)
        self._n_model = int(self.get_parameter('n_model_points').value)
        self._voxel = float(self.get_parameter('voxel_size_m').value)
        self._max_target = int(self.get_parameter('max_target_points').value)
        self._max_corr = float(self.get_parameter('max_corr_dist_m').value)
        self._max_iter = int(self.get_parameter('max_iter').value)

        results_dir = str(self.get_parameter('results_dir').value)
        self._results_dir = (Path(results_dir) if results_dir
                             else resolve_transfer_dir().parent / 'sam6d_results')
        self._transfer_dir = resolve_transfer_dir()

        # Load + sample the CAD model once (metres).
        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            raise RuntimeError('model_path parameter is required (path to .ply).')
        scale = 0.001 if str(self.get_parameter('model_units').value).lower() == 'mm' else 1.0
        verts, faces = load_ply_mesh(Path(model_path).expanduser())
        self._rng = np.random.default_rng(0)
        self._model = sample_mesh_surface(verts, faces, self._n_model, self._rng) * scale
        self.get_logger().info(
            f'model {Path(model_path).name}: {len(self._model)} pts, '
            f'extent(m)={np.round(self._model.max(0) - self._model.min(0), 3)}')

        self._latest_cloud: Optional[PointCloud2] = None
        self.create_subscription(
            PointCloud2, str(self.get_parameter('pointcloud_topic').value),
            self._on_cloud, qos_profile_sensor_data)

        # Latched: each trigger is a discrete result, so a late-joining RViz
        # still sees the last ICP output.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._scene_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('scene_cloud_topic').value), latched)
        self._model_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('model_cloud_topic').value), latched)
        self._pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('refined_pose_topic').value), latched)
        self._tf = TransformBroadcaster(self)

        self.create_service(Trigger, '~/run_icp', self._on_trigger)
        self.get_logger().info(
            f'ICP refiner ready. scene_from={self._scene_from}, '
            f'results_dir={self._results_dir}. Call ~/run_icp to refine.')

    # ── Cloud cache ──────────────────────────────────────────────────────
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    # ── Trigger ──────────────────────────────────────────────────────────
    def _on_trigger(self, request, response):
        try:
            ok, message = self._run_once()
        except Exception as exc:  # noqa: BLE001 - report any failure to caller
            self.get_logger().error(f'ICP failed: {exc}')
            response.success = False
            response.message = f'ICP failed: {exc}'
            return response
        response.success = ok
        response.message = message
        return response

    # ── Detection selection ──────────────────────────────────────────────
    def _load_detection(self):
        pem = json.loads((self._results_dir / 'detection_pem.json').read_text())
        ism = np.load(self._results_dir / 'detection_ism.npz')
        masks = ism['segmentation']  # (K,H,W)
        scores = [float(d.get('score', 0.0)) for d in pem]
        idx = int(np.argmax(scores)) if self._det_index < 0 else self._det_index
        det = pem[idx]
        R = np.array(det['R'], dtype=np.float64)
        t = np.array(det['t'], dtype=np.float64) / 1000.0  # mm -> m
        init = np.eye(4)
        init[:3, :3], init[:3, 3] = R, t
        return idx, det, init, masks[idx] > 0.5

    # ── Scene cloud (organized xyz + normals) ────────────────────────────
    def _scene_organized(self):
        if self._scene_from == 'depth_png':
            from PIL import Image
            cam = json.loads((self._transfer_dir / 'camera.json').read_text())
            K = np.array(cam['cam_K'], dtype=np.float64).reshape(3, 3)
            depth = np.array(Image.open(self._transfer_dir / 'depth.png'))
            depth_m = depth.astype(np.float64) * float(cam['depth_scale']) / 1000.0
            xyz = backproject_depth(depth_m, K)
            return xyz, estimate_normals_organized(xyz), self._camera_frame

        if self._latest_cloud is None:
            raise RuntimeError('no PointCloud2 received yet on the camera topic')
        msg = self._latest_cloud
        if msg.height <= 1:
            raise RuntimeError('cloud is not organized (height<=1); need the '
                               'depth_image_proc organized cloud for normals')
        arr = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        xyz = arr.reshape(msg.height, msg.width, 3).astype(np.float64)
        return xyz, estimate_normals_organized(xyz), msg.header.frame_id or self._camera_frame

    # ── One refinement ───────────────────────────────────────────────────
    def _run_once(self):
        idx, det, init, mask = self._load_detection()
        xyz, normals, frame_id = self._scene_organized()

        if mask.shape != xyz.shape[:2]:
            return False, (f'mask {mask.shape} does not match cloud '
                           f'{xyz.shape[:2]}; scene and SAM-6D frame differ')

        valid = mask & np.isfinite(xyz).all(2) & np.isfinite(normals).all(2)
        scene = xyz[valid]
        scene_n = normals[valid]
        if len(scene) < 10:
            return False, f'segmented scene has too few points ({len(scene)})'

        scene, scene_n = voxel_downsample(scene, self._voxel, scene_n)
        if len(scene) > self._max_target:  # cap for brute-force NN speed
            sel = self._rng.choice(len(scene), self._max_target, replace=False)
            scene, scene_n = scene[sel], scene_n[sel]

        T, info = icp_point_to_plane(
            self._model, scene, scene_n, init=init,
            max_corr_dist=self._max_corr, max_iter=self._max_iter)

        self._publish(frame_id, scene, T)
        self._log_metrics(idx, det, init, T, info, len(scene))
        return True, (f'det #{idx} refined: fitness={info["fitness"]:.3f} '
                      f'rmse={info["inlier_rmse"]:.4f}m corr={info["correspondences"]}')

    # ── Publish clouds + pose + TF ───────────────────────────────────────
    def _publish(self, frame_id, scene, T):
        stamp = self.get_clock().now().to_msg()
        header = self._make_header(frame_id, stamp)

        white = np.full((len(scene), 3), 220, dtype=np.uint8)
        self._scene_pub.publish(make_xyzrgb_cloud(header, scene, white))

        model_cam = self._model @ T[:3, :3].T + T[:3, 3]
        green = np.tile(np.array([30, 220, 60], np.uint8), (len(model_cam), 1))
        self._model_pub.publish(make_xyzrgb_cloud(header, model_cam, green))

        qx, qy, qz, qw = rotmat_to_quat(T[:3, :3])
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = \
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self._pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header = header
        tf.child_frame_id = self._object_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self._tf.sendTransform(tf)

    def _make_header(self, frame_id, stamp):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = stamp
        h.frame_id = frame_id
        return h

    def _log_metrics(self, idx, det, init, T, info, n_scene):
        dt = float(np.linalg.norm(T[:3, 3] - init[:3, 3]))
        dR = T[:3, :3] @ init[:3, :3].T
        ang = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
        self.get_logger().info(
            '── ICP point-to-plane ─────────────────────────────\n'
            f'  detection #{idx}  category={det.get("category_id")}  '
            f'score={float(det.get("score", 0.0)):.3f}\n'
            f'  scene points (after voxel/cap): {n_scene}\n'
            f'  iterations: {info["iterations"]}  converged: {info["converged"]}\n'
            f'  fitness: {info["fitness"]:.3f}   inlier_rmse: {info["inlier_rmse"]:.4f} m\n'
            f'  correspondences: {info["correspondences"]}\n'
            f'  refinement delta: translation {dt * 1000:.1f} mm  rotation {ang:.2f} deg\n'
            f'  refined t (m): [{T[0,3]:.4f}, {T[1,3]:.4f}, {T[2,3]:.4f}]\n'
            '───────────────────────────────────────────────────')


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = IcpPoseRefinerNode()
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
