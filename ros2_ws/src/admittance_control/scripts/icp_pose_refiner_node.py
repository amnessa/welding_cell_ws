#!/usr/bin/env python3
"""Real-time ICP object tracking, seeded by a SAM-6D detection.

Two phases, following notes/realtime_icp.md:

Phase 1 - Initialization (``~/run_icp``, runs once per object)
    Take the SAM-6D segmentation mask (detection_ism.npz) to cut the object
    region out of the camera pointcloud, place the CAD .ply at the SAM-6D 6D
    pose, and run point-to-plane ICP to get the initial pose T0. This seeds the
    tracker's ``current_pose`` and (if ``auto_track``) starts Phase 2.

Phase 2 - Tracking loop (timer at ``tracking_rate_hz``, no SAM-6D)
    Each tick: grab the live organized cloud, build a **dynamic CropBox** around
    ``current_pose`` (the model AABB + ``crop_margin_m``) to isolate the object
    -- this replaces the SAM-6D mask -- then run **Fast & Robust ICP**
    (Welsch-reweighted, dynamic-nu, Anderson-accelerated point-to-plane;
    ``robust`` + ``anderson_depth``) from ``current_pose`` and update it. The
    Welsch kernel smoothly rejects a gripper/fixture that enters the crop box. The crop box is published as an RViz Marker so you can watch it
    follow the object as you move it. If ICP fitness drops below
    ``lost_fitness`` (object escaped the box) tracking pauses; re-run SAM-6D and
    call ``~/run_icp`` again to re-seed.

Triggers
--------
    ros2 service call /icp_pose_refiner/run_icp        std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/stop_tracking  std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/start_tracking std_srvs/srv/Trigger

Init inputs (from the SAM-6D capture, read fresh on run_icp)
-----------------------------------------------------------
  <results_dir>/detection_pem.json   6D poses (R, t[mm], score, category)
  <results_dir>/detection_ism.npz    per-detection masks: segmentation (K,H,W)
  scene cloud, chosen by ``scene_from``:
    'pointcloud' : latest <pointcloud_topic> (organized sensor_msgs/PointCloud2)
    'depth_png'  : <transfer_dir>/depth.png + camera.json (the exact frame SAM-6D
                   saw -> guaranteed pixel alignment with the mask)

Publishes
---------
  <scene_cloud_topic>  sensor_msgs/PointCloud2  segmented/cropped scene (white)
  <model_cloud_topic>  sensor_msgs/PointCloud2  CAD model at the refined pose (green)
  <refined_pose_topic> geometry_msgs/PoseStamped
  <crop_box_topic>     visualization_msgs/Marker  the dynamic CropBox wireframe
  TF: <camera_frame> -> <object_frame>          refined model->camera transform

The point-to-plane / Fast-ICP solver and geometry live in
admittance_control/icp.py (pure NumPy).
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
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from admittance_control.geometry import rotmat_to_quat
from admittance_control.icp import (
    backproject_depth,
    crop_box_mask,
    estimate_normals_organized,
    icp_point_to_plane,
    load_ply_mesh,
    sample_mesh_surface,
    voxel_downsample,
)
from admittance_control.sam6d_io import resolve_transfer_dir

# Open3D is optional: if present it does the per-frame voxel downsample + normal
# estimation on the *cropped* cloud (fast C++/KD-tree). Without it we fall back
# to the pure-NumPy path (organized normals + numpy voxel), which also keeps the
# robot side dependency-free.
try:
    import open3d as o3d
    _HAS_OPEN3D = True
except Exception:  # noqa: BLE001 - any import failure -> numpy fallback
    _HAS_OPEN3D = False


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
        self.declare_parameter('crop_box_topic', '/perception/icp/crop_box')
        self.declare_parameter('detection_index', -1)   # -1 = best PEM score
        # ICP / sampling knobs.
        self.declare_parameter('n_model_points', 2500)
        self.declare_parameter('voxel_size_m', 0.006)
        self.declare_parameter('max_target_points', 15000)
        self.declare_parameter('max_corr_dist_m', 0.02)
        self.declare_parameter('max_iter', 30)
        self.declare_parameter('anderson_depth', 5)     # 0 = plain point-to-plane
        self.declare_parameter('robust', True)          # Welsch kernel + dynamic-nu (Robust-ICP)
        self.declare_parameter('use_open3d', True)       # fast crop-cloud voxel+normals
        # Real-time tracking loop (Phase 2).
        self.declare_parameter('crop_margin_m', 0.03)
        self.declare_parameter('tracking_rate_hz', 15.0)
        self.declare_parameter('auto_track', True)
        self.declare_parameter('lost_fitness', 0.1)
        self.declare_parameter('min_scene_points', 50)

        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._object_frame = str(self.get_parameter('object_frame').value)
        self._scene_from = str(self.get_parameter('scene_from').value)
        self._det_index = int(self.get_parameter('detection_index').value)
        self._n_model = int(self.get_parameter('n_model_points').value)
        self._voxel = float(self.get_parameter('voxel_size_m').value)
        self._max_target = int(self.get_parameter('max_target_points').value)
        self._max_corr = float(self.get_parameter('max_corr_dist_m').value)
        self._max_iter = int(self.get_parameter('max_iter').value)
        self._anderson = int(self.get_parameter('anderson_depth').value)
        self._robust = bool(self.get_parameter('robust').value)
        self._use_o3d = bool(self.get_parameter('use_open3d').value) and _HAS_OPEN3D
        if bool(self.get_parameter('use_open3d').value) and not _HAS_OPEN3D:
            self.get_logger().warn(
                'use_open3d=true but open3d is not importable; falling back to '
                'the NumPy voxel/normals path (slower). pip install open3d.')
        self._crop_margin = float(self.get_parameter('crop_margin_m').value)
        self._track_hz = float(self.get_parameter('tracking_rate_hz').value)
        self._auto_track = bool(self.get_parameter('auto_track').value)
        self._lost_fitness = float(self.get_parameter('lost_fitness').value)
        self._min_scene = int(self.get_parameter('min_scene_points').value)

        # Tracking state (Phase 2).
        self._current_pose: Optional[np.ndarray] = None
        self._tracking = False

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
        # Model-frame AABB -> dynamic CropBox extent (+ margin) for tracking.
        self._model_lo = self._model.min(0)
        self._model_hi = self._model.max(0)
        self.get_logger().info(
            f'model {Path(model_path).name}: {len(self._model)} pts, '
            f'extent(m)={np.round(self._model_hi - self._model_lo, 3)}')

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
        self._box_pub = self.create_publisher(
            Marker, str(self.get_parameter('crop_box_topic').value), latched)
        self._tf = TransformBroadcaster(self)

        self.create_service(Trigger, '~/run_icp', self._on_trigger)
        self.create_service(Trigger, '~/stop_tracking', self._on_stop_tracking)
        self.create_service(Trigger, '~/start_tracking', self._on_start_tracking)

        # Tracking timer (Phase 2): always spinning, but a no-op until seeded.
        period = 1.0 / self._track_hz if self._track_hz > 0 else 1.0 / 15.0
        self._track_timer = self.create_timer(period, self._on_track_tick)

        self.get_logger().info(
            f'ICP refiner ready. scene_from={self._scene_from}, '
            f'anderson_depth={self._anderson}, robust={self._robust}, '
            f'track@{self._track_hz:g}Hz, '
            f'scene_prep={"open3d" if self._use_o3d else "numpy"}. '
            f'results_dir={self._results_dir}. Call ~/run_icp to seed + track.')

    # ── Cloud cache ──────────────────────────────────────────────────────
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    # ── Triggers ─────────────────────────────────────────────────────────
    def _on_trigger(self, request, response):
        """Phase 1: SAM-6D-mask ICP init, then (auto_track) start tracking."""
        try:
            ok, message = self._run_once()
        except Exception as exc:  # noqa: BLE001 - report any failure to caller
            self.get_logger().error(f'ICP init failed: {exc}')
            response.success = False
            response.message = f'ICP init failed: {exc}'
            return response
        if ok and self._auto_track:
            self._tracking = True
            self.get_logger().info('tracking started (Phase 2).')
            message += ' | tracking started'
        response.success = ok
        response.message = message
        return response

    def _on_stop_tracking(self, request, response):
        self._tracking = False
        response.success = True
        response.message = 'tracking stopped'
        self.get_logger().info('tracking stopped.')
        return response

    def _on_start_tracking(self, request, response):
        if self._current_pose is None:
            response.success = False
            response.message = 'no pose to track yet; call ~/run_icp first'
            return response
        self._tracking = True
        response.success = True
        response.message = 'tracking started'
        self.get_logger().info('tracking started (Phase 2).')
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
            max_corr_dist=self._max_corr, max_iter=self._max_iter,
            anderson_depth=self._anderson, robust=self._robust)

        self._current_pose = T                  # seed the tracker (Phase 2)
        self._publish(frame_id, scene, T)
        self._log_metrics(idx, det, init, T, info, len(scene))
        return True, (f'det #{idx} refined: fitness={info["fitness"]:.3f} '
                      f'rmse={info["inlier_rmse"]:.4f}m corr={info["correspondences"]}')

    # ── Live raw cloud (organized xyz only; tracking uses the latest cloud) ─
    def _live_xyz(self):
        if self._latest_cloud is None:
            raise RuntimeError('no PointCloud2 received yet on the camera topic')
        msg = self._latest_cloud
        if msg.height <= 1:
            raise RuntimeError('cloud is not organized (height<=1)')
        arr = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        xyz = arr.reshape(msg.height, msg.width, 3).astype(np.float64)
        frame = msg.header.frame_id or self._camera_frame
        return xyz, frame

    # ── CropBox the live cloud around current_pose (the "macro-filter") ────
    def _crop_live_scene(self):
        """Return (scene_pts, scene_normals_or_None, frame_id, (lo, hi)).

        Open3D path: crop the raw cloud (normals computed later on the small
        cropped set). NumPy path: compute organized normals up-front, then crop
        both points and normals together.
        """
        lo = self._model_lo - self._crop_margin
        hi = self._model_hi + self._crop_margin
        if self._use_o3d:
            xyz, frame_id = self._live_xyz()
            pts = xyz[np.isfinite(xyz).all(2)]
            inside = crop_box_mask(pts, self._current_pose, lo, hi)
            return pts[inside], None, frame_id, (lo, hi)

        xyz, frame_id = self._live_xyz()
        normals = estimate_normals_organized(xyz)
        finite = np.isfinite(xyz).all(2) & np.isfinite(normals).all(2)
        pts, nrm = xyz[finite], normals[finite]
        inside = crop_box_mask(pts, self._current_pose, lo, hi)
        return pts[inside], nrm[inside], frame_id, (lo, hi)

    # ── Voxel downsample + normals of the cropped scene (Open3D fast path) ─
    def _prep_scene_o3d(self, scene_pts):
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scene_pts))
        if self._voxel > 0:
            pc = pc.voxel_down_sample(self._voxel)
        radius = max(self._voxel * 3.0, 0.01)
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        # optical-frame origin = camera centre; point-to-plane is sign-agnostic
        # but a consistent orientation keeps the normals tidy for viz/debug.
        pc.orient_normals_towards_camera_location(np.zeros(3))
        return np.asarray(pc.points), np.asarray(pc.normals)

    # ── Tracking loop tick (Phase 2) ─────────────────────────────────────
    def _on_track_tick(self):
        if not self._tracking or self._current_pose is None:
            return
        try:
            scene, scene_n, frame_id, (lo, hi) = self._crop_live_scene()
        except RuntimeError:
            return                              # no live cloud yet; wait

        # Keep the box visible (and its object frame fresh) every tick.
        self._send_object_tf(frame_id, self._current_pose)
        self._publish_cropbox(lo, hi)

        if len(scene) < self._min_scene:
            self.get_logger().warn(
                f'only {len(scene)} pts in crop box; object may have escaped -- '
                're-run SAM-6D + ~/run_icp to re-seed', throttle_duration_sec=2.0)
            return

        # Downsample + normals of the cropped set only (crop-first = the speedup).
        if self._use_o3d:
            scene, scene_n = self._prep_scene_o3d(scene)
        else:
            scene, scene_n = voxel_downsample(scene, self._voxel, scene_n)
        if len(scene) > self._max_target:
            sel = self._rng.choice(len(scene), self._max_target, replace=False)
            scene, scene_n = scene[sel], scene_n[sel]
        if len(scene) < 6:
            self.get_logger().warn(
                f'too few points after downsample ({len(scene)})',
                throttle_duration_sec=2.0)
            return

        T, info = icp_point_to_plane(
            self._model, scene, scene_n, init=self._current_pose,
            max_corr_dist=self._max_corr, max_iter=self._max_iter,
            anderson_depth=self._anderson, robust=self._robust)

        if info['fitness'] < self._lost_fitness:
            self._tracking = False
            self.get_logger().warn(
                f'tracking lost (fitness {info["fitness"]:.3f} < '
                f'{self._lost_fitness}); paused. Re-run SAM-6D + ~/run_icp.')
            return

        self._current_pose = T
        self._publish(frame_id, scene, T)
        self.get_logger().info(
            f'track: fitness={info["fitness"]:.3f} rmse={info["inlier_rmse"]:.4f}m '
            f'iters={info["iterations"]} scene={len(scene)}',
            throttle_duration_sec=1.0)

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

        self._send_object_tf(frame_id, T, stamp)

    # ── TF: <camera_frame> -> <object_frame> = refined model->camera pose ──
    def _send_object_tf(self, frame_id, T, stamp=None):
        stamp = stamp or self.get_clock().now().to_msg()
        qx, qy, qz, qw = rotmat_to_quat(T[:3, :3])
        tf = TransformStamped()
        tf.header = self._make_header(frame_id, stamp)
        tf.child_frame_id = self._object_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self._tf.sendTransform(tf)

    # ── Dynamic CropBox wireframe (drawn in the object frame; follows TF) ──
    def _publish_cropbox(self, lo, hi):
        # 8 corners of the AABB [lo, hi], expressed in the object frame.
        corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                            for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        # 12 edges as index pairs into the corner list (bit-flip neighbours).
        edges = [(i, i ^ b) for i in range(8) for b in (1, 2, 4) if i < (i ^ b)]

        mk = Marker()
        mk.header.frame_id = self._object_frame
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'icp_crop_box'
        mk.id = 0
        mk.type = Marker.LINE_LIST
        mk.action = Marker.ADD
        mk.scale.x = 0.002  # line width (m)
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.55, 0.0, 0.9
        mk.pose.orientation.w = 1.0
        for a, b in edges:
            for c in (corners[a], corners[b]):
                mk.points.append(Point(x=float(c[0]), y=float(c[1]), z=float(c[2])))
        self._box_pub.publish(mk)

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
