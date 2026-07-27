#!/usr/bin/env python3
"""Real-time ICP object tracking, seeded by a 6D pose detection.

The seed comes from the pose bridge (``foundationpose_bridge_node.py``, which
saves the server's artifacts into ``results_dir``). The file format is SAM-6D's
-- the FoundationPose server re-emits its result in it deliberately -- so this
node reads the same two files it always has, whichever backend produced them.

Two phases, following notes/realtime_icp.md:

Phase 1 - Initialization (``~/run_icp``, runs once per object)
    Take the segmentation mask (detection_ism.npz) to cut the object region out
    of the camera pointcloud, place the CAD .ply at the detected 6D pose, and
    run point-to-plane ICP to get the initial pose T0. This seeds the tracker's
    ``current_pose`` and (if ``auto_track``) starts Phase 2.

Phase 2 - Tracking loop (timer at ``tracking_rate_hz``, no pose server)
    Each tick: grab the live organized cloud, build a **dynamic CropBox** around
    ``current_pose`` (the model AABB + ``crop_margin_m``) to isolate the object
    -- this replaces the detection mask -- then run **Fast & Robust ICP**
    (Welsch-reweighted, dynamic-nu, Anderson-accelerated point-to-plane;
    ``robust`` + ``anderson_depth``) from ``current_pose`` and update it. The
    Welsch kernel smoothly rejects a gripper/fixture that enters the crop box. The crop box is published as an RViz Marker so you can watch it
    follow the object as you move it. If ICP fitness drops below
    ``lost_fitness`` (object escaped the box) tracking pauses; re-trigger the
    pose bridge and call ``~/run_icp`` again to re-seed.

    The tick is budgeted, because at ``tracking_rate_hz`` it has to fit in one
    period. Three things keep it there, all of which leave the result unchanged:
      * ``roi_crop`` -- the crop box is a bounded 3-D region, so the camera sees
        it inside a bounded *rectangle*. That rectangle is computed first (from
        intrinsics recovered off the organized cloud itself) and the cloud is
        sliced to it, so the float64 cast, the finiteness test and the oriented
        box test only touch pixels that could possibly be inside the box.
      * one ``NNIndex`` per tick, shared by the noise-floor measurement and all
        ~43 of ICP's nearest-neighbour queries, instead of one structure rebuilt
        per solver iteration.
      * ``noise_floor_refresh`` -- the Welsch nu floor measures the depth
        sensor's noise, not the pose, so it is re-measured every N ticks.
    Measured on a 640x480 cloud: ~74 ms/tick before, ~23 ms after.

Multi-object assembly (Model-Based Background Subtraction)
---------------------------------------------------------
    Once an object is placed, ``~/save_object`` bakes its CAD cloud (at the
    refined pose) into a Static Environment Point Cloud (SEPC) held in
    ``static_frame`` (default base_link -- the camera is eye-in-hand, so the SEPC
    cannot live in the moving camera frame). Every subsequent tracking tick
    transforms the live crop into ``static_frame`` and deletes points within
    ``bg_subtract_dist_m`` of the SEPC, so ICP goes blind to already-assembled
    parts and won't snap onto them when the new object is pushed flush. Workflow:
    run_icp (obj A) -> ... -> save_object -> run_icp (obj B). The CAD for each
    object is selected automatically from the server's PPF classification (the
    detection's ``obj_name`` -> ``model_dir/<name>.ply``); ``model_path`` is only
    the fallback/override for when classification is off or names an unknown part.

Triggers
--------
    ros2 service call /icp_pose_refiner/run_icp        std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/stop_tracking  std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/start_tracking std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/save_object    std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/welding_points std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/export_mesh    std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/reset_environment std_srvs/srv/Trigger

Starting the next assembly
--------------------------
    ``export_mesh`` + the bridge's ``~/add_model`` turn the finished assembly
    into one classifiable part, at which point every object in the SEPC is
    stale -- and a stale SEPC keeps subtracting itself out of the live crop, so
    ICP goes blind exactly where the next part gets placed. ``~/reset_environment``
    drops the SEPC, the saved-object list and the tracking state, blanks the
    latched RViz clouds, and moves ``static_env.*`` / ``assembly.json`` into
    ``<save_dir>/previous_assemblies/<timestamp>/``. The files are archived
    rather than deleted, and they have to move: ``_sepc_or_load()`` reloads them
    from disk whenever the in-memory SEPC is None, so leaving them would
    resurrect the old assembly on the next ``welding_points`` call.

        export_mesh -> (bridge) add_model -> reset_environment -> next assembly

Welding points
--------------
    Once the assembly is frozen into the SEPC, ``welding_points`` extracts the
    seam where two near-orthogonal parts meet: a *radius* PCA neighbourhood
    (weld_radius_m, which must exceed the gap between the parts) makes surface
    variation spike on the edges bounding the gap, the threshold
    (weld_curvature_thresh) keeps only those, and a voxel grid coarser than the
    gap (weld_voxel_m) averages the two parallel edge lines into a single seam.
    The result is published red on <weld_points_topic> and written to
    <save_dir>/welding_points.ply/.npy.

Init inputs (from the pose-bridge capture, read fresh on run_icp)
----------------------------------------------------------------
  <results_dir>/detection_pem.json   6D poses (R, t[mm], score, category)
  <results_dir>/detection_ism.npz    per-detection masks: segmentation (K,H,W)
  ``results_dir`` defaults to the FoundationPose bridge's output directory
  (``scripts/foundationpose_results``). Point it at ``scripts/sam6d_results`` to
  replay an old SAM-6D capture -- but do not leave it there by accident, or the
  ICP seeds off a stale mask while the bridge writes elsewhere.
  scene cloud, chosen by ``scene_from``:
    'pointcloud' : latest <pointcloud_topic> (organized sensor_msgs/PointCloud2)
    'depth_png'  : <transfer_dir>/depth.png + camera.json (the exact frame the
                   pose server saw -> guaranteed pixel alignment with the mask)

Publishes
---------
  <scene_cloud_topic>  sensor_msgs/PointCloud2  segmented/cropped scene (white)
  <model_cloud_topic>  sensor_msgs/PointCloud2  CAD model at the refined pose (green)
  <refined_pose_topic> geometry_msgs/PoseStamped
  <crop_box_topic>     visualization_msgs/Marker  the dynamic CropBox wireframe
  <sepc_topic>         sensor_msgs/PointCloud2  frozen assembly cloud (orange, static_frame)
  <weld_points_topic>  sensor_msgs/PointCloud2  weld seam points (red, static_frame)
  TF: <camera_frame> -> <object_frame>          refined model->camera transform

The point-to-plane / Fast-ICP solver and geometry live in
admittance_control/icp.py (pure NumPy).

Object CAD selection
--------------------
By default the CAD is chosen from the classifier: the detection's ``obj_name`` is
resolved to ``model_dir/<obj_name>.ply``, and ``model_dir`` defaults to the folder
``model_path`` points at. So a single ``model_path`` set to any file in the models
folder is enough to make every classified part resolve:

    ros2 param set /icp_pose_refiner model_path src/admittance_control/models/test_objv2.ply

To pin one CAD regardless of classification (classifier off, or replaying a
capture), set ``model_path`` and the un-classified detections fall back to it.
"""

from __future__ import annotations

import json
import shutil
import struct
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException
from visualization_msgs.msg import Marker

from admittance_control.geometry import quat_to_rotmat, rotmat_to_quat
from admittance_control.icp import (
    NNIndex,
    backproject_depth,
    box_image_roi,
    crop_box_mask,
    estimate_normals_organized,
    icp_point_to_plane,
    infer_pinhole_from_organized,
    load_ply_mesh,
    nearest_neighbor,
    sample_mesh_surface,
    voxel_downsample,
    welsch_nu_end,
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

try:  # KD-tree for background subtraction against the static environment cloud
    from scipy.spatial import cKDTree as _cKDTree
except Exception:  # noqa: BLE001
    _cKDTree = None


def write_ply_points(path: Path, pts: np.ndarray) -> None:
    """Write an ascii XYZ point cloud to a .ply (viewable / reloadable)."""
    lines = ['ply', 'format ascii 1.0', f'element vertex {len(pts)}',
             'property float x', 'property float y', 'property float z',
             'end_header']
    body = '\n'.join(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}' for p in pts)
    path.write_text('\n'.join(lines) + '\n' + body + ('\n' if len(pts) else ''))


def surface_variation(pts: np.ndarray, radius: float, min_neighbors: int
                      ) -> np.ndarray:
    """Per-point PCA surface variation V = l3 / (l1+l2+l3) over a radius ball.

    A *radius* neighbourhood (not kNN) is what lets the covariance straddle the
    gap between two nearly-orthogonal parts: with R larger than the gap, a point
    on the edge of part A pulls in points from part B, and V spikes exactly as
    if the parts touched. Flat faces give V ~ 0; edges give V ~ 0.05-0.15.

    Points with fewer than ``min_neighbors`` neighbours get V = 0 (isolated
    specks would otherwise produce a degenerate, high-variation covariance).
    """
    n = len(pts)
    var = np.zeros(n)
    if n == 0:
        return var

    if _cKDTree is not None:
        neighbor_ids = _cKDTree(pts).query_ball_point(pts, radius, workers=-1)
    else:  # no scipy: O(N^2) distance matrix (SEPC clouds are small)
        d2 = np.sum((pts[:, None, :] - pts[None, :, :]) ** 2, axis=-1)
        r2 = radius * radius
        neighbor_ids = [np.flatnonzero(row <= r2) for row in d2]

    for i, ids in enumerate(neighbor_ids):
        if len(ids) < min_neighbors:
            continue
        nb = pts[ids]
        cov = np.cov((nb - nb.mean(0)).T, bias=True)
        lam = np.linalg.eigvalsh(cov)          # ascending: l3 <= l2 <= l1
        total = lam.sum()
        if total > 1e-18:
            var[i] = max(lam[0], 0.0) / total
    return var


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
        # `model_path` is the explicit / fallback CAD. `model_dir` is the directory
        # of CAD .ply files the PPF classifier's answer is resolved against
        # (model_dir/<obj_name>.ply); left empty it defaults to model_path's own
        # directory, so classification just works for the usual single-folder setup.
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_dir', '')
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
        # Front-end: project the crop box into the image and only unpack its
        # pixel rectangle. Exact (the surviving point set is unchanged), but it
        # skips the finiteness + oriented-box test on every pixel that could not
        # possibly be inside the box -- the dominant front-end cost. Turn off to
        # fall back to scanning the whole cloud.
        self.declare_parameter('roi_crop', True)
        self.declare_parameter('roi_pad_px', 2)
        # The Welsch nu floor measures the *sensor's* noise, not the pose, so it
        # is re-measured only every N ticks instead of every frame (~2.5 ms).
        self.declare_parameter('noise_floor_refresh', 60)
        # Real-time tracking loop (Phase 2).
        self.declare_parameter('crop_margin_m', 0.03)
        self.declare_parameter('tracking_rate_hz', 10.0)
        self.declare_parameter('auto_track', True)
        self.declare_parameter('lost_fitness', 0.1)
        self.declare_parameter('min_scene_points', 50)
        # Model-based background subtraction (multi-object assembly).
        # SEPC = Static Environment Point Cloud: the CAD clouds of already-placed
        # objects, held in the *static* frame (the camera is eye-in-hand, so the
        # camera frame moves; the SEPC must not). Each tick the live crop is
        # transformed into static_frame and points within bg_subtract_dist_m of
        # the SEPC are deleted, so ICP goes blind to previously assembled parts.
        self.declare_parameter('static_frame', 'base_link')
        self.declare_parameter('bg_subtract', True)
        self.declare_parameter('bg_subtract_dist_m', 0.005)
        self.declare_parameter('sepc_topic', '/perception/icp/static_env')
        self.declare_parameter('save_dir', '')          # '' -> results_dir
        # Ground/table removal: hard Z-truncation in static_frame. Anything at or
        # below ground_z_m is deleted before ICP (sim floor and the real bench
        # sit at different heights, so this is a knob to tune per setup).
        self.declare_parameter('ground_removal', True)
        self.declare_parameter('ground_z_m', -0.10)     # base_link Z of the floor
        # Welding-seam extraction from the SEPC (~/welding_points). All lengths
        # are in metres and live in static_frame, like the SEPC itself.
        self.declare_parameter('weld_points_topic', '/perception/icp/welding_points')
        # radius must exceed (gap + SEPC point spacing) so the PCA ball bridges
        # the gap, yet stay under the part thickness or it bridges a plate's own
        # two faces and every point looks curved.
        self.declare_parameter('weld_radius_m', 0.006)
        self.declare_parameter('weld_curvature_thresh', 0.03)
        self.declare_parameter('weld_min_neighbors', 5)
        self.declare_parameter('weld_voxel_m', 0.004)       # > gap: merges the
        #                                    two parallel edge lines into one seam
        # A part's own outer border is a 90-degree fold too, so curvature alone
        # also fires there. Keeping only edge points that see a *different*
        # object within weld_radius_m leaves just the joint between the parts.
        self.declare_parameter('weld_require_cross_object', True)

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
        self._roi_crop = bool(self.get_parameter('roi_crop').value)
        self._roi_pad = int(self.get_parameter('roi_pad_px').value)
        self._noise_refresh = int(self.get_parameter('noise_floor_refresh').value)
        if bool(self.get_parameter('use_open3d').value) and not _HAS_OPEN3D:
            self.get_logger().warn(
                'use_open3d=true but open3d is not importable; falling back to '
                'the NumPy voxel/normals path (slower). pip install open3d.')
        self._crop_margin = float(self.get_parameter('crop_margin_m').value)
        self._track_hz = float(self.get_parameter('tracking_rate_hz').value)
        self._auto_track = bool(self.get_parameter('auto_track').value)
        self._lost_fitness = float(self.get_parameter('lost_fitness').value)
        self._min_scene = int(self.get_parameter('min_scene_points').value)
        self._static_frame = str(self.get_parameter('static_frame').value)
        self._bg_subtract = bool(self.get_parameter('bg_subtract').value)
        self._bg_dist = float(self.get_parameter('bg_subtract_dist_m').value)
        self._ground_removal = bool(self.get_parameter('ground_removal').value)
        self._ground_z = float(self.get_parameter('ground_z_m').value)
        save_dir = str(self.get_parameter('save_dir').value)

        # Tracking state (Phase 2). Guarded by _state_lock: with the
        # MultiThreadedExecutor (see main()) the tracking tick and the Trigger
        # services run on different threads and both mutate this state, so a
        # save_object landing halfway through a tick would otherwise freeze a
        # torn pose. The cloud subscription deliberately does *not* take the
        # lock -- keeping it free to run during a tick is the point of the split.
        self._state_lock = threading.RLock()
        self._current_pose: Optional[np.ndarray] = None
        self._tracking = False

        # Pinhole K recovered from the organized cloud (for the ROI crop), and
        # the cached Welsch noise floor. Both are properties of the camera, not
        # of the object, so they survive across ticks.
        self._pinhole_K: Optional[np.ndarray] = None
        self._pinhole_shape = None
        self._noise_floor: Optional[float] = None
        self._noise_age = 0

        # Assembly state: Static Environment Point Cloud (in static_frame) and
        # its KD-tree, plus a record of every object frozen into it so far.
        self._sepc: Optional[np.ndarray] = None
        self._sepc_tree = None
        self._saved: list = []
        self._weld: Optional[np.ndarray] = None

        results_dir = str(self.get_parameter('results_dir').value)
        self._results_dir = (Path(results_dir) if results_dir
                             else resolve_transfer_dir().parent / 'foundationpose_results')
        self._transfer_dir = resolve_transfer_dir()
        self._save_dir = Path(save_dir).expanduser() if save_dir else self._results_dir

        # Load + sample the CAD model (metres). The model is re-resolved on every
        # run_icp from the classifier's answer, so the next object in the assembly
        # loads its own CAD automatically. Load eagerly here only if a static
        # model_path is set; otherwise wait for the first classified detection.
        self._rng = np.random.default_rng(0)
        self._model_name = ''
        self._model = None
        if str(self.get_parameter('model_path').value):
            self._load_model()

        # Three callback groups so the MultiThreadedExecutor can overlap them.
        # The tracking tick is long (tens of ms); on the default single group it
        # blocks the cloud subscription for its whole duration, so every tick
        # ends up refining against a frame that is at least one tick stale --
        # a latency bug in a *tracker*. Separate groups let the newest cloud keep
        # arriving while ICP runs. Services share one group so they serialize
        # with each other, and the lock serializes them against the tick.
        self._cloud_cbg = MutuallyExclusiveCallbackGroup()
        self._timer_cbg = MutuallyExclusiveCallbackGroup()
        self._srv_cbg = MutuallyExclusiveCallbackGroup()

        self._latest_cloud: Optional[PointCloud2] = None
        self.create_subscription(
            PointCloud2, str(self.get_parameter('pointcloud_topic').value),
            self._on_cloud, qos_profile_sensor_data,
            callback_group=self._cloud_cbg)

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
        self._sepc_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('sepc_topic').value), latched)
        self._weld_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('weld_points_topic').value), latched)
        self._tf = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        for name, handler in (
                ('~/run_icp', self._on_trigger),
                ('~/stop_tracking', self._on_stop_tracking),
                ('~/start_tracking', self._on_start_tracking),
                ('~/save_object', self._on_save_object),
                ('~/welding_points', self._on_welding_points),
                ('~/export_mesh', self._on_export_mesh),
                ('~/reset_environment', self._on_reset_environment)):
            self.create_service(Trigger, name, handler,
                                callback_group=self._srv_cbg)

        # Live-tunable filter knobs (ros2 param set ... takes effect next frame).
        self.add_on_set_parameters_callback(self._on_set_params)

        # Tracking timer (Phase 2): always spinning, but a no-op until seeded.
        period = 1.0 / self._track_hz if self._track_hz > 0 else 1.0 / 15.0
        self._track_timer = self.create_timer(period, self._on_track_tick,
                                              callback_group=self._timer_cbg)

        self.get_logger().info(
            f'ICP refiner ready. scene_from={self._scene_from}, '
            f'anderson_depth={self._anderson}, robust={self._robust}, '
            f'track@{self._track_hz:g}Hz, '
            f'scene_prep={"open3d" if self._use_o3d else "numpy"}, '
            f'roi_crop={self._roi_crop}, nn={NNIndex(np.zeros((1, 3))).backend}, '
            f'bg_subtract={self._bg_subtract}@{self._bg_dist * 1000:g}mm, '
            f'ground_removal={self._ground_removal}@z>{self._ground_z:g}m '
            f'(static_frame={self._static_frame}). '
            f'results_dir={self._results_dir}. Call ~/run_icp to seed + track.')

    # ── CAD model (re-resolved each run_icp to the classified object) ─────
    def _resolve_model_path(self, object_name: str) -> str:
        """Which .ply to load: the classified object first, then model_path.

        The PPF classifier on the server names the CAD it registered against and
        puts that name in the detection (`obj_name`). Resolving it here to
        ``model_dir/<name>.ply`` is what keeps ICP matching the *same* mesh the
        server used -- without it, the second object in an assembly is refined
        against the first object's CAD, which is exactly the "wrong point cloud"
        symptom. `model_path` remains the fallback (classifier off, or an unknown
        name) and the override (set it and leave the object un-classified).
        """
        model_path = str(self.get_parameter('model_path').value)
        model_dir = str(self.get_parameter('model_dir').value)
        # Default the CAD directory to model_path's own folder, so an existing
        # single-object setup resolves classified names with no extra config.
        if not model_dir and model_path:
            model_dir = str(Path(model_path).expanduser().parent)

        if object_name and model_dir:
            cand = Path(model_dir).expanduser() / f'{object_name}.ply'
            if cand.is_file():
                return str(cand)
            self.get_logger().warn(
                f"classifier named '{object_name}' but {cand} does not exist; "
                f"falling back to model_path={model_path or '(unset)'}")
        if model_path:
            return model_path
        raise RuntimeError(
            f"no CAD to load: classification returned '{object_name or 'nothing'}' "
            f"with no matching .ply in model_dir={model_dir or '(unset)'}, and "
            'model_path is unset.')

    def _load_model(self, object_name: str = '') -> None:
        resolved = self._resolve_model_path(object_name)
        name = Path(resolved).name
        # Skip the (fairly costly) mesh sample if nothing changed since last call.
        n_model = int(self.get_parameter('n_model_points').value)
        units = str(self.get_parameter('model_units').value).lower()
        key = (resolved, n_model, units)
        if getattr(self, '_model_key', None) == key:
            return
        scale = 0.001 if units == 'mm' else 1.0
        verts, faces = load_ply_mesh(Path(resolved).expanduser())
        self._n_model = n_model
        self._model = sample_mesh_surface(verts, faces, n_model, self._rng) * scale
        # Model-frame AABB -> dynamic CropBox extent (+ margin) for tracking.
        self._model_lo = self._model.min(0)
        self._model_hi = self._model.max(0)
        self._model_name = name
        self._model_key = key
        self.get_logger().info(
            f'model {name}{f" (classified {object_name})" if object_name else ""}: '
            f'{len(self._model)} pts, '
            f'extent(m)={np.round(self._model_hi - self._model_lo, 3)}')

    # ── Cloud cache ──────────────────────────────────────────────────────
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    # ── Triggers ─────────────────────────────────────────────────────────
    def _on_trigger(self, request, response):
        """Phase 1: FoundationPose-mask ICP init, then (auto_track) start tracking."""
        with self._state_lock:
            try:
                # _run_once resolves the CAD from the classified detection itself.
                ok, message = self._run_once()
            except Exception as exc:  # noqa: BLE001 - report any failure to caller
                self.get_logger().error(f'ICP init failed: {exc}')
                response.success = False
                response.message = f'ICP init failed: {exc}'
                return response
            # Re-seeding is the one moment the scene really can change (new part,
            # new stand-off), so drop the cached sensor-noise estimate with it.
            self._noise_floor, self._noise_age = None, 0
            if ok and self._auto_track:
                self._tracking = True
                self.get_logger().info('tracking started (Phase 2).')
                message += ' | tracking started'
        response.success = ok
        response.message = message
        return response

    def _on_stop_tracking(self, request, response):
        with self._state_lock:
            self._tracking = False
        response.success = True
        response.message = 'tracking stopped'
        self.get_logger().info('tracking stopped.')
        return response

    def _on_start_tracking(self, request, response):
        with self._state_lock:
            if self._current_pose is None:
                response.success = False
                response.message = 'no pose to track yet; call ~/run_icp first'
                return response
            self._tracking = True
        response.success = True
        response.message = 'tracking started'
        self.get_logger().info('tracking started (Phase 2).')
        return response

    def _on_set_params(self, params):
        """Apply live changes to the filter knobs (ground / bg-subtract / lost).

        Lets you sweep ground_z_m etc. with ``ros2 param set`` while tracking,
        without relaunching -- handy since sim and real ground heights differ.
        """
        for p in params:
            if p.name == 'ground_removal':
                self._ground_removal = bool(p.value)
            elif p.name == 'ground_z_m':
                self._ground_z = float(p.value)
            elif p.name == 'bg_subtract':
                self._bg_subtract = bool(p.value)
            elif p.name == 'bg_subtract_dist_m':
                self._bg_dist = float(p.value)
            elif p.name == 'lost_fitness':
                self._lost_fitness = float(p.value)
        self.get_logger().info(
            f'params updated: ground_removal={self._ground_removal} '
            f'ground_z_m={self._ground_z:g}m bg_subtract={self._bg_subtract} '
            f'bg_dist={self._bg_dist:g}m lost_fitness={self._lost_fitness:g}')
        return SetParametersResult(successful=True)

    def _on_save_object(self, request, response):
        """Freeze the current object into the SEPC (Model-Based Background Sub).

        Bakes the CAD model at its refined pose into the Static Environment
        Point Cloud (held in static_frame), rebuilds the background KD-tree,
        persists it, and clears the tracking state so the next object can be
        introduced (set model_path if it differs, then call ~/run_icp).
        """
        with self._state_lock:
            if self._current_pose is None:
                response.success = False
                response.message = 'no refined pose to save; call ~/run_icp first'
                return response
            try:
                ok, message = self._save_object()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'save_object failed: {exc}')
                response.success = False
                response.message = f'save_object failed: {exc}'
                return response
        response.success = ok
        response.message = message
        return response

    def _on_export_mesh(self, request, response):
        """Build a watertight CAD mesh of the assembled scene, ready to upload.

        The SEPC (static_env.ply) is a point cloud with no faces, so the host's
        /add_model rejects it. But each saved object's original CAD and its refined
        ``pose_static`` are known, so we re-instantiate those CADs at their poses and
        boolean-union them (admittance_control.assembly_mesh) into a faced, watertight
        .ply in millimetres -- exactly what /add_model wants. Then:

            ros2 param set /foundationpose_bridge model_ply_path <written path>
            ros2 service call /foundationpose_bridge/add_model std_srvs/srv/Trigger
        """
        try:
            from admittance_control.assembly_mesh import (
                build_assembly_mesh, load_assembly)
        except Exception as exc:  # noqa: BLE001 - missing trimesh/manifold3d etc.
            response.success = False
            response.message = (f'could not import assembly_mesh ({exc}); '
                                'pip install trimesh manifold3d')
            return response

        # Prefer the fresh in-memory manifest; fall back to the persisted one so a
        # restarted node can still export the last assembly. Snapshot it under the
        # lock so a concurrent save_object cannot grow the list mid-read.
        with self._state_lock:
            objects = [(o['model'],
                        np.asarray(o['pose_static'], dtype=np.float64).reshape(4, 4))
                       for o in self._saved]
        if not objects:
            assembly_json = self._save_dir / 'assembly.json'
            if not assembly_json.is_file():
                response.success = False
                response.message = ('no saved objects and no assembly.json; '
                                    'call ~/save_object first')
                return response
            _, objects = load_assembly(assembly_json)

        # Same CAD folder and unit convention the model loader uses.
        model_dir = str(self.get_parameter('model_dir').value)
        model_path = str(self.get_parameter('model_path').value)
        if not model_dir and model_path:
            model_dir = str(Path(model_path).expanduser().parent)
        mesh_scale = 0.001 if str(self.get_parameter('model_units').value).lower() == 'mm' else 1.0
        out_path = self._save_dir / 'assembly_mesh.ply'

        try:
            mesh, stats = build_assembly_mesh(objects, model_dir, mesh_scale=mesh_scale)
            self._save_dir.mkdir(parents=True, exist_ok=True)
            mesh.export(str(out_path))
        except Exception as exc:  # noqa: BLE001 - report any build/export failure
            self.get_logger().error(f'export_mesh failed: {exc}')
            response.success = False
            response.message = f'export_mesh failed: {exc}'
            return response

        if not stats['watertight']:
            self.get_logger().warn(
                'assembly mesh is NOT watertight; the union may have holes. It will '
                'still upload, but check it before trusting the model.')
        summary = (f"wrote {out_path} from {len(objects)} part(s): "
                   f"{stats['vertices']}v/{stats['faces']}f, "
                   f"watertight={stats['watertight']}, extents(mm)={stats['extents_mm']}. "
                   f"Set /foundationpose_bridge model_ply_path to it, then ~/add_model.")
        self.get_logger().info(summary)
        response.success = True
        response.message = summary
        return response

    # ── Start a new assembly ─────────────────────────────────────────────
    def _on_reset_environment(self, request, response):
        """Forget the current assembly so the next one starts from an empty scene.

        Needed once the exported mesh has been uploaded with ~/add_model: from
        that point the assembly is a single classifiable part, and every object
        baked into the SEPC is stale. Left in place it keeps subtracting itself
        out of the live crop, so ICP goes blind to the very region the next part
        is placed in.

        Clearing memory alone is not enough. `_sepc_or_load()` lazily reloads
        `static_env.npy` (and `assembly.json`) from `save_dir` whenever `_sepc`
        is None, so an in-memory-only reset would look clean until the next
        ~/welding_points or ~/export_mesh silently resurrected the old assembly.
        The files are moved aside, not deleted -- you have usually just exported a
        mesh from them, and losing that provenance to a reset would be its own
        bug.
        """
        with self._state_lock:
            archived = self._archive_assembly()

            had = len(self._saved)
            n_points = 0 if self._sepc is None else len(self._sepc)
            self._sepc = None
            self._sepc_tree = None
            self._saved = []
            self._weld = None
            self._tracking = False
            self._current_pose = None
            self._noise_floor, self._noise_age = None, 0

        # The SEPC and weld topics are latched, so RViz holds the last cloud it
        # was sent until something replaces it. Publish empties, or the orange
        # assembly stays on screen after the state behind it is gone -- exactly
        # the confusion this service exists to end.
        self._publish_empty_cloud(self._sepc_pub)
        self._publish_empty_cloud(self._weld_pub)

        message = (f'environment reset: dropped {had} saved object(s) '
                   f'({n_points} SEPC points) and cleared tracking. {archived}')
        self.get_logger().info(message)
        response.success = True
        response.message = message
        return response

    def _archive_assembly(self) -> str:
        """Move the on-disk assembly artifacts into a timestamped subdirectory."""
        names = ('static_env.npy', 'static_env.ply', 'assembly.json',
                 'assembly_mesh.ply', 'welding_points.npy', 'welding_points.ply')
        present = [n for n in names if (self._save_dir / n).is_file()]
        if not present:
            return 'nothing on disk to archive.'
        dest = self._save_dir / 'previous_assemblies' / time.strftime('%Y%m%d-%H%M%S')
        try:
            dest.mkdir(parents=True, exist_ok=True)
            for name in present:
                shutil.move(str(self._save_dir / name), str(dest / name))
        except Exception as exc:  # noqa: BLE001
            # A half-moved directory would leave a stale static_env.npy behind and
            # silently reload it later, so say so loudly rather than report success.
            self.get_logger().error(
                f'could not archive the old assembly ({exc}); files may remain in '
                f'{self._save_dir} and will be reloaded on the next welding_points '
                f'or export_mesh call. Move them aside by hand.')
            return f'(warning: archiving failed: {exc})'
        return f'moved {len(present)} file(s) to {dest}.'

    def _publish_empty_cloud(self, publisher) -> None:
        header = self._make_header(self._static_frame,
                                   self.get_clock().now().to_msg())
        empty = np.zeros((0, 3), dtype=np.float32)
        publisher.publish(make_xyzrgb_cloud(header, empty,
                                            np.zeros((0, 3), dtype=np.uint8)))

    def _on_welding_points(self, request, response):
        """Extract the weld seam from the SEPC and publish it (red) for RViz."""
        with self._state_lock:
            try:
                ok, message = self._welding_points()
            except Exception as exc:  # noqa: BLE001 - report any failure to caller
                self.get_logger().error(f'welding_points failed: {exc}')
                response.success = False
                response.message = f'welding_points failed: {exc}'
                return response
        response.success = ok
        response.message = message
        return response

    # ── Weld seam: PCA curvature over a gap-bridging radius ───────────────
    def _sepc_or_load(self) -> Optional[np.ndarray]:
        """The in-memory SEPC, or the last persisted one (node may have restarted)."""
        if self._sepc is not None:
            return self._sepc
        cached = self._save_dir / 'static_env.npy'
        if not cached.exists():
            return None
        self._sepc = np.load(cached)
        self._sepc_tree = _cKDTree(self._sepc) if _cKDTree is not None else None
        manifest = self._save_dir / 'assembly.json'
        if manifest.exists():   # restore _saved so object ids can be rebuilt
            self._saved = json.loads(manifest.read_text()).get('objects', [])
        self.get_logger().info(f'loaded SEPC from {cached} ({len(self._sepc)} pts)')
        return self._sepc

    def _sepc_object_ids(self) -> Optional[np.ndarray]:
        """Per-point object index, from the order objects were stacked into the SEPC."""
        counts = [int(o['n_points']) for o in self._saved]
        if len(counts) < 2 or sum(counts) != len(self._sepc):
            return None
        return np.repeat(np.arange(len(counts)), counts)

    def _welding_points(self):
        """Find the seam where two near-orthogonal CAD clouds meet.

        The SEPC holds the CAD clouds of the assembled parts, separated by a
        physical gap. A radius (not kNN) PCA neighbourhood spans that gap, so
        surface variation spikes on both edges bounding it; thresholding keeps
        those. Each part's own outer border is a fold as well, so we additionally
        require an edge point to see a *different* object inside the same radius
        -- that condition is only true along the joint. A voxel grid coarser than
        the gap then averages the two parallel edge lines into one seam.
        """
        sepc = self._sepc_or_load()
        if sepc is None:
            return False, ('no static environment cloud; call ~/save_object at '
                           'least once (or point save_dir at a static_env.npy)')

        radius = float(self.get_parameter('weld_radius_m').value)
        thresh = float(self.get_parameter('weld_curvature_thresh').value)
        min_nb = int(self.get_parameter('weld_min_neighbors').value)
        voxel = float(self.get_parameter('weld_voxel_m').value)
        want_cross = bool(self.get_parameter('weld_require_cross_object').value)

        var = surface_variation(sepc, radius, min_nb)
        keep = var >= thresh
        if not keep.any():
            return False, (f'no points above curvature {thresh:g} (max was '
                           f'{var.max():.4f}); lower weld_curvature_thresh or '
                           f'raise weld_radius_m (now {radius * 1000:g}mm, must '
                           'exceed the gap)')

        n_edges = int(keep.sum())
        ids = self._sepc_object_ids() if want_cross else None
        if want_cross and ids is None:
            self.get_logger().warn(
                'weld_require_cross_object=true but the SEPC holds fewer than '
                'two objects (or assembly.json disagrees with static_env.npy); '
                'keeping every high-curvature point, which will include each '
                "part's outer borders, not just the joint.")
        elif ids is not None:
            keep &= self._cross_object_mask(sepc, ids, radius, keep)
            if not keep.any():
                return False, (f'{n_edges} high-curvature points, but none lie '
                               f'within {radius * 1000:g}mm of another object; '
                               'raise weld_radius_m or check the parts touch')

        # Voxel coarser than the gap: averages the two parallel edge lines into
        # a single line of points down the middle of the joint.
        seam, _ = voxel_downsample(sepc[keep], voxel)

        self._weld = seam
        self._publish_weld(seam)
        note = self._persist_weld(seam)
        self.get_logger().info(
            f'weld seam: {n_edges} edge pts (V>={thresh:g}, R={radius * 1000:g}mm) '
            f'-> {int(keep.sum())} on the joint -> {len(seam)} seam pts after '
            f'{voxel * 1000:g}mm voxel. {note}')
        return True, (f'{len(seam)} welding points from {n_edges} edge points '
                      f'(of {len(sepc)} SEPC points)')

    def _cross_object_mask(self, sepc, ids, radius, candidates):
        """True where a candidate point has a neighbour from another object within radius."""
        mask = np.zeros(len(sepc), dtype=bool)
        idx = np.flatnonzero(candidates)
        if _cKDTree is not None:
            tree = self._sepc_tree if self._sepc_tree is not None else _cKDTree(sepc)
            neighbor_ids = tree.query_ball_point(sepc[idx], radius, workers=-1)
        else:
            d2 = np.sum((sepc[idx][:, None, :] - sepc[None, :, :]) ** 2, axis=-1)
            neighbor_ids = [np.flatnonzero(row <= radius * radius) for row in d2]
        for i, nb in zip(idx, neighbor_ids):
            mask[i] = bool((ids[nb] != ids[i]).any())
        return mask

    def _publish_weld(self, seam):
        header = self._make_header(self._static_frame,
                                   self.get_clock().now().to_msg())
        red = np.tile(np.array([255, 20, 20], np.uint8), (len(seam), 1))
        self._weld_pub.publish(make_xyzrgb_cloud(header, seam, red))

    def _persist_weld(self, seam):
        try:
            self._save_dir.mkdir(parents=True, exist_ok=True)
            np.save(self._save_dir / 'welding_points.npy', seam)
            write_ply_points(self._save_dir / 'welding_points.ply', seam)
        except Exception as exc:  # noqa: BLE001 - persistence is best-effort
            return f'(warning: could not persist weld seam: {exc})'
        return f'wrote welding_points.ply/.npy to {self._save_dir}.'

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
            return xyz, estimate_normals_organized(xyz), self._camera_frame, None

        if self._latest_cloud is None:
            raise RuntimeError('no PointCloud2 received yet on the camera topic')
        msg = self._latest_cloud
        if msg.height <= 1:
            raise RuntimeError('cloud is not organized (height<=1); need the '
                               'depth_image_proc organized cloud for normals')
        arr = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        xyz = arr.reshape(msg.height, msg.width, 3).astype(np.float64)
        return (xyz, estimate_normals_organized(xyz),
                msg.header.frame_id or self._camera_frame, msg.header.stamp)

    # ── One refinement ───────────────────────────────────────────────────
    def _run_once(self):
        idx, det, init, mask = self._load_detection()
        # Match the CAD to whatever the server classified this frame as, before
        # ICP (and the tracker it seeds) touch self._model.
        self._load_model(str(det.get('obj_name') or ''))
        xyz, normals, frame_id, stamp = self._scene_organized()

        if mask.shape != xyz.shape[:2]:
            return False, (f'mask {mask.shape} does not match cloud '
                           f'{xyz.shape[:2]}; scene and FoundationPose frame differ')

        valid = mask & np.isfinite(xyz).all(2) & np.isfinite(normals).all(2)
        scene = xyz[valid]
        scene_n = normals[valid]
        if len(scene) < 10:
            return False, f'segmented scene has too few points ({len(scene)})'

        # Subtract already-assembled objects (only meaningful for the live cloud,
        # whose frame_id + stamp match the current TF; the stored depth_png frame
        # may predate arm motion, so skip it there).
        if self._scene_from == 'pointcloud':
            keep = self._static_keep_mask(scene, frame_id, stamp)
            if keep is not None:
                scene, scene_n = scene[keep], scene_n[keep]
                if len(scene) < 10:
                    return False, (f'only {len(scene)} points left after '
                                   'background subtraction')

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
        """Organized (H,W,3) cloud in its native dtype, plus frame and stamp.

        Deliberately *not* cast to float64 here: the ROI crop below slices this
        down to the crop box's pixel rectangle first, and casting the ~0.3 Mpx
        full frame only to throw away 80-95% of it is wasted bandwidth.
        """
        if self._latest_cloud is None:
            raise RuntimeError('no PointCloud2 received yet on the camera topic')
        msg = self._latest_cloud
        if msg.height <= 1:
            raise RuntimeError('cloud is not organized (height<=1)')
        arr = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False)
        xyz = arr.reshape(msg.height, msg.width, 3)
        frame = msg.header.frame_id or self._camera_frame
        return xyz, frame, msg.header.stamp

    # ── Pinhole K of the live cloud (recovered from the cloud itself) ──────
    def _pinhole_for(self, xyz):
        """Cached K for ``xyz``'s image size, or None if it isn't a projection.

        Read off the organized cloud rather than a CameraInfo subscription, so
        it can never disagree with the cloud actually being processed and needs
        no extra topic wiring. Only *successful* fits are cached, so a frame that
        arrives too empty to fit doesn't poison the cache for the whole session.
        """
        if not self._roi_crop:
            return None
        shape = xyz.shape[:2]
        if self._pinhole_shape == shape:
            return self._pinhole_K
        K = infer_pinhole_from_organized(xyz)
        if K is None:
            self.get_logger().warn(
                'could not recover the camera intrinsics from the organized '
                'cloud; cropping the full frame this tick (slower). Set '
                'roi_crop=false to silence this.', throttle_duration_sec=10.0)
            return None
        self._pinhole_K, self._pinhole_shape = K, shape
        self.get_logger().info(
            f'ROI crop enabled: recovered K from the {shape[1]}x{shape[0]} cloud '
            f'(fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} '
            f'cx={K[0, 2]:.1f} cy={K[1, 2]:.1f})')
        return self._pinhole_K

    # ── CropBox the live cloud around current_pose (the "macro-filter") ────
    def _crop_live_scene(self):
        """Return (scene_pts, scene_normals_or_None, frame_id, (lo, hi)).

        The crop box is a bounded 3-D region, so the camera sees it inside a
        bounded *rectangle*. That rectangle is computed first and the organized
        cloud is sliced to it, so the expensive per-point work (float64 cast,
        finiteness test, oriented-box test, and on the NumPy path the normals)
        only ever touches pixels that could possibly be inside the box. This is
        exact -- the surviving point set is identical to scanning the whole frame
        -- and it is the front-end speedup (38 ms -> 5 ms on a 640x480 cloud).
        When the box straddles the image plane or K is unavailable the ROI is
        None and the full-frame path runs unchanged.

        Open3D path: crop the raw cloud (normals computed later on the small
        cropped set). NumPy path: compute organized normals up-front, then crop
        both points and normals together. In both paths the cropped set then
        goes through Model-Based Background Subtraction (removing points that
        belong to already-assembled objects) before it is returned.
        """
        lo = self._model_lo - self._crop_margin
        hi = self._model_hi + self._crop_margin
        xyz, frame_id, stamp = self._live_xyz()

        K = self._pinhole_for(xyz)
        roi = (box_image_roi(xyz.shape[:2], self._current_pose, lo, hi, K,
                             pad=self._roi_pad) if K is not None else None)
        if roi is not None:
            v0, v1, u0, u1 = roi
            if not self._use_o3d:
                # estimate_normals_organized needs one pixel of context on each
                # side (it central-differences and NaNs the border), so widen the
                # slice by one before cutting it out of the frame.
                v0, v1 = max(v0 - 1, 0), min(v1 + 1, xyz.shape[0])
                u0, u1 = max(u0 - 1, 0), min(u1 + 1, xyz.shape[1])
            xyz = xyz[v0:v1, u0:u1]
        xyz = np.ascontiguousarray(xyz, dtype=np.float64)

        if self._use_o3d:
            pts = xyz[np.isfinite(xyz).all(2)]
            inside = crop_box_mask(pts, self._current_pose, lo, hi)
            pts = pts[inside]
            keep = self._static_keep_mask(pts, frame_id, stamp)
            if keep is not None:
                pts = pts[keep]
            return pts, None, frame_id, (lo, hi)

        normals = estimate_normals_organized(xyz)
        finite = np.isfinite(xyz).all(2) & np.isfinite(normals).all(2)
        pts, nrm = xyz[finite], normals[finite]
        inside = crop_box_mask(pts, self._current_pose, lo, hi)
        pts, nrm = pts[inside], nrm[inside]
        keep = self._static_keep_mask(pts, frame_id, stamp)
        if keep is not None:
            pts, nrm = pts[keep], nrm[keep]
        return pts, nrm, frame_id, (lo, hi)

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
        # Held for the whole tick: the services run on another thread now and
        # mutate the same pose/SEPC state. The cloud subscription is in its own
        # group and never waits on this.
        with self._state_lock:
            self._track_tick_locked()

    def _track_tick_locked(self):
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
                're-run FoundationPose + ~/run_icp to re-seed', throttle_duration_sec=2.0)
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

        # Build the NN structure once and share it between the noise-floor
        # measurement and every ICP iteration (~43 queries hit this same target).
        index = NNIndex(scene)
        T, info = icp_point_to_plane(
            self._model, scene, scene_n, init=self._current_pose,
            max_corr_dist=self._max_corr, max_iter=self._max_iter,
            anderson_depth=self._anderson, robust=self._robust,
            noise_floor=self._noise_floor_for(scene, scene_n, index),
            index=index)

        if info['fitness'] < self._lost_fitness:
            self._tracking = False
            self.get_logger().warn(
                f'tracking lost (fitness {info["fitness"]:.3f} < '
                f'{self._lost_fitness}); paused. Re-run FoundationPose + ~/run_icp.')
            return

        self._current_pose = T
        self._publish(frame_id, scene, T)
        self.get_logger().info(
            f'track: fitness={info["fitness"]:.3f} rmse={info["inlier_rmse"]:.4f}m '
            f'iters={info["iterations"]} scene={len(scene)}',
            throttle_duration_sec=1.0)

    # ── Welsch noise floor (a camera property, so cached across ticks) ────
    def _noise_floor_for(self, scene, scene_n, index):
        """The cached Welsch nu floor, re-measured every ``noise_floor_refresh``.

        ``welsch_nu_end`` measures the local surface thickness of the scene --
        i.e. the depth sensor's own noise. That does not change as the object
        moves, but re-deriving it costs ~2.5 ms of a tracking tick, so it is
        measured once and refreshed only occasionally (and whenever the tracker
        is re-seeded, which is where the scene really can change).
        """
        if self._noise_refresh > 0 and self._noise_floor is not None:
            self._noise_age += 1
            if self._noise_age < self._noise_refresh:
                return self._noise_floor
        if scene_n is None or len(scene) < 2:
            return self._noise_floor
        self._noise_floor = welsch_nu_end(scene, scene_n, index=index)
        self._noise_age = 0
        return self._noise_floor

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

    # ── TF + Model-Based Background Subtraction (assembly) ────────────────
    def _lookup_tf(self, target, source, stamp=None):
        """4x4 mapping points in ``source`` frame into ``target`` frame, or None.

        Tries the cloud's own stamp first (correct for the eye-in-hand camera at
        capture time), then falls back to the latest available transform.
        """
        from rclpy.time import Time
        whens = ([Time.from_msg(stamp)] if stamp is not None else []) + [Time()]
        for when in whens:
            try:
                tf = self._tf_buffer.lookup_transform(target, source, when)
            except TransformException:
                continue
            r = tf.transform.rotation
            t = tf.transform.translation
            T = np.eye(4)
            T[:3, :3] = quat_to_rotmat([r.x, r.y, r.z, r.w])
            T[:3, 3] = [t.x, t.y, t.z]
            return T
        return None

    def _static_keep_mask(self, pts_cam, frame_id, stamp):
        """Boolean 'keep' mask built in the static frame: ground cut + SEPC sub.

        Both filters need the points in ``static_frame``, so the (small) cropped
        live cloud is transformed there once via TF, then:
          * ground removal -- drop anything at/below ``ground_z_m`` (floor/table);
          * background subtraction -- drop points within ``bg_dist`` of the SEPC
            (an already-assembled object), so ICP never sees the previous part.
        Returns None when neither filter is active or TF is unavailable (caller
        then keeps every point).
        """
        want_ground = self._ground_removal
        want_sepc = self._bg_subtract and self._sepc is not None
        if (not (want_ground or want_sepc)) or len(pts_cam) == 0:
            return None
        T = self._lookup_tf(self._static_frame, frame_id, stamp)
        if T is None:
            self.get_logger().warn(
                f'static filter: TF {self._static_frame}<-{frame_id} unavailable; '
                'keeping all points this frame', throttle_duration_sec=2.0)
            return None
        pts_static = pts_cam @ T[:3, :3].T + T[:3, 3]
        keep = np.ones(len(pts_cam), dtype=bool)
        if want_ground:
            keep &= pts_static[:, 2] > self._ground_z        # delete the floor
        if want_sepc:
            if self._sepc_tree is not None:
                dist, _ = self._sepc_tree.query(pts_static, workers=-1)
            else:                               # no scipy: brute-force fallback
                _, dist = nearest_neighbor(pts_static, self._sepc)
            keep &= dist > self._bg_dist
        return keep

    def _save_object(self):
        """Bake the current object's CAD into the SEPC; clear tracking state."""
        frame_id = self._camera_frame
        stamp = None
        if self._latest_cloud is not None:
            frame_id = self._latest_cloud.header.frame_id or self._camera_frame
            stamp = self._latest_cloud.header.stamp
        T_sc = self._lookup_tf(self._static_frame, frame_id, stamp)
        if T_sc is None:
            return False, (f'TF {self._static_frame}<-{frame_id} unavailable; '
                           'cannot place the object in the static frame')

        # CAD model at its refined pose: model -> camera -> static_frame.
        T = self._current_pose
        model_cam = self._model @ T[:3, :3].T + T[:3, 3]
        model_static = model_cam @ T_sc[:3, :3].T + T_sc[:3, 3]
        pose_static = T_sc @ T                  # final model->static 6D pose

        self._sepc = (model_static if self._sepc is None
                      else np.vstack([self._sepc, model_static]))
        self._sepc_tree = _cKDTree(self._sepc) if _cKDTree is not None else None
        self._saved.append({'model': self._model_name,
                            'n_points': int(len(model_static)),
                            'pose_static': pose_static.tolist()})

        self._publish_sepc()
        note = self._persist_sepc()

        self._tracking = False                  # ready for the next object
        self._current_pose = None
        n = len(self._saved)
        self.get_logger().info(
            f'saved object #{n} ({self._model_name}) into SEPC: '
            f'+{len(model_static)} pts -> {len(self._sepc)} total. {note} '
            'Tracking cleared; trigger the next object and ~/run_icp -- its CAD is '
            'selected from the classifier automatically.')
        return True, (f'object #{n} ({self._model_name}) saved into SEPC '
                      f'({len(self._sepc)} pts total); tracking cleared')

    def _publish_sepc(self):
        if self._sepc is None:
            return
        header = self._make_header(self._static_frame,
                                   self.get_clock().now().to_msg())
        colors = np.tile(np.array([230, 120, 20], np.uint8),  # orange = frozen
                         (len(self._sepc), 1))
        self._sepc_pub.publish(make_xyzrgb_cloud(header, self._sepc, colors))

    def _persist_sepc(self):
        """Write the SEPC (.ply + .npy) and the assembly manifest (.json)."""
        try:
            self._save_dir.mkdir(parents=True, exist_ok=True)
            np.save(self._save_dir / 'static_env.npy', self._sepc)
            write_ply_points(self._save_dir / 'static_env.ply', self._sepc)
            (self._save_dir / 'assembly.json').write_text(json.dumps(
                {'static_frame': self._static_frame, 'objects': self._saved},
                indent=2))
        except Exception as exc:  # noqa: BLE001 - persistence is best-effort
            return f'(warning: could not persist SEPC: {exc})'
        return f'wrote static_env.ply/.npy + assembly.json to {self._save_dir}.'

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
    # MultiThreaded, not the default single-threaded spin: the tracking tick
    # takes tens of milliseconds, and on one thread it blocks the pointcloud
    # subscription for that whole time, so each tick refines against a frame at
    # least one tick old. The node's three callback groups (cloud / timer /
    # services) let the newest cloud keep landing while ICP runs; shared state is
    # guarded by the node's lock. Three threads is enough -- one per group.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        # SIGINT gives KeyboardInterrupt, but SIGTERM (ros2 launch teardown, a
        # supervisor) just invalidates the context underneath spin(), which
        # surfaces as ExternalShutdownException or -- if a worker thread is
        # already rebuilding its wait set -- a raw RCLError. All three mean the
        # same thing here: stop spinning and tear down.
        pass
    finally:
        # Teardown itself races the shutdown that got us here, and a second
        # signal can land mid-destroy; none of that should turn a normal Ctrl-C
        # into a traceback and a non-zero exit.
        for close in (executor.shutdown, node.destroy_node):
            try:
                close()
            except (KeyboardInterrupt, Exception):  # noqa: B014 - best effort
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
