"""`~/refine_pose`: the multi-view close-range refinement on the robot (step 5 of
notes/multiview_refine_plan.md). Attached to the ICP node, which owns the saved parts.

One call, after the parts are saved and before welding_points:

    ros2 service call /icp_pose_refiner/refine_pose std_srvs/srv/Trigger
    ros2 service call /icp_pose_refiner/abort_refine std_srvs/srv/Trigger    # stop it

1. plan the views (multiview.plan_views): around the centre of the saved parts, at
   `mv_view_distance_m`, collision-free and on the elbow-up branch, from the current joints;
   publish them in RViz (/perception/icp/multiview_views);
2. `mv_dry_run` (default false, the user's choice 2026-10-01): stop there - only the views in RViz
   (`mv_capture_here`: no plan, no motion - one view from where the arm is, then 5.);
3. otherwise, per view: the transit (TrajectoryExecutor: force watchdog, joint-jump gate,
   bias re-measured before each move), settle, the per-pixel median of `mv_n_frames` FRESH
   organized clouds, base_link <- camera from TF at their stamp;
4. back home (the scan home of config/marking.json, the wrist unwound);
5. save the capture (<results>/multiview/<timestamp>/, multiview_capture.py), refine it
   (multiview_capture.refine_capture: the same code the offline replay runs), publish the
   stacked views (/perception/icp/multiview_cloud); the ICP node applies the accepted poses
   to its saved parts, rebuilds and persists the SEPC + assembly.json;
6. record the extrinsic estimate into notebooks/selfcal_history.json (`mv_record_selfcal`).

Safety, before any motion: at least one saved part; joint states; the extrinsic in TF
(tool0 -> camera) equal to `mv_extrinsic_path` (the file the capture records and the
self-calibration is keyed on); the views planned from where the arm IS. Every move is a
collision-checked path at the transit clearance, watched at `mv_abort_force_n`. A failed
or aborted move stops the run where it is: no automatic recovery motion.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from . import multiview as mv
from . import multiview_capture as mc
from . import multiview_refine as mr
from . import selfcal as sc
from .collision import CollisionModel
from .kinematics import active_kinematics, use_kinematics
from .marking import time_joint_path
from .motion import TrajectoryExecutor
from .tack_reach import load_marking_config
from .tool_model import load_tool_model

PKG = Path(__file__).resolve().parents[1]

PARAMS: dict[str, Any] = {
    "dry_run": False,
    # no motion at all: one view from where the arm IS, then the normal capture / refine /
    # apply path - for testing that path on live data, or a quick "refine from here"
    "capture_here": False,
    "view_distance_m": 0.40,
    "n_views": 4,
    "elevations_deg": [45.0, 60.0],
    "azimuth_step_deg": 30.0,
    "max_incidence_deg": 60.0,
    "n_frames": 16,                  # 8 until 2026-10-02 (the user: more frames, they are cheap)
    "settle_s": 0.5,
    "frame_timeout_s": 8.0,          # the cloud arrives at ~6 Hz: 16 frames take ~2.7 s
    "v_joint_rad_s": 0.3,
    "extrinsic_path": str(PKG / "notebooks" / "T_tcp_to_cam.npy"),
    "kinematics_file": str(PKG / "config" / "ur5e_calibration.yaml"),
    "tool_config": "",
    "marking_config": "",
    "extrinsic_tf_tol_mm": 0.1,
    "record_selfcal": True,
    "selfcal_history": str(PKG / "notebooks" / "selfcal_history.json"),
    "online_selfcal": True,
    "max_correction_mm": 10.0,
    "max_correction_deg": 4.0,
    # the pull toward the saved pose (multiview_refine.RefineConfig.prior_sigma_*). A
    # looser prior trusts the views more - but on the 2026-10-06 capture 3 / 8 / 15 mm
    # changed the result by < 0.5 mm: the four close views already dominate.
    "prior_sigma_mm": 3.0,
    "prior_sigma_deg": 1.0,
    "max_relative_mm": 2.0,
    "max_relative_deg": 1.0,
    "resting_contact": False,
    "quality_level": "C",
    "voxel_m": 0.003,
}


def _T_from_tf(node, target: str, source: str, stamp=None) -> Optional[np.ndarray]:
    return node._lookup_tf(target, source, stamp)


class MultiviewRefiner:
    """The orchestration behind ~/refine_pose (module doc). `node` is the ICP node: its
    `_latest_cloud`, `_lookup_tf`, `_static_frame`, `_saved`, `_sepc_or_load`,
    `_apply_multiview` and `_model_dir` are used."""

    def __init__(self, node, callback_group) -> None:
        self.node = node
        for name, default in PARAMS.items():
            node.declare_parameter("mv_" + name, default)
        kin = str(self.p("kinematics_file"))
        node.get_logger().info("refine_pose kinematics: " + use_kinematics(kin or None))
        self.exec = TrajectoryExecutor(node, prefix="mv_", touch_force_n=1e9, callback_group=callback_group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.views_pub = node.create_publisher(MarkerArray, "/perception/icp/multiview_views", latched)
        self.cloud_pub = node.create_publisher(PointCloud2, "/perception/icp/multiview_cloud", latched)
        self.busy = False

    def p(self, name: str):
        return self.node.get_parameter("mv_" + name).value

    # ---------------------------------------------------------------- pieces ------------
    def _configs(self):
        vcfg = mv.ViewConfig(distance_m=float(self.p("view_distance_m")), n_views=int(self.p("n_views")),
                             elevations_deg=tuple(float(e) for e in self.p("elevations_deg")),
                             azimuth_step_deg=float(self.p("azimuth_step_deg")),
                             max_incidence_deg=float(self.p("max_incidence_deg")))
        rcfg = mr.RefineConfig(online_selfcal=bool(self.p("online_selfcal")),
                               max_correction_mm=float(self.p("max_correction_mm")),
                               max_correction_deg=float(self.p("max_correction_deg")),
                               prior_sigma_mm=float(self.p("prior_sigma_mm")),
                               prior_sigma_deg=float(self.p("prior_sigma_deg")),
                               max_relative_mm=float(self.p("max_relative_mm")),
                               max_relative_deg=float(self.p("max_relative_deg")),
                               resting_contact=bool(self.p("resting_contact")),
                               quality_level=str(self.p("quality_level")))
        pcfg = mc.PreprocessConfig(voxel_m=float(self.p("voxel_m")))
        return vcfg, rcfg, pcfg

    def _check_extrinsic(self, tool) -> Optional[str]:
        """The extrinsic in TF must be the file the capture will record."""
        node = self.node
        cam_frame = tool.camera_frame or node._camera_frame
        T_tf = _T_from_tf(node, "tool0", cam_frame)
        if T_tf is None:
            return f"no TF tool0 <- {cam_frame}"
        T_file = tool.T_tool0_cam
        dt = np.linalg.norm(T_tf[:3, 3] - T_file[:3, 3]) * 1000
        dr = np.degrees(np.linalg.norm(mr._rotvec(T_tf[:3, :3] @ T_file[:3, :3].T)))
        if dt > float(self.p("extrinsic_tf_tol_mm")) or dr > 0.01:
            return (f"the extrinsic in TF differs from mv_extrinsic_path={self.p('extrinsic_path')} by "
                    f"{dt:.2f} mm / {dr:.3f} deg - set mv_extrinsic_path to the file the launch publishes")
        return None

    def _publish_views(self, plan: mv.ViewPlan) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        frame = self.node._static_frame
        for k, v in enumerate(plan.views):
            c, z = v.T_cam[:3, 3], v.T_cam[:3, 2]
            arrow = Marker()
            arrow.header.frame_id = frame
            arrow.ns, arrow.id, arrow.type, arrow.action = "views", 2 * k, Marker.ARROW, Marker.ADD
            arrow.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])),
                            Point(x=float(c[0] + 0.1 * z[0]), y=float(c[1] + 0.1 * z[1]), z=float(c[2] + 0.1 * z[2]))]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.02
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.2, 0.6, 1.0, 1.0
            arrow.pose.orientation.w = 1.0
            text = Marker()
            text.header.frame_id = frame
            text.ns, text.id, text.type, text.action = "views", 2 * k + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = float(c[0]), float(c[1]), float(c[2] + 0.03)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.03
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f"view {k + 1}"
            ma.markers.append(arrow)
            ma.markers.append(text)
        t = Marker()
        t.header.frame_id = frame
        t.ns, t.id, t.type, t.action = "views", 1000, Marker.SPHERE, Marker.ADD
        t.pose.position.x, t.pose.position.y, t.pose.position.z = (float(x) for x in plan.target_m)
        t.pose.orientation.w = 1.0
        t.scale.x = t.scale.y = t.scale.z = 0.015
        t.color.r, t.color.g, t.color.a = 1.0, 0.8, 1.0
        ma.markers.append(t)
        self.views_pub.publish(ma)

    def _publish_cloud(self, views) -> None:
        pal = np.array([[230, 60, 60], [60, 200, 60], [60, 120, 240], [230, 200, 40], [200, 60, 220],
                        [40, 210, 210]], np.uint8)
        pts = np.vstack([v.pts for v in views]) if views else np.zeros((0, 3))
        col = np.vstack([np.tile(pal[k % len(pal)], (len(v.pts), 1)) for k, v in enumerate(views)]) \
            if views else np.zeros((0, 3), np.uint8)
        from std_msgs.msg import Header
        h = Header()
        h.frame_id = self.node._static_frame
        h.stamp = self.node.get_clock().now().to_msg()
        self.cloud_pub.publish(self.node._make_cloud(h, pts, col))

    def _grab(self, n: int, timeout: float, since_ns: int):
        """The per-pixel median of n fresh organized clouds (stamped after `since_ns`),
        their frame id and the stamp of the first."""
        frames, stamps, last = [], [], None
        t_end = time.time() + timeout
        frame_id = None
        while len(frames) < n and time.time() < t_end:
            msg = self.node._latest_cloud
            if msg is not None and msg is not last:
                st = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if st > since_ns and msg.height > 1:
                    arr = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=False)
                    frames.append(arr.reshape(msg.height, msg.width, 3).astype(np.float32))
                    stamps.append(msg.header.stamp)
                    frame_id = msg.header.frame_id or self.node._camera_frame
                last = msg
            time.sleep(0.01)
        if len(frames) < max(1, n // 2):
            return None, None, None
        with np.errstate(all="ignore"):
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                med = np.nanmedian(np.stack(frames), axis=0).astype(np.float32)
        return med, frame_id, stamps[0]

    # ---------------------------------------------------------------- the run ----------
    def run(self) -> tuple[bool, str]:
        if self.busy:
            return False, "refine_pose is already running"
        self.busy = True
        try:
            return self._run()
        finally:
            self.busy = False

    def _run(self) -> tuple[bool, str]:
        node = self.node
        node._sepc_or_load()
        if not node._saved:
            return False, "no saved parts: call ~/save_object first"
        q0 = self.exec.current_q()
        if q0 is None:
            return False, "no /joint_states (is the UR driver running?)"
        self.exec.clear_abort()
        vcfg, rcfg, pcfg = self._configs()
        mcfg = load_marking_config(str(self.p("marking_config")) or None)
        tool = load_tool_model(str(self.p("tool_config")) or None, str(self.p("extrinsic_path")))
        if tool.T_tool0_cam is None:
            return False, f"no extrinsic at mv_extrinsic_path={self.p('extrinsic_path')}"
        err = self._check_extrinsic(tool)
        if err:
            return False, err
        models = node._model_dir()
        assembly = {"static_frame": node._static_frame, "objects": [dict(o) for o in node._saved]}
        if bool(self.p("capture_here")):
            raw = self._capture_view({"capture_here": True, "q": np.asarray(q0).tolist()})
            if isinstance(raw, str):
                return False, raw
            return self._finish([raw], assembly, models, rcfg, pcfg, "capture_here: one view, no motion")
        objects = [(o["model"], np.asarray(o["pose_static"], float)) for o in node._saved]
        surfaces, boxes = mv.surfaces_from_models(objects, models)
        model = CollisionModel(tool=tool, scene_boxes=boxes, table_z=mcfg.table_z_m, clearance=mcfg.clearance_m)
        self.exec.set_sim(q0)
        plan = mv.plan_views(surfaces, boxes, tool, model, mcfg, q0, vcfg)
        self._publish_views(plan)
        summary = plan.summary()
        node.get_logger().info(summary)
        if not plan.ok:
            return False, summary
        if self.exec.dry_run:
            return True, ("DRY RUN (mv_dry_run:=true) - views planned and shown in RViz "
                          "(/perception/icp/multiview_views), nothing moved:\n" + summary)

        # pause tracking for the whole run (the live crop would chase a moving camera)
        with node._state_lock:
            was_tracking, node._tracking = node._tracking, False
        try:
            raws = []
            v_joint = float(self.p("v_joint_rad_s"))
            for k, (path, view) in enumerate(zip(plan.paths, plan.views)):
                ok, msg = self.exec.execute(time_joint_path(path, v_joint), f"view {k + 1} transit", rebias=True)
                if not ok or self.exec.aborted:
                    return False, f"stopped at view {k + 1}: {msg} (the arm stays where it is)"
                time.sleep(float(self.p("settle_s")))
                raw = self._capture_view({"azimuth_deg": view.azimuth_deg, "elevation_deg": view.elevation_deg,
                                          "roll_deg": view.roll_deg,
                                          "q": np.asarray(self.exec.current_q()).tolist()})
                if isinstance(raw, str):
                    return False, f"view {k + 1}: {raw}"
                raws.append(raw)
                node.get_logger().info(f"view {k + 1}/{len(plan.views)} captured "
                                       f"({int(np.isfinite(raw.xyz).all(2).sum())} valid pixels)")
            ok, msg = self.exec.execute(time_joint_path(plan.home_path, v_joint), "home", rebias=True)
            if not ok:
                node.get_logger().warn(f"refine_pose: the way home failed ({msg}); the capture is kept")
        finally:
            with node._state_lock:
                node._tracking = was_tracking and node._current_pose is not None

        return self._finish(raws, assembly, models, rcfg, pcfg, summary)

    def _capture_view(self, meta: dict):
        """RawView from the median of fresh frames at the current pose, or an error text."""
        node = self.node
        since = node.get_clock().now().nanoseconds
        xyz, frame, stamp = self._grab(int(self.p("n_frames")), float(self.p("frame_timeout_s")), since)
        if xyz is None:
            return "no fresh organized clouds on the camera topic"
        T_bc = node._lookup_tf(node._static_frame, frame, stamp)
        if T_bc is None:
            return f"no TF {node._static_frame} <- {frame}"
        return mc.RawView(xyz, T_bc, meta)

    def _finish(self, raws, assembly, models, rcfg, pcfg, summary: str) -> tuple[bool, str]:
        """Save the capture, refine it, apply, record - after the motion."""
        node = self.node
        ext = str(self.p("extrinsic_path"))
        out = node._save_dir / "multiview" / time.strftime("%Y%m%d-%H%M%S")
        mc.save_capture(out, raws, assembly, {
            "extrinsic_file": ext, "extrinsic_sha1": sc.file_sha1(ext), "kinematics": active_kinematics(),
            "ground_plane_file": str(node.get_parameter("ground_plane_file").value),
            "plan": summary})
        cap = mc.load_capture(out)
        rp = mc.refine_capture(cap, models, rcfg, pcfg)
        self._publish_cloud(rp.views)
        report = mc.format_replay(rp)
        applied = node._apply_multiview(rp.results, str(out))
        lines = [report, applied]
        sig = rp.diag.extrinsic_sigma_mm
        determined = sig is not None and bool((np.asarray(sig) <= rcfg.extrinsic_max_sigma_mm).any())
        if bool(self.p("record_selfcal")) and rp.diag.extrinsic_d_mm is not None and not determined:
            lines.append("not recorded for the self-calibration: these views determine no direction of d")
        if bool(self.p("record_selfcal")) and rp.diag.extrinsic_d_mm is not None and determined:
            e = sc.record_run(str(self.p("selfcal_history")), rp.diag.extrinsic_d_mm, rp.diag.extrinsic_info, ext,
                              {"capture": str(out), "n_views": len(rp.views)})
            lines.append(f"recorded d = {np.round(e['d_cam_mm'], 2).tolist()} mm for the self-calibration "
                         f"(python3 scripts/selfcal_extrinsic.py)")
        text = "\n".join(lines)
        node.get_logger().info(text)
        return all(r.accepted for r in rp.results), text
