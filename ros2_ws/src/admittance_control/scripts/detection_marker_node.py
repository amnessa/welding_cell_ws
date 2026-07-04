#!/usr/bin/env python3
"""SAM-6D detection -> RViz MarkerArray.

RViz2 has no native display for ``vision_msgs/Detection3DArray``, so the SAM-6D
bridge's ``/perception/detections`` output is invisible even though it is being
published. This node converts each detection into RViz markers so the pose
estimation (the detection_pem R,t) can be seen in the pointcloud.

Per detection it draws, in the detection's own frame (camera_color_optical_frame):
  - a coordinate triad (LINE_LIST): X red, Y green, Z blue, at the estimated pose
    -> shows the full 6D transform (position + orientation)
  - a small sphere at the position
  - a text label "<class_id>: <score>"

For the single highest-scoring detection it additionally draws an oriented 3D
bounding box (a wireframe cuboid, like rviz_visual_tools' publishWireframeCuboid):
the CAD model's min/max extent, placed at the estimated pose. The box edges are a
LINE_LIST in the model's local frame with marker.pose = the detection pose, so the
box sits exactly where SAM-6D thinks the object is -- a direct visual check of the
pose against the pointcloud. The model origin need not be the model centre (e.g.
test_objv3.ply's origin is a corner), so the local min/max is used verbatim.

Subscribes
----------
  <detections_topic>  vision_msgs/Detection3DArray  (default /perception/detections)

Publishes
---------
  <markers_topic>     visualization_msgs/MarkerArray (default /perception/detection_markers)

Notes
-----
- Uses LINE_LIST with marker.pose set to the detection pose, so RViz rotates the
  local axis segments for us -- no manual quaternion math needed.
- Both subscription and publication are TRANSIENT_LOCAL (latched) to match the
  bridge's latched detections and so a late-joining RViz still gets the markers.
- ``min_score`` drops low-confidence detections; ``axis_length_m`` /
  ``line_width_m`` size the triad.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


# The 12 edges of a cuboid as index pairs into the 8 corners produced by
# _corner_order() (corner index bits = x<<2 | y<<1 | z, low=min, high=max).
_CUBE_EDGES = ((0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
               (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7))


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def load_ply_bounds(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Return (min_xyz, max_xyz) of a PLY's vertices (ascii or binary LE).

    Minimal parser (no open3d dependency): reads the header, then the first
    three float properties of each vertex. Sufficient for axis-aligned bounds.
    """
    with open(path, 'rb') as f:
        header = b''
        while b'end_header' not in header:
            line = f.readline()
            if not line:
                raise ValueError(f'{path}: no end_header found')
            header += line
        lines = header.decode('ascii', 'replace').splitlines()
        fmt = next(l for l in lines if l.startswith('format'))
        n = int(next(l for l in lines if l.startswith('element vertex')).split()[-1])
        props = [l.split()[-1] for l in lines
                 if l.startswith('property') and 'list' not in l]
        if 'ascii' in fmt:
            verts = np.array([[float(x) for x in f.readline().split()[:3]]
                              for _ in range(n)])
        else:  # binary_little_endian, float32 properties
            stride = len(props)
            data = np.frombuffer(f.read(n * stride * 4),
                                 dtype='<f4').reshape(n, stride)
            verts = data[:, :3].astype(np.float64)
    return verts.min(axis=0), verts.max(axis=0)


def _corner_order(mn: np.ndarray, mx: np.ndarray):
    """The 8 cuboid corners ordered to match _CUBE_EDGES bit indexing."""
    axes = list(zip(mn, mx))
    return [(axes[0][xi], axes[1][yi], axes[2][zi])
            for xi in (0, 1) for yi in (0, 1) for zi in (0, 1)]


class DetectionMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('detection_marker')

        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('markers_topic', '/perception/detection_markers')
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('axis_length_m', 0.05)
        self.declare_parameter('line_width_m', 0.005)
        self.declare_parameter('sphere_diameter_m', 0.01)
        self.declare_parameter('text_height_m', 0.02)
        # Oriented CAD bounding box for the single highest-scoring detection.
        self.declare_parameter('bbox_model_path', '')      # '' disables the box
        self.declare_parameter('bbox_model_units', 'mm')   # CAD units -> metres
        self.declare_parameter('bbox_line_width_m', 0.004)

        self._min_score = float(self.get_parameter('min_score').value)
        self._axis_len = float(self.get_parameter('axis_length_m').value)
        self._line_w = float(self.get_parameter('line_width_m').value)
        self._sphere_d = float(self.get_parameter('sphere_diameter_m').value)
        self._text_h = float(self.get_parameter('text_height_m').value)
        self._bbox_line_w = float(self.get_parameter('bbox_line_width_m').value)

        # Load the CAD bounds once (metres). None => no box drawn.
        self._bbox_min = self._bbox_max = None
        model_path = str(self.get_parameter('bbox_model_path').value)
        if model_path:
            units = str(self.get_parameter('bbox_model_units').value).lower()
            scale = 0.001 if units == 'mm' else 1.0
            try:
                mn, mx = load_ply_bounds(Path(model_path).expanduser())
                self._bbox_min, self._bbox_max = mn * scale, mx * scale
                self.get_logger().info(
                    f'CAD bbox from {model_path}: '
                    f'extent(m)={np.round(self._bbox_max - self._bbox_min, 4)}')
            except Exception as exc:  # noqa: BLE001 - box is best-effort
                self.get_logger().warn(
                    f'could not load bbox model {model_path}: {exc}; '
                    f'skipping CAD box')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        detections_topic = str(self.get_parameter('detections_topic').value)
        markers_topic = str(self.get_parameter('markers_topic').value)
        self._pub = self.create_publisher(MarkerArray, markers_topic, latched)
        self.create_subscription(
            Detection3DArray, detections_topic, self._on_detections, latched)

        self.get_logger().info(
            f'detection markers: {detections_topic} -> {markers_topic} '
            f'(min_score={self._min_score}, axis_length={self._axis_len} m)')

    def _on_detections(self, msg: Detection3DArray) -> None:
        out = MarkerArray()
        # Clear the previous frame's markers so stale poses don't linger.
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)

        drawn = 0
        best_idx, best_pose, best_score = -1, None, self._min_score
        for idx, det in enumerate(msg.detections):
            score = det.results[0].hypothesis.score if det.results else 0.0
            if score < self._min_score:
                continue
            pose = det.results[0].pose.pose if det.results else det.bbox.center
            class_id = det.results[0].hypothesis.class_id if det.results else det.id

            out.markers.append(self._triad(msg.header, idx, pose))
            out.markers.append(self._sphere(msg.header, idx, pose))
            out.markers.append(self._label(msg.header, idx, pose, class_id, score))
            drawn += 1

            # Track the single highest-scoring detection for the CAD box.
            if score >= best_score:
                best_idx, best_pose, best_score = idx, pose, score

        # Oriented CAD wireframe box on the best detection only.
        if best_pose is not None and self._bbox_min is not None:
            out.markers.append(self._cad_box(msg.header, best_pose))

        self._pub.publish(out)
        self.get_logger().info(
            f'drew {drawn}/{len(msg.detections)} detection(s) as markers'
            + (f'; CAD box on det #{best_idx} (score {best_score:.2f})'
               if best_pose is not None and self._bbox_min is not None else ''),
            throttle_duration_sec=1.0)

    def _base(self, header, ns: str, idx: int, mtype: int) -> Marker:
        m = Marker()
        m.header = header
        m.ns = ns
        m.id = idx
        m.type = mtype
        m.action = Marker.ADD
        m.frame_locked = True
        return m

    def _triad(self, header, idx: int, pose) -> Marker:
        m = self._base(header, 'pose_axes', idx, Marker.LINE_LIST)
        m.pose = pose  # RViz rotates the local axis segments by the pose.
        m.scale.x = self._line_w
        L = self._axis_len
        red, green, blue = _color(1, 0, 0), _color(0, 1, 0), _color(0, 0, 1)
        origin = Point(x=0.0, y=0.0, z=0.0)
        m.points = [origin, Point(x=L, y=0.0, z=0.0),
                    origin, Point(x=0.0, y=L, z=0.0),
                    origin, Point(x=0.0, y=0.0, z=L)]
        m.colors = [red, red, green, green, blue, blue]
        return m

    def _sphere(self, header, idx: int, pose) -> Marker:
        m = self._base(header, 'pose_point', idx, Marker.SPHERE)
        m.pose = pose
        m.scale.x = m.scale.y = m.scale.z = self._sphere_d
        m.color = _color(1.0, 1.0, 0.0, 0.9)
        return m

    def _cad_box(self, header, pose) -> Marker:
        """Oriented CAD wireframe cuboid at the pose (12 edges, LINE_LIST).

        Corners are in the model's local frame (metres); marker.pose = the
        detection pose, so RViz rotates+translates the box onto the object.
        """
        m = self._base(header, 'detection_bbox', 0, Marker.LINE_LIST)
        m.pose = pose
        m.scale.x = self._bbox_line_w
        m.color = _color(0.1, 1.0, 0.2, 1.0)  # bright green, like a fit box
        corners = _corner_order(self._bbox_min, self._bbox_max)

        def pt(c):
            return Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))

        pts = []
        for a, b in _CUBE_EDGES:
            pts.extend((pt(corners[a]), pt(corners[b])))
        m.points = pts
        return m

    def _label(self, header, idx: int, pose, class_id: str, score: float) -> Marker:
        m = self._base(header, 'pose_label', idx, Marker.TEXT_VIEW_FACING)
        m.pose.position.x = pose.position.x
        m.pose.position.y = pose.position.y
        m.pose.position.z = pose.position.z + self._text_h
        m.pose.orientation.w = 1.0
        m.scale.z = self._text_h
        m.color = _color(1.0, 1.0, 1.0, 1.0)
        m.text = f'{class_id}: {score:.2f}'
        return m


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DetectionMarkerNode()
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
