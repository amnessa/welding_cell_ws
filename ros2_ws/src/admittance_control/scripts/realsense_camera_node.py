#!/usr/bin/env python3
"""RealSense D435-series camera node.

Streams color + depth from the wrist RealSense and publishes the topics the
pose-server bridge (`foundationpose_bridge_node.py`; the older
`sam6d_bridge_node.py` reads the same ones), the FoundationPose tracker and the
gesture node consume:

Publishes
---------
  <rgb_topic>          sensor_msgs/Image       (default /camera/color/image_raw, rgb8)
  <depth_topic>        sensor_msgs/Image       (default /camera/depth/image_rect_raw, 16UC1 mm)
  <camera_info_topic>  sensor_msgs/CameraInfo  (default /camera/color/camera_info)

Design notes
------------
- Depth is aligned to the color frame (``rs.align``) so both images share the
  same pixels and the same intrinsics. The pose server needs a single ``cam_K``
  that is valid for the RGB image *and* the registered depth, which is exactly
  what alignment gives us.
- Both images and the CameraInfo carry one shared stamp, so the bridges' and the
  tracker's RGB/depth pairing (``max_sync_delta_sec``) always passes. The stamp is
  the COLOUR frame's capture time on the host clock (``stamp_source``, see
  ``admittance_control/frame_stamp.py``): by default the middle of its exposure,
  from librealsense's global time and the frame metadata. Until 2026-10-22 it was
  ``now()`` after the alignment, the publish time, so every consumer that looks the
  camera pose up at the stamp (the aim's latency compensation, the tracker's
  ego-motion, TF of a captured frame) got the pose of a later instant. A frame
  whose own time is unusable is stamped ``now()``, with a throttled warning; every
  ``stamp_report_sec`` the node logs how old the stamps were when published.
- Depth is published as 16UC1 in **millimetres**. The bridge and the pose server
  treat the depth PNG as ``depth_mm = pixel * depth_scale``; with millimetre
  pixels the matching ``depth_scale`` is ``1.0`` (see camera.json).
- With ``write_camera_json`` (default true) the node writes the live color
  intrinsics to ``scripts/rgb_depth_to_send/camera.json`` so the bridge always
  POSTs the intrinsics of the frames it actually received.
"""

from __future__ import annotations

import collections
import json
import time
from typing import Optional

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from admittance_control.frame_stamp import (SOURCES, FrameTimes, age_summary,
                                            capture_stamp_ns)
from admittance_control.sam6d_io import resolve_transfer_dir


def make_image_msg(data: np.ndarray, encoding: str, stamp, frame_id: str) -> Image:
    """Build a sensor_msgs/Image from a contiguous numpy array."""
    array = np.ascontiguousarray(data)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(array.strides[0])
    msg.data = array.tobytes()
    return msg


def frame_times(frame) -> FrameTimes:
    """A frame's own timing: its timestamp, the timestamp's domain and the two
    device-clock metadata `frame_stamp.capture_stamp_ns` needs (None when absent)."""
    def md(key):
        if not frame.supports_frame_metadata(key):
            return None
        return int(frame.get_frame_metadata(key))
    return FrameTimes(float(frame.get_timestamp()),
                      frame.get_frame_timestamp_domain().name,
                      md(rs.frame_metadata_value.frame_timestamp),
                      md(rs.frame_metadata_value.sensor_timestamp))


def make_camera_info(intrinsics: rs.intrinsics, stamp, frame_id: str) -> CameraInfo:
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width = int(intrinsics.width)
    info.height = int(intrinsics.height)
    # RealSense reports Brown-Conrady / inverse Brown-Conrady; both map to plumb_bob.
    info.distortion_model = 'plumb_bob'
    info.d = [float(c) for c in intrinsics.coeffs]
    fx, fy = float(intrinsics.fx), float(intrinsics.fy)
    cx, cy = float(intrinsics.ppx), float(intrinsics.ppy)
    info.k = [fx, 0.0, cx,
              0.0, fy, cy,
              0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0,
              0.0, fy, cy, 0.0,
              0.0, 0.0, 1.0, 0.0]
    return info


class RealSenseCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('realsense_camera')

        self.declare_parameter('serial_no', '')
        self.declare_parameter('color_width', 1280)
        self.declare_parameter('color_height', 720)
        self.declare_parameter('depth_width', 1280)
        self.declare_parameter('depth_height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('write_camera_json', True)
        # header.stamp: 'mid_exposure' (the colour frame's mid-exposure on the host
        # clock), 'frame' (its start of readout), 'now' (the publish time, as before
        # 2026-10-22); a frame time further than max_stamp_offset_sec from now is
        # not trusted
        self.declare_parameter('stamp_source', 'mid_exposure')
        self.declare_parameter('max_stamp_offset_sec', 0.5)
        self.declare_parameter('stamp_report_sec', 30.0)    # 0 = no report

        serial_no = str(self.get_parameter('serial_no').value)
        color_width = int(self.get_parameter('color_width').value)
        color_height = int(self.get_parameter('color_height').value)
        depth_width = int(self.get_parameter('depth_width').value)
        depth_height = int(self.get_parameter('depth_height').value)
        fps = int(self.get_parameter('fps').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._write_camera_json = bool(self.get_parameter('write_camera_json').value)
        self._stamp_source = str(self.get_parameter('stamp_source').value)
        if self._stamp_source not in SOURCES:
            raise ValueError(f'stamp_source {self._stamp_source!r}: '
                             f'expected one of {SOURCES}')
        self._max_stamp_offset = float(
            self.get_parameter('max_stamp_offset_sec').value)
        self._stamp_report_sec = float(
            self.get_parameter('stamp_report_sec').value)
        self._stamp_ages_ms = collections.deque(maxlen=900)
        self._stamp_how = collections.Counter()
        self._depth_lag_ms = collections.deque(maxlen=900)
        self._next_report = time.monotonic() + self._stamp_report_sec

        rgb_topic = str(self.get_parameter('rgb_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self._rgb_pub = self.create_publisher(Image, rgb_topic, 10)
        self._depth_pub = self.create_publisher(Image, depth_topic, 10)
        self._info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)

        self._pipeline = rs.pipeline()
        config = rs.config()
        if serial_no:
            config.enable_device(serial_no)
        config.enable_stream(rs.stream.color, color_width, color_height,
                             rs.format.rgb8, fps)
        config.enable_stream(rs.stream.depth, depth_width, depth_height,
                             rs.format.z16, fps)

        self.get_logger().info(
            f'Starting RealSense: color {color_width}x{color_height}, '
            f'depth {depth_width}x{depth_height} @ {fps} fps'
            + (f' (serial {serial_no})' if serial_no else ''))
        profile = self._pipeline.start(config)

        # Frame timestamps on the host clock (librealsense's device-to-host
        # fit): what the image stamps are made from.
        for sensor in profile.get_device().query_sensors():
            if sensor.supports(rs.option.global_time_enabled):
                try:
                    sensor.set_option(rs.option.global_time_enabled, 1.0)
                except RuntimeError as exc:
                    self.get_logger().warn(
                        'global time not enabled on '
                        f'{sensor.get_info(rs.camera_info.name)}: {exc}')

        # Align depth into the color frame so both share one set of intrinsics.
        self._align = rs.align(rs.stream.color)

        # Raw z16 depth is in device depth units; convert to millimetres so the
        # published 16UC1 image matches camera.json depth_scale = 1.0.
        depth_scale_m = profile.get_device().first_depth_sensor().get_depth_scale()
        self._depth_units_to_mm = float(depth_scale_m) * 1000.0

        self._color_intrinsics = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        self.get_logger().info(
            f'Color intrinsics: fx={self._color_intrinsics.fx:.2f} '
            f'fy={self._color_intrinsics.fy:.2f} '
            f'ppx={self._color_intrinsics.ppx:.2f} '
            f'ppy={self._color_intrinsics.ppy:.2f}; '
            f'depth_scale={depth_scale_m:.6f} m/unit')

        if self._write_camera_json:
            self._write_camera_json_file()

        # Drop the first frames while auto-exposure settles.
        for _ in range(5):
            try:
                self._pipeline.wait_for_frames(2000)
            except RuntimeError:
                break

        self._timer = self.create_timer(1.0 / max(fps, 1), self._on_timer)
        self.get_logger().info(
            f'Publishing rgb={rgb_topic} depth={depth_topic} '
            f'camera_info={camera_info_topic} frame_id={self._frame_id}; '
            f'stamps: {self._stamp_source}')

    def _write_camera_json_file(self) -> None:
        intr = self._color_intrinsics
        cam_K = [
            float(intr.fx), 0.0, float(intr.ppx),
            0.0, float(intr.fy), float(intr.ppy),
            0.0, 0.0, 1.0,
        ]
        payload = {'cam_K': cam_K, 'depth_scale': 1.0}
        camera_path = resolve_transfer_dir() / 'camera.json'
        try:
            camera_path.write_text(
                json.dumps(payload, separators=(',', ':')), encoding='utf-8')
            self.get_logger().info(f'Wrote live intrinsics to {camera_path}')
        except OSError as exc:
            self.get_logger().warn(f'Could not write camera.json: {exc}')

    def _on_timer(self) -> None:
        try:
            frames = self._pipeline.wait_for_frames(1000)
        except RuntimeError as exc:
            self.get_logger().warn(f'wait_for_frames timed out: {exc}',
                                   throttle_duration_sec=5.0)
            return

        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            self.get_logger().warn('Incomplete frame (missing color or depth)',
                                   throttle_duration_sec=5.0)
            return

        color_image = np.asanyarray(color_frame.get_data())  # HxWx3 uint8, rgb8
        depth_raw = np.asanyarray(depth_frame.get_data())     # HxW uint16, device units

        if abs(self._depth_units_to_mm - 1.0) > 1e-6:
            depth_mm = np.clip(
                np.rint(depth_raw.astype(np.float32) * self._depth_units_to_mm),
                0.0, float(np.iinfo(np.uint16).max)).astype(np.uint16)
        else:
            depth_mm = depth_raw.astype(np.uint16, copy=False)

        # One shared stamp - the colour frame's capture time - keeps the
        # RGB/depth pairing of the bridges and the tracker exact.
        now_ns = self.get_clock().now().nanoseconds
        color_times = frame_times(color_frame)
        stamp_ns, how = capture_stamp_ns(color_times, now_ns, self._stamp_source,
                                         self._max_stamp_offset)
        self._note_stamp(now_ns, stamp_ns, how, color_times,
                         frames.get_depth_frame())
        stamp = Time(nanoseconds=stamp_ns).to_msg()
        self._rgb_pub.publish(
            make_image_msg(color_image, 'rgb8', stamp, self._frame_id))
        self._depth_pub.publish(
            make_image_msg(depth_mm, '16UC1', stamp, self._frame_id))
        self._info_pub.publish(
            make_camera_info(self._color_intrinsics, stamp, self._frame_id))

    def _note_stamp(self, now_ns: int, stamp_ns: int, how: str,
                    color_times: FrameTimes, depth_frame) -> None:
        """Warn when a frame's own time was not usable; every
        `stamp_report_sec`, log how old the stamps were at publish and how far
        the depth frame's time is from the colour frame's."""
        self._stamp_how[how.split(':')[0]] += 1
        if how.startswith('now: '):
            self.get_logger().warn(f'image stamped {how}',
                                   throttle_duration_sec=5.0)
        elif how != 'now':
            self._stamp_ages_ms.append((now_ns - stamp_ns) / 1e6)
            if depth_frame and (depth_frame.get_frame_timestamp_domain().name
                                == color_times.domain):
                self._depth_lag_ms.append(float(depth_frame.get_timestamp())
                                          - color_times.timestamp_ms)
        if (self._stamp_report_sec <= 0
                or time.monotonic() < self._next_report):
            return
        self._next_report = time.monotonic() + self._stamp_report_sec
        lag = ('; depth - colour frame time median '
               f'{np.median(self._depth_lag_ms):+.1f} ms'
               if self._depth_lag_ms else '')
        self.get_logger().info(
            f'image stamps ({dict(self._stamp_how)}): age at publish '
            f'{age_summary(self._stamp_ages_ms)}{lag}')
        self._stamp_how.clear()
        self._stamp_ages_ms.clear()
        self._depth_lag_ms.clear()

    def destroy_node(self) -> bool:
        try:
            self._pipeline.stop()
        except RuntimeError:
            pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = RealSenseCameraNode()
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
