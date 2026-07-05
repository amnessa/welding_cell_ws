#!/usr/bin/env python3
"""Simulated-camera sensor-noise relay.

Bridges the clean RGB-D that Isaac Sim publishes into the noisy topics the rest
of the pipeline (SAM-6D bridge etc.) consumes, so simulation better matches the
real RealSense D435.

Subscribes (clean, from Isaac Sim)
----------------------------------
  <color_in_topic>  sensor_msgs/Image   (default /camera/color/image_raw_sim)
  <depth_in_topic>  sensor_msgs/Image   (default /camera/depth/image_rect_raw_sim)

Publishes (noisy, same names the real camera node would use)
------------------------------------------------------------
  <color_out_topic>   sensor_msgs/Image      (default /camera/color/image_raw)
  <depth_out_topic>   sensor_msgs/Image      (default /camera/depth/image_rect_raw)
  <camera_info_topic> sensor_msgs/CameraInfo (only if publish_camera_info=true)

Digital-twin fixups (parameters, off by default so real-camera use is unchanged)
--------------------------------------------------------------------------------
- ``output_frame_id``: Isaac's ROS2CameraHelper stamps the images with an
  empty frame_id that is not in the robot TF tree, so RViz can't place the
  pointcloud in a robot/world fixed frame. Set this to the frame the static
  camera extrinsic feeds (``camera_color_optical_frame``) to rewrite it.
- ``publish_camera_info``: Isaac's camera graph in welding_world.usda has no
  CameraInfo publisher, but depth_image_proc's PointCloudXyzrgb needs one.
  Enable this to synthesize CameraInfo from ``fx/fy/cx/cy`` (defaults = the
  real RealSense intrinsics, which the twin shares).

Noise model
-----------
- Depth: additive Gaussian with a distance-dependent standard deviation
      sigma(z) = depth_noise_std_m + depth_noise_z2_coeff * z**2   [metres]
  which mimics the way stereo-depth error grows with range on a D435. Invalid
  pixels (0) stay 0 so holes remain holes. Both 16UC1 (millimetres) and 32FC1
  (metres) depth encodings are handled, and the output keeps the input encoding.
- RGB: passed through unchanged by default (baseline). An optional Gaussian in
  8-bit intensity units (``rgb_noise_std``, default 0 = off) is wired in for
  when we want to noise colour too, so this node is the single place noise lives.

The original header (stamp + frame_id) is preserved on every message so the
bridge's RGB/depth time-sync check keeps working.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from admittance_control.sam6d_io import image_msg_to_numpy


def make_image_msg(data: np.ndarray, encoding: str, header) -> Image:
    """Build a sensor_msgs/Image from a contiguous numpy array, reusing a header."""
    array = np.ascontiguousarray(data)
    msg = Image()
    msg.header = header
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(array.strides[0])
    msg.data = array.tobytes()
    return msg


def make_camera_info(width: int, height: int, fx: float, fy: float,
                     cx: float, cy: float, header) -> CameraInfo:
    """Build a pinhole (no-distortion) CameraInfo, reusing an image header.

    Isaac's ROS2CameraHelper graph in this scene has no CameraInfo publisher, so
    depth_image_proc's PointCloudXyzrgb node has no intrinsics and emits nothing.
    We synthesize the intrinsics here from the simulated lens (focal length +
    aperture) so the cloud node and the SAM-6D bridge both get a valid cam_K.
    """
    info = CameraInfo()
    info.header = header
    info.width = int(width)
    info.height = int(height)
    info.distortion_model = 'plumb_bob'
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
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


class RealSenseSimCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('realsense_sim_camera')

        self.declare_parameter('color_in_topic', '/camera/color/image_raw_sim')
        self.declare_parameter('depth_in_topic', '/camera/depth/image_rect_raw_sim')
        self.declare_parameter('color_out_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_out_topic', '/camera/depth/image_rect_raw')
        # Depth Gaussian noise: sigma(z) = std_m + z2_coeff * z^2  (metres).
        self.declare_parameter('depth_noise_std_m', 0.02)
        self.declare_parameter('depth_noise_z2_coeff', 0.005)
        # RGB Gaussian noise in 8-bit intensity units; 0 = passthrough baseline.
        self.declare_parameter('rgb_noise_std', 0.0)
        # Seed for reproducibility; <0 uses a nondeterministic generator.
        self.declare_parameter('seed', -1)
        # Isaac stamps its camera images with an empty/scene frame_id that is not
        # in the robot TF tree. Rewrite it here so the pointcloud lands in a frame
        # the static extrinsic (tool0 -> camera_color_optical_frame) connects to.
        # Empty string ('') keeps Isaac's original frame_id (passthrough).
        self.declare_parameter('output_frame_id', '')
        # Isaac's camera graph in welding_world.usda has no CameraInfo publisher,
        # which PointCloudXyzrgb needs. Synthesize one from the simulated lens.
        self.declare_parameter('publish_camera_info', False)
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        # Pinhole intrinsics for the synthesized CameraInfo. The digital twin uses
        # the *same* intrinsics as the real RealSense D435 (see camera.json /
        # notebooks/realsense_intrinsics.npy), so the defaults are the real
        # camera's cam_K at 1280x720 — note the off-centre principal point.
        self.declare_parameter('fx', 919.46923828125)
        self.declare_parameter('fy', 918.9381103515625)
        self.declare_parameter('cx', 650.8524169921875)
        self.declare_parameter('cy', 350.6199951171875)

        self._depth_std_m = float(self.get_parameter('depth_noise_std_m').value)
        self._depth_z2 = float(self.get_parameter('depth_noise_z2_coeff').value)
        self._rgb_std = float(self.get_parameter('rgb_noise_std').value)
        self._out_frame = str(self.get_parameter('output_frame_id').value)
        self._publish_info = bool(self.get_parameter('publish_camera_info').value)
        self._fx = float(self.get_parameter('fx').value)
        self._fy = float(self.get_parameter('fy').value)
        self._cx = float(self.get_parameter('cx').value)
        self._cy = float(self.get_parameter('cy').value)

        seed = int(self.get_parameter('seed').value)
        self._rng = np.random.default_rng(seed if seed >= 0 else None)

        color_in = str(self.get_parameter('color_in_topic').value)
        depth_in = str(self.get_parameter('depth_in_topic').value)
        color_out = str(self.get_parameter('color_out_topic').value)
        depth_out = str(self.get_parameter('depth_out_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self._color_pub = self.create_publisher(Image, color_out, 10)
        self._depth_pub = self.create_publisher(Image, depth_out, 10)
        self._info_pub = (
            self.create_publisher(CameraInfo, camera_info_topic, 10)
            if self._publish_info else None)
        self.create_subscription(Image, color_in, self._on_color, 10)
        self.create_subscription(Image, depth_in, self._on_depth, 10)

        self.get_logger().info(
            f'sim camera noise relay: {color_in} -> {color_out}, '
            f'{depth_in} -> {depth_out}\n'
            f'  depth sigma(z) = {self._depth_std_m:.4f} + '
            f'{self._depth_z2:.4f}*z^2 m; rgb_noise_std={self._rgb_std:.1f}\n'
            f"  output_frame_id={self._out_frame or '<passthrough>'}; "
            f'publish_camera_info={self._publish_info}'
            + (f' -> {camera_info_topic}' if self._publish_info else ''))

    def _stamp_frame(self, header):
        """Override the frame_id in-place if an output frame was configured."""
        if self._out_frame:
            header.frame_id = self._out_frame
        return header

    def _publish_camera_info(self, width: int, height: int, header) -> None:
        if self._info_pub is None:
            return
        self._info_pub.publish(
            make_camera_info(width, height, self._fx, self._fy,
                             self._cx, self._cy, header))

    # ── Color ────────────────────────────────────────────────────────────
    def _on_color(self, msg: Image) -> None:
        if self._rgb_std <= 0.0:
            # Baseline: republish untouched (single hop, no re-encode).
            out = msg
        else:
            try:
                image = image_msg_to_numpy(msg).astype(np.float32)
                noisy = image + self._rng.standard_normal(
                    image.shape).astype(np.float32) * self._rgb_std
                noisy = np.clip(noisy, 0.0, 255.0).astype(np.uint8)
                out = make_image_msg(noisy, msg.encoding, msg.header)
            except ValueError as exc:
                self.get_logger().warn(f'color passthrough (unsupported: {exc})',
                                       throttle_duration_sec=5.0)
                out = msg
        self._stamp_frame(out.header)
        self._color_pub.publish(out)
        # CameraInfo must share the color image's stamp + frame so the pointcloud
        # node's message_filter sync (rgb / depth / info) lines up.
        self._publish_camera_info(out.width, out.height, out.header)

    # ── Depth ────────────────────────────────────────────────────────────
    def _on_depth(self, msg: Image) -> None:
        encoding = msg.encoding.lower()
        try:
            depth = image_msg_to_numpy(msg)
        except ValueError as exc:
            self.get_logger().warn(f'depth passthrough (unsupported: {exc})',
                                   throttle_duration_sec=5.0)
            self._stamp_frame(msg.header)
            self._depth_pub.publish(msg)
            return

        if encoding in ('16uc1', 'mono16'):
            noisy = self._noise_depth_mm(depth)
            out_encoding = '16UC1'
        elif encoding == '32fc1':
            noisy = self._noise_depth_m(depth)
            out_encoding = '32FC1'
        else:
            self.get_logger().warn(
                f'depth passthrough (unhandled encoding {msg.encoding})',
                throttle_duration_sec=5.0)
            self._stamp_frame(msg.header)
            self._depth_pub.publish(msg)
            return

        out = make_image_msg(noisy, out_encoding, msg.header)
        self._stamp_frame(out.header)
        self._depth_pub.publish(out)

    def _noise_depth_mm(self, depth: np.ndarray) -> np.ndarray:
        d = depth.astype(np.float32)
        valid = d > 0.0
        z_m = d / 1000.0
        sigma_mm = (self._depth_std_m + self._depth_z2 * z_m * z_m) * 1000.0
        noise = self._rng.standard_normal(d.shape).astype(np.float32) * sigma_mm
        d = d + np.where(valid, noise, 0.0)
        d = np.clip(np.rint(d), 0.0, float(np.iinfo(np.uint16).max))
        out = d.astype(np.uint16)
        out[~valid] = 0
        return out

    def _noise_depth_m(self, depth: np.ndarray) -> np.ndarray:
        d = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        valid = d > 0.0
        sigma_m = self._depth_std_m + self._depth_z2 * d * d
        noise = self._rng.standard_normal(d.shape).astype(np.float32) * sigma_m
        d = d + np.where(valid, noise, 0.0)
        d = np.clip(d, 0.0, None)
        d[~valid] = 0.0
        return d


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = RealSenseSimCameraNode()
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
