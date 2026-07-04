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
  <color_out_topic> sensor_msgs/Image   (default /camera/color/image_raw)
  <depth_out_topic> sensor_msgs/Image   (default /camera/depth/image_rect_raw)

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
from sensor_msgs.msg import Image

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


class RealSenseSimCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('realsense_sim_camera')

        self.declare_parameter('color_in_topic', '/camera/color/image_raw_sim')
        self.declare_parameter('depth_in_topic', '/camera/depth/image_rect_raw_sim')
        self.declare_parameter('color_out_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_out_topic', '/camera/depth/image_rect_raw')
        # Depth Gaussian noise: sigma(z) = std_m + z2_coeff * z^2  (metres).
        self.declare_parameter('depth_noise_std_m', 0.05)
        self.declare_parameter('depth_noise_z2_coeff', 0.01)
        # RGB Gaussian noise in 8-bit intensity units; 0 = passthrough baseline.
        self.declare_parameter('rgb_noise_std', 0.0)
        # Seed for reproducibility; <0 uses a nondeterministic generator.
        self.declare_parameter('seed', -1)

        self._depth_std_m = float(self.get_parameter('depth_noise_std_m').value)
        self._depth_z2 = float(self.get_parameter('depth_noise_z2_coeff').value)
        self._rgb_std = float(self.get_parameter('rgb_noise_std').value)

        seed = int(self.get_parameter('seed').value)
        self._rng = np.random.default_rng(seed if seed >= 0 else None)

        color_in = str(self.get_parameter('color_in_topic').value)
        depth_in = str(self.get_parameter('depth_in_topic').value)
        color_out = str(self.get_parameter('color_out_topic').value)
        depth_out = str(self.get_parameter('depth_out_topic').value)

        self._color_pub = self.create_publisher(Image, color_out, 10)
        self._depth_pub = self.create_publisher(Image, depth_out, 10)
        self.create_subscription(Image, color_in, self._on_color, 10)
        self.create_subscription(Image, depth_in, self._on_depth, 10)

        self.get_logger().info(
            f'sim camera noise relay: {color_in} -> {color_out}, '
            f'{depth_in} -> {depth_out}\n'
            f'  depth sigma(z) = {self._depth_std_m:.4f} + '
            f'{self._depth_z2:.4f}*z^2 m; rgb_noise_std={self._rgb_std:.1f}')

    # ── Color ────────────────────────────────────────────────────────────
    def _on_color(self, msg: Image) -> None:
        if self._rgb_std <= 0.0:
            # Baseline: republish untouched (single hop, no re-encode).
            self._color_pub.publish(msg)
            return
        try:
            image = image_msg_to_numpy(msg).astype(np.float32)
        except ValueError as exc:
            self.get_logger().warn(f'color passthrough (unsupported: {exc})',
                                   throttle_duration_sec=5.0)
            self._color_pub.publish(msg)
            return
        noisy = image + self._rng.standard_normal(image.shape).astype(np.float32) * self._rgb_std
        noisy = np.clip(noisy, 0.0, 255.0).astype(np.uint8)
        self._color_pub.publish(make_image_msg(noisy, msg.encoding, msg.header))

    # ── Depth ────────────────────────────────────────────────────────────
    def _on_depth(self, msg: Image) -> None:
        encoding = msg.encoding.lower()
        try:
            depth = image_msg_to_numpy(msg)
        except ValueError as exc:
            self.get_logger().warn(f'depth passthrough (unsupported: {exc})',
                                   throttle_duration_sec=5.0)
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
            self._depth_pub.publish(msg)
            return

        self._depth_pub.publish(make_image_msg(noisy, out_encoding, msg.header))

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
