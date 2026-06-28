#!/usr/bin/env python3
"""Capture one RGB-D frame from ROS 2 topics and send it to SAM-6D."""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from urllib import error as urllib_error
from urllib import request as urllib_request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image


DEFAULT_SERVER_URL = 'http://127.0.0.1:5000/predict_pose'
DEFAULT_RGB_TOPIC = '/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/depth/image_rect_raw'
DEFAULT_TIMEOUT_SEC = 15.0
DEFAULT_REQUEST_TIMEOUT_SEC = 30.0
DEFAULT_MAX_SYNC_DELTA_SEC = 0.25
PACKAGE_NAME = 'admittance_control'


def parse_args(argv: list[str]) -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description='Capture one RGB-D pair from ROS 2 topics and send it to SAM-6D.'
	)
	parser.add_argument('--server-url', default=DEFAULT_SERVER_URL)
	parser.add_argument('--rgb-topic', default=DEFAULT_RGB_TOPIC)
	parser.add_argument('--depth-topic', default=DEFAULT_DEPTH_TOPIC)
	parser.add_argument('--timeout-sec', type=float, default=DEFAULT_TIMEOUT_SEC)
	parser.add_argument('--request-timeout-sec', type=float, default=DEFAULT_REQUEST_TIMEOUT_SEC)
	parser.add_argument('--max-sync-delta-sec', type=float, default=DEFAULT_MAX_SYNC_DELTA_SEC)
	cli_args = remove_ros_args(args=argv)[1:]
	return parser.parse_args(cli_args)


def resolve_transfer_dir() -> Path:
	script_dir = Path(__file__).resolve().parent
	direct_candidates = [script_dir / 'rgb_depth_to_send']
	source_candidates: list[Path] = []
	share_candidates: list[Path] = []

	for parent in script_dir.parents:
		source_candidates.append(parent / 'src' / PACKAGE_NAME / 'scripts' / 'rgb_depth_to_send')
		share_candidates.append(parent / 'share' / PACKAGE_NAME / 'scripts' / 'rgb_depth_to_send')

	candidates = direct_candidates + source_candidates + share_candidates

	seen: set[Path] = set()
	for candidate in candidates:
		if candidate in seen:
			continue
		seen.add(candidate)
		if (candidate / 'camera.json').exists():
			return candidate

	output_dir = candidates[0]
	output_dir.mkdir(parents=True, exist_ok=True)
	return output_dir


def load_camera_payload(camera_path: Path) -> Dict[str, Any]:
	if not camera_path.exists():
		raise FileNotFoundError(
			f'camera.json was not found at {camera_path}. '
			'Place it under scripts/rgb_depth_to_send before running the client.'
		)

	with camera_path.open('r', encoding='utf-8') as camera_file:
		payload = json.load(camera_file)

	if 'cam_K' not in payload:
		raise ValueError(f'{camera_path} does not contain a cam_K entry.')

	return payload


def image_msg_to_numpy(msg: Image) -> np.ndarray:
	encoding = msg.encoding.lower()
	if msg.is_bigendian:
		raise ValueError('Big-endian image messages are not supported by this client.')

	encoding_map: Dict[str, Tuple[np.dtype[Any], int]] = {
		'rgb8': (np.dtype(np.uint8), 3),
		'bgr8': (np.dtype(np.uint8), 3),
		'rgba8': (np.dtype(np.uint8), 4),
		'bgra8': (np.dtype(np.uint8), 4),
		'mono8': (np.dtype(np.uint8), 1),
		'mono16': (np.dtype(np.uint16), 1),
		'8uc1': (np.dtype(np.uint8), 1),
		'16uc1': (np.dtype(np.uint16), 1),
		'32fc1': (np.dtype(np.float32), 1),
		'32sc1': (np.dtype(np.int32), 1),
	}

	if encoding not in encoding_map:
		raise ValueError(f'Unsupported image encoding: {msg.encoding}')

	dtype, channels = encoding_map[encoding]
	row_elems = msg.step // dtype.itemsize
	flat = np.frombuffer(msg.data, dtype=dtype)
	expected_elems = msg.height * row_elems

	if flat.size != expected_elems:
		raise ValueError(
			f'Image buffer size mismatch for {msg.encoding}: '
			f'expected {expected_elems} values, got {flat.size}.'
		)

	if channels == 1:
		return flat.reshape(msg.height, row_elems)[:, :msg.width].copy()

	compact = flat.reshape(msg.height, row_elems)[:, :msg.width * channels]
	return compact.reshape(msg.height, msg.width, channels).copy()


def normalize_color_image(color_image: np.ndarray, encoding: str) -> np.ndarray:
	normalized = encoding.lower()

	if normalized == 'bgr8':
		return color_image
	if normalized == 'rgb8':
		return cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
	if normalized == 'rgba8':
		return cv2.cvtColor(color_image, cv2.COLOR_RGBA2BGR)
	if normalized == 'bgra8':
		return cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
	if normalized == 'mono8':
		return cv2.cvtColor(color_image, cv2.COLOR_GRAY2BGR)

	raise ValueError(f'Unsupported RGB image encoding: {encoding}')


def normalize_depth_image(depth_image: np.ndarray, encoding: str, camera_payload: Dict[str, Any]) -> Tuple[np.ndarray, Dict[str, Any]]:
	normalized = encoding.lower()
	resolved_camera_payload = dict(camera_payload)

	if normalized in ('16uc1', 'mono16'):
		return depth_image.astype(np.uint16, copy=False), resolved_camera_payload

	if normalized == '32fc1':
		depth_m = np.nan_to_num(depth_image.astype(np.float32, copy=False), nan=0.0, posinf=0.0, neginf=0.0)
		depth_mm = np.clip(np.rint(depth_m * 1000.0), 0.0, float(np.iinfo(np.uint16).max))
		resolved_camera_payload['depth_scale'] = 1.0
		return depth_mm.astype(np.uint16), resolved_camera_payload

	raise ValueError(f'Unsupported depth image encoding: {encoding}')


def encode_png(image: np.ndarray, label: str) -> bytes:
	success, encoded = cv2.imencode('.png', image)
	if not success:
		raise RuntimeError(f'Failed to encode {label} as PNG.')
	return encoded.tobytes()


def encode_multipart_formdata(files: Dict[str, Tuple[str, bytes, str]]) -> Tuple[bytes, str]:
	boundary = f'----Sam6DRosClient{uuid.uuid4().hex}'
	body_parts: list[bytes] = []

	for field_name, (filename, payload, content_type) in files.items():
		body_parts.extend([
			f'--{boundary}\r\n'.encode('utf-8'),
			(
				f'Content-Disposition: form-data; name="{field_name}"; '
				f'filename="{filename}"\r\n'
			).encode('utf-8'),
			f'Content-Type: {content_type}\r\n\r\n'.encode('utf-8'),
			payload,
			b'\r\n',
		])

	body_parts.append(f'--{boundary}--\r\n'.encode('utf-8'))
	return b''.join(body_parts), boundary


def post_files(url: str, files: Dict[str, Tuple[str, bytes, str]], timeout_sec: float) -> Tuple[int, bytes]:
	body, boundary = encode_multipart_formdata(files)
	request = urllib_request.Request(url, data=body, method='POST')
	request.add_header('Content-Type', f'multipart/form-data; boundary={boundary}')
	request.add_header('Content-Length', str(len(body)))

	try:
		with urllib_request.urlopen(request, timeout=timeout_sec) as response:
			return int(response.status), response.read()
	except urllib_error.HTTPError as exc:
		return int(exc.code), exc.read()


def stamp_to_nanoseconds(msg: Image) -> int:
	return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class Sam6DRosClient(Node):
	def __init__(self, args: argparse.Namespace, transfer_dir: Path, camera_path: Path) -> None:
		super().__init__('sam6d_ros_client')
		self._server_url = args.server_url
		self._request_timeout_sec = args.request_timeout_sec
		self._max_sync_delta_sec = args.max_sync_delta_sec
		self._transfer_dir = transfer_dir
		self._camera_path = camera_path

		self._latest_rgb_msg: Optional[Image] = None
		self._latest_depth_msg: Optional[Image] = None
		self.finished = False
		self.error: Optional[Exception] = None

		self._rgb_sub = self.create_subscription(Image, args.rgb_topic, self._on_rgb, 10)
		self._depth_sub = self.create_subscription(Image, args.depth_topic, self._on_depth, 10)

		self.get_logger().info(f'Waiting for RGB on {args.rgb_topic} and depth on {args.depth_topic}')
		self.get_logger().info(f'Using transfer directory {self._transfer_dir}')

	def _on_rgb(self, msg: Image) -> None:
		if self.finished:
			return

		self._latest_rgb_msg = msg
		self.get_logger().info(
			f'Received RGB frame: {msg.width}x{msg.height} {msg.encoding}',
			once=True,
		)
		self._maybe_process_pair()

	def _on_depth(self, msg: Image) -> None:
		if self.finished:
			return

		self._latest_depth_msg = msg
		self.get_logger().info(
			f'Received depth frame: {msg.width}x{msg.height} {msg.encoding}',
			once=True,
		)
		self._maybe_process_pair()

	def _maybe_process_pair(self) -> None:
		if self.finished or self._latest_rgb_msg is None or self._latest_depth_msg is None:
			return

		stamp_delta_sec = abs(
			stamp_to_nanoseconds(self._latest_rgb_msg) - stamp_to_nanoseconds(self._latest_depth_msg)
		) / 1_000_000_000.0

		if stamp_delta_sec > self._max_sync_delta_sec:
			self.get_logger().warning(
				f'RGB/depth stamp delta is {stamp_delta_sec:.3f}s; waiting for a closer pair.'
			)
			return

		try:
			self._save_and_send(self._latest_rgb_msg, self._latest_depth_msg)
			self.finished = True
		except Exception as exc:
			self.error = exc
			self.finished = True

	def _save_and_send(self, rgb_msg: Image, depth_msg: Image) -> None:
		camera_payload = load_camera_payload(self._camera_path)

		rgb_image = normalize_color_image(image_msg_to_numpy(rgb_msg), rgb_msg.encoding)
		depth_image, resolved_camera_payload = normalize_depth_image(
			image_msg_to_numpy(depth_msg),
			depth_msg.encoding,
			camera_payload,
		)

		self._transfer_dir.mkdir(parents=True, exist_ok=True)
		rgb_path = self._transfer_dir / 'rgb.png'
		depth_path = self._transfer_dir / 'depth.png'

		rgb_png = encode_png(rgb_image, 'rgb image')
		depth_png = encode_png(depth_image, 'depth image')

		rgb_path.write_bytes(rgb_png)
		depth_path.write_bytes(depth_png)

		camera_json = json.dumps(resolved_camera_payload, separators=(',', ':'))
		self._camera_path.write_text(camera_json, encoding='utf-8')

		files = {
			'rgb': ('rgb.png', rgb_png, 'image/png'),
			'depth': ('depth.png', depth_png, 'image/png'),
			'camera': ('camera.json', camera_json.encode('utf-8'), 'application/json'),
		}

		self.get_logger().info('Saved rgb.png, depth.png, and camera.json. Sending request to SAM-6D server...')
		status_code, response_body = post_files(
			self._server_url,
			files,
			self._request_timeout_sec,
		)

		if status_code != 200:
			raise RuntimeError(
				f'Server returned {status_code}: {response_body.decode("utf-8", errors="replace")}'
			)

		try:
			payload = json.loads(response_body.decode('utf-8'))
			pretty_payload = json.dumps(payload, indent=4)
		except ValueError:
			pretty_payload = response_body.decode('utf-8', errors='replace')

		self.get_logger().info('SAM-6D response:')
		print(pretty_payload)


def main(argv: Optional[list[str]] = None) -> int:
	argv = list(sys.argv if argv is None else argv)
	args = parse_args(argv)

	transfer_dir = resolve_transfer_dir()
	camera_path = transfer_dir / 'camera.json'

	rclpy.init(args=argv)
	node = Sam6DRosClient(args, transfer_dir, camera_path)
	deadline = time.monotonic() + args.timeout_sec

	try:
		while rclpy.ok() and not node.finished:
			rclpy.spin_once(node, timeout_sec=0.1)
			if time.monotonic() >= deadline:
				raise TimeoutError(
					f'Timed out after {args.timeout_sec:.1f}s while waiting for synchronized RGB-D frames.'
				)

		if node.error is not None:
			raise node.error
	except KeyboardInterrupt:
		node.get_logger().warning('Interrupted before the request completed.')
		return 130
	except Exception as exc:
		node.get_logger().error(str(exc))
		return 1
	finally:
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

	return 0


if __name__ == '__main__':
	sys.exit(main())
