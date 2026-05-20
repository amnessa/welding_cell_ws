#!/usr/bin/env python3
"""Convert a saved SAM3 segmentation result into safe 3D probe targets.

This node consumes the latest artifact bundle produced by segment_area.py,
projects the segmented mask into robot-base coordinates using the saved camera
calibration, generates contour-safe probe targets, saves the resulting overlay
and JSON summary, and publishes the 3D targets for the drawing GUI.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def _resolve_source_package_root() -> Path:
	script_package_root = Path(__file__).resolve().parents[1]
	candidates = [script_package_root]

	try:
		pkg_share = Path(get_package_share_directory('sand_drawer')).resolve()
		candidates.append(pkg_share)
		marker = '/install/sand_drawer/share/sand_drawer'
		pkg_share_str = pkg_share.as_posix()
		if marker in pkg_share_str:
			workspace_root = Path(pkg_share_str.split(marker)[0])
			candidates.extend([
				workspace_root / 'src' / 'admittance_control',
				workspace_root / 'src' / 'sand_drawer',
			])
	except Exception:
		pass

	for candidate in candidates:
		if (candidate / 'scripts').exists() and (candidate / 'notebooks').exists():
			return candidate

	return script_package_root


SOURCE_PACKAGE_ROOT = _resolve_source_package_root()
NOTEBOOK_DIR = SOURCE_PACKAGE_ROOT / 'notebooks'
DEFAULT_SEGMENT_ARTIFACT_DIR = SOURCE_PACKAGE_ROOT / 'artifacts' / 'segment_area'
DEFAULT_INTRINSICS_PATH = NOTEBOOK_DIR / 'realsense_intrinsics.npy'
DEFAULT_EXTRINSICS_PATH = NOTEBOOK_DIR / 'T_base_to_cam.npy'


def load_calibration_matrices(
	intrinsics_path: Path,
	extrinsics_path: Path,
) -> Tuple[np.ndarray, np.ndarray]:
	intrinsics_path = Path(intrinsics_path).expanduser().resolve()
	extrinsics_path = Path(extrinsics_path).expanduser().resolve()

	if not intrinsics_path.exists():
		raise FileNotFoundError(f'Intrinsics file does not exist: {intrinsics_path}')
	if not extrinsics_path.exists():
		raise FileNotFoundError(f'Extrinsics file does not exist: {extrinsics_path}')

	K_matrix = np.asarray(np.load(intrinsics_path), dtype=float)
	T_base_to_cam = np.asarray(np.load(extrinsics_path), dtype=float)

	if K_matrix.shape != (3, 3):
		raise ValueError(
			f'Expected a 3x3 intrinsics matrix, found shape {K_matrix.shape}')
	if T_base_to_cam.shape != (4, 4):
		raise ValueError(
			f'Expected a 4x4 extrinsics matrix, found shape {T_base_to_cam.shape}')

	return K_matrix, T_base_to_cam


def resolve_segment_run_dir(
	segment_artifact_dir: Path,
	segment_run_dir: str,
) -> Path:
	if segment_run_dir.strip():
		run_dir = Path(segment_run_dir).expanduser().resolve()
		if not run_dir.exists():
			raise FileNotFoundError(f'Segment run directory does not exist: {run_dir}')
		return run_dir

	artifact_dir = Path(segment_artifact_dir).expanduser().resolve()
	if not artifact_dir.exists():
		raise FileNotFoundError(
			f'Segment artifact directory does not exist: {artifact_dir}')

	candidates = sorted(
		[
			path for path in artifact_dir.iterdir()
			if path.is_dir() and (path / 'metadata.json').exists()
		],
		key=lambda path: path.stat().st_mtime,
	)
	if not candidates:
		raise FileNotFoundError(
			f'No segmentation runs with metadata.json were found in {artifact_dir}')

	return candidates[-1]


def load_segment_artifacts(run_dir: Path):
	metadata_path = run_dir / 'metadata.json'
	if not metadata_path.exists():
		raise FileNotFoundError(f'Segment metadata is missing: {metadata_path}')

	with open(metadata_path, 'r', encoding='utf-8') as input_file:
		metadata = json.load(input_file)

	artifact_paths = metadata.get('artifact_paths', {})
	color_path = Path(artifact_paths.get('color_bgr_png', run_dir / 'color_bgr.png'))
	mask_path = Path(artifact_paths.get('mask_png', run_dir / 'mask.png'))
	depth_path = Path(artifact_paths.get('depth_m_npy', run_dir / 'depth_m.npy'))

	color_image = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
	if color_image is None:
		raise FileNotFoundError(f'Could not read color image: {color_path}')

	mask_image = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
	if mask_image is None:
		raise FileNotFoundError(f'Could not read mask image: {mask_path}')

	if not depth_path.exists():
		raise FileNotFoundError(f'Could not read depth file: {depth_path}')
	depth_image_m = np.asarray(np.load(depth_path), dtype=float)

	return color_image, mask_image.astype(bool), depth_image_m, metadata


def get_robot_target_from_mask(
	sam_mask: np.ndarray,
	depth_image_m: np.ndarray,
	K_matrix: np.ndarray,
	T_base_to_cam: np.ndarray,
	window_radius_px: int,
):
	y_indices, x_indices = np.where(sam_mask > 0)
	if len(x_indices) == 0:
		raise ValueError('Selected mask is empty.')

	u_center = int(np.mean(x_indices))
	v_center = int(np.mean(y_indices))

	depth_window = depth_image_m[
		max(0, v_center - window_radius_px):min(
			depth_image_m.shape[0], v_center + window_radius_px),
		max(0, u_center - window_radius_px):min(
			depth_image_m.shape[1], u_center + window_radius_px),
	]
	valid_depths = depth_window[np.isfinite(depth_window) & (depth_window > 0.0)]
	if valid_depths.size == 0:
		raise ValueError(
			'No valid depth values were found around the mask centroid.')

	depth_z_meters = float(np.median(valid_depths))
	fx, fy = float(K_matrix[0, 0]), float(K_matrix[1, 1])
	cx, cy = float(K_matrix[0, 2]), float(K_matrix[1, 2])

	x_cam = (u_center - cx) * depth_z_meters / fx
	y_cam = (v_center - cy) * depth_z_meters / fy
	p_cam = np.array([x_cam, y_cam, depth_z_meters, 1.0], dtype=float)
	p_base = T_base_to_cam @ p_cam

	debug_info = {
		'centroid_uv': [u_center, v_center],
		'depth_m': depth_z_meters,
		'valid_depth_count': int(valid_depths.size),
	}
	return float(p_base[0]), float(p_base[1]), float(p_base[2]), debug_info


def extract_primary_mask_contour(sam_mask: np.ndarray):
	mask_uint8 = sam_mask.astype(np.uint8) * 255
	contours, _ = cv2.findContours(
		mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if not contours:
		raise ValueError('Could not find any contour in the selected mask.')

	plate_contour = max(contours, key=cv2.contourArea)
	if cv2.contourArea(plate_contour) <= 0:
		raise ValueError('The selected contour has zero area.')
	return plate_contour


def generate_safe_probe_targets(
	sam_mask: np.ndarray,
	depth_image_m: np.ndarray,
	K_matrix: np.ndarray,
	T_base_to_cam: np.ndarray,
	num_points: int,
	edge_margin_px: int,
	depth_window_radius_px: int,
	max_attempts: int,
	rng_seed: int,
):
	mask_bool = np.asarray(sam_mask, dtype=bool)
	plate_contour = extract_primary_mask_contour(mask_bool)
	x, y, width, height = cv2.boundingRect(plate_contour)

	rng = np.random.default_rng(rng_seed)
	visited_pixels = set()
	probe_targets = []
	attempt_count = 0

	while len(probe_targets) < num_points and attempt_count < max_attempts:
		attempt_count += 1
		u = int(rng.integers(x, x + width))
		v = int(rng.integers(y, y + height))
		if (u, v) in visited_pixels:
			continue
		visited_pixels.add((u, v))

		if not mask_bool[v, u]:
			continue

		dist_to_edge = float(
			cv2.pointPolygonTest(plate_contour, (float(u), float(v)), True))
		if dist_to_edge < edge_margin_px:
			continue

		row_start = max(0, v - depth_window_radius_px)
		row_end = min(depth_image_m.shape[0], v + depth_window_radius_px + 1)
		col_start = max(0, u - depth_window_radius_px)
		col_end = min(depth_image_m.shape[1], u + depth_window_radius_px + 1)
		depth_window = depth_image_m[row_start:row_end, col_start:col_end]
		valid_depths = depth_window[np.isfinite(depth_window) & (depth_window > 0.0)]
		if valid_depths.size == 0:
			continue

		depth_z_meters = float(np.median(valid_depths))
		fx, fy = float(K_matrix[0, 0]), float(K_matrix[1, 1])
		cx, cy = float(K_matrix[0, 2]), float(K_matrix[1, 2])
		x_cam = (u - cx) * depth_z_meters / fx
		y_cam = (v - cy) * depth_z_meters / fy
		p_cam = np.array([x_cam, y_cam, depth_z_meters, 1.0], dtype=float)
		p_base = T_base_to_cam @ p_cam

		probe_targets.append(
			{
				'index': len(probe_targets) + 1,
				'pixel_uv': [u, v],
				'distance_to_edge_px': dist_to_edge,
				'depth_m': depth_z_meters,
				'camera_point': p_cam[:3].tolist(),
				'base_point': p_base[:3].tolist(),
			}
		)

	if len(probe_targets) < 3:
		raise RuntimeError(
			f'Only {len(probe_targets)} safe probe targets were found after '
			f'{attempt_count} attempts. Increase the segmented area or reduce '
			'edge_margin_px.')

	sampling_debug = {
		'attempt_count': attempt_count,
		'bounding_rect': [x, y, width, height],
		'edge_margin_px': edge_margin_px,
	}
	return probe_targets, plate_contour, sampling_debug


def build_probe_targets_overlay(
	image_bgr: np.ndarray,
	mask: np.ndarray,
	probe_targets: List[dict],
	contour,
	centroid_uv: Tuple[int, int],
) -> np.ndarray:
	overlay = image_bgr.copy()
	mask_bool = np.asarray(mask, dtype=bool)
	highlight_color = np.array([0, 255, 255], dtype=np.uint8)
	overlay[mask_bool] = (
		0.4 * overlay[mask_bool] + 0.6 * highlight_color).astype(np.uint8)

	if contour is not None:
		cv2.drawContours(overlay, [contour], -1, (255, 0, 255), 2)

	cv2.circle(overlay, tuple(centroid_uv), 8, (0, 0, 255), -1)

	for target in probe_targets:
		u, v = [int(value) for value in target['pixel_uv']]
		cv2.circle(overlay, (u, v), 7, (0, 255, 0), -1)
		cv2.putText(
			overlay,
			str(target['index']),
			(u + 8, v - 8),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.55,
			(255, 255, 255),
			2,
			cv2.LINE_AA,
		)

	return overlay


def save_probe_artifacts(run_dir: Path, overlay: np.ndarray, metadata: dict) -> dict:
	overlay_path = run_dir / 'probe_targets_overlay_bgr.png'
	metadata_path = run_dir / 'safe_probe_targets.json'

	cv2.imwrite(str(overlay_path), overlay)

	metadata = dict(metadata)
	metadata['artifact_paths'] = {
		'probe_overlay_bgr_png': str(overlay_path),
		'safe_probe_targets_json': str(metadata_path),
	}

	with open(metadata_path, 'w', encoding='utf-8') as output_file:
		json.dump(metadata, output_file, indent=2)

	return metadata


def build_pose_array(probe_targets: List[dict], frame_id: str, stamp) -> PoseArray:
	pose_array = PoseArray()
	pose_array.header.stamp = stamp
	pose_array.header.frame_id = frame_id
	poses = []

	for target in probe_targets:
		pose = Pose()
		base_x, base_y, base_z = [float(value) for value in target['base_point']]
		pose.position.x = base_x
		pose.position.y = base_y
		pose.position.z = base_z
		pose.orientation.w = 1.0
		poses.append(pose)

	pose_array.poses = poses

	return pose_array


class DepthToSafeProbeNode(Node):
	def __init__(self):
		super().__init__('depth_to_safe_probe')

		self.declare_parameter('segment_artifact_dir', str(DEFAULT_SEGMENT_ARTIFACT_DIR))
		self.declare_parameter('segment_run_dir', '')
		self.declare_parameter('intrinsics_path', str(DEFAULT_INTRINSICS_PATH))
		self.declare_parameter('extrinsics_path', str(DEFAULT_EXTRINSICS_PATH))
		self.declare_parameter('probe_point_count', 5)
		self.declare_parameter('edge_margin_px', 15)
		self.declare_parameter('probe_depth_window_radius_px', 3)
		self.declare_parameter('max_point_sampling_attempts', 2000)
		self.declare_parameter('probe_random_seed', 7)
		self.declare_parameter('window_radius_px', 5)
		self.declare_parameter('base_frame_id', 'base_link')
		self.declare_parameter('save_artifacts', True)
		self.declare_parameter('keep_alive', True)

		qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
		self._probe_targets_pub = self.create_publisher(
			PoseArray, '/visualizer/safe_probe_targets', qos)
		self._probe_metadata_pub = self.create_publisher(
			String, '/visualizer/safe_probe_metadata', qos)
		self._private_probe_targets_pub = self.create_publisher(
			PoseArray, '~/probe_targets', qos)
		self._private_metadata_pub = self.create_publisher(
			String, '~/metadata', qos)
		self._keep_alive = True

	def _parameter_value(self, name: str) -> Any:
		value = self.get_parameter(name).value
		if value is None:
			raise RuntimeError(f'Required parameter {name!r} is unset.')
		return value

	def run_once(self):
		segment_artifact_dir = Path(str(self._parameter_value('segment_artifact_dir')))
		segment_run_dir = str(self._parameter_value('segment_run_dir'))
		intrinsics_path = Path(str(self._parameter_value('intrinsics_path')))
		extrinsics_path = Path(str(self._parameter_value('extrinsics_path')))
		probe_point_count = int(self._parameter_value('probe_point_count'))
		edge_margin_px = int(self._parameter_value('edge_margin_px'))
		probe_depth_window_radius_px = int(
			self._parameter_value('probe_depth_window_radius_px'))
		max_point_sampling_attempts = int(
			self._parameter_value('max_point_sampling_attempts'))
		probe_random_seed = int(self._parameter_value('probe_random_seed'))
		window_radius_px = int(self._parameter_value('window_radius_px'))
		base_frame_id = str(self._parameter_value('base_frame_id'))
		save_artifacts = bool(self._parameter_value('save_artifacts'))
		self._keep_alive = bool(self._parameter_value('keep_alive'))

		run_dir = resolve_segment_run_dir(segment_artifact_dir, segment_run_dir)
		self.get_logger().info(f'Using segmentation run: {run_dir}')

		K_matrix, T_base_to_cam = load_calibration_matrices(
			intrinsics_path=intrinsics_path,
			extrinsics_path=extrinsics_path,
		)
		color_image, target_mask, depth_image_m, segment_metadata = load_segment_artifacts(run_dir)

		target_x, target_y, target_z, target_debug = get_robot_target_from_mask(
			sam_mask=target_mask,
			depth_image_m=depth_image_m,
			K_matrix=K_matrix,
			T_base_to_cam=T_base_to_cam,
			window_radius_px=window_radius_px,
		)

		probe_targets, plate_contour, probe_sampling_debug = generate_safe_probe_targets(
			sam_mask=target_mask,
			depth_image_m=depth_image_m,
			K_matrix=K_matrix,
			T_base_to_cam=T_base_to_cam,
			num_points=probe_point_count,
			edge_margin_px=edge_margin_px,
			depth_window_radius_px=probe_depth_window_radius_px,
			max_attempts=max_point_sampling_attempts,
			rng_seed=probe_random_seed,
		)

		overlay = build_probe_targets_overlay(
			image_bgr=color_image,
			mask=target_mask,
			probe_targets=probe_targets,
			contour=plate_contour,
			centroid_uv=tuple(target_debug['centroid_uv']),
		)

		metadata = {
			'source_segment_run_dir': str(run_dir),
			'centroid_target': {
				'base_point': [target_x, target_y, target_z],
				'centroid_uv': target_debug['centroid_uv'],
				'depth_m': target_debug['depth_m'],
				'valid_depth_count': target_debug['valid_depth_count'],
			},
			'probe_sampling_debug': probe_sampling_debug,
			'probe_targets': probe_targets,
			'segment_metadata': segment_metadata,
		}
		if save_artifacts:
			metadata = save_probe_artifacts(run_dir=run_dir, overlay=overlay, metadata=metadata)
			self.get_logger().info(
				f"Saved safe-probe artifacts to {metadata['artifact_paths']['safe_probe_targets_json']}")

		stamp = self.get_clock().now().to_msg()
		probe_targets_msg = build_pose_array(probe_targets, base_frame_id, stamp)
		metadata_json = json.dumps(metadata, indent=2)

		self._probe_targets_pub.publish(probe_targets_msg)
		self._private_probe_targets_pub.publish(probe_targets_msg)
		self._probe_metadata_pub.publish(String(data=metadata_json))
		self._private_metadata_pub.publish(String(data=metadata_json))

		self.get_logger().info(
			f'Published {len(probe_targets)} safe probe targets in frame {base_frame_id}')
		for target in probe_targets:
			base_x, base_y, base_z = target['base_point']
			self.get_logger().info(
				f"Probe {target['index']}: pixel={tuple(target['pixel_uv'])}, "
				f"edge={target['distance_to_edge_px']:.1f}px, depth={target['depth_m']:.4f} m -> "
				f'X={base_x:.4f}, Y={base_y:.4f}, Z={base_z:.4f}')

	@property
	def keep_alive(self) -> bool:
		return self._keep_alive


def main(args=None):
	rclpy.init(args=args)
	node = DepthToSafeProbeNode()
	try:
		node.run_once()
		if node.keep_alive:
			node.get_logger().info(
				'Safe probe targets are latched for the GUI. Press Ctrl-C to exit.')
			rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
