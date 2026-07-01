#!/usr/bin/env python3
"""ROS 2 helper for one-shot SAM3 segmentation on aligned RGB-D frames.

This node captures a single aligned RGB-D frame from a connected RealSense
camera, applies the patched SAM3 prompt-segmentation flow from the notebook,
publishes the mask and overlay as latched topics, saves artifacts, and shows
an optional preview window when an interactive Matplotlib backend is available.
"""

from __future__ import annotations

import json
import os
import shutil
import time
import urllib.request
import importlib
from pathlib import Path
from typing import Any, Optional, Tuple, cast

import cv2
import matplotlib

if os.environ.get('DISPLAY'):
	try:
		matplotlib.use('Qt5Agg')
	except Exception:
		pass

import matplotlib.pyplot as plt
import numpy as np
import pyrealsense2 as rs
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
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
DEFAULT_ARTIFACT_DIR = SOURCE_PACKAGE_ROOT / 'artifacts' / 'segment_area'

_SAM3_PROCESSOR = None
_SAM3_DEVICE = None
_SAM3_MODEL = None


def _log(logger: Optional[Any], level: str, message: str) -> None:
	if logger is None:
		print(message)
		return

	target_logger = logger.get_logger() if hasattr(logger, 'get_logger') else logger
	severity = str(level).lower()

	if severity in ('warn', 'warning'):
		warning_method = getattr(target_logger, 'warning', None)
		if callable(warning_method):
			warning_method(message)
			return
		warn_method = getattr(target_logger, 'warn', None)
		if callable(warn_method):
			warn_method(message)
			return
		print(message)
		return

	if severity == 'error':
		error_method = getattr(target_logger, 'error', None)
		if callable(error_method):
			error_method(message)
			return
		print(message)
		return

	if severity == 'debug':
		debug_method = getattr(target_logger, 'debug', None)
		if callable(debug_method):
			debug_method(message)
			return
		print(message)
		return

	if severity == 'fatal':
		fatal_method = getattr(target_logger, 'fatal', None)
		if callable(fatal_method):
			fatal_method(message)
			return
		print(message)
		return

	info_method = getattr(target_logger, 'info', None)
	if callable(info_method):
		info_method(message)
		return

	print(message)


def _parse_manual_roi(raw_roi: str) -> Optional[Tuple[int, int, int, int]]:
	raw_roi = raw_roi.strip()
	if not raw_roi:
		return None

	parts = [part.strip() for part in raw_roi.split(',')]
	if len(parts) != 4:
		raise ValueError(
			'manual_roi must be empty or formatted as x,y,w,h')

	x, y, width, height = [int(part) for part in parts]
	return x, y, width, height


def connect_realsense_rgbd(
	width: int,
	height: int,
	depth_width: int,
	depth_height: int,
	fps: int,
	warmup_frames: int,
):
	rs_module = cast(Any, rs)
	ctx = rs_module.context()
	devices = list(ctx.query_devices())
	if not devices:
		raise RuntimeError('No RealSense device is visible in this container.')

	device = devices[0]
	device_name = (
		device.get_info(rs_module.camera_info.name)
		if device.supports(rs_module.camera_info.name)
		else 'RealSense camera'
	)
	serial_number = (
		device.get_info(rs_module.camera_info.serial_number)
		if device.supports(rs_module.camera_info.serial_number)
		else None
	)
	sensor_names = [
		sensor.get_info(rs_module.camera_info.name)
		for sensor in device.query_sensors()
		if sensor.supports(rs_module.camera_info.name)
	]
	if 'RGB Camera' not in sensor_names:
		raise RuntimeError(
			f'RGB Camera sensor is missing. Visible sensors: {sensor_names}')

	pipeline = rs_module.pipeline()
	config = rs_module.config()
	if serial_number:
		config.enable_device(serial_number)
	config.enable_stream(
		rs_module.stream.color,
		width,
		height,
		rs_module.format.bgr8,
		fps,
	)
	config.enable_stream(
		rs_module.stream.depth,
		depth_width,
		depth_height,
		rs_module.format.z16,
		fps,
	)

	profile = pipeline.start(config)
	depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
	align = rs_module.align(rs_module.stream.color)

	for _ in range(max(int(warmup_frames), 0)):
		pipeline.wait_for_frames()

	camera_info = {
		'device_name': device_name,
		'serial_number': serial_number,
		'sensor_names': sensor_names,
		'depth_scale': float(depth_scale),
		'rgb_resolution': [int(width), int(height)],
		'depth_resolution': [int(depth_width), int(depth_height)],
		'fps': int(fps),
	}
	return pipeline, align, float(depth_scale), camera_info


def capture_aligned_rgbd_frame(pipeline, align):
	frames = pipeline.wait_for_frames()
	aligned_frames = align.process(frames)

	color_frame = aligned_frames.get_color_frame()
	depth_frame = aligned_frames.get_depth_frame()
	if not color_frame or not depth_frame:
		raise RuntimeError('Failed to capture an aligned RGB-D frame.')

	color_image = np.asanyarray(color_frame.get_data())
	depth_image = np.asanyarray(depth_frame.get_data())
	return color_image, depth_image


def _resolve_sam3_bpe_path(sam3_module, logger: Optional[Node]) -> str:
	bpe_filename = 'bpe_simple_vocab_16e6.txt.gz'
	package_dir = Path(sam3_module.__file__).resolve().parent
	candidate_paths = [
		package_dir.parent / 'assets' / bpe_filename,
		package_dir / 'assets' / bpe_filename,
	]

	for candidate_path in candidate_paths:
		if candidate_path.exists():
			return str(candidate_path)

	cache_path = NOTEBOOK_DIR / '.cache' / 'sam3' / bpe_filename
	if cache_path.exists():
		return str(cache_path)

	cache_path.parent.mkdir(parents=True, exist_ok=True)
	bpe_urls = [
		'https://raw.githubusercontent.com/facebookresearch/sam3/main/sam3/assets/bpe_simple_vocab_16e6.txt.gz',
		'https://raw.githubusercontent.com/openai/CLIP/main/clip/bpe_simple_vocab_16e6.txt.gz',
	]

	last_error = None
	for bpe_url in bpe_urls:
		try:
			cache_path.unlink(missing_ok=True)
			with urllib.request.urlopen(bpe_url, timeout=30) as response:
				with open(cache_path, 'wb') as output_file:
					shutil.copyfileobj(response, output_file)
			_log(logger, 'info', f'Downloaded SAM3 tokenizer BPE to: {cache_path}')
			return str(cache_path)
		except Exception as exc:
			last_error = exc

	raise RuntimeError(
		'Could not locate or download the SAM3 tokenizer BPE file. '
		f'Tried: {candidate_paths + [cache_path]}') from last_error


def ensure_sam3_processor(
	confidence_threshold: float,
	repo_id: str,
	logger: Optional[Any] = None,
):
	global _SAM3_DEVICE, _SAM3_MODEL, _SAM3_PROCESSOR

	if _SAM3_PROCESSOR is not None:
		return _SAM3_PROCESSOR, _SAM3_DEVICE

	missing_dependencies = []
	try:
		import sam3  # noqa: F401
		import torch  # noqa: F401
	except ModuleNotFoundError as exc:
		missing_dependencies.append(exc.name)

	try:
		from huggingface_hub import hf_hub_download  # noqa: F401
	except ModuleNotFoundError:
		missing_dependencies.append('huggingface_hub')

	try:
		from sam3.model_builder import build_sam3_image_model  # noqa: F401
		from sam3.model.sam3_image_processor import Sam3Processor  # noqa: F401
		import sam3.model.vitdet as vitdet  # noqa: F401
	except ModuleNotFoundError:
		missing_dependencies.append('sam3')

	if missing_dependencies:
		missing_names = ', '.join(sorted(set(missing_dependencies)))
		raise RuntimeError(
			'Missing SAM3 setup dependencies in the current environment: '
			f'{missing_names}. Install them from '
			'/workspaces/welding_cell_ws/requirements.txt and ensure the '
			'workspace venv is active before running segment_area.py.')

	import sam3
	import torch
	from huggingface_hub import hf_hub_download
	from sam3.model_builder import build_sam3_image_model
	from sam3.model.sam3_image_processor import Sam3Processor
	import sam3.model.vitdet as vitdet

	try:
		fused = importlib.import_module('sam3.perflib.fused')
	except ModuleNotFoundError:
		fused = None
		_log(
			logger,
			'warn',
			'sam3.perflib.fused is not available in this SAM3 build. '
			'Using the vitdet fallback patch only.',
		)

	def _addmm_act_float32(activation, linear, mat1):
		out = torch.nn.functional.linear(mat1, linear.weight, linear.bias)
		if activation in [torch.nn.functional.gelu, torch.nn.GELU]:
			return torch.nn.functional.gelu(out)
		if activation in [torch.nn.functional.relu, torch.nn.ReLU]:
			return torch.nn.functional.relu(out)
		raise ValueError(f'Unexpected activation {activation}')

	if fused is not None and hasattr(fused, 'addmm_act'):
		setattr(fused, 'addmm_act', _addmm_act_float32)
	setattr(vitdet, 'addmm_act', _addmm_act_float32)

	_SAM3_DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
	checkpoint_path = hf_hub_download(repo_id=repo_id, filename='sam3.pt')
	sam3_bpe_path = _resolve_sam3_bpe_path(sam3, logger)
	_log(logger, 'info', f'Using SAM3 tokenizer BPE: {sam3_bpe_path}')

	_SAM3_MODEL = build_sam3_image_model(
		checkpoint_path=checkpoint_path,
		bpe_path=sam3_bpe_path,
		device=_SAM3_DEVICE,
	)
	_SAM3_PROCESSOR = Sam3Processor(
		_SAM3_MODEL,
		device=_SAM3_DEVICE,
		confidence_threshold=confidence_threshold,
	)
	_log(
		logger,
		'info',
		f'SAM3 loaded on {_SAM3_DEVICE} '
		f'(confidence_threshold={confidence_threshold})',
	)
	return _SAM3_PROCESSOR, _SAM3_DEVICE


def _validate_roi(image_bgr: np.ndarray, roi: Tuple[int, int, int, int]):
	x, y, width, height = [int(value) for value in roi]
	if width <= 0 or height <= 0:
		raise RuntimeError(f'Invalid manual ROI: {roi}')

	image_height, image_width = image_bgr.shape[:2]
	x = max(0, min(x, image_width - 1))
	y = max(0, min(y, image_height - 1))
	width = min(width, image_width - x)
	height = min(height, image_height - y)
	if width <= 0 or height <= 0:
		raise RuntimeError(
			f'Manual ROI falls outside the current image bounds: {roi}')
	return x, y, width, height


def manual_roi_mask(image_bgr: np.ndarray, roi: Tuple[int, int, int, int]):
	x, y, width, height = _validate_roi(image_bgr, roi)
	mask = np.zeros(image_bgr.shape[:2], dtype=bool)
	mask[y:y + height, x:x + width] = True
	return mask, {'mode': 'manual_roi', 'roi': [x, y, width, height]}


def default_sam3_segmenter(
	image_rgb: np.ndarray,
	prompt_text: str,
	roi: Optional[Tuple[int, int, int, int]],
	confidence_threshold: float,
	repo_id: str,
	logger: Optional[Any] = None,
):
	processor, device = ensure_sam3_processor(
		confidence_threshold=confidence_threshold,
		repo_id=repo_id,
		logger=logger,
	)
	import torch
	from PIL import Image as PILImage

	x_offset = 0
	y_offset = 0
	segment_image = image_rgb

	if roi is not None:
		x_offset, y_offset, roi_width, roi_height = roi
		segment_image = image_rgb[
			y_offset:y_offset + roi_height,
			x_offset:x_offset + roi_width,
		]
		if segment_image.size == 0:
			raise RuntimeError('Selected ROI is empty after cropping.')

	with torch.inference_mode():
		inference_state = processor.set_image(PILImage.fromarray(segment_image))
		output = processor.set_text_prompt(
			state=inference_state,
			prompt=prompt_text,
		)

	scores = output['scores'].detach().cpu().numpy()
	masks = output['masks'].detach().cpu().numpy()
	boxes = output['boxes'].detach().cpu().numpy() if 'boxes' in output else None

	if len(scores) == 0:
		raise RuntimeError(
			f"SAM3 found no detections for prompt '{prompt_text}'. Try a "
			'broader prompt or use a tighter ROI.')

	best_index = int(np.argmax(scores))
	crop_mask = masks[best_index].squeeze().astype(bool)

	full_mask = np.zeros(image_rgb.shape[:2], dtype=bool)
	full_mask[
		y_offset:y_offset + crop_mask.shape[0],
		x_offset:x_offset + crop_mask.shape[1],
	] = crop_mask

	info = {
		'mode': 'sam3_prompt',
		'prompt': prompt_text,
		'score': float(scores[best_index]),
		'device': device,
	}
	if roi is not None:
		info['roi'] = [int(value) for value in roi]

	if boxes is not None and len(boxes) > best_index:
		box_xyxy = np.asarray(boxes[best_index], dtype=float).reshape(-1)[:4]
		box_xyxy[[0, 2]] += x_offset
		box_xyxy[[1, 3]] += y_offset
		info['box_xyxy'] = box_xyxy.tolist()

	return full_mask, info


def sam3_prompt_mask(
	image_bgr: np.ndarray,
	prompt_text: str,
	roi: Optional[Tuple[int, int, int, int]],
	confidence_threshold: float,
	repo_id: str,
	logger: Optional[Any] = None,
):
	image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
	mask, info = default_sam3_segmenter(
		image_rgb=image_rgb,
		prompt_text=prompt_text,
		roi=roi,
		confidence_threshold=confidence_threshold,
		repo_id=repo_id,
		logger=logger,
	)

	mask = np.asarray(mask)
	if mask.ndim > 2:
		mask = np.squeeze(mask)
	if mask.shape != image_bgr.shape[:2]:
		raise ValueError(
			f'SAM3 mask shape {mask.shape} does not match image shape '
			f'{image_bgr.shape[:2]}')

	return mask.astype(bool), info


def select_target_mask(
	image_bgr: np.ndarray,
	mode: str,
	prompt_text: str,
	manual_roi: Optional[Tuple[int, int, int, int]],
	confidence_threshold: float,
	repo_id: str,
	logger: Optional[Any] = None,
):
	if mode == 'manual_roi':
		if manual_roi is None:
			raise RuntimeError(
				'mask_mode=manual_roi requires manual_roi to be set as x,y,w,h.')
		return manual_roi_mask(image_bgr, manual_roi)

	if mode == 'sam3_prompt':
		return sam3_prompt_mask(
			image_bgr=image_bgr,
			prompt_text=prompt_text,
			roi=None,
			confidence_threshold=confidence_threshold,
			repo_id=repo_id,
			logger=logger,
		)

	if mode == 'sam3_prompt_with_roi':
		if manual_roi is None:
			raise RuntimeError(
				'mask_mode=sam3_prompt_with_roi requires manual_roi to be '
				'set as x,y,w,h.')
		roi = _validate_roi(image_bgr, manual_roi)
		return sam3_prompt_mask(
			image_bgr=image_bgr,
			prompt_text=prompt_text,
			roi=roi,
			confidence_threshold=confidence_threshold,
			repo_id=repo_id,
			logger=logger,
		)

	raise ValueError(f'Unsupported mask_mode: {mode}')


def get_mask_depth_summary(
	mask: np.ndarray,
	depth_image: np.ndarray,
	depth_scale: float,
	window_radius_px: int,
):
	y_indices, x_indices = np.where(mask > 0)
	if len(x_indices) == 0:
		raise ValueError('Selected mask is empty.')

	u_center = int(np.mean(x_indices))
	v_center = int(np.mean(y_indices))
	depth_window = depth_image[
		max(0, v_center - window_radius_px):min(
			depth_image.shape[0], v_center + window_radius_px),
		max(0, u_center - window_radius_px):min(
			depth_image.shape[1], u_center + window_radius_px),
	]
	valid_depths = depth_window[depth_window > 0]

	depth_m = None
	if valid_depths.size > 0:
		depth_m = float(np.median(valid_depths) * depth_scale)

	return {
		'centroid_uv': [u_center, v_center],
		'depth_m': depth_m,
		'valid_depth_count': int(valid_depths.size),
		'mask_pixel_count': int(mask.sum()),
	}


def build_mask_overlay(
	image_bgr: np.ndarray,
	mask: np.ndarray,
	centroid_uv: Optional[Tuple[int, int]] = None,
	selection_info: Optional[dict] = None,
) -> np.ndarray:
	overlay = image_bgr.copy()
	mask_bool = np.asarray(mask, dtype=bool)
	highlight_color = np.array([0, 255, 255], dtype=np.uint8)
	overlay[mask_bool] = (
		0.4 * overlay[mask_bool] + 0.6 * highlight_color).astype(np.uint8)

	if selection_info is not None and selection_info.get('roi') is not None:
		x, y, width, height = [int(value) for value in selection_info['roi']]
		cv2.rectangle(overlay, (x, y), (x + width, y + height), (255, 0, 0), 2)

	if selection_info is not None and selection_info.get('box_xyxy') is not None:
		x1, y1, x2, y2 = [
			int(round(value)) for value in selection_info['box_xyxy']
		]
		cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)

	if centroid_uv is not None:
		cv2.circle(overlay, centroid_uv, 8, (0, 0, 255), -1)

	return overlay


def save_segmentation_artifacts(
	output_dir: Path,
	color_image: np.ndarray,
	depth_image: np.ndarray,
	depth_scale: float,
	mask: np.ndarray,
	overlay: np.ndarray,
	metadata: dict,
):
	output_dir.mkdir(parents=True, exist_ok=True)
	timestamp = time.strftime('%Y%m%d_%H%M%S')
	run_dir = output_dir / timestamp
	run_dir.mkdir(parents=True, exist_ok=True)

	color_path = run_dir / 'color_bgr.png'
	overlay_path = run_dir / 'overlay_bgr.png'
	mask_path = run_dir / 'mask.png'
	depth_path = run_dir / 'depth_m.npy'
	metadata_path = run_dir / 'metadata.json'

	cv2.imwrite(str(color_path), color_image)
	cv2.imwrite(str(overlay_path), overlay)
	cv2.imwrite(str(mask_path), mask.astype(np.uint8) * 255)
	np.save(depth_path, depth_image.astype(np.float32) * float(depth_scale))

	metadata = dict(metadata)
	metadata['artifact_paths'] = {
		'run_dir': str(run_dir),
		'color_bgr_png': str(color_path),
		'overlay_bgr_png': str(overlay_path),
		'mask_png': str(mask_path),
		'depth_m_npy': str(depth_path),
		'metadata_json': str(metadata_path),
	}

	with open(metadata_path, 'w', encoding='utf-8') as output_file:
		json.dump(metadata, output_file, indent=2)

	return metadata


def numpy_to_image_msg(
	image: np.ndarray,
	encoding: str,
	stamp,
	frame_id: str,
) -> Image:
	image = np.ascontiguousarray(image)
	msg = Image()
	msg.header.stamp = stamp
	msg.header.frame_id = frame_id
	msg.height = int(image.shape[0])
	msg.width = int(image.shape[1])
	msg.encoding = encoding
	msg.is_bigendian = False
	msg.step = int(image.strides[0])
	msg.data = image.tobytes()
	return msg


class SegmentAreaNode(Node):
	def __init__(self):
		super().__init__('segment_area')

		self.declare_parameter('rgb_width', 1280)
		self.declare_parameter('rgb_height', 720)
		self.declare_parameter('depth_width', 848)
		self.declare_parameter('depth_height', 480)
		self.declare_parameter('fps', 30)
		self.declare_parameter('capture_warmup_frames', 15)
		self.declare_parameter('mask_mode', 'sam3_prompt')
		self.declare_parameter(
			'sam3_prompt_text',
			'metal plate or sheet metal plate part in the middle of the MDF board',
		)
		self.declare_parameter('manual_roi', '')
		self.declare_parameter('sam3_confidence_threshold', 0.1)
		self.declare_parameter('sam3_repo_id', 'facebook/sam3')
		self.declare_parameter('window_radius_px', 5)
		self.declare_parameter('show_plot', True)
		self.declare_parameter('save_artifacts', True)
		self.declare_parameter('artifact_dir', str(DEFAULT_ARTIFACT_DIR))
		self.declare_parameter('camera_frame_id', 'camera_color_optical_frame')

		qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
		self._mask_pub = self.create_publisher(Image, '~/mask', qos)
		self._overlay_pub = self.create_publisher(Image, '~/overlay', qos)
		self._depth_pub = self.create_publisher(Image, '~/depth_m', qos)
		self._metadata_pub = self.create_publisher(String, '~/metadata', qos)

		self._pipeline = None
		self._align = None
		self._depth_scale = None
		self._figure = None
		self._axes = None

	def _parameter_value(self, name: str) -> Any:
		value = self.get_parameter(name).value
		if value is None:
			raise RuntimeError(f'Required parameter {name!r} is unset.')
		return value

	def run_once(self):
		rgb_width = int(self._parameter_value('rgb_width'))
		rgb_height = int(self._parameter_value('rgb_height'))
		depth_width = int(self._parameter_value('depth_width'))
		depth_height = int(self._parameter_value('depth_height'))
		fps = int(self._parameter_value('fps'))
		warmup_frames = int(self._parameter_value('capture_warmup_frames'))
		mask_mode = str(self._parameter_value('mask_mode'))
		prompt_text = str(self._parameter_value('sam3_prompt_text'))
		manual_roi = _parse_manual_roi(str(self._parameter_value('manual_roi')))
		confidence_threshold = float(
			self._parameter_value('sam3_confidence_threshold'))
		repo_id = str(self._parameter_value('sam3_repo_id'))
		window_radius_px = int(self._parameter_value('window_radius_px'))
		show_plot = bool(self._parameter_value('show_plot'))
		save_artifacts = bool(self._parameter_value('save_artifacts'))
		artifact_dir = Path(str(self._parameter_value('artifact_dir')))
		camera_frame_id = str(self._parameter_value('camera_frame_id'))

		self.get_logger().info('Connecting to RealSense and capturing an aligned RGB-D frame...')
		self._pipeline, self._align, self._depth_scale, camera_info = connect_realsense_rgbd(
			width=rgb_width,
			height=rgb_height,
			depth_width=depth_width,
			depth_height=depth_height,
			fps=fps,
			warmup_frames=warmup_frames,
		)
		color_image, depth_image = capture_aligned_rgbd_frame(
			self._pipeline,
			self._align,
		)

		self.get_logger().info(
			f'Running SAM3 segmentation with mask_mode={mask_mode} and prompt={prompt_text!r}')
		mask, mask_info = select_target_mask(
			image_bgr=color_image,
			mode=mask_mode,
			prompt_text=prompt_text,
			manual_roi=manual_roi,
			confidence_threshold=confidence_threshold,
			repo_id=repo_id,
			logger=self.get_logger(),
		)
		depth_summary = get_mask_depth_summary(
			mask=mask,
			depth_image=depth_image,
			depth_scale=self._depth_scale,
			window_radius_px=window_radius_px,
		)

		centroid_uv = tuple(depth_summary['centroid_uv'])
		overlay = build_mask_overlay(
			image_bgr=color_image,
			mask=mask,
			centroid_uv=centroid_uv,
			selection_info=mask_info,
		)

		metadata = {
			'camera_info': camera_info,
			'mask_info': mask_info,
			'depth_summary': depth_summary,
			'mask_mode': mask_mode,
			'prompt_text': prompt_text,
		}
		if save_artifacts:
			metadata = save_segmentation_artifacts(
				output_dir=artifact_dir,
				color_image=color_image,
				depth_image=depth_image,
				depth_scale=self._depth_scale,
				mask=mask,
				overlay=overlay,
				metadata=metadata,
			)
			self.get_logger().info(
				f'Saved segmentation artifacts under '
				f"{metadata['artifact_paths']['run_dir']}")

		self._publish_result(
			overlay=overlay,
			mask=mask,
			depth_image=depth_image,
			depth_scale=self._depth_scale,
			metadata=metadata,
			frame_id=camera_frame_id,
		)

		score = mask_info.get('score')
		score_suffix = f' score={score:.4f}' if score is not None else ''
		self.get_logger().info(
			f'Segmentation complete: mask_pixels={depth_summary["mask_pixel_count"]}, '
			f'centroid={tuple(depth_summary["centroid_uv"])}{score_suffix}')

		if show_plot:
			self._show_plot(
				color_image=color_image,
				mask=mask,
				overlay=overlay,
				mask_info=mask_info,
				depth_summary=depth_summary,
			)

	def _publish_result(
		self,
		overlay: np.ndarray,
		mask: np.ndarray,
		depth_image: np.ndarray,
		depth_scale: float,
		metadata: dict,
		frame_id: str,
	):
		stamp = self.get_clock().now().to_msg()
		overlay_rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
		mask_mono = mask.astype(np.uint8) * 255
		depth_m = depth_image.astype(np.float32) * float(depth_scale)

		self._overlay_pub.publish(
			numpy_to_image_msg(overlay_rgb, 'rgb8', stamp, frame_id))
		self._mask_pub.publish(
			numpy_to_image_msg(mask_mono, 'mono8', stamp, frame_id))
		self._depth_pub.publish(
			numpy_to_image_msg(depth_m, '32FC1', stamp, frame_id))
		self._metadata_pub.publish(String(data=json.dumps(metadata, indent=2)))

	def _show_plot(
		self,
		color_image: np.ndarray,
		mask: np.ndarray,
		overlay: np.ndarray,
		mask_info: dict,
		depth_summary: dict,
	):
		backend_name = str(matplotlib.get_backend()).lower()
		if 'agg' in backend_name:
			self.get_logger().warn(
				'Matplotlib is using a non-interactive backend; skipping the '
				'preview window. Check the saved artifacts instead.')
			return

		if self._figure is None or self._axes is None:
			plt.ion()
			self._figure, self._axes = plt.subplots(
				1, 3, figsize=(16, 5), num='SAM3 Segmentation Preview')

		for axis in self._axes:
			axis.clear()

		self._axes[0].imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
		self._axes[0].set_title('Aligned RGB frame')
		self._axes[1].imshow(mask.astype(np.uint8), cmap='gray')
		self._axes[1].set_title('SAM3 mask')
		self._axes[2].imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))

		overlay_title = 'Segmentation overlay'
		if mask_info.get('score') is not None:
			overlay_title += f"\nscore={mask_info['score']:.3f}"
		self._axes[2].set_title(overlay_title)

		for axis in self._axes:
			axis.axis('off')

		centroid_uv = tuple(depth_summary['centroid_uv'])
		depth_m = depth_summary.get('depth_m')
		depth_str = f'{depth_m:.4f} m' if depth_m is not None else 'unavailable'
		self._figure.suptitle(
			f'SAM3 centroid={centroid_uv}, depth={depth_str}',
			fontsize=13,
		)
		self._figure.tight_layout()
		self._figure.canvas.draw_idle()
		plt.show(block=False)
		plt.pause(0.001)

	def spin_plot_window(self):
		if self._figure is None:
			time.sleep(0.2)
			return

		while rclpy.ok() and plt.fignum_exists(self._figure.number):
			rclpy.spin_once(self, timeout_sec=0.1)
			plt.pause(0.05)

	def cleanup(self):
		if self._pipeline is not None:
			try:
				self._pipeline.stop()
			except Exception:
				pass
			self._pipeline = None


def main(args=None):
	rclpy.init(args=args)
	node = SegmentAreaNode()
	try:
		node.run_once()
		node.spin_plot_window()
	except KeyboardInterrupt:
		pass
	finally:
		node.cleanup()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
