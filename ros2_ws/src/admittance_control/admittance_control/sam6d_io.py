"""ROS-agnostic I/O helpers for talking to the SAM-6D Flask server.

This module holds the pure image/HTTP/JSON plumbing so the ROS node
(`sam6d_bridge_node.py`) stays thin. Nothing here imports rclpy, so it can be
unit-tested without a running ROS graph. `image_msg_to_numpy` reads the fields
of a `sensor_msgs/Image` but only via duck typing (width/height/step/data/...).
"""

from __future__ import annotations

import base64
import json
import uuid
from pathlib import Path
from typing import Any, Dict, List, Tuple
from urllib import error as urllib_error
from urllib import request as urllib_request

import cv2
import numpy as np

PACKAGE_NAME = 'admittance_control'


# ── Camera intrinsics ────────────────────────────────────────────────────

def resolve_transfer_dir() -> Path:
    """Locate the directory that holds camera.json / rgb.png / depth.png.

    Prefers a checkout under `src/`, falls back to the installed `share/`
    location, and finally creates the sibling `rgb_depth_to_send` folder.
    """
    module_dir = Path(__file__).resolve().parent
    candidates: List[Path] = []
    for base in [module_dir, *module_dir.parents]:
        candidates.append(base / 'scripts' / 'rgb_depth_to_send')
        candidates.append(base / 'src' / PACKAGE_NAME / 'scripts' / 'rgb_depth_to_send')
        candidates.append(base / 'share' / PACKAGE_NAME / 'scripts' / 'rgb_depth_to_send')

    seen: set[Path] = set()
    ordered: List[Path] = []
    for candidate in candidates:
        if candidate not in seen:
            seen.add(candidate)
            ordered.append(candidate)

    for candidate in ordered:
        if (candidate / 'camera.json').exists():
            return candidate

    output_dir = ordered[0]
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def load_camera_payload(camera_path: Path) -> Dict[str, Any]:
    if not camera_path.exists():
        raise FileNotFoundError(
            f'camera.json was not found at {camera_path}. '
            'Place it under scripts/rgb_depth_to_send before running the bridge.'
        )
    with camera_path.open('r', encoding='utf-8') as camera_file:
        payload = json.load(camera_file)
    if 'cam_K' not in payload:
        raise ValueError(f'{camera_path} does not contain a cam_K entry.')
    return payload


# ── ROS Image → numpy (duck-typed on sensor_msgs/Image) ──────────────────

def image_msg_to_numpy(msg: Any) -> np.ndarray:
    encoding = msg.encoding.lower()
    if msg.is_bigendian:
        raise ValueError('Big-endian image messages are not supported.')

    encoding_map: Dict[str, Tuple[np.dtype, int]] = {
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


def normalize_depth_image(depth_image: np.ndarray, encoding: str,
                          camera_payload: Dict[str, Any]
                          ) -> Tuple[np.ndarray, Dict[str, Any]]:
    normalized = encoding.lower()
    resolved = dict(camera_payload)
    if normalized in ('16uc1', 'mono16'):
        return depth_image.astype(np.uint16, copy=False), resolved
    if normalized == '32fc1':
        depth_m = np.nan_to_num(depth_image.astype(np.float32, copy=False),
                                nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm = np.clip(np.rint(depth_m * 1000.0), 0.0,
                           float(np.iinfo(np.uint16).max))
        resolved['depth_scale'] = 1.0
        return depth_mm.astype(np.uint16), resolved
    raise ValueError(f'Unsupported depth image encoding: {encoding}')


def encode_png(image: np.ndarray, label: str) -> bytes:
    success, encoded = cv2.imencode('.png', image)
    if not success:
        raise RuntimeError(f'Failed to encode {label} as PNG.')
    return encoded.tobytes()


# ── HTTP multipart POST ──────────────────────────────────────────────────

def encode_multipart_formdata(files: Dict[str, Tuple[str, bytes, str]]
                              ) -> Tuple[bytes, str]:
    boundary = f'----Sam6DBridge{uuid.uuid4().hex}'
    parts: List[bytes] = []
    for field_name, (filename, payload, content_type) in files.items():
        parts.extend([
            f'--{boundary}\r\n'.encode('utf-8'),
            (f'Content-Disposition: form-data; name="{field_name}"; '
             f'filename="{filename}"\r\n').encode('utf-8'),
            f'Content-Type: {content_type}\r\n\r\n'.encode('utf-8'),
            payload,
            b'\r\n',
        ])
    parts.append(f'--{boundary}--\r\n'.encode('utf-8'))
    return b''.join(parts), boundary


def post_files(url: str, files: Dict[str, Tuple[str, bytes, str]],
               timeout_sec: float) -> Tuple[int, bytes]:
    body, boundary = encode_multipart_formdata(files)
    request = urllib_request.Request(url, data=body, method='POST')
    request.add_header('Content-Type', f'multipart/form-data; boundary={boundary}')
    request.add_header('Content-Length', str(len(body)))
    try:
        with urllib_request.urlopen(request, timeout=timeout_sec) as response:
            return int(response.status), response.read()
    except urllib_error.HTTPError as exc:
        return int(exc.code), exc.read()


# ── Response parsing ─────────────────────────────────────────────────────

def parse_pem_response(status_code: int, body: bytes
                       ) -> Tuple[bool, str, List[dict], Dict[str, str]]:
    """Parse the SAM-6D `/predict_pose` reply.

    The server returns ``{"status": "success", "pose": [<detection_pem>...],
    "artifacts": {<filename>: <base64>}}`` on success, or
    ``{"status"/"error": ..., "message"/"details": ...}`` on failure. Returns
    ``(ok, message, detections, artifacts)`` where ``detections`` is the raw
    detection_pem list (each entry has ``R``, ``t`` [mm], ``score``,
    ``category_id``, ``bbox``) and ``artifacts`` maps each raw output filename
    to its base64 payload (empty when the server sent none).
    """
    text = body.decode('utf-8', errors='replace')
    if status_code != 200:
        return False, f'server returned HTTP {status_code}: {text[:500]}', [], {}
    try:
        payload = json.loads(text)
    except ValueError:
        return False, f'response was not valid JSON: {text[:500]}', [], {}

    if isinstance(payload, dict) and payload.get('status') == 'success':
        pose = payload.get('pose', [])
        if not isinstance(pose, list):
            return False, 'response "pose" field was not a list', [], {}
        artifacts = payload.get('artifacts', {})
        if not isinstance(artifacts, dict):
            artifacts = {}
        return True, f'{len(pose)} raw detections', pose, artifacts

    message = ''
    if isinstance(payload, dict):
        message = str(payload.get('message') or payload.get('error')
                      or payload.get('details') or payload)
    return False, f'SAM-6D reported failure: {message[:500]}', [], {}


def save_artifacts(artifacts: Dict[str, str], dest_dir: Path) -> List[str]:
    """base64-decode the raw pipeline outputs and write them into ``dest_dir``.

    Files are overwritten each call. ``base64.b64decode`` reproduces the bytes
    exactly, so the binary ``.npz`` survives the round-trip. The stored name is
    reduced to its basename to guard against path traversal from the payload.
    Returns the list of filenames written.
    """
    dest_dir.mkdir(parents=True, exist_ok=True)
    written: List[str] = []
    for name, encoded in artifacts.items():
        safe_name = Path(name).name
        if not safe_name:
            continue
        (dest_dir / safe_name).write_bytes(base64.b64decode(encoded))
        written.append(safe_name)
    return written
