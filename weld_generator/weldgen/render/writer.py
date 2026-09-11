"""Write a rendered scene's tier-2 layer - and nothing else.

    <scene_id>/
      scene.json, cloud.npz, seams.npz, scene.sha256   <- tier 1, NEVER touched
      render.json                                      <- this pass: ids, draws, views
      render.sha256                                    <- sha256 over every view's depth +
                                                          masks + the resolved config
      views/<k>/rgb.png depth.png depth_valid.png mask_seam.png mask_tack.png
                mask_object.png view.json

`render.sha256` is what makes two renders comparable: depth is a ray cast and the masks
are constructed, so the same render_id on another machine must reproduce it bit for bit.
RGB is path-traced and NOT bit-stable across GPUs, so its per-view sha256 is recorded in
`view.json` as information and never enters `render.sha256`.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

from ..hashing import canonical_json
from .conventions import DEPTH_SCALE_MM, decode_depth, encode_depth

RENDER_SCHEMA_VERSION = "1.0.0"
VIEW_FILES = ("rgb.png", "depth.png", "depth_valid.png", "mask_seam.png", "mask_tack.png",
              "mask_object.png", "view.json")


def _png(path: Path, arr: np.ndarray) -> None:
    Image.fromarray(arr).save(path, compress_level=6)


def _sha(arr: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(arr).tobytes()).hexdigest()


def write_view(view_dir: Path, rgb: np.ndarray, depth_mm: np.ndarray, valid: np.ndarray,
               mask_seam: np.ndarray, mask_tack: np.ndarray, mask_object: np.ndarray,
               meta: dict, sensor_valid: np.ndarray | None = None) -> dict:
    """Write one view folder; return the entry that goes into render.json."""
    view_dir = Path(view_dir); view_dir.mkdir(parents=True, exist_ok=True)
    png16 = encode_depth(depth_mm, valid)
    _png(view_dir / "rgb.png", np.asarray(rgb, np.uint8))
    _png(view_dir / "depth.png", png16)
    # depth_valid = the DETERMINISTIC sensor validity (D16 grazing dropout + blind zone) on the
    # rendered pixels - see render/sensor.py. Without a sensor model it is the render validity.
    dv = (png16 > 0) if sensor_valid is None else ((png16 > 0) & np.asarray(sensor_valid, bool))
    dv8 = dv.astype(np.uint8) * 255
    _png(view_dir / "depth_valid.png", dv8)
    _png(view_dir / "mask_seam.png", np.asarray(mask_seam, np.uint8))
    _png(view_dir / "mask_tack.png", np.asarray(mask_tack, np.uint8))
    _png(view_dir / "mask_object.png", np.asarray(mask_object, np.uint8))
    entry = {**meta, "files": list(VIEW_FILES),
             "depth": {"file": "depth.png", "encoding": "uint16", "scale_mm": DEPTH_SCALE_MM, "invalid_value": 0,
                       "valid_fraction": float((png16 > 0).mean()), "sensor_valid_fraction": float(dv.mean())},
             "rgb_sha256": _sha(np.asarray(rgb, np.uint8)),
             "hashed": {"depth": _sha(png16), "depth_valid": _sha(dv8), "mask_seam": _sha(np.asarray(mask_seam, np.uint8)),
                        "mask_tack": _sha(np.asarray(mask_tack, np.uint8)),
                        "mask_object": _sha(np.asarray(mask_object, np.uint8))}}
    (view_dir / "view.json").write_text(json.dumps(entry, indent=1, sort_keys=True))
    return entry


def render_hash(view_entries: list[dict], hashable_config: dict) -> str:
    """sha256 over the resolved config and, per view in order, the hashed arrays' digests."""
    h = hashlib.sha256(canonical_json(hashable_config).encode())
    for e in view_entries:
        for name in ("depth", "depth_valid", "mask_seam", "mask_tack", "mask_object"):
            h.update(f"{e['view']}:{name}:".encode()); h.update(e["hashed"][name].encode())
    return h.hexdigest()


def write_render(scene_dir: Path, render_meta: dict, view_entries: list[dict], hashable_config: dict) -> str:
    """Write render.json + render.sha256; return the digest. Tier-1 files are not opened."""
    scene_dir = Path(scene_dir)
    digest = render_hash(view_entries, hashable_config)
    doc = {"render_schema_version": RENDER_SCHEMA_VERSION, **render_meta,
           "config": hashable_config, "views": view_entries}
    (scene_dir / "render.json").write_text(json.dumps(doc, indent=1, sort_keys=True))
    (scene_dir / "render.sha256").write_text(digest + "\n")
    return digest


def read_view(view_dir: Path) -> dict:
    view_dir = Path(view_dir)
    depth_mm, valid = decode_depth(np.array(Image.open(view_dir / "depth.png")))
    return {"rgb": np.array(Image.open(view_dir / "rgb.png")), "depth_mm": depth_mm, "valid": valid,
            "sensor_valid": np.array(Image.open(view_dir / "depth_valid.png")) > 0,
            "mask_seam": np.array(Image.open(view_dir / "mask_seam.png")),
            "mask_tack": np.array(Image.open(view_dir / "mask_tack.png")),
            "mask_object": np.array(Image.open(view_dir / "mask_object.png")),
            "meta": json.loads((view_dir / "view.json").read_text())}


def verify_render(scene_dir: Path) -> tuple[bool, str]:
    """Re-hash the views on disk against render.sha256 (the tier-2 `weldgen verify`)."""
    scene_dir = Path(scene_dir)
    doc = json.loads((scene_dir / "render.json").read_text())
    entries = []
    for e in doc["views"]:
        if e.get("view_kind") == "undrawable":
            continue
        v = read_view(scene_dir / "views" / str(e["view"]))
        png16 = np.array(Image.open(scene_dir / "views" / str(e["view"]) / "depth.png"))
        dv8 = np.array(Image.open(scene_dir / "views" / str(e["view"]) / "depth_valid.png"))
        entries.append({"view": e["view"], "hashed": {"depth": _sha(png16), "depth_valid": _sha(dv8), "mask_seam": _sha(v["mask_seam"]),
                                                     "mask_tack": _sha(v["mask_tack"]), "mask_object": _sha(v["mask_object"])}})
    want = (scene_dir / "render.sha256").read_text().strip()
    got = render_hash(entries, doc["config"])
    return want == got, got
