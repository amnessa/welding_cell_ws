"""Render configuration and its id - Phase 8.

`render_id` plays for a render pass the role `config_id` plays for a scene: the first 8 hex
characters of the sha256 of the canonical resolved config. Two things are deliberately
outside the hash - filesystem paths (machine-specific) - and one thing deliberately
inside it - the background set's own hash from its manifest, so a different set of
photographs is a different render_id.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

from ..hashing import canonical_json

PATH_KEYS = ("backgrounds_manifest", "panoramas_dir", "render_assets_manifest")


def load_render_config(path: str | Path) -> dict[str, Any]:
    cfg = yaml.safe_load(Path(path).read_text())
    cfg["_source"] = str(path)
    return cfg


def load_backgrounds(cfg: dict[str, Any], root: str | Path) -> dict[str, Any]:
    """The backgrounds manifest, with absolute photo/panorama paths attached."""
    root = Path(root)
    mpath = root / cfg["environment"]["backgrounds_manifest"]
    m = json.loads(mpath.read_text())
    m["_dir"] = str(mpath.parent)
    m["_panoramas"] = sorted(str(p) for p in (root / cfg["environment"]["panoramas_dir"]).glob("*")
                             if p.suffix.lower() in (".jpeg", ".jpg", ".png", ".hdr", ".exr"))
    return m


def load_render_assets(cfg: dict[str, Any], root: str | Path) -> dict[str, Any] | None:
    """The CC0 asset manifest (scripts/fetch_render_assets.py) with absolute paths, or None."""
    rel = cfg["environment"].get("render_assets_manifest")
    if not rel:
        return None
    mpath = Path(root) / rel
    if not mpath.exists():
        return None
    m = json.loads(mpath.read_text()); m["_dir"] = str(mpath.parent)
    return m


def hashable_config(cfg: dict[str, Any], backgrounds: dict[str, Any] | None, assets: dict[str, Any] | None = None) -> dict[str, Any]:
    out = json.loads(json.dumps({k: v for k, v in cfg.items() if not k.startswith("_")}))
    env = dict(out.get("environment", {}))
    for k in PATH_KEYS:
        env.pop(k, None)
    out["environment"] = env
    if backgrounds is not None:
        out["backgrounds_set_hash"] = backgrounds["set_hash"]
        out["panoramas"] = [Path(p).name for p in backgrounds["_panoramas"]]
    if assets is not None:
        out["render_assets_set_hash"] = assets["set_hash"]
        out["render_assets_version"] = assets.get("version")
    return out


def render_id(cfg: dict[str, Any], backgrounds: dict[str, Any] | None, assets: dict[str, Any] | None = None) -> str:
    return hashlib.sha256(canonical_json(hashable_config(cfg, backgrounds, assets)).encode()).hexdigest()[:8]
