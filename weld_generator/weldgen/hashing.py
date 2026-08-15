"""Canonical serialisation and content hashing — SCHEMA.md §6.2.

The Phase 1 determinism gate (D15) is defined on **content**, not on file bytes:
`np.savez` writes a zip and zip entries embed a wall-clock timestamp, so re-running the
generator produces a byte-different file with identical contents. Hashing canonical JSON
plus raw array bytes is both achievable and the stronger property.

`provenance` is excluded from the hash precisely because it holds a timestamp and library
versions — otherwise re-running next month would "fail" a gate it actually passes.
"""

from __future__ import annotations

import hashlib
import json
from typing import Any, Mapping

import numpy as np

#: Excluded from the content hash (SCHEMA.md §6.2).
UNHASHED_KEYS = ("provenance",)


def canonical_json(obj: Any) -> str:
    """Deterministic JSON text.

    `sort_keys` makes key order irrelevant; `allow_nan=False` rejects the non-standard
    NaN/Infinity literals the schema forbids. Python 3's ``float.__repr__`` already emits
    the shortest round-tripping representation, so floats survive a write/read cycle
    exactly.
    """
    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def content_hash(scene: Mapping[str, Any], arrays: Mapping[str, np.ndarray]) -> str:
    """SHA-256 over canonical scene JSON + every array, in a documented order.

    Args:
        scene: the ``scene.json`` mapping. ``provenance`` is dropped before hashing.
        arrays: ``{"cloud.npz:xyz": arr, "seams.npz:seam_0": arr, ...}``. Keys are
            hashed in sorted order, each with its dtype and shape, so a silent dtype
            change cannot slip past.
    """
    hashable = {k: v for k, v in scene.items() if k not in UNHASHED_KEYS}
    h = hashlib.sha256()
    h.update(canonical_json(hashable).encode("utf-8"))
    for name in sorted(arrays):
        arr = np.ascontiguousarray(arrays[name])
        h.update(name.encode("utf-8"))
        h.update(str(arr.dtype).encode("utf-8"))
        h.update(str(arr.shape).encode("utf-8"))
        h.update(arr.tobytes())
    return h.hexdigest()


def config_id(config: Mapping[str, Any]) -> str:
    """First 8 hex chars of the canonical config hash. Part of `scene_id`."""
    return hashlib.sha256(canonical_json(config).encode("utf-8")).hexdigest()[:8]


def twin_key(geometry_config: Mapping[str, Any], seed: int) -> str:
    """16 hex chars identifying an ablation twin set — SCHEMA.md §6.4.

    Computed from the config subset consumed by substreams 0-2 plus the seed, so scenes
    sharing a `twin_key` have bit-identical workpiece geometry and seam truth and differ
    only in placement, sampling, sensor or tier.
    """
    h = hashlib.sha256()
    h.update(canonical_json(geometry_config).encode("utf-8"))
    h.update(int(seed).to_bytes(8, "big"))
    return h.hexdigest()[:16]
