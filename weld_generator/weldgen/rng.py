"""Seeded RNG substreams — SCHEMA.md §6.1.

One seed per scene, spawned into eight **named** substreams in a fixed order.

The split is not arbitrary: substreams 0-2 determine workpiece geometry and seam truth,
substreams 3-6 determine everything that is an ablation axis. That is what makes
`twin_key` (SCHEMA.md §6.4) well-defined, and it is why fixture presence lives in
`placement` (3) rather than in `joint_config` (0).

Rules, which the determinism gate depends on:
  * Substreams are APPENDED, never reordered.
  * Within a substream, draws are APPENDED, never reordered.
  * No `random`, no `np.random` global. Only explicit Generator objects.
"""

from __future__ import annotations

import numpy as np

#: Fixed substream order. Index is the spawn key — never reorder, only append.
SUBSTREAMS = (
    "joint_config",    # 0 - joint type, part dims, thicknesses, included angle
    "defects",         # 1 - root gap, linear/angular misalignment, quality level
    "seam_curve",      # 2 - seam shape, length, curvature / control points
    "placement",       # 3 - assembly world pose, fixture presence/pose/dims, contact_mode
    "surface_sample",  # 4 - point sampling, density
    "camera",          # 5 - sensor profile, camera pose, intrinsics jitter
    "noise",           # 6 - depth-noise realization
    "_reserved7",      # 7
)

#: Substreams that determine geometry + seam truth (SCHEMA.md §6.4).
GEOMETRY_SUBSTREAMS = SUBSTREAMS[:3]


class Streams:
    """Eight named `numpy.random.Generator`s derived from one scene seed."""

    __slots__ = ("seed", "_gens")

    def __init__(self, seed: int):
        self.seed = int(seed)
        children = np.random.SeedSequence(self.seed).spawn(len(SUBSTREAMS))
        self._gens = {
            name: np.random.Generator(np.random.PCG64(child))
            for name, child in zip(SUBSTREAMS, children)
        }

    def __getitem__(self, name: str) -> np.random.Generator:
        try:
            return self._gens[name]
        except KeyError:
            raise KeyError(
                f"unknown substream {name!r}; expected one of {SUBSTREAMS}"
            ) from None

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Streams(seed={self.seed})"
