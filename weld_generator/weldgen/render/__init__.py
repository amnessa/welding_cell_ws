"""Tier-2 rendering — Phase 8 (`notes/phase8_plan.md`, `dataset_plan.md` Phase 8).

Layering, so D9 holds (tier 1 never imports a simulator):

    conventions.py   pure numpy: camera/units conventions, depth codec, labels
    gate.py          pure numpy/trimesh: the twin gate (residual, coverage, object ids)
    usd_stage.py     builds the USD scene - imports `pxr` INSIDE functions
    replicator.py    render products + annotators - imports `omni` INSIDE functions

Nothing in this package imports Isaac Sim at module import time. The two Isaac-facing
modules can only be *called* after `isaacsim.SimulationApp` exists, which the scripts
under `scripts/` do; the tests exercise the pure modules under plain pytest and skip the
Isaac ones unless `isaacsim` is importable.
"""
