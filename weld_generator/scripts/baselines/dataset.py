"""Reading generated scenes back for evaluation — the harness side of Phase 4.

Deliberately thin. It answers three questions a baseline run has to get right, and each of
them is a decision rather than plumbing:

  * **which points does the method see** — the full surface, or only what one camera
    returns, with or without sensor noise. The plan wants full-visibility and single-view
    reported *separately*, because a method tuned on 360° CAD clouds is being flattered.
  * **which seams count as truth** — primary seams only (what the joint type declares), or
    every weldable one. A seam no sensor could see is unevaluable and must be excluded from
    recall, or a baseline is scored for missing something invisible.
  * **which of the three D19 curves** — `nominal` is stored, `root` and `gap_mid` are
    derived, and they differ by O(gap). Reporting against the wrong one silently shifts
    every error by about a root gap.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

CURVES = ("nominal", "root", "gapmid")


def load_scene(scene_dir: str | Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    """`(scene.json, arrays)` for one scene directory, keyed as the content hash expects."""
    d = Path(scene_dir)
    scene = json.loads((d / "scene.json").read_text())
    arrays: dict[str, np.ndarray] = {}
    for npz in sorted(d.glob("*.npz")):
        with np.load(npz) as z:
            for k in z.files:
                arrays[f"{npz.name}:{k}"] = z[k]
    return scene, arrays


def scene_dirs(out_dir: str | Path) -> list[Path]:
    """Paths of the emitted scenes, without loading any of them.

    Holding 44 loaded scenes costs ~500 MB, which is enough to kill a notebook kernel once
    a corpus sweep's own working set is added on top. Iterate paths and load one at a time.
    """
    root = Path(out_dir)
    index = root / "index.jsonl"
    if index.exists():
        return [root / json.loads(ln)["scene_id"]
                for ln in index.read_text().splitlines()
                if json.loads(ln).get("emitted")]
    return sorted(p for p in root.iterdir() if (p / "scene.json").exists())


def balanced_corpus(root: str | Path = "out/bench", per_type: int = 50,
                    joint_types: tuple[str, ...] | None = None) -> dict[str, list[Path]]:
    """`{joint_type: [scene_dir, ...]}` — exactly `per_type` scenes of each type.

    The benchmark corpus is generated **one config per joint type** into
    `out/bench/<joint_type>/`, because `joint_type` is sampled per seed and a mixed config
    gives whatever the draw gives: `out/phase3` came out 15 butt against 2 lap, and three
    per-type conclusions drawn from it have since been overturned.

    The generated counts are not equal and cannot be made equal by asking. Yields differ by
    a factor of five across types — measured 59-81% for butt/T against **38% for lap**, the
    rest lost to `NoVisibleSeams` — and seeds are deliberately not backfilled inside a run
    (it would break twin pairing), so each type is generated with a generous budget and
    trimmed here. Selection is the `per_type` **lowest seeds**, so it is a pure function of
    the corpus and does not depend on generation order or on how much surplus a type has.

    Raises if a type is short, rather than returning an unbalanced corpus quietly - being
    silently down to 37 scenes of one type is the exact failure this function exists to stop.
    """
    root = Path(root)
    types = joint_types or tuple(sorted(p.name for p in root.iterdir() if p.is_dir()))
    out: dict[str, list[Path]] = {}
    short = {}
    for jt in types:
        rows = []
        index = root / jt / "index.jsonl"
        if not index.exists():
            short[jt] = 0
            continue
        for ln in index.read_text().splitlines():
            r = json.loads(ln)
            if r.get("emitted"):
                rows.append((int(r["seed"]), root / jt / r["scene_id"]))
        rows.sort()
        if len(rows) < per_type:
            short[jt] = len(rows)
        out[jt] = [p for _, p in rows[:per_type]]
    if short:
        raise ValueError(
            f"corpus is not balanced at {per_type} per type: {short}. "
            f"Generate more seeds for those types - `configs/bench_<type>.yaml`.")
    return out


def iter_scenes(out_dir: str | Path):
    """Every emitted scene under an output directory, in index order.

    Skipped seeds are in `index.jsonl` too (`emitted: false`) and are not yielded here -
    they have no directory. Read the index directly to characterise what was dropped.
    """
    for d in scene_dirs(out_dir):
        yield load_scene(d)


def cloud_for(scene: dict, arrays: dict, view: str = "full", noisy: bool = False
              ) -> dict[str, np.ndarray]:
    """The point set a baseline is allowed to see.

    Args:
        view: `"full_exterior"` = every point a **perfect multi-view scan** could return
            (`exterior == True`) — **the Task 1 input condition**. `"single"` = only
            `visible_from_cam` — one shot, Task 2. `"full"` = the whole surface sample
            including buried interior faces: the 360° CAD-cloud condition the literature
            evaluates on, kept for exactly that comparison — but per the plan, full
            geometry is **truth only, never a method input** in this project's own
            conditions. The interior-face false positives it produces are an artifact of
            CAD-built clouds, not of reality.
        noisy: apply the analytic stereo model. The realisation is not stored (SCHEMA.md
            §5.1), so it is recomputed here - deterministic in `noise_model.seed`.

    Returns `xyz`, `normals`, `object_id`, plus the `valid` mask from the sensor model.
    """
    xyz = arrays["cloud.npz:xyz"].astype(float)
    normals = arrays["cloud.npz:normals"].astype(float)
    object_id = arrays["cloud.npz:object_id"]
    valid = np.ones(len(xyz), dtype=bool)

    if noisy:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        from weldgen.noise import apply as apply_noise
        xyz, valid = apply_noise(xyz, normals, np.array(scene["camera"]["T_world_cam"]),
                                 scene["noise_model"])

    if view == "single":
        m = arrays["cloud.npz:visible_from_cam"]
        # Sensor dropout is deliberately NOT part of `visible_from_cam` (it is geometry,
        # SCHEMA.md §5.2), so the single-view arm has to apply both to be honest.
        m = m & valid if noisy else m
    elif view == "full_exterior":
        if "cloud.npz:exterior" not in arrays:
            raise KeyError(
                "this scene predates the `exterior` flag - run the backfill "
                "(scratchpad/backfill_exterior.py) or regenerate")
        m = arrays["cloud.npz:exterior"]
        m = m & valid if noisy else m
    elif view == "full":
        m = np.ones(len(xyz), dtype=bool)
        m = m & valid if noisy else m
    else:
        raise ValueError(f'view must be "full_exterior", "single" or "full", got {view!r}')
    return {"xyz": xyz[m], "normals": normals[m], "object_id": object_id[m],
            "valid": valid[m]}


def ground_truth(scene: dict, arrays: dict, curve: str = "nominal",
                 primary_only: bool = True, min_visible_fraction: float | None = None,
                 include_rejected: bool = False,
                 with_underside: bool = False,
                 with_meta: bool = False) -> list[np.ndarray]:
    """Truth polylines for one scene.

    Args:
        curve: one of `nominal` (stored), `root`, `gapmid` (derived) — D19, SCHEMA.md §1.3.
        primary_only: keep only seams this joint type declares as its own. An off-class
            seam is a real weld, so a method that finds it is not wrong; it just is not
            what the label promised.
        min_visible_fraction: drop seams the camera barely returns. Set it for the
            single-view arm — the lower toe of a lap joint is visible from **no** viewpoint
            above the table, so scoring recall against it charges a baseline for missing
            something no sensor could see.
        with_underside: also return the per-seam D31 `underside` flags (the structural,
            visibility-independent statement of the same lap-lower-toe fact), as
            `(curves, flags)` — the harness filters single-view scoring targets on them.
        with_meta: instead return `(curves, meta)` where meta is one dict per curve:
            `{id, closed, underside, seam_class}` — what Task 2 needs to map a matched
            curve back to the seam the MPS rule named, and what D39 needs to know before
            reporting an endpoint error (a closed ring has no endpoints).
    """
    if curve not in CURVES:
        raise ValueError(f"curve must be one of {CURVES}, got {curve!r}")
    out = []
    meta = []
    for s in scene["seams"]:
        if not s["weldable"] and not include_rejected:
            continue
        if not s["weldable"] and s.get("reject_reason") in (
                "fixture_contact", "bisector_blocked", "degenerate_dihedral", "no_contact"):
            continue                                   # not a reachable interface at all
        if primary_only and not s["matches_joint_type"]:
            continue
        if (min_visible_fraction is not None
                and s["visible_fraction"] < min_visible_fraction):
            continue
        key = s["sampled"]["array"]
        suffix = "" if curve == "nominal" else f"_{curve}"
        out.append(arrays[f"seams.npz:{key}{suffix}"].astype(float))
        meta.append({"id": int(s["id"]), "closed": bool(s.get("closed")),
                     "underside": bool(s.get("underside")),
                     "seam_class": s.get("seam_class")})
    if with_meta:
        return out, meta
    if with_underside:
        return out, [m["underside"] for m in meta]
    return out


def scene_facts(scene: dict) -> dict[str, Any]:
    """The covariates every Phase 4 plot groups by, pulled out of one scene.

    `t_min_mm`/`t_max_mm` are MEMBER GAUGES — the thinner cross-section dimension per
    part, not the raw `thickness_mm` (which for a curved-butt swept band is its ~80 mm
    in-plane band width; the tackrule-0.1 gauge bug, found 2026-09-02). `seam_family` /
    `prep` / `primitives` are what the per-family Phase 4 corpus groups by.
    """
    import sys
    from pathlib import Path as _P
    sys.path.insert(0, str(_P(__file__).resolve().parents[2]))
    from weldgen.tacks import _member_gauge_mm
    ts = [_member_gauge_mm(o) for o in scene["objects"] if o["role"] == "workpiece"]
    prim = [s for s in scene["seams"] if s["weldable"] and s["matches_joint_type"]]
    return {
        "scene_id": scene["scene_id"],
        "seed": scene["seed"],
        "joint_type": scene["joint"]["type"],
        "seam_family": scene["joint"].get("seam_family"),
        "prep": scene["joint"].get("prep"),
        "primitives": ",".join(sorted({o.get("primitive", "slab")
                                       for o in scene["objects"]
                                       if o["role"] == "workpiece"})),
        "quality_level": scene["joint"]["quality_level"],
        "included_angle_deg": scene["joint"]["included_angle_deg"],
        "root_gap_mm": scene["fit"]["root_gap_mm"],
        "t_min_mm": min(ts),
        "t_max_mm": max(ts),
        "density_per_mm2": scene["cloud"]["density_per_mm2"],
        "sampling_mode": scene["cloud"]["sampling_mode"],
        "sensor_profile": scene["noise_model"]["profile"],
        "standoff_mm": scene["camera"]["standoff_mm"],
        "elevation_deg": scene["camera"]["elevation_deg"],
        "fixture_present": any(o["role"] == "fixture" for o in scene["objects"]),
        "twin_key": scene["twin_key"],
        "n_primary": len(prim),
        "mean_occluded_fraction": float(np.mean([s["occluded_fraction"] for s in prim]))
        if prim else float("nan"),
        "best_visible_fraction": float(max(s["visible_fraction"] for s in prim))
        if prim else 0.0,
    }
