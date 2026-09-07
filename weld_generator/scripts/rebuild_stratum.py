#!/usr/bin/env python3
"""Regenerate ONE source stratum of a `make_bench6b.py` corpus in place.

Written for the 2026-09-07 grooved-butt fix: `configs/grooved_butt.yaml` gained the D28
outline keys (geometry keys -> new config_id -> new scene ids), so the `butt/line_grooved`
stratum of `out/bench_phase4` must be regenerated while every other stratum stays
byte-identical. The old stratum's scene ids are saved next to the manifest so the
Phase 4 batch rows computed on them can be labelled as the pre-fix stratum (the user
ruled the methods are NOT re-run on it: the groove, which is what defeats them, did not
change - only the plate outline).

Same seed policy as the builder: the source walks its own residues modulo the class
stride from BASE_SEED, rejected seeds recorded never backfilled, source predicate
applied. Refuses to run if the class index does not contain the source.

Usage:
    python scripts/rebuild_stratum.py out/bench_phase4 butt line_grooved
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from make_bench6b import ATTEMPT_CAP, STRATA, _curved, _plate, _seeds  # noqa: E402
from weldgen.scene import SceneRejected  # noqa: E402
from weldgen.writer import write_scene  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corpus")
    ap.add_argument("joint_type")
    ap.add_argument("source")
    args = ap.parse_args()
    root = Path(args.corpus)
    class_dir = root / args.joint_type
    manifest = json.loads((root / "manifest.json").read_text())
    per_family = manifest.get("per_family")
    sources = STRATA[args.joint_type]
    stride = sum(len(res) for _, _, res, _, _ in sources)
    entry = next((s for s in sources if s[0] == args.source), None)
    if entry is None:
        sys.exit(f"no source {args.source!r} in class {args.joint_type!r}")
    name, loader, residues, target, keep = entry
    if per_family is not None:
        target = per_family

    rows = [json.loads(ln) for ln in (class_dir / "index.jsonl").read_text().splitlines()]
    old = [r for r in rows if r["source"] == name]
    if not old:
        sys.exit(f"index has no rows for source {name!r} - refusing")
    old_ids = [r["scene_id"] for r in old if r.get("emitted")]
    (root / f"retired_{args.joint_type}_{name}_scene_ids.json").write_text(
        json.dumps({"retired_at": time.strftime("%Y-%m-%d"), "source": name,
                    "joint_type": args.joint_type, "scene_ids": old_ids}, indent=2))
    for sid in old_ids:
        d = class_dir / sid
        if d.exists():
            shutil.rmtree(d)
    print(f"retired {len(old_ids)} scenes of {args.joint_type}/{name}", flush=True)

    cfg, gen = (_plate(loader[1]) if loader[0] == "plate" else _curved(loader[1]))
    new_rows, emitted, attempts, t0 = [], 0, 0, time.time()
    for seed in _seeds(residues, stride):
        if emitted >= target or attempts >= ATTEMPT_CAP * target:
            break
        attempts += 1
        row = {"seed": seed, "source": name, "emitted": False}
        try:
            scene, arrays = gen(cfg, seed)
        except SceneRejected as e:
            row["reason"] = type(e).__name__
        else:
            if keep is not None and not keep(scene):
                row["reason"] = "filtered_by_source_predicate"
            else:
                write_scene(class_dir, scene, arrays)
                row.update(emitted=True, scene_id=scene["scene_id"])
                emitted += 1
        new_rows.append(row)
    kept_rows = [r for r in rows if r["source"] != name] + new_rows
    with open(class_dir / "index.jsonl", "w") as f:
        for r in kept_rows:
            f.write(json.dumps(r) + "\n")
    manifest["classes"][args.joint_type]["sources"][name] = {
        "config": list(loader), "residues": residues, "target": target,
        "emitted": emitted, "attempts": attempts,
        "seconds": round(time.time() - t0, 1),
        "rebuilt": time.strftime("%Y-%m-%d")}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2))
    status = "OK" if emitted >= target else "SHORT"
    print(f"[{args.joint_type}/{name}] {emitted}/{target} in {attempts} attempts "
          f"({time.time() - t0:.0f}s) {status}", flush=True)
    return 0 if emitted >= target else 1


if __name__ == "__main__":
    sys.exit(main())
