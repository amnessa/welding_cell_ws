#!/usr/bin/env python3
"""Retro-apply the versioned rule blocks (`mps`, `tacks`) to a stored corpus, in place.

This is the D8 / D25 design cashed in: both blocks are PURE functions of the stored
files (`mps_rule-0.1` of `scene.json`, `tackrule-0.1` of `scene.json` + `seams.npz`),
so a corpus generated with the flags off can be brought to the tack-complete state
without regeneration. Identity is preserved - `scene_id` (config_id + seed) and
`twin_key` do not change - and the content is what changes, so `scene.sha256` and the
index row's `content_hash` are rewritten to match (`weldgen verify` stays green).
The blocks are self-describing (`rule_version` + `params`), and the corpus manifest
records the application.

Idempotent: a scene that already carries both blocks is skipped unless `--force`.

Usage:
    python scripts/apply_rule_blocks.py out/bench_phase4 out/bench_phase4_fx
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from baselines.dataset import load_scene  # noqa: E402
from weldgen.hashing import content_hash  # noqa: E402
from weldgen.mps import RULE_VERSION as MPS_VERSION, mps_rule  # noqa: E402
from weldgen.tacks import RULE_VERSION as TACK_VERSION, tack_rule  # noqa: E402


def apply_root(root: Path, force: bool) -> dict:
    stats = {"scenes": 0, "updated": 0, "skipped": 0, "tacks_total": 0,
             "single_tack_seams": 0, "closed_seams_tacked": 0, "mps_null": 0}
    for class_dir in sorted(p for p in root.iterdir() if (p / "index.jsonl").exists()):
        rows = [json.loads(ln) for ln in (class_dir / "index.jsonl").read_text().splitlines()]
        changed = False
        for row in rows:
            if not row.get("emitted"):
                continue
            d = class_dir / row["scene_id"]
            scene, arrays = load_scene(d)
            stats["scenes"] += 1
            if scene.get("mps") and scene.get("tacks") and not force:
                stats["skipped"] += 1
                continue
            scene["mps"] = mps_rule(scene)
            scene["tacks"] = tack_rule(scene, arrays)
            (d / "scene.json").write_text(
                json.dumps(scene, indent=2, sort_keys=True, allow_nan=False))
            digest = content_hash(scene, arrays)
            (d / "scene.sha256").write_text(digest + "\n")
            row["content_hash"] = digest
            changed = True
            stats["updated"] += 1
            tb = scene["tacks"]
            stats["tacks_total"] += len(tb["seam_id"])
            if scene["mps"]["seam_id"] is None:
                stats["mps_null"] += 1
            by_seam = {}
            for sid in tb["seam_id"]:
                by_seam[sid] = by_seam.get(sid, 0) + 1
            closed = {s["id"] for s in scene["seams"] if s.get("closed")}
            for sid, n in by_seam.items():
                if n == 1:
                    stats["single_tack_seams"] += 1
                if sid in closed:
                    stats["closed_seams_tacked"] += 1
        if changed:
            with open(class_dir / "index.jsonl", "w") as f:
                for r in rows:
                    f.write(json.dumps(r) + "\n")
        print(f"  {root.name}/{class_dir.name}: {stats['scenes']} seen so far", flush=True)
    mf = root / "manifest.json"
    manifest = json.loads(mf.read_text()) if mf.exists() else {}
    manifest["rule_blocks"] = {"mps": MPS_VERSION, "tacks": TACK_VERSION,
                               "applied": time.strftime("%Y-%m-%d"),
                               "note": "retro-applied by scripts/apply_rule_blocks.py; "
                                       "scene_id/twin_key unchanged, content hashes "
                                       "rewritten"}
    mf.write_text(json.dumps(manifest, indent=2))
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("roots", nargs="+")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    for r in args.roots:
        t0 = time.time()
        st = apply_root(Path(r), args.force)
        print(f"{r}: {st}  ({time.time() - t0:.0f}s)", flush=True)


if __name__ == "__main__":
    main()
