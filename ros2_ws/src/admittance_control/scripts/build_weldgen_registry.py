#!/usr/bin/env python3
"""Build / verify `models/weldgen_objects.json` - mode A's part registry.

    python scripts/build_weldgen_registry.py                 # derive from models/*.ply
    python scripts/build_weldgen_registry.py --verify        # also check every entry
                                                             #   against its mesh,
                                                             #   both ways
Box meshes become slabs, pipes tubes (flat, mitred or saddle-cut base), rounded-rect
tubes closed swept_slabs - all fitted and verified (admittance_control/weldgen_registry.py).
Anything else is written as unsupported with the reason; such a part can be added by
hand (mark it "hand_edited": true) and re-run with --verify.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from admittance_control.weldgen_registry import (  # noqa: E402
    build_registry, load_registry, save_registry, verify_both_ways, _load_mesh)


def k_kind(x):
    """A nested param (a cut, a spine) shown by its kind only."""
    return x.get("kind", "...") if isinstance(x, dict) else x


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--models-dir", default=str(HERE.parent / "models"))
    ap.add_argument("--out", default=None, help="default: <models-dir>/weldgen_objects.json")
    ap.add_argument("--weldgen-path", default="/workspaces/welding_cell_ws/weld_generator")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--tol-mm", type=float, default=0.05, help="fit tolerance")
    ap.add_argument("--accept-mm", type=float, default=0.25,
                    help="verification budget, both directions (the D34 chord budget)")
    args = ap.parse_args()
    out = Path(args.out) if args.out else Path(args.models_dir) / "weldgen_objects.json"
    existing = load_registry(out) if out.exists() else None
    sys.path.insert(0, args.weldgen_path)               # the curved fits need weldgen
    import weldgen  # noqa: F401
    reg = build_registry(args.models_dir, existing, args.tol_mm)
    if args.verify:
        for name, e in reg["parts"].items():
            if e.get("primitive"):
                ver = verify_both_ways(
                    e, _load_mesh(Path(args.models_dir) / e["source"]), weldgen)
                e["verified"] = ver
                e["verified_max_dev_mm"] = max(ver["cad_to_prim_mm"], ver["prim_to_cad_mm"])
                if e["verified_max_dev_mm"] > args.accept_mm or ver["volume_rel_err"] > 0.01:
                    e["verify_failed"] = True
    save_registry(reg, out)
    ok = [n for n, e in reg["parts"].items() if e.get("primitive")]
    bad = {n: e.get("reason") for n, e in reg["parts"].items() if not e.get("primitive")}
    for n in ok:
        e = reg["parts"][n]
        extra = ""
        if "verified" in e:
            v = e["verified"]
            extra = (f"  verified {v['cad_to_prim_mm']:.3f} / {v['prim_to_cad_mm']:.3f} mm,"
                     f" vol {100 * v['volume_rel_err']:.2f} %"
                     + ("  FAILED" if e.get("verify_failed") else ""))
        what = e.get("dims_mm") or {k: (round(x, 3) if isinstance(x, float) else k_kind(x))
                                     for k, x in (e.get("params") or {}).items()}
        print(f"  {n:24s} {e['primitive']:10s} {what}{extra}")
    for n, r in bad.items():
        print(f"  {n:24s} UNSUPPORTED - {r}")
    print(f"wrote {out}: {len(ok)} supported, {len(bad)} unsupported")
    return 0


if __name__ == "__main__":
    sys.exit(main())
