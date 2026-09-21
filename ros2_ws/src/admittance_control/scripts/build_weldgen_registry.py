#!/usr/bin/env python3
"""Build / verify `models/weldgen_objects.json` - mode A's part registry.

    python scripts/build_weldgen_registry.py                 # derive from models/*.ply
    python scripts/build_weldgen_registry.py --verify        # also check every entry
                                                             #   against its mesh
Box meshes are derived automatically (slab + frame). Anything else is written as
unsupported with the reason; add such parts by hand (tube / swept_slab entries in the
same JSON, see admittance_control/weldgen_registry.py) and re-run with --verify.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from admittance_control.weldgen_registry import (  # noqa: E402
    build_registry, load_registry, save_registry, verify_entry, _load_mesh)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--models-dir", default=str(HERE.parent / "models"))
    ap.add_argument("--out", default=None, help="default: <models-dir>/weldgen_objects.json")
    ap.add_argument("--weldgen-path", default="/workspaces/welding_cell_ws/weld_generator")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--tol-mm", type=float, default=0.05)
    args = ap.parse_args()
    out = Path(args.out) if args.out else Path(args.models_dir) / "weldgen_objects.json"
    existing = load_registry(out) if out.exists() else None
    reg = build_registry(args.models_dir, existing, args.tol_mm)
    if args.verify:
        sys.path.insert(0, args.weldgen_path)
        import weldgen  # noqa: F401
        for name, e in reg["parts"].items():
            if e.get("primitive"):
                e["verified_max_dev_mm"] = verify_entry(
                    e, _load_mesh(Path(args.models_dir) / e["source"]), weldgen)
    save_registry(reg, out)
    ok = [n for n, e in reg["parts"].items() if e.get("primitive")]
    bad = {n: e.get("reason") for n, e in reg["parts"].items() if not e.get("primitive")}
    for n in ok:
        e = reg["parts"][n]
        extra = f"  verified {e['verified_max_dev_mm']:.3f} mm" if "verified_max_dev_mm" in e else ""
        print(f"  {n:24s} {e['primitive']:6s} {e.get('dims_mm') or e.get('params')}{extra}")
    for n, r in bad.items():
        print(f"  {n:24s} UNSUPPORTED - {r}")
    print(f"wrote {out}: {len(ok)} supported, {len(bad)} unsupported")
    return 0


if __name__ == "__main__":
    sys.exit(main())
