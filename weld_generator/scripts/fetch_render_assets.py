"""Fetch the CC0 render assets Phase 8 M5 uses, with provenance - into out/render_assets/.

    python scripts/fetch_render_assets.py [--root out/render_assets] [--res 2k]

Everything here is CC0 1.0 (Poly Haven HDRIs, ambientCG PBR surface sets), chosen from the
two sites' public APIs by name so the choice is reproducible and citable rather than tuned
by eye. The manifest records source URL, licence, md5/sha256 and size per asset, and a
`set_hash` over the asset digests that enters `render_id`. Idempotent: existing files with
the right hash are kept.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import pathlib
import sys
import urllib.request
import zipfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from weldgen.render.hdr import hdr_stats  # noqa: E402

#: Poly Haven HDRIs: indoor workshops, machine shops, garages, hangars (what a welding cell sees).
HDRIS = ["machine_shop_01", "machine_shop_02", "machine_shop_03", "industrial_workshop_foundry",
         "small_workshop", "workshop", "university_workshop", "empty_workshop", "autoshop_01",
         "auto_service", "garage", "skylit_garage", "aerodynamics_workshop", "aircraft_workshop_01",
         "boiler_room", "carpentry_shop_01", "carpentry_shop_02", "small_hangar_01", "hangar_interior",
         "empty_warehouse_01", "old_depot", "industrial_pipe_and_valve_01", "vintage_measuring_lab",
         "distribution_board"]

#: ambientCG surface sets per surface condition (the dataset's vocabulary, dataset_plan.md Phase 8).
TEXTURES = {
    "ground":     ["Metal009", "Metal011", "Metal012", "Metal038", "Metal003", "Metal050C", "Metal049A"],
    "mill_scale": ["Metal046A", "Metal046B", "Metal063", "Metal055A", "Metal052C", "Metal041A"],
    "rusted":     ["Rust006", "Rust007", "Rust008", "Rust009", "Rust010", "Metal041B", "Metal041C",
                   "Metal053B", "Metal053C", "Metal056C"],
    "primed":     ["Metal027", "Metal028", "Metal029", "PaintedMetal010", "PaintedMetal012", "PaintedMetal014"],
    # `oily` reuses the ground sets with a roughness scale < 1 (a film, not a different surface)
}


def _get(url: str, timeout: int = 120) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "weldgen-phase8/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def _sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def fetch_hdris(root: pathlib.Path, res: str) -> list[dict]:
    out = []
    d = root / "hdris"; d.mkdir(parents=True, exist_ok=True)
    for name in HDRIS:
        info = json.loads(_get(f"https://api.polyhaven.com/files/{name}"))["hdri"][res]["hdr"]
        path = d / f"{name}_{res}.hdr"
        if not path.exists() or hashlib.md5(path.read_bytes()).hexdigest() != info["md5"]:
            data = _get(info["url"])
            assert hashlib.md5(data).hexdigest() == info["md5"], name
            path.write_bytes(data)
            print(f"  hdri {name}: {len(data) / 1e6:.1f} MB", flush=True)
        out.append({"name": name, "file": str(path.relative_to(root)), "source": "polyhaven.com", "license": "CC0 1.0",
                    "url": info["url"], "md5": info["md5"], "sha256": _sha(path.read_bytes()), "size": path.stat().st_size,
                    "resolution": res, **hdr_stats(path)})     # mean/log-mean/p999 luminance: the exposure rule's input
    return out


def fetch_textures(root: pathlib.Path, res: str = "1K") -> list[dict]:
    out = []
    d = root / "textures"; d.mkdir(parents=True, exist_ok=True)
    ids = [a for lst in TEXTURES.values() for a in lst]
    meta = json.loads(_get("https://ambientcg.com/api/v2/full_json?id=" + ",".join(ids) +
                           "&include=downloadData,dimensionsData,tagData"))["foundAssets"]
    by_id = {a["assetId"]: a for a in meta}
    for cond, lst in TEXTURES.items():
        for aid in lst:
            a = by_id[aid]
            dl = next(x for x in a["downloadFolders"]["default"]["downloadFiletypeCategories"]["zip"]["downloads"]
                      if x["attribute"] == f"{res}-JPG")
            tdir = d / aid; tdir.mkdir(exist_ok=True)
            stamp = tdir / "source.json"
            if not stamp.exists() or json.loads(stamp.read_text()).get("url") != dl["downloadLink"]:
                data = _get(dl["downloadLink"])
                with zipfile.ZipFile(io.BytesIO(data)) as z:
                    z.extractall(tdir)
                stamp.write_text(json.dumps({"url": dl["downloadLink"], "sha256_zip": _sha(data), "size": len(data)}))
                print(f"  texture {aid} ({cond}): {len(data) / 1e6:.1f} MB", flush=True)
            src = json.loads(stamp.read_text())
            files = {}
            for f in sorted(tdir.glob("*.jpg")):
                key = f.stem.split("_")[-1].lower()          # Color, Roughness, NormalGL, Metalness, Displacement
                files[key] = str(f.relative_to(root))
            dim = a.get("dimensionX") or 0
            out.append({"asset_id": aid, "condition": cond, "dir": str(tdir.relative_to(root)), "files": files,
                        "source": "ambientcg.com", "license": "CC0 1.0", "url": src["url"], "sha256_zip": src["sha256_zip"],
                        "size": src["size"], "tile_mm": float(dim) * 10.0 if dim else 1000.0,   # ambientCG dims are cm
                        "tags": a.get("tags", [])[:10]})
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(ROOT / "out" / "render_assets"))
    ap.add_argument("--res", default="2k", help="HDRI resolution (1k/2k/4k)")
    args = ap.parse_args()
    root = pathlib.Path(args.root); root.mkdir(parents=True, exist_ok=True)
    print("== HDRIs", flush=True); hdris = fetch_hdris(root, args.res)
    print("== textures", flush=True); tex = fetch_textures(root)
    digests = [h["sha256"] for h in hdris] + [t["sha256_zip"] for t in tex]
    manifest = {"version": "render_assets-2026-09-11", "license": {"polyhaven.com": "CC0 1.0", "ambientcg.com": "CC0 1.0"},
                "hdris": hdris, "textures": tex,
                "conditions": {**{c: lst for c, lst in TEXTURES.items()}, "oily": TEXTURES["ground"]},
                "set_hash": hashlib.sha256(json.dumps(sorted(digests)).encode()).hexdigest()[:16]}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(f"manifest: {len(hdris)} HDRIs, {len(tex)} surface sets, set_hash {manifest['set_hash']}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
