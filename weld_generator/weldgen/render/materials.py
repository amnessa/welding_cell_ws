"""Materials - `materials-1.0` (Phase 8 M5).

Two published sources, so nothing here is tuned by eye:

* **Alloy base reflectance (F0, linear RGB)** from Real-Time Rendering, 4th ed. (Akenine-Moller
  et al. 2018), Table 9.2 "Values of F0 for various metals" - iron, aluminium, chromium,
  titanium, brass (C260), copper. Assignments to the dataset's seven alloys: mild steel and
  cast iron -> iron; stainless -> chromium (the passive layer that sets its reflectance);
  aluminium, titanium, brass -> themselves; bronze (not tabulated) -> INTERPOLATED between
  copper and tin-darkened copper, marked as such. With `metallic = 1` a UsdPreviewSurface
  uses `diffuseColor` as the specular colour, i.e. F0.
* **Surface condition** from CC0 ambientCG surface sets (`out/render_assets/manifest.json`,
  fetched by `scripts/fetch_render_assets.py`): colour, roughness and normal maps per
  condition - brushed/scratched steel for `ground`, dark oxide for `mill_scale`, rust sets,
  powder-coat/painted sets for `primed`; `oily` reuses the ground sets with a roughness scale
  below one. The albedo map is multiplied by the alloy's F0 so the same rust set reads
  differently on brass and on steel.
"""

from __future__ import annotations

RULE = "materials-1.0"

#: RTR4 Table 9.2, linear RGB F0.
F0_TABLE = {
    "iron":      (0.562, 0.565, 0.578),
    "aluminium": (0.913, 0.922, 0.924),
    "chromium":  (0.549, 0.556, 0.554),
    "titanium":  (0.542, 0.497, 0.449),
    "brass":     (0.910, 0.778, 0.423),
    "copper":    (0.955, 0.638, 0.538),
}

#: dataset alloy -> (F0 source, F0, note)
ALLOYS = {
    "mild_steel":      ("iron",      F0_TABLE["iron"],      "RTR4 Table 9.2 iron"),
    "cast_iron":       ("iron",      F0_TABLE["iron"],      "RTR4 Table 9.2 iron; roughness from the surface set, +0.15"),
    "stainless_steel": ("chromium",  F0_TABLE["chromium"],  "RTR4 Table 9.2 chromium (passive layer)"),
    "aluminium":       ("aluminium", F0_TABLE["aluminium"], "RTR4 Table 9.2 aluminium"),
    "titanium":        ("titanium",  F0_TABLE["titanium"],  "RTR4 Table 9.2 titanium"),
    "brass":           ("brass",     F0_TABLE["brass"],     "RTR4 Table 9.2 brass (C260)"),
    "bronze":          ("copper*",   (0.80, 0.55, 0.40),    "INTERPOLATED: copper (RTR4) darkened toward tin; not tabulated"),
}

#: surface condition -> (metallic, roughness scale, roughness bias, albedo blend toward the map)
#: `albedo_blend` = how much of the map's colour survives vs the bare alloy F0 tint:
#:   bare conditions keep the alloy (map modulates), coatings are the map (paint/rust hide the alloy).
SURFACE = {
    "ground":     {"metallic": 1.0, "roughness_scale": 1.0, "roughness_bias": 0.0,  "albedo_blend": 0.35},
    "oily":       {"metallic": 1.0, "roughness_scale": 0.6, "roughness_bias": 0.0,  "albedo_blend": 0.35},
    "mill_scale": {"metallic": 0.7, "roughness_scale": 1.0, "roughness_bias": 0.15, "albedo_blend": 0.85},
    "rusted":     {"metallic": 0.2, "roughness_scale": 1.0, "roughness_bias": 0.10, "albedo_blend": 0.95},
    "primed":     {"metallic": 0.0, "roughness_scale": 1.0, "roughness_bias": 0.0,  "albedo_blend": 1.0},
}


def recipe(alloy: str, surface: str, texture: dict, roughness_jitter: float = 1.0) -> dict:
    """Everything the stage builder needs for one part's material."""
    src, f0, note = ALLOYS[alloy]
    sc = SURFACE[surface]
    rough_scale = sc["roughness_scale"] * float(roughness_jitter) + (0.15 if alloy == "cast_iron" else 0.0)
    return {"rule": RULE, "alloy": alloy, "f0_source": src, "f0": list(f0), "f0_note": note, "surface": surface,
            "texture": texture["asset_id"], "texture_dir": texture["dir"], "files": texture["files"],
            "tile_mm": float(texture.get("tile_mm", 1000.0)), "metallic": sc["metallic"],
            "roughness_scale": rough_scale, "roughness_bias": sc["roughness_bias"], "albedo_blend": sc["albedo_blend"]}
