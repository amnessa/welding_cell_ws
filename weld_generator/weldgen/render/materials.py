"""Placeholder alloy palette - `materials-0.1-placeholder`. M5 replaces this with real PBR
materials and surface maps; what must survive is the DRAW ORDER (alloy, then one surface
condition per part) and the seven names, which are the dataset's material vocabulary
(user, 2026-09-11: mild steel, stainless steel, aluminium, cast iron, bronze, brass, titanium).
"""

from __future__ import annotations

#: alloy -> (base colour, metallic, roughness)
ALLOYS = {
    "mild_steel":      ((0.42, 0.42, 0.44), 1.0, 0.45),
    "stainless_steel": ((0.75, 0.76, 0.78), 1.0, 0.25),
    "aluminium":       ((0.85, 0.86, 0.88), 1.0, 0.35),
    "cast_iron":       ((0.28, 0.28, 0.29), 0.9, 0.70),
    "bronze":          ((0.60, 0.40, 0.22), 1.0, 0.40),
    "brass":           ((0.85, 0.65, 0.25), 1.0, 0.30),
    "titanium":        ((0.55, 0.53, 0.50), 1.0, 0.40),
}

#: surface condition -> (albedo multiplier, roughness delta)
SURFACE = {
    "mill_scale": ((0.55, 0.52, 0.50), +0.35),
    "ground":     ((1.00, 1.00, 1.00), -0.10),
    "rusted":     ((0.80, 0.50, 0.35), +0.40),
    "primed":     ((0.70, 0.45, 0.40), +0.20),
    "oily":       ((0.95, 0.95, 0.95), -0.20),
}


def pbr_spec(alloy: str, surface: str) -> dict:
    base, metallic, rough = ALLOYS[alloy]
    mul, drough = SURFACE[surface]
    return {"base": tuple(min(1.0, b * m) for b, m in zip(base, mul)),
            "metallic": metallic if surface not in ("rusted", "primed") else 0.3,
            "roughness": float(min(0.95, max(0.05, rough + drough)))}
