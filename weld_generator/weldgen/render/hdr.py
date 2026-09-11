"""Radiance .hdr (RGBE) reader and the exposure key - pure numpy.

Poly Haven HDRIs are absolute radiance captured at wildly different levels (a dim workshop
vs a skylit hangar), so a dome at a fixed intensity renders one black and the next clipped.
The dataset normalises every HDRI to a common **log-average luminance** (Reinhard et al.
2002, "the key of the scene") and only then applies the per-scene exposure draw. Both the
per-file statistic and the target are recorded, so the choice is a stated rule, not a knob.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


def read_hdr(path: str | Path) -> np.ndarray:
    """Float32 (H, W, 3) linear RGB from a Radiance RGBE file (new-style RLE or flat)."""
    data = Path(path).read_bytes()
    pos = data.index(b"\n\n") + 2
    header, rest = data[:pos].decode("latin-1"), data[pos:]
    nl = rest.index(b"\n"); dims = rest[:nl].decode().split(); rest = rest[nl + 1:]
    H, W = int(dims[1]), int(dims[3])
    buf = np.frombuffer(rest, dtype=np.uint8)
    out = np.empty((H, W, 4), dtype=np.uint8)
    p = 0
    for y in range(H):
        if buf[p] == 2 and buf[p + 1] == 2 and (int(buf[p + 2]) << 8 | int(buf[p + 3])) == W:
            p += 4
            for c in range(4):
                x = 0
                while x < W:
                    n = int(buf[p]); p += 1
                    if n > 128:
                        n -= 128; out[y, x:x + n, c] = buf[p]; p += 1
                    else:
                        out[y, x:x + n, c] = buf[p:p + n]; p += n
                    x += n
        else:                                              # flat scanline
            out[y] = buf[p:p + 4 * W].reshape(W, 4); p += 4 * W
    e = out[..., 3].astype(np.int32)
    scale = np.where(e > 0, np.ldexp(1.0, e - 136), 0.0).astype(np.float32)   # 2^(e-128) / 256
    return out[..., :3].astype(np.float32) * scale[..., None]


def luminance(rgb: np.ndarray) -> np.ndarray:
    return 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]


def hdr_stats(path: str | Path, delta: float = 1e-4) -> dict:
    """Mean luminance, log-average luminance (Reinhard's key) and the 99.9th percentile,
    equal-area weighted over the equirectangular map (rows weighted by cos(latitude))."""
    img = read_hdr(path); L = luminance(img)
    H = L.shape[0]; w = np.cos((np.arange(H) + 0.5) / H * np.pi - np.pi / 2)[:, None]
    w = np.broadcast_to(w, L.shape); w = w / w.sum()
    return {"mean_luminance": float((L * w).sum()),
            "log_mean_luminance": float(np.exp((np.log(L + delta) * w).sum())),
            "p999_luminance": float(np.percentile(L, 99.9)), "shape": [int(L.shape[0]), int(L.shape[1])]}


def dome_intensity(log_mean_luminance: float, target_key: float, exposure: float = 1.0,
                   clamp: tuple[float, float] = (0.02, 50.0)) -> float:
    """Dome intensity that brings the HDRI's key to `target_key`, times the exposure draw."""
    return float(np.clip(target_key / max(log_mean_luminance, 1e-6), *clamp) * exposure)


def write_hdr_flat(path: str | Path, rgb: np.ndarray) -> None:
    """Write linear RGB as an UNCOMPRESSED Radiance RGBE file (flat scanlines, no RLE)."""
    rgb = np.asarray(rgb, dtype=np.float32); H, W = rgb.shape[:2]
    m = rgb.max(axis=-1)
    e = np.zeros(m.shape, dtype=np.int32); mant = np.zeros(m.shape, dtype=np.float32)
    nz = m > 1e-32
    mant[nz], e[nz] = np.frexp(m[nz])
    scale = np.where(nz, mant / np.where(nz, m, 1.0) * 256.0, 0.0).astype(np.float32)
    rgbe = np.zeros((H, W, 4), dtype=np.uint8)
    rgbe[..., :3] = np.clip(rgb * scale[..., None], 0, 255).astype(np.uint8)
    rgbe[..., 3] = np.where(nz, e + 128, 0).astype(np.uint8)
    header = b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n" + f"-Y {H} +X {W}\n".encode()
    Path(path).write_bytes(header + rgbe.tobytes())


def dome_intensity_for(dome_draw: dict, lighting_cfg: dict, hdri_by_name: dict) -> float:
    """The lighting rule from the render config, for one dome draw (HDRI or LDR panorama)."""
    if dome_draw.get("kind") == "hdri":
        h = hdri_by_name[dome_draw["name"]]
        return float(lighting_cfg["dome_base"]) * float(lighting_cfg["dome_target_mean_luminance"]) \
            / max(float(h["mean_luminance"]), 1e-6) * float(dome_draw["exposure"])
    return float(lighting_cfg.get("dome_base_ldr", 900.0)) * float(dome_draw["exposure"])
