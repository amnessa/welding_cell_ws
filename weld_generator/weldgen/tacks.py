"""Phase 7: tack placement as a VERSIONED RULE — the D8 pattern, never stored truth.

    tackrule-0.1 (D38):
        tack_len(t) = clip(4t, 10, 50) mm
        d_max(t)    = min(33 t, 400) mm         spacing ceiling
        d_min(t)    = 10 t mm                   spacing floor
        margin(t)   = max(2t, tack_len(t))      end margin, open seams only

        open seam:   L_eff = L - 2 margin; if L_eff <= tack_len -> ONE tack at L/2
                     (degenerate, recorded); else n = max(2, ceil(L_eff/d_max) + 1),
                     lowered toward the d_min floor but never below the two
                     effective-end tacks; tacks evenly over [margin, L - margin].
                     Weld order: ends first, then recursive interval bisection.
        closed seam: n = max(4, ceil(L/d_max)) rounded UP to even; d_min lowers n but
                     never below 4; positions phase-shifted (D39). Weld order:
                     opposite pairs, pairs in bisection order over the half-loop.

    `t` is the THINNER member (the same reading ISO 5817 uses everywhere else here).
    Tacks are SHORT WELDS, not points: each carries `tack_length_mm`, and the margin
    exists so no tack sits in the run-on/run-off zone — this is also where the plan's
    "no tack within ~2t of a free edge" constraint lives now. The old "both endpoints
    mandatory" constraint survives as: both EFFECTIVE endpoints (at the margin) are
    always tacked on a non-degenerate open seam; `d_min` never forces below two.

    D39 — the closed-loop label problem: any rotation of a closed tack set is equally
    valid, the D19 kind of definitional ambiguity. Convention: the phase is derived
    from sha256(scene_id, seam id) — deterministic, seed-free, recorded per seam in
    `params.phase_by_seam`, and it applies retroactively to any stored scene. Phase 4
    tack scoring must therefore be ROTATION-INVARIANT for closed seams, or a correct
    set starting at 3 o'clock scores as wrong.

    Multi-seam scenes (D38): same-class primary seams are staggered BY SEQUENCE — the
    global `order` interleaves them round-robin (A0, B0, A1, B1, ...), which is the
    heat-balance move. A positional half-pitch offset was considered and REJECTED:
    with both effective endpoints mandatory it breaks the `d_max` bound the rule
    guarantees, and the two constraints cannot both hold.

    Provenance of the constants, stated plainly: these are SHOP-PRACTICE conventions,
    not code limits. The thickness scaling and the 400 mm ceiling are attributed to
    JASS 6 via the Kobelco handbook; the 4t/50 mm tack length is attributed to
    EN 1011-2 (NOT verified against the standard's text — we do not hold it); Tomków,
    Sobota & Krajewski 2020 is the literature anchor named by the plan. Every constant
    is a parameter of the rule, so nobody argues with 33 — they re-run it.

    Which seams: PRIMARY (weldable AND matches_joint_type) — you tack what you weld.
"""

from __future__ import annotations

import hashlib
import math
from typing import Any, Mapping

import numpy as np

RULE_VERSION = "tackrule-0.1"

DEFAULT_PARAMS: dict[str, float] = {
    "tack_len_per_t": 4.0, "tack_len_min_mm": 10.0, "tack_len_max_mm": 50.0,
    "k_max": 33.0, "d_abs_mm": 400.0, "k_min": 10.0, "margin_per_t": 2.0,
}


def _bisect_order(n: int) -> list[int]:
    """Ends first, then recursive interval bisection: n=5 -> [0, 4, 2, 1, 3]."""
    if n == 1:
        return [0]
    out = [0, n - 1]
    queue = [(0, n - 1)]
    while queue:
        lo, hi = queue.pop(0)
        if hi - lo < 2:
            continue
        mid = (lo + hi) // 2
        out.append(mid)
        queue += [(lo, mid), (mid, hi)]
    return out


def _phase(scene_id: str, seam_id: int) -> float:
    """D39: deterministic, seed-free rotation of a closed tack set, in [0, 1)."""
    h = hashlib.sha256(f"{scene_id}:{seam_id}".encode()).digest()
    return int.from_bytes(h[:8], "big") / 2.0 ** 64


def _interp(poly: np.ndarray, closed: bool, s_query: np.ndarray) -> np.ndarray:
    """Points at arclengths along a stored polyline (closed: wrapping)."""
    pts = np.asarray(poly, dtype=float)
    if closed:
        pts = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.mod(s_query, cum[-1]) if closed else np.clip(s_query, 0.0, cum[-1])
    out = np.empty((len(s), 3))
    for k in range(3):
        out[:, k] = np.interp(s, cum, pts[:, k])
    return out


def _counts_open(L: float, t: float, p: Mapping[str, float]):
    tack_len = float(np.clip(p["tack_len_per_t"] * t,
                             p["tack_len_min_mm"], p["tack_len_max_mm"]))
    margin = max(p["margin_per_t"] * t, tack_len)
    d_max = min(p["k_max"] * t, p["d_abs_mm"])
    d_min = p["k_min"] * t
    L_eff = L - 2.0 * margin
    if L_eff <= tack_len:                              # degenerate: one tack at L/2
        return tack_len, [0.5 * L]
    n = max(2, math.ceil(L_eff / d_max) + 1)
    if n > 2 and L_eff / (n - 1) < d_min:
        n = max(2, math.floor(L_eff / d_min) + 1)
    return tack_len, [margin + k * L_eff / (n - 1) for k in range(n)]


def _counts_closed(L: float, t: float, phase: float, p: Mapping[str, float]):
    tack_len = float(np.clip(p["tack_len_per_t"] * t,
                             p["tack_len_min_mm"], p["tack_len_max_mm"]))
    d_max = min(p["k_max"] * t, p["d_abs_mm"])
    d_min = p["k_min"] * t
    n = max(4, math.ceil(L / d_max))
    n += n % 2
    if L / n < d_min:
        n = max(4, 2 * math.floor(L / (2.0 * d_min)))
    s0 = phase * L
    return tack_len, [(s0 + k * L / n) % L for k in range(n)]


def tack_rule(scene: Mapping[str, Any], arrays: Mapping[str, np.ndarray],
              params: Mapping[str, float] | None = None) -> dict[str, Any]:
    """The `tacks` block for one scene — a pure function of (scene.json, seams.npz)."""
    p = {**DEFAULT_PARAMS, **(params or {})}
    thick = [o["thickness_mm"] for o in scene["objects"]
             if o.get("role", "workpiece") == "workpiece"]
    t = float(min(thick))

    per_seam: list[dict] = []                          # one entry per primary seam
    phase_by_seam: dict[str, float] = {}
    for s in scene["seams"]:
        if not (s.get("weldable") and s.get("matches_joint_type", True)):
            continue
        closed = bool(s.get("closed"))
        L = float(s["length_mm"])
        poly = arrays[f'seams.npz:{s["sampled"]["array"]}']
        if closed:
            ph = _phase(str(scene["scene_id"]), int(s["id"]))
            phase_by_seam[str(s["id"])] = ph
            tack_len, ss = _counts_closed(L, t, ph, p)
            half = len(ss) // 2
            weld_seq = [j for i in _bisect_order(half) for j in (i, i + half)]
        else:
            tack_len, ss = _counts_open(L, t, p)
            weld_seq = _bisect_order(len(ss))
        pts = _interp(poly, closed, np.asarray(ss, dtype=float))
        per_seam.append({"seam": s, "s": ss, "pts": pts, "len": tack_len,
                         "seq": weld_seq})

    # stagger BY SEQUENCE: same-class groups interleave round-robin (D38)
    groups: dict[str, list[dict]] = {}
    for e in per_seam:
        groups.setdefault(e["seam"]["seam_class"], []).append(e)
    scene_seq: list[tuple[int, int]] = []              # (per_seam index, tack index)
    for cls in sorted(groups):
        g = groups[cls]
        for round_i in range(max(len(e["seq"]) for e in g)):
            for e in g:
                if round_i < len(e["seq"]):
                    scene_seq.append((per_seam.index(e), e["seq"][round_i]))

    seam_id, arclength, points, lengths = [], [], [], []
    order = [0] * len(scene_seq)
    flat: dict[tuple[int, int], int] = {}
    for e_i, e in enumerate(per_seam):
        for k in range(len(e["s"])):
            flat[(e_i, k)] = len(seam_id)
            seam_id.append(int(e["seam"]["id"]))
            arclength.append(float(e["s"][k]))
            points.append([float(v) for v in e["pts"][k]])
            lengths.append(float(e["len"]))
    for rank, key in enumerate(scene_seq):
        order[flat[key]] = rank

    return {
        "rule_version": RULE_VERSION,
        "params": {**{k: float(v) for k, v in p.items()},
                   "t_mm": t, "phase_by_seam": phase_by_seam},
        "points_mm": points,
        "seam_id": seam_id,
        "arclength_mm": arclength,
        "tack_length_mm": lengths,
        "order": order,
    }
