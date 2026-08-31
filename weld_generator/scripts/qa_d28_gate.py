#!/usr/bin/env python3
"""D28 anti-shortcut gate — is the axis-alignment prior actually gone? (patch_phase6)

For every primary seam in a corpus, the LENGTH-WEIGHTED distribution of angles between
the seam and every free boundary edge of the two workpieces is collected. The gate is
corpus-level: the histogram must not concentrate at 0/90 degrees, because "find a long
straight boundary, the seam is parallel (or perpendicular) to it" must not be a valid
heuristic on the finished dataset.

Joint-CONSTRAINED edges are excluded first, per the amendment: the seam-bearing edges
(a fillet necessarily runs along B's bottom edges) and the seam-terminating end edges
(they meet the seam's endpoints) are pinned by the joint type itself and carry no
shortcut information. Both kinds live within a few mm of the seam, so the exclusion is
by nearness to the seam segment alone.

Two earlier statistics measured their own artefacts rather than the prior, and are
recorded here so they are not rebuilt: (1) "angle to the nearest edge excluding
near-AND-parallel ones" is always won by a terminating end edge (touches the endpoint,
~90 deg); (2) the same with all near edges excluded is won by whatever sits just past
the cutoff - the winners' distances clustered at 12.0-12.5 mm against a 12 mm
threshold, the signature of a metric measuring its own exclusion boundary. Weighting
every free edge by its length instead measures what a shortcut learner actually sees,
and makes the tiny vertical (thickness) corner edges count for what they are.

Curved scenes (Phase 6b step 5 ruling)
--------------------------------------
Curved corpora are handled in three parts, only the last of which is binding:

1. **Closed seams are exempt.** A closed seam's tangent sweeps every direction in its
   plane, so "parallel to a long straight boundary" is unlearnable from it. Families
   2-5 are all closed, so this removes most curved scenes from the statistic.
2. **Open curved seams get a per-point statistic, REPORTED but not gated.** The seam
   contributes its stored per-sample tangents (each weighted `edge_length / n_samples`,
   so a seam's total weight against an edge matches the plate convention). It is not
   pooled into the plate gate: families 6-7 draw their spines as random B-splines/arcs,
   so the sampler axis-alignment prior D28 polices cannot arise there by construction -
   gating would test what construction already guarantees. The histogram is printed so
   the number exists; the plate gate stays the binding one.
3. **Joint-constrained exclusion is by PROVENANCE, not proximity, for curved parts.**
   The 12 mm nearness proxy cannot catch a swept band's far edges: the longitudinal
   ones are offsets of the seam's own spine - exactly parallel at every point but a
   full band-width away - and even the far vertical corners have their positions
   pinned by the spine and read 90 deg under every possible draw. That geometry means
   "this is a stiffener / a curved butt", not "the sampler leaked the answer", and
   counting it would make the printed number say the opposite of what it means. So a
   part whose spine carries the seam's own rigid-motion-invariant signature is a
   D29-derived part and ALL its boundary edges are excluded; edges of independent
   parts (the plate a spine was drawn onto) are what the statistic keeps, with the
   nearness rule still handling bearing/terminating edges. Family 7 therefore reports
   nothing - both its parts are derived from the curve, so no sampler-free direction
   exists in the scene - and family 6 reports seam tangents against the plate outline.

The D34 chord gate is also swept here and IS binding: every scene that records
`cloud.max_chord_error_mm` must be at or under 0.25 mm.

Usage:
    python scripts/qa_d28_gate.py out/smoke [out/bench6a out/bench6b ...] [--spike-tol 2.0]

Exit code 0 iff the binding gates pass on the pooled corpus: the plate histogram's
terminal-bin mass stays below `--spike-tol` times the uniform expectation (vacuous when
the corpus holds no plate scenes), and no scene exceeds the D34 chord budget.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from weldgen.curves import from_parametric  # noqa: E402

#: D34: the mesh chord budget every emitted scene was gated on at generation time.
CHORD_MM = 0.25

#: An edge is joint-constrained - and therefore excluded before taking the nearest -
#: when its closest approach to the seam SEGMENT is under this many mm. That covers both
#: kinds the joint itself pins: the seam-BEARING edges (parallel, a thickness plus a root
#: gap away) and the seam-TERMINATING end edges (they touch the seam's endpoints, so they
#: are always the nearest edge and always ~90 deg - the first run of this gate measured
#: exactly that artefact, a 43/62 terminal-bin spike made of end edges).
NEAR_MM = 12.0


def _slab_edges(dims, T):
    L, W, t = (float(v) for v in dims[:3])
    h = np.array([L, W, t]) / 2.0
    T = np.asarray(T, dtype=float)
    out = []
    for run in range(3):
        o1, o2 = [a for a in range(3) if a != run]
        for s1 in (-1, 1):
            for s2 in (-1, 1):
                a = np.zeros(3)
                b = np.zeros(3)
                a[run], b[run] = -h[run], h[run]
                a[o1] = b[o1] = s1 * h[o1]
                a[o2] = b[o2] = s2 * h[o2]
                out.append((a, b))
    return [(p @ T[:3, :3].T + T[:3, 3], q @ T[:3, :3].T + T[:3, 3]) for p, q in out]


def _prism_edges(outline_uv, thickness, T):
    o = np.asarray(outline_uv, dtype=float)
    t2 = float(thickness) / 2.0
    T = np.asarray(T, dtype=float)
    k = len(o)
    out = []
    for i in range(k):
        a2, b2 = o[i], o[(i + 1) % k]
        for w in (-t2, t2):                       # both cap boundaries
            out.append((np.array([*a2, w]), np.array([*b2, w])))
        out.append((np.array([*a2, -t2]), np.array([*a2, t2])))   # vertical corner
    return [(p @ T[:3, :3].T + T[:3, 3], q @ T[:3, :3].T + T[:3, 3]) for p, q in out]


def _seg_seg_dist(a, b, p0, p1):
    """Min distance between segment ab and seam segment p0p1 (sampled, 17x exact
    point-to-segment - exact enough for a 12 mm threshold on plate-scale geometry)."""
    ts = np.linspace(0.0, 1.0, 17)[:, None]
    pts = a[None, :] * (1 - ts) + b[None, :] * ts
    d = p1 - p0
    L2 = float(d @ d)
    u = np.clip((pts - p0[None, :]) @ d / max(L2, 1e-12), 0.0, 1.0)
    proj = p0[None, :] + u[:, None] * d[None, :]
    return float(np.linalg.norm(pts - proj, axis=1).min())


def _apply(T, pts):
    T = np.asarray(T, dtype=float)
    return np.asarray(pts, dtype=float) @ T[:3, :3].T + T[:3, 3]


def _rigid_signature(par: dict, n: int = 8) -> tuple:
    """Pose-independent identity of a curve: sorted pairwise distances between `n`
    points at fixed arclength fractions. Equal up to rigid motion => equal signature,
    which is what makes the provenance test work across the part-local/world divide."""
    c = from_parametric(par)
    s = np.linspace(0.0, c.length_mm, n)
    pts = c.point(c.t_at_arclength(s))
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    iu = np.triu_indices(n, k=1)
    return tuple(np.round(np.sort(d[iu]), 3))


def _seam_spine_par(par: dict) -> dict:
    """The base curve a seam is built on: an offset seam's spine, else the seam."""
    return par["spine"] if par.get("kind") == "offset" else par


def _swept_end_edges(params: dict, T) -> list[tuple[np.ndarray, np.ndarray]]:
    """The 8 straight boundary edges of a swept band's two end faces (part-local
    spine + offsets/heights, posed to world). The longitudinal boundaries are offsets
    of the spine and are NOT returned - that is the provenance exclusion."""
    spine = from_parametric(params["spine"])
    lo, hi = float(params["offset_lo_mm"]), float(params["offset_hi_mm"])
    z0, z1 = float(params["z0_mm"]), float(params["z1_mm"])
    out = []
    for t_end in (0.0, spine.t_period):
        c = np.asarray(spine.point(t_end), dtype=float).reshape(3)
        tg = np.asarray(spine.tangent(t_end), dtype=float).reshape(3)
        tg = tg / max(np.linalg.norm(tg), 1e-12)
        nrm = np.cross([0.0, 0.0, 1.0], tg)          # in-plane offset direction
        nrm = nrm / max(np.linalg.norm(nrm), 1e-12)
        corner = {(o, z): c + o * nrm + np.array([0.0, 0.0, z])
                  for o in (lo, hi) for z in (z0, z1)}
        out += [(corner[(lo, z0)], corner[(lo, z1)]),   # verticals
                (corner[(hi, z0)], corner[(hi, z1)]),
                (corner[(lo, z0)], corner[(hi, z0)]),   # transverse
                (corner[(lo, z1)], corner[(hi, z1)])]
    return [(_apply(T, a), _apply(T, b)) for a, b in out]


def _seg_polyline_dist(a, b, poly: np.ndarray) -> float:
    ts = np.linspace(0.0, 1.0, 9)[:, None]
    pts = a[None, :] * (1 - ts) + b[None, :] * ts
    d = pts[:, None, :] - poly[None, :, :]
    return float(np.linalg.norm(d, axis=2).min())


def collect_curved(root: Path):
    """Per-point rows (family, angle, weight) for OPEN curved seams, plus counters.

    Kept edges are the straight boundaries of parts INDEPENDENT of the seam's curve:
    slab/prism outlines, and the end faces of a swept band with a foreign spine. A
    swept band whose spine carries the seam's own rigid signature is a D29-derived
    part and contributes nothing (provenance exclusion). A foreign-spine band's
    longitudinal edges would need chord sampling - no such part exists in the current
    families, and the `foreign_spine` counter would show one arriving.
    """
    rows = []
    st = {"scenes": 0, "closed_exempt": 0, "open_seams": 0,
          "provenance_excluded": 0, "near_excluded": 0, "foreign_spine": 0}
    for sj in sorted(root.rglob("scene.json")):
        scene = json.loads(sj.read_text())
        fam = scene["joint"].get("seam_family")
        if not fam:
            continue
        st["scenes"] += 1
        primary = [s for s in scene["seams"]
                   if s.get("weldable") and s.get("matches_joint_type", True)]
        open_seams = [s for s in primary if not s.get("closed")]
        st["closed_exempt"] += len(primary) - len(open_seams)
        if not open_seams:
            continue
        npz = np.load(sj.parent / "seams.npz")
        for s in open_seams:
            st["open_seams"] += 1
            sig = _rigid_signature(_seam_spine_par(s["parametric"]))
            pts = npz[s["sampled"]["array"]].astype(float)
            tans = npz[s["sampled"]["array"] + "_tangent"].astype(float)
            edges = []
            for o in scene["objects"]:
                if o.get("role") != "workpiece":
                    continue
                prim = o.get("primitive", "slab")
                if prim == "slab":
                    edges += _slab_edges(o["dims_mm"], o["T_world_part"])
                elif prim == "prism":
                    edges += _prism_edges(o["outline_uv"], o["thickness_mm"],
                                          o["T_world_part"])
                elif prim == "swept_slab":
                    if _rigid_signature(o["params"]["spine"]) == sig:
                        # D29 curve-first: this part was DERIVED from the seam's own
                        # curve, so every one of its boundary edges - longitudinal
                        # offsets AND end-face edges whose positions the spine pins -
                        # is joint-constrained. Even the far vertical corners only
                        # ever read 90 deg, invariant under every sampler draw.
                        st["provenance_excluded"] += 12
                    else:
                        st["foreign_spine"] += 1           # longitudinal need chords
                        edges += _swept_end_edges(o["params"], o["T_world_part"])
                # tube rims only occur with closed seams, which never reach here
            w_pt = 1.0 / max(len(pts), 1)
            for a, b in edges:
                e = b - a
                ln = float(np.linalg.norm(e))
                if ln < 1e-9:
                    continue
                if _seg_polyline_dist(a, b, pts) < NEAR_MM:
                    st["near_excluded"] += 1
                    continue          # bearing / terminating edge, same rule as plates
                e = e / ln
                cosang = np.clip(np.abs(tans @ e), 0.0, 1.0)
                angs = np.degrees(np.arccos(cosang))
                rows += [(fam, float(g), ln * w_pt) for g in angs]
    return rows, st


def chord_sweep(root: Path):
    """(n_recorded, worst, violations) for `cloud.max_chord_error_mm` - D34, binding."""
    n, worst, bad = 0, 0.0, []
    for sj in sorted(root.rglob("scene.json")):
        scene = json.loads(sj.read_text())
        v = scene.get("cloud", {}).get("max_chord_error_mm")
        if v is None:
            continue
        n += 1
        worst = max(worst, float(v))
        if float(v) > CHORD_MM:
            bad.append((scene.get("scene_id", sj.parent.name), float(v)))
    return n, worst, bad


def collect(root: Path):
    """(joint_type, mechanism, angle, edge_length) rows, one per (seam, free edge)."""
    rows = []
    for sj in sorted(root.rglob("scene.json")):
        scene = json.loads(sj.read_text())
        if scene["joint"].get("seam_family"):
            continue                  # curved pipeline: handled by collect_curved
        edges = []
        for o in scene["objects"]:
            if o.get("role") != "workpiece":
                continue
            if o.get("primitive", "slab") == "prism":
                es = _prism_edges(o["outline_uv"], o["thickness_mm"], o["T_world_part"])
            else:
                es = _slab_edges(o["dims_mm"], o["T_world_part"])
            edges += [(o["id"], a, b) for a, b in es]
        jt = scene["joint"]["type"]
        yawed = abs(float(scene["joint"].get("in_plane_yaw_deg", 0.0))) > 1e-9
        outlined = any(o.get("primitive") == "prism" for o in scene["objects"])
        mech = ("yaw+out" if yawed and outlined else "yaw" if yawed
                else "outline" if outlined else "none")
        for s in scene["seams"]:
            if not (s.get("weldable") and s.get("matches_joint_type", True)):
                continue
            par = s["parametric"]
            if par.get("kind") != "line":
                continue
            p0 = np.asarray(par["p0_mm"], dtype=float)
            p1 = np.asarray(par["p1_mm"], dtype=float)
            t = p1 - p0
            t = t / max(np.linalg.norm(t), 1e-12)
            for pid, a, b in edges:
                e = b - a
                ln = float(np.linalg.norm(e))
                if ln < 1e-9:
                    continue
                e = e / ln
                if _seg_seg_dist(a, b, p0, p1) < NEAR_MM:
                    continue          # joint-constrained (bearing or terminating) edge
                ang = float(np.degrees(np.arccos(min(1.0, abs(float(e @ t))))))
                rows.append((jt, mech, ang, ln))
    return rows


def fold(angles):
    """Fold to [0, 45]: parallel and perpendicular are the SAME shortcut."""
    a = np.asarray(angles, dtype=float)
    return np.minimum(a, 90.0 - a)


def report_curved(rows, st) -> None:
    """The step-5 ruling: printed so the number exists, binding on nothing."""
    if not st["scenes"]:
        return
    print(f"\ncurved scenes: {st['scenes']}  "
          f"(closed primary seams exempt: {st['closed_exempt']}, "
          f"open: {st['open_seams']})")
    print(f"  edges excluded - provenance (seam's own spine): "
          f"{st['provenance_excluded']}, nearness: {st['near_excluded']}")
    if st["foreign_spine"]:
        print(f"  WARNING: {st['foreign_spine']} swept part(s) with a spine that is "
              f"NOT the seam's - their longitudinal edges are not sampled yet")
    if not rows:
        print("  no (sample, free-edge) pairs survive exclusion - nothing to report")
        return
    bins = np.arange(0.0, 100.0, 10.0)
    print("  REPORTED, not gated (spines are drawn at random - the D28 prior cannot "
          "arise by construction):")
    for fam in sorted({r[0] for r in rows}) + ["ALL"]:
        sel = rows if fam == "ALL" else [r for r in rows if r[0] == fam]
        a = np.asarray([r[1] for r in sel])
        w = np.asarray([r[2] for r in sel])
        h, _ = np.histogram(a, bins=bins, weights=w)
        pct = 100.0 * h / max(h.sum(), 1e-12)
        print(f"  {fam:12s} {len(sel):7d}  " + " ".join(f"{v:4.0f}" for v in pct))


def report_chord(n, worst, bad) -> bool:
    if n == 0:
        print("\nD34 chord gate: no scene records max_chord_error_mm - vacuous")
        return True
    print(f"\nD34 chord gate: {n} scenes record max_chord_error_mm, "
          f"worst {worst:.4f} mm vs budget {CHORD_MM} mm")
    for sid, v in bad[:10]:
        print(f"  VIOLATION {sid}: {v:.4f} mm")
    print("  D34 gate:", "PASS" if not bad else f"FAIL - {len(bad)} scene(s) over")
    return not bad


def report(rows, spike_tol: float) -> bool:
    if not rows:
        print("no primary line seams found - nothing to gate")
        return False
    print(f"{len(rows)} (seam, free-edge) pairs")
    hdr = "  {:8s} {:8s} {:6s}  length-weighted angle histogram [%, 10 deg bins]"
    print(hdr.format("joint", "mech", "n"))
    bins = np.arange(0.0, 100.0, 10.0)

    def line(sel_rows, jt, mech):
        a = np.asarray([r[2] for r in sel_rows])
        w = np.asarray([r[3] for r in sel_rows])
        h, _ = np.histogram(a, bins=bins, weights=w)
        pct = 100.0 * h / max(h.sum(), 1e-12)
        print(f"  {jt:8s} {mech:8s} {len(sel_rows):6d}  " +
              " ".join(f"{v:4.0f}" for v in pct))
        return h

    for jt in ("T", "corner", "butt", "lap", "edge"):
        for mech in ("none", "yaw", "outline", "yaw+out"):
            sel = [r for r in rows if r[0] == jt and r[1] == mech]
            if sel:
                line(sel, jt, mech)
    h = line(rows, "ALL", "")

    # The gate: mass in the terminal bins ([0,10) and [80,90]) vs uniform expectation.
    spike = float((h[0] + h[-1]) / max(h.sum(), 1e-12))
    expect = 20.0 / 90.0
    print(f"\n  terminal-bin mass {spike:.2f} vs uniform {expect:.2f} "
          f"(tolerance {spike_tol:.1f}x)")
    ok = spike <= spike_tol * expect
    print("  D28 gate:", "PASS - the axis-alignment prior is broken"
          if ok else "FAIL - seams still concentrate at 0/90 deg")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("roots", nargs="+", help="corpus roots to pool (walked recursively)")
    ap.add_argument("--spike-tol", type=float, default=2.0,
                    help="allowed terminal-bin mass as a multiple of uniform")
    args = ap.parse_args()
    rows, crows = [], []
    cst = {"scenes": 0, "closed_exempt": 0, "open_seams": 0,
           "provenance_excluded": 0, "near_excluded": 0, "foreign_spine": 0}
    chord = [0, 0.0, []]
    for r in args.roots:
        rows += collect(Path(r))
        cr, st = collect_curved(Path(r))
        crows += cr
        for k in cst:
            cst[k] += st[k]
        n, worst, bad = chord_sweep(Path(r))
        chord = [chord[0] + n, max(chord[1], worst), chord[2] + bad]

    if rows:
        plate_ok = report(rows, args.spike_tol)
    else:
        plate_ok = True               # vacuous on a purely curved corpus
        print("no plate scenes - the D28 plate gate is vacuous here")
    report_curved(crows, cst)
    chord_ok = report_chord(*chord)
    sys.exit(0 if (plate_ok and chord_ok) else 1)


if __name__ == "__main__":
    main()
