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

Usage:
    python scripts/qa_d28_gate.py out/smoke [out/bench6a ...] [--spike-tol 2.0]

Exit code 0 iff the gate passes on the pooled corpus: the mass of the two terminal
10-degree bins stays below `--spike-tol` times the uniform expectation.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

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


def collect(root: Path):
    """(joint_type, mechanism, angle, edge_length) rows, one per (seam, free edge)."""
    rows = []
    for sj in sorted(root.rglob("scene.json")):
        scene = json.loads(sj.read_text())
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
    rows = []
    for r in args.roots:
        rows += collect(Path(r))
    sys.exit(0 if report(rows, args.spike_tol) else 1)


if __name__ == "__main__":
    main()
