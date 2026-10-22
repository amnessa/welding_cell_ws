#!/usr/bin/env python3
"""Registration error of a tube-on-plate joint, measured by the pen's own contacts
(for a swept band on a plate - RR1, SP3 - each contact against the registered
surfaces, without a fit).

    python scripts/contact_error.py                                  # the save dir's last run
    python scripts/contact_error.py --marks <save_dir>/marks_history/tack_marks_<stamp>.json

The marking node stops each descent at the touch force and records the pen tip there
(`tip_contact_m`): a point on the REAL part. A tack's descent comes in at ~45 deg, so a
registration error moves the contact up the member's wall (the real wall is nearer than
registered) or along the plate (it is farther). Each contact must lie on the real plate
or on the real wall; this fits the two errors that explain them:

  * `lateral_mm`: where the real tube's axis is, relative to the registered one, in the
    plate's plane (2 numbers);
  * `plate_mm`: how far the real plate top is above (+) the registered one.

The fit is a grid search (+-25 mm lateral, +-10 mm plate) of the sum over contacts of the
smaller of the two surface residuals, with the side conditions (a plate contact lies
outside the real tube, a wall contact above the real plate) and a small pull to zero
when the contacts leave a direction free. Then every mark's distance from the REAL seam
root, the number a ruler measures, is reported next to the registration's own prediction.
A fit that leaves more than 1 mm rms (or sits on the search bound) is flagged: the error
was not a translation (a tilt or a spin of the registered part), and only the raw
per-contact heights mean anything then.
Vision-only stays the method: this is evaluation, never fed back.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG))


def fit_offsets(contacts: np.ndarray, plate_n: np.ndarray, plate_d: float, axis_p: np.ndarray,
                axis_u: np.ndarray, r: float, e1: np.ndarray, e2: np.ndarray,
                span: float = 25.0, step: float = 0.25, dz_span: float = 10.0,
                tol: float = 0.5, pull: float = 1e-3) -> dict:
    """(lateral offset (2), plate offset) that put every contact on the real plate or
    the real wall; mm in, mm out. See the module docstring."""
    g = np.arange(-span, span + 1e-9, step)
    gz = np.arange(-dz_span, dz_span + 1e-9, step)
    D1, D2 = np.meshgrid(g, g, indexing="ij")
    shift = D1[..., None] * e1 + D2[..., None] * e2                     # (G, G, 3)
    h = contacts @ plate_n + plate_d                                    # above reg. top
    cost = np.zeros((len(g), len(g), len(gz)))
    for c, hc in zip(contacts, h):
        w = c - (axis_p + shift)                                        # to the shifted axis
        rho = np.linalg.norm(w - (w @ axis_u)[..., None] * axis_u, axis=-1)
        r_wall = (rho - r)[..., None]                                   # (G, G, 1)
        r_plate = (hc - gz)[None, None, :]                              # (1, 1, Z)
        on_plate = r_plate ** 2 + 1e6 * np.clip(r - tol - rho, 0, None)[..., None] ** 2
        on_wall = r_wall ** 2 + 1e6 * np.clip(gz[None, None, :] - tol - hc, 0, None) ** 2
        cost += np.minimum(on_plate, on_wall)
    cost += pull * (D1[..., None] ** 2 + D2[..., None] ** 2 + gz[None, None, :] ** 2)
    i, j, k = np.unravel_index(np.argmin(cost), cost.shape)
    d = np.array([g[i], g[j]])
    dz = float(gz[k])
    shift_ij = d[0] * e1 + d[1] * e2
    out = []
    for c, hc in zip(contacts, h):
        w = c - (axis_p + shift_ij)
        rho = float(np.linalg.norm(w - (w @ axis_u) * axis_u))
        rw, rp = rho - r, hc - dz
        out.append({"surface": "wall" if abs(rw) <= abs(rp) else "plate",
                    "residual_mm": float(min(abs(rw), abs(rp))),
                    "height_above_real_plate_mm": float(rp), "outside_real_wall_mm": float(rw)})
    rms = float(np.sqrt(np.mean([o["residual_mm"] ** 2 for o in out])))
    at_bound = bool(np.abs(d).max() >= span - 1e-9 or (dz_span > 0 and abs(dz) >= dz_span - 1e-9))
    return {"lateral_mm": d.tolist(), "lateral_norm_mm": float(np.linalg.norm(d)),
            "explained": bool(rms <= 1.0 and not at_bound),
            "plate_mm": dz, "rms_mm": rms,
            "contacts": out}


def _root_distance(c: np.ndarray, seam_pts: np.ndarray) -> float:
    return float(np.linalg.norm(seam_pts - c, axis=1).min())


def band_wall_offset(band, c: np.ndarray) -> tuple[float, str]:
    """(mm outside the band's nearest wall, which wall '-w' / '+w'), for a point `c` (mm,
    world): positive = outside the band, negative = inside it. The wall is the offset
    surface at offset_lo ('-w') or offset_hi ('+w') of the spine, extruded along z."""
    T = np.asarray(band.T_world_part, float)
    loc = (np.asarray(c, float) - T[:3, 3]) @ T[:3, :3]
    sp = band.spine
    ts = np.linspace(0.0, sp.t_period, 4000, endpoint=not sp.closed)
    P = sp.point(ts)[:, :2]
    k = int(np.argmin(np.linalg.norm(P - loc[:2], axis=1)))
    tan = sp.tangent(ts[k:k + 1])[0, :2]
    nrm = np.array([-tan[1], tan[0]]) / np.linalg.norm(tan)
    off = float((loc[:2] - P[k]) @ nrm)
    lo, hi = float(band.offset_lo_mm), float(band.offset_hi_mm)
    if off >= 0.5 * (lo + hi):
        return off - hi, "+w"
    return lo - off, "-w"


def _report_band(marks_doc, objects, parts, slab, band) -> int:
    """A swept band on a plate (RR1, SP3): no fit - its error is not one shift - only each
    contact against the registered surfaces."""
    T = np.asarray(band.T_world_part, float)
    centre = T[:3, 3] + 0.5 * (band.z0_mm + band.z1_mm) * T[:3, 2]
    face = max(("+w", "-w"), key=lambda f: slab.face_plane(f).n @ centre + slab.face_plane(f).d)
    pl = slab.face_plane(face)
    print(f"{objects[parts.index(band)][0]} (swept band) on {objects[parts.index(slab)][0]}: "
          "each contact against the registered surfaces (no fit for bands)")
    print(f"  {'tack':>5} {'depth':>7} {'above reg. plate':>17} {'outside reg. wall':>18}")
    for m in marks_doc["marks"]:
        if not (m.get("contact") and m.get("tip_contact_m")):
            print(f"  {m['seam_id']}.{m['tack_no']:<3}  no contact")
            continue
        c = np.asarray(m["tip_contact_m"], float) * 1000.0
        out, wall = band_wall_offset(band, c)
        print(f"  {m['seam_id']}.{m['tack_no']:<3} {m['contact_depth_m'] * 1000:+6.1f} "
              f"{pl.n @ c + pl.d:+13.1f} mm {out:+12.1f} mm ({wall})")
    return 0


def _tube_wall_mm(tube, c: np.ndarray) -> float:
    """mm outside a tube's outer wall (negative = inside its outer radius)."""
    T = np.asarray(tube.T_world_part, float)
    loc = (np.asarray(c, float) - T[:3, 3]) @ T[:3, :3]
    return float(np.hypot(loc[0], loc[1]) - tube.r_outer_mm)


def _report_saddle(marks_doc, objects, parts, tubes) -> int:
    """A branch on a run pipe (S3 on R2): each contact against both registered walls."""
    main, branch = sorted(tubes, key=lambda t: -t.r_outer_mm)
    print(f"{objects[parts.index(branch)][0]} (branch) on {objects[parts.index(main)][0]} (run): "
          "each contact against the registered walls (no fit for a saddle)")
    print(f"  {'tack':>5} {'depth':>7} {'outside reg. run wall':>22} {'outside reg. branch wall':>25}")
    for m in marks_doc["marks"]:
        if not (m.get("contact") and m.get("tip_contact_m")):
            print(f"  {m['seam_id']}.{m['tack_no']:<3}  no contact")
            continue
        c = np.asarray(m["tip_contact_m"], float) * 1000.0
        print(f"  {m['seam_id']}.{m['tack_no']:<3} {m['contact_depth_m'] * 1000:+6.1f} "
              f"{_tube_wall_mm(main, c):+18.1f} mm {_tube_wall_mm(branch, c):+21.1f} mm")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--save-dir", default=str(PKG / "scripts" / "foundationpose_results"))
    ap.add_argument("--marks", default=None, help="a tack_marks*.json (default: the save dir's)")
    ap.add_argument("--registry", default=str(PKG / "models" / "weldgen_objects.json"))
    ap.add_argument("--weldgen-path", default="/workspaces/welding_cell_ws/weld_generator")
    ap.add_argument("--json", default=None, help="also write the result here")
    ap.add_argument("--plate-on-table", action="store_true",
                    help="the plate lies on the touched table (notebooks/table_plane.json): its top "
                         "is the table + its thickness, and only the tube's sideways offset is "
                         "fitted. Round a tilted tube (E2) the plate height and the sideways "
                         "offset trade off; four contacts alone cannot tell them apart")
    args = ap.parse_args()
    sys.path.insert(0, args.weldgen_path)
    from weldgen.curves import ellipse_from_plane_cylinder

    from admittance_control import seam_from_registration as sfr
    from admittance_control.weldgen_registry import load_registry

    save = Path(args.save_dir)
    marks_doc = json.loads(Path(args.marks or save / "tack_marks.json").read_text())
    assembly = marks_doc.get("assembly") or json.loads((save / "assembly.json").read_text())
    objects = [(o["model"], np.asarray(o["pose_static"], float).reshape(4, 4))
               for o in assembly["objects"]]
    parts = sfr.posed_parts(objects, load_registry(args.registry))
    slab = next((p for p in parts if type(p).__name__ == "Slab"), None)
    tube = next((p for p in parts if type(p).__name__ == "Tube"), None)
    band = next((p for p in parts if type(p).__name__ == "SweptSlab"), None)
    if slab is not None and tube is None and band is not None:
        return _report_band(marks_doc, objects, parts, slab, band)
    tubes = [p for p in parts if type(p).__name__ == "Tube"]
    if slab is None and len(tubes) == 2:
        return _report_saddle(marks_doc, objects, parts, tubes)
    if slab is None or tube is None:
        print("needs a tube or a swept band standing on a slab (the curved bench); found "
              + ", ".join(type(p).__name__ for p in parts))
        return 1
    T = np.asarray(tube.T_world_part, float)
    centre = T[:3, 3] + 0.5 * tube.length_mm * T[:3, 2]
    face = max(("+w", "-w"), key=lambda f: slab.face_plane(f).n @ centre + slab.face_plane(f).d)
    pl = slab.face_plane(face)
    n = np.asarray(pl.n, float)
    e1 = np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    marks = [m for m in marks_doc["marks"] if m.get("contact") and m.get("tip_contact_m")]
    if len(marks) < 3:
        print(f"{len(marks)} contacts: at least 3 are needed")
        return 1
    C = np.array([m["tip_contact_m"] for m in marks]) * 1000.0
    plate_d = float(pl.d)
    if args.plate_on_table:
        tp = json.loads((PKG / "notebooks" / "table_plane.json").read_text())["plane"]
        k = float(np.linalg.norm([-tp["a"], -tp["b"], 1.0]))
        n = np.array([-tp["a"], -tp["b"], 1.0]) / k
        e1 = np.cross(n, [1.0, 0.0, 0.0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
        thick = float(min(slab.dims_mm))
        plate_d = -tp["c"] * 1000.0 / k - thick                  # the plate top: table + thickness
        print(f"  the plate on the table: its top is the touched table + {thick:.1f} mm")
    fit = fit_offsets(C, n, plate_d, T[:3, 3], T[:3, 2], float(tube.r_outer_mm), e1, e2,
                      dz_span=0.0 if args.plate_on_table else 10.0)

    # the seam the registration computed, and the real one (registered moved by the fit)
    def seam(shift, dz):
        curve = ellipse_from_plane_cylinder(-plate_d * n + dz * n, n, T[:3, 3] + shift, T[:3, 2],
                                            tube.r_outer_mm)
        return curve.point(np.linspace(0.0, curve.t_period, 2000, endpoint=False))
    reg_seam = seam(np.zeros(3), 0.0)
    shift = fit["lateral_mm"][0] * e1 + fit["lateral_mm"][1] * e2
    real_seam = seam(shift, fit["plate_mm"])

    print(f"{Path(args.marks or save / 'tack_marks.json').name}: {tube.id} = {objects[parts.index(tube)][0]} "
          f"on {objects[parts.index(slab)][0]}, {len(marks)} contacts")
    print(f"  real tube axis vs registered: {fit['lateral_norm_mm']:.1f} mm in the plate plane "
          f"({fit['lateral_mm'][0]:+.1f}, {fit['lateral_mm'][1]:+.1f} along e1, e2 = "
          f"{np.round(e1, 2).tolist()}, {np.round(e2, 2).tolist()})")
    print(f"  real plate top vs registered: {fit['plate_mm']:+.1f} mm   (fit rms {fit['rms_mm']:.2f} mm)")
    if not fit["explained"]:
        print("  NOT EXPLAINED by a sideways shift + a plate offset (rms > 1 mm or the search bound):"
              " the registration is also tilted / spun - read the per-contact heights, not these"
              " offsets (2026-10-14 15:07, E2: 3 of 4 contacts 5-18 mm up the wall, 1 at the root)")
    print(f"  {'tack':>5} {'surface':>8} {'depth':>7} {'mark->real root':>16} {'mark->reg. seam':>16}")
    for m, c, o in zip(marks, C, fit["contacts"]):
        print(f"  {m['seam_id']}.{m['tack_no']:<3} {o['surface']:>8} {m['contact_depth_m'] * 1000:+6.1f}  "
              f"{_root_distance(c, real_seam):13.1f} mm {_root_distance(c, reg_seam):13.1f} mm")
        o["mark_to_real_root_mm"] = _root_distance(c, real_seam)
        o["tack"] = f"{m['seam_id']}.{m['tack_no']}"
    if args.json:
        Path(args.json).write_text(json.dumps(fit, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
