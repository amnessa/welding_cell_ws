# Curved seams in the cell (mode A for the curved strata): plan

*Written 2026-10-09. Context: `todo.md` → OPEN — perception ("Mode A covers only the 5
plate strata"); the printed parts in `curved_parts_print_list.md`.*

## The idea: WeldSet's verification arm, used forward

- **WeldSet** draws the seam curve first and builds the parts around it (D29,
  curve-first).
- **To check itself,** weld_generator also goes the other way:
  `verify_curved.rediscover_seam` recomputes the seam from the placed parts alone
  (poses and radii, never the drawn curve).
- **The cell has exactly that input.** Every registered CAD is a placed part with a
  known pose. So the seam is the intersection of the registered parts' analytic
  surfaces, with no seam detection in the point cloud.

| stratum (printed part) | seam from the registered parts | weld_generator |
|---|---|---|
| T/circle (C1), T/ellipse (E2) | the plate's top-face plane ∩ the tube's outer cylinder: a circle, or an ellipse if tilted | `curves.ellipse_from_plane_cylinder` |
| T/saddle (S3 on R2) | branch cylinder ∩ run cylinder | `curves.saddle_from_cylinders` |
| T/rounded_rect (RR1) | the tube's outer outline (its closed spine) laid onto the plate plane | `SweptSlab` spine, `rounded_rect_curve` |
| T/swept_path (SP3) | the band's two side faces on the plate: the spine offset ± t/2 | `OffsetCurve` |
| butt arc (BA, laser-cut MDF) | the two matching arc edges (gap middle) | `Arc3D` |

**Why it tolerates registration error**, as the plate seam does:
- a plane ∩ cylinder curve does not change if the tube slides along its own axis, or
  sits a few mm above or inside the plate;
- the gap or penetration between the tube's cut end and the plate along the curve is
  the fit-up value, as `fitup_mm` is now.

**Per-point frames and verdicts** come from `verify_curved.curved_seam_set`: exact
normals of both surfaces, the approach (their bisector), the dihedral, roles
weld / bore / toe. `seam_verdict` / `cone_clear_fractions` give the D4-equivalent torch
check per point. **Closed-seam tacks:** `tacks.py` (an even count, phase from the
scene id).

## Steps

1. **Registry entries for the printed parts.**
   - `tube` for C1, E2, R2, S3, with the base cut: flat, plane (mitre), or a cylinder
     (the saddle against R2).
   - `swept_slab` for RR1 (a closed rounded-rect spine, band [0, wall]) and SP3 (an open
     spine, band ± t/2).
   - Each entry: the parameters, and `T_cad_prim` (primitive frame → CAD frame).
   - **Derived from the mesh** (`derive_tube` / `derive_extrusion` in
     `weldgen_registry.py`, run by `scripts/build_weldgen_registry.py --verify`):
     - axis from the lateral faces' normals;
     - radii from the radial distances;
     - cut surfaces fitted to the end faces;
     - spines fitted to the band's centre surface.
   - **Verified both ways:** CAD vertices → primitive surface (the existing
     `verify_entry`), and primitive mesh → CAD surface, which catches an entry that is
     too long.
   - Accepted at the D34 chord budget, or with the measured deviation recorded.
2. **A curved branch in `seam_from_registration.compute_seams`.**
   - Recognise the pair: tube–slab, tube–tube, swept–slab, slab–slab with arc edges.
   - Intersect with the functions above at the registered poses (mm, static frame).
   - Pick the plate face the part stands on (nearest within `pose_tol_mm`).
   - Output the same `welding_seams.json` format: polyline, per-point normals and
     approach, class, verdict, fit-up along the curve.
3. **Tacks on curved and closed seams:** `tackrule-0.1` open/closed, per-tack approach
   axis; `tack_reachability` already works per tack. Full loops around a pipe need large
   wrist rolls: expect unreachable tacks on the far side, and report them.
4. **Bench, simplest first:**
   1. C1 on the 8 mm plate;
   2. E2;
   3. RR1, SP3;
   4. S3 on R2 (two V-blocks).
5. **Multi-view refinement for curved parts.** The overlap rule and the fit-up gap use
   part BOXES now, which is wrong for a tube. Turn the overlap term off for curved parts
   until it uses the signed distance to the tube / band surface (`todo.md`, multi-view).

## Status

**Step 1 done (2026-10-09).** `models/weldgen_objects.json` rebuilt with
`python3 scripts/build_weldgen_registry.py --verify`. The fits run inside
`build_registry`: box → tube → extrusion, the first that verifies wins.

| part | entry | fitted values | CAD→prim / prim→CAD, volume |
|---|---|---|---|
| C1 | tube, flat base | r 31.000, wall 3.000, length 100; frame = CAD | 0.048 / 0.010 mm, 0.17 % |
| E2 | tube, plane cut (mitre 25.0°) | r 45.003, wall 5.006, length 110 | 0.049 / 0.019 mm, 0.00 % |
| R2 | tube, flat base | r 50.000, wall 6.000, length 180; frame = CAD | 0.049 / 0.016 mm, 0.09 % |
| S3 | tube, cylinder cut (saddle) | r 25.009, wall 4.018, length 99.34; **cut r 51.04**, its axis at 70.0° to the branch, crossing it | 0.057 / 0.026 mm, 0.24 % |
| RR1 | swept_slab, closed rounded rect | 64 × 64, corner r 12, band [0, 3], height 100 (along CAD y) | 0.037 / 0.004 mm, 0.11 % |
| SP3 | **refused** | the wall changes from 2.6 to 5.0 mm along the band | — |

All five verify far inside the 0.25 mm budget; what is left is the meshes' chord error.

**Findings:**
- **S3's saddle is cut at r 51.04, not R2's 50.** S3 on R2 therefore leaves a root gap of
  about 1 mm all round (a print allowance). Step 2 reports it as fit-up, like a plate gap.
- **SP3 is not a constant-thickness band.** Its second side is the first S-curve
  **shifted 5 mm along CAD x**, not offset along the normal. Where the curve runs
  steeply (±60° from z) the wall is only about 2.4 mm. `swept_slab` holds a constant
  band about one spine, so SP3 is refused with the range rather than approximated
  (the best constant band would miss by about 1.3 mm).
  - **Fix in CAD:** draw the second side as a true offset of the spline (FreeCAD:
    Sketcher Offset, or Part → 2D Offset, of the spline by 5 mm), or sweep a 5 × 80
    rectangle along it; then reprint.
  - The open-spine B-spline fit is built when the first constant-wall band arrives
    (`derive_extrusion` already measures the wall and says "constant-wall open band").
- **Two flat ends:** C1, R2 and RR1 can stand on either end. The entry puts the base at
  the lower end along the axis (RR1: at CAD y = −100). Step 2 must try both caps
  against the plate, not assume the base.
- Side faces of a meshed spline surface are not exactly parallel to the extrusion axis
  (SP3: up to 2.7°), so the extrusion test trusts the caps, not the sides.

**A guard until step 2:** `compute_seams` judges flat pairs only (`FLAT_PRIMITIVES`). A
pair with a tube or a band comes back as one non-weldable seam, class `curved`, with
the reason "seam not computed yet (curved_seams_plan.md step 2)". It does not crash.
`collision.boxes_from_parts` now boxes a tube or band around its mesh. Before, a tube
would have been boxed about its base frame, half its length off. The box is
conservative (a pipe's box corners stand r(√2 − 1) proud of the wall); step 3 decides
whether the collision model needs a cylinder.

**Tests:** `test/test_curved_registry.py`, 12 tests:
- synthetic weldgen tubes (flat, mitred, saddle-cut) at a skewed pose, fitted back;
- a rounded-rect swept_slab;
- a sideways-shifted band, refused;
- the five printed parts verified, and SP3 refused;
- the curved-pair guard.

**Next:** step 2, the curved branch in `compute_seams`, starting with C1 on the 8 mm
plate.
