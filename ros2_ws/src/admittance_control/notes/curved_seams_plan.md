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
| SP3 (redrawn) | swept_slab, open band | spine = side A, a cubic with 4 control points (0, −110), (−200, 0), (200, 0), (0, 110), recovered to 3·10⁻⁶ mm; band [0, 5.00], height 80 (along CAD y) | 0.050 / 0.047 mm, 0.00 % |

All six verify far inside the 0.25 mm budget; what is left is the meshes' chord error.
`270circle` also comes back as a spline band now (0.067 / 0.041 mm); `vplate` (a V with
a sharp kink) stays refused.

**Findings:**
- **S3's saddle is cut at r 51.04, not R2's 50.** Seated on R2 it touches along the
  crown, and the gap opens to only about 0.15 mm at the saddle's flanks. Placed coaxially
  as designed, step 2 reports 1.10–1.27 mm along the branch axis.
- **The first SP3 was not a constant-thickness band.** Its second side was the S-curve
  shifted 5 mm along CAD x, so the wall varied from 2.4 to 5 mm and the registry refused
  it. It was redrawn (FreeCAD would not offset the spline) and reprinted the same day.
  The new one has a constant 5.00 mm wall.
- **The spine fit recovers the drawn spline.** The cap outline splits at its four 90°
  corners into side A, side B and the two ends. Side A, side B and the midline are each
  fitted with a cubic (weldgen's clamped uniform knots), solving control points and
  point parameters together. Control-point counts go upward; the first fit within
  0.025 mm wins. Side A won with 4 control points: the curve that was drawn. (The 5-point
  sketch spline is the same curve, since its middle three control points are collinear.)
  - The solver needs a good start (linear fit alternated with re-projection). From
    chord-length parameters it stalls at 0.06 mm.
  - The error is the solver's own residual. A nearest-dense-sample measure added up to
    0.1 mm of sampling error.
- **Two flat ends:** C1, R2 and RR1 can stand on either end. The entry puts the base at
  the lower end along the axis (RR1: at CAD y = −100); step 2 tries both.
- Side faces of a meshed spline surface are not exactly parallel to the extrusion axis
  (up to 2.7°), so the extrusion test trusts the caps, not the sides.

**Step 2 done (2026-10-09).** `admittance_control/curved_seams.py`; `compute_seams`
sends every pair with a tube or a band to `pair_seams`.

| pair | seam | done how |
|---|---|---|
| tube on a plate (C1, E2, R2, S3 on its flat end) | plate plane ∩ outer wall: circle or ellipse, closed | `ellipse_from_plane_cylinder` |
| tube on a tube (S3 on R2) | branch ∩ run outer walls: the saddle, closed | `saddle_from_cylinders` |
| band on a plate (RR1, SP3) | each wall traced along the band's axis onto the plate; closed outline = weld + bore, open band = two fillets | `OffsetCurve` of the spine |
| anything else (band on a pipe, two bands, ...) | one non-weldable record: "no seam rule for a X-Y pair yet" | |

- **Which end stands on the other part is found, not assumed.** Both ends and both
  plate faces are tried. An end counts when its distance to the curve, along the
  member's axis, stays within `pose_tol_mm` all round, and the member must stand at
  least 10° off the face.
- **That distance is the fit-up**: `fitup_mm = {member: [min, max]}`, positive = gap.
  The curve lies on the surfaces the pose error does not move (the plate plane, the
  wall), so a 3° tilt shows up as a 0.4–3.6 mm gap range on C1, not as a different
  seam.
- **Per-point frames:** `n_a_per_point` (base), `n_b_per_point` (wall),
  `approach_per_point` (bisector); `dihedral_deg_range`.
  - C1: 90°;
  - E2: 65–115°;
  - the saddle: 90° at the crown, 120° at the flanks.
- **The verdict is weldgen's `seam_verdict`**: the torch cone at every point, weldable
  at ≥ 95 % clear.
  - Bores are negatives unless the tool fits: `bore_min_diameter_mm` = 200 for the pen
    holder and camera (weldgen's 80 is a torch's).
  - A curve running off the plate is `off_plate_edge`.
- **Tacks** (the first part of step 3): `compute_tacks` passes `closed` to `tackrule-0.1`
  (an even count around a loop, phase from the scene id). Each tack's approach is
  interpolated from the per-point axes at its arclength; round C1 they rotate at 45°.
- `runtime_access` takes the sheet-thickness cap from flat parts only (a tube wall is
  not a sheet).
- Bench-like scenes run in 0.1–0.8 s (C1, E2, RR1, S3 on R2).
- `collision.boxes_from_parts` boxes a tube or band around its mesh. Before, a tube
  would have been boxed about its base frame, half its length off. The box is
  conservative (a pipe's box corners stand r(√2 − 1) proud of the wall); step 3
  decides whether the collision model needs a cylinder.

**Tests:**
- `test/test_curved_registry.py` (13):
  - synthetic tubes (flat, mitred, saddle-cut), a rounded-rect band and a cubic spline
    band, each fitted back at a skewed pose;
  - the sideways-shifted band, refused;
  - the six printed parts;
  - the tube collision box.
- `test/test_curved_seams.py` (11):
  - C1-like on a plate: circle length, fit-up under a 3° tilt, 45° approach, bore
    confined;
  - standing on the top cap;
  - the mitre ellipse (Ramanujan length, 65–115°);
  - off the edge, and floating;
  - the saddle (gap along the branch, points on both walls, 90–120°);
  - the printed S3 on R2;
  - RR1-like outline and bore;
  - the open band's two fillets, with frames still matched after re-orientation;
  - the printed SP3 on a plate;
  - closed-seam tacks with their own approach;
  - an unsupported pair.

**Next:** step 3. Run `tack_reachability` on a closed seam: full loops need large wrist
rolls, so expect and report unreachable far-side tacks, and decide on the cylinder in
the collision model. Then step 4, the bench (C1 on the 8 mm plate first).
