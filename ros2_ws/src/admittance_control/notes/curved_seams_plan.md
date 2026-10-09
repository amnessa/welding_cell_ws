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
- `collision.boxes_from_parts` boxed a tube or band around its mesh (before, a tube
  was boxed about its base frame, half its length off); step 3 replaced the one box.

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

**Step 3 done (2026-10-09).** Reachability, collision boxes and strokes for curved
seams.

- **One roll per seam does not work round a pipe.** The approach axis rotates with the
  seam. The planner's rule (one roll per seam, at most 69° of joint step between
  consecutive tacks) failed C1 at its second tack: the wrist would have to turn 115°.
  - **Fix:** on a curved seam (`seam_curved` on the tack), `tack_reach._plan_per_tack`
    solves every tack on its own.
  - The best (tilt, roll) is chosen by clearance up to 10 mm, then by the smallest joint
    step from the previous tack.
  - An unreachable tack is reported with its reason, and the others go on.
  - The step limit does not apply: each tack has its own APS transit.
  - The seam reports `n_reachable / n_tacks`. `marking.visit_order` already marks only
    the reachable tacks.
- **The collision model gets the curved parts' real shape**, still as boxes, so the C++
  planner is unchanged.
  - **Pipe:** 6 boxes rotated about the axis. The union covers the disk and stands at
    most 3.3 % of r outside it (1.0 mm on C1); the single box stood 41 % proud.
  - **Cut pipe** (E2, S3): radial half-box sectors, each starting at its own lowest cut
    point (E2: 33, S3: 23). They stand 0.2 mm outside the wall and hang at most 4.5 mm
    below the cut under the wall, which is inside the 8 mm plate or inside R2.
  - **Band:** a chain of boxes along the spine, each within 1 mm of the band (RR1: 9,
    SP3: 12). A pipe's boxes are solid (bore included); a band's follow its wall.
- **Strokes:** in `tack` mode, a tack on a curved seam draws its own piece of the curve
  (`marking.seam_section`). The chord p0→p1 missed C1 by 0.6 mm at a 12 mm tack. In
  `seam` mode a curved seam draws per-tack pieces too, because one pen axis cannot draw
  a loop round a pipe. `stroke_modes` tells the node, so its once-per-seam skip does not
  drop the loop's other tacks.

Bench-like scenes (8 mm plate top at z 20 mm, the part centred at (0.55, 0.10) m,
1 mm gap, the configured 3 mm clearance):

| part | tacks | reachable | min clearance | unreachable because | plan time |
|---|---|---|---|---|---|
| C1 | 4 | 4 | 9.7 mm | | 14 s |
| C1, one roll per seam (old) | 4 | fails at tack 2 | | joint step 115° > 69° | 7 s |
| E2 | 4 | 4 | 6.6 mm | | 55 s |
| RR1 | 4 | 4 | 9.6 mm | | 21 s |
| SP3 (two fillets) | 6 | 4 | 9.1 mm | at the lobes' concave sides the holder is 1.7–1.8 mm from the band | 24 s |
| S3 on R2 (R2 along x, axis 80 mm up) | 4 | 3 | 10.0 mm | the far-side tack has no IK on the elbow-up branch | 77 s |

- The unreachable tacks are the expected kinds: the far side of a saddle, and a
  concave fillet tighter than the pen holder. They are reported, not forced.
  Turning the part by hand, or a slimmer holder, would reach them.
- Planning is 14–77 s per part, offline (`scripts/tack_reachability.py`); the marking
  node reads the result.

**Tests:** `test/test_curved_reach.py` (6):
- pipe boxes cover it, ≈ 1 mm proud;
- mitre sectors hug the cut;
- band chain covers the wall and leaves RR1's interior free;
- the C1 loop reached with a roll per tack, and the old one-roll rule failing on it;
- curved strokes follow the seam, and `seam` mode becomes per-tack.

`test_curved_registry.py`'s tube-box test now expects the 6 boxes.

**Step 4, first bench (2026-10-09, perception only).** All printed parts except SP3
(still printing) went through capture → SAM2 → PPF → FoundationPose → tracking → save.

| part | PPF margin over the runner-up | notes |
|---|---|---|
| C1 | 0.285 / 0.379 (over RR1) | tracker LOST once (fit 0.32) |
| E2 | 0.386 (over smallplate) | tracker LOST (fit 0.12) |
| RR1 | 0.103 (over C1) | tracker LOST (fit 0.49) |
| R2 | **0.040** (over vplate) | the thinnest margin; ICP at save refused (spin) |
| S3 | named S3 twice (scores 105, 108) | saved with ICP 1.1 mm / 1.3° |

**What the bench found, and the fixes:**
1. **Mode A never ran.** `curved_seams.py` was missing from CMakeLists' explicit install
   list, so the installed package could not import it, and `~/welding_points` fell back
   to the radius-PCA detector. The tests import from the source tree, so they passed.
   - **Fixed:** the module is added, and `test/test_install_list.py` now fails on any
     module missing from the list.
2. **Re-registration switched parts.** On LOST, the tracker sends one frame and a mask
   projected from the last pose to the desktop. The desktop's PPF reclassified that
   mask and answered the plate under the part: C1 → `test_objv2_ear`, E2 → ear → base.
   - **Fixed:** `fp_stream.post_predict_pose(..., object_name=)` sends the tracked name.
     `fp_server.py` (the copy in `scripts_in_foundationpose`; **deploy it to the
     desktop**) registers that CAD and skips PPF. `fp_tracker_node` refuses an answer
     that names another part and stays LOST.
3. **ICP at save was refused on the pipes:** R2 5.7 mm / 30.8°, C1 8.5 mm / 9.0°, against
   the 8° gate. A flat-ended pipe looks the same at any spin about its own axis, so
   that spin is arbitrary.
   - **Fixed:** `pose_stats.remove_twist` keeps the tracker's spin, and applies and
     gates only the axis tilt. It is used for uncut tubes only
     (`weldgen_registry.symmetry_axis`; a cut end fixes the spin).
4. **A near miss was silent.** In the replay of the saved assembly, S3 on R2 gives the
   saddle at the right angle (69.4° vs 70°). But S3's axis is registered 7 mm to the
   side of R2's, so the gap reads −5.3 to +10.5 mm against the 10 mm tolerance, and the
   pair was dropped without a word.
   - **Fixed:** within 2× the tolerance a pair is reported, rejected, as
     `fitup_beyond_pose_tol (lo..hi mm)`.
   - In that replay C1 and R2 overlap (axes 11 mm apart) because the parts were swapped
     after saving, so C1's `bisector_blocked` there is not a finding.
5. Not ours: the laptop tracker replied "Logger severity cannot be changed between
   calls" after several LOST events (the FoundationPose docker side).

**Still to bench (step 4 proper).** Each joint is assembled physically and left as
saved: `~/reset_environment`, then save the base, then the part, without moving
anything afterwards. Per joint:
1. `~/welding_points`: the seam in RViz on the real joint; the log's summary
   (`fillet … gap a..b mm`) is the registration's fit-up;
2. `tack_reachability.py`: which tacks are reachable;
3. `tack_marking` `~/plan` (dry run), then `~/all`;
4. measure every mark's offset from the real seam root (the 2.3–2.7 mm of the plate T
   is the number to compare).

Order: C1, E2, RR1, S3 on R2 (two V-blocks), SP3 when printed. Restart the launch after
`colcon build` (the rebuild installs `curved_seams.py`).

**Then step 5,** multi-view refinement for curved parts. The S3/R2 replay already shows
why: a 7 mm sideways registration error is too much for a saddle.
