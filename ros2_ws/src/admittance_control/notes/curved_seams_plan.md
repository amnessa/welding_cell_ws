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
5. **Multi-view refinement for curved parts** (started 2026-10-14). The refinement
   itself is part-agnostic (CAD surface samples, point clouds, ICP). What assumed flat
   parts is each part as ONE bounding box. The pieces:
   1. **True shapes.** `part_shapes.py` gives each registered part a signed distance to
      its real surface, from its registry primitive:
      - a slab: its box;
      - a pipe: its cylinder, with the mitre or saddle cut;
      - a band: its walls, with RR1's interior free.
      `multiview_refine` uses it for the overlap rule, the contact samples and the fit-up
      gaps, wherever it used the neighbour's box. Bounding boxes made S3's saddle end
      (it wraps round R2) read as ~5 mm inside R2's box (the limit is 1 mm), and the plate under E2's
      mitre read as being inside E2. A part without a registry entry keeps its box.
   2. **The view planner** takes the near-exact collision boxes (`boxes_from_parts`) for
      its seam targets, its occlusion test and its collision model. SP3's bounding box hid
      the plate along the band.
   3. **Symmetry:** a flat-ended pipe's spin about its own axis is reported as such, not
      as an unmeasured direction (the prior holds it).
   4. **Tests**, synthetic as for the T:
      - a pipe on a plate registered 8 mm off sideways (the bench's C1 / E2 error) is
        brought back by views from several sides;
      - S3 on R2 is not "penetrating";
      - a band (RR1) refines.
   5. **Bench:** `refine_pose` before `welding_points` on C1 / E2, marks in
      `stroke_mode:=dot`, compared with step 4's numbers.

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
| ↳ **invalid** (found 2026-10-14): that scene had S3 hanging UNDER R2 (S3's body lies along its CAD −y, and the rotation sent +y up). The real bench reached 4/4. | | | | | |

- The unreachable tacks are the expected kinds: a concave fillet tighter than the pen
  holder (SP3, confirmed on the bench). The "far side of a saddle" came from the invalid
  S3 scene above. They are reported, not forced.
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

**Step 4, first marks (2026-10-14): C1 and E2 on the 8 mm plate**, `stroke_mode:=tack`,
4 tacks each, all reached and marked.

| part | marks, measured by hand from the root (opposite pairs) | where the far-off marks are |
|---|---|---|
| C1 | 14 / 1.2 mm, 6.5 / 3.7 mm | up C1's wall, not on the plate |
| E2 | 14.4 / 1.2 mm, 11.7 / ~0 mm | up E2's wall |

**The contacts say why** (`scripts/contact_error.py`: every descent stops at the touch,
so its tip is a point on the real part).
- E2: two contacts sit 5.0–7.0 mm above the registered plate top and 5.1–5.5 mm outside
  the registered wall, so they are on the real wall. The opposite two sit 2.3–3.6 mm
  below the registered plate top, on the real plate.
- The fit that puts all four on a real surface (rms 0.48 mm): **the real E2 axis is
  8.1 mm from the registered one, sideways, and the real plate top 3.0 mm lower.** C1
  (contacts rebuilt from the log's depths): 8.8 mm sideways, plate 1.2 mm low.
- With a ~45° pen, a sideways error lands one side's tack up the wall and the other's
  near the root, which is the pattern measured. The fit puts the marks at 10.2 / 8.7 mm
  (wall side) and 4.0 / 3.3 mm (plate side). The pair sums match the ruler (14.2 vs
  15.6, 12.0 vs 11.7); the split differs by ~3 mm. The fit has no tilt error, and the
  20 mm stroke drifts after the contact.
- The direction is not the camera's line of sight every time (C1: mostly along it, E2:
  across it), so it is not a plain depth offset. A small curved part seen from one side
  is the limit: **step 5 is the remedy** (views from several sides).
- Save-time ICP moved C1 13.1 mm (rmse 3.6 mm) and E2 7.6 mm (rmse 4.1 mm). Whether it
  helped is not knowable from these runs. `assembly.json` now keeps the tracker's pose
  before ICP (`icp_refine.pose_static_tracker`).

**Motion, from the same runs:**
- **"Invalid start / goal" on most transits.** It was not the time budget.
  `transit_model` sets the clearance to 90 % of an approach pose's own, and APS's 2 mm
  margin on top made that pose invalid, so OMPL's RRT-Connect threads never started.
  The Python RRT-Connect fallback took 1.5–3 min per transit round the E2 scene's 34 boxes (E2's 33 and the plate), and
  the re-checks of APS paths, their smoothing and the TOTG samples ran in Python too.
  - **Fixed:** the margin is capped at half of the end poses' room.
  - Past `transit_budget_s` (now **2 s**, as asked) APS waits for a first path up to
    `transit_max_budget_s` 10 s.
  - The C++ RRT-Connect comes before the Python one.
  - The re-checks run on the C++ model (`marking.fast_is_valid`; same model, ~590×).
  - E2's plan, replayed offline: **478 s → 17.6 s**, every transit by APS.
- **No table in the model.** `marking.json` had `table_z_m: null`, so transits could run
  through the table.
  - **Fixed:** the collision model (Python and C++) now takes the touched, tilted plane
    of `notebooks/table_plane.json` by default: −96.1 mm at its centroid, 1.70°, about
    −99.8 mm under the bench parts.
  - Fixtures go in `marking.json` `fixtures`: 3–4 pen-tip points on a fixture's top
    become a box standing on the table.
- Every marking run is now kept in `marks_history/` with its assembly. C1's contacts
  were overwritten by E2's run.

**Transits round a pipe (2026-10-14 afternoon).** With the table in and APS fixed, the
pen still swung in big loops between tacks: up to 0.9 m off, hitting things ~75 cm out.
E2's tack 2 → 3 transit, measured on the plan: 102 mm of tip chord, 1,977 mm of tip path,
637 mm off the chord, the base joint 170° for a 68° change.

| what was tried (offline, E2, the same transit) | joint cost | tip path | max off the chord |
|---|---|---|---|
| APS 2 s, from the 35 mm approach points | 3.7–11.0 | 497–1,873 mm | 234–879 mm |
| APS 3 s | 3.0–5.2 | 340–743 mm | 134–368 mm |
| APS 5 s | 3.0–3.2 | 353–595 mm | 149–254 mm |
| APS 3 s, 8 threads instead of 4 | 3.7–6.0 | 410–1,036 mm | worse |
| 3 parallel APS runs, the shortest tip path kept | 3.6–8.6 each | 263–942 mm kept | the runs compete for the cores |
| APS 3 s from 100 mm departure points | 3.02–3.06 | 194–206 mm | 39–55 mm (but 1→2: 432–1,623 mm) |
| rolls chosen together (DP) | endpoints within a few degrees of before | | the gaps are geometric |
| **straight Cartesian move between departure points** | **3.35 / 3.02 / 1.14** | **211 / 191 / 141 mm** | **the same every run** |

- **What changed:**
  - **Departure points.** Every transit now ends `departure_m` = 100 mm out along the
    pen (`marking.json`). The descent runs from there: fast (`v_approach_m_s` 0.05) to
    the 35 mm approach point, then 20 mm/s, the last 6 mm at 2 mm/s. The retract goes
    back out to it. A step whose departure segment is not clear keeps the 35 mm point
    (`departure_note`).
  - **The tool's own straight line** (`marking.cartesian_transit`) is tried after the
    straight joint edge and before APS. It is an IK chain on the locked branch, every
    pose and edge checked; when the chord passes too near the pipe it is bowed outward
    by 30, 60 or 100 mm.
  - **The planned transit is driven.** After a stroke and its retract the arm stands near
    the planned start; the node joins the planned path from there (`_join_planned`)
    instead of re-planning a fresh random one.
  - APS: `transit_budget_s` 3 s (only for transits the first two cannot do).
  - The rolls of a curved seam are chosen together: an exact DP minimising the weighted
    joint travel plus 0.15 rad per mm of clearance under 10 mm.
  - E2's whole plan, replayed: 7 s, tip paths 400 (from home, straight), 211, 191 and
    135 mm. Before the fixes it took 478 s.
- **Marks that afternoon** (a new E2 registration): pairs near perfect, one 8.7 mm on E2.
  The contacts of the last run are 3 of 4 on E2's wall (4.9, 8.3, 17.7 mm up), one at
  the root. That is not a sideways shift (the best one leaves 2 mm rms): this
  registration was also tilted or spun. `contact_error.py` now flags such a fit.
- **RR1 on the plate (2026-10-14, the new transits).**
  - All four transits were Cartesian lines or straight edges, the planned paths joined
    and driven; "the path is amazing".
  - Marks: two close to the root but not touching it, one on the plate ~3 mm from the
    root, one on RR1's wall ~2 mm up.
  - Contact depths along the pen: +10.3, −0.6, −3.1, −2.0 mm.
  - That is clearly better than C1 / E2 (12–14 mm outliers): a rounded-rect tube's flat
    sides pin its pose, a round pipe's side does not.
  - Strokes still stop on registration error: twice "pen lifted off", once over 8 N.
    The stroke follows the registered seam.
  - `contact_error.py` fits tubes only. RR1's run is in `marks_history/` for when bands
    are added.
- **Conclusion of the curved bench so far:** without multi-view, a curved part's mark
  error is set by how well one view pins its pose: ~3 mm with flat sides (RR1), 8–14 mm
  round (C1, E2). **Step 5 (multi-view for curved parts) is needed.**
- **SP3 on the plate (2026-10-14).**
  - Reach 4/6: the two concave-side tacks have the holder body 0.3 / 0.9 mm from the
    band, refused as step 3 predicted.
  - Marked:
    - one tack had no contact within the overshoot (the real surface is farther than
      registered);
    - three are 9.5–12.7 mm up SP3's wall (by hand).
  - The contacts (`contact_error.py`, bands now per contact): 3.7, 0.2 and 7.7 mm above
    the registered plate top, and 2.6, 0.2, 5.3 mm outside the registered walls.
  - Marks up the wall on **both** sides of the band are not a sideways shift (that puts
    one side on the plate): a rotation or shape error of the registered S.
  - The hand numbers exceed the contacts' because a `tack` stroke runs 20 mm along the
    REGISTERED seam after the contact. Round SP3's 23 mm bends, a slightly rotated
    registered seam climbs the real wall over those 20 mm (one stroke "lifted off").
  - **For accuracy numbers, mark in `stroke_mode:=dot`:** the mark is then the contact
    point, and the ruler and `contact_error.py` measure the same thing. `tack` strokes
    are the demonstration.
- **S3 on R2 (2026-10-14).**
  - Reach 4/4 (min clearance 10.3 mm); all marked, all on S3.
  - By hand (stroke start → end): 7.2 → 5.8, 1 → 1.5, 2.6 → 2.6 and 11 mm (lost contact
    during the stroke).
  - Contacts: tacks 1–2 within ~1 mm of the registered root, tacks 3–4 3.4–5.3 mm off
    it, on S3's wall (`contact_error.py`, saddle per contact).
  - A contact at the registered root only means a real surface passes there, so the
    ruler is the number here.

**Step 4 done: the curved bench without multi-view** (2026-10-14, `stroke_mode:=tack`,
by hand from the root):

| joint | reached | marks | note |
|---|---|---|---|
| plate T (reference, 6 Oct) | all | 2.3–2.7 mm | flat faces |
| RR1 on the plate | 4/4 | within ~3 mm | flat sides pin the pose |
| S3 on R2 | 4/4 | 1–7 mm, one 11 mm (lifted off) | saddle found as computed |
| C1 on the plate | 4/4 | 14 / 1.2, 6.5 / 3.7 mm | contacts: 8.8 mm sideways |
| E2 on the plate | 4/4 | 14.4 / 1.2, 11.7 / ~0 mm; a 2nd registration: pairs near perfect, one 8.7 mm | contacts: 8.1 mm sideways; the 2nd also tilted/spun |
| SP3 on the plate | 4/6 | 9.5–12.7 mm, one no contact | 2 concave-side tacks refused (holder); stroke climbs |
| SP3, **with `refine_pose`** (15 Oct) | 4/6 | 2 on the root, one 4.4 mm (one on a fixture) | fit-up 0.0–0.3 mm, was −0.9..10.4 |
| E2, single view (15 Oct, the new plate) | 4/4 | 3.7, 6, 4.9, 5.1 mm | refine refused (overlap) |
| E2, **`refine_pose` + spin settled** (15 Oct) | 4/4 | 3.2, 2.1, 2 mm; 3–4 mm on the tilted side | spin +36.2°, gap range 26.2 → 0.4 mm |

Seams, tacks, reach and motion work for every curved stratum. The error is the
single-view registration of round parts. **Step 5, multi-view refinement for curved
parts, is next.** Mark in `stroke_mode:=dot` when measuring.

**Step 5, multi-view for curved parts: built (2026-10-14), not yet benched.**
- `part_shapes.py`: each registered part's signed distance to its real surface (a
  slab's box; a pipe's cylinder with its cut, the bore solid; a band's walls). It checks
  against every printed CAD: 0 mm on each vertex, normals out, signs as the primitive's
  own inside test.
- `multiview_refine` judges neighbours by it (the overlap rule, the contact samples, the
  fit-up gaps). `parts_from_assembly` attaches it from `models/weldgen_objects.json`; a
  part without an entry keeps its box.
- `multiview.surfaces_from_models` gives the view planner the near-exact boxes (6 per
  pipe, a chain per band) for targets, occlusion and its collision model.
- A flat-ended pipe's spin is named "spin about its own axis (symmetric)" and kept out
  of the relative-pose rule (no fit-up depends on it).

Synthetic views as for the T (`test/test_multiview_curved.py`), true shapes vs the old
bounding boxes:

| case | true shapes | bounding boxes (before) |
|---|---|---|
| C1 on the plate, 8 mm off sideways | accepted, 0.08 mm from the truth | the same, 0.08 mm |
| E2 on its mitre, 5 mm off | accepted, 0.03 mm | plate pushed 49.7 mm, both refused, E2 left 5 mm off |
| S3 on R2, 1.8 mm off | accepted, 0.08 mm, no penetration | both refused ("overlaps by 1.9 / 2.0 mm"), 1.8 mm off |
| RR1 on the plate, 3 mm off | accepted, < 1 mm | |
| view plan round the real C1 (archived 14 Oct) | 4 views at azimuths 60/150/270/330 | |

- **So C1's 8.8 mm needed only `refine_pose`.** The existing multi-view refinement
  removes it even with boxes, and the curved bench never ran it.
- E2, S3 on R2 (and SP3 for the planner) need the true shapes.

**First try, SP3 (2026-10-15): no seam at all, because the registry was stale.**
- The plate CAD had been replaced that day: `test_objv2_base.ply`, now 220 × 220 ×
  5.5 mm with its thickness along CAD z. The registry still held the old 256 × 249 × 8
  plate (thickness along y).
- Mode A put the old shape at the new pose, turned 90°: its face stood vertical, so no
  cap of SP3 lay on it and no seam record existed, not even a rejected one.
- `refine_pose` judged SP3 against that wrong plate too. It kept both as saved: SP3's
  4.5° was over the 4° limit, and the plate is held with it.
- **Fixes:**
  - the registry rebuilt; only that entry changed;
  - **a stale entry is now caught.** A registered CAD's vertices must lie within 0.5 mm
    of its primitive (they do at 0.00–0.07 mm). If not, `welding_points` refuses and
    names `build_weldgen_registry.py --verify`, and `refine_pose` judges that part by
    its real mesh's box, with a WARNING line;
  - `mv_max_correction_deg` 4 → **5** (the user: multi-view is the more accurate
    estimate).
  - With the right plate, SP3's two fillets come out at a fit-up of −0.9..10.4 mm: SP3
    registered tilted, one end ~10 mm up. That is a near miss at 10 mm, weldable at
    15 mm; `refine_pose` is what should level it.
- The tests that replay recorded bench poses and captures now use a frozen copy of the
  bench CADs (`test/data/bench_models/`: the old plate from git and its registry). A part
  swapped in `models/` no longer breaks them.

**SP3 with multi-view (2026-10-15, the first curved part refined on the bench).**
- `refine_pose` (4 views) brought SP3's registered tilt out. Its fit-up along the two
  fillets went from −0.9..10.4 mm (one end ~10 mm up, a near miss) to **0.0..0.3 mm**,
  both weldable at the default 10 mm.
- Reach 4/6, the two concave-side tacks again. The clearances are lower (4.1–4.5 mm,
  from ~9) because SP3 now sits flush on the plate.
- Marked (`tack` strokes), by hand:
  - **tacks 0.2 and 1.3 on the root**, where the two parts meet;
  - **tack 1.2: 4.4 mm**.
  - In step 4, without multi-view: 9.5–12.7 mm and one no-contact.
  - The first tack (0.1) touched an unmodelled fixture, 20 mm early.
- Contacts: the three tacks on SP3 touched 2.8–4.2 mm early along the pen (2.2–3.5 mm
  above the registered plate); in step 4, 0–9.4 mm.
- Still to do: marks in `stroke_mode:=dot`, and the fixtures entered in `marking.json`.

**E2 with multi-view (2026-10-15): refused, then the acceptance rules changed.**
- Marks on the saved poses: 3.7, 6, 4.9 and 5.1 mm. `refine_pose` refused both parts for
  "overlap" (plate 4.3 mm, E2 2.6 mm), although both corrections were within 10 mm / 5°.
- **The user's rule: a correction within the limits is applied.** Replaying the capture
  (`multiview_refine_offline.py`) showed three faults in the acceptance, all fixed:
  1. **A pipe's bore counted as solid** in the true-shape distance. The plate under E2's
     bore read as 4.3 mm inside E2. The bore is hollow now (the planner's boxes stay
     solid).
  2. **The overlap rule ignored the saved assembly.** The saved E2 sat as deep in the
     plate, or deeper, and was kept. Now an overlap over 1 mm refuses only a refinement
     that makes it WORSE than the saved poses; otherwise it is applied with a warning.
  3. **A pipe's spin counted against the rotation limit.** E2's 5.13° was 5.08° of spin
     about its own axis plus 0.71° of tilt. The spin does not move the seam, so the limit
     now judges the axis tilt and the spin is reported.
- Replayed: both parts accepted. The seam moves 4.5–5.3 mm, mostly downwards: the
  refinement puts the plate flush on the touched table, where the saved plate floated
  2.7–6.4 mm above it, tilted.
- **Is that the right way?** The pen's four contacts are all on E2's wall, 5.8–7.9 mm
  above a plate lying on the table: the saved seam was too high, and the refinement
  lowers it.
  - That reading holds **if the plate rests directly on the table** (to be confirmed).
  - Round a tube tilted 25°, the plate height and the tube's sideways offset trade off,
    and four contacts alone cannot separate them. The free fit had chosen "plate +1.5 mm,
    E2 4.5 mm sideways", the opposite.
  - `contact_error.py --plate-on-table` fixes the plate height from the touched table.

**E2 re-run with the new rules (2026-10-15).** The plate lies directly on the table
(confirmed).
- `refine_pose` accepted both parts (E2: 1.95 mm / 3.41° of tilt+spin beyond the camera
  error, 3.4° of it spin). `welding_points`: fit-up −7.6..3.4 mm. A wrong spin turns the
  mitre's low side, so a pipe's fit-up is the least reliable number; its seam is not
  moved by it.
- Contacts above the table-flush plate top, saved → refined: tack 4 6.4 → 2.4 mm, tack 1
  5.8 → 5.4 mm. Only 2 tacks were marked, so the hand marks will decide.
- Two tacks were refused:
  - **tack 0.2, "pen × table 2.5 mm":** a pen touching a 5.5 mm plate lying on the table
    comes that close to the table. The pen's allowed contacts at a tack are now the part
    AND the table (`tack_reach.pen_contact_pair`); at or below the table it is still a
    collision. Re-run: 4/4 reachable, min clearance 6.9 mm;
  - **tack 0.3, holder 2.5 mm from E2** at the stroke START (10 mm from the tack centre).
    With the new reach it plans in both `tack` and `dot` mode.

**E2, another placement (2026-10-15): no seam after multi-view.** The cause was
neither overlap nor the tolerance: **E2's spin about its own axis was ~36° wrong**.
- Its axis stood at 25.2° to the plate (right for a part on its 25° mitre), but its cut
  plane was 15.4° off the plate. Along the seam the gap ran −14.8..+11.5 mm, a 26 mm
  range, refused at any sensible tolerance.
- A cut pipe looks the same at every spin but for its cut, and the cut is hidden against
  the plate: single-view registration got it wrong, and multi-view only added 3°.
- **Fix: `curved_seams.settle_cut_spins`, run by `welding_points` before the seams.** A
  pipe with a cut end standing on a part is turned about its own axis until the cut
  fits the base best (a 1-D search; the seam curve does not depend on the spin).
  - Accepted when the gap range is then ≤ 1.5 mm (or a quarter of before) and at least
    1 mm better.
  - The settled pose goes into `assembly.json` and the part's SEPC points, so
    reachability and marking (the collision boxes at the foot) use it.
  - On that assembly: +36.25°, gap range 26.2 → 0.36 mm, one weldable seam (E2 1.5–1.8 mm
    into the plate, evenly).
- Not done: settling inside `refine_pose`. Its fit-up and overlap warnings for E2 come
  from the wrong spin; the next refine starts from the settled pose.

- **Result** (the user's marks, `tack` strokes): 3.2, 2.1 and 2 mm, and 3–4 mm on the
  tilted side, against 3.7–6 mm from the single-view poses and up to 14.4 mm in step 4.
  All four tacks were reached and marked; every transit was a Cartesian line or a
  straight edge, and the planned paths were driven.

**Bench (next):** per joint: save the base and the part → `refine_pose` (Fist held 2 s)
→ `welding_points` (Palm) → `tack_reachability.py` → marking in `stroke_mode:=dot`.
Compare with step 4's table: C1 and E2 first, then S3 on R2.
