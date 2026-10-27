# Phase 9 — the real test set, as it will be built (2026-10-27)

*The metal has arrived, and this plan turns `phase9_plan.md` (2026-09-21) into a two-week build.
Its rules stay:*
- *a test set only;*
- *truth constructed from measured dimensions and registered poses, with its uncertainty
  measured;*
- *no annotator;*
- *≥ 3 part geometries per stratum.*

*What changes: the matrix (6 configurations per stratum, not 10), the views (the cell's own
capture), and a cut list made for what was delivered.*

## What arrived, and what it changes

- **1 m of each, 304 stainless, 2 mm wall:**
  - RHS 60×60, 80×80 and 50×100;
  - pipes Ø42.4, Ø60.3, Ø76.1 and Ø101.6.

  Plus **3 mm stainless sheet**. The 2026-09-21 list also named Ø114.3, but it is not on the
  invoice: **Ø101.6 is the only run pipe** for the saddles.
- **1–2 mm high-carbon steel sheet** (2026-10-27). It goes to the edge joints, which the
  generator makes only at 1–2 mm (ISO 9692-1 ref 1.1, `edge_max_thickness_mm`). Edge scenes are
  then all carbon steel and every other scene all stainless, so each scene stays one stock, as
  the renderer's `materials-1.1` assumes. Its tier-2 twin uses the renderer's `mild_steel`
  (iron F0), the nearest of its seven alloys.
- **The CNC is a router, bed at least 600×600** (2026-10-27). What that means for 3 mm
  stainless is under Risks.
- **304 is (practically) non-magnetic.** Magnets do not hold it. Fix parts with clamps, wedges,
  putty or double-sided tape, and clamp the base plate to the table. The carbon steel is
  magnetic, so magnets do hold the edge pairs.
- **Outside the generator's draw ranges** (labelled extensions per scene, as `phase9_plan.md` §1
  says; checked against `constructors.py`, `d29.py` and `config.py` on 2026-10-27):
  - the 2 mm walls: the tube families draw 3–8 mm, the swept stiffener 4–10 mm;
  - the 3 mm base under the pipe, tube and stiffener scenes: `_plate_for` draws 4–12 mm;
  - C42 as a stub on a plate: outer radius 21.2 against 30–80 (as the saddle branch S42 it is
    inside, 0.3–0.7 R);
  - S76 on Ø101.6: r/R 0.75 against at most 0.7;
  - RR50: 50 against 60–160. Probably every RHS corner radius too, since the generator draws
    8 mm and up: measure them;
  - the Ø76.1 stiffener arc: span ≈ 110 against 120–300.

  Inside: Ø60.3 (outer radius 30.15 against 30, so its measured OD decides), the mitres and the
  saddle run, 3 mm in the plate strata (1–12) and in the butt arc (3–10, so a sheet measuring
  under 3.0 flags it), and the thin steel in the edge joints. `label_real_scan.py` computes the
  flags itself (below), so none of this rests on the list above.
- **Stainless is specular** for an active-stereo camera. Day 1 decides whether it is scanned
  raw (see Risks).

## Scope: 11 strata × 6 configurations = 66 configurations, 330 views

- **The 6 configurations:** 3 part geometries × 2 fit-ups (nominal, and off-nominal by gap or
  angle). That is the smallest matrix that keeps the "≥ 3 geometries" rule (the held-out
  geometry split) and one fit-up axis.
- **Growing later:** to the original 10 per stratum, by adding fit-ups to the same parts. No new
  cutting.
- **Excluded:** butt grooved, as decided on 2026-09-21.
- **Butt arc is in (2026-10-27):** its plate pairs are cut on the router, together with every
  other flat part (below).
- **Plates are polygons, as in the corpus (D28).** In `bench_phase4`, none of the 180 corner, butt
  and edge plate scenes has a rectangular part, and each of the 120 T-line and lap scenes has
  exactly one (the lower plate). Rectangles let "the seam runs parallel to a plate edge" pass for
  seam finding, the shortcut D28 removed on purpose. A rectangular real set would give that
  shortcut back to the methods that use it, and confound real vs synthetic.
  - On the router a polygon costs the same as a rectangle, so every flat part except the two
    bases B1 and B2 is cut from the **generator's own outlines** (DXF from `outline_uv`).
  - Every T and lap placement also gets a measured in-plane yaw, the corpus's other mechanism.

**Every part is used several times:**

| part (cut from) | strata it serves |
|---|---|
| base plate B1, 300×240×3 | T line and lap: 12 configurations. 240 wide because the plate strata draw widths of 50–250 |
| base plate B2, 200×200×3 | T pipe circle, T pipe cut, T rect. tube, T swept: 24 configurations. The curved scenes' base is the seam's extent plus 50–170 mm (`_plate_for`), and 200 is inside that for every stub here |
| polygon pairs A, B, C (each with a matching seam edge) | corner and butt square: the same two plates, placed two ways; one plate of a pair is the upper part of a lap on B1 |
| thin polygon pairs D, E, F (1–2 mm carbon steel) | edge |
| pipe stubs, square-cut | T pipe circle |
| mitred stubs | T pipe cut |
| the Ø101.6 run + fishmouthed branches | T saddle |
| pipe rings cut to arcs | T swept stiffener: the pipe *is* the arc, so nothing is rolled |
| RHS stubs | T rect. tube |

## Cut list (≈ 25 cuts, most by hand)

| stock | pieces (mm) | left over |
|---|---|---|
| Ø101.6 pipe | C101 stub 120; E101 mitre (tilt 15°) ~140; R101 run 250; 2 rings × 70 → a 170° arc and a 150° arc from one, the other spare | ~300 (spares) |
| Ø76.1 pipe | C76 stub 120; E76 mitre (tilt 20°) ~150; ring 70 → a 170° arc; S76 branch 120 (fishmouth to Ø101.6; r/R 0.75 = extension) | ~520 |
| Ø60.3 pipe | C60 stub 120; E60 mitre (tilt 30°) ~160; S60 branch 120 (fishmouth to Ø101.6) | ~550 |
| Ø42.4 pipe | C42 stub 120 (r 21.2 = extension); S42 branch 120 (fishmouth to Ø101.6) | ~700 |
| RHS 60×60, 80×80, 50×100 | RR60 100, RR80 120, RR50 120 (50 wide = extension) | ~850 each |
| 3 mm sheet, **on the router** from DXF | B1 300×240 and B2 200×200 (the only rectangles: shear or grind them instead if router time is short); P1, P2, P3: three standing polygons for T line; pairs A, B, C: three polygon pairs with matching seam edges; butt-arc pairs R200, R250, R300, spans ≤ 200, cut at a 0.5 mm gap | — |
| 1–2 mm carbon steel, **on the router** from DXF | edge pairs D, E, F; one pair of dissimilar thickness if both gauges are there | — |

**Mitre tilts and stiffener arcs sit inside the generator's draws:**
- The mitre family draws tilts of 10–40°, so E101 is at 15°, off the floor.
- The stiffener family draws spans of 120–300 mm and at most a half circle (radius ≥ span/π).
  So the Ø101.6 arcs are 170° (148 mm along the mid-wall) and 150° (130 mm). The Ø76.1 arc
  (170°, 110 mm) is the one below range.

**The flat parts come from the generator, so each real part is a corpus part:**
- A script runs the generator's own layout code for the plate strata and the butt arc: at t = 3
  for the stainless, at the measured t for the thin steel. It keeps outlines that fit the
  router's bed with a clamp margin and, for T and lap, fit on B1, and writes them as DXF. The
  fit test matters: a drawn outline can be up to 2.1 times its seam length across (a trapezoid
  whose far edge overhangs both ends).
- The outlines are convex, so there are no inside corners to round. The butt-arc concave edge
  (R ≥ 200) is far larger than any cutter radius.
- **For the router:** each seam edge goes on its own DXF layer, so the CAM keeps the holding
  tabs off it; tabs go on free edges and are filed flush. Deburr without chamfering a seam edge.
- The lap's upper plates are chosen 100–200 mm deep, so the 30–60 mm overlaps are 0.15–0.60 of
  the depth, as the generator draws them.
- Measure after cutting, as for the hand-cut parts: the seam edge and one diagonal with
  calipers, and t at 4 points. The truth outline is the DXF outline, checked by these.

**How the pipe cuts are made accurately by hand:**
- A small script prints **wrap-around templates** for every non-square cut from the primitive
  that will model it:
  - the mitre's unrolled ellipse, z(φ) = r·tan τ·cos φ;
  - the saddle's fishmouth, z(φ) = R − √(R² − r² sin² φ).
- Print them at 1:1, wrap them round the pipe, mark, then cut with the grinder and file.
- 3D-print the angle wedges (75°, 110°) and a spare mitre guide on the K1 Max.
- **After cutting, measure what was made:**
  - the pipe end heights at 4 points;
  - the OD at 3 angles;
  - the wall;
  - the RHS corner radius and its outside dimensions;
  - the plate L, W and t.

  The truth is built from the measurements, never from the nominal sizes.

## The configurations

| stratum | 3 geometries | the 6 configurations |
|---|---|---|
| T line | P1, P2, P3 polygons on B1, each at a measured yaw | P1 90°/gap 0, P1 75°/gap 1; P2 90°/0, P2 110°/0; P3 90°/0, P3 90°/2 |
| T pipe (circle) | C42, C60, C76, C101 on B2 | each at gap ~0.2 (4); C60 and C101 at gap ~1 (2) |
| T pipe (cut) | E60, E76, E101 on B2 | each at gap ~0.2 and ~1 |
| T saddle | S42, S60, S76 on R101 | each at gap ~0.2 and ~1 |
| T rect. tube | RR60, RR80, RR50 on B2 | each at gap ~0.2 and ~1 |
| T swept stiffener | 170° and 150° arcs of Ø101.6, 170° arc of Ø76.1, on B2 | each at gap ~0.2 and ~1 |
| corner | polygon pairs A, B, C | A 90°, A 75°, B 90°, B 110°, C 90°, A 90° with gap 1 |
| butt square | polygon pairs A, B, C | A gap 0 / 1 / 3, A misaligned ~1 mm (0.3 t), B gap 0, C gap 1 |
| lap | one plate of A, B, C on B1, each at a measured yaw | A overlap 30 / 60, A 30 with a 1 mm shim, B 40, B 40 + shim, C 40 |
| edge | thin pairs D, E, F | D flush / offset 5 / gap 1, E flush / offset 5, F flush |
| butt arc | R200, R250, R300 pairs (router) | each concentric at 0.5, and at ~1.5 with the inner plate moved 1 mm toward the arc's centre |

**Fit-up:**
- gaps from feeler gauges, or shims at 3 points under a standing part; angles from the printed
  wedges;
- **inside the generator's gap ranges:** the plate strata draw 0–2 mm with a tail to 3, the
  curved T families 0.1–1.2 and the butt arc 0.3–2.0. So the curved T fit-ups are ~0.2 and ~1
  (shims), never contact, and the butt arcs 0.5 and ~1.5. The arc pairs are cut at 0.5. Moving
  the inner plate 1 mm toward the centre makes the gap 1.5 in the middle and about 0.1 mm less
  at the ends (spans ≤ 200, R ≥ 200), so no extra plates are cut;
- **measured afterwards** (feelers at both ends, a protractor) and stored as measured.

**Speed-ups at capture:**
- **Swap parts, not bases.** Capture all the B2 configurations in one block and all the B1 ones
  in another. Change only the standing part while the base stays clamped, and reuse the base's
  registration.
- **Change the fit-up in place.** The gap 0 → gap ~1 variant is a shim under the same placement:
  re-register only the moved part.

## Capture and truth, per configuration (~12 min)

1. Place the parts, set the fit-up, record what was set (2–5 min).
2. Register each part with the cell, as for any assembly: aim, capture, click, FoundationPose →
   ICP, save (2–4 min).
3. `refine_pose` with `mv_n_views` 5: the planned close views (0.40 m) and the joint refinement
   (2–3 min). It already saves `multiview/<stamp>/` (organized clouds, camera poses, extrinsic
   sha1).
4. **These 5 views are the record.** They sit at the near end of the synthetic standoff range
   (300–1200 mm), because the camera is on the wrist. This is stated as such, and the twins use
   the same camera poses.
5. `label_real_scan.py` builds the truth:
   - the measured primitives at the refined poses;
   - weldgen's D4 rule, giving `scene.json` and `seams.npz`;
   - one `cloud.npz` per view (`tier: 3`) and one fused multi-view cloud, with `object_id` per
     point from the parts' true-shape distance (`part_shapes`, within 3 mm);
   - `provenance.real`: ICP residuals, the refinement diagnostics, the measured fit-up, how the
     parts were fixed, the alloy and surface condition, and the extension flags;
   - **the flags come from a table** of the generator's draw ranges, each row citing its source
     line. A test samples every family and checks that each draw falls inside its row. A value
     is flagged only when it is out of range by more than its measurement uncertainty.

**One folder per configuration, three tiers in it**, the layout the rendered set already uses.
`render_tier2.py` writes `views/<k>/` and `render.json` beside the scene's tier-1 files, which it
only reads. A real configuration's folder holds:
- **tier 1, the analytic model:** `scene.json` (every part as a primitive with its measured
  parameters and its pose; the cameras; the tacks) and `seams.npz` (the exact seams). Next to
  them goes `cloud.npz`, the analytic twin cloud from the real camera poses;
- **tier 3, the real frames:** `real/<k>/`, with the same file names as `views/<k>/`: `rgb.png`,
  `depth.png`, `depth_valid.png`, and `mask_seam`, `mask_tack` and `mask_object` projected from
  the truth by the generator's own mask code. `view.json` holds K and the camera pose from the
  kinematics and the extrinsic;
- **`real.json`:** provenance and the measured uncertainty;
- **tier 2, later:** the renderer can render the same folder into `views/<k>/` (the tier-2 twin)
  without touching the other two.

A loader written for the rendered set therefore reads the real set unchanged, and every real
frame comes with exact masks.

**The truth's uncertainty is measured on subsets, not assumed:**
- **Repeatability:** on ~10 configurations, capture and register again without touching the
  parts. How far the seam moves is the repeatability.
- **Touch bound:** on ~6 configurations, the pen touches 6–10 points on each part's faces near
  the seam. Their distance to the registered surfaces is the bound. This is the cell's existing
  contact tooling, used for evaluation only, which replaces the AprilTag board of the old plan.

## Software to write first (days 2–4, in parallel with the cutting)

0. **`export_cnc_dxf.py`:** the generator's outlines (plate strata at t = 3, edge pairs at the
   thin steel's measured t) and the butt-arc pairs, nested into 600×600 blanks. It writes DXF in
   mm: outlines on a cut layer, seam edges on their own layer, part ids on a text layer. It also
   writes a parts table with their primitives. ezdxf is not installed, so the script writes
   plain DXF itself. Needed on day 1, because the router job is the longest single job.
1. **`make_real_parts.py`:** a table of measured dimensions → weldgen primitives (prism, slab, tube
   with a base cut, a closed rounded-rect band, an arc band). It writes the PLY meshes, the
   registry entries and the server's `CAD_DIR` copies. No CAD drawing at all.
   - The cell's mode A takes prism faces since 2026-10-27.
     - Before, `_face_corners` read slab faces only, and a prism's side face raised
       `ValueError`.
     - `test_prism_standing_plate_yields_its_two_fillets` covers a prism standing on a slab.
     - Prism corner, butt, lap and edge pairs are untested in the cell: run one of each on
       the pilot day.
2. **The templates script:** the mitre and fishmouth wraps, as PDFs at 1:1.
3. **`label_real_scan.py`**, tested on a synthetic "fake scan" (a tier-1 scene made to look
   like a capture).
4. **A QA view:** the truth seams over the fused cloud, and the residual per part. Reject
   configurations above threshold.

## Schedule (≈ 9 working days)

| day | work |
|---|---|
| 1 | `export_cnc_dxf.py`; a router test cut in the 3 mm 304 (a 100 mm line and an R200 arc, through): minutes per metre, burr and tool wear decide router or laser shop; measure the stock; scan a raw pipe stub and a plate (specularity go/no-go); print the templates and wedges |
| 2–3 | the router job (≈ 10 m of 3 mm outline, ≈ 4 m of thin); the pipe and RHS cuts; deburr and measure; generate the parts and copy them to the server |
| 2–4 | the scripts above, with tests |
| 4 | pilot: 3 configurations end to end (T pipe circle, T line, butt square), through the seven methods |
| 5–7 | capture the 66 configurations, ~22 per day |
| 8 | the repeatability and touch subsets; QA; re-captures |
| 9 | benchmark the real scenes and their tier-1 twins (same primitives, poses and camera poses); the first `notebooks/17_real_subset.ipynb` table |
| later | the tier-2 twins on the desktop (same folders); growing to 10 configurations per stratum |

## Risks, and what to do

- **Specular stainless.** If the raw depth leaves the parts too sparse to register (pilot, day
  1), use a removable matte scanning spray and record `surface: sprayed`. Scan a raw subset (one
  configuration per stratum) anyway: the drop-outs on metal are themselves a result.
- **Hand-cut accuracy.** The truth uses the measured cut, so a rough cut costs fit-up quality,
  not truth accuracy. Spare stock allows a re-cut.
- **Saddles.** If fishmouthing by template fails twice, ask the workshop for a pipe notcher, or
  drop the stratum and say so.
- **The router on 304.** Stainless work-hardens where a tool rubs, and a router's light frame
  and fast spindle make rubbing likely: expect shallow passes, cutting fluid and broken bits.
  The day-1 test cut decides. If it needs more than about an hour per metre (≈ 10 m in all) or
  bits keep breaking, send the same DXF to a laser-cutting shop. The polygons have only
  straight edges, so a shear or a grinder can cut them too, measured afterwards; the butt arcs
  are the only parts that need a machine.
- **The thin carbon steel.** It rusts quickly: keep it dry, wipe it before capture, and record
  its surface condition. Check each plate on the table: one that rocks by more than about
  0.5 mm is not a flat slab, so flatten or re-cut it. Thin sheet lifts on a router, so hold it
  down well. A nominal 2 mm sheet can measure over 2.0, the edge-joint limit: the measured value
  goes into the truth, and the flag decides.

## Needed from you before day 1

*Answered on 2026-10-27: the CNC is a router with a bed of at least 600×600, and 1–2 mm
high-carbon steel sheet is available.*

- The size and number of the 3 mm sheets. If they are larger than the bed, they are cut into
  600×600 blanks first (probably two or three).
- The thin steel: which gauges exactly (both 1 and 2 mm?), and the sheet size.
- Which tools cut the pipes and RHS (band saw, metal chop saw, grinder).
- Has the router cut steel before? How long does a job wait?
- Whether a matte spray is acceptable if the raw scans fail.

**The same parts also serve the thesis cell:** registration and tack marking on metal, and the
advisor's one to three real tacks.
