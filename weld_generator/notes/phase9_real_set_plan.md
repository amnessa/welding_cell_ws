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
- **304 is (practically) non-magnetic.** Magnets do not hold it. Fix parts with clamps, wedges,
  putty or double-sided tape, and clamp the base plate to the table.
- **The walls (2 mm) sit below the generator's 3–8 mm; Ø42.4 and Ø60.3 sit below its pipe
  radius (31–80 mm).** Flag them as labelled extensions per scene, as `phase9_plan.md` §1
  already says.
- **Stainless is specular** for an active-stereo camera. Day 1 decides whether it is scanned
  raw (see Risks).

## Scope: 11 strata × 6 configurations = 66 configurations, 330 views

- **The 6 configurations:** 3 part geometries × 2 fit-ups (nominal, and off-nominal by gap or
  angle). That is the smallest matrix that keeps the "≥ 3 geometries" rule (the held-out
  geometry split) and one fit-up axis.
- **Growing later:** to the original 10 per stratum, by adding fit-ups to the same parts. No new
  cutting.
- **Excluded:** butt grooved, as decided on 2026-09-21.
- **Butt arc is in (2026-10-27):** its plate pairs are cut on the CNC, together with every other
  flat part (below).
- **Plates are polygons, as in the corpus (D28).** In `bench_phase4`, none of the 180 corner, butt
  and edge plate scenes has a rectangular part, and each of the 120 T-line and lap scenes has
  exactly one (the lower plate). Rectangles let "the seam runs parallel to a plate edge" pass for
  seam finding, the shortcut D28 removed on purpose. A rectangular real set would give that
  shortcut back to the methods that use it, and confound real vs synthetic.
  - With a CNC a polygon costs the same as a rectangle, so every flat part except the T/lap base
    B1 is cut from the **generator's own outlines** (DXF from `outline_uv`).
  - Every T and lap placement also gets a measured in-plane yaw, the corpus's other mechanism.

**Every part is used several times:**

| part (cut from) | strata it serves |
|---|---|
| base plate B1, 300×300×3 | T line, T pipe circle, T pipe cut, T rect. tube, T swept: ~30 configurations |
| polygon pairs A, B, C (each with a matching seam edge) | corner, butt square and edge: the same two plates, placed three ways; one plate of a pair is the upper part of a lap on B1 |
| pipe stubs, square-cut | T pipe circle |
| mitred stubs | T pipe cut |
| the Ø101.6 run + fishmouthed branches | T saddle |
| pipe rings cut to arcs | T swept stiffener: the pipe *is* the arc, so no rolling and no laser |
| RHS stubs | T rect. tube |

## Cut list (≈ 25 cuts, most by hand)

| stock | pieces (mm) | left over |
|---|---|---|
| Ø101.6 pipe | C101 stub 120; E101 mitre (tilt 10°) ~140; R101 run 250; 2 rings × 70 → a half ring and a 120° arc | ~300 (spares) |
| Ø76.1 pipe | C76 stub 120; E76 mitre (tilt 20°) ~150; ring 70 → half ring; S76 branch 120 (fishmouth to Ø101.6; r/R 0.75 = extension) | ~520 |
| Ø60.3 pipe | C60 stub 120; E60 mitre (tilt 30°) ~160; S60 branch 120 (fishmouth to Ø101.6) | ~550 |
| Ø42.4 pipe | C42 stub 120; S42 branch 120 (fishmouth to Ø101.6) | ~700 |
| RHS 60×60, 80×80, 50×100 | RR60 100, RR80 120, RR50 120 | ~850 each |
| 3 mm sheet, **on the CNC** from DXF | B1 300×300 (rectangle, the only one); P1, P2, P3: three standing polygons for T line; pairs A, B, C: three polygon pairs with matching seam edges, outlines drawn by the generator at t = 3 inside the sheet; butt-arc pairs R200, R250, R300 | — |

**The flat parts come from the generator, so each real part is a corpus part:**
- A script runs the generator's own layout code at t = 3 for the plate strata and the butt arc,
  keeps outlines that fit the sheet, and writes them as DXF.
- The outlines are convex, so there are no inside corners to round. The butt-arc concave edge
  (R ≥ 200) is far larger than any cutter radius.
- Measure after cutting, as for the hand-cut parts.

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
| T pipe (circle) | C42, C60, C76, C101 on B1 | each at gap 0 (4); C60 and C101 at gap ~1 (2) |
| T pipe (cut) | E60, E76, E101 on B1 | each at gap 0 and ~1 |
| T saddle | S42, S60, S76 on R101 | each at gap 0 and ~1 |
| T rect. tube | RR60, RR80, RR50 on B1 | each at gap 0 and ~1 |
| T swept stiffener | half ring Ø101.6, half ring Ø76.1, 120° arc Ø101.6, on B1 | each at gap 0 and ~1 |
| corner | polygon pairs A, B, C | A 90°, A 75°, B 90°, B 110°, C 90°, A 90° with gap 1 |
| butt square | polygon pairs A, B, C | A gap 0 / 1 / 3, A misaligned ~1 mm (0.3 t), B gap 0, C gap 1 |
| lap | one plate of A, B, C on B1, each at a measured yaw | A overlap 30 / 60, A 30 with a 1 mm shim, B 40, B 40 + shim, C 40 |
| edge | polygon pairs A, B, C | A flush / offset 5 / gap 1, B flush / offset, C flush (3 mm: extension, since the range is 1–2 mm; 1–2 mm steel plates if they are still there) |
| butt arc | R200, R250, R300 pairs (CNC) | each at gap 0 and ~1 |

**Fit-up:**
- gaps from feeler gauges, angles from the printed wedges;
- **measured afterwards** (feelers at both ends, a protractor) and stored as measured.

**Speed-ups at capture:**
- **Swap parts, not bases.** Change only the standing part while B1 stays clamped, and reuse the
  base's registration.
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
     parts were fixed, and the extension flags.

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

0. **`export_cnc_dxf.py`:** the generator's outlines (plate strata, t = 3) and butt-arc pairs,
   chosen to fit the sheet, written as DXF, plus a parts table with their primitives. This is
   needed on day 1, because the CNC job is the longest external dependency.
1. **`make_real_parts.py`:** a table of measured dimensions → weldgen primitives (prism, slab, tube
   with a base cut, a closed rounded-rect band, an arc band). It writes the PLY meshes, the
   registry entries and the server's `CAD_DIR` copies. No CAD drawing at all.
2. **The templates script:** the mitre and fishmouth wraps, as PDFs at 1:1.
3. **`label_real_scan.py`**, tested on a synthetic "fake scan" (a tier-1 scene made to look
   like a capture).
4. **A QA view:** the truth seams over the fused cloud, and the residual per part. Reject
   configurations above threshold.

## Schedule (≈ 9 working days)

| day | work |
|---|---|
| 1 | `export_cnc_dxf.py`, then the CNC job (every flat part, butt-arc pairs included); measure the stock; scan a raw pipe stub and a plate (specularity go/no-go); print the templates and wedges |
| 2–3 | cut, deburr and measure; generate the parts and copy them to the server |
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
- **Edge joints in 3 mm** lie outside the generator's 1–2 mm. Use the 1–2 mm steel plates if
  they are still in the lab; otherwise flag them as an extension.

## Needed from you before day 1

- The size and number of the 3 mm sheets. Are the 1 mm and 2 mm steel plates still there?
- Which tools cut the pipes and RHS (band saw, metal chop saw, grinder). The CNC covers the
  flat parts: which machine it is (laser, plasma, waterjet, router), its bed size, and how long a
  job waits.
- Whether a matte spray is acceptable if the raw scans fail.

**The same parts also serve the thesis cell:** registration and tack marking on metal, and the
advisor's one to three real tacks.
