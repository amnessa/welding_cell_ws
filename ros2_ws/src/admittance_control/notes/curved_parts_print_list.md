# Parts to 3D print: the strata MDF cannot make (rough list, 2026-10-08)

**Status (2026-10-08).**
- **Printing now** on the Creality K1 Max (300 × 300 × 300 mm bed): **C1, E2, R2, S3, RR1,
  SP3**.
- **BA parts:** laser-cut from MDF.
- **The cell is not building the dataset,** so not every item below will be made: one
  geometry per curved stratum is enough for the cell.
- **The MDF stock is 8 mm:** no plate thinner than 8 mm (the t 4 / t 6 variants of BA and
  G do not apply).

**What MDF covers.** The plate strata (T/line, corner, butt square, lap, edge) are cut
from MDF, and so are the base plates every printed part below stands on.

**What gets printed:** the six curved strata, plus butt grooved:
- butt grooved was excluded for metal (it needs ≥ 6 mm plate the lab cannot process);
- printing makes it possible.

**Ranges.** Every size below is inside weld_generator's own range
(`weldgen/d29.py`, `phase9_plan.md` §2). So a printed part never needs the "labelled
extension" flag the bought pipes needed: their 2 mm walls were below the 3–8 mm range.

**Phase 9 rule per stratum:** ≥ 3 distinct geometries and ≥ 2 wall/plate thicknesses.

*Printer:* Creality K1 Max, 300 × 300 × 300 mm bed, so every part fits.

## The list

| id | stratum | part | key dimensions (mm) | generator range |
|---|---|---|---|---|
| **C1** | T/circle | pipe stub, square end | Ø 62 (r 31), wall 3, length 100 | r 30–80, wall 3–8, L 80–220 |
| **C2** | T/circle | pipe stub | Ø 100, wall 5, length 120 | |
| **C3** | T/circle | pipe stub | Ø 150, wall 8, length 80 | |
| **E1** | T/ellipse | pipe, tilted 15°, base cut flush to the plate | Ø 62, wall 3, length 110 on the long side | r 30–70, tilt 10–40° |
| **E2** | T/ellipse | tilted pipe | Ø 90, wall 5, tilt 25° | |
| **E3** | T/ellipse | tilted pipe | Ø 120, wall 6, tilt 35° | |
| **R1** | T/saddle | run (main) pipe, lying | Ø 120 (R 60), wall 5, length 200 | R 40–80 |
| **R2** | T/saddle | run pipe | Ø 100 (R 50), wall 6, length 180 | |
| **S1** | T/saddle | branch with saddle ("fishmouth") end, 90°, on R1 | Ø 40 (r 20 = 0.33 R), wall 3, length 90 | r 0.3–0.7 R, tilt 0–30° |
| **S2** | T/saddle | branch, 90°, on R1 | Ø 80 (r 40 = 0.67 R), wall 4, length 100 | |
| **S3** | T/saddle | branch, 70° (tilt 20°), on R2 | Ø 50 (r 25), wall 4, length 100 | |
| **S4** | T/saddle | branch, 90°, **off-centre** by 0.3 (R − r), on R2 | Ø 60 (r 30), wall 3 | offset ±0.4 (R − r) |
| **RR1** | T/rounded_rect | closed rounded-rectangle tube | 64 × 64, corner R 12, wall 3, height 100 | w, h 60–160, R 8–0.25 min |
| **RR2** | T/rounded_rect | rounded-rect tube | 80 × 120, R 15, wall 5, height 120 | |
| **RR3** | T/rounded_rect | rounded-rect tube | 150 × 90, R 20, wall 8, height 80 | |
| **SP1** | T/swept_path | open curved band, stood on edge | arc R 150, span 200, band 6 thick, height 60 | span 120–300, R ≥ span/π … 400; band 4–10, height 50–156 |
| **SP2** | T/swept_path | curved band | arc R 350, span 220, band 8, height 100 | |
| **SP3** | T/swept_path | curved band, **S-curve** (B-spline, 5 control points) | span 220, band 5, height 80 | |
| **BA1** | butt arc | plate pair with matching arc edges | R 150, span 150, plate 140 × 100, t 6 | R 100–400, span 100–300, t 3–10 |
| **BA2** | butt arc | plate pair | R 250, span 200, 200 × 120, t 8 | |
| **BA3** | butt arc | plate pair | R 400, span 200, 200 × 140, t 4 | |
| **G1** | butt grooved | plate pair, single-V 60° (30° bevel each) | 180 × 100, t 10, root face 2 | t 4–20, ISO 9692-1 preps |
| **G2** | butt grooved | plate pair, single bevel 45° | 180 × 100, t 12, root face 2 | |
| **G3** | butt grooved | plate pair, U-groove (R 6) | 180 × 100, t 16, root face 2 | |

**Totals:** 22 printed geometries (24 counting pairs as two). Every stratum has 3
geometries and 2–3 thicknesses.

- The base plates for the T-type parts come from MDF: 6 and 10 mm, 200–300 mm long.
- **BA1–BA3** are flat. If the MDF can be cut along a curve (CNC or laser), cut them
  there instead of printing.

## Drawing and printing notes

- **One CAD per part, in millimetres, watertight.** Export the PLY (for the cell and the
  server, the same file name in both places) and the STL (to print) from the same model.
  A PLY that differs from the print places the pose wrong.
- **Datum:** put each part's origin at a meaningful place: the pipe axis at the base
  centre; a band's spine start on the bottom edge. The registry entry (`tube`,
  `swept_slab`) is then easy to write.
- **Colour and finish:** matte, light (white or light grey). Black, glossy or
  translucent filament gives the D435i holes or wrong depth.
- **Strength / flatness:**
  - walls ≥ 3 mm;
  - print the bands (SP*) and plates (BA*, G*) lying flat, so the curve and the bevels
    are exact;
  - pipes stand upright, with the cut end up where possible;
  - a brim against warping on the long run pipes (R1, R2).
- **Measure after printing** (calipers): diameter, wall, height, band thickness. Record
  them with the part, as Phase 9 stores the measured dimensions and the "in range" flag.
- **Symmetry:** a plain pipe (C1–C3) can spin about its axis without the image changing.
  - FoundationPose then needs `SYMMETRY_INFO`, or its pose wanders about the axis.
  - The seam (a circle) does not care, but the tack positions along it do.
  - Optional: a small flat or notch on the outer wall near the top, far from the seam,
    breaks the symmetry. It must be in the CAD too.
  - The tilted, saddle and rounded-rect parts are not fully symmetric.
- **Fixtures:** the pipes and bands stand by themselves on the plate. The saddle runs
  (R1, R2) need two V-blocks; those can be MDF.

## The current library: candidates to remove (the user decides)

- `test_objv3.ply`: the same outer size as `test_objv2.ply` (256 × 250 × 108); hard to
  tell apart from one view.
- `plate.ply` vs `test_objv1_base.ply`: two thin rectangles 30 mm apart in one edge.
- `assembly_mesh.ply`: written by `~/export_mesh` and sitting in `CAD_DIR`, so the
  classifier treats it as a part.
- `Power Drill-ply.ply`: a scanned object, not a weld part (20 MB, the slowest to load).
- The composites `test_objv2.ply`, `test_objectv1.ply`: whole assemblies as one part.
  Keep them only if classifying an assembled pair is wanted.

## After printing, per part (into the cell)

1. `models/<name>.ply`, and the same file in the server's `CAD_DIR`; rebuild the PPF
   library.
2. A registry entry in `models/weldgen_objects.json`: `tube` / `swept_slab` (hand-written)
   or a slab (`build_weldgen_registry.py --verify`).
3. **Mode A for the curved strata** (`todo.md`, perception) before the cell computes their
   seams; until then registration and tracking work, but `welding_points` falls back.
4. One bench cycle: register → track → save → `refine_pose` → seams.
