# Phase 9 plan — the real subset (written 2026-09-21)

*Companion to `dataset_plan.md` Phase 9. What the real subset is for, how its truth is
constructed, what to cut and buy, the 120-configuration matrix, the capture protocol,
and the order of work. Ranges quoted below are the generator's, measured on
`bench_phase4` (`scene.json` of all 720 scenes) so that the real parts land INSIDE the
synthetic distribution — that is what makes the real set a test of the benchmark.*

---

## 0. What it is, and the rules it inherits

- **A test set, never training data.** 11 strata × 10 configurations × 5 views =
  550 frames, strictly held out, so the tier-1 → tier-2 → real ablation stays clean.
- **Truth is constructed, never detected — from measured poses.** Each part is a weldgen
  primitive with its *measured* dimensions (calipers, not nominal); the lab pipeline
  (`admittance_control`: D435i → SAM2 → FoundationPose → ICP-to-CAD) registers each part →
  `T_A`, `T_B`; a `scene.json` is built from those poses and the seam is the analytic
  intersection under the same D4 rule as every synthetic scene. The scan is then the
  **tier-3 twin** of a scene that also exists analytically (tier 1) and rendered in any
  alloy (tier 2). No annotator anywhere.
- **Truth quality = pose accuracy, and it is measured, not assumed.** Per scene:
  ICP residual, repeatability (re-scan without touching the parts), and on a subset a
  bound from a fiducial board / calibrated jig. Stored as `pose_uncertainty_mm`; every
  real-subset number is reported with it (`dataset_plan.md` Phase 9 item 2).
- **Depth is the deliverable; RGB is stored (D10), never benchmarked.** Painted MDF vs
  steel texture is irrelevant to every method in the comparison. What DOES differ is the
  **sensor on metal** (active stereo drops out on specular steel, not on matte paint) —
  so the real set is metal, from the lab's inventory, and a flat-target session measures
  the `d435i` constants at the same time (`d435i_measured` profile; never edit `d435i`).
- **Bare joints.** Masks are proposals on an unwelded joint. Parts are fixed by a single
  tack on the far side / underside, out of every planned view, by magnets, or by
  wedges + clamps. The fixture tack is recorded (`fixture: {tack: true, where}`) and is
  fabrication, not the experiment (the no-physical-welding scope stands).

## 1. Generator ranges vs inventory — what fits, what to buy

Inventory (2026-09-21): plates 1 and 2 mm high-carbon steel, 3 mm stainless; square /
rectangular hollow profiles 25×25, 20×40, 30×30, 30×50, 40×60 (wall unknown, likely ≤ 2,5).
**Ordered 2026-09-21 (7 items):** RHS 60×60, 80×80, 50×100; pipe Ø42,4×2, Ø60,3×2,
Ø76,1×2, Ø101,6×2, Ø114,3×2 (1 m each). **Not ordered, by decision:** 6 / 8 mm plate —
the lab cannot process it — so plates stay at 1 / 2 / 3 mm and there are no grooved
butts in the real set.

| stratum | generator range (bench_phase4) | inventory fits | to buy / make |
|---|---|---|---|
| T/line, corner, butt square, lap | plates t 1–12 mm, L 80–600, W 50–250, α 60–120° (T/corner), gap 0–5 mm, polygon outlines | 1, 2, 3 mm ✓ (the thin third of the range — stated) | laser-cut 4–6 non-rectangular outlines from DXF (`outline_uv`) in 2 mm steel and 3 mm stainless |
| edge | t 1–2 mm only | 1, 2 mm ✓ | — |
| butt grooved | t 4–20 mm; V 36 / bevel 16 / U 8; bevel 8–58° | **nothing** — `valid_preps(t ≤ 3) = square` | **EXCLUDED (decision 2026-09-21):** needs ≥ 6 mm plate the lab cannot process. Named in the release notes; the synthetic stratum stands alone |
| T/circle | tube r_out 31–80 (Ø62–160), wall 3–8, length 80–220, on plate 3–12 mm | — | **ordered:** Ø60,3, Ø76,1, Ø101,6, Ø114,3, all ×2 mm wall (wall below the 3–8 range — labelled extension; base plate 3 mm stainless = the range's low edge), cut to 100–200 mm |
| T/ellipse | same tubes, cut at 60–80° | — | same pipes, mitre-cut 60° / 70° / 80° |
| T/saddle | branch r 15–80 on run r ≤ 80, α 60–89° | — | **ordered:** branch Ø42,4×2 and Ø60,3×2 onto runs Ø101,6 / Ø114,3, saddle ("fishmouth") cut — pipe notcher or laser; mostly 90°, two at ~70° |
| T/rounded_rect | closed rounded-rect stiffener, footprint 64–194 mm, corner R ≈ 15, wall 3–8, height 80–220 | 25×25 … 40×60 are **below range** (and thin-walled) | **ordered:** RHS 60×60, 80×80, 50×100 (wall as delivered — record it; if 2 mm, labelled extension like the pipes), cut to 100–200 mm |
| T/swept_path | open stiffener, band 4–10 mm thick, height 50–156, spine arc / B-spline, on plate 220–460 long | — | laser-cut curved strips from **3 mm stainless** (width 60–100 = the height, arc R 150–300 and one S-curve), stood on edge — band 3 mm is below the 4–10 range: labelled extension, or drop the stratum if the strips prove too flimsy to fix |
| butt arc | two plates with matching arc edges, R ≈ 250, band 50–145 wide, t 3–10 | 3 mm ✓ | laser-cut 3 pairs (R 200 / 250 / 300; widths 60 / 100 / 140) from 3 mm stainless (one pair from 2 mm steel as an extension) |

Fit-up hardware: feeler gauges / shims (0,5–4 mm), 3D-printed or machined angle wedges
(60, 75, 90, 105, 120°), magnets, small clamps, a flat fixture plate with an **AprilTag
board** glued at a known offset (the pose-error bound).

**What still has to be cut (from inventory sheet, by laser/plasma from DXF):** the
non-rectangular plate outlines, three arc-butt pairs, three curved stiffener strips; and
for the pipes: mitre cuts at 60/70/80° and the saddle notches. Ask the supplier or the
lab workshop which of these they can do — the saddle cut is the only one that needs a
notcher or a laser.

**The thickness axis is narrow and it is said so.** With 1 / 2 / 3 mm stock the real set
covers the thin third of the synthetic thickness range (1–12 mm), the pipe and profile
walls (2 mm) sit just under the synthetic 3–8 mm, and the stiffener band (3 mm) just
under 4–10. None of this weakens the twin comparison — every twin is constructed from
the *measured* dimensions — but a per-stratum "in range / extension" flag is stored and
reported, and if a reviewer wants an in-range synthetic control, a `bench_real_ranges`
stratum (the generator with these exact ranges) is a config file away.

## 2. Reuse without losing worth — the rule

The benchmark's difficulty axes are fit-up (gap, misalignment, included angle),
thickness, curvature, outline and viewpoint. The synthetic corpus itself varies most of
them on a fixed geometry (that is what a twin is), so **reusing parts across fit-ups and
views is exactly the dataset's own logic**. Worth is lost only when one geometry stands
for a whole stratum — then the held-out-geometry split (D11) degenerates to one part.
Rule per stratum: **≥ 3 distinct part geometries, ≥ 2 thicknesses where the stock allows,
and (plate strata) at least one non-rectangular outline**; 10 configurations = 3 geometries
× 3 fit-ups + 1.

Fit-up vocabulary (mirrors ISO 5817 B / C / D / below-D so each real scene gets a
`quality_level` like a synthetic one): gap ∈ {0, ~1, ~3 mm}; linear misalignment
∈ {0, ~0,15 t, ~0,3 t}; angular ∈ {0, ~1°}; T / corner included angle ∈ {90°, 75°,
110°} (the ISO 17659 T-joint vs angle-joint split); lap overlap two values; edge flush
vs offset. Measure what was set, store what was measured.

## 3. The 110 configurations (11 strata × 10)

| stratum | part sets (3+) | fit-ups → 10 configs |
|---|---|---|
| T/line | 2 on 2; 3 SS on 3 SS; 2 on 3 (one standing plate a laser-cut trapezoid) | α 90/75/110 × gap 0/1/3, +1 with misalignment |
| corner | 2/2, 3/3, 2/3 (one pentagon outline) | α 60/90/120 × gap, +1 |
| butt square | 1/1, 2/2, 3/3 | gap 0/1/3, h 0 / 0,3 t, +1 dissimilar (2 on 3, flush one face) |
| butt arc | R 200 / 250 / 300 pairs (3 mm; one 2 mm pair) | gap 0/1/2, +1 |
| lap | 1/1, 2/2, 2/3 | overlap 2 values × gap 0/1, yaw, +1 |
| edge | 1/1, 2/2, 1/2 | flush / offset × gap 0/1/3, +1 |
| T/circle | Ø60,3, Ø76,1, Ø101,6, Ø114,3 on 3 mm SS (2 mm base as extension) | 4 pipes × 2 bases (+2 with 1 mm gap) |
| T/ellipse | Ø60,3, Ø76,1, Ø101,6 cut 60/70/80° on 3 mm | 3 × 3 + 1 |
| T/saddle | Ø42,4 & Ø60,3 on Ø101,6 & Ø114,3 | 4 pairs at 90° × 2 fit-ups + 2 at ~70° |
| T/rounded_rect | 60×60, 80×80, 50×100 on 3 mm (2 mm as extension) | 3 × 3 + 1 |
| T/swept_path | 3 mm strips R 150, R 300, S-curve on 3 mm / 2 mm plates | 3 × 3 + 1 |

Views: 5 per configuration, **drawn by the tier-1 camera sampler seeded per
configuration** (standoff 300–1200 mm, elevation 15–85° incl. one grazing view, azimuth
uniform, aim jitter 0,15 × span) and executed as robot targets — so the real views follow
the synthetic view distribution and each view's `K`, `T_world_cam` (eye-in-hand
kinematics + the calibrated extrinsic) is the view's twin camera. 110 × 5 = 550 frames.

## 4. Pipeline to build (before any metal is cut)

1. `scripts/label_real_scan.py` (constructs the TRUTH for a scan; nothing is generated): measured part dims + registered poses → weldgen
   primitives → `scene.json` / `seams.npz` under the existing schema (a `real_v1` config
   whose geometry ranges are widened to the measured dims, so `config_id` is honest),
   D4 rule run as-is, `provenance.real = {icp_residual_mm, repeatability_mm,
   fiducial_bound_mm, fixture_tack}`; `cloud.npz` holds the *scan* (xyz + object_id from
   the registered segmentation) with `tier: 3`. Tests on a synthetic "fake scan".
2. `scripts/plan_real_views.py`: per configuration, the 5 camera poses from the tier-1
   sampler → robot joint targets (through the existing IK / drawing pipeline).
3. Fiducial board + pose-error measurement script (repeat scans, board offset).
4. The `d435i_measured` flat-target session (three ranges) → new profile name.
5. **Pilot first:** 3 configurations end to end (one plate joint, one pipe on plate, one arc butt)
   through registration, scene construction, the twin gate against a tier-2 render of
   the same scene, and the seven methods — before the full order is cut.
6. Capture (≈ 15 min per configuration → ~30 lab hours over ~2 weeks), then
   `notebooks/17_real_subset.ipynb`: real vs tier 1 vs tier 2 per stratum, pose
   uncertainty, sensor validity on metal vs the model.

## 5. Stated exclusions

Grooved butts (no plate ≥ 6 mm processable in the lab — and ISO 9692-1 has no bevel
row below it), plates > 3 mm, angled saddles beyond two; and the extensions below the
synthetic ranges (2 mm pipe / profile walls, 3 mm stiffener band, 2 mm bases under
pipes) flagged per scene — each named in the release notes rather than approximated.
