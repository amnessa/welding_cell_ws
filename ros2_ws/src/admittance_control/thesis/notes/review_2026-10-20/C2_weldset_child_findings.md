# WeldSet: findings of the detailed child reviews (condensed, 2026-10-20)

Source: two background readers of the WeldSet reviewer that reported directly to the coordinator
("sensor/visibility/noise deep read" and "curved families, corpora, gates"). All read-only; file:line
relative to /workspaces/welding_cell_ws/weld_generator.

## Code defects (affect data or metadata)
1. EXTERIOR FLAG on thin stacked plates (visibility.py:489-510 `exterior_scan_subsampled`): exact
   grazing-cone test only on the first point per 2 mm voxel (part A first), others copy the nearest
   representative -> plate B's open top face inherits A's buried-interface flag. Measured vs exact per-point
   scan on bench_phase4: edge 12.4 % of points misflagged (worst scene 25.9 %), lap 2.4 % (worst 16.4 %),
   T/line 0.36 %, corner 0.00 %. Upper top face (must be 100 % exterior): edge stored mean 0.601, lap 0.936.
   Corrupts the Task-1 `full_exterior` input for edge/lap; Task 2 (`single`, visible_from_cam) unaffected.
   Exact scan costs ~0.25 s/plate scene (240 scenes in 59 s). -> fix + re-run edge/lap Task-1 on desktop.
2. PLATE FIXTURE TWINS not at the same relative pose (scene.py:250-256): fixture-height draw precedes the
   yaw/tilt draws in substream 3 -> on/off twins get different assembly yaw (seeds 11-13: 28.8 vs 98.9°,
   21.6 vs -129.5°, -170.3 vs 160.5°); camera identical -> viewpoint relative to the joint differs.
   Curved twins (2026-10-19) append the lift after the yaw and are fine. Bears on the fixture-price table.
3. ARC BUTT (family 7) thickness: SweptSlab.thickness_mm = band width W (50-150 mm) used as t_min
   (scene_curved.py:413) -> joint.quality_level "B" for all 60 arc scenes, fit.throat 0.7 W; objects[].
   thickness_mm stores W. facts.csv and the tack rule use the true gauge (tacks._member_gauge_mm): correct.
4. Grooved plates (PreparedSlab/PreparedPrism) occlusion still K = 9 sampled containment (visibility.py:
   235-281), not exact (3 misses in 19 937 hits on random rays: tiny).

## Docs vs code (fix the docs or state correctly in the thesis)
- Substreams 6 (noise) and 7 (render) spawned but unused: noise uses default_rng(scene seed) (noise.py:48;
  scene.py:616) -> strata sharing a seed get correlated noise; render uses sha256(scene_id|render_id).
- camera_raster = one area-uniform pass at scalar rho = (f/s)^2 (s = standoff): uniform density, not
  range/incidence dependent as the docs say. Benchmark is area_uniform anyway.
- Determinism holds only under pinned numpy + trimesh (hashes changed numpy 2.3.5 -> 2.5.3 on 2026-09-11);
  requirements.txt unpinned; SCHEMA 6.2 hash pseudo-code differs (dtype .str, little-endian swap).
- No sensor-profile twins exist in any corpus on disk; presets in PARAMETERS 6 (sensor_ablation,
  fixture_ablation, default, thin_sheet, lab_fixture, annotation_study) don't exist in configs/.
- Grazing: exterior cone 70° nominal but rays at 0/38.5/66.5° (0.55, 0.95 x 70°; 17 rays, phase from
  default_rng(0)); noise dropout 75° (|r.n| >= cos 75°). Not "the same quantity".
- tests/schema_contract.py stale (standalone; 5 positive cases FAIL).
- Version strings inconsistent (SCHEMA header 2.3.0, code 2.4.0, scene.schema.json 2.2.0, __init__ 2.0.0,
  PARAMETERS 2.2.0); generator_version 0.1.0.
- Elevation measured from world XY (not the fixture plane); "uniform_sphere" has no code path (elevation
  uniform in angle, not area-uniform); azimuth/roll/framing/jitter not stored.
- Curvature floor 30 mm applies only to families 6-7 (rounded-rect corners to 8 mm, ellipse vertex ~23 mm).
- Square-butt and edge gaps use the fillet no. 617 machinery, not ISO 9692-1 "≈ t".
- D34 "measured 0.05": 0.05 is the design tolerance/analytic estimate; true deviations up to 0.079 mm on
  tube cut rings, 1.66x on swept offset edges; all far below the 0.25 mm gate.
- D28 on open curved seams: swept_path terminal mass 0.4501 (ungated) > plate threshold 0.4444.
- B-spline tangent is a central finite difference (h = 1e-6), not analytic.
- Curved scenes store no D19 root/gap-mid arrays (nominal only); curved pipeline skips the D21
  watertightness assert (tests/emit_meshes check it).
- MANUAL: "six methods", "460 tests"; README: seven, 564 tests.

## Ranges as coded (for the strata table; replaces phase9_plan §1 where they disagree)
- Plates (config.py): L 80-400 (outline bbox to ~604), W 50-250, t 1-12 (30 % dissimilar), alpha 60-120 (T,
  corner), quality target B/C/D/below_D 25 % each; gap B/C/D U(0, min(617 limit, 2.0)), below_D up to ~5.3 at
  t = 12; h by no. 5071, beta {1,2,4}° per level, below_D x U(1,2); lap overlap U(0.15,0.60) H_B, beta <= 0.4°;
  edge t U(1,2), equal widths p 0.35.
- Grooved: t U(3,20); valid_preps(min t): <=4 square (filtered), 4-8 square/bevel/V, 8-10 bevel/V, 10-12 V,
  >12 V/U; bevel beta 35-60 b 2-4 c 1-2; V (t<=10, row 1.3) 40-60°, b 1-4, c 0.5-2; V (t>10, row 1.5)
  58-62°, b 1-4, c 2-4; U beta 8-12, b 1-3, c 4-5, R 6-9. 2/60 scenes outside their row.
- Circle #2: r U(30,80), wall U(3, min(8, 0.4 r)), length U(80,220), gap U(0.1,1.2) vertical; base plate t U(4,12).
- Ellipse #3: b U(30,70), tilt U(10°,40°) DRAWN, emitted 10.2-30.6° (all 36 seeds > 30.62° rejected:
  acute-side dihedral < ~59.4° -> cone clear 0.80-0.92 < 0.95 -> bisector_blocked; curved verdict has no ±45°
  work-angle fallback, unlike the plate D4 rule); a = b/cos(tilt); gap along the tube axis.
- Saddle #4: R U(40,80), r U(max(15, 0.3R), 0.7R) (<= 56), tilt U(0,30°), offset ±0.4 (R-r); general
  branch-parametrised quadratic (z = sqrt(R^2 - r^2 sin^2) only for tilt = offset = 0).
- Rounded rect #5: w,h U(60,160), r_c U(8, 0.25 min), wall min(U(3,8), 0.8 r_c), H U(80,220);
  L = 2(w+h) - (8 - 2 pi) r_c.
- Swept #6: span U(120,300); arc R U(max(30, span/pi), 400) or clamped cubic B-spline (4-5 ctrl pts,
  kappa <= 1/30), 50/50; band t U(4,10); height U(50,160) (NOT 50-156); welds = offsets ±t/2 (spine is mid-material).
- Arc butt #7: R U(100,400), span U(100, min(300, 0.9 pi R)), gap U(0.3,2.0) (the drawn arc is the GAP
  CENTRELINE; plate edges at R ∓ g/2), t_A U(3,10), t_B = t_A p 0.7, W min(U(50,150), 0.6R).
- Bores: bore_min_diameter_mm = 80 [ours]; 95 of 240 bores weldable inner fillets (circle 43, ellipse 31,
  saddle 0, rounded rect 21).
- Corpora: bench6b 300 (511 attempts); bench_phase4 720 (1097 attempts, laptop; curved strata pre-visfix
  here); bench_phase4_fx 634 = 274 plate + 360 curved (only 274 on the laptop); train_v1 3600 (desktop only).
  Rejects per source (laptop): ellipse 60/107 (47 NoVisibleSeams), corner 60/128, butt square 60/118, ...
- D28 gate history: 0.33 PASS (08-27), 0.34 (08-31), bench6b 0.3782, bench_phase4 0.45 FAIL (09-03, the
  rectangular grooved plates), 0.3365 PASS after PreparedPrism (09-07); threshold 2 x 20/90 = 0.4444.

## Sensor model as coded (for 4.7)
- Profiles: d435i b 50 mm, f 674 px (640/tan 43.5°), sigma_d 0.08 px, z_min 280; stereo_good 120/1100/0.05/200;
  stereo_poor 35/450/0.15/400; all 1280x720, pinhole, principal point at centre, no distortion.
- Noise (applied at test time, D14): X~ = X + e0 sigma_z(z) r_hat + sigma_l(z)(e1 x_c + e2 y_c),
  sigma_z = sigma_d z^2/(f b) along the unit view ray; sigma_l = 0.8 z/f px-equivalent along image axes;
  valid iff |r_hat . n| >= cos 75° and |z| > z_min; noise axis k in {0,1,2} x (sigma_d, sigma_l); k = 0 no dropout.
- Visibility: frustum + blind zone (z > z_min) + front-facing + unoccluded (exact slab/convex-prism clip;
  Moller-Trumbore entry-only on the <= 0.05 mm chord mesh for tubes and swept parts with a 0.25 mm self-hit
  floor; K = 9 sampling for grooved plates).
- Pose: standoff s = clip(f E/(phi min(W,H)), max(z_min, 300), 1200), framing phi U(0.35,1.45), E = longest
  world-AABB side; elevation U(15,85) from world XY; azimuth U(0,360); roll U(-15,15); aim jitter 0.15 L in a
  3-D cube. Benchmark median standoff d435i 353 mm (36.7 % at the 300 clamp), stereo_good 602, stereo_poor 400.
- Density U(0.25,4) pts/mm²; spacing 1/sqrt(rho) is lattice-equivalent (mean NN ~ 0.5/sqrt(rho)).
- Exterior flag: 17-ray grazing-cone escape test on a 2 mm voxel subsample (the defect above).
