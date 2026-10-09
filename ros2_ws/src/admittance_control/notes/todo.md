# Open work, both projects (2026-10-08)

*Open items only. What was done, why and with which numbers lives in the README (§14
placement accuracy, §15 multi-view refinement, §16 camera depth calibration), in
`notes/thesis_notes.md` (the running log of experiences, newest first) and in the plans:
- `realtime_fp.md` (FoundationPose tracking);
- `aps_transit_plan.md` (the transit planner);
- `multiview_refine_plan.md`;
- `seam_two_modes_plan.md` (perception);
- `pen_marking_plan.md` (motion);
- `curved_parts_print_list.md` (parts to print);
- `weld_generator/notes/dataset_plan.md` + `phase9_plan.md` (dataset).*

## Where we are

Mode A runs end to end on the robot, by gestures, and the pen marks every tack within
**2.3–2.7 mm** (2026-10-06, possibly within the measurement error; ~8 mm on 24 Sep).
The chain:

1. calibrated kinematics; camera depth Tare'd against the robot; extrinsic tilt refined;
2. register (desktop: SAM2 → PPF → FoundationPose);
3. **track the part while it is moved: FoundationPose on the laptop's GPU** (25–29 Hz);
4. **save:** robust mean of the tracker poses at rest, then **ICP on 5 live clouds**;
5. `~/refine_pose`: 4 close views, online camera-offset correction, ISO 5817 fit-up check;
6. seams and tacks (mode A, D4 rule, tackrule-0.1);
7. reachability; transits by **OMPL's AnytimePathShortening (C++)**, spline, TOTG;
   descents with a roll fallback;
8. force-gated touch; dot or stroke.

## Decisions

- **2026-10-02: no touch sensing.** The method is vision-only. Touches are used only to
  calibrate (pen TCP, table plane, Tare distance) and as the independent reference that
  measures the error (`touch_probe.py`).
- **2026-10-02/06: tracking is FoundationPose's** (`tracking_source: fp`, the default).
  - The desktop registers; the laptop tracks.
  - ICP keeps the stationary work: `run_icp`, the ICP at `save_object`, `refine_pose`.
  - No Kalman fusion.
- **2026-10-06: transits by APS** (`transit_planner: aps`, the default), with RRT-Connect
  as the fallback.

## NEXT (2026-10-08): CAD parts for the strata MDF cannot make

The plate strata (T/line, corner, butt square, lap, edge) are MDF cuts. The curved strata
(T/circle, T/ellipse, T/saddle, T/rounded_rect, T/swept_path, butt arc), and butt grooved
(excluded for metal), are 3D printed. The list, sizes and print notes are in
`curved_parts_print_list.md`.

1. Prune the CAD library: near-duplicates and non-weld parts (the list names candidates).
2. Draw the parts (mm, watertight); export PLY (+ STL to print) from the same CAD.
3. Print, then measure the printed dimensions and record them.
4. Into the cell, per part:
   - the CAD into `models/` and the server's `CAD_DIR` (the same name on both);
   - the PPF library rebuilt;
   - a registry entry;
   - one bench cycle.
5. **Mode A for the curved strata** (`curved_seams_plan.md`): steps 1 and 2 done on
   2026-10-09. All six printed parts are in the registry (verified to 0.06 mm), and
   `compute_seams` computes their seams (circle, ellipse, saddle, rounded-rect, band
   fillets) with fit-up and per-point approach. Closed seams get tacks. Next: step 3
   (reachability round a loop, cylinder collision), then the bench, C1 first.

## Benchmarks to run (for the thesis)

- **APS vs RRT-Connect** (`aps_transit_plan.md` step 5): `scripts/benchmark_transits.py` on
  a real session, every transit of the plan + home.
  - Planners: RRT-Connect + shortcut + spline, against APS at 0.5 / 1 / 2 s with 2 / 4
    threads.
  - Per transit:
    - planning time;
    - weighted joint travel;
    - tool-tip path length;
    - how far the tip strays from the straight line;
    - the smallest clearance;
    - TOTG duration;
    - validity calls.
  - Output: a table + JSON, and an entry in `thesis_notes.md`.
- **FoundationPose vs ICP tracking** (`realtime_fp.md` step 5): `pose_jitter_probe.py` on
  `/perception/fp/pose` and `/perception/icp/refined_pose` (still part, slow push): noise,
  delay, drift.
- **PPF pose vs FoundationPose pose** on 3–5 scenes: accuracy and time (perception side,
  the 1 Oct meeting).

## LATER, but important: measure everything on this side of the pipeline

The FoundationPose dockers are measured separately (`realtime_fp.md`, the perception
notes). Here: every stage from the received pose to the pen mark, measured, not
estimated, so the thesis can report them. Per stage:

| stage | what to measure |
|---|---|
| bridge reply → detection | delay |
| tracker pose → `base_link` | TF lookup time; rate actually used |
| `save_object` | robust mean; save-time ICP time per cloud, fitness, rmse, correction |
| `refine_pose` | view planning time; motion time per view; capture time (16 frames); preprocessing; refinement rounds and iterations; d estimate; total |
| `welding_points` | seam + tack time |
| `tack_reachability` | time, IK calls, collision checks |
| `~/plan` | per transit: planner, time, cost, validity calls, path points; descent checks; TOTG time |
| execution | per transit duration, descent / contact time, the whole cycle |

- **Quantities:** time (wall and CPU), peak memory (RSS), rmse / fitness wherever ICP
  runs, and point counts.
- **Complexities:** how each stage scales with points, parts, views and transits, both
  measured and stated.
- **How:** one timing/metrics logger shared by the nodes, writing JSON per run, plus a
  script that tabulates runs.

## Multi-view refinement (`multiview_refine_plan.md`)

- **Anchor the camera offset to the pen-measured table** (the main accuracy lever left).
  - On the 2026-10-06 capture, the table per view against `table_plane.json`:

    | | views 0–3 | spread |
    |---|---|---|
    | raw | +2.5 / −1.6 / +2.6 / +5.3 mm | 6.9 mm |
    | with R_v·d taken out | −1.1 / −3.9 / −1.6 / −0.2 mm | 3.7 mm, mean −1.7 mm |

  - So the online correction is real; dropping its vertical part would make it worse.
    But it leaves the scene ~1.7 mm low.
  - Fix: add the table points (every view sees the table) as an observation, with the
    pen plane as the known geometry, in the joint estimate of d.
  - A looser prior does not help: 3 / 8 / 15 mm changed the result by < 0.5 mm
    (`mv_prior_sigma_mm/deg` are parameters now).
- **Self-calibration bench check:**
  1. 3 or more runs from **different** part arrangements;
  2. `selfcal_extrinsic.py --write`;
  3. promote;
  4. a run with d near 0;
  5. `touch_probe` and the marks.

  - Key the history by scene (assembly hash).
  - The 2026-10-06 15:36 entry in `notebooks/selfcal_history.json` came from
    interpenetrating saves, d = (−5.4, −2.3, −1.1): **remove or mark it.**
- **The base's slide in its own plane** varies 1–3 mm between runs while reported as
  measured (grazing, correlated edge points). A noise model that grows for steep points
  would make the σ honest.
- **Turns vs one combined solve:** keep 2 rounds; switch only if the rounds oscillate or
  depend on the order (watch `rounds_run`).
- **Overlap rule and fit-up gap for curved parts:** the distance to the registry's
  `tube` / `swept_slab` surfaces. Together with mode A for the curved strata.
- **A refusal at `save_object`** when `check_registration` fails.
- **FoundationPose per view** as a second measurement in the refinement (plan step 8).
- **Planner transits on APS** (`multiview.plan_views` still uses RRT-Connect).
- **To try:** viewing distance 0.32–0.35 m; a 30° elevation where reachable; tune
  `mv_max_relative_mm`, the 4° limit, `online_selfcal_min_mm`.

## Accuracy: small items

- **Fixture or clamp the parts.** Magnet-held parts can move ~1 mm between scan, marking
  and touches.
- **The ear's foot measured 0.3–2.9 mm above the base,** consistently, with an ISO level C
  warning. Check with a feeler gauge: real, or a camera effect near the contact?
- **Pen length:** the bare table reads +1–3 mm from the plane at some spots. Re-check the
  4-point TCP (it changes only `pen_tool.json` and the table plane).

## OPEN — motion (`pen_marking_plan.md`, `aps_transit_plan.md`)

- **Box–box clearance** (APS plan step 8). A tool box (the camera body) against a part box
  is only an intersection test (0 / ∞), in both the Python and the C++ models: no warning
  margin. On 2026-10-06 it refused a descent ("0.0 mm"), which the roll fallback now works
  around. Add an exact box–box distance (or FCL) to both, with the equivalence test kept
  green.
- **The 29 Sep tack-0 refusal** ("descent clearance 0.0 mm (overshoot)") is probably the same
  box–box case; check that it plans now with the roll fallback.
- **Holding 1.5 N on metal:** UR's `force_mode_controller` (loaded, inactive), or a sprung
  holder. Strokes still stop with "pen lifted off" on some tacks.
- **Marking overshoot:** `overshoot_m` 5–6 mm until the camera offset is anchored.
- Stroke `seam` mode: first full-seam run on the bench.
- Collision model: the tilted table plane (now `table_z` only); fixture boxes when clamps
  replace magnets.
- Twin: an adapter from the trajectory action to the Isaac joint-command topic.

## OPEN — perception (`seam_two_modes_plan.md`, `realtime_fp.md`)

- **Mode A covers only the 5 plate strata.** The curved ones need:
  - registry `tube` / `swept_slab` entries;
  - pipe-on-plate and pipe-on-pipe from the registered poses
    (`curves.ellipse_from_plane_cylinder` / `saddle_from_cylinders`, the cone test of
    `verify_curved`);
  - new code for a box tube and a curved strip on a plate;
  - motion: full loops around pipes need large wrist rolls.

  Needed for the printed parts above.
- **FoundationPose tracking, small items:**
  - does RViz load our PLY meshes for `/perception/fp/marker`? The green model cloud
    works either way;
  - `use_roi` (off): does the crop raise the rate?
  - symmetric parts (tubes) need `SYMMETRY_INFO` on the server, or the pose may spin about
    the axis.
- **Mode B** (no CAD): PPF no-match, SAM2 per-part masks, region growing + lit-quadric.
- **Mode A+:** per-part sensor points kept at save, labelled by CAD face, lit-quadric
  refinement, fit-up diagnostic.
- Quality field and DP tack selection (thesis stages 1–3).
- **Tack length:** 4t (32 mm on 8 mm plate). Shorter tacks for the cell, or a `tackrule-0.2`
  for both projects?
- **The professor's "walking points" seam method** (1 Oct meeting): two points walk toward
  each other over their own surfaces; where they stop lies near the seam; many pairs fit
  the seam. Prototype it outside the pipeline, step by step and visualised; investigate
  why the edges attract (a proof).
- **Laser pointer** next to the pen, to show a tack before touching it (the 1 Oct
  meeting).

## OPEN — parts, data, paper

- **Phase 9 real subset** (on hold until the metal arrives): `label_real_scan.py`, the view
  planner (`multiview.py` can serve), a fiducial-board pose bound, `d435i_measured`
  (README §16), a 3-configuration pilot.
- **weld_generator:** commit the final ICRA figures/PDF/zip from the desktop; notebook 16; a
  training script on `train_v1`; pin numpy; annotator repeat; lap-overlap citation; the
  advisor's written no-welding scope.

## LATER

- **Torch instead of pen:** force-controlled seam following, the distortion-aware tack
  `order` (already computed).
- **MoveIt planning scene** when fixtures become real.
- **Thesis:** the validation chapter (fit-up, contact depth, README §14's error budget,
  `thesis_notes.md`).
