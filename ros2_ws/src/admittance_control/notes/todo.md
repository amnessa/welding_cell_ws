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
5. **Mode A for the curved strata** (`curved_seams_plan.md`): steps 1–3 done on
   2026-10-09.
   - All six printed parts are in the registry (verified to 0.06 mm).
   - `compute_seams` computes their seams with fit-up and per-point approach.
   - Tacks on curved seams are planned with a roll per tack. In front of the robot: C1,
     E2 and RR1 4/4; SP3 4/6 (concave lobes); S3 on R2 3/4 (far side).
   - Pipes and bands are boxed to within ~1 mm.
   - Step 4, first bench (perception only), 2026-10-09: PPF named every part (R2 by a
     thin 0.040 margin). Mode A did not run (an install-list bug, now fixed). Also
     fixed: re-registration switching to the plate, ICP refused on a pipe's arbitrary
     spin, silent near misses.
   - **Deploy the updated `scripts_in_foundationpose/fp_server.py` to the desktop**
     (`object_name` on re-registration).
   - 2026-10-14, C1 and E2 marked:
     - the far marks are 12–14 mm up the part's wall;
     - the contacts say the registered tube is 8–9 mm off sideways and the plate
       1–3 mm high (`scripts/contact_error.py`);
     - **next: step 5** (multi-view refinement for curved parts).
     - Also fixed: APS "Invalid start/goal" (margin), the Python fallbacks (E2 plan
       478 s → 18 s), no table in the collision model (the touched plane is now used),
       fixtures in `marking.json`, runs archived in `marks_history/`.
   - **Fill `marking.json` `fixtures`** (V-blocks, holders): 3–4 pen-tip points on each
     fixture's top, read off the pendant.
   - Transits round a pipe swung up to 0.9 m off (hitting things ~75 cm out). Fixed:
     - 100 mm departure points;
     - the tool's straight (or bowed) Cartesian line tack to tack;
     - the planned path driven instead of re-planned;
     - APS 3 s.
     - E2 tip paths are now 135–211 mm tack to tack, the same every run.
   - RR1 marked (2026-10-14): all within ~3 mm (two close but not touching the root,
     one 3 mm out on the plate, one 2 mm up the wall), clearly better than the round
     pipes' 8–14 mm.
   - **Multi-view (step 5) is needed for curved parts**: one view pins a round part's
     pose poorly.
   - SP3 marked (2026-10-14): reach 4/6 (the concave sides, as predicted); one no
     contact, three 9.5–12.7 mm up the band. Its contacts are 0–8 mm off: a rotated S,
     and the 20 mm stroke climbing the real wall. **Measure accuracy in
     `stroke_mode:=dot`.**
   - S3 on R2 marked (2026-10-14): 4/4, 1–7 mm and one 11 mm (lifted off). **Step 4
     done** (the bench table is in `curved_seams_plan.md`).
   - **Step 5 built (2026-10-14):** the refinement and the view planner use each part's
     true shape (`part_shapes.py`).
     - Synthetic: C1, E2, S3 on R2 and RR1 all come back to within 0.1 mm, where the old
       boxes refused E2 and S3 on R2.
     - C1 needed only `refine_pose`.
     - **Next: bench it**: `refine_pose` before `welding_points`, marks in
       `stroke_mode:=dot`, C1 and E2 first.
     - 2026-10-15 (SP3): first try found no seam because the plate CAD was replaced
       after the registry was built. Rebuilt; a stale entry is now refused loudly.
       `mv_max_correction_deg` raised to 5.
     - SP3 with `refine_pose` (2026-10-15): fit-up −0.9..10.4 → 0.0..0.3 mm. Marks: 2 on
       the root, one 4.4 mm (was 9.5–12.7 mm without multi-view); contacts 2.8–4.2 mm
       early along the pen (was 0–9.4 mm). The first tack hit a fixture: enter
       the fixtures in `marking.json`. The user is testing multi-view further before
       the other todos.
     - E2 (2026-10-15): `refine_pose` refused for overlap; the acceptance rules changed
       (the user's rule, "apply a correction within the limits"):
       - the overlap must not get worse than the saved assembly's;
       - a pipe's spin is not judged;
       - the bore is hollow.
       Replayed, it is accepted and lowers E2's seam 4.5–5.3 mm onto the table-flush plate,
       which the contacts support if the plate lies on the table. **Re-run E2 with
       `refine_pose`, then mark in `stroke_mode:=dot`.**
     - A cut pipe's spin is unobservable to single-view registration (E2 36° off,
       2026-10-15). `welding_points` now settles it on the cut (`settle_cut_spins`).
       E2 with `refine_pose` + the settled spin: marks 2–3.2 mm, 3–4 mm on the tilted
       side (single view: 3.7–6 mm; step 4: up to 14.4 mm). Maybe
       later: the same inside `refine_pose`.
     - The camera error the views estimate is consistent run to run: d ≈ (0–2, 3,
       −3.3..−4.4) mm (SP3, E2), ~5 mm. Applying it to the extrinsic file
       (`scripts/selfcal_extrinsic.py`; first mark or remove the bad 2026-10-06 15:36
       history entry) would take it out of every single-view registration too.
     - **After replacing any CAD in `models/`: `python3
       scripts/build_weldgen_registry.py --verify`** (and the desktop's PPF library).
   - `contact_error.py` reports bands per contact (no fit yet).

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
- **Single view vs multi-view, per stratum** (the curved bench): marks by hand in
  `stroke_mode:=dot`, plus the pen's contacts (`scripts/contact_error.py`, `marks_history/`)
  for each joint. 2026-10-15 so far: E2 3.7–6 → 2–3.2 mm, SP3 9.5–12.7 → 0–4.4 mm. Also the
  multi-view's own numbers: the correction, the camera error d, the view spread.
- **For the thesis, collect the comparisons the project already made** (the user,
  2026-10-15, "not now"):
  - FoundationPose vs ICP tracking;
  - APS vs RRT-Connect (time, path length);
  - single vs multi-view;
  - box vs true-shape neighbours (synthetic: E2 refused → 0.03 mm);
  - the C++ vs Python collision model (590×, E2's plan 478 → 18 s);
  - the Cartesian transit vs APS (tip path 0.14–0.21 vs 0.26–1.6 m).
  `thesis_notes.md` holds most of the numbers.

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
- Collision model: the tilted table plane and `fixtures` are in (2026-10-14); enter the
  bench's V-blocks and holders in `marking.json` `fixtures`.
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
  **Probably dropped** (the user, 2026-10-15: "too much hassle and not much improvement
  for a robotics-oriented thesis"; not yet decided).
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

## OPEN — human–robot interaction

- **Finger-pointing camera aim, before the first capture** (asked 2026-10-14; BUILT
  2026-10-15, works on the robot; measurements pending).
  - **Built:** `admittance_control/camera_aim.py` (the control law, 14 tests including a
    closed loop on the UR kinematics), `scripts/camera_aim_node.py`, and `/gesture/aim` in
    the gesture node. It was tested against a fake controller manager and arm: 240 px
    (21°) → 10 px in ~3.5 s, centre drift ≤ 4 mm, the joints handed back 0.6 s after the
    hand drops, and Ctrl-C mid-turn stops it. README §E has the details.
  - **On the bench:**
    1. ~~dry run, then live~~: works on the robot (2026-10-15). Everyday start, both nodes
       in one terminal: `ros2 launch admittance_control gestures.launch.py`
       (`aim_dry_run:=true` to only log, `aim:=false` for gestures without the aim);
    2. measure time-to-aim, overshoot and the hand-lost stop (thesis);
    3. check the image's stamp and `/joint_states` share a clock. If they don't, the node
       says so once ("latency not compensated") and still works, with a little
       overshoot.
  - **To fix: the wrist buzzes when the error is small** (seen on the robot 2026-10-15).
    Large turns are smooth; near the centre the wrist makes a buzzing sound.
    - **Probably not the missing I or D term.** A buzz is a high-frequency excitation.
      A D term would amplify the landmark noise, and an I term winds up over the
      latency. A loop that is oscillating would show as a visible wobble at a few Hz,
      not a buzz. Suspects, most likely first:
      1. **50 Hz steps.** The node sends a new velocity every 20 ms and the UR holds
         each one for 10 of its 2 ms cycles. The arm gets a staircase, not a smooth
         curve.
      2. **Tiny velocities.** Near the centre the commands are tiny: the creep at the
         edge of the deadband, and the optical-centre hold pulling back a few mm (a
         2 /s gain on mm of drift). The wrist joints may stick and slip at such speeds.
      3. **Start/stop at the deadband edge.** Hand tremor around the 5 % edge
         starts and stops the motion again and again.
    - **Diagnose first.** Record `ros2 bag record /forward_velocity_controller/commands
      /joint_states /camera_aim/status` during a buzz. Then look at the commands: a
      staircase (1), a stream of near-zero values (2), or on/off toggling (3).
    - **Fixes to try, cheapest first:**
      - once centred, ramp to zero and then send exact zeros: no creep, and the centre
        hold only beyond ~2 mm;
      - below ~0.003 rad/s, command 0;
      - a minimum stop time (e.g. 0.5 s) so tremor at the edge cannot restart it;
      - a lower gain for small errors (gain scheduling);
      - if (1) (the user's verdict on the robot, 2026-10-15: it is): smooth commands at
        the controller's rate. The camera's 30/60 Hz is NOT a limit here: between images
        the node re-expresses the fixed base-frame fingertip ray with the latest joint
        state (500 Hz), and the smoother turns the 30 Hz target steps into a continuous
        rate. The limit is Python: one step (law + collision check) is 2.5 ms, so at
        most ~400 Hz and realistically 125 Hz. Quick test first: `rate_hz:=125.0` on the
        aim node (a parameter already). Options for 500 Hz:
        - a 250–500 Hz interpolating stream (a C++ node or a dedicated thread);
        - short spline segments streamed to `scaled_joint_trajectory_controller`'s
          topic, which interpolates at 500 Hz, with no controller switching either;
        - MoveIt Servo (500 Hz, with its own smoothing filters).
  - **What.** When the gesture node starts, with the robot at home, `Pointing_Up` is a
    live AIM mode: the robot turns the wrist camera to follow the fingertip. The
    operator points the camera at a part in real time. The first `ILoveYou` (capture)
    freezes the frame and hands over to what exists: the FoundationPose bridge, and the
    dwell cursor on the frozen frame.
    - Once a frame is frozen, `Pointing_Up` means the cursor again, so no new gesture is
      needed: the mode follows whether a frame is frozen.
  - **Motion: orientation only.** The camera turns about its own optical centre and its
    position is held.
    - Aiming the optical axis takes 2 rotations (pan, tilt). The third, roll about the
      optical axis, is the over-actuated one: hold it at the home value, so the image
      stays upright.
  - **Control: image-based visual servoing, a filtered P loop, not a full PID.**
    - Error = the index fingertip's pixel (MediaPipe landmark) minus the image centre.
      That error gives an angular velocity about the camera's x and y axes.
    - A gain, a deadband (~5 % of the image, so it stops when roughly centred), a speed
      cap (~15°/s), and acceleration/jerk limits.
    - No integral term: the camera-to-command latency (~70–100 ms) makes it wind up and
      overshoot. No derivative: the landmark is noisy.
    - Smooth the landmark itself: a One-Euro filter at the 30 Hz frame rate (low lag
      when it moves, steady when it is still).
  - **Execution: a streaming interface, not the trajectory action.** Chosen: the velocity
    controller with our own Jacobian. It needs no MoveIt Servo config, and our collision
    model and branch lock run on every cycle anyway. The options were:
    - MoveIt Servo (twist in, joint commands out, with its own singularity and collision
      slowdown and Ruckig/Butterworth smoothing) on the UR driver's
      `forward_position_controller`; or
    - the driver's `forward_velocity_controller` with our own Jacobian solve (6×6, the
      calibrated kinematics).
    - Switch controllers on entering and leaving AIM. `scaled_joint_trajectory_controller`
      must be active again before `refine_pose` and marking.
  - **Safety.**
    - Deadman: no fingertip for 0.3 s → decelerate to a stop. Any other gesture stops it
      too.
    - Excursion limits from home, e.g. ±35° pan and ±25° tilt.
    - Every commanded pose checked with the C++ collision model (µs) and the branch lock.
    - The pendant's speed slider still scales everything.
  - **After it.**
    - The capture happens from the aimed pose: `T_base_cam` comes from TF at the image
      stamp, as now.
    - Decide whether `refine_pose` and marking start from there or from `home_q`. Both
      already plan from the current joints.
  - **Later, option B:** aim where the finger POINTS (a 3D ray from the hand landmarks,
    with depth, onto the table), not where the fingertip is.
  - **Thesis:** a short HRI section. Measure time-to-aim, overshoot, and the hand-lost
    stops.

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


## important
path finding algorithm fins a path which has sections go through the table in actual robot this causes collision and robot stops.