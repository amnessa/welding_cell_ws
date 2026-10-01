# Multi-view close-range pose refinement (`~/refine_pose`) — plan

*Written 2026-09-29, to be implemented in the next session. Context: `todo.md` →
"DECISION — to touch or not to touch", the no-touch route.*

## Why

After today's fixes, the marks land 5–8 mm off the root. Pen touches (`scripts/touch_probe.py`)
split that into terms of 1–3 mm each: pen length, base-plate height and tilt, ear faces. Every
term is at the D435i's own level for a single scan from about 0.5 m.

Two properties of the sensor make multiple close views worth trying:

- The depth error grows roughly with the square of the distance.
- The camera-side errors turn with the view: the extrinsic's translation turns with the
  wrist, and the depth bias points along each view's line of sight.

So several close views from different directions should average most of those errors out,
without touching the parts.

## Where it sits in the pipeline, and the one command

```
run_icp → save_object (each part) → refine_pose → welding_points → tack_reachability → marking
```

```bash
ros2 service call /icp_pose_refiner/refine_pose std_srvs/srv/Trigger
```

One call does everything, with no command per view or per capture:

1. Plan the views.
2. Move to each view.
3. Capture.
4. Refine every saved part.
5. Show the result in RViz.
6. Rewrite `assembly.json` and the SEPC.
7. Return to the scan home.

The reply summarises the result: the correction per part in mm and degrees, the spread
between views, and the extrinsic estimate. `welding_points` then runs on the refined poses,
unchanged.

## Design

### D1. Motion inside the ICP node, through a shared executor

The marking node's `_execute` becomes `admittance_control/motion.py`. It is a
`TrajectoryExecutor` that:

- sends FollowJointTrajectory goals to `/scaled_joint_trajectory_controller/...`;
- runs the wrench watchdog, aborting at `abort_force_n` 8 N on every move;
- refuses a goal whose first point jumps too far from the current joints (the max joint
  jump check);
- supports dry run;
- checks the controller result.

Both nodes use it. The marking node's behaviour does not change: same messages, same tests.
The ICP node adds an action client, a wrench subscription and a `/joint_states`
subscription, each in its own callback group. Its executor is already multi-threaded.

### D2. View planning: `admittance_control/multiview.py`, pure and unit-tested

- **Target:** the centre of the combined bounding box of the saved parts (decided
  2026-10-01: one centre target).
- **Candidates:** look-at camera poses at `view_distance_m` 0.40, with:
  - elevation 45–60° from the table;
  - azimuths every 30°;
  - a roll about the optical axis in 90° steps;
  - camera +z pointing at the target.

  Each camera pose becomes a `tool0` pose through the inverse of `T_tcp_to_cam`.
- **Feasibility for each candidate:**
  - IK on the locked elbow-up branch (`solve_on_branch`, MarkingConfig `home_q`);
  - clear of the collision model (tool with the camera body, the saved parts as boxes, the
    table) at `transit_clearance_m`;
  - target depth between 0.28 m (the D435i minimum) and 0.6 m.
- **Score:** how well the view sees the faces next to weldable seams. For the T, those are
  the base top and both ear faces:
  - count the model points on those faces that face the camera within 60° of incidence and
    are not occluded by another part (a ray test against the registered models);
  - add a bonus for directions not yet covered, i.e. a greedy pick of K views maximising
    joint coverage.

  The seams are not computed until `welding_points` runs, so "faces next to a seam" means
  faces that touch another saved part, found from the poses as mode A's `face_pair` step
  does.
- **Order:** nearest-neighbour in joint space from the current joints, with the transits
  planned by `marking.transit_path` using `transit_model`.
- **Default:** K = 4 views. With too few feasible views (fewer than 2), refuse and say why.

### D3. Capture, per view

1. **Settle:** wait until the joint speed is near 0, then 0.3 s more.
2. **Frames:** grab `n_frames` 8 organized clouds and take the per-pixel median, ignoring
   invalid pixels. This cuts the temporal noise about 3× before any geometry.
3. **Camera pose:** from TF at the cloud's stamp, `base_link ← camera_color_optical_frame`.
4. **Store** `{q, T_base_cam, xyz_median, stamp}` in memory and in
   `<results>/multiview/view_<k>.npz`, so every run can be replayed offline (step 4).

Tracking is paused for the whole run, otherwise the live crop would chase a moving camera.
The previous tracking state is restored afterwards.

### D4. Preprocessing, per view

- Crop to the saved parts' combined bounding box plus 30 mm.
- Apply the table-plane cut (`ground_plane_file`).
- Voxel-downsample at `mv_voxel_m`, 3 mm.
- Estimate normals in the camera frame and orient them towards that view's camera. The ICP
  normal gate relies on this orientation.
- Transform to `base_link`.

### D5. Refinement of touching parts, against all views at once

*No robot touching anywhere in this step. "Touching" here means parts touching each
other in the assembly (the ear standing on the base). The overlap rule below is computed
purely from the CAD models at their registered poses.*

The saved parts touch or nearly touch: that is the point of an assembly. The danger is not
a gross jump. That is impossible: refinement only starts from the saved pose, which is
right to about 3 mm, there is no global search, and D7 rejects corrections over 10 mm or 3°.
The danger is a quiet slide of a few mm: toward a neighbour, or along a direction the data
does not pin down. Four layers guard against it.

**Layer 1. Point ownership: a part never fits another part's points.**

- **Ownership:** each scene point belongs to the part whose registered surface is nearest
  (the rule in `check_registration.py`).
- **Dead band where parts touch:** points within `mv_dead_band_m` (4 mm) of two parts'
  surfaces at once are dropped. That is the strip where two parts meet, such as the ear's
  foot on the base.
  Those are exactly the points that pull both ways, and the weld root does not need them:
  it is defined by the faces either side.
- **Fixed within a round:** ownership is computed from the poses at the start of a round
  (layer 3) and only recomputed between rounds. ICP never changes its own data inside a
  solve.
- **The normal gate separates parts that meet at a right angle:**
  - a base-top point (normal up) cannot pair with the ear's vertical face (normal
    sideways): 90° apart, the gate allows 60°;
  - nor with its hidden bottom face (normal down);
  - the ear's top edge also faces up, but it is 99 mm away, beyond any matching distance.

  For tees and corners, the gate does most of the separating.
- **Parts meeting face to face (lap joints) are the hard case** (in theory: the step 3
  tests found no swap, so the cap below is OFF by default, see step 3). The two top faces are parallel and
  a plate thickness apart, so the gate cannot tell them apart. Safe rule:
  `max_corr_dist` is no more than half the smallest gap between parallel faces of
  different parts. That is 4 mm for 8 mm plates, with a 10 mm cap. It is computed from the
  registered poses (the mode A face pairs) at the start of the run.

**Layer 2. A prior on the correction: move only as far as the evidence supports.**

- **The prior:** each part's solve minimises the ICP energy plus `ξᵀ Σ⁻¹ ξ`, where `ξ` is
  the correction from the saved pose (a 6-vector in se(3)). `Σ` is diagonal, from the
  expected error of the saved pose: `mv_prior_sigma_mm` 3 and `mv_prior_sigma_deg` 1. This
  is a maximum-a-posteriori estimate with the saved pose as the prior.
  - In the Gauss-Newton step it is one extra term: the prior's information matrix, scaled
    to the ICP residual units, added to `AᵀA`, and its pull added to the right-hand side.
  - In `icp.py` it is a new optional argument, `prior=(T0, Σ)`. Without it the solver
    behaves exactly as now.
- **What that buys:** where the views constrain the pose strongly, the data wins. Where they
  do not, the part stays put instead of drifting. For a flat plate seen mostly from above,
  sliding in its own plane and spinning about its normal are fixed only by its edges, and
  so are the weak directions.
- **Report what was measured.** The eigen-decomposition of the data's information
  matrix `AᵀWA`, at the solution and without the prior, shows how well each direction is
  measured. Per part, report the directions whose eigenvalue is below
  `mv_observable_ratio` (0.05) times the largest, as "not measured, held at the prior",
  for example "sliding along the plate's long edge". This goes into `assembly.json`.

**Layer 3. Parts stay physically consistent: no interpenetration, gaps allowed.**

- **One-sided overlap rule:** on each part, sample `mv_overlap_samples` (200) points on
  the faces that touch a neighbouring part (from the mode A face pairs at the start of the
  run). For each, take the signed distance to the neighbour's surface, using its face
  planes, which is exact for the plate parts in the registry. A distance below
  `−mv_penetration_tol_m` (0.5 mm) is penalised quadratically with weight
  `mv_overlap_weight`. A gap costs nothing.
- **Why one-sided:** the fit-up gap is real information. Mode A measures and reports it, and
  welding cares about it. Forcing the parts together would erase it, while overlap is physically
  impossible.
- **Turns ("rounds"), not one big solve.** In plain terms: hold the ear still and fit the
  base, then hold the base still and fit the ear, and repeat, like two people straightening a
  picture frame by taking turns. Each round refines every part against the others held fixed
  (with the overlap rule against them), then recomputes ownership. Repeat until no part
  moves more than 0.2 mm and 0.05°, at most `mv_rounds` times: 2 by default, since the cell registers two parts (decided 2026-10-01). With more
  parts, raise it; the run still stops early once nothing moves. This is block
  coordinate descent: it converges in a few rounds and reuses the single-part solver.
  Solving all parts at once (6N unknowns with the overlap terms between them) stays as
  open question 3, if the rounds oscillate.
- **Order within a round:** the part with the most owned points first, usually the base.
  It is the best constrained and becomes the reference for the parts on it.

**Layer 4. View weights.** Each view gets equal total weight, so the closest or densest view
does not dominate. That needs a per-point weight input in `icp.py`; the Welsch weights
multiply it.

**Solver settings per part:**

- `icp_point_to_plane` with `init = pose_static`;
- model normals and the 60° normal gate;
- Welsch weighting;
- the `max_corr_dist` from layer 1;
- the prior and the overlap rule;
- the view weights.

### D6. Diagnostics, per view

For each part, also register each view on its own, starting from the joint result.

- **Spread:** the translation std and max rotation across views is the camera error of a
  single view.
- **Extrinsic estimate:** fit `t_i ≈ t0 + R_tool_i · d` by least squares over the views.
  `d` is the extrinsic's translation error: the 180° test, now done automatically on every
  run. Report it only. Correcting the extrinsic from it (self-calibration) is left for later.
- **Depth bias:** each view's median signed offset against the joint pose, compared with
  its viewing distance.
- **Face check:** the `check_registration` face check (offset and drift per face) on the
  stacked views. Factor it into `admittance_control/registration_check.py` so the script
  and the node share it.

### D7. Accept, persist, show

- **Reject** a part's refinement, keeping the old pose and saying why, if:
  - the correction exceeds `mv_max_correction_mm` 10 or `mv_max_correction_deg` 3;
  - the fitness is below 0.2;
  - the face check fails;
  - two parts overlap by more than `mv_max_penetration_mm` 1, using the refined poses and
    the exact face planes;
  - the pose between two touching parts changed by more than `mv_max_relative_mm` 2 or
    `mv_max_relative_deg` 1. That relative pose is the fit-up, so it may only change that
    much when its observability (layer 2) says the views measured it. Otherwise the part
    keeps its relative placement from the saved poses.
- **Fallback:** when one part is rejected, it is held at its saved pose and the others are
  refined once more against it, so no accepted part was fitted next to a rejected one.
- **Reply** per part: the correction, the directions that were not measured, the
  fit-up gap before and after, and the largest overlap.
- **Persist:** replace `pose_static` in `self._saved`, rebuild the SEPC from the refined
  poses, and write `assembly.json` with a `refine` block per object:
  - `pose_before`;
  - `n_views`;
  - `correction_mm` and `correction_deg`;
  - the view spread;
  - the extrinsic estimate;
  - the face check.
- **RViz:**
  - the SEPC and the object TFs are republished, so the refined poses appear on the
    existing displays;
  - new latched topic `/perception/icp/multiview_cloud` with the stacked views, coloured by
    view;
  - new latched topic `/perception/icp/multiview_views`, a MarkerArray of the camera frusta
    and view numbers.
- **End:** return to the scan home with the home move that unwinds the wrist.

### D8. Safety

- `mv_dry_run` plans and publishes the view markers without moving. Default FALSE
  (changed 2026-10-01 at the user's request): `refine_pose` moves unless it is set true.
- Every move is collision-planned at `transit_clearance_m` 20 mm and watched at 8 N.
- An abort service, `~/abort_refine`, stops the run and leaves the poses untouched.
- The service refuses when:
  - no part is saved;
  - there are no joint states;
  - the controller is not reachable.

## Implementation steps (next session), each with its check

1. **Shared executor.** Move `_execute` and its helpers into `motion.py`, with the marking
   node using it.
   *Check:* the marking tests pass; a marking dry run behaves the same.
   **DONE 2026-10-01:**
   - `admittance_control/motion.py` `TrajectoryExecutor`:
     - joints, wrench bias and trace;
     - FollowJointTrajectory with the force watchdog, joint-jump gate, abort and dry run;
     - parameters declared under a prefix, '' for the marking node, `mv_` for the ICP node.
   - The marking node keeps its parameter names and messages.
   - `test/test_motion.py` (7 tests, needs a sourced shell); full suite 58 passed, plus 1
     skip without ROS.
   - The old and new nodes, dry-run on the 29 Sep session (plan, 5 × next, home, in an
     isolated ROS domain with fake joint states): identical replies, plan file and marks
     file.
   - Not yet run on the real arm with goals.
2. **View planner.** `multiview.py`: look-at, IK, collision, coverage score, greedy pick,
   ordering.
   *Check:* unit tests on the bench T geometry: 4 feasible views; both ear faces covered at
   under 60° incidence; the camera never within 0.28 m.
   **DONE 2026-10-01:** `admittance_control/multiview.py`.
   - **API:** `ViewConfig`, `look_at`, `surfaces_from_models` / `box_from_model`,
     `seam_targets` (with owner per point), `visible`, `nearest_in_limits`,
     `plan_views` → `ViewPlan`.
   - **Order of work:** visibility first, which is cheap. Then a lazy greedy pick, so IK
     and collision run only on the candidates the pick wants, about 12 of 96.
   - **Choosing among rolls** of a direction: the least joint travel from home.
   - **Rejections** beyond the plan:
     - the elbow nearly straight: a far-side view at 0.78 m reach came out at elbow
       −0.16 rad;
     - joints beyond ±2π after unwrapping: a plain unwrap put wrist_3 at −7.97 rad, so
       `nearest_in_limits` is used instead.
   - **Coverage** is reported against what any candidate can see. 67% of the seam band is
     seeable at all, since the base's underside below the ear is in the band.
   - **On the 2026-10-01 bench T:** 4 views at 400 mm (azimuths 60/300/0/240, elevations
     45/60); 99% of the seeable seam region seen once, 49% by two views or more; 26 s
     including the transits and the way home.
   - **Tests:** `test/test_multiview_views.py`, 6 tests.
3. **Joint ICP and diagnostics** in `multiview.py`, plus per-point weights and the
   `prior=` term in `icp.py`, the ownership/dead band, the overlap rule and the turns
   (D5).
   *Check:* a synthetic test with 4 simulated views of the T, each with a different
   camera-side error (an extrinsic translation `d` rotated with the view, and depth bias).
   Single-view poses scatter; the joint pose lands within 0.5 mm of the truth; the `d`
   estimate recovers the injected translation.
   *Check (touching parts, `test/test_multiview_touching_parts.py`), on the synthetic T:*
   - **Slide held by the prior:** views that see only the base's top face, with the base
     started 3 mm off along its long edge. The prior keeps it within 0.5 mm of its start
     instead of drifting, and the observability report names that direction as not
     measured. Add edge views: the slide is now measured and corrected to within 0.5 mm.
   - **No sinking:** the ear started 2 mm down into the base, with scene points from the
     true geometry. The refined ear overlaps the base by less than 0.5 mm, and a real 1 mm
     gap placed in the truth is kept, not closed.
   - **Dead band:** the base's top-face points near the ear's foot are owned by neither
     part. Without the dead band, the same test pulls the ear toward the base. The test
     shows that difference.
   - **Lap joint:** two parallel plates 8 mm apart. With `max_corr_dist` at half the gap,
     neither plate takes the other's points. With 10 mm, the test shows the swap the rule
     prevents.
   - **Rounds:** converge within the 2 default rounds on the two-part T. Order independence: base-first and ear-first end
     within 0.3 mm of each other.
   - **Rejection:** a refinement that would change the ear-to-base relative pose by 3 mm,
     with that direction not measured, is rejected and the saved pose kept.

   **DONE 2026-10-01:** `admittance_control/multiview_refine.py`,
   `test/test_multiview_touching_parts.py` (8 tests; full suite 72 passed).

   *Deviations from the design above:*
   - **Own solver.** A focused Gauss-Newton solver (`refine_part`) instead of adding
     weights and `prior=` to `icp.py`, which is unchanged. Two reasons:
     - the prior has to act about the part's own centre: a 1° turn about the `base_link`
       origin moves the part about 9 mm;
     - the overlap rule is a second kind of residual.
     It reuses `NNIndex`, the normal gate and the Welsch weights.
   - **Robust weighting starts wider:** at max(3 × median, 90th percentile) of the
     residuals. With the median alone, the few faces that do see a slide (a plate's end
     faces) were weighted away before the first step.
   - **The half-gap matching cap (layer 1) is OFF by default** (`parallel_gap_rule`); the
     gap is still measured and reported.
   - **Fallback (D7):** a rejected part is held at its saved pose, and the others are
     refined once more against it. An accepted part whose fit-up with a held part would
     change beyond the limit is kept as saved too, so the assembly is refined together or
     not at all.
   - **One assembly-level extrinsic estimate**, not one per part. Each part's per-view
     poses are projected onto the directions that part measures (from the translation
     block of its information matrix), and its own offset cancels by subtracting the mean
     over the views.

   *What the synthetic tests showed (claims above corrected):*
   - **No camera error:**
     - every measured direction lands on the truth: base tilt 0.40° → 0.02°, base height
       2.25 → 0.18 mm, the ear's position across its face (the weld root) → 0.01 mm;
     - the base's in-plane slide and the ear's slide along its length are not measured
       and stay at the saved pose, reported as weak.
   - **From 0.4 m at 45–60°, the base's 8 mm edges are never seen:** more than 60°
     incidence. Their slide becomes measured, and is corrected, only with additional 20°
     views. Low views alone see no top face (fitness 0.04) and are rightly rejected.
   - **Extrinsic error d = (3, −2, 1) mm:**
     - the estimate recovers (2.94, −2.17, 1.20);
     - single views scatter by more than 1 mm;
     - the joint fit would change the fit-up by 7 mm partly in barely measured
       directions, so both parts are kept as saved;
     - after taking the estimate out of the views (self-calibration), both are accepted
       and land within 0.5 mm.
     So the joint pose does NOT average out an extrinsic error by itself, because the
     part shared by all views stays. The self-calibration (step 7) is what removes it.
   - **Dead band and ownership change nothing for the T.** Parts meeting at a right angle
     are already separated by the normal gate. The claim above that "the test shows the
     difference" was wrong.
   - **Lap joint:** no swap, even without ownership at 10 mm. The lower plate's exposed top
     lies beside the upper plate, never under its visible top. The half-gap cap would have
     blocked a 6 mm correction that the default makes.
   - **Overlap rule:** the ear started 2 mm into the base ends no deeper than the 0.5 mm
     tolerance, and a real 1 mm gap is kept.
   - Two rounds suffice, and the order doesn't matter (within 0.3 mm).
   - **Not covered yet:** depth bias per view (only the extrinsic translation was
     injected).
4. **Offline driver.** `scripts/multiview_refine_offline.py <results>/multiview/` replays the
   saved views and prints everything the service would.
   *Check:* runs on the synthetic set, then on the first real capture.
   **DONE 2026-10-01 (synthetic; the first real capture comes with step 5):**
   - **New files:**
     - `admittance_control/multiview_capture.py`: the capture format (D3), D4
       preprocessing, `refine_capture` + `format_replay` (step 5 will call these same
       two), and a synthetic renderer;
     - `scripts/multiview_refine_offline.py`;
     - `test/test_multiview_capture.py`: 6 tests; full suite 84 passed.
   - **Capture format:** `<results>/multiview/<timestamp>/` holds `capture.json` (time,
     extrinsic file and sha1, kinematics, table-plane file, per view angles / q /
     T_base_cam), `view_<k>.npz` (organized cloud in the CAMERA frame, so a capture can be
     replayed with another extrinsic) and `assembly.json` (the saved parts at capture
     time).
   - **Script usage:** `multiview_refine_offline.py [capture] [--write] [--record]
     [--set name=value] [--make-synthetic DIR --d x y z]`.
     - `--write` writes `<capture>/assembly_refined.json`; the results' `assembly.json`
       is never touched.
     - `--record` appends `d` to the self-calibration history, only when the capture's
       extrinsic sha1 matches the file in use.
   - **Synthetic captures, bench T, 4 planned views:**
     - no camera error: both parts land within 0.09 mm / 0.08° of the truth (saved
       1.9–3.5 mm / 0.3° off), and `d` comes out at 0.01 mm;
     - d = (3, −2, 1): estimated as (2.99, −1.99, 1.00), and both parts again within
       0.09 mm / 0.03° (via the online correction below).
     - Each replay takes about 6 s.

   *Findings:*
   - **The first synthetic renderer was biased.** It z-buffered surface samples, re-cast
     along the pixel-centre ray, and so pulled every view about 1 mm toward its camera,
     which looked exactly like an extrinsic error along the optical axis. It now ray-casts
     exactly against the part boxes (the plate parts are boxes).
   - **NEW: online self-calibration** (`RefineConfig.online_selfcal`, default on;
     `online_selfcal_min_mm` 0.5).
     - **Why:** with d = (3, −2, 1) on rendered data, the uncorrected joint fit tilted the
       base by a wrong 1.2° and was ACCEPTED. The signs were only visible in diagnostics
       (rms 0.9 vs 0.35 mm, single views spread 4.4 mm).
     - **What it does:** when this run's `d` exceeds 0.5 mm, `R_cam d` is taken out of
       every view and the refinement runs again.
     - **What gets recorded:** the history still receives the ORIGINAL `d`, and the reply
       shows what the corrected views still say (≈ 0).
     - **With it off,** a `d` that large rejects every part ("the views disagree"), and
       both are kept as saved.
   - **The single-view spread** now counts only the directions a part measures (a
     plate's unmeasured slide wandered per view and meant nothing).
5. **Service wiring.** `~/refine_pose` in the ICP node: capture loop, refinement, persist,
   RViz. `check_registration.py` switches to the shared module.
   *Check:* dry run on the robot (views in RViz, no motion), then a live run that returns home.
   **DONE 2026-10-01 up to the live motion:**
   - **New module:** `admittance_control/multiview_service.py` (`MultiviewRefiner`, the
     `mv_*` parameters). It is attached to the ICP node, which gained `~/refine_pose`,
     `~/abort_refine` (in its own callback group), `_apply_multiview` (accepted poses →
     `_saved`, the SEPC rebuilt with each object's `n_points`, `assembly.json` persisted
     with a `refine` block and `pose_before_refine`), and 6 executor threads.
   - **Before any motion:**
     - at least one saved part, and `/joint_states`;
     - the extrinsic in TF (tool0 → camera) must equal `mv_extrinsic_path` (0.1 mm), the
       file the capture records and the self-calibration is keyed on;
     - the calibrated kinematics are loaded.
   - **Motion:** the views are planned from the current joints. The transits go through
     the shared TrajectoryExecutor, with the bias re-measured and 8 N abort. A failed or
     aborted move stops where it is, with no recovery motion.
   - **Capture:** the per-pixel median of 8 FRESH clouds (stamped after the settle), and
     the TF at their stamp.
   - **After:** the capture is saved, then `refine_capture` and `format_replay` (the same
     code as the offline replay). The stacked views go on `/perception/icp/multiview_cloud`
     and the views on `/perception/icp/multiview_views`. A run is recorded for the
     self-calibration only if it determines a direction of `d`.
   - **`mv_capture_here`:** one view from where the arm is, no plan, no motion, then the
     normal path. For testing on live data, or as "refine from here".
   - **Tested live** (a second instance of the node, `icp_mv_test`, writing into a scratch
     copy of the results):
     - the dry run planned 4 views from the live joints, the extrinsic check passed, and
       the markers were published;
     - `capture_here` captured 907k valid pixels, then saved, refined, applied and
       correctly did NOT record a 1-view run.
   - **Still to do on the robot:** `colcon build` (the install space links files
     one by one, so the new modules are not there until a build), restart the perception
     launch, then a live run with `mv_dry_run:=false`.
6. **Bench validation, vision-only with touch as the reference.**
   - `touch_probe` on the base top ×2 and the ear faces ×2, before and after `refine_pose`
     (today: +2.3 / +4.3 and +0.9 / −1.2 mm).
   - Then mark, and record the contact depths and mark errors.
   - Compare single scan vs multi-view. Record the result in README §14.
7. **Self-calibration of the extrinsic translation.** Every run estimates `d` (D6), the
   extrinsic's translation error.
   - Keep each run's `d` with its conditioning in `notebooks/selfcal_history.json`. `d` is
     only measurable when the views' wrist orientations differ enough, so report the
     conditioning of the stack of `R_tool_i`, and skip runs where it is poor.
   - Once at least 3 runs agree, their median correction moves less than 1 mm when any
     one run is left out, and their spread is under 1 mm:
     - write `notebooks/T_tcp_to_cam_selfcal.npy` (the current extrinsic with `d` applied)
       with a sidecar `.json`, like the table refinement: runs used, spread, date,
       kinematics;
     - promote it by hand.
   - The rotation stays the table refinement's job, at least at first.

   *Check:* a synthetic test where an injected `d` is recovered within 0.5 mm from
   3 runs. On the bench:
   - a run after the update shows `d` near 0 and a smaller spread between views;
   - `touch_probe` and the marks improve, or at least don't get worse.

   **DONE 2026-10-01, done before step 4** (the user's choice: it is the part that
   addresses the measured camera-to-robot error).
   - **New files:**
     - `admittance_control/selfcal.py`: history, information-weighted combination,
       agreement rules, `corrected_extrinsic`, `write_selfcal`;
     - `scripts/selfcal_extrinsic.py`: shows the history and the verdict; `--write`;
     - `test/test_selfcal.py`: 6 tests.
   - **The sign:** an error Δ in tool0 appears as `R_cam d` with `d = R_tcᵀ Δ`, so the
     correction is `t − R_tc d`. A chain test confirms it.

   *What changed on the way (the design above was wrong in two places):*
   - **`d` is estimated from points, not from per-view poses.** The first estimator
     registered each part to each view alone and regressed the positions on `R_cam`. Even
     starting from the TRUE poses it returned (7, 13, −15) mm for an injected (3, −2, 1).
     `multiview_refine.estimate_extrinsic` now solves for every part's correction (about
     its own centre, no prior) and `d` together, from all owned points, each residual
     `n · (T p − q + R_cam d)`.
   - **"Determined" is absolute.** `d`'s information is the Schur complement with the
     parts eliminated. A direction counts when its std is at most 0.5 mm
     (`extrinsic_max_sigma_mm`). Directions above that are named and set to 0.
     - A shift that moves every view's cloud straight up or down is the weakest direction:
       for views at one elevation it is indistinguishable from all parts sitting higher.
     - The planned T views (45/60°) still measure it to 0.16 mm. Over 4 noise seeds and 2
       offsets, the actual errors were 0.2–0.33 mm.
     - A first "5% of the largest eigenvalue" rule had thrown this direction away.
   - **The conditioning of the stack of `R_tool_i` is not used.** Each run stores `d` and
     its 3×3 information (1/mm²). Runs are combined as d = (ΣI)⁺ Σ I d, so a direction
     one run can't see is filled in by another. Agreement is judged per run, within its
     own determined directions. Runs with another extrinsic (sha1 of the file) or with
     nothing determined are skipped.
   - **The planner's elevation spread is optional, and off.** A 30° view would halve the
     weakest std (0.16 → 0.08 mm), but on the bench at 0.4 m none is reachable (IK or a
     stretched elbow), and the 30° candidates doubled the planning time. 75° does not help
     either: from there the ear's vertical faces are seen at more than 60° incidence.

   *Synthetic results:*
   - 3 runs of the T with an extrinsic error of (2, −3, 1.5) mm in tool0 give a corrected
     extrinsic within 0.5 mm of the true one. The rotation is untouched, and the note file
     lists the runs used.
   - Uncorrected views refine to an inconsistent assembly, so both parts are kept as
     saved. After taking `d` out, both are accepted and within 0.5 mm.

   *Note for step 4 (real replays):* the 2 mm fit-up rule (`max_relative_mm`) rejected a
   legitimate correction in a test whose saved poses were off by about 3 mm
   independently. Tune it on real data, where saved poses share one scan's error.

   *Bench check still to do:* 3 or more real runs, `--write`, promote, then a run that
   shows `d` near 0, and `touch_probe` plus the marks.
8. **FoundationPose per view (later, after 1–7 are tested).** Send each view's image and
   depth to the server with the known CAD, take its pose as a second, independent
   measurement per view, and compare it with the per-view ICP poses (D6). This is the
   measurement step of the Kalman-filter idea in todo's NEXT. Fuse only if FoundationPose's
   error is smaller than the ICP's or independent of it.

## Parameters (ICP node, `mv_` prefix)

`mv_view_distance_m` 0.40 · `mv_n_views` 4 · `mv_elevations_deg` [45, 60] ·
`mv_azimuth_step_deg` 30 · `mv_max_incidence_deg` 60 · `mv_n_frames` 8 · `mv_voxel_m` 0.003 ·
`mv_max_corr_m` 0.010 · `mv_max_correction_mm` 10 · `mv_max_correction_deg` 3 ·
`mv_dry_run` true · `mv_v_joint_rad_s` (as the marking node) · `mv_save_views` true

Touching parts (D5, D7): `mv_dead_band_m` 0.004 · `mv_prior_sigma_mm` 3 ·
`mv_prior_sigma_deg` 1 · `mv_observable_ratio` 0.05 · `mv_overlap_samples` 200 ·
`mv_penetration_tol_m` 0.0005 · `mv_overlap_weight` (tune; start at 10× the data term's
per-point weight) · `mv_rounds` 2 · `mv_max_penetration_mm` 1 · `mv_max_relative_mm` 2 ·
`mv_max_relative_deg` 1. `max_corr_dist` = min(half the smallest parallel-face gap between
different parts, `mv_max_corr_m`).

## Decisions (2026-10-01)

1. **Target:** one centre point, the middle of the saved parts' combined box. One target
   per seam is not planned.
2. **Distance:** 0.40 m to start. It is a parameter (`mv_view_distance_m`) to lower later;
   the D435i minimum is 0.28 m.
3. **Several parts:** take turns (D5, layer 3). Move to one combined solve only if the turns
   keep going back and forth or depend on which part goes first; the step 3 test shows it.
4. **Self-calibration:** yes, step 7, after the bench validation.
5. **FoundationPose per view:** yes, as a second measurement per view (the CAD is known, so
   it can be asked for a pose from every view), but only after steps 1–7 are tested. Step
   8, not implemented now.

## Still open

- **Overlap rule for curved parts.** This concerns parts touching each other, not the
  robot. The overlap rule measures how far one CAD model pokes into another. For flat
  plates that is a distance to a plane, which is done. For a pipe standing on a plate, it
  needs the distance to a curved surface, the registry's tube and swept-slab entries. Not
  needed until the curved parts arrive (todo: mode A for curved strata).
