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
with no contact.

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

- **Target:** the centre of the combined bounding box of the saved parts. Later, one target
  per seam may be better.
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
  faces in contact with another saved part, found from the poses as mode A's `face_pair` step
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
   `<results>/multiview/view_<k>.npz`, so every run can be replayed offline (D7).

Tracking is paused for the whole run, otherwise the live crop would chase a moving camera.
The previous tracking state is restored afterwards.

### D4. Preprocessing, per view

- Crop to the saved parts' combined bounding box plus 30 mm.
- Apply the table-plane cut (`ground_plane_file`).
- Voxel-downsample at `mv_voxel_m`, 3 mm.
- Estimate normals in the camera frame and orient them towards that view's camera. The ICP
  normal gate relies on this orientation.
- Transform to `base_link`.

### D5. Refinement, per part, against all views at once (joint ICP)

- **Scene:** the stacked views, keeping only points closer to this part's registered
  surface than to any other part's (the ownership rule from `check_registration.py`).
- **Solver:** `icp_point_to_plane` with:
  - `init = pose_static`;
  - model normals and the 60° normal gate;
  - robust Welsch weighting;
  - `max_corr_dist` 10 mm (the start is already within a few mm).
- **View weights:** each view gets equal total weight, so the closest or densest view does
  not dominate. This needs a per-point weight input in `icp.py`, which is a small addition.
- **Several parts:** refine them in `save_object` order. Refining them jointly as one rigid
  assembly with per-part corrections is left for later (open question 3).

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
  - the face check fails.
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

- `mv_dry_run` (default true for the first bench session) plans and publishes the view
  markers without moving; a second call with it false executes.
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
2. **View planner.** `multiview.py`: look-at, IK, collision, coverage score, greedy pick,
   ordering.
   *Check:* unit tests on the bench T geometry: 4 feasible views; both ear faces covered at
   under 60° incidence; the camera never within 0.28 m.
3. **Joint ICP and diagnostics** in `multiview.py`, plus per-point weights in `icp.py`.
   *Check:* a synthetic test with 4 simulated views of the T, each with a different
   camera-side error (an extrinsic translation `d` rotated with the view, and depth bias).
   Single-view poses scatter; the joint pose lands within 0.5 mm of the truth; the `d`
   estimate recovers the injected translation.
4. **Offline driver.** `scripts/multiview_refine_offline.py <results>/multiview/` replays the
   saved views and prints everything the service would.
   *Check:* runs on the synthetic set, then on the first real capture.
5. **Service wiring.** `~/refine_pose` in the ICP node: capture loop, refinement, persist,
   RViz. `check_registration.py` switches to the shared module.
   *Check:* dry run on the robot (views in RViz, no motion), then a live run that returns home.
6. **Bench validation, vision-only with touch as the reference.**
   - `touch_probe` on the base top ×2 and the ear faces ×2, before and after `refine_pose`
     (today: +2.3 / +4.3 and +0.9 / −1.2 mm).
   - Then mark, and record the contact depths and mark errors.
   - Compare single scan vs multi-view. Record the result in README §14.

## Parameters (ICP node, `mv_` prefix)

`mv_view_distance_m` 0.40 · `mv_n_views` 4 · `mv_elevations_deg` [45, 60] ·
`mv_azimuth_step_deg` 30 · `mv_max_incidence_deg` 60 · `mv_n_frames` 8 · `mv_voxel_m` 0.003 ·
`mv_max_corr_m` 0.010 · `mv_max_correction_mm` 10 · `mv_max_correction_deg` 3 ·
`mv_dry_run` true · `mv_v_joint_rad_s` (as the marking node) · `mv_save_views` true

## Open questions

1. One centre target or one target per seam? Per seam gives closer, more oblique views of
   each root, at the cost of more views.
2. Is 0.40 m close enough? At 0.35 m the D435i is still above its minimum, but more
   candidates collide.
3. Refine parts one by one, or as one assembly with the contact constraint (the ear stands
   on the base) as a soft term?
4. Self-calibration: once `d` from D6 is consistent across runs, fold it into
   `T_tcp_to_cam` (with a sidecar note, like the table refinement).
5. FoundationPose (NEXT in todo): it could give a per-view pose as a second measurement in
   the same framework, which leads into the Kalman-filter idea.
