# Open work, both projects (2026-10-02)

*Open items only. What was done, why and with which numbers lives in the README (§14
placement accuracy, §15 multi-view refinement, §16 camera depth calibration), in
`notes/thesis_notes.md` (the running log of experiences, newest first) and in the plans:
`multiview_refine_plan.md`, `seam_two_modes_plan.md` (perception), `pen_marking_plan.md`
(motion), `weld_generator/notes/dataset_plan.md` + `phase9_plan.md` (dataset).*

## Where we are

Mode A runs end to end on the robot, and the pen marks every tack within **~2.5 mm**
(2026-10-02; from ~8 mm on 24 Sep). The chain:

1. calibrated kinematics;
2. camera depth Tare'd against the robot, and the extrinsic tilt refined;
3. register;
4. `~/refine_pose` (4 close views, online camera-offset correction, fit-up gap check);
5. seams and tacks;
6. elbow-up collision-free motion;
7. force-gated touch;
8. dot or stroke.

## Decisions (2026-10-02)

- **No touch sensing.** The method is vision-only. Touches are used only to calibrate (pen
  TCP, the table plane, the Tare distance) and as the independent reference that measures
  the error (`touch_probe.py`); never inside the cycle to correct a tack.
- **Tracking moves to FoundationPose.** ICP is CPU-bound and too slow to track a moving
  part. It stays for what it is good at: refining a stationary part (`~/run_icp` seed,
  `~/refine_pose`). No Kalman fusion for now.

## NEXT: FoundationPose live tracking in ROS and RViz → `notes/realtime_fp.md`

The plan, with speed as the priority. Steps:

0. Benchmark `track_one` alone on the host GPU and on the laptop's RTX 4060; this decides
   where tracking runs.
1. Check the Tailscale path: direct, not via DERP.
2. A WebSocket tracking endpoint in `fp_server.py`.
3. `fp_tracker_node.py`:
   - latest-wins streaming;
   - region-of-interest crop;
   - robot-motion compensation;
   - TF + latched mesh marker in RViz.
4. Lost detection and automatic re-seed.
5. Compare with ICP (`pose_jitter_probe`).
6. `tracking_source: fp` in the ICP node, feeding `save_object`.

Targets: ≥ 15 Hz (goal 30) and ≤ 100 ms delay (goal 50).

Open questions, in the plan:
- Is the GPU host on the same LAN?
- Which GPU is in the host?
- Does the camera node really reach 30 Hz?

## Multi-view refinement: what is left (`multiview_refine_plan.md`)

All of the plan's open items live here now; the plan keeps the design and the history.

- **Vertical over-correction, about 3–5 mm** (contacts still 3–6 mm early).
  - Cause: the direction that moves every view up or down together is the one the 45°/60°
    views separate worst.
  - Fix: the online correction should apply only the horizontal part of `d`, and the height
    comes from the pen-referenced table. `extrinsic_check` agrees with the pen to ~2 mm
    across the range, with a constant +2 mm still to remove.
- **Self-calibration (plan step 7):**
  - Bench check still to do:
    1. 3 or more real runs from **different** part arrangements;
    2. `selfcal_extrinsic.py --write`;
    3. promote;
    4. a run that shows `d` near 0;
    5. then `touch_probe` and the marks.
  - Key the history by scene (assembly hash), so repeats of one arrangement count once.
  - Don't promote the runs so far: they carry the vertical bias and are one-scene repeats.
  - Promote the horizontal part first.
- **The base's slide in its own plane** varies 1–3 mm between runs while the observability
  says "measured" (grazing, correlated edge points).
  - It doesn't move the roots.
  - A noise model that grows for steep points would make the σ report honest.
- **Turns vs one combined solve** (plan decision 3): keep the 2 rounds of taking turns.
  Switch to one solve of all parts together (6N unknowns with the overlap terms) only if a
  run shows the rounds going back and forth, or a result that depends on which part goes
  first. Watch `rounds_run` and the per-round moves in the log.
- **Overlap rule for curved parts** (moved from the plan's "Still open").
  - The rule measures how far one CAD model pokes into another (parts touching each other,
    not the robot).
  - Flat plates are done: a distance to a plane.
  - A pipe standing on a plate needs the distance to a curved surface, i.e. the registry's
    `tube` / `swept_slab` entries.
  - The fit-up gap measurement needs the same.
  - Do it together with mode A for the curved strata, when curved parts arrive.
- **A refusal at `save_object`** when `check_registration` fails (open since 29 Sep).
- **Step 8, FoundationPose per view,** as a second measurement in the same refinement:
  after the live tracking above.
- **Thresholds to keep tuning on real data:** `mv_max_relative_mm` (2), the 4° correction
  limit, `online_selfcal_min_mm`.
- **Viewing distance:** 0.40 m now; try 0.32–0.35 m (D435i minimum 0.28 m, error ∝ z²)
  once the vertical offset is fixed.
- **Elevation spread:** a 30° view would halve the weakest σ but was unreachable at 0.4 m on
  the bench. Retry if the parts move or the distance drops.

## Accuracy: small items

- **Fixture or clamp the parts.** Magnet-held parts can move ~1 mm between scan, marking
  and touches; the touches cannot separate that from the camera.
- **The ear's foot measured 0.5–2.8 mm above the base,** consistently, with an ISO level C
  warning. Check by hand with a feeler gauge: is it real, or a systematic camera effect
  near the contact?
- **Pen length:** the bare table reads +1–3 mm from the plane at some spots. Re-check the
  4-point TCP; it changes only `pen_tool.json` and the table plane, not the extrinsic.

## OPEN — motion (`pen_marking_plan.md`)

- **Tack 0 of the 29 Sep 18:10 session is refused at planning:** "descent clearance
  0.0 mm (overshoot)". Find which pair goes to 0 in the overshoot zone.
- **Holding 1.5 N on metal:** UR's `force_mode_controller` (loaded, inactive) for strokes
  on steel, or a sprung holder. The two-speed descent and gentle stroke are the stopgap;
  strokes still stop with "pen lifted off" on some tacks.
- **Marking overshoot:** tests use `overshoot_m` 6 mm until the vertical camera offset is
  removed.
- Stroke (`stroke_mode:=tack|seam`): first full-seam run on the bench.
- Collision model: tilted table plane, fixture boxes when clamps replace magnets.
- Twin: an adapter from the trajectory action to the Isaac joint-command topic.

## OPEN — perception (`seam_two_modes_plan.md`)

- **FoundationPose tracking instead of ICP tracking:** see NEXT.
- **Mode A covers only the 5 plate strata** (T/line, corner, butt square, lap, edge). The
  6 curved ones need:
  - registry `tube` / `swept_slab` entries;
  - pipe-on-plate and pipe-on-pipe via `curves.ellipse_from_plane_cylinder` /
    `saddle_from_cylinders` from the registered poses, plus the per-point cone test of
    `verify_curved`;
  - new code for a box tube and a curved strip on a plate;
  - motion: full loops around pipes need large wrist rolls.

  Needed before Phase 9's curved strata.
- **Curved parts in the multi-view refinement:** see the multi-view section above.
- **Mode B** (no CAD): PPF no-match, SAM2 per-part masks, region growing + lit-quadric;
  no library save without CAD.
- **Mode A+:** per-part sensor points kept at save, labelled by CAD face, lit-quadric
  refinement, fit-up diagnostic.
- Quality field and DP tack selection (thesis stages 1–3).
- **Tack length:** tacks are 4t (32 mm on 8 mm plate, `tackrule-0.1`). Decide on shorter
  tacks for the cell (ROS parameters) or a `tackrule-0.2` for both projects.

## OPEN — parts, data, paper

- **More CAD parts** with matching MDF/metal pieces: CAD in `models/`, a server PPF entry,
  a registry entry (`build_weldgen_registry.py --verify`), one bench cycle.
- **Phase 9 real subset** (on hold until the metal arrives): `label_real_scan.py`, the
  view planner (`multiview.py` can serve), a fiducial-board pose bound,
  `d435i_measured` (now with the measured depth law, README §16), a 3-configuration
  pilot.
- **weld_generator:** commit the final ICRA figures/PDF/zip from the desktop; notebook 16;
  a training script on `train_v1`; pin numpy; annotator repeat; lap-overlap citation;
  the advisor's written no-welding scope.

## LATER

- **Torch instead of pen:** force-controlled seam following, the distortion-aware tack
  `order` (already computed).
- **MoveIt planning scene** when fixtures become real.
- **Thesis:** the validation chapter (fit-up, contact depth, README §14's error budget,
  `thesis_notes.md`).
