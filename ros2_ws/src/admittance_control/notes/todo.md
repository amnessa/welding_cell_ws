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

## NEXT: FoundationPose live tracking on the bench → `notes/realtime_fp.md`

**Built and wired (2026-10-06).** The desktop registers; the laptop tracks
(`fp_track_server.py` in its container + `fp_tracker_node.py`). Measured: 25–29 Hz,
70–90 ms, re-seed 4 s. `tracking_source` is `fp` by default (2026-10-06; `icp` = the old ICP tracking). Details are in the plan's status section.

Still to do:

1. **First real run:**
   1. `fp_track_server.py` in the laptop container;
   2. `pointcloud.launch.py launch_foundationpose:=true ...` (fp tracking is the default);
   3. register a part and move it by hand; RViz shows the `fp_object` mesh;
   4. `~/save_object`, the second part, `refine_pose`, marks.
2. **Step 5:** FP vs ICP with `pose_jitter_probe.py` (still part and slow push): noise,
   delay, drift. Then the saved pose vs `refine_pose`'s correction.
3. **Check the mesh marker in RViz.** It loads `package://admittance_control/models/<object>.ply`
   at scale 0.001: does RViz's mesh loader take our PLYs?
4. **`use_roi`** (off): measure whether the crop raises the rate.

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

- **Rough transits between tacks on opposite sides of a plate** (2026-10-05). Going from a
  front tack to a back one, the arm swung drastically through the workspace.
  - **Implemented 2026-10-05:**
    - `marking.smooth_transit`: the drawing server's Catmull-Rom spline through the
      shortcut path, every sample re-checked against the collision model, and the
      straight path kept if any sample fails;
    - `TrajectoryExecutor.time_path`: TOTG through `/compute_totg`, every sample checked,
      `marking.time_trapezoid` as the fallback;
    - node parameters `smooth_transits` (true) and `a_joint_rad_s2` (0.5);
    - the TOTG node is started by `pointcloud.launch.py`;
    - tests: `test_transit_smoothing.py`, `test_motion.py`.
  - **Still to do on the robot:**
    1. restart the perception launch (it now starts the TOTG node) and the marking node.
       No rebuild needed: the changed files are symlinked into the install space;
    2. dry-run `~/plan` on a front↔back pair and compare the tip path in RViz;
    3. check the `~/status` line "timed by TOTG";
    4. then run it on the arm.
  - The spline rounds corners; it does not change the route. If the shortcut path itself
    swings around the robot, that remains. Then: best-of-N seeds, or a via pose above
    the assembly.
- **RRT\*-Connect for the transits** (2026-10-05). The smoothed path still deviates too
  much: the spline rounds corners, but the route is whatever the first RRT-Connect
  connection found.
  - Paper: Klemm et al., *RRT\*-Connect: Faster, Asymptotically Optimal Motion Planning*,
    ROBIO 2015 (`weld_generator/papers/RRT-Connect_Faster_asymptotically_optimal_motion_planning.pdf`).
  - The idea, as the paper describes it:
    - RRT-Connect's two trees (start and goal), alternating, with RRT\*'s `Extend*`.
      A new node picks the cheapest parent among its neighbours within
      r = min(γ·(log|V|/|V|)^(1/d), η), d = 6. Then the neighbours are rewired through it
      when that is cheaper.
    - `Connect*` grows the other tree towards the new node with the same `Extend*`.
    - Unlike RRT-Connect it **does not stop at the first connection.** Every node reached
      by both trees is a candidate, and the best path goes through
      argmin over V_a ∩ V_b of cost_a(x) + cost_b(x).
    - Asymptotically optimal like RRT\*, but its first solution comes much sooner. In
      their benchmarks the median first solution came 2–4× sooner; on the maze RRT\*
      solved 21 % of runs and RRT\*-Connect 100 %.
    - The authors implemented it inside OMPL.
  - Implementation plan, in `kinematics.py` next to `rrt_connect`, same `is_valid` /
    `_edge_valid` interface:
    1. **Cost:** joint travel, weighted toward the base and shoulder (they sweep the most
       space), or the tool-tip path length. Decide which matches "deviates too much".
    2. **Anytime with a time budget** (e.g. 0.5 / 1 / 2 s per transit): keep the best
       path found so far and return it when the budget runs out. The first solution is
       an RRT-Connect-quality path.
    3. **Speed tricks for Python:**
       - choose-parent checks the neighbours sorted by cost and stops at the first clear
         edge (lazy collision checking);
       - cache edge results;
       - nearest-neighbour search vectorised in numpy (a few thousand nodes).

       The collision model's `is_valid` is the expensive call, so count calls per transit.
    4. Then the existing chain: `shortcut_path` → `smooth_transit` → TOTG.
  - **Switch criterion:** benchmark on the bench session's front↔back pair and the full
    T-joint plan, against today's RRT-Connect + shortcut + spline:
    - planning time per transit and for the whole `~/plan`;
    - path cost (weighted joint travel, tip path length);
    - the largest tip distance from the parts;
    - collision-check count.

    Switch if a 1 s budget per transit gives a clearly shorter route and the whole plan
    stays within a few seconds. Otherwise keep it as an option (`planner:=rrt_star_connect`).
  - **OMPL checked (2026-10-05): it does not have RRT\*-Connect.** Neither the ROS copy
    (`ros-jazzy-ompl` 1.7.0, C++ only, no Python bindings) nor the PyPI wheel
    (`ompl` 2.0.1) contains it; the paper's OMPL implementation was never merged.
  - **What OMPL does have** that targets the same problem (a short path, anytime, and
    the planner internals in C++):
    - **AIT\*** and **EIT\*** (informed trees): asymptotically optimal and
      bidirectional in spirit. A reverse search from the goal provides the heuristic for
      the forward search; they are OMPL's current best optimizing planners.
    - **BIT\* / ABIT\***, **RRT#**, **Informed RRT\***: asymptotically optimal.
    - **AnytimePathShortening**: runs several planners (e.g. RRT-Connect) in parallel,
      **hybridizes** their paths and shortcuts the result, anytime. It is "best of N" done
      properly.
    - **BiTRRT**: bidirectional, follows a state-cost map; not optimal for path length.
    - `PathSimplifier`: shortcutting, vertex reduction, B-spline smoothing.
  - **Route: the PyPI wheel, not writing it ourselves.**
    - `ompl-2.0.1-cp312-...-manylinux` matches this Python 3.12 / x86-64. It bundles its
      own `libompl.so` and Boost, so it does not touch ROS's 1.7 library.
    - Installing it is an environment change, so decide how: a venv with
      `--system-site-packages` (keeps rclpy), or `pip install --break-system-packages`
      in the container (Noble's system Python is externally managed). Add it to
      `requirements.txt` either way.
    - **Wiring:**
      - a 6-D `RealVectorStateSpace` with the joint limits;
      - state validity = our `CollisionModel.is_valid` (a Python callback; the
        nearest-neighbour search and rewiring stay in C++);
      - motion check at 0.01 rad, as `_edge_valid`;
      - the weighted joint cost without a Python cost callback: plan in scaled
        coordinates q_j·w_j (base and shoulder heavier), so OMPL's own Euclidean
        distance *is* the weighted travel.
    - Then the existing chain: `shortcut_path` → `smooth_transit` → TOTG.
  - **Benchmark** (the switch criterion above), with 0.5 / 1 / 2 s budgets per transit:
    today's RRT-Connect + shortcut + spline vs AIT\*, EIT\* and
    AnytimePathShortening(RRT-Connect).
  - Write RRT\*-Connect ourselves (the plan above) only if none of these meets the
    criterion.
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
