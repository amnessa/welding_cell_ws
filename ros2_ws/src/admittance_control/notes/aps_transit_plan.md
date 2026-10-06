# Transits by OMPL's official AnytimePathShortening (C++): plan

*Written 2026-10-06, revised the same day: the official, multi-threaded APS, not an
APS-like loop in Python. Context: `todo.md` → OPEN — motion.*
- *The marking node's transits come from the first RRT-Connect path. Shortcut, spline and
  TOTG (2026-10-05) smooth its corners, but its route still deviates too much.*
- *2026-10-06 bench run: tack 0's transit had 277 path points and took 14.3 s; tack 2's
  had 161.*

## Status (2026-10-06): steps 1–4 done, APS is the marking node's default

**Built:**
- `src/transit_cpp.cpp` → `admittance_control._transit_cpp`;
- `CollisionModel.export_spec()`;
- `kinematics.NOMINAL_KIN`;
- `marking.aps_path` / `transit_path(planner=...)`;
- node parameters `transit_planner` (`aps`), `transit_budget_s` (1.0),
  `aps_num_planners` (4), `aps_max_paths` (8).

**Measured** (`test/test_transit_cpp.py`):

| check | result |
|---|---|
| FK | equal to the Python frames to 1e-12, nominal and calibrated |
| collision | `is_valid` identical on 2 × 4 400 configurations (random + near contact) in the T-joint scene and a real session with the table; distances equal to 1e-9 (one exact tie resolved differently in the last bit, accepted) |
| speed | 8.6 µs per check vs 5 100 µs in Python: ~590× |
| APS, front↔back T-joint transit, 1 s | weighted joint travel 2.9 vs 17.6–18.0 for a single RRT-Connect (median of 5), i.e. ~6× shorter |
| APS planner threads | CPU/wall 4.8: they run in parallel |
| full plan, 0.5 s budget | the straight moves stay straight; the front↔back transit and home go through APS in ~0.5 s each |

**Two findings on the way:**
- `py::array_t<bool>(n)` filled through `mutable_unchecked` returned every element as the
  last row's verdict. `is_valid_many` now builds the array from explicit bytes.
- The Python `_edge_valid` resolution is in a *weighted* metric (wrists weighted
  0.3–0.5), so it lets a wrist move up to ~0.03 rad between checks. The C++ side checks
  ≤ 0.01 rad on every joint, so it is at least as strict.

**Still to do:**
- step 5, the benchmark on a real session;
- step 7, the bench run;
- step 8, box–box clearance;
- the multi-view planner still uses RRT-Connect (`transit_path`'s default).

## What APS is, and why the Python wheels do not do

**AnytimePathShortening** (OMPL 1.7, `ompl/geometric/planners/AnytimePathShortening.h`):
1. runs several planners (RRT-Connect by default) **in parallel threads**;
2. keeps their solution paths;
3. **hybridizes** them (`PathHybridization`: splices the best segments of different
   paths where they cross);
4. **shortcuts** the result (`PathSimplifier`);
5. repeats until the time runs out.

The result can beat every single seed, and it improves with time.

**Neither Python wheel has the class** (checked 2026-10-06):
- `ompl` 2.0.1 exposes no APS and no PathHybridization;
- `ompl` 1.7.0 exposes PathHybridization but not APS.

Assembling it in Python would also lose the threads: our collision check is a Python
callback, so they would queue on the GIL.

So: **the official C++ APS from ROS's own `libompl` 1.7** (`ros-jazzy-ompl`, already
installed, with `omplConfig.cmake`), with the collision check in C++. Nothing is
pip-installed.

## Design

**D1. A pybind11 module of this package: `admittance_control/_transit_cpp`** (in
`src/transit_cpp.cpp`).
- Not a ROS service: Python calls it synchronously, nothing is serialized, and it is
  unit-testable without ROS.
- `pybind11-dev` 2.11 is installed.
- Built by `CMakeLists.txt` (`find_package(ompl)`, `find_package(pybind11 CONFIG)`,
  `pybind11_add_module`), installed next to the Python modules.
- The **GIL is released** while it plans, so APS's threads run in parallel.

**D2. The collision model in C++, a faithful port of `collision.py`.** The Python model
stays the single source of truth: a new `CollisionModel.export_spec()` hands the C++
side everything it uses, so the two cannot drift apart:

| item | source |
|---|---|
| kinematics: the 6 joint origins (x, y, z, roll, pitch, yaw), nominal or the factory calibration | `kinematics.load_kinematics`, the active `use_kinematics` file |
| the frame chain: Rz(π) base, joint origin · Rz(q_i), tool0 = … · rpy(0, −π/2, −π/2) · rpy(π/2, 0, π/2) | `ur5e_link_frames_params` |
| the 6 arm capsules: base, shoulder, upper_arm, forearm, wrist_1, wrist_2, with `SHOULDER_OFFSET` 0.138 and `ELBOW_OFFSET` 0.007 and the radii | `ur5e_capsules` |
| tool primitives (capsules, boxes) in tool0 | `ToolModel.primitives` |
| scene boxes (centre, R, half) | `scene_boxes` |
| table: `table_z`, `table_exempt` (base, shoulder) | the same fields |
| tool-vs-arm pairs (`self_pairs`: base, shoulder, upper_arm) | the same fields |
| clearance; joint limits | `clearance`; `kinematics.JOINT_LIMITS` |

The distances are the same algorithms as the Python ones, so results match to rounding:
- segment–segment: closed form;
- point–box: in the box frame;
- segment–box: golden-section search to `tol` 1e-4, plus both ends;
- box–box: the separating-axis test (0 or +∞);
- table: the lowest point of the primitive.

`is_valid(q)` = within the joint limits AND min distance ≥ clearance. It is const and
thread-safe: no shared mutable state, so APS's threads can call it at once.

*Known quirk kept on purpose:* box–box is an intersection test, so a tool **box** (the
camera body) gets no clearance against a part box, only "not touching". Fixing that
changes behaviour and gets its own step (e.g. FCL box–box distance) after the port is
proven equal.

**D3. The planning problem in C++** (`plan_aps(spec, q_from, q_to, options) -> (path,
stats)`):
- **State space:** a 6-D `RealVectorStateSpace` in **scaled coordinates** y_j = w_j·q_j,
  with bounds w·joint limits.
  - Path length then *is* the weighted joint travel (`PathLengthOptimizationObjective`).
  - Weights start as (2, 2, 1.5, 1, 1, 0.5): base and shoulder sweep the most space.
- **Validity:** a C++ `StateValidityChecker` wrapping D2's `is_valid` (q = y / w), with
  `DiscreteMotionValidator`.
  - Resolution: no joint moves more than `edge_resolution` (0.01 rad) between checks,
    the same density as `_edge_valid`.
  - In scaled units that is 0.01·min(w), set as a fraction of the space's maximum
    extent.
- **APS:**
  - `num_planners` RRT-Connect planners (default 4; one per thread), each with the same
    `range` our RRT uses (0.15 rad, scaled);
  - `setHybridize(true)`, `setShortcut(true)`, `setMaxHybridizationPath(max_paths)`.
- **Termination:** `timedPlannerTerminationCondition(budget_s)`. APS keeps improving
  until then; an `exactSolution`-only stop is the fallback.
- **Output:** the best path, unscaled, plus stats:
  - solve time;
  - number of solutions and hybridizations;
  - cost of the first solution and of the final path;
  - validity calls.
- **Endpoints:** Python unwraps the goal to the 2π-equivalent nearest the start (our
  `_unwrap_to`; `unwrap=False` for home) and checks the straight edge first, before
  calling C++.
- **Reproducibility:** `ompl::RNG::setSeed` per call. Parallel threads are not fully
  deterministic, so the tests check properties, not exact paths.

**D4. Hand-off to what exists.** The APS path goes to:
1. `marking.smooth_transit` (the spline, every sample re-checked by the *Python* model:
   a second, independent check of the C++ result);
2. then TOTG.

`shortcut_path` is skipped, since APS already shortcut the path.

**D5. Integration.**
- `marking.transit_path(..., planner='rrt_connect' | 'aps', budget_s, num_planners,
  max_paths)`:
  - the straight edge first, as now;
  - if `_transit_cpp` is not built, or APS finds no path, fall back to RRT-Connect and
    log why.
- Marking node parameters: `transit_planner` (`rrt_connect` until the benchmark),
  `transit_budget_s` (1.0), `aps_num_planners` (4), `aps_max_paths` (8).
- All transits are planned at `~/plan`, so the cost is about budget × transits: ~5 s for
  4 tacks + home, reported in the plan log.
- Later: `multiview.plan_views` transits through the same option.

## Steps, each with its check

1. **FK in C++.**
   - Build: CMake for pybind11 + OMPL; `_transit_cpp.fk(spec, q)` → the link frames.
   - Check, `test/test_transit_cpp.py`: for 1000 random q, nominal and the calibration
     file, the C++ frames equal `ur5e_link_frames_params` to 1e-12.
2. **The collision model in C++.**
   - `export_spec()` in `collision.py`; `_transit_cpp.min_distance(spec, q)`,
     `is_valid(spec, q)`.
   - Check, the equivalence test:
     - scenes: the T-joint scene of `test_marking` and today's real session
       (`assembly.json` + `pen_tool.json`, transit clearance);
     - 20 000 random q, plus 2 000 q near contact (sampled along edges that cross the
       clearance boundary);
     - `is_valid` must be **identical**, and min distance and the closest pair equal to
       1e-9 (box–box: same verdict).
   - Report speed: C++ vs Python checks per second (expected ≥ 100×).
3. **APS in C++.** `_transit_cpp.plan_aps(...)`. Checks:
   - every edge of the result passes the **Python** `_edge_valid` at 0.01 rad;
   - the final cost ≤ the first solution's cost;
   - the budget is respected (±50 ms);
   - with `num_planners` 4, CPU use > 1 core;
   - a front↔back transit on the T-joint scene is found.
4. **Integration** into `transit_path` and the node (D5): parameters, fallback, plan log.
   Check: the existing marking tests pass with `planner='rrt_connect'` (unchanged) and
   with `'aps'`.
5. **Benchmark** (`scripts/benchmark_transits.py`) on a real session: every transit of
   the plan + home. Compare today's RRT-Connect + shortcut + spline against APS at
   0.5 / 1 / 2 s and `num_planners` 2 / 4. Per transit:
   - planning time;
   - weighted joint travel;
   - tool-tip path length (FK);
   - the farthest the tip strays from the straight start→goal line;
   - the smallest clearance;
   - TOTG duration.

   Output: a table + JSON; the result goes to `thesis_notes.md`.
6. **Decide** with the criterion in `todo.md`:
   - a clearly shorter route at a 1 s budget;
   - the whole `~/plan` within a few seconds.

   Then `transit_planner` = `aps` by default.
7. **Bench:** dry-run `~/plan` and compare the tip paths in RViz; then on the arm, with
   the front↔back pair first.
8. *(Separate, afterwards)* Box–box clearance for the camera body (FCL or exact box–box
   distance) in both models at once, with the equivalence test kept green.
