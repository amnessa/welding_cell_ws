# Pen-marking the tacks: approach, touch, mark (plan, 2026-09-22)

*The next step after mode A. Input: `welding_tacks.json` (tack point, segment, approach
axis, `tack_no`, `order`) in `base_link`. Output: a pen mark on every tack, and a record
of where the pen actually touched. The touch record is the fiducial-independent check the
whole pipeline is validated against (`seam_two_modes_plan.md`, "Validation"). No arc,
no heat, no puddle: the literature report's thermal arguments (work/travel angles,
gravity, distortion sequencing) do not apply here; its motion hierarchy does.*

## What we have

| Piece | Where | State |
|---|---|---|
| Tack points, segments, approach axis (cleared bisector), numbering | mode A, `welding_tacks.json` | done, on the bench |
| Registered parts as boxes at their poses | `weldgen_objects.json` + `assembly.json` | done: the collision geometry of the workpieces is already here |
| FK / damped-LS IK / RRT-Connect / Catmull-Rom / TOTG timing | `admittance_control/kinematics.py`, `totg_service_node` | proven on the real robot by the drawing stack |
| Phase state machine RETRACT → RRT → DESCENT → DRAWING → ASCENT | `drawing_action_server.py` | proven, but for one orthogonal tool on a plane |
| 1-DoF admittance along the tool axis, path following in X/Y | `cartesian_admittance_controller.py` → MoveIt Servo | exists; tuned for a plane, force target −3 N |
| Approach a point along a ray with IK + joint-jump gate | `move_to_object_node.py` | exists (raw test node) |
| Collision checking in the RRT | `kinematics._is_valid` | **joint limits only** — the gap |
| Pen TCP | `tool_length_m` = 0.19 (move_to_object), `orthogonal_tool_length_m` (drawing) | needs a measured value for the pen |

## The motion, per tack (the four phases, as the literature does it)

```
      P_app = point + d_standoff · approach          d_standoff = 35 mm
  1. transit   RRT in joint space, current q → q(P_app)        collision-checked
  2. approach  LIN along −approach, 20 mm/s, force watched     stop at contact (|F| > F_touch)
                                                               or at point + 3 mm overshoot → "no contact"
  3. mark      hold F_hold along the pen axis (admittance), draw p0 → p1 (the tack length)
  4. retract   LIN along +approach to P_app; next tack: arch transfer if same assembly
```

The pen axis is the tack's `approach` (the cleared bisector: 45° to both plates of a
fillet, so the tip lands in the root). The pen is axisymmetric, so the **roll about the
pen axis is free** — that one degree of freedom is what we spend on reachability and on
keeping the arm elbow-up, never a branch flip.

Contact is the measurement: the tip position at `|F| > F_touch` along the approach line,
compared with the commanded tack point, is the seam-root placement error along the
approach axis. The lateral error comes from the mark itself (photo or caliper against a
scribed reference line). Both go into `tack_marks.json` next to the fit-up numbers.

## Options

### A. Motion-planning backend

| | A1. Extend our stack | A2. MoveIt 2 planning scene |
|---|---|---|
| Collision model | write it: UR5e links + pen as capsules, workpieces as the registered boxes, table as a half-space; capsule–box distance is closed form (the "8-cylinder" model of Liu et al. 2023) | FCL against the registered CAD meshes inserted as collision objects; self-collision from the SRDF for free |
| Transit planner | our RRT-Connect with `_is_valid` upgraded | OMPL RRT-Connect |
| LIN approach | Cartesian interpolation + IK seeded from the previous solution (as the drawing server's DESCENT) | `computeCartesianPath` |
| Force / admittance | our admittance node → Servo (already there) | same node; Servo is MoveIt anyway |
| Execution | `scaled_joint_trajectory_controller` via TOTG, as the drawing stack | MoveIt trajectory execution |
| Cost | ~150 lines of geometry + tests; everything else exists and is proven on this robot | a MoveIt config for UR5e + pen TCP, a planning-scene publisher for the registered parts, a second motion stack next to the drawing server |
| Risk | our checker is only as good as its capsules; fixtures/clamps must be added by hand | integration time; two ways of moving the same robot |

**Recommendation: A1 now, A2 as the upgrade path.** The workpieces are already boxes in
the registry, the arm is six capsules, and the rest of the stack is proven. The moment
clamps and fixtures with real shapes enter the cell, or self-collision becomes a
concern, move transit planning to MoveIt and keep the approach/mark phases as they are.

### B. Approach and touch

| | B1. Force-gated LIN descent | B2. Position-only descent to the computed point |
|---|---|---|
| What | descend along −approach through Servo; the admittance node holds a small force; stop on contact | descend to the tack point exactly, no sensing |
| Measures | the placement error along the approach axis (contact depth) | nothing; a pose error of 5 mm either digs in or draws in air |
| Needs | FT wrench (UR5e built-in), Servo running, controller switch trajectory ↔ forward-position | nothing new |

**B1.** The registration error we measured is 4–11 mm (fit-up). Position-only marking
would put the pen 5 mm into the plate or 5 mm above it. Force gating turns that error
into a number instead of a crash, and the number is the validation.

### C. Marking the tack

| | C1. Dot at the contact point | C2. Short stroke p0 → p1 under admittance |
|---|---|---|
| What | touch, dwell, retract | after contact, follow the seam for the tack length holding F_hold along the pen axis |
| Gives | placement error along the approach | the same, plus the mark shows the seam *direction* and the lateral error becomes measurable along a line |
| Needs | B1 only | the admittance controller generalised from "Z of a plane" to "the tool axis" (it is a frame choice in the node: `target_orientation_axis_angle` + the twist frame) |

**C1 first, C2 on the same run once C1 works.** C2 is the surface following the plan
asked for; it is one parameterisation away from what the node does on a plane.

### D. Orientation, elbow-up, no aggressive turns

* **Branch lock.** Read the configuration branch off the current (elbow-up) pose — the
  signs of `shoulder_lift`, `elbow`, `wrist_1` — and reject every IK solution on another
  branch. All tack poses are solved *before* moving; a tack whose only solution needs a
  flip is retried with another roll (see next), then reported unreachable. Never flip
  mid-sequence.
* **Roll about the pen axis** is chosen per seam, not per tack: sample roll in 15° steps,
  keep the solutions on the locked branch, score by distance from joint limits and by
  the smallest joint move from the previous tack, take the best; the same roll for every
  tack of a seam so the wrist does not twist between neighbours.
* **Continuity.** Seed IK from the previous solution (the drawing server already does),
  gate every step by the joint-jump limit (`max_joint_jump_rad`, as `move_to_object`),
  and time everything with TOTG under lowered velocity/acceleration limits for marking.
* **Arch transfers between tacks of one assembly** (retract to a safe height along the
  approach, joint move at that height, descend) instead of a fresh RRT per tack — RRT
  only from/to home, and it is still collision-checked.

### E. Visiting order for the pen

There is no heat, so the rule's `order` (heat-balance) is not the right order for
marking. Use travel: nearest-neighbour over the tacks with arch transfers, seam by
seam, `tack_no` ascending within a seam. Keep `order` in the file for the day the torch
replaces the pen; the sequence field in `tack_marks.json` says which one was used.

## Milestones

1. **Pen TCP.** Measure the pen: tool length along tool0 +Z and the tip offset, by
   touch-off on a known point (or the UR's 4-point TCP). Store it as `pen_tcp` in a
   config the marking node and the collision model both read.
2. **Collision model + tests** (`admittance_control/collision.py`): capsules for the six
   UR5e links from FK, the pen capsule, boxes from `weldgen_objects.json` at
   `assembly.json` poses, the table half-space; `_is_valid(q)` in the RRT uses it.
   Tests: a configuration that sweeps the pen through the standing plate is rejected;
   the parked pose is accepted; the drawing stack's regression paths still plan.
3. **Reachability report, no motion.** For the live `welding_tacks.json`: every tack's
   pose on the locked branch, the roll chosen per seam, joint moves between tacks,
   collision margin at P_app and at the tack. Published as markers (pen axis at every
   tack) so RViz shows the plan before the robot moves. This alone answers "can the arm
   reach the seams elbow-up" on the current fixture.
4. **Dry run in the digital twin** (`digital_twin_pointcloud.launch.py` + Isaac): the
   full four phases with the twin's contact-less descent stopping at the point.
5. **Real robot, one tack:** transit, force-gated descent, dot, retract, and the first
   `tack_marks.json` entry with its contact depth. Then all six.
6. **Stroke marking (C2)** on the same assembly; compare the marks with the seam by photo.

## Open questions (they change the work)

* **Controller switching.** Transit runs on `scaled_joint_trajectory_controller`; the
  admittance node commands Servo, which needs `forward_position_controller` (or the
  Servo-side JTC). Confirm what the UR driver has active and whether we switch per phase
  (`controller_manager/switch_controller`) or run the descent as a slow trajectory with
  a wrench watchdog that cancels it (simpler, no switch, but no force *holding*).
* **F_touch.** UR5e FT noise is ~0.5 N; 2–3 N is a safe threshold for a pen against
  steel/MDF. Confirm the pen is compliant enough (spring-loaded holder?) that 3 N does
  not bend it.
* **Table and fixture geometry** for the collision model: the bench plane (`ground_z_m`
  is already measured for the ground cut) and any clamp — box it by hand for now.

## RViz: parts moving when the robot moves

Everything that should stay put is published in `base_link` (the SEPC, the seams, the
tacks) with the fixed frame `base_link`; those cannot move with the arm. What does move:
anything latched in the **camera** frame — the model and scene clouds after tracking
stops, the detection markers, and the crop box drawn in the object frame whose TF stops
updating — because RViz re-places an old message with the current transform. The
model/scene clouds and the crop box are now cleared at `save_object` (2026-09-21). To
pin down what is left:

```
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame   # must change as the arm moves
ros2 topic echo /perception/icp/static_env --field header.frame_id  # must be base_link
```

If the orange SEPC itself moves, the robot description's `base_link` and the ICP node's
`static_frame` are not the same frame (check `ros2 run tf2_tools view_frames`); if only
green/white/marker displays move, they are camera-frame leftovers and the remaining
publishers (detection markers) should be moved to `base_link` or cleared on save.
