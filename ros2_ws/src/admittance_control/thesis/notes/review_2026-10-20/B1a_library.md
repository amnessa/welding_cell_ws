# B1a: material map of the library modules and C++ code (admittance_control)

Scope: every module in `admittance_control/admittance_control/*.py`, the C++ in `src/*.cpp`,
`CMakeLists.txt`, `package.xml`, `srv/ComputeTOTG.srv` (there is no `msg/`; there is a legacy
`action/ExecuteDrawing.action`), and the tests in `test/*.py` (used only to say what covers
what). The draft is checked against the code: `thesis/manuscript/chapters/ch3_methodology_cell.tex`
(ch3) and, where the cell's library is concerned, `ch4_methodology_weldset.tex` (ch4).

Conventions:
- Paths are relative to `/workspaces/welding_cell_ws/ros2_ws/src/admittance_control/` unless they start
  with `/`. `file.py:NN` gives the line in the current tree (2026-10-20).
- Section names are the ch3/ch4 titles: "Coordinate Frames and Calibration",
  "Finger-Pointing Camera Aim", "Stationary Refinement and Saving", and so on.
- When a value set in a config or a node differs from the library default, the report gives both.
  **The thesis should quote the value actually used** (marked "in use").
- Statements about scripts and nodes (gesture node, PPF classifier, FoundationPose server, hand-eye
  helper, tare script) are out of scope. They are checked here only where a library statement
  depends on them, and those places are marked "(node)".

Contents
1. Summary table (module, thesis section, tests)
2. Per-module entries (29), grouped by pipeline stage
3. (a) What the draft is missing
4. (b) All draft/code discrepancies
5. (c) Candidate algorithm boxes (15)
6. Appendices: A config values in use vs. library defaults; B stale docstrings/comments and one code
   defect; C test coverage; D dated incidents from the code comments (chronological)

**Headline findings.**
- (1) **Code defect.** `kinematics.ur5e_jacobian` has doubled angular rows, so the camera aim turns at
  half its commanded rate (7.5°/s at the "15°/s" cap). See A1, B1 and #36.
- (2) The **ISO 5817 gap check** is not a seam check: it is a warning inside the multi-view refinement,
  and the draft's formula lacks the caps (#7, #8).
- (3) The **registry** "verified both ways" is optional and only flags a failure; only open bands are
  B-splines (#1, #2).
- (4) The **Bézier bow** control point is $x_m+2\beta b$ (#18).
- (5) The **acceptance table**: the overlap rule needs "> 1 mm and"; the "fit" rows are fitness < 0.2
  and an unmeasured relative change (#9-#11).
- (6) **Closed-seam tacks** have no ends (#20).
- (7) ch4's "seam rule applied unchanged" is wrong for the seam rule (#5).

---------------------------------------------------------------------------------------------

## 1. Summary table

| Module | One-line role | Thesis section (ch3 unless noted) | Tests |
|---|---|---|---|
| `kinematics.py` | UR5e FK (nominal + factory-calibrated), numeric Jacobian, DLS IK, RRT-Connect, Catmull-Rom | Kinematics, Tool Centre Point and Table; Reachability; Transit Planning (fallback) | test_kinematics_params; indirectly test_collision, test_tack_reach, test_transit_cpp |
| `tool_model.py` | Pen tool: tip, collision envelope, camera extrinsic, tip-target pose, 4-point TCP solve | Kinematics, TCP and Table; Collision Model | test_tool_model |
| `table_probe.py` | Table touch pattern, plane fit, TCP error from touches, camera-vs-table checks, tilt separation | Kinematics, TCP and Table; Hand-Eye Calibration; Depth Calibration | test_table_probe |
| `geometry.py` | Rotation matrix <-> quaternion | none (implementation) | none |
| `camera_aim.py` | Finger-pointing aim control law | Finger-Pointing Camera Aim | test_camera_aim |
| `src/aim_rate_relay.cpp` | 125 Hz -> 500 Hz linear-ramp velocity relay with its own stop | Finger-Pointing Camera Aim (Command interpolation); Performance-Critical Components | none |
| `sam6d_io.py` | Image/HTTP/JSON plumbing for the pose server (legacy name) | not in the draft (ROS 2 Architecture) | none |
| `fp_stream.py` | WebSocket wire protocol for live FoundationPose tracking, ROI, ego-motion, latest-wins slot | 6D Pose Registration and Live Tracking | test_fp_stream_copy (copy identity only) |
| `icp.py` | Pure-NumPy Fast & Robust point-to-plane ICP with normal gate; cloud utilities | Stationary Refinement and Saving | test_icp_normal_gate |
| `pose_stats.py` | Stationary tail, robust pose mean, swing-twist removal | Stationary Refinement and Saving; Symmetric and Cut Parts | test_pose_stats |
| `assembly_mesh.py` | Boolean union of the saved CADs into one watertight model | not in the draft | none |
| `multiview.py` | View planner for close-range refinement | View Planning | test_multiview_views, test_multiview_curved |
| `multiview_capture.py` | Capture format on disk, preprocessing (D4), exact synthetic renderer, replay | Multi-View Close-Range Refinement (not in draft yet) | test_multiview_capture |
| `multiview_refine.py` | Joint MAP refinement of touching parts, observability, extrinsic estimate, online self-cal, acceptance, fit-up warnings | Joint Refinement; Observability; Online Estimation of the Extrinsic Translation; Acceptance; Fit-Up Gap | test_multiview_touching_parts, test_multiview_curved, test_multiview_capture, test_selfcal |
| `multiview_service.py` | `~/refine_pose` orchestration on the robot | Multi-View Close-Range Refinement; ROS 2 Architecture | none (ROS) |
| `selfcal.py` | Persistent extrinsic-translation self-calibration from many runs | Online Estimation of the Extrinsic Translation; calibration table | test_selfcal |
| `part_shapes.py` | True-shape SDF of registered parts, planning boxes, stale-entry check | Joint Refinement (true shapes); CAD Parts as Primitives | test_multiview_curved, test_curved_registry |
| `weldgen_registry.py` | CAD -> exact primitive (slab/tube/swept slab) fitting and verification | CAD Parts as Primitives; Symmetric and Cut Parts | test_curved_registry, test_seam_from_registration, test_pose_stats |
| `seam_from_registration.py` | Mode A: planar seams by a registration-tolerant D4 rule; tacks by tackrule-0.1 | Seams from the Registered Assembly (Planar Seams; Fit-Up); Tack Placement; ch4 "WeldSet and the Robotic Cell" | test_seam_from_registration, test_curved_seams, test_tack_reach |
| `curved_seams.py` | Curved seams (tube/plate, tube/tube, band/plate), per-point frames, cut-spin settling | Curved Seams; Symmetric and Cut Parts | test_curved_seams |
| `collision.py` | Capsule/box/plane collision model of arm + tool vs parts + fixtures + table | Collision Model | test_collision, test_curved_reach, test_transit_cpp |
| `src/transit_cpp.cpp` | C++ port of the collision model + OMPL APS / RRT-Connect (pybind11) | Transit Planning; Collision Model; Performance-Critical Components | test_transit_cpp |
| `tack_reach.py` | Branch-locked reachability; per-seam (tilt, roll) grid; per-tack Viterbi; cable window | Reachability | test_tack_reach, test_curved_reach |
| `marking.py` | Transit cascade, descent chain/check, departure chain, timing, strokes, marking plan | Transit Planning; Force-Gated Descent and Marking | test_marking, test_transit_smoothing, test_curved_reach, test_transit_cpp |
| `motion.py` | TrajectoryExecutor: FollowJointTrajectory with wrench watchdog, touch detection, TOTG timing | Force-Gated Descent and Marking; Transit Planning (timing) | test_motion |
| `src/totg_service_node.cpp` + `srv/ComputeTOTG.srv` | MoveIt TOTG as a ROS service | Transit Planning (timing); Performance-Critical Components | none (test_motion uses a fake service) |
| `src/jacobian_calculator_node.cpp` | Legacy Isaac Sim / sand-drawer Jacobian velocity node | not part of the cell | none |
| `action/ExecuteDrawing.action` | Legacy drawing-server action | not part of the cell | none |
| `__init__.py`, `CMakeLists.txt`, `package.xml` | Packaging: extend_path, explicit install list, C++ targets | Implementation Details | test_install_list |

---------------------------------------------------------------------------------------------

## 2. Per-module entries

### Group A: kinematics, tool and calibration geometry

#### A1. `kinematics.py` (814 lines)

**Purpose.** UR5e forward kinematics `base_link -> tool0` and every joint frame, a numerical
Jacobian, damped-least-squares IK, and a joint-space RRT-Connect with shortcut and Catmull-Rom
smoothing. The FK can run on either the nominal URDF chain or the robot's factory-calibrated
kinematics file. Which one is active is a module-global switch that every entry point sets once.
The module docstring still calls it "UR5e RRT-Connect Planner", which is historical (`kinematics.py:2-10`).

**Algorithms (as implemented).**
1. *Nominal FK* `_ur5e_fk_nominal` (`:82-139`). The URDF chain, each joint origin followed by $R_z(q_i)$:
   $T = R_z(\pi)\,T_z(0.1625)R_z(q_1)\,R_x(\tfrac{\pi}{2})R_z(q_2)\,T_x(-0.425)R_z(q_3)\,
   T_x(-0.3922)T_z(0.1333)R_z(q_4)\,T_y(-0.0997)R_x(\tfrac{\pi}{2})R_z(q_5)\,
   T_y(0.0996)\,\mathrm{RPY}(\tfrac{\pi}{2},\pi,\pi)R_z(q_6)\,\mathrm{RPY}(0,-\tfrac{\pi}{2},-\tfrac{\pi}{2})\,
   \mathrm{RPY}(\tfrac{\pi}{2},0,\tfrac{\pi}{2})$, with URDF RPY $=R_z(y)R_y(p)R_x(r)$ (`:73-75`).
2. *Parametrised FK* `ur5e_link_frames_params` (`:211-225`). Every joint origin is
   $\mathrm{Trans}(x_i,y_i,z_i)\,\mathrm{RPY}(r_i,p_i,y_i)$ from a ur_description kinematics YAML
   (`load_kinematics`, a hand parser with no PyYAML, `:171-189`), then $R_z(q_i)$. `NOMINAL_KIN`
   (`:196-203`) is the nominal chain written as such a table, so the C++ port only has to run the
   parametrised chain.
3. *Active kinematics* `use_kinematics(path)` / `ur5e_fk` / `ur5e_link_frames` (`:241-273`).
   `None` means nominal. The entry points that command or interpret the robot through this module
   call `use_kinematics(config/ur5e_calibration.yaml)` once at start: camera_aim_node,
   tack_marking_node, tack_reachability, tack_reach_marker_node, multiview_service (refine_pose),
   table_touchoff, pen_tip_touchoff, tare_distance.
4. *Jacobian* `ur5e_jacobian` (`:276-305`), by central differences with $\varepsilon=10^{-6}$:
   $J_{v,i}=\frac{p(q+\varepsilon e_i)-p(q-\varepsilon e_i)}{2\varepsilon}$, and
   $J_{\omega,i}=\mathrm{vee}\big(R(q+\varepsilon e_i)R(q-\varepsilon e_i)^{\top}\big)/(2\varepsilon)$
   from the skew part. The rows are $[v;\omega]$ of the tool0 origin in base_link.

   **FINDING (verified numerically, 2026-10-20): the angular rows are twice the true angular
   Jacobian.** $dR=R(q+\varepsilon e_i)R(q-\varepsilon e_i)^{\top}\approx I+2\varepsilon[\omega_i]_\times$,
   so $(dR_{21}-dR_{12})/(2\varepsilon)=2\omega_{i,x}$ (`:299-303`). The correct divisor is $4\varepsilon$
   (or use $\log(R(q+\varepsilon e_i)R(q)^{\top})/\varepsilon$). Check at home_q: every non-negligible
   angular entry is exactly 2.000× the true one, and the linear rows are exact. Consequences:
   - (i) `camera_aim.joint_rates` achieves **half** the commanded angular rate. Commanding 15°/s about
     the camera y gives 7.50°/s, and the optical centre, which should be still, moves at 8.2 mm/s
     ($\|p_E-p_C\|$ = 110 mm), held back only by the P pull. So the aim's effective gain is
     K = 1.25 s⁻¹, its effective cap 7.5°/s, and its roll gain 0.75 s⁻¹.
   - (ii) `ik_solve` still converges to the right pose (its error and tolerances are computed from the
     true FK), but its orientation steps are halved.
   - (iii) `test_camera_aim.test_tool0_twist_keeps_the_optical_centre_still` checks `ur5e_jacobian @ qd`,
     i.e. with the same Jacobian, so it cannot catch this. The closed-loop tests integrate the true FK
     and pass with the halved rate (centring in < 4 s, drift < 10 mm).
5. *IK* `ik_solve` (`:343-384`). The error is $e=[p^{*}-p;\ \log(R^{*}R^{\top})^{\vee}]$
   (`pose_error`, `:329-336`), and each step is $\Delta q=J^{\top}(JJ^{\top}+\lambda^{2}I)^{-1}e$
   with $\lambda=0.05$, clipped to $\|\Delta q\|\le0.3$ rad. It stops when
   $\|e_p\|<0.1$ mm and $\|e_\theta\|<10^{-3}$ rad (at most 200 iterations), then wraps every joint
   to $[-\pi,\pi]$ (`_wrap_joints`, `:387-397`) and checks the limits (±2π, elbow ±π, `:22-29`).
6. *RRT-Connect* `rrt_connect` (`:530-596`). Bidirectional. The weighted metric is
   $\|w\odot\Delta q\|$ with $w=(1,1,0.8,0.5,0.5,0.3)$ (`:421-425`), step 0.2, goal bias 0.1,
   5000 iterations, and edges are checked at resolution 0.05 in that metric (`_edge_valid`, `:466-481`).
   Bug fix in a comment: after an odd number of tree swaps tree_a is rooted at the goal, so both
   halves must be reversed. The pen planner's consecutive-edge check found this (`:582-591`).
7. *Shortcut* `smooth_path` (`:603-624`): 150 random shortcuts, checked against **joint limits only**.
   marking.py has its own collision-aware `shortcut_path` for that reason (`marking.py:48-62`).
8. *Centripetal Catmull-Rom* `bezier_smooth_path` (`:701-756`): α = 0.5, with phantom end points
   $2p_0-p_1$ and $2p_n-p_{n-1}$. A segment that leaves the joint limits falls back to linear.
   The result is C1 and passes through every waypoint.
9. `plan_to_pose` (`:763-814`): IK from seeds (current, given, 10 random), then RRT-Connect,
   shortcut and spline. This is the legacy drawing-server path.

**Key parameters.** IK λ = 0.05, step ≤ 0.3 rad, tolerances 0.1 mm / 1e-3 rad, 200 iterations.
RRT step 0.2 (marking uses 0.15), 5000 iterations (marking uses 8000), edge resolution 0.05
(marking uses 0.01). Limits ±6.2832 rad (elbow ±3.1416). Kinematics file: `config/ur5e_calibration.yaml`
(node parameter `kinematics_file`).

**Inputs/outputs; users.** Joint vectors in UR order; 4x4 frames. Library users: camera_aim,
collision, marking, multiview, multiview_service, tack_reach. Script users: camera_aim_node,
tack_marking_node, tack_reachability, tack_reach_marker_node, table_touchoff, pen_tip_touchoff,
tare_distance, tcp_offset_probe, and the legacy drawing_action_server, drawing_dispatcher and
move_to_object_node.

**Design decisions and incidents.**
- 2026-09-29: the robot controller and RTDE poses use the factory-calibrated chain. It is
  "2.4-4.2 mm and ~0.5 deg apart from the nominal one at the tip ... which put the marks ~4 mm off"
  (`:233-240`). The README gives 0.55°, and its §14 table says "TF and the planner used the nominal
  UR5e chain, the robot and the hand-eye capture its calibrated one". The fix: the calibration goes
  in the URDF, the driver (`kinematics_params_file:=`) and every tool.
- The C++ collision model runs only the parametrised chain; the nominal case is handed over as a table (`:192-195`).

**Tests.** `test_kinematics_params.py`: the nominal file reproduces the FK and link frames to machine
precision; the calibration file is a small (mm-level) perturbation; an incomplete file is rejected;
`use_kinematics` switches FK/links/IK and resets; at the scan home the calibrated chain turns the
camera by ~0.5° (the 2026-09-29 mismatch). Indirect coverage: `test_transit_cpp.test_fk_matches_python`
(C++ frames = Python, nominal and calibrated); `test_collision.test_rrt_with_the_model_plans_around_a_standing_plate`.

**Thesis section.** Coordinate Frames and Calibration / Kinematics, Tool Centre Point and Table
(why the calibrated chain). Reachability (IK on the branch). Transit Planning (the Python RRT-Connect
fallback).

**FACT CHECK (ch3).**
- Table `tab:calib_chain`, "Kinematics: factory calibration used in every tool; reference: the controller".
  **Confirmed, with a caveat.** The library defaults to the nominal chain (`:241-242`). Every entry
  point that uses the library's FK switches to `ur5e_calibration.yaml` by default, and TF uses it
  through the driver's `kinematics_params_file` (README §14). A stale comment in `config/pen_tool.json`
  ("The planner's FK is the nominal UR5e chain") contradicts this; see Appendix B.
- TODO "nominal vs. calibrated differ by millimetres at the tip". **Confirmed:** 2.4-4.2 mm, ~0.5°
  (`:236-238`).

#### A2. `tool_model.py` (182 lines)

**Purpose.** The pen tool on the flange as geometry, read from one file (`config/pen_tool.json`,
frame tool0, metres): the pen tip, the touch force, the standoff, the collision envelope (capsules
and boxes, camera body included), and the hand-eye extrinsic for reference. It also gives the tool0
pose that puts the tip on a target with the pen along a given axis, and the classic 4-point TCP solve.

**Algorithms.**
1. `load_tool_model` (`:140-182`) parses the primitives. A missing extrinsic file is not an error:
   the camera is then absent and the marking node refuses to move.
2. `T_tool0_for_tip(tip, axis, roll)` (`:83-100`): $z=a/\|a\|$; $r=e_x$ if $|z_x|<0.9$ else $e_y$;
   $x=(r\times z)/\|r\times z\|$; $y=z\times x$;
   $R=[\,c\,x+s\,y,\ -s\,x+c\,y,\ z\,]$ with $(c,s)=(\cos\varphi,\sin\varphi)$; origin
   $o=p_{\mathrm{tip}}-R\,d_{\mathrm{tip}}$. The pen axis tool0 +Z points *into* the joint
   (minus the seam's approach). The roll φ is the redundancy of an axisymmetric tool.
3. `solve_tip_offset` (`:119-137`) is the 4-point TCP. With the tip held on one fixed point $p$ from
   $n\ge3$ orientations, $R_i d+t_i=p$, so $[R_i\ \ -I]\,[d;p]=-t_i$ is solved by least squares.
   The RMS residual (mm) is the repeatability ("aim for < 1 mm").
4. `transform_primitive` / `primitives_in` (`:46-81`) move the primitives rigidly by FK.

**Parameters (in use, `config/pen_tool.json`).** Pen tip (1.07, -1.35, 180.91) mm: the pendant's
4-point TCP calibration of 2026-09-28, cap off. Earlier values: 2026-09-25 (1.35, -0.17, 181.73),
2026-09-24 (0, 0, 183.78), with the cap 192.5 (not used). `touch_force_n` 1.5 N; `standoff_m` 0.035 m.
Primitives:
- `flange_adapter` box, half (47, 47, 25) mm
- `holder_flange` capsule r 48 mm, z 58-67 mm
- `holder_body` capsule r 20.5 mm, z 70-130 mm
- `pen` capsule r 8 mm, from z 120 mm to tip minus r (the round cap ends at the tip)
- `camera_arm` box
- `camera_body` box, half (50, 20, 20) mm, centre (0, 90, 45) mm

Camera extrinsic: `notebooks/T_tcp_to_cam.npy`, which is tool0 -> camera_color_optical_frame despite
the "tcp" in its name (`:16-19`).

**Users.** marking node, collision model, tack_reach/tack_reachability, camera_aim_node,
multiview_service, tool_model_marker_node, tack_reach_marker_node, table_touchoff, pen_tip_touchoff.

**Design decisions and incidents.**
- The camera body is a box measured on the bench. Deriving it from the calibrated sensor origin put
  it in the wrong place (`:14-19`). The calibrated optical origin is drawn as a marker and "has to
  sit on the lens face".
- pen_tool.json, 2026-09-29: the holder envelope comes from `world/penholder_assembly.usda` with
  +3 mm radial margin. It used to be one capsule r 42 mm to 120 mm, which in a 90° T cleared the
  plates by only 1.1 mm, so square tees were unreachable.
- The camera body box dates from 2026-09-22.

**Tests.** `test_tool_model.py`: the package config loads with the measured tip and the calibrated
camera; primitives move rigidly with tool0; `T_tool0_for_tip` puts the tip on target along the axis
for any roll; unknown primitive types are rejected; the touch-off recovers an off-axis tip.

**Thesis section.** Kinematics, Tool Centre Point and Table (TCP); Collision Model (tool envelope).

**FACT CHECK.**
- `tab:calib_chain`, "Pen tip (TCP): 4-point TCP; roll and tilt touches; reference table plane".
  **Confirmed.** `solve_tip_offset` is the 4-point solve; roll and tilt touches are `table_probe.solve_tcp_error` (A3).
- §Collision Model, "the pen tool is its CAD envelope (capsules for the pen and holder, boxes for the
  camera body)". **Incomplete.** The envelope also has boxes for the flange adapter and the camera arm
  (pen_tool.json `primitives`).

#### A3. `table_probe.py` (307 lines)

**Purpose.** The geometry behind the table touch-off and the camera-vs-table checks, testable
without the robot: the touch pattern, the plane fit, touch repeatability, the TCP error from touches
on a known plane, the dominant flat surface in a cloud against a known plane, splitting table tilt
into world and camera parts, and the camera tilt correction.

**Algorithms.**
1. `touch_pattern` (`:24-40`): a regular n-gon with edge $e$ and circumradius $r=e/(2\sin(\pi/n))$,
   at angles $\psi_0+\pi/2+2\pi k/n$, plus the centre. Default: a hexagon with 10 cm edges plus the
   centre, i.e. 7 touches. Four touches are spare, so the RMSE is meaningful (`:9-14`).
2. `fit_plane` (`:43-71`) is ordinary least squares of **height**, $z=ax+by+c$ (vertical residuals,
   not orthogonal regression). It returns $n=(-a,-b,1)/\|\cdot\|$, the tilt $\arccos n_z$, the height
   at the centroid ("THE table height to use"), the residuals, the RMSE, and `rmse_meaningful` (n > 3).
3. `repeatability` (`:74-79`): per-spot std of z, pooled as an RMS.
4. `check_orientations` (`:109-132`): tilt 0° at azimuths 0/90/180/270 (the pure roll test) and
   tilt 30° at the same azimuths with the tool yaw held fixed. Rolling the tool with the tilt
   direction (the first draft) gave an ill-conditioned fit.
5. `solve_tcp_error` (`:135-175`). The real tip is $p_i+R_ie$ and lies on the plane:
   $n^{\top}(p_i+R_ie-p_0)=0\Rightarrow(R_i^{\top}n)^{\top}e+\delta=-h_i$, where δ is a free plane
   offset (the plane was measured with the same TCP). Least squares in $(e,\delta)$. Conditioning:
   the lateral columns after regressing out the others (> 0.1 is good). Vertical touches give about
   $\sin(\text{table tilt})\approx0.02$, 30° tilts about 0.3, and the along-pen component only about
   0.05 at 30° ("use 45° tilts if the pen length itself is in doubt").
6. `surface_vs_plane` (`:179-224`): the points within ±0.25 m of the known plane; the densest 2 mm
   height bin; the points within ±8 mm of it; three rounds of plane fit with 3·MAD trimming
   (MAD·1.4826 + 0.05 mm). Returns the height offset at the surface centre (+ means the camera sees
   the table higher), the tilt and its azimuth, and the extrapolation distance. Large errors are
   measured, not filtered.
7. `separate_tilt` (`:227-257`): per view, $t_i=a+\mathrm{Rot}(\psi_i)\,b$ (2-D small angle), with
   $a$ world-fixed and $b$ camera-fixed. Least squares. "Separable" requires a yaw spread ≥ 90° and
   $\sigma_{\min}(A)>0.3$.
8. `refine_camera_rotation` (`:272-307`): to first order, $n_{\mathrm{obs},i}-n_0\approx a+n_0\times(R_iw)$,
   with $a\perp n_0$ (2 dof) and $w=(w_x,w_y,0)$ in the camera frame. The yaw about the optical axis is
   unobservable from a plane seen from above. Apply as $T_{\mathrm{tool0,cam}}\leftarrow T\cdot\mathrm{diag}(e^{[w]},1)$.
9. `rot_log`, `rot_exp`, `slerp_R` (`:83-106`): geodesic helpers.

**Parameters.** Edge 0.10 m; six sides; tilts (0, 30)°; azimuths 0/90/180/270; surface search ±0.25 m,
band 8 mm, min 500 points; separability: spread 90°, σ_min 0.3.

**Users.** `scripts/table_touchoff.py` (writes `notebooks/table_plane.json`), `scripts/extrinsic_check.py`,
`helper/calibration/refine_extrinsic_from_table.py`. The resulting plane is read by
`collision.load_table_plane`, `multiview_capture.table_plane` and `MarkingConfig.table()`.

**Incidents.** marking.json, 2026-10-02: the plane is -96.1 mm at (-0.143, 0.426) m, tilted 1.70°,
i.e. about -99.8 mm under the bench parts at (-0.50, 0.08) m. Before 2026-10-14 the collision model
used no table at all (`table_z_m` null): "transits ran through it".

**Tests.** `test_table_probe.py`: the default pattern; the plane fit recovers height, tilt and noise;
repeatability pooling; the TCP error from contact heights; a large error measured and the floor
ignored; tilt split into world and camera parts; a known camera tilt recovered.

**Thesis section.** Kinematics, Tool Centre Point and Table; Hand-Eye Calibration (separating camera
tilt from table tilt); Depth Calibration Against the Robot (`surface_vs_plane`).

**FACT CHECK.**
- `tab:calib_chain`, "Table plane: pen touches, plane fit; reference the robot". **Confirmed:** 7 touches.
- TODO "the table as a plane $n^{\top}x+d=0$ fitted to pen touches, tilted". **Precise form:** the fit
  is $z=ax+by+c$ by least squares on height, converted to $(n,d)$ with
  $n=(-a,-b,1)/k$, $d=-c/k$ (`collision.py:289-300`). The tilt is 1.70° (2026-10-02).
- `tab:calib_chain`, "Extrinsic rotation: camera tilt separated from table tilt at four wrist yaws".
  **Consistent.** The library needs any yaw spread ≥ 90° (`:247-249`). Four yaws is the script's
  choice. `refine_camera_rotation` corrects only the two tilt angles, never the yaw about the optical
  axis (`:281-283`). The thesis should say so.

#### A4. `geometry.py` (57 lines)

**Purpose.** `rotmat_to_quat`: branch on the trace or the largest diagonal element (Shepperd-style),
then normalise; returns [x, y, z, w]. `quat_to_rotmat` is the inverse. **Users:** most scripts that
publish TF or poses (camera_extrinsic_tf_publisher, foundationpose_bridge_node, fp_tracker_node,
icp_pose_refiner_node, sam6d_bridge_node, table_touchoff, tcp_offset_probe, touch_probe, the marker
nodes, check_registration, extrinsic_check, and the legacy drawing scripts). **Tests:** none.
**Thesis:** none needed. **FACT CHECK:** nothing in the draft.

### Group B: operator interaction (camera aim)

#### B1. `camera_aim.py` (387 lines)

**Purpose.** The finger-pointing camera aim as a ROS-free control law. While the operator points
(MediaPipe `Pointing_Up`) and no frame is frozen, the wrist camera turns about its own optical centre
until the index fingertip is in the middle of the image. Its position is held, so the operator steers
only where it looks. `scripts/camera_aim_node.py` runs `CameraAim.step` once per control cycle and
streams the joint rates through the C++ relay to the UR's forward_velocity_controller.

**Algorithm (one cycle, as implemented; the module docstring `:9-31`).**
1. *Observation* `CameraAim.observe` (`:344-347`) runs per image that has a fingertip. The node takes
   $q(t_k)$ at the image stamp from `JointHistory.at` (linear interpolation; it keeps 2 s, allows the
   end sample up to 0.25 s beyond, and returns None outside, `:262-291`). Then
   $r_B=\mathrm{OneEuro}\big(t_k,\ R_{BC}(t_k)\,\hat d\big)$ with
   $\hat d=\mathrm{normalize}\big(((u-c_x)/f_x,\ (v-c_y)/f_y,\ 1)\big)$ (`pixel_ray`, `:147-150`),
   and $r_B\leftarrow r_B/\|r_B\|$.
2. *One-Euro filter* (`:84-117`, Casiez et al.) on the 3-vector:
   $\alpha(f_c,\Delta t)=1/(1+\tau/\Delta t)$, $\tau=1/(2\pi f_c)$;
   $\hat{\dot x}\leftarrow\hat{\dot x}+\alpha(f_d)(\dot x-\hat{\dot x})$;
   $f_c=f_{\min}+\beta\|\hat{\dot x}\|$; $\hat x\leftarrow\hat x+\alpha(f_c)(x-\hat x)$.
3. *Error in the current camera frame* `step` (`:358-374`): $d=R_{BC}(t)^{\top}r_B$. If $d_z\ge0.3$
   (`min_ray_z`), then $e=(d_x/d_z,\ d_y/d_z)$; otherwise the observation is ignored. Because the
   camera only rotates about its optical centre, a still fingertip has a fixed base-frame ray whatever
   its depth. Re-expressing it in the current frame therefore compensates the image latency exactly,
   and nothing is predicted.
4. *P law with deadband hysteresis* `aim_rates` (`:172-191`). The node sets the deadband in tangent
   units as $\delta=0.05\,W/f_x$, i.e. 5 % of the image width (`camera_aim_node.py:384`).
   Engaged when $\|e\|>\delta$, or when already engaged and $\|e\|>0.4\delta$. When engaged,
   $s=(\|e\|-0.2\delta)/\|e\|$ (the rate is measured from half the inner radius, otherwise it would
   creep forever), $\omega_x=-K e_y s$, $\omega_y=K e_x s$, and $\|\omega_{xy}\|$ is capped at 15°/s.
   Roll hold: $\omega_z=\mathrm{clip}(-k_r\,\mathrm{wrap}(\psi),\pm15^{\circ}/\mathrm{s})$ with
   $\psi=$ `twist_about_axis(R_home,R)`, i.e. $2\,\mathrm{atan2}(q_z,q_w)$ of $R_{\mathrm{home}}^{\top}R$
   (a swing-twist split about the optical z, `:161-165`).
5. *Excursion limit* `limit_excursion` (`:194-215`). For $i\in\{x,y\}$, drop $\omega_i$ if
   $R\,\exp([\omega_i e_i\cdot0.35\,\mathrm s])$ would put |pan| > 35° or |tilt| > 25° **and** further
   out than now; components that bring the axis back are kept. Pan and tilt come from `pan_tilt`
   (`:153-158`): pan $=\mathrm{atan2}(d_x,d_z)$, tilt $=\mathrm{atan2}(-d_y,\sqrt{d_x^2+d_z^2})$
   of the optical axis in the home camera frame. The look-ahead of 0.35 s is "~ the stopping time from max_rate".
6. *Smoother* `RateSmoother` (`:120-143`): two cascaded first-order lags with $a=\Delta t/(\tau+\Delta t)$,
   τ = 60 ms each: $w_1\leftarrow w_1+a(w^{*}-w_1)$, $\Delta w=a(w_1-w)$, with
   $\|\Delta w\|\le\dot\omega_{\max}\Delta t$ (60°/s²). The result has continuous acceleration and no
   overshoot.
7. *Optical-centre hold* `tool0_twist` (`:218-228`):
   $v_C=\mathrm{sat}_{20\,\mathrm{mm/s}}\big(k_p(p_{C,0}-p_C)\big)$ and
   $v_E=v_C+\omega\times(p_E-p_C)$ with $\omega=R_{BC}\,\omega_{\mathrm{cmd}}$. The anchor $p_{C,0}$ is
   the optical centre when the session started (`start`, `:337-342`), not at home.
8. *DLS* `joint_rates` (`:231-240`): $\dot q=J^{\top}(JJ^{\top}+\lambda^2I)^{-1}[v_E;\omega]$ with
   λ = 0.02, scaled uniformly when $\max_j|\dot q_j|>0.5$ rad/s (the direction is kept). J is the tool0
   Jacobian of A1.
9. *Safety look-ahead* `check_ahead` (`:243-258`), run by the node every cycle on
   $q_1=q+0.3\,\mathrm s\cdot\dot q$. It refuses when $|\sin q_{1,5}|<\sin0.35$ (wrist), when
   $\mathrm{sign}(q_1[1,2,4])$ differs from home's (branch lock), or when `is_valid(q1)` fails
   (collision model: table plane, fixtures, arm and pen tool; no parts, since none is registered
   before the capture, `camera_aim_node.py:125-131`).
10. *Losing the hand* `lose()` (`:349-352`). The node calls it when no fingertip has arrived for
    `deadman_s` = 0.3 s. Only the roll hold remains and the smoother ramps the rate to zero.

**Key parameters (AimConfig, `:48-80`).** gain $K$ = 2.5 s⁻¹; deadband 5 % of the image width,
release 40 % of it; max rate 15°/s; max acceleration 60°/s²; τ = 0.06 s (each lag); roll gain 1.5 s⁻¹;
pan ±35°, tilt ±25° from home; look-ahead 0.35 s; position gain 2.0 s⁻¹ with 20 mm/s cap; λ = 0.02;
max joint speed 0.5 rad/s; check-ahead 0.3 s; wrist margin 0.35 rad; min_ray_z 0.3; One-Euro
$f_{\min}$ = 1 Hz, β = 5 Hz per rad/s, $f_d$ = 1 Hz. Node parameters (`camera_aim_node.py:93-114`):
rate_hz 125, deadman_s 0.3, idle_s 0.3, home_tol_m 0.05, still_rad_s 0.02, abort_force_n 15; the
AimConfig fields in `AIM_PARAMS` are exposed as ROS parameters.

**Users.** `scripts/camera_aim_node.py` (with `launch/gestures.launch.py`); `test/test_camera_aim.py`.

**Design decisions (docstrings).**
- Gain choice: "the loop's lag is the smoother and the arm, ~0.15 s; 2.5 /s keeps a ~50 deg phase
  margin even if the compensation is off (0.25 s)" (`:50-52`).
- No integral (it winds up over the ~0.1 s latency) and no derivative (a derivative of a hand landmark
  is noise) (`:16-19`).
- Roll is the over-actuated DOF and is held at home so the image keeps its orientation (`:20-23`).

**Tests.** `test_camera_aim.py` closes the loop on UR kinematics with a 30 Hz camera, images 0.1 s
late, and an arm that lags. It checks the rate signs; deadband hysteresis and the rate cap; roll about
the optical axis only; the excursion limit blocking outward and keeping inward motion; the smoother
having no overshoot; the One-Euro filter; optical-centre hold; `check_ahead` naming its refusal; the
joint history refusing gaps; a still fingertip centred in < 4 s with roll < 0.5° and centre drift
< 10 mm (`:176-184`); and latency compensation removing overshoot at 0.2 s latency (`:187-195`).

**Thesis section.** Operator Interaction / Finger-Pointing Camera Aim.

**FACT CHECK (ch3 §mc_aim).**
- "rotation about the camera's own optical centre: the camera's position is held". **Confirmed**
  (a P pull capped at 20 mm/s).
- Eq. `eq:aim_error`. **Confirmed.** $K^{-1}[u\ v\ 1]^{\top}$ equals `pixel_ray` for a skew-free K;
  the ray is turned with $R_{BC}(t_k)$ from the joint history, One-Euro filtered in the base frame,
  and re-expressed with $R_{BC}(t)$.
- Image "delivered about 0.1 s later". **Confirmed** (docstring and tests).
- Eq. `eq:aim_law`, $\omega_C=(-Ke_y,\ Ke_x,\ -k_r\psi)$. **Confirmed** up to the deadband factor $s$
  and the caps. The thesis should add $s=(\|e\|-0.2\delta)/\|e\|$, the 15°/s cap, the clip on
  $\omega_z$, and $K$ = 2.5 s⁻¹, $k_r$ = 1.5 s⁻¹.
- ψ as the twist of $R_{BC_0}^{\top}R_{BC}$ about the optical axis, from a swing-twist split.
  **Confirmed** (`:161-165`). Note that $C_0$ is the *home* view (from `q_home`, `:321-323`).
- "No integral term ... no derivative term". **Confirmed** (`:17-19`).
- Eq. `eq:aim_dls`. **Confirmed as the design.** Add λ = 0.02, $k_p$ = 2 s⁻¹ with the 20 mm/s cap, and
  the uniform scaling to 0.5 rad/s. **But the implementation's Jacobian has doubled angular rows**
  (A1 FINDING), so the arm turns the camera at half the commanded rate: 7.5°/s at the "15°/s" cap,
  K_eff = 1.25 s⁻¹. The optical centre drifts at ~8 mm/s at full rate, corrected by the P pull. Either
  fix `kinematics.ur5e_jacobian` (divide by 4ε) and re-tune, or quote the achieved values in the thesis.
  The rates the node logs (`rate_deg_s`) are the commanded ones.
- "Every cycle, the pose 0.3 s ahead is checked against the collision model, the branch lock and the
  wrist singularity". **Confirmed** (`check_ahead_s`=0.3, wrist margin 0.35 rad, branch joints (1,2,4)).
  The model contains no parts during aiming.
- "a fingertip lost for 0.3 s ramps the rate to zero". **Confirmed** (node `deadman_s` 0.3, then
  `lose()` and smoother decay).
- "The control law runs at 125 Hz (one cycle costs 2.5 ms in Python); the controller runs at 500 Hz".
  **Confirmed** (`camera_aim_node.py:105`, `aim_rate_relay.cpp:3-9`). The library's own docstring
  still says "(50 Hz)" (`camera_aim.py:6-7`); that is stale, not a draft error.
- TODO list ("Deadband with hysteresis; the rate and acceleration limits and the two-lag smoother; the
  excursion limits looked ahead by the stopping distance"). All present, with the values above.
- **Missing safety items** (node): the |F| > 15 N abort (bias taken at the start), stale /joint_states
  (> 0.2 s), the latch until the hand has been away 1 s, and the dry run (`camera_aim_node.py:23-35, 349-368`).

#### B2. `src/aim_rate_relay.cpp` (140 lines)

**Purpose.** Streams the aim's joint rates to the UR at its 500 Hz control rate. It ramps linearly from
the value it is sending to each new input over one measured input period, so the commanded velocity
is continuous, and it has a stop of its own whatever the Python node does.

**Algorithm.**
1. `on_input` (`:63-83`). An input of the wrong size or with a non-finite value is dropped. If streaming
   and $0<\text{gap}<0.1$ s, the period is updated as
   $T\leftarrow0.8T+0.2\,\mathrm{clip}(\text{gap},2\,\mathrm{ms},50\,\mathrm{ms})$
   (only consecutive inputs count; the initial T is 1/125 s). Then start ← out (the value being sent),
   goal ← input, $t_{\mathrm{in}}$ ← now.
2. `tick` at 500 Hz (`:85-119`), with dt clipped to [0.5, 20] ms. If no input for `timeout_s` = 0.1 s,
   it is stopping and the target is 0. Otherwise
   $\dot q_{\mathrm{want}}=\dot q_{\mathrm{start}}+\mathrm{clip}\big((t-t_{\mathrm{in}})/T,0,1\big)(\dot q_{\mathrm{goal}}-\dot q_{\mathrm{start}})$.
   Per joint: $\mathrm{out}\leftarrow\mathrm{clip}\big(\mathrm{out}+\mathrm{clip}(\mathrm{want}-\mathrm{out},\pm a\,\Delta t),\ \pm0.6\big)$
   with $a$ = 4 rad/s² (2 rad/s² when stopping). It publishes, and goes quiet once at zero after a
   timeout or after reaching a zero goal.

**Parameters (`:38-45`).** input_topic `/camera_aim/joint_rates`; output_topic
`/forward_velocity_controller/commands`; rate_hz 500; timeout_s 0.1; max_speed_rad_s 0.6;
max_acc_rad_s2 4.0; stop_acc_rad_s2 2.0; n_joints 6. Built with -O2 (`CMakeLists.txt:57-60`); started
by `launch/gestures.launch.py:79`.

**Incident (`:3-9`).** "Sent straight to forward_velocity_controller, every new value is a velocity
step the UR applies as a torque kick: at 50 Hz the wrist buzzed, one kick every 20 ms
(notes/thesis_notes.md, 2026-10-16)". Cost: "one input period of delay (8 ms at 125 Hz)". The timeout
"also covers a hard kill of the aim node mid-turn, which otherwise left the last rate on the velocity
controller" (`:13-16`).

**Tests.** None automated.

**Thesis section.** Finger-Pointing Camera Aim (Command interpolation); Performance-Critical Components.

**FACT CHECK.**
- Eq. `eq:relay`. **Confirmed in substance.** Precisely, the ramp starts from the value being sent when
  the input arrives (equal to $\dot q_{k-1}$ once the previous ramp has finished), T is the EMA-measured
  input period, it holds at the goal after T, and each tick is acceleration- and speed-limited.
- "stops the arm by itself when the commands stop". **Confirmed:** 0.1 s timeout, ramp at 2 rad/s².

### Group C: perception and registration

#### C1. `sam6d_io.py` (247 lines)

**Purpose.** ROS-agnostic plumbing for the pose server's HTTP API. The name is legacy (SAM-6D); the
FoundationPose bridge uses the same protocol. It covers image conversion from `sensor_msgs/Image` by
duck typing, colour/depth normalisation, PNG encoding, multipart POST, response parsing, and artifact
saving.

**Algorithms and details.**
- `image_msg_to_numpy` (`:71-104`): rgb8/bgr8/rgba8/bgra8/mono8/mono16/8UC1/16UC1/32FC1/32SC1, with
  row padding handled by `step`; big-endian refused.
- `normalize_depth_image` (`:122-136`): 16UC1 kept in mm; 32FC1 metres converted to uint16 mm (NaN/inf
  set to 0, `depth_scale` 1.0).
- `encode_multipart_formdata` (`:148-176`): "fields" without `filename=` land in Flask's `request.form`,
  which the server's click/click_labels handling depends on.
- `parse_pem_response` (`:195-228`): detections (R, t [mm], score, category_id, bbox) plus base64 artifacts.
- `save_artifacts` (`:231-247`): base64 decode with a basename-only guard against path traversal.

**Users.** foundationpose_bridge_node, gesture_control_node, icp_pose_refiner_node,
realsense_camera_node, realsense_sim_camera_node, sam6d_bridge_node.
**Tests.** None. **Thesis:** not in the draft (it belongs in ROS 2 Architecture, if anywhere).
**FACT CHECK:** no draft statement.

#### C2. `fp_stream.py` (357 lines)

**Purpose.** The wire protocol for live FoundationPose tracking (realtime_fp.md S2-S5, S7). It is pure
NumPy/OpenCV/stdlib, shared byte-for-byte by the GPU container's tracking server and by every client
(`fp_tracker_node.py`). It also provides the ROI helpers, ego-motion prediction, a latest-wins mailbox,
and the `/predict_pose` registration call.

**Protocol (module docstring `:8-50`).** One WebSocket. Text control messages:
`start` (seed "last" or "pose" with `T_cam_obj` in metres, CAD frame, optional `object`, optional
`T_base_cam`, plus options `refine_iter`, `fit_tol_m`, `fit_lost`, `fit_recover`, `lost_frames`,
`ego_motion`, `auto_reseed`, `reseed_interval_s`) and `stop`. Binary frames: `FPT1 | uint32 header
length | header JSON | rgb | depth`. The header carries seq, the image stamp, t_sent, the full-frame K,
full_hw, roi, encodings, depth scale, `T_base_cam` at the image stamp (S7), and an optional
`want_reseed_mask`. Server replies: `pose` messages with state TRACKING/LOST/IDLE, the 4x4 pose, fit,
coverage, a hint {u,v,r} for the next crop, the dropped count, reseeded, timings, and an optional
reseed mask (the CAD silhouette at the last good pose, dilated). The tracking-only server answers
`can_register: false`; re-seeding goes through `/predict_pose`.

**Algorithms.**
1. Encoding (`:72-117`): RGB as JPEG quality 90 or raw; depth PNG (compression 1) or raw. "Lossless
   only: never JPEG depth".
2. ROI (S4): `shift_K` moves only the principal point, $c_x-x_0$, $c_y-y_0$ (`:189-195`).
   `roi_from_hint` (`:198-214`) makes a square of radius $r+40$ px (at least 32 px), clamped.
   `projection_hint` (`:251-260`) gives $u=f_xx/z+c_x$, $v=f_yy/z+c_y$, $r=f_x\cdot1.2\,D/2/z$.
   `register_roi` (`:224-248`) crops the registration input to what the hypotheses can reach: radius
   $\max\big(f\,D\cdot1.2/(2z_{\min}),\ \text{half the mask's extent}\big)+40$ px about the mask's
   box centre, where $z_{\min}$ is the nearest valid depth in the mask.
3. Ego-motion (S7) `predict_ego` (`:265-273`):
   $T_{C_{\mathrm{now}}O}=T_{BC_{\mathrm{now}}}^{-1}\,T_{BC_{\mathrm{prev}}}\,T_{C_{\mathrm{prev}}O}$,
   assuming the object stayed put in the base frame.
4. `LatestSlot` (S2, `:278-313`): a one-element mailbox. `put` overwrites and counts the drop; `get`
   blocks for the newest. A slow consumer never builds a backlog.
5. `post_predict_pose` (`:318-357`) sends rgb.png, a 16-bit depth.png, camera.json and mask.png, plus
   an optional `object_name` for a re-registration that skips PPF (a server without the field
   classifies; the caller compares the name). It requires `units == "m"`.

**Parameters.** DEFAULT_PORT 5001; JPEG quality 90; ROI margin 40 px; minimum ROI 64 px; crop_ratio 1.2.

**Design decision/incident.** `register_roi`: the scorer's full-frame warps for "~250 hypotheses at
once ... at 1280x720 need 2.6 GB on their own and do not fit an 8 GB GPU" (`:228-232`).

**Users.** `scripts/fp_tracker_node.py`; the mirror copy `scripts/scripts_in_foundationpose/fp_stream.py`
(server side).
**Tests.** `test_fp_stream_copy.py` (the two copies are identical). No functional test.

**Thesis section.** Perception and Registration / 6D Pose Registration and Live Tracking.

**FACT CHECK.**
- "the camera's own motion is cancelled by looking up Eq. frame_chain at each image's time stamp".
  **Confirmed:** `T_base_cam` at the image stamp is in every frame header, and `predict_ego` uses it.
- "Registration runs on the desktop GPU, tracking on the laptop's". **Consistent** with the
  tracking-only server (`can_register: false`) and with `package.xml:36` ("WebSocket client to the
  laptop's FoundationPose tracker"). The machines themselves are not in the library.
- TODO "lost-track detection and re-registration locked to the tracked model's name". The protocol
  provides LOST state, fit and coverage thresholds (`fit_tol_m`, `fit_lost`, `fit_recover`,
  `lost_frames`), auto-reseed, reseed masks and the `object_name` lock. The decision logic is in the
  server and node (out of scope).

#### C3. `icp.py` (962 lines)

**Purpose.** A pure-NumPy port of Zhang et al.'s *Fast and Robust ICP* (point-to-plane), with an
optional normal-compatibility gate, plus the cloud utilities the ICP node builds on: PLY loading,
area-uniform sampling with outward normals, depth back-projection, organized normals, voxel
downsampling, oriented crop box, pinhole inference, image ROI of a box, and a nearest-neighbour index
with runtime backend choice.

**Algorithms.**
1. `sample_mesh_surface` (`:130-171`): fan triangulation; triangles chosen by area; barycentric
   samples (with the $u+w>1$ fold). Normals are the face normal from the winding, flipped as a whole
   if the signed volume $\sum v_0\cdot(v_1\times v_2)<0$ (an inside-out mesh).
2. `backproject_depth` (`:175-191`): $x=(u-c_x)z/f_x$, $y=(v-c_y)z/f_y$; invalid pixels become NaN.
3. `estimate_normals_organized` (`:194-217`): $n=\mathrm{normalize}\big((X_{u+1}-X_{u-1})\times(X_{v+1}-X_{v-1})\big)$,
   flipped to face the camera ($n\cdot X>0\Rightarrow-n$). No KD-tree.
4. `voxel_downsample` (`:221-267`): per-voxel mean, with a mixed-radix flat int64 key instead of a
   lexsort, and normals renormalised.
5. `crop_box_mask` (`:271-284`) is the tracking "macro-filter" that replaces the segmentation mask:
   points inside an oriented box at the current pose. `box_image_roi` (`:335-369`) projects the box's
   corners to a pixel rectangle; the result is exact and bit-identical. `infer_pinhole_from_organized`
   (`:287-332`) recovers K by two 1-D least-squares fits ($u=f_x\,x/z+c_x$), verified to ≤ 0.5 px.
6. `NNIndex` (`:396-468`): Open3D `core.nns`, else scipy cKDTree (4 workers), else chunked brute force.
   It is built once per ICP call and reused by every iteration.
7. *Point-to-plane Gauss-Newton step* `_p2plane_gn_step` (`:542-574`): rows $[\,p_i\times n_i,\ n_i\,]$,
   rhs $-(p_i-q_i)\cdot n_i$, solve `lstsq` for $x=(\omega,t)$, $T\leftarrow\mathrm{Rodrigues}(\omega),t\cdot T$.
   Pairs are kept if $\mathrm{dist}<d_{\max}$ and they pass the normal gate.
8. *Normal gate* `_normal_ok` (`:534-539`): $(Rn^{\mathrm{model}}_i)^{\top}n^{\mathrm{scene}}_j\ge\cos60^{\circ}$.
   Scene normals must face the camera.
9. *Welsch* (`:578-587`): $w_i=\exp(-r_i^2/2\nu^2)$ and energy $\sum(1-e^{-r^2/2\nu^2})$. Gated pairs
   count as full outliers (energy 1) so iterates stay comparable (`:658-660`). The robust step has
   **no hard correspondence cutoff** (`:638-642`).
10. *Dynamic ν* (`icp_point_to_plane` robust branch, `:846-881`):
    $\nu_0=\max(3\cdot\mathrm{median}|r^{(0)}|,\ \nu_{\mathrm{end}})$ and
    $\nu_{\mathrm{end}}=\tfrac{1}{3\sqrt3}\,\mathrm{median}_i\,\mathrm{median}_{k\in\mathrm{kNN}_7(i)}|(q_k-q_i)\cdot n_i|$
    (`welsch_nu_end`, a port of `FindKnearestNormMed`, `:590-631`). The stages run at fixed ν with
    6 inner iterations, rising by 1 per stage up to 10; $\nu\leftarrow\max(\nu/2,\nu_{\mathrm{end}})$;
    convergence when $\nu=\nu_{\mathrm{end}}$.
11. *Anderson acceleration* (Fast ICP, `:900-962`, `_welsch_stage` `:672-739`). A fixed point in se(3)
    increments around the initial pose, $g(x)=\log(T_{\mathrm{next}}T_0^{-1})$. The candidate is
    $x=g_k-\Delta G\,\theta$ with $\theta=\arg\min\|\Delta F\theta-f_k\|$. It is accepted if the energy
    does not increase; otherwise a plain step is taken and the history reset. The se(3) exp/log maps
    use the left Jacobian (`:500-531`).
12. Final metrics (`:742-752`): fitness = inliers (dist < $d_{\max}$ ∧ gate) / |source|, plus the inlier RMSE.

**Key parameters.** Library defaults: max_corr 0.01 m, max_iter 40, tol 1e-6, anderson_depth 0,
robust False, NU_BEGIN_K 3, NU_END_K $1/(3\sqrt3)$, NU_ALPHA 0.5, NORMAL_GATE_DEG 60, NN_WORKERS 4.
**In use** (ICP node, `icp_pose_refiner_node.py:363-373`): voxel 6 mm, max_corr 20 mm, anderson_depth 5,
robust True, normal_gate 60°.

**Users.** `icp_pose_refiner_node.py` (live ICP tracking when `tracking_source=icp`; the one-shot ICP
at save), `check_registration.py`, `touch_probe.py`; the library modules assembly_mesh, multiview,
multiview_capture, multiview_refine and part_shapes (PLY loading, sampling, NNIndex, Welsch weights).

**Design decisions and incidents (dated).**
- 2026-09-29, normal gate (`:65-73`). The model is sampled from every face but the camera sees only
  the faces turned towards it. On an 8 mm plate this made "a false minimum where the model straddles
  its own two faces ... (the bench ear: 6.5 deg lean and the weld root 10 mm off, while the real plate
  stood square)".
- Performance on 640x480 organized clouds, a 2500-point CAD and a ~2000-point crop (`:24-33`, `:373-375`,
  `:346`, `:232-236`, `:601-603`): ICP is ~80 % of a tracking tick, almost all of it NN lookup, ~43
  queries per call. Open3D nns takes 0.31 ms vs cKDTree's 1.02 ms (2500 queries into 2247 points).
  cKDTree with workers=-1 takes 1.02 ms vs 0.57 ms at workers=4. Brute force takes 19+ ms. The box ROI
  cuts the tracking front end from 38 ms to 5 ms. The flat voxel key cuts 4.2 ms to 0.6 ms on 7.6k
  points. The ν noise floor (~2.5 ms) is cached across ticks.

**Tests.** `test_icp_normal_gate.py`: sampled normals point outward, also for an inside-out mesh;
from the straddled start (the 2026-09-29 bench case, an 8 mm × 99 mm plate) the gated ICP returns to
the truth with the root within 1.5 mm and < 1°; with the gate off, the old behaviour.

**Thesis section.** Perception and Registration / Stationary Refinement and Saving (also the ICP
tracker in 6D Pose Registration and Live Tracking, for the "why FoundationPose replaced it" TODO).

**FACT CHECK (§per_stationary).**
- Eq. `eq:p2plane`, $E=\sum_iw_i\big((Rp_i+t-q_i)^{\top}n_i\big)^2$ with $w_i=\exp(-r_i^2/2\nu^2)$.
  **Confirmed.** Add the ν schedule (3·median → $\nu_{\mathrm{end}}$, halving), the Anderson
  acceleration, and that the gate zeroes the weights of incompatible pairs. In the robust mode there is
  no distance cutoff; $d_{\max}$ only defines the reported fitness.
- Eq. `eq:normal_gate`, ≥ cos 60°. **Confirmed** (`:73`, `:539`).
- "which stops a thin plate's model from straddling its own two faces". **Confirmed** (2026-09-29;
  6.5° lean, root 10 mm off).
- Citations `zhang2022fricp, chen1992p2plane`. **Consistent** (`:777-778`, "NumPy port of Zhang et al.").
- "refined by point-to-plane ICP against the scene" at save. **Confirmed** (node): 5 fresh clouds,
  each cropped around the averaged tracker pose with ground cut, background subtraction and normal gate;
  results robust-averaged; applied only if mean fitness ≥ 0.2 and the correction ≤ 20 mm / 8° from the
  tracker pose (`icp_pose_refiner_node.py:418-422`, `:1938-2033`).

#### C4. `pose_stats.py` (142 lines)

**Purpose.** A robust mean of a set of rigid poses and its spread, used for the saved pose; the
newest "resting" run of a pose window; and the swing-twist removal of an unobservable spin about a
symmetry axis.

**Algorithms.**
1. `chordal_mean` (`:32-39`): $R=UV^{\top}$ from $\mathrm{SVD}(\sum_iR_i)$, with the last column of U
   flipped if $\det<0$.
2. `stationary_tail(move_mm=8, move_deg=3, min_agree=3)` (`:47-75`) walks back from the newest pose.
   A tick joins the tail while it is within 8 mm of the running median translation and within 3° of the
   chordal mean of the tail. Single jump ticks are tolerated; 3 consecutive disagreeing ticks end the
   tail, which counts as a move.
3. `robust_pose_mean(k_mad=3.5, rot_gate_deg=3)` (`:78-103`): the median $\tilde t$;
   $\mathrm{MAD}=1.4826\,\mathrm{median}|t-\tilde t|+10^{-6}$ per axis. Inliers satisfy
   $|t-\tilde t|\le3.5\,\mathrm{MAD}$ on every axis **and** are within 3° of the chordal mean of all
   poses. If fewer than $\max(3,\lfloor n/3\rfloor)$ agree, all are kept. The output is the per-axis
   median translation and the chordal mean rotation of the inliers, plus statistics: n, n_used, std_mm
   (inliers), range_mm (all), std_deg, max_deg. The statistics are written into `assembly.json` next to
   the pose.
4. `remove_twist(R_ref, R_new, a)` (`:106-142`): $R_d=R_{\mathrm{ref}}^{\top}R_{\mathrm{new}}$ as the
   quaternion $(w,v)$ (with the 180° case taken from the symmetric part). The twist is
   $\mathrm{normalize}(w,(v\cdot a)a)$, $R_{\mathrm{swing}}=R_dR_{\mathrm{twist}}^{\top}$, and the result
   is $R_{\mathrm{ref}}R_{\mathrm{swing}}$ with the swing and twist angles.

**Parameters (in use).** Node `save_pose_window` 30 and `save_pose_min` 5 (ICP source);
`fp_save_pose_window` 75 (~3 s at the tracker's ~25 Hz) (`icp_pose_refiner_node.py:393-407`).

**Users.** `icp_pose_refiner_node.py` (`_save_object`, `_fp_pose_for_save`, `_fp_icp_refine`);
`multiview_refine._tube_spin_deg` (judging a pipe by its axis tilt).

**Incidents.**
- 2026-09-25 (pose_jitter_probe): "The ICP tracker's pose on a STATIONARY part wandered by 5 mm std /
  24 mm range and up to 6 deg ... taking the last tracked pose at `save_object` was a single draw from
  that distribution" (`:3-7`).
- 2026-10-09 (node): R2's spin "corrected" by 30.8° by ICP, hence the swing-twist at save.

**Tests.** `test_pose_stats.py`: the mean recovers the truth and discards jumps; a single pose and the
chordal mean are sane; the stationary tail excludes the previous resting place; `remove_twist` keeps
the axis tilt and drops the spin (the R2 bench case of 30.8°); only an uncut tube has a symmetry axis.

**Thesis section.** Stationary Refinement and Saving; Symmetric and Cut Parts.

**FACT CHECK.**
- "its pose is averaged over the still frames (median translation, chordal mean of the rotations)".
  **Confirmed.** Add the MAD and 3° inlier gates and the stationary-tail rule (8 mm / 3° / 3 ticks).
- Eq. `eq:swing_twist`, $R_{\mathrm{ref}}^{\top}R_{\mathrm{new}}=R_{\mathrm{swing}}R_{\mathrm{twist}}(a)$,
  $R_{\mathrm{saved}}=R_{\mathrm{ref}}R_{\mathrm{swing}}$. **Confirmed** (`:117-142`). In the save path
  $R_{\mathrm{ref}}$ is the tracker's averaged pose and $R_{\mathrm{new}}$ is the save-ICP result; the
  gate then judges the swing angle only (`icp_pose_refiner_node.py:2004-2012`).
- "A plain tube's rotation about its own axis is not observable, so ICP and the tracker each return an
  arbitrary one". **Confirmed** (`:110-112`; the symmetry axis is defined in `weldgen_registry.symmetry_axis`).

#### C5. `assembly_mesh.py` (131 lines)

**Purpose.** Builds a watertight CAD mesh of the saved assembly, so the whole assembly can be uploaded
to the pose server as a new model (`export_mesh`, then the bridge's `add_model`). The original CADs are
re-instantiated at their stored poses and boolean-unioned, instead of reconstructing a mesh from the
static point cloud (SEPC), which would be "lossy at 2500 pts, and it needs a
Delaunay/graph-cut/manifold-repair stack" (`:1-24`).

**Algorithm.** `load_assembly` (`assembly.json`: model, `pose_static`); `_placed_trimesh` (scale the CAD
mm→m, fan-triangulate, refuse point clouds, apply `pose_static`); `trimesh.boolean.union` with the
manifold3d backend; re-centre on the bounding-box centre; scale back to mm. Statistics: vertices,
faces, components, watertight, volume, extents (`:74-119`). `build_and_write` writes the PLY.

**Users.** `icp_pose_refiner_node.py` (`~/export_mesh`). **Tests.** None. **Thesis.** Not in the draft
(optional: re-registering a tacked subassembly as one part). **FACT CHECK:** nothing in the draft.

### Group D: multi-view close-range refinement

#### D1. `multiview.py` (427 lines): the view planner

**Purpose.** Decides where the camera looks from after the parts are saved: K close views (0.40 m
instead of ~0.5 m from the scan home) from several directions at the centre of the assembly. The
per-view camera errors then average out in the joint refinement: the extrinsic translation, which
turns with the wrist, and the depth bias along each line of sight (`:1-9`). It is the D2 design of
`notes/multiview_refine_plan.md`.

**Algorithm (`plan_views`, `:275-427`).**
1. *Surfaces and boxes* `surfaces_from_models` (`:107-131`): 6000 area-uniform samples with outward
   normals per saved part, in the static frame. Each part's collision boxes are its registry shape's
   near-exact boxes (`part_shapes.Shape.boxes`: 6 per pipe, a chain per band), else its bounding box.
2. *Targets* `seam_targets` (`:161-174`). For each part's points, $d$ is the distance to the nearest box
   of **another** part. Weight 1 if 1 mm < d < 30 mm (the seam band, "where the weld roots will be");
   weight 0.1 elsewhere (background); points with d ≤ 1 mm (touching faces, hidden) are dropped.
3. *Candidates.* The target $c$ is the centre of the combined bounding box of all samples. Camera
   positions are $c+0.4\,(\cos e\cos a,\cos e\sin a,\sin e)$ for $e\in\{45^{\circ},60^{\circ}\}$, $a$
   every 30° and roll every 90°, i.e. 96 candidates. `look_at` (`:79-96`) puts the optical z at the
   target with roll 0 meaning image-down along world −Z.
4. *Visibility, cheap and first* `visible` (`:177-198`): depth $0.28<z<0.60$ m; inside the field of
   view (69° × 42°, D435i colour at 1280×720); incidence $n\cdot(c-p)\ge\cos60^{\circ}\,\|c-p\|$; and no
   part box on the segment from the camera to the point stopped 2 mm short of it (vectorised slab test,
   `_segments_hit_box`, `:139-158`). Candidates that see no seam-region point are dropped.
5. *Lazy greedy pick with diminishing returns* (`:333-361`). The value of a target point's k-th
   sighting is $\rho_k\in(1,0.5,0.25)$ and 0 after that. The gain is $g(v)=\sum_iw_i\,\rho_{c_i}\,[\text{seen}_i(v)]$.
   Directions within 30° of an earlier pick are excluded. The best remaining candidate is checked for
   feasibility, and the first feasible one is taken. Among the same direction at other rolls with gain
   ≥ 98 % of it, the one with the least L1 joint travel from home is chosen. This repeats n_views = 4 times.
6. *Feasibility* `feasible` (`:313-331`): IK on the locked elbow-up branch (`solve_on_branch`, seeds:
   the current joints, home, and 6 random seeds around home). It requires $|\sin q_5|\ge\sin0.35$ (wrist)
   and $|\sin q_3|\ge\sin0.35$ (elbow straight is the workspace edge), and a minimum distance ≥ the
   transit clearance (20 mm) for the arm and the tool (camera body included) against the parts'
   boxes, the fixtures and the table.
7. *Optional elevation spread* (`:362-379`): swap the weakest pick for the best feasible view that widens
   the spread. It is OFF (`min_elevation_spread_deg` = 0, `:55-59`).
8. *Order* (`:382-392`): nearest neighbour in L1 joint distance from the current joints. Each q is the
   2π-equivalent nearest the previous one inside the joint limits (`nearest_in_limits`, `:256-272`, with
   a 0.1 rad margin and wrist_3 in the cable window).
9. *Transits* (`:394-409`): `marking.transit_path` with `transit_model` (20 mm clearance), `unwrap=False`.
   With the **default planner** this means the straight joint edge, else the Python RRT-Connect plus
   collision-aware shortcut (C++ validity checks if built). There is no Cartesian stage and no APS for
   these views. The plan includes a path home.
10. *Result*: ok if ≥ 2 views and a home path exist. Coverage of the seam region that any candidate can
    see is reported (≥ 1 view, ≥ 2 views).

**Parameters (ViewConfig `:51-75`; ROS parameters `mv_*` in multiview_service).** distance 0.40 m;
elevations (45, 60)°; azimuth step 30°; roll step 90°; n_views 4; min_views 2; min separation 30°;
max incidence 60°; depth 0.28-0.60 m; hfov 69°, vfov 42°; seam band 30 mm; contact gap 1 mm; background
weight 0.1; repeat weights (1, 0.5, 0.25); occlusion margin 2 mm; 6000 points per part.

**Design decisions and incidents.**
- The elevation spread is off: "the planned 45/60 deg views already measure it to 0.16 mm, a 30 deg
  view is out of reach on the bench at 0.4 m, and its candidates doubled the planning time" (`:55-58`).
- Registry boxes, not bounding boxes: "SP3's bounding box covered the plate along the S and hid the
  seam region from every candidate view" (`:112-114`).
- The elbow check exists because "a far-side view at 0.78 m from the base came out at elbow -0.16 rad" (`:22-23`).
- 2026-10-01: "a plain unwrap to the previous view put wrist_3 at -7.97 rad, beyond the UR's ±2π" (`:259-261`).

**Users.** `multiview_service.py` (on the robot); `multiview_capture.py` and `multiview_refine.py`
(`_points_box_distance`); `scripts/multiview_refine_offline.py`; the ICP node.

**Tests.** `test_multiview_views.py`: look-at frame; visibility with depth range, incidence and
occlusion; the seam region is the strip beside the joint; `nearest_in_limits`; the plan for the bench
T has 4 feasible views at 0.40 m (never under 0.28 m), sees both faces of the ear, stays inside the
joint limits and away from the wrist and elbow singularities, and has collision-free transits ending
exactly at home; too few views fails cleanly. `test_multiview_curved.test_views_are_planned_round_the_bench_c1`.

**Thesis section.** Multi-View Close-Range Refinement / View Planning.

**FACT CHECK.**
- TODO for View Planning ("targets on the seam regions; per candidate view the visibility of each
  target (range, field of view, incidence, occlusion); greedy selection with diminishing returns; IK
  on the branch, singularity and collision checks; the order of the views and the transits"). **All
  present**, with the values above. Also: an *elbow* singularity check besides the wrist, and roll
  siblings resolved by joint travel.
- §mc_multiview intro, "the robot moves the camera to a few close views of the seams". **Nuance:** all
  views look at the centre of the assembly's bounding box from 0.40 m. They are *chosen* by how much of
  the seam band they see; they are not aimed at individual seams.

#### D2. `multiview_capture.py` (300 lines)

**Purpose.** The capture format on disk (one refine_pose run's raw data), view preprocessing (D4),
an exact synthetic renderer that writes the same format, and the replay that runs D4 plus D5-D7 on a
capture. The robot run and the offline script share one code path (`:1-23`).

**Algorithms.**
1. `save_capture`/`load_capture` (`:65-90`): `<results>/multiview/<timestamp>/` holds `capture.json`
   (time, extrinsic file and sha1, kinematics, table-plane file, and per view azimuth/elevation/roll,
   q, T_base_cam), `view_<k>.npz` (the organized cloud in the **camera** frame, per-pixel median, NaN
   invalid), and `assembly.json`. Keeping the camera frame lets a capture be replayed with another extrinsic.
2. `parts_from_assembly` (`:100-117`): 4000 CAD samples with outward normals, the vertices for the box,
   `pose_static` as the saved pose, `T_static_camera` as `T_scan`, and the true shape (`part_shapes`)
   when the registry has a non-stale entry.
3. `preprocess_view` (D4, `:129-149`): organized normals facing the camera; depth 0.15-1.0 m; transform
   to base with the T_base_cam stored at capture time; crop to within 30 mm of the saved parts'
   **bounding** boxes; table cut (drop points less than 2 mm above $z=ax+by+c$); 3 mm voxel downsampling
   with normals renormalised.
4. `render_view` (`:158-187`): every pixel ray is intersected exactly with each part's box (the plate
   parts are boxes); the nearest hit is kept; Gaussian noise is added along the ray (0.5 mm default).
5. `render_synthetic` (`:190-214`): rendered from the TRUE camera pose but stored with the pose the
   robot would believe, $T_{\mathrm{believed}}=T_{\mathrm{true}}$ with $t\leftarrow t+R_{\mathrm{cam}}d$.
   This is the error of an extrinsic whose translation is off by $R_{tc}d$ in tool0.
6. `refine_capture` (`:228-242`) and `format_replay` (`:245-300`): the report refine_pose replies with
   (accept/keep, correction, fitness, RMSE, owned points, overlap, single-view spread, unmeasured
   directions, resting gaps with their ISO 5817 no. 617 limit, the camera's scan-pose share, and the
   extrinsic estimate with its σ).

**Parameters (PreprocessConfig `:41-46`).** crop margin 30 mm; ground offset 2 mm; voxel 3 mm (also
ROS `mv_voxel_m`); depth 0.15-1.0 m. Synthetic: 424×240 pinhole with 69° hfov, noise 0.5 mm.

**Incident.** In the renderer, "A first version z-buffered surface samples and re-cast them along the
pixel-centre ray; at 1.3 mm pixels and 45-60 deg incidence that biased every view ~1 mm towards its
camera - which looks exactly like an extrinsic error along the optical axis" (`:163-166`).

**Users.** `multiview_service.py`; `scripts/multiview_refine_offline.py`.

**Tests.** `test_multiview_capture.py`: disk round trip; the renderer is exact (a 200×200×8 plate 0.4 m
below the camera reads 0.4 m exactly); preprocessing crops, cuts the table and keeps camera-facing
normals; a replay of the bench T without camera error lands within 0.5 mm / 0.2° of the truth; with a
camera error, online self-calibration; the offline script; the 2026-10-02 case (saved poses carrying
R_scan d); a real 3 mm fit-up gap measured and warned (level C limit 1.6 mm for 8 mm plates); and
resting contact would hide that gap.

**Thesis section.** Multi-View Close-Range Refinement (the capture and preprocessing are not in the
draft yet); Implementation Details (offline replay).

**FACT CHECK.** No direct draft statement. The draft omits the 16-frame per-pixel median, the crop and
table cut, and the voxel size (see (a)).

#### D3. `multiview_refine.py` (951 lines): the joint refinement (D5-D7)

**Purpose.** Given the saved parts (CAD samples with outward normals, `pose_static`) and K views (base
frame clouds with normals facing each view's camera), it computes refined poses, per-view diagnostics,
an estimate of the extrinsic translation error, and an accept/reject verdict per part. Four guards stop
touching parts drifting into each other (`:9-36`): ownership, a prior on the correction, an overlap
rule, and equal weight per view.

**Algorithms.**
1. *Stacking and view weights* `_stack` (`:529-534`): each point of view v gets $\omega_v=1/N_v$,
   normalised by the mean ("Equal weight per view", `:31-32`).
2. *Ownership* (`:280-291`). With $d_k(x)$ the distance from scene point x to part k's posed samples,
   $\mathrm{owner}(x)=\arg\min_kd_k$. It is −1 if $\min_kd_k>15$ mm (nobody's) or if the second-nearest
   $d<4$ mm (the dead band where parts meet, which pulls both ways). Ownership is fixed inside a solve
   and recomputed between rounds.
3. *Per-part MAP solve* `refine_part` (`:356-483`), centred on the part:
   $c=T_{\mathrm{prior}}\bar p$ (all coordinates are shifted by −c, so rotations turn about the part:
   "a 1 deg turn about the base_link origin, 0.5 m away, would move the part 9 mm"). Each Gauss-Newton
   iteration (at most 30):
   - correspondences $q_{\mathrm{NN}}$ for the posed samples, kept if dist < 10 mm and the normal gate
     $\ge\cos60^{\circ}$ holds; $r_i=n_i^{\top}(Tp_i-q_i)$;
   - IRLS weights $w_i=\exp(-r_i^2/2\nu^2)\cdot\omega_{v(i)}/\bar\omega$, with
     $\nu_0=\max(3\,\mathrm{median}|r|,\ P_{90}|r|,\ \sigma)$ shrinking as $\nu\leftarrow\max(0.7\nu,\sigma)$,
     $\sigma$ = 1.5 mm;
   - overlap rows for the contact samples $s_c$ (up to 200 model points within 8 mm of a neighbour):
     $sd_c$ is the signed distance to the neighbour's true shape (or box) with its normal $n_c$; active
     only if $sd_c<-\varepsilon$, residual $\sqrt{w_o}\,(sd_c+\varepsilon)$, row $\sqrt{w_o}[s_c\times n_c,\ n_c]$,
     with $w_o$ = 10 and ε = 0.5 mm;
   - optional resting rows (off by default);
   - prior rows $\mathrm{diag}(L)\,(\xi+x)$, with
     $L=(\sigma/\sigma_\theta\cdot\mathbf 1_3,\ \sigma/\sigma_t\cdot\mathbf 1_3)$, $\sigma_\theta$ = 1°,
     $\sigma_t$ = 3 mm, and $\xi=(\mathrm{rotvec}(D),D_t)$ for $D=T\,T_{\mathrm{prior}}^{-1}$;
   - solve the stacked least squares, $T\leftarrow\exp(x)\,T$.

   The linearised objective is therefore
   $$\min_{\xi}\ \sum_i\omega_{v(i)}\,w_i\,r_i^2+\sigma^2\,\xi^{\top}\Sigma^{-1}\xi+w_o\sum_c\max(0,-sd_c-\varepsilon)^2,$$
   with $\Sigma=\mathrm{diag}(\sigma_\theta^2I_3,\sigma_t^2I_3)$.
   Afterwards it computes the data information
   $H=\sum_i\omega_iJ_i^{\top}J_i$ with $J_i=[(p_i\times n_i)/\rho,\ n_i]$, where ρ is the 90th-percentile
   radius of the part's samples (rotation expressed in metres at the part's size), plus fitness,
   RMSE, the largest penetration, and the min/max gap of down-facing faces resting on each neighbour.
4. *Rounds* `refine_assembly` (`:537-641`): 2 rounds (block coordinate descent). Parts are taken in
   order of owned-point count, each against the others at their current poses; it stops early when no
   part moved more than 0.2 mm / 0.05°.
5. *Observability* `weak_directions` (`:486-504`): eigen-decompose H. A direction is weak if
   $\sigma/\sqrt\lambda>0.5$ mm, i.e. $\lambda<(\sigma/0.5\,\mathrm{mm})^2$. It is named a "slide" or a
   "turn". `_symmetric_spin` (`:507-525`) renames a pipe's spin about its own axis "symmetric: needs no
   measuring" and removes it from the judged vectors.
6. *Per-view diagnostic* `_per_view` (`:687-718`): for every view with ≥ 50 owned points, the part
   registered to that view alone, **translation only and without the prior**. The spread of the centres
   is reported only in measured directions. ("letting it turn, or pulling it back with the prior, bent
   the per-view poses and biased the extrinsic estimate by up to 1.5 mm", `:367-370`.)
7. *Extrinsic estimate* `estimate_extrinsic` (`:721-819`): one least-squares problem over all owned
   points with unknowns $(\xi_1..\xi_n,d)$:
   $r_{v,i}=n_i^{\top}(T_kp_i-q_i+R_vd)$, Jacobian blocks $[(p-c_k)\times n,\ n]$ for part k and
   $n^{\top}R_v$ for d, with Welsch IRLS ($\nu_0=\max(P_{90},2\sigma)$, ×0.7 down to σ; 20 iterations).
   Then the normal matrix $N=A^{\top}A$ gives the Schur complement $S=N_{dd}-N_{dx}N_{xx}^{+}N_{xd}$.
   Its information in mm⁻² is $\lambda/\sigma^2/10^6$, the std per eigen-direction is
   $1/\sqrt{\lambda_{\mathrm{mm}}}$, and only components with σ ≤ 0.5 mm are kept:
   $d\leftarrow U_sU_s^{\top}d$. The weak directions, the conditioning, the RMS and the full 3×3
   information (for selfcal) are reported.
8. *Online self-calibration* (`:588-629`). If $\|d\|>0.5$ mm and `online_selfcal`: set $d_{\mathrm{tot}}=d$
   and repeat up to 4 times: remove $R_vd_{\mathrm{tot}}$ from every view's points and
   $R_{\mathrm{scan}}d_{\mathrm{tot}}$ from every saved pose's translation (the camera that registered
   the part, `T_static_camera`); refine again (inner call); stop if the residual d < 0.5 mm, else add it.
   Results are judged against the corrected prior and reported against the saved pose. The converged
   $d_{\mathrm{tot}}$ is what the history records. If `online_selfcal` is off, a large d rejects every
   part (`:630-637`).
9. *Acceptance* `judge` (`:885-951`). A part is refused if:
   (i) its correction exceeds 10 mm or 5° (for a pipe, the axis tilt from `remove_twist`; a spin over
   0.5° is warned); (ii) fitness < 0.2; (iii) its penetration exceeds 1.0 mm **and** the saved assembly's
   by more than 0.2 mm (over 1.0 mm but not worse: applied with a warning); (iv) the relative pose of two
   parts changed more than 2 mm or 1° **and** more than 2 mm of a part's own correction lies along the
   directions its views did not measure.
10. *Fallback cascade* `_fallback` (`:644-684`): a rejected part keeps its saved pose; every accepted part
    is refined once more against it; an accepted part whose pose relative to a held one still changed
    beyond 2 mm / 1° is held too, repeated until stable. "refined together or not at all".
11. *Fit-up warnings* `fitup_warnings` (`:841-858`) with `fitup_gap_limit_mm` (`:826-838`): the gap of the
    down-facing faces resting on a neighbour is compared with ISO 5817:2023 Table 1 no. 617 (fillet
    root gap) at quality level C, using weldgen's `root_gap_limit(t, 0.7t, level)` with t the smaller
    of the two parts' **smallest bounding-box extents** (`_thickness_mm`, `:822-823`). Over the limit:
    a WARNING, never a rejection.
12. `parallel_face_gap` / `corr_distance` (`:294-338`): the smallest gap between same-facing parallel
    faces of different parts that a camera can see. It is reported. The cap $d_{\max}\le\mathrm{gap}/2$
    (`parallel_gap_rule`) is OFF.

**Key parameters (RefineConfig `:58-129`; in use via multiview_service PARAMS).** σ (noise_m) 1.5 mm;
max_corr 10 mm; normal gate 60°; dead band 4 mm; own_max 15 mm; prior 3 mm / 1°; overlap band 8 mm;
200 contact samples; ε (penetration_tol) 0.5 mm; $w_o$ 10; rounds 2; max_iter 30; welsch_k 3;
observable_max_sigma 0.5 mm; extrinsic_max_sigma 0.5 mm; online_selfcal True, min 0.5 mm, 4 iterations;
converge 0.2 mm / 0.05°; max correction 10 mm / 5°; min fitness 0.2; max penetration 1.0 mm; max
relative 2 mm / 1°; resting_contact False; quality level C.

**Design decisions and incidents (dated, from the comments).**
- Ownership: the normal gate already separates parts meeting at a right angle, so ownership changes
  nothing for a T. It exists for parallel faces (`:15-18`).
- parallel_gap_rule OFF: "in the synthetic lap joint no swap happened even without ownership at 10 mm
  ... while the cap cost correction range (a 6 mm error was corrected without it, stuck with it)" (`:62-67`).
- resting_contact OFF, 2026-10-02, the user's call: "the gap must be MEASURED and warned about ... On the
  bench the pull overrode the data (a synthetic 3 mm gap closed to < 1 mm, unwarned) and ... the views
  alone already determine the standing plate (4 real runs, two registrations: consistent to 0.4 mm)"
  (`:75-90`). Only down-facing faces rest: "pulling the base's top up to the ear's foot tilted the base
  ... (bench, 2026-10-02: the base differed 2-3 mm run to run)" (`:424-428`).
- The observability rule is absolute, not relative: "a relative rule called the ear's in-plane rotation
  'not measured' although its resting contact (~50 samples) pinned it" (`:100-104`). For d: "the planned
  T views measure their weakest direction of d to 0.16 mm (actual errors 0.2-0.33 mm over 4 noise
  draws) - a 5 % relative rule had thrown it away" (`:105-109`).
- Online self-cal: "synthetic: within 0.5 mm, against a 1.2 deg tilt the uncorrected joint fit had
  accepted" (`:111-114`). Iterated because on the "bench 2026-10-02: 0.8 mm left after one pass, which
  the inner pass then REJECTED" (`:117-120`). R_scan d is removed from the prior because without it
  "what the views do not measure stayed at the uncorrected saved pose: a fake 8 mm fit-up change on the
  bench, 2026-10-02" (`:592-596`).
- Max rotation limit history: "3 until 2026-10-02 (an ear correction of 3.07 deg was refused), 4 until
  2026-10-15 (SP3's 4.5 deg refused; the user raised it)" (`:125`).
- The judge, 2026-10-15, the user's rule ("a correction within the limits is applied"): "E2's saved pose
  overlapped its plate as much or more, and the refinement was refused for exceeding max_penetration_mm,
  keeping the worse assembly"; and "E2's 5.13 deg was 5.08 spin + 0.71 tilt, and the 5 deg limit refused
  it" (`:891-897`, `:905-907`).
- ν₀ uses the 90th percentile too: "when most matched points slide along their faces (residual 0) the
  median says nothing, and the few faces that DO see the error (a plate's end faces, 3 mm) were weighted
  away before the first step" (`:399-402`).
- The first version of the extrinsic estimate regressed per-view positions on R_cam: "from the TRUE
  poses it returned (7, 13, -15) mm for an injected (3, -2, 1)" (`:738-740`).
- The parallel-face check counts only faces a camera sees: "the base's underside and the ear's foot face
  are 8.9 mm apart and both face down" (`:298-300`).
- Why its own GN loop and not `icp_point_to_plane`: the prior must be about the part's centre and the
  overlap rule is a second residual type (`:38-42`).

**Users.** `multiview_capture.refine_capture` (hence `multiview_service` and the offline script);
`test_selfcal.py`.

**Tests.**
- `test_multiview_touching_parts.py` (bench T, synthetic views with 0.5 mm noise): measured directions
  land on the truth and unmeasured ones stay and are reported; a camera error d is estimated, the parts
  rejected, then self-calibrated; rounds converge independently of order; ownership dead band; the prior
  holds an unmeasured slide and low views measure it; overlap (2 mm sunk → < 0.5 mm, a real 1 mm gap
  kept); the lap joint; the acceptance rules.
- `test_multiview_curved.py`: C1 registered 8 mm off is brought back to < 1 mm; S3 on R2 with true shapes
  does not penetrate (with boxes it sat ~5 mm inside and did not refine); RR1 3 mm off is brought back;
  E2 on its mitre refines (with boxes the plate was pushed 49.7 mm); the 2026-10-15 judge rules.
- `test_multiview_capture.py`; `test_selfcal.py`.

Note: the module docstring of `test_multiview_touching_parts.py` says the lap joint is kept apart by
"ownership plus max_corr <= half the gap". That contradicts the test body and RefineConfig: the rule is
off and not needed. The docstring is stale.

**Thesis section.** Joint Refinement of Touching Parts; Observability; Online Estimation of the Camera's
Extrinsic Translation; Acceptance; Fit-Up Gap and Weldability (the ISO check lives here).

**FACT CHECK (ch3 §mc_multiview).**
- Eq. `eq:mv_objective`. **Mostly confirmed; three precisions needed.**
  (1) The data term carries the per-view weight $\omega_v=1/N_v$ (guard 4), which the equation omits.
  (2) It is IRLS with Welsch weights, $\sum\omega_vw(r)r^2$; "$\rho(r)^2$" is loose notation, and
      correspondences are gated (≤ 10 mm, normal gate 60°).
  (3) Values: σ = 1.5 mm, $\Sigma=\mathrm{diag}((1^{\circ})^2I,(3\,\mathrm{mm})^2I)$, $w_o$ = 10,
      ε = 0.5 mm. The prior and overlap terms are **confirmed** as written.
- "The neighbours are their true shapes, not bounding boxes: a tube's bore is hollow, a cut tube ends at
  its cut". **Confirmed** when the registry entry exists and is not stale (`box_of`, `:227-236`;
  `part_shapes._sdf_tube`). Otherwise the bounding box is used.
- "The parts take turns (block coordinate descent)". **Confirmed:** 2 rounds, most-owned first.
- TODO "Ownership with a dead band; the signed distance of each primitive (slab, tube with its bore, cut
  tube, swept band); two rounds". All present: dead band 4 mm, own_max 15 mm, SDFs in D6, 2 rounds.
- Eq. `eq:observability`, $\sigma_{\mathrm{dir}}=\sigma_{\mathrm{noise}}/\sqrt\lambda\le0.5$ mm.
  **Confirmed** (`:104`, `:493`), with $\sigma_{\mathrm{noise}}$ = 1.5 mm, H in $(\rho\omega,t)$ coordinates
  (the rotation scaled by the part's ρ), and $W$ = the per-view weights (no Welsch weights in H).
- "Directions that are not measured keep their prior value". **Nuance.** They are *held by the prior*
  (MAP with σ 3 mm / 1°), not frozen. The weak directions are used by the relative-pose acceptance rule,
  and a pipe's spin is exempted as symmetric.
- Eq. `eq:extrinsic_d` and the Schur complement. **Confirmed** (`:753`, `:804`). The threshold is
  σ ≤ 0.5 mm per eigen-direction; undetermined components are set to 0.
- TODO "Removing $R_vd$ from the views and $R_{\mathrm{scan}}d$ from the saved poses; why the joint fit
  alone does not remove d". The material is above (online self-cal steps; the 2026-10-02 fake 8 mm
  fit-up change; selfcal.py `:5-8`: "the joint refinement does not average such an error out (the part
  shared by all views stays in the pose)").
- `tab:mv_acceptance`:
  - Correction size > 10 mm or > 5°, for a pipe its axis tilt and not its spin. **Confirmed**
    (`:124-125`, `:903-914`). With online self-cal the limit judges the correction *beyond* the
    camera's scan-pose share $R_{\mathrm{scan}}d$.
  - Overlap "worse than the saved assembly's by more than 0.2 mm". **Incomplete:** the refusal needs
    penetration > **1.0 mm** *and* > saved + 0.2 mm; over 1.0 mm but no worse is applied with a warning
    (`:127`, `:917-924`).
  - Fit "to fill". The rules are fitness < 0.2 (`:126`, `:915`), and the relative pose of two parts
    changing > 2 mm or > 1° with > 2 mm of it along unmeasured directions (`:128-129`, `:927-951`).
  - Rows missing from the table: an extrinsic disagreement $\|d\|>0.5$ mm with online self-cal off
    keeps every part as saved (`:630-637`); the fallback cascade (`:644-684`); fit-up beyond ISO is a
    warning, not a refusal (`:841-858`).

#### D4. `multiview_service.py` (341 lines): `~/refine_pose` on the robot

**Purpose.** The orchestration behind the ICP node's `~/refine_pose` (and `~/abort_refine`): plan,
move, capture, return home, save, refine, apply, record (`:1-28`).

**Algorithm (`MultiviewRefiner._run`, `:236-301`; `_finish`, `:315-341`).**
1. Preconditions: ≥ 1 saved part; joint states; the extrinsic in TF (tool0 ← camera) equal to
   `mv_extrinsic_path` within 0.1 mm / 0.01° (`_check_extrinsic`, `:137-150`).
2. Plan the views (`plan_views`) from where the arm is. The collision model contains the parts' boxes
   plus the fixtures and the table plane, with the tack clearance as base; the transits raise it to
   20 mm. The views are published as RViz markers. In a dry run it stops here. `capture_here` takes one
   view from the current pose, with no motion.
3. Tracking is paused for the run ("the live crop would chase a moving camera").
4. Per view: execute the transit (`TrajectoryExecutor`, prefix `mv_`: force watchdog, joint-jump gate,
   bias re-measured before the move) timed by `time_joint_path` (a speed cap of 0.3 rad/s, **not TOTG**);
   settle 0.5 s; take the per-pixel nanmedian of 16 fresh organized clouds stamped after the arrival
   (at least n/2 required); look up T_base_cam from TF at the first stamp.
5. Return home (wrist unwound). Save the capture, refine it (`refine_capture`), and publish the stacked
   views. The ICP node applies the accepted poses and rebuilds and persists the SEPC and `assembly.json`.
   The extrinsic estimate is recorded into `notebooks/selfcal_history.json` only if at least one
   direction is determined.
6. A failed or aborted move stops the run where it is, with no automatic recovery motion.

**Parameters (`PARAMS`, `:58-92`, declared as `mv_<name>`).** dry_run False ("the user's choice
2026-10-01"); capture_here False; view_distance 0.40; n_views 4; elevations [45, 60]; azimuth step 30;
max incidence 60; **n_frames 16** ("8 until 2026-10-02 (the user: more frames, they are cheap)");
settle 0.5 s; frame_timeout 8 s ("the cloud arrives at ~6 Hz: 16 frames take ~2.7 s"); v_joint 0.3 rad/s;
extrinsic_tf_tol 0.1 mm; record_selfcal True; online_selfcal True; max correction 10 mm / 5° ("4 until
2026-10-15 (the user: SP3's 4.5 deg refused)"); prior 3 mm / 1° ("on the 2026-10-06 capture 3 / 8 / 15 mm
changed the result by < 0.5 mm: the four close views already dominate"); max relative 2 mm / 1°;
resting_contact False; quality level C; voxel 3 mm. The executor's own `mv_abort_force_n` is 8 N, and
the touch detection is disabled (`touch_force_n=1e9`).

**Users.** `scripts/icp_pose_refiner_node.py`. **Tests.** None (ROS-bound). Its pieces are tested in D1-D3.

**Thesis section.** Multi-View Close-Range Refinement (the procedure); ROS 2 Architecture
(services as pipeline steps). **FACT CHECK:** the draft has no statements beyond D1-D3. Missing items:
16-frame median, settle, paused tracking, extrinsic-TF check, no recovery motion, speed-capped timing.

#### D5. `selfcal.py` (189 lines)

**Purpose.** Persistent self-calibration of the extrinsic's **translation** from many refine_pose runs
(step 7 of the plan). Each run's $d$ (camera frame) and its 3×3 information are recorded with the sha1
of the extrinsic file they were made with. Runs are combined by information, and a corrected extrinsic
is written next to the one in use when enough runs agree. **Promotion is by hand** (`:1-30`).

**Algorithms.**
1. Sign (`:10-14`): with the true translation $t_{\mathrm{true}}=t_{\mathrm{used}}-\Delta$ in tool0, a camera
   point is displaced by $R_{BT}\Delta=R_{BC}(R_{tc}^{\top}\Delta)$, so $d=R_{tc}^{\top}\Delta$ and the
   correction is $t_{\mathrm{true}}=t_{\mathrm{used}}-R_{tc}d$ (`corrected_extrinsic`, `:57-61`).
2. `combine` (`:112-119`): $I=\sum_rI_r$, $d=I^{+}\sum_rI_rd_r$, projected onto the directions with
   $\lambda(I)\ge1/\sigma_{\max}^2$ (σ_max = 0.5 mm); the undetermined directions are named.
3. `evaluate` (`:122-160`): use only runs made with this extrinsic file (same sha1) that determine at
   least one direction. Require ≥ 3 runs. Spread = $\sqrt{\mathrm{mean}_r\|P_r(d_r-d)\|^2}$ ≤ 1 mm, with
   $P_r$ projecting on the directions run r determines. Leave-one-out: the maximum shift of $d$ when one
   run is dropped ≤ 1 mm.
4. `write_selfcal` (`:163-189`): `T_tcp_to_cam_selfcal.npy` plus a JSON sidecar (base extrinsic and sha1,
   d, the tool0 shift, spread, LOO, undetermined directions, runs, kinematics). The rotation is
   unchanged; that stays the table refinement's job.

**Parameters (SelfcalConfig `:46-50`).** min_runs 3; max_spread 1.0 mm; max_loo 1.0 mm; max_sigma 0.5 mm.

**Users.** `multiview_service.py` (record_run); `scripts/selfcal_extrinsic.py`;
`scripts/multiview_refine_offline.py`.

**Tests.** `test_selfcal.py`: the sign through the chain; three synthetic runs recover the extrinsic
within 0.5 mm; an undetermined direction is left as it was; the agreement rules; runs fill each other's
blind directions; nothing is written without agreement.

**Thesis section.** Online Estimation of the Camera's Extrinsic Translation; the calibration table row
"Extrinsic translation".

**FACT CHECK.** `tab:calib_chain`, "Extrinsic translation: online, from multi-view disagreement".
**Needs precision.** *Online*, each run removes its own $d$ from that run's views and priors
(multiview_refine). The *calibration file* is corrected offline from ≥ 3 agreeing runs (spread ≤ 1 mm,
LOO ≤ 1 mm, same extrinsic sha1) and promoted by hand.

#### D6. `part_shapes.py` (245 lines)

**Purpose.** The true surface of a registered part, from its registry entry, for the multi-view
refinement (neighbour distances, overlap and fit-up) and for view planning (near-exact boxes). It also
detects a registry entry that no longer describes its CAD (`:1-27`).

**Algorithms.**
1. `Shape.sdf` (`:74-84`): points in the CAD frame (m) → the primitive frame (mm) → the signed distance
   (negative inside) and the outward normal of the nearest surface piece. `_combine` (`:86-102`) is a
   box-like combination of per-axis terms: inside, the largest term; outside, the norm of the positive
   terms. It is exact on faces and approximate near edges.
2. Slab: box terms $|x_k|-h_k$ (`:104-112`).
3. Tube (`:114-146`): terms $\rho-r_o$, $r_i-\rho$ (the **bore is hollow**), $z-L$, and the base term
   $d_{\mathrm{base}}=(h_{\mathrm{cut}}(\phi,\rho)-z)\cdot\mathrm{clip}(|n_{\mathrm{base},z}|,0.2,1)$.
   The cut height is evaluated at the point's own radius (clipped to $r_o$). The base normal is
   −z (flat), the plane normal (mitre), or towards the cut cylinder's axis (saddle).
4. Band (`:148-171`): the offset across the spine $s=(x-P)\cdot n$ against $[o_{lo},o_{hi}]$, the z caps
   $[z_0,z_1]$, and the end caps for an open spine. The nearest spine sample comes from a KD-tree over
   ≥ 512 samples.
5. `boxes` (`:174-181`): `collision.boxes_from_parts` at a pose (6 rotated boxes per pipe, sectors under
   a cut, a box chain per band).
6. `stale_entries` / `load_shapes(check=True)` (`:188-245`): an entry is stale if any CAD vertex lies more
   than **0.5 mm** from the primitive surface (`mesh_deviation_mm`, one direction). A stale entry is not
   used; the part keeps its bounding box in the multi-view, with a note.

**Incidents.**
- (`:4-8`) "S3's saddle end wraps round R2 and sits deep inside R2's box, so S3 read as ~5 mm inside it
  (five times the overlap rule's 1 mm). The plate under E2's mitred end lies inside E2's box. SP3's box
  covers most of the plate along the S, and it hid the very seam region the views are planned to see."
- Hollow bore, 2026-10-15 (`:141-143`): "with the bore solid, E2 tilted 2.6 deg on its plate made the
  plate read 4.3 mm 'inside' E2 and refine_pose refused it". The planner's boxes stay solid.
- Cut height at the point's radius (`:120-123`): "a saddle's varies across the pipe by up to 4.5 mm over
  S3's bore: evaluated at the bore radius, R2's surface under S3's bore read as 3.6 mm inside S3".
- Stale entry, 2026-10-15 (`:184-187`): "the new 220 x 220 x 5.5 mm plate kept the old 256 x 249 x 8 mm
  entry, turned 90 deg, and mode A found no seam at all, silently".

**Users.** `multiview.surfaces_from_models`, `multiview_capture.parts_from_assembly`; the ICP node
(`stale_entries` before mode A refuses, `icp_pose_refiner_node.py:1165-1173`).

**Tests.** `test_multiview_curved.test_shapes_match_their_cad` (every CAD vertex on its shape, normals
out, sign agreeing with the primitive's inside test);
`test_curved_registry.test_a_cad_replaced_after_the_registry_is_caught`.

**Thesis section.** Joint Refinement of Touching Parts (true shapes); CAD Parts as Primitives (stale check).

**FACT CHECK.** §mc_registry, "a registry entry whose CAD file has changed since the fit is detected and
refused". **Confirmed in effect, with a nuance.** The detection is *geometric* (a CAD vertex > 0.5 mm
off the primitive), not a file hash or timestamp. Mode A then refuses with a message
(`icp_pose_refiner_node.py:1167-1173`); the multi-view falls back to the bounding box with a warning.

### Group E: seams and tacks from the registered assembly

#### E1. `weldgen_registry.py` (621 lines): CAD → exact primitive

**Purpose.** Mode A needs every library CAD described as an exact `weld_generator` primitive, together
with $T_{\mathrm{cad,prim}}$ (primitive-local mm → CAD frame mm). This module derives that description
automatically and verifies it. A slab may be accepted as the envelope of a notched or tabbed plate,
with the ignored fraction recorded. Anything it cannot verify is recorded as unsupported, with the
reason, "never guessed". Units are mm; the registry version is `weldgen_registry-1.0` (`:1-35`).

**Algorithms.**
1. `derive_box` (`:43-102`): the oriented bounding box, with axes sorted u = longest, v = middle,
   w = thickness, made right-handed.
   - **exact**: every vertex is a corner (within 0.05 mm) and the volume error ≤ 1 %;
   - **envelope**: every vertex lies on the box surface (inside within 0.05 mm and on at least one face)
     and the mesh fills ≥ 90 % of the box (deficit between −1 % and 10 %). The features are ignored: a
     seam across a notch is labelled at its nominal length; a tab through the other part shows as a
     nominal penetration in `fitup_mm`;
   - otherwise unsupported.
2. `derive_tube` (`:250-315`).
   - *Sweep axis* `_sweep_axis` (`:181-194`): the least-spread direction of the area-weighted face
     normals, the smallest eigenvector of $\sum_fA_fn_fn_f^{\top}$, re-estimated 4× on faces with
     $|n\cdot a|<0.15$.
   - *Lateral faces*: those with $|n\cdot a|<10^{-3}$; a Kasa circle fit gives the centre (`:197-202`).
     All lateral vertices must lie on two radii within 0.05 mm, with $r_o-r_i\ge0.5$ mm.
   - *Each end* (`_classify_end`, `:232-247`) is **flat**, **plane** (a mitre: all normals within 0.05°),
     or **cylinder** (a saddle: `_fit_cylinder` with a free axis, by least squares on
     $\|(p-q)\times m\|-r$, `:205-229`). Each end must fit within 0.05 mm, and a flat end is required
     for the top cap.
   - *Frame*: local +z along the axis, base at z = 0, top at z = L; `params` = r_outer, wall, length,
     base_cut.
3. `derive_extrusion` (`:462-556`): a straight extrusion with two flat caps, with the sweep axis as
   above (side faces may lean up to 2.7° on SP3, so "the caps decide, not the sides").
   - *Closed rounded-rect tube* (two cap loops): an oriented 2-D box; walls equal on both sides within
     0.05 mm; corners fitted as arcs (least squares on r). The spine is weldgen's exact
     `rounded_rect_curve` (lines and arcs, **not a B-spline**); band [0, wall].
   - *Open band* (one loop): the wall measured along inward vertex normals (`_band_thickness`). If the
     5th-95th percentile range exceeds 0.05 mm it is refused "with the range" ("not an offset of one
     spine"). Otherwise `_fit_open_spine` (`:417-459`) finds the 4 corners, sides A and B and the
     midline, and tries 4..16 control points for all three. The first cubic within tol/2 = 0.025 mm wins.
     `_fit_bspline` (`:381-414`) is a clamped uniform cubic with the ends interpolated; control points
     and every point's parameter are solved together by nonlinear least squares, started by 30 rounds of
     linear fit alternating with re-projection.
4. `derive_entry` (`:559-576`): slab, then tube, then extrusion; the most specific refusal is reported.
5. `build_registry` (`:579-600`): every `*.ply`. Hand-edited entries, and non-slab entries not
   "fitted" here, are kept.
6. `verify_entry` / `verify_both_ways` (`:105-142`): CAD vertices → primitive mesh distance, primitive
   mesh vertices → CAD distance, and the volume relative error ("an entry that is too long, or a cut on
   the wrong side, shows here, and the first direction cannot see it"; volume "catches a hollow / solid
   mix-up"). The budget is the D34 chord budget, 0.25 mm.
7. `symmetry_axis` (`:611-621`): only an uncut tube (C1, R2) is fully symmetric. A cut end (E2's mitre,
   S3's saddle) fixes the spin; a rounded-rect tube is only 4-fold symmetric.

**Parameters.** fit tolerance 0.05 mm; envelope max deficit 10 %; the verification budget 0.25 mm and
volume 1 % (`scripts/build_weldgen_registry.py:38-55`, `--verify` only).

**Current registry (models/weldgen_objects.json, as built).** Exact slabs: plate 150×100×4,
smallplate 100×50×4, test_objv2_base 220×220×5.5, test_objv2_ear 250×99×8. Fitted tubes, verified both
ways: C1 0.048 mm, E2 0.049 mm, R2 0.049 mm, S3 0.057 mm. Swept slabs: RR1 0.037 mm, SP3 0.050 mm,
270circle 0.067 mm. Volume errors are ≤ 0.24 %. Unsupported: assembly_mesh, vplate. (These are
results, for Chapter 6.)

**Users.** `scripts/build_weldgen_registry.py`; `part_shapes`; the ICP node (`symmetry_axis`, the
registry for mode A); `tack_marking_node`, `tack_reachability`, `contact_error`.

**Tests.** `test_curved_registry.py`: flat, mitred and saddle-cut tubes fitted back and verified both
ways; rounded-rect → closed swept_slab; a sideways-shifted band refused with its wall range; the printed
C1/E2/R2/S3/RR1 verify; SP3 recovers its drawn spline to micrometres; a constant-wall spline band is
fitted back; a tube's collision box wraps the tube; a replaced CAD is caught.
`test_seam_from_registration.py`: a library plate is a slab with its corners on the box; a bent or
composite mesh is unsupported; a notched plate is its envelope slab.
`test_pose_stats.test_only_an_uncut_tube_has_a_symmetry_axis`.

**Thesis section.** Seams from the Registered Assembly / CAD Parts as Primitives; Symmetric and Cut Parts.

**FACT CHECK (§mc_registry).**
- "a slab (plate), a tube (with its bore, and its cuts), or a swept slab". **Confirmed**, with two
  precisions. A tube has **one** cut end (the base: flat, plane or cylinder) and a flat top cap. A slab
  can be an **envelope** of a notched plate.
- "a swept slab (a band along a spine curve, fitted as a B-spline)". **Partly wrong.** Only an *open*
  band gets a fitted cubic B-spline. A closed rounded-rect tube's spine is an exact rounded rectangle
  (`:549-555`).
- "A fit is accepted when it is verified both ways ... within a fraction of a millimetre, and the volumes
  agree". **Not what the code enforces.** A fit is *accepted* by the derive functions' own tolerance
  (0.05 mm; the slab volume ≤ 1 %). The both-ways verification (0.25 mm budget, volume ≤ 1 %) runs only
  under `build_weldgen_registry.py --verify` and only sets `verify_failed`, which no consumer reads.
  The library parts do verify at 0.037-0.067 mm. Suggested wording: "each fit is accepted within
  0.05 mm and then verified in both directions against the CAD (0.25 mm budget, volume within 1 %); all
  library parts verify to below 0.07 mm".

#### E2. `seam_from_registration.py` (580 lines): mode A, planar seams and tacks

**Purpose.** Computes the seams from the registered poses (never detected): the parts are posed weldgen
primitives, and each inter-part face pair is judged by a **registration-tolerant** version of WeldSet's
D4 accessibility rule. Every candidate is kept with its class, polyline, approach axis, weldability,
reject reason and fit-up. Tacks come from weldgen's `tackrule-0.1` unchanged (`:1-31`).

**Algorithms.**
1. `posed_parts` (`:66-95`): $T_{\mathrm{world,part}}=T_{\mathrm{static,cad}}\,(\text{m}\to\text{mm})\cdot T_{\mathrm{cad,prim}}$,
   then `weldgen.geom.from_object`. A missing entry raises `RegistryError`, naming the part; the ICP
   node then falls back to its radius-PCA edge detector unless `weld_fallback_pca=false` (see (a)).
2. `runtime_access` (`:98-123`): `pose_tol_mm` = 10 (clipped to [0.5, 30]), i.e. "the relative pose
   error of the two registrations, not the root gap", from the bench (8-11 mm). `coplanar_tol` =
   min(tol, ½ t_min of the flat parts); `min_seam_length` 10 mm; `bore_min_diameter` 200 mm. Node
   parameter `weld_pose_tol_mm`.
3. `judge_registered(A, fa, B, fb)` (`:229-306`):
   - (a) Parallel planes: coplanar ones go to weldgen's coplanar candidate (butt/edge); facing ones
     within the tolerance and overlapping are a non-weldable "bisector_blocked" or "fixture_contact".
   - (b) Otherwise intersect the planes: $d=(n_a\times n_b)/\|n_a\times n_b\|$ and $x_0$ with
     $n_i^{\top}x_0=n_i^{\top}p_i$ (`acc.intersect_planes`).
   - (c) Clip to both faces with `clip_registered` (`:175-226`): the in-face axis the line holds
     **nearly** constant gets the tolerance as slack, the other axis is strict, so the run is never
     extended along the seam.
   - (d) The line separation (`acc._line_separation`) must be ≤ tol.
   - (e) The face-extent rule `_extends_beyond` (`:148-172`): each face must reach past the other's plane
     by more than the tolerance ("yes"); "behind" means drop (the wrap-around phantom of a plate's own
     underside); "short" means non-weldable `member_within_pose_tol` ("a lapping sheet thinner than the
     pose error cannot be told from a penetrating one").
   - (f) Fit-up `_edge_fitup` (`:141-145`): the signed distances of the two near corners of each face to
     the other plane (> 0 gap, < 0 penetration). A penetration is registration error: reported, not a
     rejection.
   - (g) Verdict in order: member_within_pose_tol, fixture_contact, degenerate_dihedral (weldgen's
     dihedral range), too_short (< 10 mm), bisector_blocked (no clear torch axis about the bisector
     $n_a+n_b$, `acc._clear_axis` over 5 samples). Otherwise weldable, with that approach axis.
4. `enumerate_registered` (`:319-344`): all flat-flat face pairs, then weldgen's post-passes
   (`_suppress_toes`, `_drop_cross_runs`) and ordering.
5. `compute_seams` (`:347-406`): the planar seams as polylines at 1 point/mm, then **canonical direction**
   (the dominant component of $p_1-p_0$ positive, so the two fillets of one joint count from the same
   end), then the curved pairs (`curved_seams.pair_seams`), then sorting (weldable first, longer first)
   and renumbering.
6. `compute_tacks` (`:409-471`): weldgen `tack_rule` on the weldable seams with t = the thinnest member.
   Per tack: `tack_no` (1-based along its seam from $p_0$, the label "to show and to reach by"); `order`
   (the scene-wide weld sequence: ends first, then bisection, same-class seams interleaved round-robin);
   $p_0/p_1$ bounding `tack_length_mm` along the polyline; `approach` interpolated at its arclength on
   curved seams (`_approach_at`, `:474-482`); and the flags `seam_closed`, `seam_curved`. Note that the
   node passes `scene_id=str(save_dir)` (`icp_pose_refiner_node.py:1198`). weldgen derives a closed
   seam's tack *phase* from scene_id and seam id (weldgen/tacks.py:80-83), so the phase depends on the
   save-directory path.
7. Output helpers: `seams_points_m`, `tacks_points_m` (coloured by weld order), `summarize*`, `abutting_fitup`.

**Incidents.** (`:17-27`) "On the bench the standing plate of a T-joint came back 8-11 mm INSIDE the
base in one run and 1-8 mm above it (tilted) in the next - ICP cannot observe a thin plate sliding in
its own plane. The seam line itself (the intersection of the two face PLANES) is invariant to exactly
that slide". In the clip (`:183-190`): "the standing plate is tilted end to end (2 mm into the base at
one end, 11 mm above it at the other, measured) ... the strict clip keeps only the run where the plate
reaches the base plane (44 of 250 mm)".

**Users.** The ICP node (`~/welding_points`, writes `welding_seams.json`/`welding_tacks.json`),
`tack_marking_node`, `tack_reachability`, `contact_error`; `part_shapes` and `multiview_refine`
(`import_weldgen`, `posed_parts`).

**Tests.** `test_seam_from_registration.py` (13 tests): a T at exact poses gives two fillets on the
intersection line with unit approach axes; at registered poses (floating, tilted, driven in) both
fillets are kept, with fit-up; a wide tolerance does not pair the underside on thin sheet; a lap on
sheet thinner than the tolerance is undecidable; the bench assemblies give their T fillets; a missing
entry is an error naming the part; a tilted plate's seam runs as far as the member is within tolerance;
tacks follow the generator rule and are numbered along each seam. `test_curved_seams.test_tacks_on_a_closed_seam_carry_their_own_approach`.

**Thesis section.** Seams from the Registered Assembly (intro, Planar Seams, Fit-Up Gap and Weldability);
Tack Placement and Ordering; ch4 "Seams by the Accessibility Rule", "WeldSet and the Robotic Cell: One Geometry".

**FACT CHECK.**
- §mc_seams, "Seams are not detected in the point cloud: they are computed". **Confirmed for mode A**,
  but there is a **fallback**: with a part lacking a registry entry (or `weld_method=radius_pca`) the
  node falls back to the radius-PCA detector on the static cloud, unless `weld_fallback_pca=false`
  (`icp_pose_refiner_node.py:1131-1142`, parameter at `:478`). The thesis should mention or exclude it.
- §mc_seams, "the seam rule of WeldSet (§ws_d4) is applied to the primitives at their registered poses";
  ch4 §ws_link, "the seam rule ..., the curve constructors ... and the tack rule ... are applied
  **unchanged**". **Discrepancy for the seam rule.** The cell uses `judge_registered`: a pose tolerance
  instead of the contact tolerance, a clip with slack, the face-extent rule, a material probe both ways
  instead of the mutual-visibility probe, and penetration reported rather than rejected (`:229-239`).
  The tack rule *is* unchanged (`:414-415`). ch3 says this itself in Planar Seams ("uses a pose tolerance
  instead of a contact tolerance"), so ch4's "unchanged" must be restricted to the tack rule and the
  curve constructors.
- Eq. `eq:plane_intersection`, "clipped to where both faces exist". **Confirmed**, with the slack on the
  near-constant axis (`clip_registered`). Values: pose_tol 10 mm, minimum length 10 mm.
- §mc_tacks, "Tacks follow WeldSet's tack rule ... applied to the computed seams, open or closed: both
  ends tacked, the rest evenly spaced". **Imprecise for closed seams.** A closed seam has no ends:
  $n=\max(4,\lceil L/d_{\max}\rceil)$ rounded up to even, evenly spaced with a deterministic phase
  (weldgen/tacks.py:14-16, 115-124).
- "The rule also fixes the order ...: the ends first, then by bisection, alternating between seams".
  **Confirmed with precision:** same-*class* seams are interleaved round-robin (weldgen/tacks.py:174-184);
  on a loop, opposite pairs come in bisection order (`:166`). Note that the **pen** visits seam by seam,
  `tack_no` ascending (`marking.visit_order`), not in weld order.

#### E3. `curved_seams.py` (434 lines)

**Purpose.** The seams of curved parts at their registered poses, computed by intersecting the placed
primitives' surfaces (the WeldSet *verification* direction), each with per-point frames. It also settles
the spin of a cut pipe on what it stands on (`:1-29`).

**Algorithms.**
1. Dispatch `pair_seams` (`:419-434`): tube+slab → `_tube_on_slab`; tube+tube → `_tube_on_tube` (either
   way round); band+slab → `_band_on_slab`; any other pair → one non-weldable record "no seam rule for
   a X-Y pair yet" (reported, not dropped).
2. *Tube on plate* `_tube_on_slab` (`:168-204`). For each slab broad face (±w) and each tube end (base
   cut or top cap), the body must stand on the face's outer side with its axis at least 10° from the
   plane: $s\,(n\cdot z)\ge\sin10^{\circ}$ (`MIN_AXIS_TO_FACE_DEG`). The curve is the plane ∩ outer
   cylinder (weldgen `ellipse_from_plane_cylinder`: a circle, or an ellipse with semi-axes $r/|\cos\theta|$
   and $r$). It is sampled at 1 point/mm (closed, ≥ 16 points). The fit-up per point is the signed axial
   distance from the curve to that end, $g=s\,(z_{\mathrm{end}}(\phi,r)-z_{\mathrm{loc}})$ (`_end_gap`).
   The best (face, end) is the one with the smallest $\max|g|\le2\cdot$tol. Then:
   - rejected `fitup_beyond_pose_tol (min..max mm, pose_tol)` if $\max|g|>$ tol (`_near_miss`);
   - rejected `off_plate_edge` if any point lies off the slab rectangle (+tol);
   - the **bore** (inner cylinder ∩ plane) is kept as a "bore" record with cavity $2r_i$, a negative
     unless the bore is ≥ 200 mm (the cell's tool).
3. *Branch on main pipe* `_tube_on_tube` (`:209-250`): branch $r<$ main $r$. For each end, the saddle is
   weldgen's `saddle_from_cylinders` (it raises if grazing or missing). The fit-up is along the branch
   axis; the points must lie on the main pipe's length ± tol. The bore saddle is a negative.
4. *Band on plate* `_band_on_slab` (`:268-312`): each wall is the spine offset at $o_{lo}$ or $o_{hi}$
   (`OffsetCurve`), traced along the band's extrusion axis $z_w$ onto the plate plane:
   $\lambda=-(p_0\cdot n+d)/(n\cdot z_w)$, $x=p_0+\lambda z_w$. This is exact at any tilt, since the wall
   is a ruled surface along that axis. The fit-up is $s\,(z_{\mathrm{cap}}-\lambda)$. For a closed band the
   exterior wall is the weld and the interior a bore (cavity = its minimum width).
5. *Record* `_record` (`:131-163`): per-point frames $n_a$ (base face normal), $n_b$ (member wall normal,
   radial for a pipe), and $\mathrm{approach}=\mathrm{normalize}(n_a+n_b)$ (the face normal where they
   cancel). The dihedral is $180^{\circ}-\angle(n_a,n_b)$ per point, reported as the median and range.
   The verdict is weldgen's `seam_verdict`: the D4 torch cone at every point, weldable if ≥ 95 % clear;
   a bore is a negative unless the tool fits (`_verdict`, `:119-128`).
6. *Cut-spin settling* `settle_cut_spins` (`:372-404`, with `_cut_gap_vs_spin` `:330-369`). For each tube
   with a cut base and each candidate base part (a slab face standing on it, or a larger pipe for a
   saddle), take the seam curve $x_j$ (independent of the spin) and its local angles $\phi_j$ and heights
   $z_j$. For $\psi\in[-180^{\circ},180^{\circ})$ in 1° steps, compute
   $\mathrm{range}(\psi)=\max_j g_j-\min_j g_j$ with $g_j=h_{\mathrm{cut}}(\phi_j-\psi,r_o)-z_j$, then
   refine at ±1° in 0.05° steps. The end must be *on* the base: $\max|g|\le2\cdot$tol. The settled spin
   is accepted only if range_after ≤ max(1.5 mm, ¼ range_before) **and** range_before − range_after ≥ 1 mm.
   It is applied as $T\leftarrow T\,R_z(\psi)$. The node saves the settled pose (assembly.json and the
   SEPC) so reachability and marking use it too.

**Parameters.** CELL_BORE_MIN_DIAMETER_MM 200 ("weldgen's 80 mm is a welding torch's; the cell's tool
is wider"); MIN_AXIS_TO_FACE_DEG 10; NEAR_MISS 2.0; SETTLE_FIT_MM 1.5; SETTLE_MIN_GAIN_MM 1.0; density
1 point/mm (node `weld_seam_density_per_mm`).

**Incidents.**
- 2026-10-09 (`:47-51`): "S3 sat on R2 with its axis registered 7 mm aside, -5.3..10.5 mm against 10,
  and nothing at all was shown", hence the near-miss reporting.
- 2026-10-15 (`:376-379`): "E2 36 deg off, the mitre 15 deg off the plate, a 26 mm gap range, and its
  seam refused", hence the settling. (Test docstring: the gap ran −14.8..+11.5 mm.)

**Users.** `seam_from_registration.compute_seams`; the ICP node (`_settle_cut_spins` before
compute_seams, `icp_pose_refiner_node.py:1178`, `:1212-1240`).

**Tests.** `test_curved_seams.py` (12 tests): a pipe on a plate gives one closed circle of $2\pi r$ with
the gap as fit-up, the approach at 45° all round, and the bore as a negative; standing on its top cap;
a mitred pipe on its cut gives the ellipse with the dihedral sweeping 65-115°; off the edge refused;
floating just beyond tolerance reported, far beyond it no seam; the branch saddle; the printed S3 on R2;
the rounded-rect outer outline as the weld; an open band's two fillets with frames matched to points;
the printed SP3; tacks on a closed seam with their own approach; a pair without a rule reported; a
mitred pipe with the wrong spin settled on its cut.

**Thesis section.** Seams from the Registered Assembly / Curved Seams; Perception and Registration /
Symmetric and Cut Parts (the settling).

**FACT CHECK.**
- "A tube standing on a plate meets it along a plane-cylinder intersection, an ellipse". **Confirmed**
  (a circle when perpendicular).
- Eq. `eq:ellipse_seam`. **Mathematically correct** (solve $n^{\top}x=n^{\top}p_0$ for h). The code uses
  weldgen's equivalent centre/semi-axes form (`/workspaces/welding_cell_ws/weld_generator/weldgen/curves.py:512-535`).
  Either form may be used. Its validity condition is $n^{\top}a\neq0$, with the cell's 10° rule.
- "A branch tube on a main tube meets it along a saddle". **Confirmed** (branch radius < main radius;
  grazing branches are refused by weldgen).
- "a curved band on a plate meets it along the offsets of its spine". **Confirmed, more precisely:**
  the two offset curves traced along the band's extrusion axis onto the plate plane.
- TODO "a member must not lie in the face's plane (minimum angle)": **10°**. TODO "the frame per point
  (normals of both members, approach)": $n_a$, $n_b$, approach = normalize$(n_a+n_b)$. It rotates on a
  pipe because $n_b$ is radial.
- §per_symmetry, Eq. `eq:spin_settle`, $\psi^{*}=\arg\min(\max g-\min g)$. **Confirmed.** Add: the grid
  (1°, then 0.05°), the acceptance (≤ max(1.5 mm, ¼ before) and ≥ 1 mm gain), $g$ as the *axial* gap at
  the outer radius, and that it also applies to **saddle-cut** pipes on a pipe, not only mitres.
- §mc_fitup, "a seam whose gap exceeds what a welder could bridge is reported, not silently dropped".
  **Discrepancy.** What is reported is a gap beyond the *pose tolerance*: curved pairs within 2×tol are
  kept as `fitup_beyond_pose_tol`, and every seam carries `fitup_mm`. The *ISO 5817* comparison is not
  made on seams at all; it is a warning in the multi-view refinement (D3 step 11). The TODO "Near misses
  (up to twice the tolerance) reported with the reason" refers to this pose-tolerance rule (NEAR_MISS = 2).

### Group F: motion planning and execution

#### F1. `collision.py` (483 lines)

**Purpose.** The collision model for every arm motion: the arm as capsules on its joint frames, the
tool as its envelope primitives, and the scene as boxes (the registered parts and the fixtures) plus the
touched, tilted table plane. Everything is closed-form geometry on primitives, so it can sit inside a
sampling planner's validity test (`:1-29`).

**Model.**
1. *Arm* `ur5e_capsules` (`:348-364`): six capsules on the active-kinematics frames.
   - `base` (0,0,0)→(0,0,0.1625), r 75 mm;
   - `shoulder` from the shoulder origin to lift + 0.138·z_lift, r 70 mm;
   - `upper_arm` lift→elbow, displaced by 0.138 m along the lift axis, r 60 mm;
   - `forearm` elbow→wrist_1, displaced by 0.007 m along the elbow axis, r 50 mm;
   - `wrist_1` and `wrist_2` (wrist_1→wrist_2→wrist_3), r 50 mm.

   The radii are the ~90/75 mm tubes plus margin (`:40-44`). There is no wrist_3 capsule; the tool
   primitives cover the flange.
2. *Tool*: `tool_model` primitives placed by FK at tool0 (`bodies`, `:400-407`).
3. *Parts* `boxes_from_parts` (`:150-173`), in metres:
   - a slab is one box `part_<id>`;
   - a flat-based **tube** is 6 full-diameter boxes rotated by $j\pi/6$ about its axis, each with
     half-width $r\sin(\pi/12)$. The union covers the disk and stands at most
     $r(\sqrt{1+\sin^2(\pi/12)}-1)$ = 3.3 % of r proud (1.0 mm on C1), where one box stood 41 % proud
     (`:58-62`, `:76-90`);
   - a **cut** tube is $n=\mathrm{clip}(\lceil2\pi\max|dh/d\phi|/4\,\mathrm{mm}\rceil,12,36)$ radial half-box
     sectors, each from its sector's lowest cut point (`:68-73`, `:91-103`). "one box from the cut's
     lowest point would hang 39 mm below a 25 deg mitre, under the plate";
   - a **band** is a chain of boxes along the spine, each piece grown while its cross-width stays within
     wall + 1 mm sag (`:63-65`, `:106-147`). A rounded-rect tube's interior stays free; a tube's boxes
     are solid (its bore counts as an obstacle in planning).
4. *Fixtures* `fixture_boxes` (`:303-344`): from ≥ 2 pen-tip points on a fixture's top outline, the
   minimum-area rectangle in plan (1° search over [0°, 90°)), from the table plane under its centre up to
   the highest point. Alternatively an explicit centre/half/yaw.
5. *Table* `load_table_plane` (`:289-300`): $z=ax+by+c$ → $n=(-a,-b,1)/k$, $d=-c/k$. Height above the
   plane is the lowest point of a primitive, $\min(n\cdot p)+d$ (capsules minus r, `:280-286`). `base`
   and `shoulder` are exempt (they stand on it).
6. *Pairs* `pair_distances` (`:409-424`): (arm + tool) × scene boxes; (arm + tool, not exempt) × table
   ($\max(0,h)$); tool × {base, shoulder, upper_arm} (`self_pairs`). Stated non-goal: full self-collision.
   The branch lock keeps the arm elbow-up (`:24-28`).
7. *Distances*:
   - capsule-capsule: closed-form segment-segment (Ericson 5.1.9) minus the radii (`:177-199`);
   - capsule-box: golden-section search on the segment parameter, exact to $10^{-4}$, because distance
     to a convex set is convex along a line (`:208-223`);
   - box-box: separating-axis test only, giving 0 (touching) or +∞ (clear) (`:226-253`, `:271`);
   - primitive-plane: the lowest point.
8. *Queries* (`:426-437`): `min_distance` (with the pair); `in_collision` = min < clearance; `is_valid`
   = joint limits ∧ ¬in_collision.
9. `export_spec` (`:442-476`): everything the C++ port needs (the active kinematics table, radii,
   offsets, tool, scene, table, pairs, clearance, limits).

**Parameters.** Dataclass default clearance 0.01 m. **In use:** tack clearance 3 mm (`marking.json
clearance_m`); free moves 20 mm (`transit_clearance_m`, via `marking.transit_model`); multi-view
feasibility 20 mm.

**Users.** `marking`, `multiview`, `multiview_service`, `part_shapes`, `tack_reach`; scripts
camera_aim_node, tack_marking_node, tack_reach_marker_node, tack_reachability.

**Incidents.** One box round a pipe "refused 45 deg tacks there" (`:156-157`). Before 2026-10-14 there
was no table plane in the model (test docstring and marking.json).

**Tests.** `test_collision.py`: known distances; SAT; arm capsules on the chain; a pen 10 mm above a
plate is clear and one 5 mm into it collides, naming the pen and the part; anything below the table
collides; RRT with the model plans around a standing plate; mode-A boxes in metres; the tilted touched
table; fixture points become a box on the table. `test_curved_reach.py` (pipe boxes ~1 mm proud, mitre
sectors hug the cut, band chain within the sag, interior free). `test_transit_cpp.py` (C++ equivalence).

**Thesis section.** Motion Planning and Execution / Collision Model.

**FACT CHECK (§mc_collision).**
- "The arm is a chain of capsules on its joint frames". **Confirmed:** six capsules, with the radii and
  offsets above.
- "the pen tool is its CAD envelope (capsules for the pen and holder, boxes for the camera body)".
  **Incomplete:** the boxes are the flange adapter, the camera arm and the camera body.
- "the parts are their primitives as boxes (a tube as rotated boxes round its axis, a cut tube as sector
  boxes, a band as a chain of boxes)". **Confirmed:** 6 boxes ≤ 3.3 % r proud; 12-36 sectors; 1 mm sag.
- "the table is the touched plane, tilted". **Confirmed:** 1.70° (2026-10-02 touch-off).
- "fixtures are boxes entered by pen-tip points". **Confirmed:** the minimum-area rectangle from the
  table up. Currently `fixtures: []` in marking.json.
- "Distances are exact between capsules and between a capsule and a box; a box against a box is an
  intersection test only". **Confirmed.** Capsule-box is exact to a $10^{-4}$ parameter tolerance. Box-box
  pairs therefore get **no clearance**; the docstring line saying the scene box is inflated by the
  clearance (`:20-22`) is not what the code does (`_boxes_intersect` is called with no inflation, `:271`;
  same in C++, `transit_cpp.cpp:212`).
- TODO "which contacts are allowed (the pen tip with the part at the tack)". See F3 `pen_contact_pair`:
  pen-part, and pen-table under a thin plate, but never the pen *at* the table.
- TODO "the C++ port of the model, proven equal to the Python one". See F2.

#### F2. `src/transit_cpp.cpp` (607 lines): C++ collision model + OMPL planners (pybind11 `_transit_cpp`)

**Purpose.** A faithful C++ port of `collision.py` (the same primitives, distance algorithms, pairs and
pair order, so `is_valid` agrees), and OMPL's official AnytimePathShortening (ROS's libompl 1.7) plus
RRT-Connect for the arm's free moves. The Python model stays the single source of truth via
`export_spec` (`:1-26`).

**Algorithms.**
1. `Model` (`:294-422`): the parametrised FK (`frames`), arm capsules, tool placement, and
   `min_distance`. It reproduces Python's `min(pairs)` tie rule ("the FIRST pair with the smallest
   distance wins"). `is_valid` = limits ∧ ¬(min < clearance). `is_valid_many` releases the GIL.
2. `plan` (`:456-552`):
   - state space $y_j=w_jq_j$ (`RealVectorStateSpace(6)`, bounds = limits × w). OMPL's Euclidean path
     length is then $c=\sum_k\|W(q_{k+1}-q_k)\|_2$ with $W=\mathrm{diag}(2,2,1.5,1,1,0.5)$ (default
     weights, `:597`);
   - validity-checking resolution = resolution·$\min_jw_j$/maximum extent, so "a scaled step ... moves
     no joint more than `resolution` rad" (`:479-480`);
   - objective `PathLengthOptimizationObjective` with cost threshold 0 ("never good enough: use the
     whole budget", `:487-489`);
   - `aps`: AnytimePathShortening with `num_planners` RRT-Connect instances, hybridisation on, shortcut
     on, max hybridisation paths `max_paths` (`:492-503`). `rrt_connect`: a single RRTConnect;
   - termination (`:521-525`): (t ≥ cap) ∨ (t ≥ budget ∧ an exact solution exists), with
     cap = max(budget, max_budget);
   - the GIL is released while solving (planner threads in parallel). It returns solved, status,
     path (q = y/w), cost, time, validity calls, and the solution count.
3. Implementation note (`:589-592`): `py::array_t<bool>` filled through `mutable_unchecked` "returned
   every element as the LAST row's verdict", so the result is copied from bytes.

**Parameters.** Binding defaults: planner "aps", budget 1.0 s, weights (2,2,1.5,1,1,0.5), resolution
0.01, range 0 (the OMPL default), num_planners 4, max_paths 8, max_budget 0. **In use**
(`tack_marking_node.py:104-117`): planner aps, budget **3.0 s** ("1.0 before 2026-10-14"), max_budget
10 s, 4 planners, 8 paths, clearance margin 2 mm. Built with -O2 (`CMakeLists.txt:75-84`).

**Users.** `marking.fast_is_valid`, `marking.aps_path`.

**Tests.** `test_transit_cpp.py` (skipped without the build):
- FK equals Python (nominal and calibrated);
- `is_valid` is IDENTICAL and min distance and pair are equal on random and near-contact configurations,
  on the T scene, on a real session with the table, and with the tilted table plus curved parts;
- APS starts beside a part with the margin capped;
- C++ is > 20× faster (asserted);
- APS finds a valid short transit within budget, with a cost ≤ the median single RRT-Connect;
- an unknown planner is refused;
- a full marking plan with APS (transits only "straight", "Cartesian" or "APS");
- the fallback without the extension.

**Thesis section.** Transit Planning; Collision Model (the port); Performance-Critical Components.

**FACT CHECK (§mc_transit).**
- "OMPL's AnytimePathShortening (parallel RRT-Connect, hybridisation and shortcutting within a time
  budget), in scaled joint coordinates $y_j=w_jq_j$ ... $W=\mathrm{diag}(2,2,1.5,1,1,0.5)$". **Confirmed.**
  Add: 4 threads, budget 3 s in use, extension to the first exact solution up to 10 s, and planning at
  clearance + margin (see F4).
- §mc_impl "the collision model and the planners (one plan from minutes to seconds)". **Supported:**
  "2644 checks at 78 ms against E2's 34 boxes, 3 of a plan's 3.5 minutes (2026-10-14)"; C++ is "~590x
  the speed" (`marking.py:100-105`); the test asserts > 20×.

#### F3. `tack_reach.py` (602 lines): reachability before anything moves

**Purpose.** For every tack of `welding_tacks.json`: can the pen tip get there on the elbow-up branch,
with the approach point out along the pen and the straight descent between them clear, and at what
(work-angle tilt, roll)? It chooses one orientation per straight seam (so the wrist does not twist
between neighbours) and one per tack on a curved seam, chosen jointly by DP (`:1-40`). It also holds
`MarkingConfig` and the IK-on-branch and cable-window helpers used everywhere.

**Algorithms.**
1. `MarkingConfig` / `load_marking_config` (`:61-148`): home_q (the scan home, 2026-09-24),
   branch_joints (1, 2, 4) = lift, elbow, wrist_2; and the parameters below. `table()` uses `table_z_m`
   if set, else the touched plane file.
2. `branch_signature` (`:157-158`) $=\mathrm{sign}(q_j)$ for j ∈ (1, 2, 4).
   `solve_on_branch` (`:166-183`): DLS IK (`kinematics.ik_solve`) from the given seeds plus 6 random
   seeds $q_{\mathrm{home}}+\mathcal N(0,0.4^2)$. Each solution is mapped by `to_cable_window`, and the
   **first one on the locked branch** wins; others are discarded, never used.
3. `to_cable_window` (`:198-209`): every joint is the 2π-equivalent nearest `ref` (`_unwrap_to`), except
   wrist_3, which is the equivalent nearest `ref` among those within ±`wrist3_window_deg` (180°) of home
   (or the one nearest home if none). The tool pose is unchanged.
4. `pen_axis_for` (`:254-262`): the pen axis is $-$approach, rotated by the tilt about the seam tangent
   ($p_1-p_0$ of the tack segment).
5. `_try_tack` (`:312-378`), one (tack, tilt, roll):
   - (a) pose-only pre-checks with no IK (`tool_only_clearance`, `:265-283`): the tool at the tack pose
     (pen excluded) and at the approach pose (pen included) must keep ≥ clearance to the boxes and the
     table;
   - (b) IK of the approach pose (tip at $p+s\,\mathrm{approach}$, s = standoff 35 mm) on the branch;
     its clearance with the pen included must be ≥ clearance;
   - (c) IK of the tack pose seeded from the approach pose; its clearance excludes the pen's *allowed*
     contacts (`pen_contact_pair`, `:286-292`: pen vs `part_*`, and pen vs table) but counts the pen
     at or below the table as a collision (`pen_below_table`, `:295-297`). The pen-part gap is reported
     as `pen_gap_m` (~0);
   - (d) $\|q_{\mathrm{tack}}-q_{\mathrm{app}}\|_\infty\le1.2$ rad;
   - (e) LIN descent: IK at the 5 intermediate points $k/6$ (`lin_steps` = 6), each seeded from the
     previous with a jump ≤ 0.5 rad, and clearance ≥ the clearance.
6. *Straight seams* `plan_tacks` (`:480-571`). For every (tilt, roll) on the grid (5 tilts × 12 rolls
   with the 30° roll step in use), try all tacks in `tack_no` order, each seeded from the previous
   approach pose and home. Combinations are rejected if any tack fails or a single joint of consecutive
   approach poses moves more than 1.2 rad. The best is chosen by the key
   $(-\mathrm{round}(\min\text{clearance},0.1\,\mathrm{mm}),\ \text{score})$ with
   $\text{score}=\sum\|q_{\mathrm{app},k}-q_{\mathrm{app},k-1}\|_1+3\cdot\#\{|\sin q_5|<\sin0.35\}$
   (`_score`, `:381-391`). A seam with no admissible combination is reported per tack with the best
   attempt's reason, and with a remark when even the best orientation's clearance is too small ("a
   longer pen reach or a slimmer holder is what raises it").
7. *Curved seams* `_plan_per_tack` (`:420-477`), used when any tack has `seam_curved`. Every admissible
   (tilt, roll) of every tack is solved first. Then a Viterbi recursion over the reachable tacks in
   order, from $q_{\mathrm{from}}$ (home, or the previous seam's last approach):
   $C_k(j)=\min_i\big[C_{k-1}(i)+\|W\,\delta(q_{k-1,i},q_{k,j})\|_2\big]+\kappa\max(0,\delta_0-\mathrm{clr}_{k,j})$.
   Here $W=\mathrm{diag}(2,2,1.5,1,1,0.5)$ (`TRANSIT_WEIGHTS`), δ wraps each joint the short way except
   wrist_3 (a plain difference, since it stays windowed, `_joint_cost` `:405-412`), clr = min(approach,
   tack, LIN clearance), $\delta_0$ = 10 mm, κ = 0.15 rad/mm. Backtrack. Unreachable tacks are reported
   with their best attempt's reason and skipped by the chain. The inter-tack joint-step limit does not
   apply here.

**Key parameters.** Dataclass defaults → **in use** (config/marking.json):
standoff 0.035 m → 0.035; departure 0.10 m → 0.10; clearance 0.01 → **0.003 m**; transit clearance
0.02 → 0.02; roll step 15° → **30°**; lin_steps 6 → 6; max joint step 1.2 rad → 1.2; wrist margin
0.35 rad → 0.35; work angles (0, −10, 10, −20, 20)° → same; table_z_m None → null (the plane file);
wrist3 window 180° → 180°; fixtures () → []. Constants: PER_TACK_ENOUGH_CLEARANCE_M 0.010,
CLEARANCE_COST_RAD_PER_MM 0.15, TRANSIT_WEIGHTS (2,2,1.5,1,1,0.5), random seeds σ = 0.4 rad.

**Design decisions and incidents.**
- marking.json: why 3 mm. "the tool envelope already carries margin, and with the tip at 183.78 mm the
  holder body clears the plates of a 90 deg T by 3.1 mm at best (63.8 · sin 45 − 42), so 5 mm would
  refuse every square fillet". Work angles: "A 90 deg corner leaves the holder 7.5 mm from both plates
  at 0; on plates that lean, a tilt away from the closer plate is what makes the tack reachable".
- DP instead of tack-by-tack, 2026-10-14 (`:430-433`): "Choosing tack by tack for clearance alone ...
  put consecutive approach poses 65-90 deg apart on three joints round E2, and the transits between
  them swung the pen up to 0.9 m off". "1 mm of clearance is worth ~9 deg of base-joint motion between
  two tacks" (`:397-399`).
- Cable window, 2026-10-16 SP3 (`:83-88`): "continuity from tack to tack wound wrist_3 to -202 and -287
  deg, and the cable's pull read as a 1.5 N 'contact' 84 mm above the part, twice; every good contact
  on E2 the day before was within ±151 deg".
- Thin plate, 2026-10-15 (`:287-291`): "a 5.5 mm plate on the table put the touching pen 2.5 mm from
  the table, under the 3 mm clearance - and it cannot reach the table without going through the plate".
- `roll_list_deg`: the attribution experiment. "the same tack at rolls 90 deg apart moves a TOOL error
  with the wrist and leaves a WORLD error (camera, registration) where it is" (`:89-92`). It is not read
  from marking.json (`:130-148`); `scripts/tack_reachability.py --roll` sets a single roll (`:75-76`),
  and `--tilt` sets a single work angle.

**Users.** marking, motion (UR_ORDER, joint reorder), camera_aim (branch_signature), multiview,
multiview_service; scripts tack_reachability (writes tack_reach.json), tack_marking_node,
camera_aim_node, pen_tip_touchoff, table_touchoff, tare_distance, tcp_offset_probe.

**Tests.** `test_tack_reach.py`: /joint_states reordered and the home branch signature; a T in front is
reachable with one roll per seam, all on the branch; a joint far behind is reported unreachable; other-
branch solutions are rejected; wrist_3 stays in its cable window (the 2026-10-16 case); the roll choice
counts wrist_3 the way a transit turns it. `test_curved_reach.py`: a closed seam round a pipe is reached
with a roll per tack while one roll fails at the second tack (> 69° of wrist); the rolls of a loop are
chosen together; a pipe on a thin plate is reachable.

**Thesis section.** Motion Planning and Execution / Reachability.

**FACT CHECK (§mc_reach).**
- "IK is solved on one branch, the elbow-up configuration of the home pose (the signs of the lift, elbow
  and second wrist joints are locked)". **Confirmed** (`:18-21`, `:64`, `:157-183`).
- "On a straight seam one pen roll and tilt is chosen per seam, maximising the smallest clearance over
  the approach pose, the tack pose and the straight descent between them". **Confirmed.** Add the
  tie-break (least joint travel + a wrist-singularity penalty), the 1.2 rad inter-tack limit, and the
  grid (5 tilts × 12 rolls).
- "On a curved seam ... a roll is chosen per tack, jointly over the seam by dynamic programming (a
  Viterbi recursion)". **Confirmed, with precision:** a **(tilt, roll)** pair per tack. $q_k$ in
  Eq. `eq:roll_dp` is the *approach* pose; the sum starts from home or the previous seam's last approach.
- Eq. `eq:roll_dp`, $\delta_0$ = 10 mm, κ = 0.15 rad/mm. **Confirmed** (`:396-399`). W is the transit
  weight matrix (2,2,1.5,1,1,0.5), with joints wrapped the short way except wrist_3.
- "Every pose is taken as the 2π-equivalent that keeps the last wrist joint within ±180° of home ...
  plain joint continuity wound it far enough for its pull to read as a contact". **Confirmed**
  (`:83-88`, `:198-209`). The numbers: −202° and −287°, a 1.5 N false contact 84 mm above the part, E2
  contacts within ±151°.

#### F4. `marking.py` (794 lines): the marking motion, computed

**Purpose.** Everything the marking node needs that is not ROS: the visit order, the transit path
(straight edge, Cartesian line, APS, RRT fallbacks), the descent as an IK chain with its clearance check,
the departure chain, timing (trapezoid, speed cap, descent speeds), strokes, contact bookkeeping, and the
full per-tack plan (`:1-29`).

**Algorithms.**
1. `transit_model` (`:65-75`): clearance = max(tack clearance, min(20 mm, 0.9·the end poses' own
   clearances)). A free move keeps real room but stays plannable from an approach pose beside a plate.
2. `fast_is_valid` (`:100-111`): the C++ `Model.is_valid` if built, else Python.
3. `cartesian_transit` (`:114-180`): the tool moves along a straight, then gently bowed, Cartesian path
   between the two tool0 poses, with the orientation slerped. Let $b=-(z_a+z_b)/\|z_a+z_b\|$ (minus the
   two pen axes, "away from the joint"). For each $\beta\in\{0,30,60,100\}$ mm the control point is
   $x_c=\tfrac12(x_a+x_b)+2\beta b$, and
   $x(s)=(1-s)^2x_a+2s(1-s)x_c+s^2x_b$, so the **midpoint** is displaced by exactly β. There are
   $n=\max(2,\lceil\ell/5\,\mathrm{mm}\rceil,\lceil\Delta\theta/3^{\circ}\rceil)$ poses. At each, IK on
   the branch is seeded from the previous; a joint jump > 0.3 rad (singularity) fails. The end is
   joined at the 2π-equivalent, and every edge is checked at 0.01 rad. The first clear β wins.
4. `aps_path` (`:183-233`): planning at clearance + margin, with
   margin = clip(min(2 mm, ½·(min end-pose clearance − clearance)), 0). Calls `_transit_cpp.plan`, then
   **re-checks every edge** at the nominal clearance with Python's edge sampling (`_edge_valid`; the
   validity function is the C++ one if built, `:228`).
5. `transit_path` (`:236-314`), the cascade:
   - (1) the straight joint edge (after unwrapping);
   - (2) the Cartesian line or bow (only when `cartesian_cfg` is given: tack-to-tack, not home);
   - (3) APS;
   - (4) C++ RRT-Connect + collision-aware shortcut;
   - (5) Python RRT-Connect (3 attempts, step 0.15, 8000 iterations, edge resolution 0.01) + shortcut.

   With `smooth`: a centripetal Catmull-Rom spline (`smooth_transit`, `:78-97`), every edge re-checked;
   it is kept only if clear. `info['how']` records which stage answered.
6. `shortcut_path` (`:48-62`): 100 random shortcuts under the *planner's* validity test.
7. Timing:
   - `time_trapezoid` (`:326-358`): the arc length is $s=\sum\|\Delta q\|_\infty$ on a path densified to
     0.02 rad; trapezoid/triangle with $v_{\max}$ = 0.3 rad/s and $a_{\max}$ = 0.5 rad/s²; the TOTG fallback;
   - `time_joint_path` (`:361-371`): per segment $\max(\|\Delta q\|_\infty/v,0.3\,\mathrm s)$, a speed cap
     used by the multi-view;
   - `time_descent` (`:396-429`): constant *tip* speed per sample, $v_{\mathrm{fast}}$ while more than
     `fast_beyond_m` (the standoff) before the point, $v_{\mathrm{slow}}$ within `slow_zone_m`, $v_{\mathrm{tip}}$
     between; each sample ≥ 20 ms.
8. `descent_chain` (`:374-393`): tips $p-(s_0-k\,L/n)\,a$ in 2 mm steps from the approach point (standoff
   $s_0$) to the point + overshoot. IK on the branch is seeded from the previous; a jump > 0.3 rad means None.
9. `descent_check` (`:604-644`), for each pose of the chain with past = $(x_{\mathrm{tip}}-p)\cdot a$:
   - if the pen is at or below the table: when past > 0.1 mm (and i > 1) the chain **ends there** ("the
     overshoot ends at the table"), otherwise refuse;
   - before the point, the clearance of all pairs except the pen's allowed contacts must be ≥ the tack
     clearance;
   - in the overshoot, the bound is $d_{\mathrm{ref}}-(\mathrm{past}-\mathrm{past}_{\mathrm{ref}})$, i.e. the
     clearance at the tack minus the distance travelled past it ("moving s along the pen changes any
     distance by at most s").
10. `departure_chain` (`:647-667`): IK out along the pen from the approach point to `departure_m`
    (100 mm) in 5 mm steps; every pose must be valid with the whole tool, pen included.
11. `build_marking_plan` (`:670-786`), per tack in `visit_order` (seam by seam, `tack_no` ascending,
    `:531-534`):
    - (a) if stroking, aim the descent at the stroke's first point;
    - (b) try roll offsets 0, ±s, ±2s, … up to ±90° (s = roll step: 0, ±30, ±60, ±90° in use,
      `_roll_offsets` `:593-601`); per roll, the approach pose (the report's own at offset 0 for a dot)
      and `descent_check`; the first clear one wins;
    - (c) the departure chain; if clear, the transit ends at the departure point and the descent runs
      departure → approach → point, else the transit goes to the approach point;
    - (d) the target is `to_cable_window(q_app, q_prev)`; `transit_path` with `transit_model`,
      `unwrap=False`, the Cartesian stage on, `smooth` and APS options from the node;
    - (e) shift the descent by the transit's whole-turn offset and check the joint limits;
    - (f) finally a path home (`unwrap=False`, no Cartesian stage).
12. `stroke_targets` / `stroke_modes` / `seam_section` / `stroke_chain` (`:461-468`, `:537-590`):
    - `tack` mode draws the tack segment $p_0\to p_1$; on a curved seam it draws the seam's own piece
      centred at the tack's arclength (a chord p0→p1 "misses a 31 mm pipe by 0.6 mm at a 12 mm tack");
    - `seam` mode draws the whole polyline once per seam; on a curved seam it becomes per-tack sections
      ("one pen axis cannot draw a loop round a pipe");
    - the stroke is pressed by `depth_m` along the pen.
13. `contact_depth_m` (`:471-473`): $\delta_c=(p-x_{\mathrm{tip,contact}})\cdot a$, + when the surface is
    met before the registered point.

**Parameters.** Library defaults: edge resolution 0.01; overshoot 3 mm (the node uses **5 mm**); bulges
(0, 30, 60, 100) mm; Cartesian steps 5 mm / 3°; max joint step 0.3 rad; APS margin 2 mm, budget 2 s
(the node: 3 s), max 10 s; RRT 3 attempts. Node (`tack_marking_node.py:104-137`): v_joint 0.3 rad/s,
a_joint 0.5 rad/s², transit_planner aps, smooth_transits **True**, v_approach 50 mm/s, v_tip 20 mm/s,
v_slow 2 mm/s, slow_zone 6 mm, overshoot 5 mm, dwell 0.5 s, stroke_mode dot, v_stroke 20 mm/s,
press 0 mm, stroke_chunk 10 mm, hold_force = the touch force, depth_gain 2e-5 m/N, max_depth_step
0.2 mm, min_contact_force 0.4 N.

**Design decisions and incidents (dated).**
- 2026-09-29, transit clearance (`:66-70`): "with the 3 mm tack clearance a transit to the far side of a
  standing plate skimmed it and, the parts sitting ~10 mm off their model, hit it at 9 N".
- 2026-10-14, Cartesian line (`:124-131`): "APS's joint-space paths between the same two poses varied
  from 0.26 to 1.6 m of tip travel (2026-10-14, E2); this one is 0.14-0.21 m and the same every time.
  The straight chord between two departure points a quarter-turn apart round a pipe passes nearer the
  pipe than either end", hence the bow.
- 2026-10-14, departure points (`:736-739` and marking.json): "from the approach point (35 mm) the
  planner first had to squeeze out from beside the plate, and its paths round a pipe swung up to 0.9 m
  off". marking.json: APS tip travel 0.5-1.9 m before, then from 100 mm "0.19-0.21 m every time, at 3 s".
- APS margin: 2026-10-06, "failed the Python collision re-check" (a path along the clearance boundary);
  capped on 2026-10-14 because "a full 2 mm made that pose itself invalid and APS refused to start
  ('Invalid start' / 'Invalid goal' on most transits ...), falling back to the slow Python RRT-Connect"
  (`:194-201`).
- C++ checks, 2026-10-14 (`:100-105`, `:282-283`): "2644 checks at 78 ms against E2's 34 boxes, 3 of a
  plan's 3.5 minutes", and the Python RRT "took minutes per transit round E2's 33 boxes". The two
  comments disagree on 33 vs 34 boxes.
- Before 2026-10-05 (`:248-250`): "the marking node drove the straight segments of the first RRT-Connect
  path, corners and all - drastic swings between tacks on opposite sides of a plate".
- 2026-09-29, homing (`:253-255`): "wrist_3 came home at 5.42 rad instead of -0.86, a turn closer to its
  ±2π limit", hence `unwrap=False` home.
- Descent speeds (`:406-409`): "the pen is not sprung, and on a rigid part the force past the 1.5 N
  threshold grows as speed x stopping time x stiffness (the table read ~10 N at 10 mm/s)".
- Roll retry, 2026-10-06 (`:700-704`): at a stroke start 14.5 mm from the centre, or in the overshoot,
  the report's roll can fail ("the camera body entered the base plate 4 mm past the point").
- Table end of overshoot, 2026-10-16 (`:617-622`): "on SP3 the root tack sat 4.9 mm over the table
  (5.5 mm plate), and 5 mm of overshoot along a pen 35 deg off vertical reached it".
- Cable window, 2026-10-16 (`:751-752`): "plain continuity wound it to -287 deg over SP3's tacks".

**Users.** `scripts/tack_marking_node.py`; `multiview` (transit_model, transit_path); `motion`
(time_trapezoid); `camera_aim_node` (fast_is_valid); `table_touchoff`.

**Tests.** `test_marking.py`: the visit order; the contact-depth sign; joint-path timing under the cap;
the descent chain along the axis; shortcuts never invalid; the full T plan; stroke targets and chains;
the two-speed descent slowing in the last millimetres. `test_transit_smoothing.py`: the spline rounds
corners and keeps the ends, is refused when it leaves free space; the trapezoid and triangle; a clear
smoothed transit on the T; a full plan with smoothing; roll offsets with the report's roll first; a
descent failing at the report's roll tries the next. `test_curved_reach.py`: marking round a pipe
departs along the pen and slides tack to tack; fast then slow descent; a curved stroke follows the seam;
the overshoot ends at the table. `test_transit_cpp.py`: APS integration.

**Thesis section.** Motion Planning and Execution / Transit Planning; Force-Gated Descent and Marking.

**FACT CHECK.**
- "Transits start and end at departure points, 100 mm out along each tack's pen axis, where the tool is
  in open space". **Confirmed** (`departure_m` 0.10). If the departure chain is blocked, the transit goes
  to the approach point (35 mm) instead (`:742-748`).
- "A transit is tried, in order, as: the straight joint-space edge; the tool's straight Cartesian line
  (orientation interpolated by slerp), then the same line bowed outward as a quadratic Bézier". **Confirmed.**
- Eq. `eq:cartesian_bow`, control point $x_m+\beta b$. **Discrepancy.** The code's control point is
  $x_m+2\beta b$, so the *curve's midpoint* is bowed by β (`:146`; docstring `:129-131` "bowed outward
  ... by each of bulge_m at its middle"). With the draft's form the midpoint moves only β/2. Correct the
  equation to $(1-s)^2x_0+2s(1-s)(x_m+2\beta b)+s^2x_1$, or define β as the midpoint offset and say so.
  β ∈ {0, 30, 60, 100} mm is **confirmed** (`:118`). b is $-(a_0+a_1)/\|a_0+a_1\|$.
- "and only then a sampling-based planner: OMPL's AnytimePathShortening". **Confirmed**, but incomplete:
  after APS comes the C++ RRT-Connect and then the Python RRT-Connect (`:275-314`). The home path skips
  the Cartesian stage; multi-view transits use straight edge or RRT only.
- "The path is smoothed, re-checked against the collision model and timed with TOTG". **Confirmed** with
  `smooth_transits=True` (node default). The spline is used only if clear; TOTG samples are
  collision-checked, with a trapezoid fallback (`motion.py:224-270`).
- TODO "the planning margin capped by the end poses' own room". **Confirmed** (`:212-216`).
- §mc_marking, "fast to the approach point (35 mm before the tack), slower to the tack, slowest in the
  last millimetres, and on past the registered point by an overshoot". **Confirmed:** 50 → 20 → 2 mm/s,
  slow zone 6 mm, overshoot 5 mm (node; the library default is 3 mm).
- "The overshoot ends at the table, since no surface lies beyond it". **Confirmed** (2026-10-16).
- "δ_c = (p_reg − p_tip)·a ... + before, − past". **Confirmed** (`:25-28`, `:471-473`).
- "The mark is a dot, or a stroke along the seam with the depth corrected from the force between chunks".
  **Confirmed** (node): chunks of 10 mm; the depth moves by $2\times10^{-5}$ m/N × (hold − measured
  force along the pen), clipped to ±0.2 mm; it stops after 2 chunks under 0.4 N. Strokes on curved
  seams are per-tack sections.

#### F5. `motion.py` (361 lines): TrajectoryExecutor

**Purpose.** Joint-trajectory execution with a wrench watchdog, shared by the marking node (prefix '')
and the ICP node's multi-view refinement (prefix 'mv_'). It handles /joint_states and the F/T wrench
(bias, trace), the FollowJointTrajectory client on `scaled_joint_trajectory_controller` (no Servo, no
controller switching), the safety gates, the dry run, and TOTG timing (`:1-23`).

**Algorithms.**
1. `measure_bias(window)` (`:179-189`): the mean of the raw force over 0.5 s, sampled every 5 ms;
   `force_mag` $=\|F-F_{\mathrm{bias}}\|$ (force only, no torque).
2. `time_path` (`:224-270`): TOTG through `/compute_totg` with the same $v_{\max}$ and $a_{\max}$ on every
   joint, path tolerance 0.005 rad and resample 0.02 s. It falls back to `time_trapezoid` if the
   interface is missing, the service is not ready in 0.5 s, the call times out after 5 s, it fails, the
   answer does not start or end within 1e-3 rad of the path or has non-increasing stamps, or **any
   sample fails `is_valid`**. It returns (timed, source).
3. `execute` (`:272-361`):
   - gate: the first point must be within 0.35 rad (∞-norm) of the current joints;
   - dry run: no goal; `where()` follows the pretended end; with `want_contact` the pretended contact
     sits at `sim_contact_fraction` along the path;
   - `rebias` re-measures the bias right before a move that starts at rest and touches nothing (transit,
     home), never one that starts in contact (retract, stroke);
   - send the goal; while it runs, poll every 2 ms: on abort or $\|F\|$ > abort (8 N; 40 N for moves
     *away* from a contact) cancel and fail; with `watch_touch` and $\|F\|$ > touch (1.5 N) and ≤ the
     abort limit, cancel and return the contact (the joints at the cancel, F).

     "no settling pause: ... the joints at the cancel are already the contact record".

**Parameters (`PARAMS`, `:73-87`).** dry_run True; action
`/scaled_joint_trajectory_controller/follow_joint_trajectory`; wrench topic
`/force_torque_sensor_broadcaster/wrench`; abort_force_n **8.0**; release_abort_force_n **40.0**;
bias_window_s 0.5; max_joint_jump_rad 0.35; totg_service `/compute_totg`; totg_path_tolerance 0.005;
totg_resample_dt 0.02. touch_force_n is set by the owner (marking: 1.5 N from pen_tool.json; mv: 1e9,
i.e. off).

**Incidents.** It was the execution half of tack_marking_node (2026-09-24 .. 09-29) and moved here on
2026-10-01 so the multi-view drives the arm with the same gates (`:16-18`). The sensor zero drifts,
"e.g. by 7 N after a power cycle (2026-10-01)", which alone could reach the 8 N abort, hence `rebias`
(`:282-285`). "on a rigid part a stopped pen presses at ~10 N", hence the 40 N limit for moves away
(`:335-337`). A rejected goal produces a long operator message (External Control not running, ...).

**Users.** `tack_marking_node.py`; `multiview_service.py`.

**Tests.** `test_motion.py` (needs rclpy): parameter prefix; existing parameters kept; the dry run moves
the pretended arm; a joint jump is refused; the dry-run contact at the given fraction; force bias and
trace; the abort flag; TOTG used when it answers (a fake service); fallback when blending leaves free
space; trapezoid without the service.

**Thesis section.** Force-Gated Descent and Marking (execution and safety); Transit Planning (timing).

**FACT CHECK.**
- Eq. `eq:touch`, $\|F-F_{\mathrm{bias}}\|>F_{\mathrm{touch}}$ = 1.5 N. **Confirmed**
  (`config/pen_tool.json touch_force_n`; `:340`).
- "with its bias measured at rest just before". **Confirmed:** the node measures it right after the
  transit, at the departure point (`tack_marking_node.py:385`).
- "The descent stops at the first sample where the force ... exceeds". **Confirmed** (2 ms polling of
  the latest wrench).
- TODO "the 8 N abort". **Confirmed** (`:78`); add the 40 N release limit.

#### F6. `src/totg_service_node.cpp` (203 lines) + `srv/ComputeTOTG.srv`

**Purpose.** Exposes MoveIt 2's time-optimal trajectory generation (Kunz & Stilman 2012) as the service
`compute_totg`, using the low-level Path/Trajectory API, so no robot model, URDF, SRDF or planning scene
is needed (`:1-11`).

**Algorithm.** Validate the sizes. Drop near-duplicate consecutive waypoints ($\|\Delta\|\le10^{-6}$);
if all are one point, return it. `Path::create(waypoints, path_tolerance)` blends corners within the
tolerance. `Trajectory::create(path, v_max, a_max, 0.001)` uses a 1 ms integration step. Reject a
non-finite or non-positive duration. Resample at `resample_dt`, the end included, returning positions,
velocities and timestamps.

**Parameters.** Request fields: `waypoints_flat`, `num_joints`, `max_velocity[]`, `max_acceleration[]`,
`path_tolerance` (≤ 0 → 0.02 rad), `resample_dt` (≤ 0 → 0.1 s). The srv comment says "Default ~0.1"
for path_tolerance, which disagrees with the node's 0.02. **In use:** 0.005 rad, 0.02 s, 0.3 rad/s,
0.5 rad/s² on every joint. It is launched by `pointcloud.launch.py` (`launch_totg` default true) and by
the legacy `admittance_control.launch.py`.

**Users.** `motion.TrajectoryExecutor.time_path`; the legacy drawing server. **Tests.** None for the
C++ node; `test_motion` uses a fake service.

**Thesis section.** Transit Planning (TOTG timing, `kunz2012totg`); Performance-Critical Components.
**FACT CHECK.** "timed with time-optimal trajectory generation [kunz2012totg]". **Confirmed**, with
collision-checked samples and a trapezoid fallback.

### Group G: legacy and packaging

#### G1. `src/jacobian_calculator_node.cpp` (381 lines): legacy

**Purpose.** The sand-drawer / Isaac Sim velocity controller (node name `sand_drawer_jacobian_node`):
end-effector twist → joint velocities by a damped pseudo-inverse. Damping is
$\lambda=0.08(1-\mu/\mu_{\mathrm{ref}})$ below $\mu_{\mathrm{ref}}$ = 0.05, with manipulability
$\mu=\sqrt{\det JJ^{\top}}$. It slows down below μ = 0.1, adds null-space manipulability ascent (a
finite-difference gradient) and a mid-range posture term, clamps to 0.5 rad/s, and publishes
`/isaac_joint_commands`. **Not launched by any cell launch file** and not part of the welding cell.
**Thesis:** do not describe it in Chapter 3 (at most as earlier work). **Tests:** none.

#### G2. `action/ExecuteDrawing.action`: legacy

The drawing server's action (RETRACT_UP → HOMING → RRT → DESCENT → DRAWING → ASCENT). It is built
because `rosidl_generate_interfaces` lists it (`CMakeLists.txt:24-28`). It is not used by the cell.

#### G3. `__init__.py`, `CMakeLists.txt`, `package.xml`

- `__init__.py` (`:1-12`): `pkgutil.extend_path`. The scripts put the **source** tree first on sys.path,
  but the generated `srv`/`action` and `_transit_cpp` exist only in the install space. Extending the
  package path lets the source modules win while the built parts are still found (motion.py's
  `_import_compute_totg` does the same, `motion.py:46-67`).
- `CMakeLists.txt`:
  - interfaces: ExecuteDrawing.action and ComputeTOTG.srv;
  - C++ executables: `admittance_control_jacobian_node` (legacy), `totg_service_node`, `aim_rate_relay`
    (-O2);
  - the pybind11 module `_transit_cpp` (OMPL, Eigen, -O2) installed next to the Python modules;
  - the Python modules are installed from an **explicit list** (`:86-126`), because
    `rosidl_generate_interfaces` already registers a Python package of the same name;
  - the script list (`:133-165`) and the shared directories.
- `package.xml`: dependencies ompl, pybind11-dev, python3-websocket ("fp_tracker_node.py: WebSocket
  client to the laptop's FoundationPose tracker"), moveit_core/moveit_ros_planning (TOTG; the Jacobian
  node), controller_manager_msgs (the aim's controller switch), depth_image_proc. The `<description>`
  is stale ("UR5e task-space plane extraction ... for Isaac Sim based admittance control workflows"),
  and so is the package name `admittance_control`.
- Incident (`test/test_install_list.py`): 2026-10-09, `curved_seams` was missing from the install list,
  so "mode A fell back to the radius-PCA detector on the whole curved-parts bench". The test now asserts
  that every module is listed. All 24 modules are currently listed.
- **Thesis section.** Implementation Details (ROS 2 Architecture; Performance-Critical Components).

---------------------------------------------------------------------------------------------

## 3. (a) What the draft is MISSING

Each item is tagged with the size it deserves, S (a sentence), P (a paragraph) or §
(a subsection/figure), and the section it belongs in. Values and sources are in the module entries.

### Coordinate Frames and Calibration
1. **P** - *Nominal vs. calibrated kinematics as an incident and a mechanism.* On 2026-09-29 the nominal
   chain was 2.4-4.2 mm / ~0.5° off at the tip, which put the marks ~4 mm off. The fix: the calibration
   in the URDF, the driver and `use_kinematics` in every tool (A1). The draft's TODO asks for this.
2. **P** - *TCP verification by touches on a known plane* (A3 `solve_tcp_error`). The model
   $(R_i^{\top}n)^{\top}e+\delta=-h_i$ includes a free plane offset δ. Vertical touches see only $e_z$;
   tilted touches make $e_x,e_y$ observable with sensitivity $\sin(\text{tilt})$. Conditioning at 30° is
   ~0.3, vs ~0.02 for vertical touches on a near-level table; use 45° for the pen length.
3. **S** - The 7-touch pattern (hexagon with 10 cm edges plus the centre) and *why more than three points*
   (a meaningful RMSE; repeats separate repeatability from flatness).
4. **P** - *Camera-vs-table check* (`surface_vs_plane`): the dominant surface by a 2 mm histogram within
   ±0.25 m, then a trimmed refit. Large errors are measured, not filtered. It underlies the depth/Tare
   and extrinsic checks.
5. **P** - *World-vs-camera tilt separation* $t_i=a+\mathrm{Rot}(\psi_i)b$ (yaw spread ≥ 90°), and that
   the rotation refinement corrects only the camera's two tilt angles, never its yaw about the optical
   axis (A3 steps 7-8).
6. **S** - The extrinsic file `T_tcp_to_cam.npy` is **tool0** → camera_color_optical_frame (the "tcp" in
   the name is the flange, not the pen TCP).
7. **P** - *Persistent extrinsic-translation self-calibration* (D5): a history keyed by the extrinsic
   file's sha1; combination by information; ≥ 3 runs, spread ≤ 1 mm, leave-one-out ≤ 1 mm; undetermined
   directions left untouched; a corrected file written next to the old one and promoted by hand.

### Operator Interaction
8. **P** - The aim's remaining safety and smoothing (the draft TODO lists some):
   - the excursion limits ±35° pan / ±25° tilt, looked ahead 0.35 s;
   - two 60 ms lags with a 60°/s² cap;
   - the deadband of 5 % of the image width, released at 40 % of it, the rate measured from half the
     inner radius;
   - the 15°/s cap; One-Euro $f_{\min}$ = 1 Hz, β = 5;
   - rays more than ~73° off-axis ignored ($d_z<0.3$);
   - the node's |F| > 15 N abort, stale joint states (0.2 s), the latch and re-arm, and the dry run;
   - the gain rationale: ~50° phase margin even with 0.25 s of uncompensated latency.
9. **S** - The relay's own safety: a 0.1 s timeout ramp at 2 rad/s², 4 rad/s² and 0.6 rad/s limits,
   malformed input dropped, and the 8 ms cost of one input period.

### Perception and Registration
10. **P** - *Tracking transport* (C2):
    - a latest-wins mailbox (no backlog);
    - ROI crops from the server's hint, where the principal point shifts but the camera frame does not;
    - `register_roi`, which makes registration fit an 8 GB GPU (2.6 GB of warps for ~250 hypotheses at
      1280×720);
    - JPEG RGB but lossless depth;
    - the ego-motion prediction $T_{BC_{\mathrm{now}}}^{-1}T_{BC_{\mathrm{prev}}}T_{CO}$;
    - re-registration with an `object_name` lock and a reseed silhouette mask.
11. **P** - *ICP as implemented* (C3): Fast (Anderson in se(3), with a monotone safeguard) plus Robust
    (Welsch, with ν annealed from 3·median to $(1/3\sqrt3)\cdot$ the local noise floor, halving). The
    configuration in use is Anderson depth 5, robust, voxel 6 mm, d_max 20 mm. Also organized-cloud
    normals, and the performance engineering (the box ROI 38 → 5 ms; NN backend 0.31 vs 1.02 ms).
12. **S** - *Why averaging and FoundationPose tracking*: the ICP tracker on a still part wandered 5 mm std,
    24 mm range and up to 6° (2026-09-25).
13. **P** - *The save path in full*:
    - stationary tail (8 mm / 3° / 3 ticks);
    - robust mean (3.5 MAD, 3° gate, ≥ max(3, n/3) inliers);
    - one-shot ICP on 5 fresh clouds, robust-averaged, gated (fitness ≥ 0.2, ≤ 20 mm / 8°);
    - swing-twist for uncut tubes (R2: 30.8° of fake spin, 2026-10-09);
    - the statistics written to `assembly.json`; the tracker pose kept for comparison (2026-10-14: C1
      moved 13.1 mm at RMSE 3.6 mm).
14. **S** - (Optional) `export_mesh`: the boolean union of the saved CADs, to register an assembly as one model.

### Multi-View Close-Range Refinement
15. **P** - *The capture procedure* (D4): plan from where the arm is; check the extrinsic in TF against
    the file; pause tracking; per view a speed-capped transit with re-biased force watchdog, a 0.5 s
    settle, and the per-pixel median of 16 fresh organized clouds with TF at the stamp; return home; no
    recovery motion; the capture saved for offline replay.
16. **S** - *Preprocessing* (D2): crop 30 mm around the saved parts, cut 2 mm above the table plane,
    3 mm voxels, normals towards the camera.
17. **P** - *Ownership and equal view weights* (guards 1 and 4), contact samples (200 within 8 mm), and
    the parallel-face gap (reported; its rule off, with the lap-joint finding).
18. **S** - *Per-view diagnostic*: translation-only and prior-free (otherwise up to 1.5 mm bias in d).
19. **P** - *Online self-calibration loop*: remove $R_vd$ from the views **and** $R_{\mathrm{scan}}d$ from
    the saved poses; iterate up to 4 times; the 2026-10-02 incidents (a fake 8 mm fit-up change; 0.8 mm
    left after one pass).
20. **S** - *Resting contact* as an option and why it is off (it hid a real 3 mm gap). The fit-up is
    measured and warned.
21. **S** - *Fallback cascade*: "refined together or not at all".
22. **P** - *Validation method*: the exact synthetic renderer (and the z-buffer that faked a 1 mm
    extrinsic error), and synthetic captures with injected $d$ and known truth.
23. **S** - Symmetric parts in the refinement: a pipe's spin is named as symmetric and exempt; the
    rotation limit judges the axis tilt.

### Seams from the Registered Assembly
24. **§** - *The registry derivation* (E1):
    - box exact/envelope (a notched plate as its envelope with the deficit recorded);
    - tube: sweep axis from face normals, two radii, the end classified flat/plane/cylinder by fits;
    - extrusion: rounded rectangle, or an open constant-wall band with a cubic B-spline spine where
      control points and parameters are solved jointly;
    - refusal with the reason instead of approximation (a sideways-shifted band is refused with its
      wall range);
    - the both-ways verification and the library's numbers (≤ 0.07 mm, ≤ 0.24 % volume).
25. **S** - Stale-entry detection (0.5 mm), with the 2026-10-15 incident (the old plate entry turned 90°,
    no seam, silently).
26. **P** - *The registration-tolerant D4 rule* (E2):
    - the clip with slack on the near-constant axis (44 of 250 mm without it);
    - the face-extent rule yes/behind/short (the phantom underside; a lap thinner than the tolerance is
      undecidable);
    - the fit-up per seam;
    - the reject reasons, with negatives kept for the planner;
    - the canonical seam direction;
    - the coplanar tolerance ≤ ½ sheet.
27. **P** - *Curved-seam verdicts*: near misses within 2× the tolerance reported (2026-10-09); bores as
    negatives unless ≥ 200 mm (the cell's tool vs weldgen's 80 mm torch); off-plate-edge; ≥ 95 % torch
    cone clearance; unsupported pairs reported.
28. **S** - *Radius-PCA fallback* when mode A is unavailable (a missing registry entry or an import
    failure). The install-list incident of 2026-10-09 triggered it silently on the whole curved bench.
29. **S** - *Tack data*: `tack_no` (along the seam, the label to reach by) vs `order` (the weld sequence),
    the tack as a segment p0/p1, a per-tack approach on curved seams, and the closed-seam phase tied to
    the scene id (the save directory).

### Motion Planning and Execution
30. **P** - *Collision-model numbers*:
    - the capsule radii and the shoulder/elbow offsets;
    - 6 tube boxes ≤ 3.3 % r proud (vs 41 % for one box);
    - 12-36 sectors under a cut (one box would hang 39 mm below a 25° mitre);
    - band sag 1 mm;
    - the self pairs (tool vs base, shoulder, upper arm), the table exemptions, and box-box pairs with
      no clearance.
31. **P** - *The C++ port and its proof*: identical `is_valid` on random and near-contact configurations;
    > 20× asserted, ~590× measured; 3 of a plan's 3.5 minutes saved; the GIL released for parallel planner
    threads; the OMPL termination rule (budget, then the first exact solution, then a hard cap).
32. **P** - *Reachability internals*:
    - pose-only pre-checks before IK;
    - the pen's allowed contacts (part; table under a thin plate, never at it);
    - the 6-step LIN check;
    - random seeds around home;
    - the tie-breaks and penalties, and the roll grid (30° in use);
    - the per-tack reasons in the report.
33. **S** - *Transit clearance logic*: 20 mm, dropping to 90 % of an end pose's own clearance, never below
    the tack clearance, with the 2026-09-29 9 N skim incident; and the APS margin capped by the end poses' room.
34. **S** - *What follows APS*: C++ RRT-Connect, then Python RRT-Connect. The spline is re-checked and kept
    only if clear; TOTG samples are checked, with the trapezoid fallback.
35. **P** - *The descent check*: the clearance bound in the overshoot is $d_{\mathrm{ref}}-(\text{past}-\text{past}_{\mathrm{ref}})$;
    the table ends the overshoot; the roll retry 0, ±30, ±60, ±90°.
36. **S** - *The departure chain* is checked with the whole tool at tack clearance, and falls back to the
    approach point.
37. **P** - *Executor safety*: the joint-jump gate 0.35 rad, abort 8 N, release 40 N for moves away, the
    re-bias rule (the 7 N drift after a power cycle), the dry-run semantics, and the controller left as is
    (no switching).
38. **S** - *After contact*: dwell 0.5 s only if F < 2× touch; the retract to the departure distance under
    the 40 N limit; `tack_marks.json` per tack (commanded point, tip at contact, depth, force, or no_contact).
39. **S** - The pen's *visit order* is seam by seam, `tack_no` ascending, not the weld order.
40. **S** - Multi-view transits are planned and timed more simply (straight or RRT; speed cap 0.3 rad/s,
    min 0.3 s per segment) than marking transits.

### Implementation Details
41. **S** - The explicit Python install list and its test (the 2026-10-09 incident); the
    source-first/install-built package path trick (`extend_path`).
42. **S** - Which C++ is cell code (relay, TOTG service, `_transit_cpp`) and which is legacy (Jacobian
    node, drawing action). The legacy parts should not appear in Chapter 3.

---------------------------------------------------------------------------------------------

## 4. (b) All discrepancies between the draft and the code

Severity: **E** = the draft states something the code does not do (fix the text); **I** = incomplete or
imprecise, the missing condition or value changes the meaning; **N** = a nuance worth a word. Line
numbers are in the current tree.

| # | Draft location | Draft says | Code does (correct value) | Source | Sev. |
|---|---|---|---|---|---|
| 1 | §mc_registry | swept slab "fitted as a B-spline" | Only an **open** constant-wall band gets a fitted cubic B-spline spine; a closed rounded-rect tube's spine is weldgen's exact `rounded_rect_curve` (lines and arcs) | `weldgen_registry.py:462-475, 505-517, 549-555` | E |
| 2 | §mc_registry | "A fit is accepted when it is verified both ways ... within a fraction of a millimetre, and the volumes agree" | Accepted by the derive functions' own tolerance (0.05 mm; slab volume ≤ 1 %, envelope fill ≥ 90 %). The both-ways check (budget 0.25 mm, volume ≤ 1 %) runs only with `build_weldgen_registry.py --verify` and only sets `verify_failed`, which no consumer reads. All library parts verify at 0.037-0.067 mm, ≤ 0.24 % | `weldgen_registry.py:43-102, 250-315, 462-556`; `scripts/build_weldgen_registry.py:38-55` | E |
| 3 | §mc_registry | "a registry entry whose CAD file has changed ... is detected and refused" | Detected **geometrically**: a CAD vertex > 0.5 mm from the primitive (one-way SDF), not a file hash. Mode A refuses with a message; the multi-view falls back to the bounding box with a warning | `part_shapes.py:184-211, 237-243`; `icp_pose_refiner_node.py:1165-1173` | N |
| 4 | §mc_registry | "a tube (with its bore, and its cuts)" | A tube has a flat top cap and **one** cut end (the base: flat, mitre plane or saddle cylinder). A slab may be an *envelope* of a notched plate | `weldgen_registry.py:250-315, 43-102` | I |
| 5 | §mc_seams; ch4 §ws_link | WeldSet's seam rule applied; ch4: "applied unchanged" | Mode A uses a **registration-tolerant variant** `judge_registered`: pose tolerance 10 mm instead of the contact tolerance, a clip with slack, the face-extent rule, a material probe both ways instead of mutual visibility, penetration reported not rejected. Only the tack rule (and the curve constructors) are unchanged | `seam_from_registration.py:17-27, 98-123, 229-306, 409-416` | E (ch4) / I (ch3) |
| 6 | §mc_seams intro | "Seams are not detected in the point cloud" | True for mode A. Without a usable registry entry (or with `weld_method=radius_pca`) the node falls back to a radius-PCA detector on the static cloud unless `weld_fallback_pca=false` | `icp_pose_refiner_node.py:470-478, 1131-1142`; `seam_from_registration.py:44-45` | I |
| 7 | §mc_fitup | "the gap along a seam is measured, and a seam whose gap exceeds what a welder could bridge is reported"; the ISO 5817 limit | No ISO check on seams. Seams are judged by the **pose tolerance** (10 mm); curved pairs within 2× it are reported as `fitup_beyond_pose_tol`; every seam carries `fitup_mm`. The ISO 5817 no. 617 limit is applied **in the multi-view refinement** to the gap of down-facing faces resting on a neighbour, as a **warning**, never a rejection | `seam_from_registration.py:281, 366-367`; `curved_seams.py:47-58, 185-195`; `multiview_refine.py:75-93, 826-858` | E |
| 8 | Eq. `eq:iso_gap` | $h_{\max}=c_0+c_1a$, (0.5, 0.1)_B, (0.5, 0.2)_C, (1, 0.3)_D, a = 0.7t | Correct for t > 3 mm but **capped**: $\min(c_1a+c_0,\ h_{\mathrm{cap}})$ with caps 2.0 (B), 3.0 (C), 4.0 (D) mm. For t ≤ 3 mm: $0.1a+0.2$ (B), $0.3$ (C), $0.5$ (D), uncapped. Level **C** in use. t = the smaller of the two parts' smallest bounding-box extents (for a pipe this is not its wall) | `/workspaces/welding_cell_ws/weld_generator/weldgen/config.py:197-209`; `multiview_refine.py:93, 822-838`; `multiview_service.py:90` | E |
| 9 | `tab:mv_acceptance`, overlap | refused when "worse than the saved assembly's by more than 0.2 mm" | Refused when penetration > **1.0 mm** *and* > saved + 0.2 mm. Over 1.0 mm but no worse: applied with a warning | `multiview_refine.py:127, 917-924` | E |
| 10 | `tab:mv_acceptance`, fit | "to fill" | fitness < 0.2; and the relative pose of two parts changed > 2 mm or > 1° with > 2 mm of a part's correction along its unmeasured directions | `multiview_refine.py:126-129, 915-916, 927-951` | I |
| 11 | `tab:mv_acceptance` | (rows missing) | (i) $\|d\|>0.5$ mm with online self-cal off → all parts kept as saved; (ii) the fallback cascade (a part held because a touching part was rejected); (iii) fit-up beyond ISO → warning only | `multiview_refine.py:590-591, 630-637, 644-684, 841-858` | I |
| 12 | `tab:mv_acceptance`, correction size | > 10 mm or > 5°, a pipe by its axis tilt | Confirmed. With online self-cal the limits judge the correction **beyond** the camera's scan-pose share $R_{\mathrm{scan}}d$; a pipe's spin > 0.5° is warned | `multiview_refine.py:124-125, 597-621, 903-914` | N |
| 13 | `tab:calib_chain`, extrinsic translation | "online, from multi-view disagreement" | Online: each run removes its own d from that run's views and saved poses. The calibration file is corrected offline from ≥ 3 agreeing runs (spread ≤ 1 mm, leave-one-out ≤ 1 mm, same extrinsic sha1) and **promoted by hand** | `multiview_refine.py:110-120, 588-629`; `selfcal.py:16-30, 46-50, 122-189` | I |
| 14 | `tab:calib_chain`, extrinsic rotation | "camera tilt separated from table tilt at four wrist yaws" | The library needs any yaw spread ≥ 90° (four is the script's choice), and corrects only the camera's **two tilt angles** (yaw about the optical axis unobservable from a plane, fixed to 0) | `table_probe.py:236-257, 272-307` | N |
| 15 | §mc_kin_tcp_table TODO | plane $n^{\top}x+d=0$ fitted to touches | Ordinary least squares on height, $z=ax+by+c$, converted to (n, d) | `table_probe.py:43-71`; `collision.py:289-300` | N |
| 16 | Eq. `eq:mv_objective` | $\sum_v\sum_i\rho(r_{v,i})^2+\sigma^2\xi^{\top}\Sigma^{-1}\xi+w_o\sum_c\max(0,-sd_c-\varepsilon)^2$ | The data term has the per-view weight $\omega_v=1/N_v$ and is IRLS with Welsch weights ($\nu_0=\max(3\,\mathrm{med},P_{90},\sigma)$, ×0.7 → σ). Correspondences are gated (≤ 10 mm, normal gate 60°). Values: σ = 1.5 mm; Σ from 1° and 3 mm; $w_o$ = 10; ε = 0.5 mm. The prior and overlap terms are as written | `multiview_refine.py:31-32, 59-96, 395-405, 409-439, 529-534` | I |
| 17 | §Observability | "Directions that are not measured keep their prior value" | They are held by the prior (MAP, σ 3 mm / 1°), not fixed. The weak directions drive the relative-pose acceptance rule; a pipe's symmetric spin is exempt. $\sigma_{\mathrm{noise}}$ = 1.5 mm; H in $(\rho\omega,t)$ coordinates with ρ the 90th-percentile radius | `multiview_refine.py:19-26, 377, 459-461, 486-525` | N |
| 18 | Eq. `eq:cartesian_bow` | control point $x_m+\beta b$ | Control point $x_m+2\beta b$, so the curve **midpoint** is displaced by β (with the draft's form only β/2). $b=-(a_0+a_1)/\|a_0+a_1\|$ | `marking.py:129-131, 136-137, 146` | E |
| 19 | §mc_transit | "... and only then a sampling-based planner: OMPL's APS" | After APS: C++ RRT-Connect + shortcut, then Python RRT-Connect (3 attempts) + shortcut. The Cartesian stage is used tack to tack only (not home). Multi-view transits: straight edge or Python RRT only, speed-capped timing | `marking.py:265-314, 780-782`; `multiview.py:398, 408`; `multiview_service.py:282, 294` | I |
| 20 | §mc_tacks | "open or closed: both ends tacked, the rest evenly spaced" | A closed seam has no ends: $n=\max(4,\lceil L/d_{\max}\rceil)$ rounded up to even, evenly spaced with a deterministic phase (from scene id = save dir, and seam id) | `/workspaces/welding_cell_ws/weld_generator/weldgen/tacks.py:14-16, 80-84, 115-124`; `icp_pose_refiner_node.py:1198` | E |
| 21 | §mc_tacks | order "ends first, then by bisection, alternating between seams" | Same-**class** seams interleaved round-robin; on a loop, opposite pairs in bisection order. The **pen** visits seam by seam, `tack_no` ascending, not in weld order | `weldgen/tacks.py:13-16, 161-184`; `seam_from_registration.py:422-424`; `marking.py:531-534` | I |
| 22 | §mc_reach, Eq. `eq:roll_dp` | "a roll is chosen per tack"; $q_k(\varphi_k)$ | A **(tilt, roll)** pair per tack from 5 work angles × 12 rolls; $q_k$ is the approach pose; the chain starts at home or the previous seam's last approach; W = diag(2,2,1.5,1,1,0.5) with joints wrapped the short way except wrist_3 | `tack_reach.py:402-412, 420-477` | I |
| 23 | §mc_reach, straight seams | max smallest clearance | Confirmed. Also: tie-break by joint travel + 3 per tack near the wrist singularity; consecutive approach poses ≤ 1.2 rad per joint; clearance rounded to 0.1 mm in the key | `tack_reach.py:381-391, 524-544` | N |
| 24 | §mc_collision | "capsules for the pen and holder, boxes for the camera body" | Boxes for the **flange adapter, the camera arm and the camera body**; capsules for the holder flange, holder body and pen | `config/pen_tool.json` primitives | I |
| 25 | §mc_collision | box-box is an intersection test only | Confirmed. So box-box pairs get **no clearance**. The `collision.py` docstring's "after inflating the scene box by the clearance" is wrong; no inflation is passed | `collision.py:20-22` vs `:271`; `transit_cpp.cpp:205-213` | N (docstring E) |
| 26 | §per_symmetry, Eq. `eq:spin_settle` | argmin of the gap range | Confirmed. Add: the grid (1° then 0.05°); accepted only if range_after ≤ max(1.5 mm, ¼ before) and gain ≥ 1 mm; g is the axial gap at the outer radius; it also applies to **saddle-cut** pipes standing on a pipe | `curved_seams.py:319-320, 330-404` | I |
| 27 | §per_stationary, Eq. `eq:p2plane` | weighted point-to-plane with Welsch weights | Confirmed. In the robust mode there is no distance cutoff ($d_{\max}$ only for the reported fitness); ν is annealed (3·median → noise floor/(3√3), halving); Anderson acceleration; in use: depth 5, robust, d_max 20 mm, voxel 6 mm | `icp.py:57-63, 634-669, 742-752, 846-881`; `icp_pose_refiner_node.py:363-373` | N |
| 28 | Eq. `eq:aim_law` | $\omega_C=(-Ke_y,Ke_x,-k_r\psi)$ with a deadband and a rate cap | Confirmed. The pan/tilt rate is scaled by $s=(\|e\|-0.2\delta)/\|e\|$ when engaged; caps 15°/s on $\|\omega_{xy}\|$ and on $|\omega_z|$; K = 2.5 s⁻¹, $k_r$ = 1.5 s⁻¹ | `camera_aim.py:53-59, 172-191` | N |
| 29 | Eq. `eq:relay` | ramp from $\dot q_{k-1}$ to $\dot q_k$ over T | The ramp starts from the value *being sent* when the input arrives; T is an EMA of consecutive gaps (clipped 2-50 ms); each tick is also acceleration- (4 rad/s²) and speed- (0.6 rad/s) limited | `aim_rate_relay.cpp:73-108` | N |
| 30 | §mc_aim "Execution and safety" | collision / branch / wrist look-ahead; 0.3 s deadman | Confirmed. Missing: the |F| > 15 N abort, the stale joint-state latch, the excursion limits; the aim's collision model contains no parts (none registered yet) | `camera_aim_node.py:23-35, 110, 125-131, 349-368` | I |
| 31 | Assumptions | "they rest on the table or on each other (they do not float)" | The multi-view refinement deliberately does **not** impose resting (`resting_contact` False since 2026-10-02); gaps are measured and warned. The resting assumption is used by the cut-spin settling | `multiview_refine.py:75-90`; `curved_seams.py:376-381` | N |
| 32 | §mc_multiview intro | "moves the camera to a few close views of the seams" | 4 views at 0.40 m, all aimed at the **centre of the assembly's bounding box**, chosen by how much of the seam band they see | `multiview.py:283-284, 296-308, 336-361` | N |
| 33 | §mc_marking | "overshoot" (no value) | In use **5 mm** (node); the library default is 3 mm. Speeds 50/20/2 mm/s, slow zone 6 mm | `tack_marking_node.py:119-126`; `marking.py:375, 671` | N |
| 34 | `tab:calib_chain`, kinematics | factory calibration "used in every tool" | True for every entry point that uses the library FK (they call `use_kinematics`) and for TF via the driver's `kinematics_params_file`; the library default is nominal. A comment in `config/pen_tool.json` still says the planner uses the nominal chain (stale) | `kinematics.py:233-273`; `config/pen_tool.json` `_comment` | N |
| 35 | ch4 §ws_curves ("constructors ... reused by the cell"), §ws_link ("curve constructors ... applied unchanged") | the cell reuses WeldSet's curve constructors | The cell uses weldgen's **verification-side** intersection factories (`ellipse_from_plane_cylinder`, `saddle_from_cylinders`, `OffsetCurve`) and `verify_curved.seam_verdict`, not the curve-first generation constructors. weldgen itself says the ellipse factory "belongs to the VERIFICATION side" | `curved_seams.py:3-6, 119-128, 169, 210, 269`; `/workspaces/welding_cell_ws/weld_generator/weldgen/curves.py:518-520` | I |
| 36 | §mc_aim (the 15°/s cap in `camera_aim.py:16-19`; Eq. `eq:aim_law`/`eq:aim_dls` gains) | the camera turns at $\omega_C$ (cap ~15°/s, K = 2.5 s⁻¹) with the optical centre still | **Code defect:** `ur5e_jacobian`'s angular rows are 2× too large, so the DLS joint rates achieve $\omega_C/2$. Verified: 15°/s commanded → 7.50°/s achieved; optical centre moving 8.2 mm/s instead of 0 (corrected by the P pull). The equations are right as the design; the achieved numbers are half. Fix the Jacobian (÷4ε) or report the achieved values | `kinematics.py:296-303`; `camera_aim.py:231-240` | E (code) |

Checked and **confirmed** without remarks (so the author knows they were verified):
- the aim's base-frame ray with exact latency compensation (`eq:aim_error`), the 0.3 s look-ahead and
  deadman, 125 Hz / 2.5 ms / 500 Hz, the DLS (`eq:aim_dls`), and the swing-twist roll ψ;
- the normal gate at cos 60° and its thin-plate rationale;
- the swing-twist split `eq:swing_twist`;
- the plane intersection `eq:plane_intersection`;
- the ellipse `eq:ellipse_seam` (mathematically; the code uses the equivalent centre/semi-axes form);
- the saddle and band seams;
- departure points at 100 mm;
- the Cartesian-line-first order and β ∈ {0, 30, 60, 100} mm;
- APS in scaled coordinates with W = diag(2,2,1.5,1,1,0.5) (`eq:transit_cost`);
- TOTG timing;
- the 1.5 N touch threshold with a bias at rest (`eq:touch`), the sign of the contact depth δ_c, the
  overshoot ending at the table, and strokes with force-corrected depth;
- the branch lock (lift, elbow, wrist_2);
- δ₀ = 10 mm and κ = 0.15 rad/mm;
- the ±180° wrist_3 cable window;
- the extrinsic model $r=n^{\top}(Tp-q+R_vd)$ and its Schur complement;
- the observability threshold of 0.5 mm;
- correction limits of 10 mm / 5° with a pipe judged by its axis tilt;
- the parts taking turns;
- true-shape neighbours (hollow bore, cut end).

All `\SRC{}` function names cited by ch3 exist with those names.

---------------------------------------------------------------------------------------------

## 5. (c) Candidate algorithm boxes (pseudocode faithful to the code)

Each box names its source functions. The numbers are the values in use.

### Algorithm 1: Finger-pointing camera aim, one control cycle
(`camera_aim.CameraAim.observe/step`, `check_ahead`; `scripts/camera_aim_node.py`)
```
Input: q (current joints), image events (t_k, u, v), K, home pose R_home, anchor p_C0
on image (t_k, u, v):                                    # ~30 Hz, ~0.1 s late
    q_k  <- JointHistory.at(t_k)                         # interpolated; none if > 0.25 s outside
    r_B  <- OneEuro(t_k, R_BC(q_k) * normalize(K^-1 [u v 1]^T));  r_B <- r_B/|r_B|
every cycle (125 Hz):
    if no fingertip for 0.3 s: r_B <- none               # deadman; smoother ramps to zero
    R <- R_BC(q);  psi <- twist_z(R_home^T R)            # roll from home (swing-twist)
    w <- 0
    if r_B and (d <- R^T r_B).z >= 0.3:
        e <- (d.x/d.z, d.y/d.z);  delta <- 0.05 * W / f_x
        engaged <- |e| > delta  or  (engaged and |e| > 0.4 delta)
        if engaged: s <- (|e| - 0.2 delta)/|e|;  w_xy <- K s (-e_y, e_x);  cap |w_xy| <= 15 deg/s
    w_z <- clip(-k_r wrap(psi), +-15 deg/s)
    for i in {x, y}: if R exp(w_i * 0.35 s) leaves |pan|<=35 / |tilt|<=25 deg outward: w_i <- 0
    w_cmd <- two first-order lags (tau 60 ms), |dw| <= 60 deg/s^2 * dt
    v_E <- sat_20mm/s( k_p (p_C0 - p_C) ) + (R w_cmd) x (p_E - p_C)
    qd  <- J^T (J J^T + lambda^2 I)^-1 [v_E; R w_cmd];  scale so max|qd_j| <= 0.5 rad/s
           # NB the code's J has doubled angular rows (A1 FINDING): it achieves R w_cmd / 2
    q1  <- q + 0.3 s * qd
    if |sin q1[4]| < sin 0.35 or sign(q1[1,2,4]) != sign(home[1,2,4]) or not valid(q1)
       or |F - F_bias| > 15 N or joint states older than 0.2 s:
        stop, hand the joints back, latch until the hand has been away 1 s
    else publish qd to the relay
```

### Algorithm 2: Rate relay at the controller rate (`src/aim_rate_relay.cpp`)
```
on input u (6 finite values):
    if streaming and 0 < gap < 0.1 s: T <- 0.8 T + 0.2 clip(gap, 2 ms, 50 ms)
    start <- out;  goal <- u;  t_in <- now;  active <- true;  stopping <- false
every tick (500 Hz, dt in [0.5, 20] ms):
    if not active: return
    if now - t_last_input > 0.1 s: stopping <- true
    want <- 0 if stopping else start + clip((now - t_in)/T, 0, 1) * (goal - start)
    a <- 2 rad/s^2 if stopping else 4 rad/s^2
    for each joint j: out_j <- clip(out_j + clip(want_j - out_j, -a dt, a dt), -0.6, 0.6)
    publish out
    if out = 0 and (stopping or (goal = 0 and now - t_in >= T)): active <- false
```

### Algorithm 3: Fast and robust point-to-plane ICP with a normal gate (`icp.icp_point_to_plane`, robust branch)
```
Input: model samples P with outward normals N_P; scene Q with camera-facing normals N_Q; T0
index <- NN(Q)
r0    <- |n_q . (T0 p - q)| over gate-passing pairs
nu_end <- (1/(3 sqrt 3)) * median_i median_{k in kNN_7(i)} |(q_k - q_i) . n_i|
nu    <- max(3 median(r0), nu_end);  inner <- 6;  T <- T0
while iterations < max_iter:
    T <- WelschStage(T, nu, inner)
    if nu = nu_end: converged; break
    nu <- max(nu/2, nu_end);  inner <- min(inner + 1, 10)
WelschStage(T, nu, m):                                # fixed nu, Anderson-accelerated
    repeat m times:
        for every p_i: q_i <- NN(T p_i);  r_i <- n_qi . (T p_i - q_i)
        w_i <- exp(-r_i^2 / 2 nu^2) * [ (R n_Pi) . n_qi >= cos 60 deg ]
        x <- argmin sum_i w_i ( (T p_i x n_qi).omega + n_qi.t + r_i )^2;  T' <- exp(x) T
        Anderson in se(3) about T0: candidate g_k - dG theta, theta = argmin |dF theta - f_k|;
        accept if the Welsch energy does not increase, else take the plain step and reset the history
return T, fitness = #{dist < d_max and gate}/|P|, inlier RMSE
```

### Algorithm 4: The pose a part is saved with
(`pose_stats.stationary_tail`, `robust_pose_mean`, `remove_twist`; node `_save_object`, `_fp_icp_refine`)
```
W <- the tracker's recent poses (75 at ~25 Hz)
tail <- [W_last]; walk back over W:                      # stationary_tail(8 mm, 3 deg, 3)
    keep T if |t - median_t(tail)| <= 8 mm and angle(R, chordal_mean(tail)) <= 3 deg
    else count a disagreement; 3 in a row ends the tail (the part was moved)
if |tail| < 5: save the last pose unaveraged (warn)
inliers <- |t - median t| <= 3.5 * 1.4826 MAD per axis  and  angle(R, chordal_mean) <= 3 deg
if |inliers| < max(3, n/3): inliers <- all
T_avg <- (chordal_mean R_inliers, median t_inliers);  record std, range, n_used
T_icp <- robust mean of ICP(T_avg) on 5 fresh clouds whose fitness >= 0.2   # Algorithm 3
if the part is an uncut tube (registry symmetry axis a):
    R_icp <- R_avg * swing( R_avg^T R_icp, a )           # drop the unobservable spin
if |T_icp - T_avg| <= 20 mm / 8 deg: save T_icp else save T_avg
```

### Algorithm 5: A CAD model as an exact primitive (`weldgen_registry.derive_entry`, `verify_both_ways`)
```
for derive in (box, tube, extrusion):                    # first that verifies wins
  box:  OBB, axes sorted (u longest, w thickness)
        exact    if every vertex is a corner (<= 0.05 mm) and |V - LWt|/LWt <= 1 %
        envelope if every vertex is on the box surface and the mesh fills >= 90 % of it
  tube: a <- smallest eigenvector of sum_f A_f n_f n_f^T (re-estimated on near-lateral faces)
        lateral faces |n.a| < 1e-3;  circle fit -> centre;  all lateral vertices on r_i or r_o (<= 0.05 mm)
        each end: flat | plane (normals within 0.05 deg) | cylinder (free-axis LSQ fit); residual <= 0.05 mm
        need a flat top; the base is the other end -> frame (z axis, base at z = 0), params, base_cut
  extrusion: caps are two planes along the sweep axis
        two cap loops -> rounded rectangle: equal walls, corner arcs fitted -> spine = rounded_rect_curve
        one loop -> wall along inward normals; if P95 - P5 > 0.05 mm: REFUSE (report the range)
                    else for m = 4..16 control points, for spine in (side A, side B, midline):
                        cubic B-spline, end-interpolated, control points and parameters solved jointly
                        accept the first with max distance <= 0.025 mm
if none: record "unsupported" with every reason          # never guessed
verify (--verify): CAD->primitive and primitive->CAD max distance <= 0.25 mm, |dV|/V <= 1 %
at run time: an entry whose CAD vertices lie > 0.5 mm off its primitive is stale -> mode A refuses
```

### Algorithm 6: Planar seam of two registered faces (`seam_from_registration.judge_registered`)
```
Input: faces (A, fa), (B, fb) at registered poses; pose tolerance tau = 10 mm
if n_a || n_b: coplanar -> butt/edge candidate; facing within tau and overlapping -> non-weldable record
(x0, d) <- intersection line of the two face planes
I_A <- clip(face A, line) with slack tau on the in-face axis the line holds nearly constant
I_B <- clip(face B, line) likewise;  [lo, hi] <- I_A intersect I_B;  if empty: no seam
if line separation from the faces > tau: no seam
reach_A, reach_B <- does each face reach past the other's plane by > tau? (yes | short | behind)
if behind in either: no seam                              # a plate's own underside: phantom
fitup <- signed distances of each face's near-edge corners to the other plane (+ gap, - penetration)
reason <- member_within_pose_tol if short in either
       else degenerate_dihedral if the dihedral is out of range
       else too_short if hi - lo < 10 mm
       else bisector_blocked if no torch axis about (n_a + n_b) is clear at 5 samples
record {p0, p1, class, approach, weldable = (reason = none), reason, fitup}  # negatives kept
```

### Algorithm 7: Curved seam of a pipe on a plate, or of a branch on a pipe
(`curved_seams._tube_on_slab`, `_tube_on_tube`, `_record`)
```
for each plate face F (+w, -w) and each pipe end E (cut base, flat top):
    require the body on F's outer side with s * (n_F . z) >= sin 10 deg
    C <- plane(F) intersect outer cylinder                # circle or ellipse (saddle: two cylinders)
    sample C at 1 point/mm;  g_j <- axial distance from C(theta_j) to end E (+ gap, - penetration)
    keep (F, E) with the smallest max|g| <= 2 tau
for that (F, E):
    n_a <- face normal; n_b <- radial wall normal; approach_j <- normalize(n_a + n_b)
    if max|g| > tau: reject "fitup_beyond_pose_tol (g range)"     # reported, not dropped
    elif some point is off the plate: reject "off_plate_edge"
    else weldable if the D4 torch cone is clear at >= 95 % of the points
    also record the bore (inner cylinder) as a negative unless 2 r_i >= 200 mm
```

### Algorithm 8: Settling a cut pipe's spin on what it stands on (`curved_seams.settle_cut_spins`)
```
for each pipe P with a cut base (mitre or saddle), for each part B it may stand on:
    C <- the seam curve of P's outer wall on B (independent of P's spin); phi_j, z_j in P's frame
    for psi = -180..180 deg step 1:  range(psi) <- max_j g_j - min_j g_j,
                                     g_j = h_cut(phi_j - psi, r_o) - z_j
    refine psi in +-1 deg, step 0.05;  require the end on B (max|g| <= 2 tau)
take the best B; accept if range_after <= max(1.5 mm, range_before/4) and range_before - range_after >= 1 mm
P.pose <- P.pose * Rz(psi*)                              # saved: assembly.json, static cloud
```

### Algorithm 9: Planning the close-range views (`multiview.plan_views`)
```
targets <- CAD samples weighted 1 if 1 < d(another part) < 30 mm, 0.1 otherwise; drop d <= 1 mm
c <- centre of the parts' bounding box
candidates <- look_at(c + 0.40 (cos e cos a, cos e sin a, sin e), c, roll)
              for e in {45, 60} deg, a step 30 deg, roll step 90 deg       # 96
keep the candidates that see a seam target (0.28 < z < 0.60 m, 69x42 deg FOV,
     incidence <= 60 deg, no part box on the ray)
count_i <- 0; picks <- []
repeat 4 times:
    gain(v) <- sum_i w_i rho[count_i] seen_i(v),  rho = (1, 0.5, 0.25, 0, ...)
    exclude directions within 30 deg of a pick
    take the best-gain candidate that is FEASIBLE (lazy):
        IK on the elbow-up branch; |sin q5| and |sin q3| >= sin 0.35;
        arm and tool (camera included) >= 20 mm from the parts, fixtures and table
    among its rolls within 98 % of the gain, prefer the least joint travel from home
    picks += v;  count_i += seen_i(v)
order the picks by nearest neighbour in joint space (2 pi equivalents inside the limits, cable window)
plan collision-free transits between the views and home (straight edge or RRT-Connect, 20 mm clearance)
ok if >= 2 views and a way home
```

### Algorithm 10: Joint refinement of touching parts, with acceptance
(`multiview_refine.refine_assembly`, `refine_part`, `judge`, `_fallback`)
```
Input: parts k (samples, normals, saved pose T_k^0, true shape), views v (points, normals, R_v)
omega_i <- 1 / N_{view(i)} (normalised);  owner(x) <- argmin_k d_k(x),
           -1 if min > 15 mm or the second nearest < 4 mm
for round = 1..2, parts in order of owned points:
    c <- centre of part k;  contacts <- <= 200 samples within 8 mm of a neighbour
    repeat <= 30 GN steps (about c):
        pairs: owned points, dist < 10 mm, normal gate 60 deg;  r_i = n_i . (T p_i - q_i)
        w_i <- omega_i exp(-r_i^2 / 2 nu^2);  nu: max(3 med, P90, sigma) -> x0.7 per step -> sigma = 1.5 mm
        rows: sqrt(w_i)[p_i x n_i, n_i] | -sqrt(w_i) r_i
              sqrt(10)[s_c x n_c, n_c]  | -sqrt(10)(sd_c + 0.5 mm)    for sd_c < -0.5 mm (true-shape SDF)
              diag(L)                   | -L * xi(T, T_k^0),  L = sigma/(1 deg, 3 mm)
        T <- exp(x) T
    H_k <- sum_i omega_i J_i^T J_i,  J_i = [(p_i x n_i)/rho, n_i]
    recompute ownership; stop early if no part moved > 0.2 mm / 0.05 deg
weak_k <- eigvecs of H_k with sigma/sqrt(lambda) > 0.5 mm (a pipe's spin: exempt)
reject k if correction > 10 mm or > 5 deg (pipe: axis tilt), or fitness < 0.2,
         or penetration > 1 mm and > saved + 0.2 mm,
         or its pose relative to a neighbour changed > 2 mm/1 deg with > 2 mm along weak_k
fallback: rejected parts keep T_k^0; refine the accepted ones again against them;
          hold every accepted part whose pose relative to a held one changed > 2 mm/1 deg (repeat)
warn (never reject) when a resting gap exceeds the ISO 5817 no. 617 limit at level C
```

### Algorithm 11: Extrinsic-translation estimate and online self-calibration
(`multiview_refine.estimate_extrinsic`, the online loop in `refine_assembly`; `selfcal.evaluate`)
```
unknowns x = (xi_1..xi_n, d)                             # parts about their centres; d in the camera frame
repeat <= 20: r = n . (T_k p - q + R_v d);  rows [(p - c_k) x n, n | n^T R_v];  Welsch IRLS;  update
N <- A^T A;  S <- N_dd - N_dx N_xx^+ N_xd;  info_mm <- S / sigma^2 / 1e6
keep d only along eigen-directions with 1/sqrt(info) <= 0.5 mm; report the others as undetermined
if |d| > 0.5 mm:
    if online self-cal is off: keep every part as saved ("the views disagree by d")
    D <- d;  repeat <= 4 times:
        views'  <- each point minus R_v D;  prior' <- each T_k^0 translation minus R_scan,k D
        refine again (Algorithm 10) -> residual d';  if |d'| < 0.5 mm: break;  D <- D + d'
    judge against prior'; report against T_k^0; record D with its information and the extrinsic's sha1
persistent (selfcal.py, offline): runs with the same sha1;
    d* = (sum I_r)^+ sum I_r d_r on the determined directions
    write T_tcp_to_cam_selfcal (t <- t - R_tc d*) only if >= 3 runs, spread <= 1 mm, leave-one-out <= 1 mm
    promotion by hand
```

### Algorithm 12: Reachability with a locked branch
(`tack_reach.plan_tacks`, `_try_tack`, `_plan_per_tack`)
```
TryTack(tack, tilt, roll):
    a <- rotate(-approach, about the seam tangent, tilt);  p <- tack point
    pose-only: the tool at the tack pose (pen excluded) and at p - 35 mm a (pen included) >= 3 mm, else fail
    q_app  <- IK_branch(T(p - 35 mm a, a, roll));  clearance with the pen >= 3 mm
    q_tack <- IK_branch(T(p, a, roll)) seeded at q_app;  clearance excluding the pen-part / pen-table contacts
              >= 3 mm; the pen at the table = collision
    |q_tack - q_app|_inf <= 1.2 rad;  5 LIN steps: IK continuity (<= 0.5 rad) and clearance >= 3 mm
IK_branch(T): DLS IK from seeds + 6 random seeds ~ N(home, 0.4^2); map wrist_3 into home +- 180 deg;
              return the first solution with sign(q[1,2,4]) = sign(home[1,2,4])
straight seam: for (tilt, roll) in {0, +-10, +-20} x {0, 30, ..., 330}:
    all tacks TryTack OK in tack_no order and consecutive approach poses <= 1.2 rad per joint
    choose max(min clearance, rounded to 0.1 mm), then min(sum |dq_app|_1 + 3 * #near-wrist-singular)
curved seam: candidates_k <- every OK (tilt, roll) of tack k
    Viterbi: C_k(j) = min_i [ C_{k-1}(i) + |W delta(q_app(k-1,i), q_app(k,j))|_2 ]
                      + 0.15 rad/mm * max(0, 10 mm - clr(k,j))
             W = diag(2, 2, 1.5, 1, 1, 0.5); delta wraps each joint except wrist_3;
             C_0 = home or the previous seam's last approach
    backtrack; unreachable tacks reported with the best attempt's reason and skipped
```

### Algorithm 13: The marking plan for one tack (`marking.build_marking_plan`, `descent_check`, `departure_chain`)
```
for each tack in (seam, tack_no) order:
    target <- the stroke's first point (stroke) or the tack point (dot)
    for d_roll in (0, +30, -30, +60, -60, +90, -90):
        q_app <- the report's own (dot, d_roll = 0) or IK_branch(approach pose at roll + d_roll)
        chain <- IK steps of 2 mm along the pen from 35 mm out to target + 5 mm overshoot
        for each pose: past <- (tip - target) . a
            if the pen is at/below the table: if past > 0.1 mm: cut the chain here (accept) else reject
            before target: clearance (allowed pen contacts excluded) >= 3 mm
            in overshoot:  clearance >= d_ref - (past - past_ref)   # can shrink by at most the travel
        if the chain is clear: take this roll; break
    dep <- IK steps of 5 mm out to 100 mm; every pose valid with the whole tool at 3 mm
    if dep: the transit ends at the departure point; the descent = dep reversed + chain
    else:   the transit ends at the approach point
    q_goal <- 2 pi equivalent nearest the previous pose, wrist_3 within home +- 180 deg
    transit <- TransitCascade(q_prev, q_goal, clearance = max(3 mm, min(20 mm, 0.9 clr(q_prev), 0.9 clr(q_goal))))
    shift the descent by the transit's whole turns; check the joint limits
finally: TransitCascade(q_last, home) without the Cartesian stage, unwrap off
```

### Algorithm 14: Transit cascade (`marking.transit_path`, `cartesian_transit`, `aps_path`)
```
if the straight joint edge is clear (checked every 0.01 rad): return it
if tack-to-tack:                                          # the tool's Cartesian path
    b <- -(z_a + z_b)/|z_a + z_b|                          # away from the joint
    for beta in (0, 30, 60, 100) mm:
        x(s) = (1-s)^2 x_a + 2 s (1-s)(x_mid + 2 beta b) + s^2 x_b; R(s) = slerp(R_a, R_b, s)
        IK chain (<= 5 mm, <= 3 deg per step, joint jump <= 0.3 rad); every edge clear -> return
margin <- clip(min(2 mm, (min end clearance - clearance)/2), 0)
path <- APS(4 x RRT-Connect, hybridise + shortcut; y = W q; budget 3 s, then the first exact solution, <= 10 s;
            clearance + margin)
if path: re-check every edge at the nominal clearance
else:    path <- C++ RRT-Connect + collision-aware shortcut, else Python RRT-Connect (3 tries) + shortcut
if smoothing: centripetal Catmull-Rom spline, re-checked; keep it only if clear
timing: TOTG (0.3 rad/s, 0.5 rad/s^2, 5 mrad blend); every sample collision-checked, else a trapezoid
```

### Algorithm 15: Force-gated descent and the contact record
(`motion.TrajectoryExecutor.execute`, `marking.time_descent`; node `tack_marking_node._run_next`, `_stroke`)
```
after the transit (arm at rest at the departure point): F_bias <- mean F over 0.5 s
timed <- for each descent pose: v = 50 mm/s while > 35 mm before the point, 2 mm/s in the last 6 mm,
                                20 mm/s otherwise; dt >= 20 ms per sample
require the first point within 0.35 rad of the current joints; send the goal
every 2 ms while running:
    f <- |F - F_bias|
    if f > 8 N or abort: cancel; fail
    if f > 1.5 N: cancel; contact <- (q now, f); break
if contact: tip <- FK(q_contact) tool tip;  delta_c <- (p_reg - tip) . a   # + met before the point
else: record no_contact (the descent ran through the 5 mm overshoot, or ended at the table)
mark: dot -> dwell 0.5 s if f < 3 N;  stroke -> 10 mm chunks, depth += 2e-5 m/N (f_hold - f_axis)
      clipped to +-0.2 mm, stop after 2 chunks under 0.4 N
retract along the pen to the departure distance; only |F| > 40 N stops it
write tack_marks.json: point, axis, roll/tilt, tip at contact, delta_c, force
```

---------------------------------------------------------------------------------------------

## 6. Appendices

### Appendix A: values in use vs. library defaults (quote the "in use" column)

| Quantity | Library default | In use | Where set |
|---|---|---|---|
| Tack clearance | 10 mm (`MarkingConfig`, `CollisionModel`) | **3 mm** | `config/marking.json clearance_m` |
| Transit clearance | 20 mm | 20 mm (≥ tack clearance, ≤ 0.9 × end-pose clearance) | marking.json; `marking.transit_model` |
| Roll grid step | 15° | **30°** (12 rolls; retry offsets ±30/60/90°) | marking.json `roll_step_deg` |
| Work angles | (0, ±10, ±20)° | same | marking.json |
| Standoff / departure | 35 mm / 100 mm | same | marking.json, pen_tool.json |
| Overshoot | 3 mm (`descent_chain`, `build_marking_plan`) | **5 mm** | `tack_marking_node` `overshoot_m` |
| Descent speeds | `time_descent(v_tip=20 mm/s)` | 50 / 20 / 2 mm/s, slow zone 6 mm | tack_marking_node |
| Touch / abort / release force | 1.5 N / 8 N / 40 N | same | pen_tool.json; `motion.PARAMS` |
| Transit planner | `transit_path(planner="rrt_connect")`; APS budget 2 s (marking) / 1 s (binding) | **APS, 3 s**, up to 10 s, 4 planners, 8 paths, margin 2 mm, smoothing on | tack_marking_node |
| TOTG tolerance / resample | node: 0.02 rad / 0.1 s if ≤ 0 | 0.005 rad / 0.02 s; 0.3 rad/s, 0.5 rad/s² | `motion.PARAMS`; tack_marking_node |
| ICP | anderson 0, robust False, d_max 10 mm | **anderson 5, robust True, d_max 20 mm, voxel 6 mm**, gate 60° | icp_pose_refiner_node |
| Save ICP | none | 5 clouds, fitness ≥ 0.2, ≤ 20 mm / 8° | icp_pose_refiner_node |
| Pose window | `stationary_tail(8 mm, 3°, 3)` | FP window 75 (~3 s), min 5 | icp_pose_refiner_node |
| Seam pose tolerance | 10 mm | 10 mm (clip 0.5-30) | `weld_pose_tol_mm` |
| Multi-view frames | n/a | 16 per view, settle 0.5 s | `mv_n_frames`, `mv_settle_s` |
| Multi-view limits | 10 mm / 5°, prior 3 mm / 1°, rel. 2 mm / 1° | same | `mv_*` (multiview_service) |
| ISO quality level | C | C | `mv_quality_level` |
| Aim control rate | docstring says 50 Hz | **125 Hz** (relay 500 Hz) | camera_aim_node `rate_hz` |
| Aim force abort | n/a | 15 N | camera_aim_node `abort_force_n` |

### Appendix B: stale or inconsistent docstrings and comments found (code hygiene), and one code defect (item 13)

1. `camera_aim.py:6-7` says "(50 Hz)"; the node runs at 125 Hz (`camera_aim_node.py:105`).
2. `collision.py:20-22` says box-box is tested "after inflating the scene box by the clearance"; no
   inflation is passed (`:271`, C++ `:212`).
3. `config/pen_tool.json` `_comment`: "The planner's FK is the nominal UR5e chain". This has been stale
   since 2026-09-29 (`use_kinematics`).
4. The `kinematics.py` header "UR5e RRT-Connect Planner" is historical. The module is mostly FK, IK and
   the Jacobian now.
5. The `package.xml` `<description>` (Isaac Sim admittance control) and the package name itself are legacy.
6. The `test/test_multiview_touching_parts.py` module docstring says a lap joint is kept apart by
   "ownership plus max_corr <= half the gap". The test body and `RefineConfig` say the rule is off and
   unnecessary.
7. `srv/ComputeTOTG.srv` says path_tolerance "Default ~0.1"; the node defaults to 0.02 rad, and the cell
   passes 0.005.
8. `marking.py:104` says E2 has "34 boxes" and `:283` says "33 boxes".
9. The `marking.aps_path` docstring says the path is "re-checked with the PYTHON model". It uses Python's
   edge sampling with `fast_is_valid` (the C++ model when built) (`:190-191` vs `:228`).
10. The `tack_marking_node.py` docstring says the bias is "re-measured at each approach point"; the code
    measures it right after the transit, at the departure point (`:385`). (Node.)
11. `multiview_refine._thickness_mm` takes the smallest bounding-box extent as "thickness". For a pipe
    that is its diameter or length, not its wall, which affects the ISO limit used in tube-tube fit-up
    warnings.
12. `MarkingConfig` defaults (clearance 10 mm, roll 15°) silently differ from the shipped marking.json
    (3 mm, 30°).
13. **Code defect (not just documentation):** `kinematics.ur5e_jacobian` divides the skew difference
    of $R(q+\varepsilon)R(q-\varepsilon)^{\top}$ by 2ε instead of 4ε, so its angular rows are doubled
    (`:299-303`). This affects the camera aim's achieved rates (halved) and slows IK's orientation
    convergence (results unaffected). See A1 and discrepancy #36.

### Appendix C: test coverage

Covered (test → module): see each entry; summary in §1.

Not covered by an automated test:
- `src/aim_rate_relay.cpp`;
- `src/totg_service_node.cpp` (only a fake service in test_motion);
- `src/jacobian_calculator_node.cpp` (legacy);
- `sam6d_io.py`, `assembly_mesh.py`, `geometry.py`;
- `multiview_service.py` (ROS orchestration);
- `fp_stream.py` beyond the copy-identity test (no protocol round-trip, ROI or ego-motion test).

Tests outside this scope that touch scripts: `test_gesture_latch.py` (the gesture node's latch: the fist
held for refine_pose, the palm quick) and `test_charuco_detection.py` (the capture script's board
detection).

Tests that need a build or ROS: `test_transit_cpp.py` (the built `_transit_cpp`, skipped otherwise) and
`test_motion.py` (rclpy). The README says to run `.venv/bin/python -m pytest test/`; the memory notes
say system python for rclpy tests.

### Appendix D: dated incidents and decisions recorded in the library code (chronological)

These come from docstrings and comments of the modules in scope, plus config comments and a few node
comments where they explain a library parameter. They are thesis material, for Chapter 3 rationale or
for Chapter 6.

- **2026-09-22**: the camera body measured on the bench as a box (pen_tool.json).
- **2026-09-24**: the scan home `home_q` recorded, whose branch signs became the lock (marking.json).
  Pen tip (0, 0, 183.78) mm.
- **2026-09-25**: the ICP tracker on a still part wandered 5 mm std / 24 mm range / 6° (pose_stats), so
  the saved pose became a robust mean. Pen tip (1.35, −0.17, 181.73).
- **2026-09-28**: pendant 4-point TCP (1.07, −1.35, 180.91) mm, cap off (pen_tool.json). (Node: a 1 mm
  press read 8 N on steel, so press_m = 0.)
- **2026-09-29**:
  - nominal vs calibrated kinematics 2.4-4.2 mm / ~0.5°, marks ~4 mm off; calibration everywhere (kinematics);
  - normal gate: the thin plate straddled its own faces, 6.5° lean, root 10 mm off (icp);
  - holder envelope re-derived; the old capsule cleared a 90° T by 1.1 mm (pen_tool.json);
  - a 3 mm-clearance transit skimmed a standing plate and hit it at 9 N, the parts ~10 mm off their
    model, giving the 20 mm transit clearance (marking);
  - wrist_3 came home a turn off (5.42 vs −0.86 rad), giving `unwrap=False` for home (marking).
- **2026-10-01**:
  - TrajectoryExecutor moved into `motion.py`, shared with the multi-view;
  - the F/T zero drifted 7 N after a power cycle, giving re-bias before free moves (motion);
  - the multi-view dry run defaults to false, the user's choice (multiview_service);
  - a plain unwrap put wrist_3 at −7.97 rad, giving `nearest_in_limits` (multiview);
  - the lap-joint finding: no swap even without ownership, so the parallel-gap rule is off (multiview_refine, test).
- **2026-10-02**:
  - table plane −96.1 mm, tilted 1.70° (marking.json);
  - 16 frames per view (was 8);
  - resting contact off, the user's call (it hid a 3 mm gap; when the base was pulled up it differed
    2-3 mm run to run);
  - online self-cal iterated (0.8 mm left after one pass);
  - $R_{\mathrm{scan}}d$ removed from the prior (a fake 8 mm fit-up change);
  - max correction 3 → 4° (an ear correction of 3.07° was refused);
  - 4 real runs over two registrations consistent to 0.4 mm (multiview_refine, multiview_service).
- **2026-10-05**: smoothed transits (spline + TOTG) replaced driving the first RRT path's corners (marking).
- **2026-10-06**:
  - APS: the 2 mm planning margin, after "failed the Python collision re-check" (marking). (Node: a
    front↔back transit 6× shorter than one RRT-Connect.)
  - roll retry: a stroke start 14.5 mm from the centre; the camera body 4 mm into the base plate past
    the point (marking);
  - prior σ 3/8/15 mm changed the result by < 0.5 mm (multiview_service).
- **2026-10-09**:
  - R2's spin "corrected" by 30.8° by ICP, giving swing-twist at save (node, test_pose_stats);
  - S3 on R2 registered 7 mm aside (−5.3..10.5 mm vs 10) showed nothing, giving near-miss reporting (curved_seams);
  - `curved_seams` missing from the install list, so mode A fell back to radius-PCA (test_install_list);
  - SP3 redrawn as a true 5 mm offset (test_curved_registry).
- **2026-10-14**:
  - no table in the collision model until then;
  - departure points at 100 mm (APS 0.5-1.9 m of tip travel became 0.19-0.21 m at 3 s) (marking.json);
  - the Cartesian transit (APS 0.26-1.6 m vs 0.14-0.21 m) (marking);
  - the per-tack Viterbi (approach poses 65-90° apart, 0.9 m swings) (tack_reach);
  - the APS margin capped ("Invalid start") (marking);
  - C++ validity, where Python took 3 of a plan's 3.5 minutes (marking);
  - the APS budget 1 → 3 s (node);
  - C1 registered 8 mm off sideways; C1's save-ICP moved 13.1 mm at RMSE 3.6 mm, with the pen contacts
    ~5.5 mm off (test, node).
- **2026-10-15**:
  - hollow bore in the SDF (E2 tilted 2.6° read the plate 4.3 mm "inside") (part_shapes);
  - stale registry entry (a new 220×220×5.5 plate with the old 256×249×8 entry, no seam) (part_shapes, node);
  - E2's spin 36° off, the mitre 15° off the plate, a 26 mm gap range, so its spin is settled on its cut (curved_seams);
  - the judge's rules (overlap compared with the saved assembly; a pipe judged by its tilt: E2 5.13° =
    5.08 spin + 0.71 tilt) (multiview_refine);
  - max correction 4 → 5° (SP3's 4.5° refused);
  - pen-table an allowed contact under a 5.5 mm plate (tack_reach).
- **2026-10-16**:
  - wrist_3 cable window (−202° and −287°; a 1.5 N false contact 84 mm above the part; E2 within ±151°)
    (tack_reach, marking.json);
  - the overshoot ends at the table (SP3 root tack 4.9 mm over the table, 5 mm overshoot on a 35° pen)
    (marking);
  - the aim relay at 500 Hz (the 50 Hz command steps buzzed the wrist) (aim_rate_relay).

