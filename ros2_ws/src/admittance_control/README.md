# admittance_control

A ROS 2 (Jazzy) package for a **UR5e welding/manipulation cell** that runs
identically against a **real robot + RealSense camera** and an **Isaac Sim
digital twin**. It bundles two cooperating pipelines:

1. **Surface drawing / admittance control** — detect a work surface, generate a
   tool path on it, plan + execute the motion, and (optionally) follow the
   surface under force control.
2. **Perception → 6D pose → ICP refinement → tracking** — capture RGB‑D, run
   FoundationPose to get an object 6D pose + mask, visualize it, refine that pose with
   point‑to‑plane ICP against the segmented point cloud, then track it live.
3. **Assembly → weld‑seam extraction** — freeze each located part's CAD cloud
   into a static assembly model, then find the joint between two near‑orthogonal
   parts by PCA curvature and publish it as a weld toolpath.

Everything is designed so the **same nodes and topics** work in sim and on
hardware; only the joint‑state source and the camera front‑end differ.

> **New here? Read this file top to bottom.** The [Big picture](#big-picture)
> and [Data flow](#data-flow) sections tell you what connects to what; the
> [Node reference](#node-reference) tells you what each piece does;
> [Algorithms](#algorithms-and-the-maths-behind-them) derives the maths and says
> why each choice was made; [Run it](#run-it) gives copy‑paste workflows.

---

## Big picture

```
                          ┌──────────────────────────────────────────┐
                          │              ROBOT + CONTROL             │
   /joint_states (real)   │  robot_state_publisher  → TF tree        │
   /isaac_joint_states ──▶│  ros2_control controllers (UR5e)         │
                          │  totg_service_node (C++)  time-param.    │
                          │  jacobian_node (C++)      Jacobian/twist │
                          └───────────────▲───────────────▲──────────┘
                                          │               │
          ┌───────────────────────────────┘               └───────────────────┐
          │  DRAWING PIPELINE                                ADMITTANCE          │
          │  drawing_dispatcher                              cartesian_          │
          │    └─action:execute_drawing─▶ drawing_action_    admittance_         │
          │                              server ─srv:compute_ controller         │
          │                              totg─▶ TOTG          (wrench → Servo)   │
          └───────────────────────────────────────────────────────────────────-┘

                          ┌──────────────────────────────────────────┐
                          │              PERCEPTION                   │
   camera (real or sim) ─▶│ realsense_camera_node  /  realsense_sim_  │
                          │ camera_node → /camera/color|depth + info  │
                          │        │                                  │
                          │        ├─ depth_image_proc ─▶ pointcloud  │
                          │        │                                  │
                          │        └─ foundationpose_bridge_node ──HTTP──▶ FoundationPose server (GPU)
                          │             → /perception/detections      │
                          │                  │            │           │
                          │   detection_marker_node   icp_pose_       │
                          │   (RViz markers + box)    refiner_node     │
                          │                           (segmented cloud │
                          │                            + ICP metrics)  │
                          └──────────────────────────────────────────┘
```

The **camera extrinsic** (`camera_extrinsic_tf_publisher`) is the bridge between
the two halves: it publishes the static `tool0 → camera_color_optical_frame`
transform (eye‑in‑hand), so perception results land in the robot's TF tree and
can be commanded to.

---

## Repository layout

| Path | What lives there |
|------|------------------|
| `scripts/*.py` | All Python ROS nodes (installed as package executables) |
| `admittance_control/*.py` | Importable pure‑Python library modules (no rclpy) |
| `src/*.cpp` | C++ nodes (`totg_service_node`, `admittance_control_jacobian_node`) |
| `launch/*.py` | Launch files (see [Launch files](#launch-files)) |
| `srv/`, `action/` | Custom interfaces (`ComputeTOTG.srv`, `ExecuteDrawing.action`) |
| `config/` | UR5e description, controllers, kinematics, joint limits, SRDF |
| `urdf/`, `meshes/` | UR5e robot description |
| `models/*.ply` | CAD models for SAM‑6D / ICP (e.g. `test_objv3.ply`) |
| `world/welding_world.usda` | Isaac Sim scene (robot + eye‑in‑hand RealSense) |
| `notebooks/` | Hand‑eye calibration outputs (`T_tcp_to_cam.npy`), intrinsics, experiments |
| `notes/*.md` | Design notes behind each algorithm (see [Algorithms](#algorithms-and-the-maths-behind-them)) |
| `scripts/rgb_depth_to_send/` | Transfer dir: last `rgb.png` / `depth.png` / `camera.json` sent to the pose server |
| `scripts/foundationpose_results/` | **Live** pose-server outputs (`detection_pem.json`, `detection_ism.npz`, `mask.png`, `vis_pose.png`) **and** assembly state (`static_env.ply/.npy`, `assembly.json`, `welding_points.ply/.npy`). The ICP node's `results_dir` default. |
| `scripts/sam6d_results/` | Frozen SAM‑6D captures from before the FoundationPose swap. Kept for replay only — nothing writes here anymore. |
| `helper/` | Calibration, GUIs, legacy tools (run directly, not installed) |
| `generated_planes/` | Saved work‑surface plane JSONs |

**Build type:** `ament_cmake` with `rosidl` interfaces. The Python library
modules under `admittance_control/` are installed by an explicit `FILES` list in
`CMakeLists.txt` (they share the package dir with the generated action/srv
bindings) — **if you add a module there, add it to that list too**. Node scripts
are installed via `install(PROGRAMS ...)` and **must be `chmod +x`** or `ros2 run`
won't find them.

---

## Data flow

### A. Surface drawing

```
plane JSON ──▶ drawing_dispatcher
                 │  generate primitive (line/triangle/square/circle) on the plane
                 │  validate reachability with the orthogonal tool
                 ▼
        action: execute_drawing (waypoints + orientation, base_link)
                 │
                 ▼
        drawing_action_server   (state machine, all planned in joint space)
          RETRACT_UP → [HOMING] → RRT → DESCENT → DRAWING → ASCENT
          uses admittance_control.kinematics (FK/IK/RRT-Connect/Bezier)
                 │  raw joint waypoints
                 ▼
        service: compute_totg  →  totg_service_node (C++, MoveIt TOTG)
                 │  time-parameterized JointTrajectory
                 ▼
        ros2_control joint_trajectory_controller  →  robot (real or Isaac)
```

### B. Admittance (surface following under force)

```
/force_torque_sensor_broadcaster/wrench ─▶ cartesian_admittance_controller
    X/Y track nominal path, Z = 1-DoF admittance to hold contact force
                 │  TwistStamped
                 ▼
       /servo_node/delta_twist_cmds  →  MoveIt Servo  →  robot
```

### C. Perception → pose → ICP

```
camera ─▶ /camera/color/image_raw ─┐
         /camera/depth/image_rect_raw ─┼─▶ depth_image_proc ─▶ /camera/depth/color/points
         /camera/color/camera_info ─┘                             (organized XYZRGB)
                 │
   trigger ▶ foundationpose_bridge_node ──HTTP POST rgb+depth+camera──▶ FoundationPose server (fp_server.py, GPU)
                 │        (operator clicks the object there; SAM2 turns the click into the mask)
                 │  ◀── pose 4x4 [m] + artifacts (detection_pem.json, detection_ism.npz, mask.png, vis_pose.png)
                 ▼
        /perception/detections  (vision_msgs/Detection3DArray, camera frame)
                 │                                   │
                 ▼                                   ▼
   detection_marker_node                    icp_pose_refiner_node
   → /perception/detection_markers          Phase 1 (~/run_icp, once): the
     (pose triads, labels, and an             mask ∩ cloud → point-to-plane ICP
      oriented CAD wireframe box on            → seed current_pose
      the highest-scoring detection)         Phase 2 (timer, no pose server): dynamic
                                               CropBox around current_pose → Fast-
                                               ICP (Anderson-accel. point-to-plane)
                                               → update pose every frame
                                               → /perception/icp/scene_cloud (white)
                                               → /perception/icp/model_cloud (green)
                                               → /perception/icp/refined_pose + TF
                                               → /perception/icp/crop_box (Marker)
```

The tracking loop is the [`notes/realtime_icp.md`](notes/realtime_icp.md)
methodology: SAM-6D only *initializes* the pose, then a CropBox that follows
`current_pose` replaces the mask and Fast-ICP re-aligns the CAD to the live
cloud every tick. Move the object and the box (and pose) follow it; if ICP
fitness drops below `lost_fitness` tracking pauses for a fresh SAM-6D re-seed.

### D. Assembly → weld seam

Once a part is tracked, `~/save_object` **bakes its CAD cloud at the refined
pose into the SEPC** (Static Environment Point Cloud) — a growing model of
everything already placed, held in `base_link` because the eye-in-hand camera
frame moves. The SEPC does two jobs: it makes ICP blind to already-assembled
parts, and it *is* the geometry the weld seam is extracted from.

```
        ~/run_icp (part A) → track → ~/save_object ─┐
        ~/run_icp (part B) → track → ~/save_object ─┤  (set model_path between parts)
                                                    ▼
                    SEPC = CAD(A) ∪ CAD(B)  in base_link
                    → /perception/icp/static_env  (orange)
                    → sam6d_results/static_env.ply + assembly.json
                                                    │
                                     ~/welding_points│
                                                    ▼
                    radius-PCA curvature → cross-object filter → voxel merge
                    → /perception/icp/welding_points  (red)
                    → sam6d_results/welding_points.ply
```

Every tracking tick, the live crop is transformed into `base_link` and points
within `bg_subtract_dist_m` of the SEPC are deleted, so part B's ICP cannot snap
onto part A when the two are pushed flush ([`notes/model_based_background.md`](notes/model_based_background.md)).
The same transform applies the **ground cut** (`z <= ground_z_m` is the bench;
[`notes/ground_removal.md`](notes/ground_removal.md)).

---

## Node reference

### Robot description & control

| Node (exec) | Lang | Role | Key I/O |
|---|---|---|---|
| `robot_state_publisher` | — | Publishes the UR5e TF tree from joint states | in: `/joint_states` (real) or `/isaac_joint_states` (sim); out: `/tf` |
| `totg_service_node` | C++ | Time‑Optimal Trajectory Generation (MoveIt TOTG, Kunz & Stilman). Turns raw joint waypoints into a timed trajectory. No URDF needed. | srv: `compute_totg` (`ComputeTOTG`) |
| `admittance_control_jacobian_node` | C++ | UR5e Jacobian / twist utilities via MoveIt robot model | in: `/joint_states`; out: twist/Jacobian |
| ros2_control controllers | — | `joint_trajectory_controller`, `force_torque_sensor_broadcaster`, etc. | see `config/ros2_controllers.yaml` |

### Drawing pipeline

| Node (exec) | Role | Key I/O |
|---|---|---|
| `drawing_dispatcher.py` | Loads a plane JSON, generates a geometric primitive on it, checks reachability, and sends it to the action server. `trajectory_key` ∈ {random, line, triangle, square, circle}. | action client: `execute_drawing` |
| `drawing_action_server.py` | Per‑goal state machine (`RETRACT_UP → [HOMING] → RRT → DESCENT → DRAWING → ASCENT`). Pre‑computes the whole motion in joint space (IK, RRT‑Connect, Bézier smoothing) then times it via TOTG. Keeps the tool normal to the surface. | action server: `execute_drawing`; srv client: `compute_totg`; out: JointTrajectory |
| `cartesian_admittance_controller.py` | Surface following: X/Y track the nominal path, Z uses a 1‑DoF admittance model to hold target contact force. | in: `/force_torque_sensor_broadcaster/wrench`; out: `/servo_node/delta_twist_cmds` (MoveIt Servo) |
| `move_to_object_node.py` | Test node: move the pen tip to a standoff in front of the best SAM‑6D detection, approaching along the camera ray. Safety‑gated by a `~/go` service; `dry_run` supported. | in: `/perception/detections`, TF; srv: `~/go`; out: `/welding/target_tip`, `/welding/target_flange`, JointTrajectory |

### Perception & camera

| Node (exec) | Role | Key I/O |
|---|---|---|
| `realsense_camera_node.py` | **Real** RealSense D435i front‑end. Streams color + depth (depth aligned to color), publishes matching `CameraInfo`, writes live intrinsics to `camera.json`. | out: `/camera/color/image_raw`, `/camera/depth/image_rect_raw` (16UC1 mm), `/camera/color/camera_info` |
| `realsense_sim_camera_node.py` | **Sim** front‑end + digital‑twin fixups. Relays Isaac's `*_sim` RGB‑D, adds a realistic depth‑noise model, and (parameters, off by default) **rewrites the empty Isaac `frame_id`** to `camera_color_optical_frame` and **synthesizes `CameraInfo`** (Isaac's graph publishes none). Intrinsics default to the real camera's (the twin shares them). | in: `/camera/color/image_raw_sim`, `/camera/depth/image_rect_raw_sim`; out: same topics as the real node |
| `camera_extrinsic_tf_publisher.py` | Broadcasts the static eye‑in‑hand extrinsic `tool0 → camera_color_optical_frame` from a 4×4 `.npy` (hand‑eye calibration, `notebooks/T_tcp_to_cam.npy`). **This is what puts perception in the robot frame.** | out: static `/tf` |
| `foundationpose_bridge_node.py` | **The pose bridge in use.** On `~/trigger`, captures a synced RGB‑D pair, POSTs it to `fp_server.py`, and republishes the reply as `Detection3DArray`. The pose comes back as a 4×4 **in metres** — no mm→m division, unlike SAM‑6D. Saves server artifacts to `foundationpose_results/`. A trigger **blocks until the operator clicks the object** in the window the server opens (hence `request_timeout_sec` = 300). | in: color+depth; srv: `~/trigger`; out: `/perception/detections` (latched) |
| `fp_server.py` | The **FoundationPose Flask server** (GPU machine, not the robot). `POST /predict_pose` with rgb/depth/camera → SAM2 mask from a click → `register()` against a fixed CAD model (`MESH_PATH`). Has **no detector of its own**: send a `mask` file or a `click` field to skip the window. Re‑emits the result in SAM‑6D's on‑disk format so the ICP node needs no changes. | HTTP `:5000/predict_pose` |
| `sam6d_bridge_node.py` | *Superseded by the FoundationPose bridge.* Same trigger→POST→publish cycle against `server.py`, parsing a list of detections (`R`, `t` mm→m). Still installed so an old SAM‑6D setup can be run for comparison. | in: color+depth; srv: `~/trigger`; out: `/perception/detections` (latched) |
| `server.py` | *Superseded by `fp_server.py`.* The SAM‑6D Flask server: instance seg (FastSAM/SAM) + pose estimation against `CAD_PATH`, re‑running `demo.sh` per request. | HTTP `:5000/predict_pose` |
| `detection_marker_node.py` | Converts `Detection3DArray` → RViz `MarkerArray` (RViz has no native `vision_msgs` display). Draws a pose triad + sphere + label per detection, and an **oriented CAD wireframe box** on the highest‑scoring detection. | in: `/perception/detections`; out: `/perception/detection_markers` |
| `icp_pose_refiner_node.py` | **Real‑time object tracker + assembly/weld model** (`notes/realtime_icp.md`). *Phase 1* `~/run_icp`: uses the SAM‑6D mask (`detection_ism.npz`) to segment the object, places the CAD `.ply` at the SAM‑6D pose, runs ICP, and seeds `current_pose`. *Phase 2* (timer at `tracking_rate_hz`, no SAM‑6D): builds a **dynamic CropBox** (model AABB + `crop_margin_m`) around `current_pose` to isolate the object, applies **ground removal** + **model‑based background subtraction**, runs **Fast‑ICP** (Anderson‑accelerated, Welsch‑robust point‑to‑plane) from `current_pose`, updates it, and publishes the CropBox as a Marker. Pauses if fitness < `lost_fitness`. *Assembly* `~/save_object`: bakes the CAD at its refined pose into the **SEPC** in `static_frame`, persists it, clears tracking for the next part. *Weld* `~/welding_points`: extracts the joint between the SEPC's parts and publishes it red. `scene_from` (init only) = `pointcloud` or `depth_png`. With `use_open3d` (default), the per‑frame voxel downsample + normals run on the *cropped* cloud via Open3D (crop‑first is the key speedup); without Open3D it falls back to NumPy. | in: `/camera/depth/color/points`, TF; srv: `~/run_icp`, `~/stop_tracking`, `~/start_tracking`, `~/save_object`, `~/welding_points`; out: `/perception/icp/{scene_cloud,model_cloud,refined_pose,crop_box,static_env,welding_points}`, TF `sam6d_object` |
| `depth_image_proc::PointCloudXyzrgbNode` | Standard package node (launched, not in this repo). Fuses color + registered depth + `camera_info` into an organized `PointCloud2`. | out: `/camera/depth/color/points` |

---

## Shared library modules (`admittance_control/`)

Pure Python, no rclpy — unit‑testable without ROS.

| Module | Contents |
|---|---|
| `geometry.py` | `rotmat_to_quat`, `quat_to_rotmat` |
| `kinematics.py` | UR5e FK/Jacobian, damped‑least‑squares `ik_solve`, `rrt_connect`, path smoothing (`bezier_smooth_path`, Catmull‑Rom), `plan_to_pose`. Used by the drawing action server. |
| `sam6d_io.py` | Image/HTTP/JSON plumbing for the SAM‑6D bridge: `resolve_transfer_dir`, image normalization, PNG encode, multipart POST, `parse_pem_response`, `save_artifacts`. |
| `icp.py` | NumPy ICP stack: `load_ply_mesh`, `sample_mesh_surface`, `backproject_depth`, `estimate_normals_organized` (organized‑cloud normals, no KD‑tree), `voxel_downsample`, `crop_box_mask` (oriented CropBox for tracking), `nearest_neighbor` (uses `scipy.spatial.cKDTree` if SciPy is present — ~15× faster — else brute force), `icp_point_to_plane` (point‑to‑plane; `anderson_depth>0` enables **Fast‑ICP** — se(3)‑parameterized Anderson acceleration with a monotone‑energy safeguard). Runs pure‑NumPy on the robot side; SciPy/Open3D are optional accelerators. |

---

## Algorithms and the maths behind them

Everything below is implemented in `admittance_control/icp.py` and
`admittance_control/kinematics.py` (pure NumPy) or in the nodes that call them.
Units are **metres and radians** unless stated. FoundationPose returns metres
directly (the server scales the CAD on load), so the bridge does **no** unit
conversion. The one place mm survives is `detection_pem.json`, which keeps
SAM‑6D's mm convention on purpose so the ICP node — which divides by 1000 — reads
both backends' captures unchanged.

### 1. Frames: how a pixel becomes a robot target

Perception happens in the camera's optical frame, but the robot only understands
`base_link`. Three transforms chain them:

```
T_base←cam(t) = T_base←tool0(q(t)) · T_tool0←cam
T_base←obj(t) = T_base←cam(t) · T_cam←obj
```

`T_base←tool0` comes from forward kinematics of the measured joint angles `q(t)`
(published by `robot_state_publisher`). `T_tool0←cam` is the **fixed hand‑eye
extrinsic**, calibrated offline into `notebooks/T_tcp_to_cam.npy`. `T_cam←obj` is
what SAM‑6D estimates and ICP refines.

The camera is **eye‑in‑hand**, so `T_base←cam` changes every time the arm moves.
This single fact drives three design decisions: the SEPC is stored in
`base_link` (a static frame) rather than the camera frame; TF lookups use the
*cloud's own timestamp*, not "now", so the pose matches where the camera was at
capture; and the SAM‑6D mask is only valid for the frame it was captured from.

### 2. Depth image → organized point cloud → normals

Back‑projection inverts the pinhole model. For pixel `(u,v)` with depth `z`:

```
x = (u − cx)·z / fx      y = (v − cy)·z / fz      point = (x, y, z)
```

Invalid pixels (`z ≤ 0`) become `NaN` so they survive as holes rather than
collapsing to the origin. Keeping the cloud **organized** (an `H×W×3` array, not
a flat list) is what lets normals be computed without a KD‑tree: neighbouring
array elements are neighbouring points in space, so central differences over the
pixel grid give two surface tangents, and their cross product is the normal.

```
∂X/∂u ≈ X[v, u+1] − X[v, u−1]
∂X/∂v ≈ X[v+1, u] − X[v−1, u]
n     = normalize(∂X/∂u × ∂X/∂v),   flipped so n·X < 0  (faces the camera)
```

This is O(N) with no neighbour search — the reason `depth_image_proc` is asked
for an *organized* cloud and why `icp_pose_refiner_node` rejects clouds with
`height ≤ 1`.

### 3. Voxel downsampling

Points are bucketed by `⌊p / voxel⌋` and each bucket is replaced by its
**centroid** (not by a representative point). Averaging is what makes the same
routine usable for the weld‑seam midline trick in §8. Normals carried along as
`extra` are averaged then renormalized.

### 4. Point‑to‑plane ICP

Given model points `pᵢ` and, for each, the nearest scene point `qᵢ` with unit
normal `nᵢ`, ICP minimizes the squared distance **to the tangent plane** rather
than to the point:

```
E(R, t) = Σᵢ ( ((R·pᵢ + t) − qᵢ) · nᵢ )²
```

Point‑to‑plane converges far faster than point‑to‑point because it lets the
model slide freely *along* a surface and only penalizes motion *through* it —
exactly the freedom you want when two sampled surfaces don't have corresponding
samples.

It is solved by **Gauss‑Newton under a small‑angle approximation**. Writing the
incremental rotation as `R ≈ I + [ω]ₓ` linearizes the residual to

```
rᵢ(x) ≈ (pᵢ − qᵢ)·nᵢ + x·[ pᵢ × nᵢ ; nᵢ ],     x = [ω(3), t(3)] ∈ ℝ⁶
```

so each correspondence contributes one row `Aᵢ = [pᵢ × nᵢ, nᵢ]` and one
right‑hand side `bᵢ = −(pᵢ − qᵢ)·nᵢ`, and the 6‑DoF update is the least‑squares
solution of `A x = b` (`np.linalg.lstsq`). The exact rotation is recovered from
`ω` via **Rodrigues**, `R = I + sin θ·K + (1 − cos θ)·K²` with `K = [ω̂]ₓ`, and
applied on the left: `T ← ΔT · T`. Correspondences beyond `max_corr_dist` are
dropped as outliers.

Reported metrics: `fitness = n_inliers / n_model` (what fraction of the CAD
found support — this is the number `lost_fitness` gates on) and `inlier_rmse`
(the RMS point‑to‑plane residual over inliers).

### 5. Fast‑ICP: Anderson acceleration on SE(3)

Plain ICP is a **fixed‑point iteration** `xₖ₊₁ = G(xₖ)` on the pose, and it
converges linearly — often needing tens of iterations. Anderson Acceleration
(AA) reuses the last `m` iterates to extrapolate a better next one, turning
linear convergence into something closer to quasi‑Newton, at negligible cost.

The subtlety: AA needs a **vector space**, and `SE(3)` is a manifold — you cannot
linearly combine 4×4 transforms. So the pose is parameterized as an increment
relative to the initial guess, in the **Lie algebra** `se(3)`:

```
ξ = log(T · T₀⁻¹) ∈ ℝ⁶,      T = exp(ξ) · T₀
```

Keeping `ξ` near the origin keeps `log` well‑conditioned. AA then solves a tiny
least‑squares problem for the mixing coefficients `θ` over the residual history
`F = G(x) − x`,

```
θ* = argmin ‖ Fₖ − ΔF·θ ‖²,       x_accel = G(xₖ) − ΔG·θ*
```

and the safeguard makes it robust: the extrapolated iterate is **accepted only
if the ICP energy did not increase**, otherwise the code falls back to one plain
step and resets the history. That guarantees monotone energy decrease — AA can
only help. `anderson_depth = 0` disables it (baseline Gauss‑Newton); `5` is the
default.

### 6. Robust ICP: the Welsch kernel with dynamic ν

A hard `max_corr_dist` cutoff is a *binary* outlier decision, and it makes the
energy discontinuous — a gripper or fixture drifting into the crop box flips
points in and out and the fit jitters. `robust=true` replaces it with the smooth
**Welsch** loss on the signed point‑to‑plane residual `rᵢ`:

```
ψ_ν(r) = 1 − exp(−r² / 2ν²)          E = Σᵢ ψ_ν(rᵢ)
```

Its IRLS weight is `wᵢ = exp(−rᵢ² / 2ν²)`: a point 3ν away contributes ~1% of
its weight, so outliers *fade out* instead of being cut off. Each iteration
solves the same normal equations as §4, but with rows scaled by `√wᵢ`. This
weighted system is the **MM (majorize‑minimize) surrogate** of the Welsch
energy, so decreasing it provably decreases `E`.

The bandwidth `ν` is what decides "how far is an outlier", and the right answer
changes during convergence — so it is **annealed** rather than fixed:

```
ν_begin = 3.0 · median(|r|)                 (loose: tolerate the initial misalignment)
ν      ← 0.5 · ν  after each stage           (tighten)
ν_end   = 1/(3√3) · noise_floor(target)      (stop at the sensor's own noise)
```

`noise_floor` is measured from the data (`welsch_nu_end`): for each scene point,
take its 7 nearest neighbours, measure how far they lie **along that point's own
normal** (the local surface "thickness"), and take the median of medians. Below
that scale, residuals are sensor noise, not misalignment, and shrinking `ν`
further would start rejecting valid points. Anderson acceleration (§5) runs
inside each fixed‑`ν` stage.

This is a NumPy port of Zhang et al., *Fast and Robust Iterative Closest Point*
(the `yaoyx689/Fast-Robust-ICP` reference implementation).

### 7. Tracking: three filters replace the mask

SAM‑6D is expensive (a GPU round trip), so it runs **once**, to initialize. Every
subsequent frame the object is isolated geometrically instead
([`notes/realtime_icp.md`](notes/realtime_icp.md)):

1. **Oriented CropBox** — the model's AABB `[lo, hi]` grown by `crop_margin_m`,
   placed at the current pose. A point is kept iff `lo ≤ Rᵀ(p − t) ≤ hi`, i.e.
   tested in the *box's own frame*, so it rotates with the object.
2. **Ground removal** — transform the crop into `base_link` and drop `z ≤
   ground_z_m`. A hard Z‑cut, not RANSAC: the bench is a known constant and
   plane‑fitting can latch onto the workpiece instead.
3. **Model‑based background subtraction** — drop points within
   `bg_subtract_dist_m` of the SEPC (KD‑tree query). ICP literally cannot see
   already‑assembled parts, so part B won't snap onto part A when pushed flush.

The survivors get voxel‑downsampled, normals estimated (Open3D on the *cropped*
cloud — cropping first is the whole speedup), and fed to Fast‑ICP seeded from
the previous pose. If `fitness < lost_fitness` the object is presumed lost and
tracking pauses for a fresh SAM‑6D seed.

### 8. Weld‑seam extraction (`~/welding_points`)

Full derivation in [`notes/welding_edge_sampling.md`](notes/welding_edge_sampling.md).
Input is the SEPC — the CAD clouds of two near‑orthogonal parts separated by a
small physical gap. Output is the line where they meet.

**Surface variation via PCA.** For each point, take all neighbours within radius
`R`, form the covariance of the centred neighbourhood, and take its eigenvalues
`λ₁ ≥ λ₂ ≥ λ₃`:

```
C = (1/N) Σⱼ (pⱼ − p̄)(pⱼ − p̄)ᵀ ,        V = λ₃ / (λ₁ + λ₂ + λ₃)
```

`V` is the fraction of variance normal to the best‑fit local plane. On a flat
face `λ₃ ≈ 0` so `V ≈ 0`; at a 90° fold the neighbourhood spans two planes and
`V` spikes to 0.05–0.15.

**Why radius search, not k‑NN.** The parts don't touch — there's a ~1 mm gap. A
k‑NN neighbourhood on a point at the edge of part A contains only part‑A points,
so it looks flat. A **radius** ball with `R` larger than the gap reaches *across*
it and pulls in part‑B points, and the covariance sees both planes. This is the
one substitution that makes the whole method work.

**`R` is bounded on both sides**, which the note doesn't say and the code
comments now do:

```
gap + point_spacing  <  R  <  part_thickness
```

Too small and the ball never crosses the gap. Too large and it bridges a plate's
*own two faces* — on our 8.4 mm‑thick T‑joint, `R = 10 mm` gives a median `V` of
0.15 across the **entire** cloud, i.e. everything looks like an edge. Measured
gap 1.1 mm, SEPC spacing 2.7 mm, thickness 8.4 mm ⇒ usable window ≈ 4–8 mm,
default `weld_radius_m = 6 mm`.

**Curvature alone is not enough.** Each plate's own outer border is also a 90°
fold, so it fires too — at `R = 6 mm` it accounted for *half* the high‑`V`
points, and the result was a 266×211×134 mm blob rather than a seam. The fix
(`weld_require_cross_object`, default on) keeps an edge point only if some point
of a **different object** lies within the same radius `R`. Object membership is
free: the SEPC is a stack of per‑object clouds and `assembly.json` records each
part's point count. That single condition is true only along the joint, and it
collapses the blob to a 232×16 mm line.

**Double‑line artifact.** Thresholding returns two roughly parallel edge lines —
one on each part, either side of the gap. A voxel grid with `weld_voxel_m >` gap
averages a pair straddling the joint into a single point in the middle of it
(§3 averages, which is why this works). *Honest caveat:* on our T‑joint the
surviving band is ~16 mm wide rather than two thin lines, because points on both
plates within `R` of each other all survive; the voxel merge narrows it only to
~15 mm. For a true single‑line toolpath, fit a line/spline through the band and
project onto it. The published seam is the band, not a centreline.

### 9. Kinematics (drawing pipeline)

- **FK** — fixed UR5e DH/URDF chain of `Rz·trans·Rx` factors, `T = Π Tᵢ(qᵢ)`.
- **Jacobian** — computed by **central finite differences** on FK
  (`J[:,i] = (FK(q+εeᵢ) − FK(q−εeᵢ)) / 2ε`, angular part read out of the skew
  part of `ΔR`). Numerically simple and fast enough at 6 joints; no analytic
  derivation to keep in sync with the URDF.
- **IK** — damped least squares (Levenberg‑Marquardt):

  ```
  Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ · e
  ```

  with `e` the 6‑vector pose error (translation + angle‑axis of `R_target R_curᵀ`)
  and `λ = ik_damping`. The damping term is what keeps `Δq` bounded near
  singularities, where `J Jᵀ` becomes ill‑conditioned and an undamped pseudo‑
  inverse would command enormous joint velocities. Steps are clipped to 0.3 rad
  and multiple random seeds (`ik_num_seeds`) are tried for a solution inside the
  joint limits.
- **RRT‑Connect** — bidirectional tree search in joint space, weighted‑L2
  distance metric, greedy `_steer` extension; used for the collision‑free
  transit between drawing strokes.
- **Smoothing** — Bézier / Catmull‑Rom resampling of the RRT polyline, so the
  path has continuous curvature before timing.
- **TOTG** — `totg_service_node` (C++) applies MoveIt's Time‑Optimal Trajectory
  Generation (Kunz & Stilman 2012): it fits the path with circular blends at the
  waypoints and integrates the velocity profile forward/backward against the
  joint velocity *and* acceleration limits, yielding the fastest timing that
  respects both. The Python side only ever produces geometry; time comes from here.

### 10. Cartesian admittance (surface following)

X/Y track the nominal path; **Z is a 1‑DoF mass‑spring‑damper driven by force
error**, integrated at `control_hz`:

```
z̈ = ( (f_target − f_measured) − D·ż − K·z ) / M
ż ← clamp(ż + z̈·dt),      z ← clamp(z + ż·dt)
```

with `M = admittance_mass_z`, `D = admittance_damping_z`, `K =
admittance_stiffness_z` (default `0`, i.e. pure damping — the tool holds a force
rather than a position). The offset `z` is added to the path setpoint, and the
resulting Cartesian error is emitted as a `TwistStamped` to MoveIt Servo. Low `M`
and high `D` give a soft, non‑oscillatory contact; `K = 0` means a steady force
error integrates into a steady offset instead of fighting the surface.

### 11. Simulated depth noise

The twin would otherwise hand ICP a perfect cloud and flatter every threshold.
`realsense_sim_camera_node` adds zero‑mean Gaussian depth noise with the
**quadratic range dependence** of real stereo depth:

```
σ(z) = depth_noise_std_m + depth_noise_z2_coeff · z²
```

Stereo triangulation error grows with the square of range (disparity resolution
is constant in pixels), so this shape — not constant σ — is what makes sim
thresholds transfer to hardware.

---

## Interfaces

| Interface | Type | Summary |
|---|---|---|
| `execute_drawing` | action `ExecuteDrawing` | Goal: Cartesian `waypoints` + `orientation` (base_link). Feedback: `current_phase`, `drawing_progress`. Result: `success`, `message`. |
| `compute_totg` | service `ComputeTOTG` | Request: flattened joint waypoints + per‑joint vel/accel limits + `path_tolerance` + `resample_dt`. Response: timed positions/velocities + timestamps. |
| `~/trigger` (foundationpose_bridge) | `std_srvs/Trigger` | Run one capture → FoundationPose → publish cycle. Blocks until the object is clicked on the server host. |
| `~/run_icp` (icp_pose_refiner) | `std_srvs/Trigger` | Phase‑1 init: segment (mask) + ICP‑refine the best detection, seed the tracker, and (if `auto_track`) start Phase‑2 tracking. |
| `~/stop_tracking` / `~/start_tracking` (icp_pose_refiner) | `std_srvs/Trigger` | Pause / resume the Phase‑2 tracking loop. |
| `~/save_object` (icp_pose_refiner) | `std_srvs/Trigger` | Freeze the tracked CAD at its refined pose into the SEPC; persist `static_env.*` + `assembly.json`; clear tracking for the next part. |
| `~/welding_points` (icp_pose_refiner) | `std_srvs/Trigger` | Extract the weld seam from the SEPC, publish it red, persist `welding_points.*`. Needs ≥ 2 saved objects. |
| `~/go` (move_to_object) | `std_srvs/Trigger` | Execute the standoff move (safety gate). |

### Key topics

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/joint_states` / `/isaac_joint_states` | `sensor_msgs/JointState` | robot/Isaac → robot_state_publisher |
| `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/color/camera_info` | `Image`/`CameraInfo` | camera node → pointcloud + bridge |
| `/camera/depth/color/points` | `PointCloud2` | depth_image_proc → ICP / RViz |
| `/perception/detections` | `vision_msgs/Detection3DArray` | sam6d_bridge → markers / ICP / move_to_object |
| `/perception/detection_markers` | `visualization_msgs/MarkerArray` | detection_marker → RViz |
| `/perception/icp/{scene_cloud,model_cloud}` | `PointCloud2` | icp_pose_refiner → RViz |
| `/perception/icp/crop_box` | `visualization_msgs/Marker` | icp_pose_refiner → RViz (dynamic CropBox wireframe) |
| `/perception/icp/static_env` | `PointCloud2` | icp_pose_refiner → RViz (orange SEPC, `base_link`) |
| `/perception/icp/welding_points` | `PointCloud2` | icp_pose_refiner → RViz (red weld seam, `base_link`) |
| `/perception/icp/refined_pose` | `PoseStamped` | icp_pose_refiner → RViz / downstream |
| `/force_torque_sensor_broadcaster/wrench` | `WrenchStamped` | FT broadcaster → admittance controller |
| `/servo_node/delta_twist_cmds` | `TwistStamped` | admittance controller → MoveIt Servo |

---

## Coordinate frames & TF tree

```
world (sim) ─ base_link ─ … ─ wrist_3_link ─ flange ─ tool0
                                                         └─(static, T_tcp_to_cam.npy)─▶ camera_color_optical_frame
                                                                                          └─(ICP)─▶ sam6d_object
```

- Robot links come from the URDF via `robot_state_publisher`.
- `tool0 → camera_color_optical_frame` is the **eye‑in‑hand extrinsic**, published
  by `camera_extrinsic_tf_publisher` from the hand‑eye calibration `.npy`.
- `camera_color_optical_frame → sam6d_object` is the **ICP‑refined object pose**.
- SAM‑6D poses/points are in the **color optical frame** (x right, y down, z fwd),
  translations in **mm** (converted to metres in the ROS nodes).

---

## Real robot vs digital twin

| | Real | Digital twin (Isaac Sim) |
|---|---|---|
| Joint states | `/joint_states` | `/isaac_joint_states` |
| Camera front‑end | `realsense_camera_node.py` | `realsense_sim_camera_node.py` (relay + noise + frame/CameraInfo fixups) |
| `frame_id` | set by driver | Isaac leaves it empty → rewritten to `camera_color_optical_frame` |
| `camera_info` | published by driver | **synthesized** by the sim node (Isaac has no CameraInfo publisher) |
| Intrinsics | live from device | same values as the real camera (twin shares them) |
| Scene | physical cell | `world/welding_world.usda` (camera mounted on `flange`) |

The perception + drawing nodes are **identical** across both; only the two rows
above differ. This is why the sim is a true digital twin.

---

## Launch files

| File | Brings up |
|---|---|
| `admittance_control.launch.py` | The **drawing stack**: robot_state_publisher + drawing_action_server + TOTG + drawing_dispatcher. Args: `real_robot`, `use_sim_time`, `trajectory_key`, `continuous`, `plane_json`, `approach_height`, `surface_z_offset`, `orthogonal_tool_length_m`, `max_joint_speed_deg`, `max_joint_accel_deg`. |
| `pointcloud.launch.py` | **Full real‑hardware perception stack**: robot_state_publisher (`/joint_states`) + camera extrinsic TF + `realsense_camera_node` + pointcloud + detection markers + (optional) FoundationPose bridge + ICP refiner + (optional) RViz. Everything runs on the wall clock (`use_sim_time:=false`). Args: as the twin below, plus `launch_camera`, `camera_serial_no`, `camera_width`, `camera_height`, `camera_fps`. |
| `digital_twin_pointcloud.launch.py` | **Full sim perception stack**: robot_state_publisher (`/isaac_joint_states`) + camera extrinsic TF + sim camera relay + pointcloud + detection markers + (optional) FoundationPose bridge + ICP refiner + (optional) RViz, all with `use_sim_time:=true`. Args: `launch_robot_state`, `launch_rviz`, `extrinsic_parent`, `extrinsic_path`, `launch_foundationpose`, `foundationpose_server_url`, `detection_min_score`, `bbox_model_path`, `bbox_model_units`, `launch_icp`, `icp_model_path`, `icp_scene_from`, `icp_anderson_depth`, `icp_use_open3d`, `icp_crop_margin_m`, `icp_tracking_rate_hz`, `icp_auto_track`, `icp_ground_removal`, `icp_ground_z_m`. |

The two are **the same node graph on the same topics**, so an RViz config, a
service call, or a downstream subscriber written against one works unchanged on
the other. The only differences are the joint‑state topic, the camera front‑end,
and the clock.

Requires `ros-jazzy-depth-image-proc` for the point‑cloud node:
`sudo apt install ros-jazzy-depth-image-proc`.

---

## Run it

### Build

```bash
cd ros2_ws
colcon build --packages-select admittance_control --symlink-install
source install/setup.bash
```

### Digital‑twin perception + ICP (with RViz)

```bash
# Isaac Sim playing welding_world.usda (publishes *_sim topics + /isaac_joint_states)
ros2 launch admittance_control digital_twin_pointcloud.launch.py \
  launch_rviz:=true \
  launch_foundationpose:=true \
  foundationpose_server_url:=http://<gpu-host>:5000/predict_pose \
  detection_min_score:=0.1

# In another terminal:
ros2 service call /foundationpose_bridge/trigger   std_srvs/srv/Trigger   # RGB-D → FoundationPose (click the object on the GPU host) → detections
ros2 service call /icp_pose_refiner/run_icp std_srvs/srv/Trigger # Phase-1 seed + start tracking
```

Once `run_icp` returns, the **tracking loop is live**: **move the object in Isaac
Sim** and the CropBox (`/perception/icp/crop_box`) and the green model cloud
follow it, re-aligned by Fast-ICP every frame. Pause/resume or re-seed with:

```bash
ros2 service call /icp_pose_refiner/stop_tracking  std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/start_tracking std_srvs/srv/Trigger
# moved too fast and lost tracking? re-seed:
ros2 service call /foundationpose_bridge/trigger     std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/run_icp std_srvs/srv/Trigger
```

RViz displays to add (Fixed Frame `base_link`):
`PointCloud2` on `/camera/depth/color/points`, `MarkerArray` on
`/perception/detection_markers`, `PointCloud2` on
`/perception/icp/{scene_cloud,model_cloud}`, `Marker` on
`/perception/icp/crop_box` (the CropBox that follows the object), and
`PointCloud2` on `/perception/icp/{static_env,welding_points}` (orange assembly,
red seam). All are latched, so adding a display after the fact still shows data.

### Real robot + real camera (same stack)

```bash
ros2 launch admittance_control pointcloud.launch.py \
  launch_rviz:=true \
  launch_foundationpose:=true \
  foundationpose_server_url:=http://<gpu-host>:5000/predict_pose

# the bench is not the sim floor — tune the ground cut live:
ros2 param set /icp_pose_refiner ground_z_m -0.10
```

The service calls below are identical in sim and on hardware.

### Assemble two parts, then extract the weld seam

```bash
# --- part A ---
ros2 param set /icp_pose_refiner model_path <ws>/src/admittance_control/models/test_objv2_base.ply
ros2 service call /foundationpose_bridge/trigger        std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/run_icp    std_srvs/srv/Trigger   # seed + track
ros2 service call /icp_pose_refiner/save_object std_srvs/srv/Trigger  # freeze into the SEPC

# --- part B (place it against A, then locate it) ---
ros2 param set /icp_pose_refiner model_path <ws>/src/admittance_control/models/test_objv2_ear.ply
ros2 service call /foundationpose_bridge/trigger        std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/run_icp    std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/save_object std_srvs/srv/Trigger

# --- the joint between them, in red ---
ros2 service call /icp_pose_refiner/welding_points std_srvs/srv/Trigger
# → "99 welding points from 314 edge points (of 5000 SEPC points)"
```

`model_path` on the ROS side must name the **same CAD** the pose server has as
`MESH_PATH`, per part. The seam is written to `foundationpose_results/welding_points.ply`
and published on `/perception/icp/welding_points` in `base_link`.

`welding_points` reloads `static_env.npy` + `assembly.json` from `save_dir` if
the node was restarted, so you can re‑run the extraction (with different `R` or
threshold) without re‑locating the parts:

```bash
ros2 param set /icp_pose_refiner weld_radius_m 0.006          # must exceed the gap...
ros2 param set /icp_pose_refiner weld_curvature_thresh 0.03   # ...and stay under part thickness
ros2 service call /icp_pose_refiner/welding_points std_srvs/srv/Trigger
```

### Real robot drawing

```bash
ros2 launch admittance_control admittance_control.launch.py real_robot:=true trajectory_key:=circle
```

### Real camera point cloud only

```bash
ros2 launch admittance_control pointcloud.launch.py launch_rviz:=true launch_icp:=false
```

---

## Extending the project — where to start

- **New perception consumer?** Subscribe to `/perception/detections` (latched) or
  `/perception/icp/refined_pose`. Both are in `camera_color_optical_frame`; use TF
  to get `base_link`.
- **Different object?** Put its CAD `.ply` in `models/`, set `CAD_PATH` on the
  SAM‑6D server to match, and pass `bbox_model_path` / ICP `model_path`. The ROS‑side
  model and the server's CAD model **must be the same object**.
- **New Python library code?** Add the file under `admittance_control/` **and** to the
  `install(FILES …)` list in `CMakeLists.txt`.
- **New node script?** Put it in `scripts/`, `chmod +x`, and add it to the
  `install(PROGRAMS …)` list.
- **Tune ICP?** Params on `icp_pose_refiner_node`: `voxel_size_m`, `max_corr_dist_m`,
  `max_iter`, `n_model_points`, `max_target_points`, `detection_index`, and
  `anderson_depth` (Fast-ICP acceleration; 0 = plain point-to-plane). Logic is in
  `admittance_control/icp.py` (pure NumPy — testable offline against
  `sam6d_results/` + a saved `depth.png`).
- **Tune tracking?** Params on `icp_pose_refiner_node`: `crop_margin_m` (CropBox
  slack around the model AABB — grow it if fast motion drops the object out of the
  box), `tracking_rate_hz`, `lost_fitness` (re-seed threshold), `min_scene_points`,
  `auto_track`, `use_open3d`. Launch args: `icp_anderson_depth`, `icp_use_open3d`,
  `icp_crop_margin_m`, `icp_tracking_rate_hz`, `icp_auto_track`.
- **Tracking too slow?** The tick cost is *crop → downsample+normals → ICP*. The
  biggest levers: install **Open3D** and **SciPy** (`use_open3d` + KD‑tree NN take
  a tick from ~190 ms to ~65 ms, ≈15 Hz); raise `voxel_size_m` (fewer scene points
  → faster normals *and* ICP); lower `n_model_points`; lower `max_iter` (Fast‑ICP
  converges in a few iterations). `pip install open3d` pulls in SciPy too.
- **Tune the weld seam?** `weld_radius_m` first — it must satisfy `gap +
  spacing < R < part_thickness` (§8), and nothing else matters until it does.
  Then `weld_curvature_thresh` (lower ⇒ longer but noisier seam),
  `weld_voxel_m`, `weld_min_neighbors`, `weld_require_cross_object`. All are
  read at call time, so `ros2 param set` then re‑call `~/welding_points`.
- **Tune the assembly filters?** `ground_z_m` (bench height in `base_link`) and
  `bg_subtract_dist_m` (how close a live point must be to the SEPC to be
  considered "already assembled"). Both are live‑settable via `ros2 param set`.
- **New drawing shape?** Extend the primitive generation in `drawing_dispatcher.py`;
  the action server + TOTG handle any waypoint list.
- **Recalibrate the camera?** Regenerate `notebooks/T_tcp_to_cam.npy` (hand‑eye);
  `camera_extrinsic_tf_publisher` picks it up as the `tool0 → camera` extrinsic.

---

## Gotchas learned the hard way

- **Node not found by `ros2 run`?** The script is missing its execute bit
  (`chmod +x scripts/your_node.py`) or isn't in `install(PROGRAMS …)`.
- **`ModuleNotFoundError: admittance_control.<x>`?** Add the module to the
  `install(FILES …)` list — this package does *not* use
  `ament_python_install_package()`.
- **RViz shows nothing for a point cloud / detection?** Check the **Fixed Frame**
  resolves through TF to the cloud's `frame_id`, and that `camera_info` is actually
  being published (in sim it must be synthesized by the sim camera node).
- **ICP mask/cloud mismatch?** The SAM‑6D mask and the point cloud must be the
  **same frame**. For a moving eye‑in‑hand camera, keep the scene static between the
  SAM‑6D trigger and the ICP trigger, or use `icp_scene_from:=depth_png`.
- **`welding_points` says "no points above curvature"?** `weld_radius_m` is
  smaller than the gap, so no PCA ball ever spans it. Raise it — but not past the
  part thickness, or the opposite failure appears: *every* point looks curved
  because the ball bridges a plate's own two faces.
- **The seam is a blob, not a line?** `weld_require_cross_object` got turned off.
  Curvature alone also fires on each part's outer border, which is a 90° fold too.
- **`welding_points` says fewer than two objects?** It needs `assembly.json` to
  agree with `static_env.npy`. Call `~/save_object` once per part; a single saved
  part has no joint to find.
- **Second part's ICP snaps onto the first?** That's what `bg_subtract` prevents —
  check the SEPC actually contains part A (`/perception/icp/static_env` should show
  it in orange) and that TF `base_link ← camera` resolves.
- **Sim thresholds don't transfer to hardware?** Check `ground_z_m` first (bench ≠
  sim floor), then remember real depth noise grows as `z²` (§11) — a threshold
  tuned at 0.4 m is ~4× tighter than the same threshold at 0.8 m.
