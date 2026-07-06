# admittance_control

A ROS 2 (Jazzy) package for a **UR5e welding/manipulation cell** that runs
identically against a **real robot + RealSense camera** and an **Isaac Sim
digital twin**. It bundles two cooperating pipelines:

1. **Surface drawing / admittance control** — detect a work surface, generate a
   tool path on it, plan + execute the motion, and (optionally) follow the
   surface under force control.
2. **Perception → 6D pose → ICP refinement** — capture RGB‑D, run SAM‑6D to get
   an object 6D pose + mask, visualize it, then refine that pose with
   point‑to‑plane ICP against the segmented point cloud.

Everything is designed so the **same nodes and topics** work in sim and on
hardware; only the joint‑state source and the camera front‑end differ.

> **New here? Read this file top to bottom.** The [Big picture](#big-picture)
> and [Data flow](#data-flow) sections tell you what connects to what; the
> [Node reference](#node-reference) tells you what each piece does; [Run it](#run-it)
> gives copy‑paste workflows.

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
                          │        └─ sam6d_bridge_node ──HTTP──▶ SAM-6D server (GPU)
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
| `scripts/rgb_depth_to_send/` | Transfer dir: last `rgb.png` / `depth.png` / `camera.json` sent to SAM‑6D |
| `scripts/sam6d_results/` | SAM‑6D outputs: `detection_pem.json`, `detection_ism.npz` |
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
   trigger ▶ sam6d_bridge_node ──HTTP POST rgb+depth+camera──▶ SAM-6D Flask server (server.py, GPU)
                 │  ◀── pose[] + artifacts (detection_pem.json, detection_ism.npz)
                 ▼
        /perception/detections  (vision_msgs/Detection3DArray, camera frame)
                 │                                   │
                 ▼                                   ▼
   detection_marker_node                    icp_pose_refiner_node
   → /perception/detection_markers          Phase 1 (~/run_icp, once): SAM-6D
     (pose triads, labels, and an             mask ∩ cloud → point-to-plane ICP
      oriented CAD wireframe box on            → seed current_pose
      the highest-scoring detection)         Phase 2 (timer, no SAM-6D): dynamic
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
| `sam6d_bridge_node.py` | On `~/trigger`, captures a synced RGB‑D pair, POSTs it to the SAM‑6D server, parses the 6D‑pose reply (`R`, `t` mm→m), and republishes as `Detection3DArray`. Saves server artifacts to `sam6d_results/`. | in: color+depth; srv: `~/trigger`; out: `/perception/detections` (latched) |
| `server.py` | The **SAM‑6D Flask server** (runs on the GPU machine, not the robot). `POST /predict_pose` with rgb/depth/camera → instance seg (FastSAM/SAM) + pose estimation against a fixed CAD model (`CAD_PATH`). Returns poses + artifacts. | HTTP `:5000/predict_pose` |
| `detection_marker_node.py` | Converts `Detection3DArray` → RViz `MarkerArray` (RViz has no native `vision_msgs` display). Draws a pose triad + sphere + label per detection, and an **oriented CAD wireframe box** on the highest‑scoring detection. | in: `/perception/detections`; out: `/perception/detection_markers` |
| `icp_pose_refiner_node.py` | **Real‑time object tracker** (`notes/realtime_icp.md`). *Phase 1* `~/run_icp`: uses the SAM‑6D mask (`detection_ism.npz`) to segment the object, places the CAD `.ply` at the SAM‑6D pose, runs ICP, and seeds `current_pose`. *Phase 2* (timer at `tracking_rate_hz`, no SAM‑6D): builds a **dynamic CropBox** (model AABB + `crop_margin_m`) around `current_pose` to isolate the object, runs **Fast‑ICP** (Anderson‑accelerated point‑to‑plane, `anderson_depth`) from `current_pose`, updates it, and publishes the CropBox as a Marker. Pauses if fitness < `lost_fitness`. `~/stop_tracking` / `~/start_tracking` gate the loop. `scene_from` (init only) = `pointcloud` or `depth_png`. With `use_open3d` (default), the per‑frame voxel downsample + normals run on the *cropped* cloud via Open3D (crop‑first is the key speedup); without Open3D it falls back to NumPy. | in: `/camera/depth/color/points`; srv: `~/run_icp`, `~/stop_tracking`, `~/start_tracking`; out: `/perception/icp/{scene_cloud,model_cloud,refined_pose,crop_box}`, TF `sam6d_object` |
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

## Interfaces

| Interface | Type | Summary |
|---|---|---|
| `execute_drawing` | action `ExecuteDrawing` | Goal: Cartesian `waypoints` + `orientation` (base_link). Feedback: `current_phase`, `drawing_progress`. Result: `success`, `message`. |
| `compute_totg` | service `ComputeTOTG` | Request: flattened joint waypoints + per‑joint vel/accel limits + `path_tolerance` + `resample_dt`. Response: timed positions/velocities + timestamps. |
| `~/trigger` (sam6d_bridge) | `std_srvs/Trigger` | Run one capture → SAM‑6D → publish cycle. |
| `~/run_icp` (icp_pose_refiner) | `std_srvs/Trigger` | Phase‑1 init: segment (mask) + ICP‑refine the best detection, seed the tracker, and (if `auto_track`) start Phase‑2 tracking. |
| `~/stop_tracking` / `~/start_tracking` (icp_pose_refiner) | `std_srvs/Trigger` | Pause / resume the Phase‑2 tracking loop. |
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
| `pointcloud.launch.py` | **Real‑camera** colored point cloud: camera node + depth_image_proc. Args: `launch_camera`, `launch_rviz`. |
| `digital_twin_pointcloud.launch.py` | **Full sim perception stack**: robot_state_publisher (`/isaac_joint_states`) + camera extrinsic TF + sim camera relay + pointcloud + detection markers + (optional) SAM‑6D bridge + (optional) ICP refiner + (optional) RViz. Args: `launch_robot_state`, `launch_rviz`, `extrinsic_parent`, `extrinsic_path`, `launch_sam6d`, `sam6d_server_url`, `detection_min_score`, `bbox_model_path`, `bbox_model_units`, `launch_icp`, `icp_scene_from`. |

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
  launch_sam6d:=true \
  sam6d_server_url:=http://<gpu-host>:5000/predict_pose \
  detection_min_score:=0.1

# In another terminal:
ros2 service call /sam6d_bridge/trigger   std_srvs/srv/Trigger   # RGB-D → SAM-6D → detections
ros2 service call /icp_pose_refiner/run_icp std_srvs/srv/Trigger # Phase-1 seed + start tracking
```

Once `run_icp` returns, the **tracking loop is live**: **move the object in Isaac
Sim** and the CropBox (`/perception/icp/crop_box`) and the green model cloud
follow it, re-aligned by Fast-ICP every frame. Pause/resume or re-seed with:

```bash
ros2 service call /icp_pose_refiner/stop_tracking  std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/start_tracking std_srvs/srv/Trigger
# moved too fast and lost tracking? re-seed:
ros2 service call /sam6d_bridge/trigger     std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/run_icp std_srvs/srv/Trigger
```

RViz displays to add (Fixed Frame `base_link`):
`PointCloud2` on `/camera/depth/color/points`, `MarkerArray` on
`/perception/detection_markers`, `PointCloud2` on
`/perception/icp/{scene_cloud,model_cloud}`, and `Marker` on
`/perception/icp/crop_box` (the CropBox that follows the object).

### Real robot drawing

```bash
ros2 launch admittance_control admittance_control.launch.py real_robot:=true trajectory_key:=circle
```

### Real camera point cloud only

```bash
ros2 launch admittance_control pointcloud.launch.py launch_rviz:=true
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
