# admittance_control

A ROS 2 (Jazzy) package for a **UR5e welding/manipulation cell** that runs
identically against a **real robot + RealSense camera** and an **Isaac Sim
digital twin**. It bundles these cooperating pipelines:

1. **Surface drawing / admittance control** — detect a work surface, generate a
   tool path on it, plan + execute the motion, and (optionally) follow the
   surface under force control.
2. **Perception → object ID → 6D pose → ICP refinement → tracking** — capture
   RGB‑D, have the server **name the part** (a PPF classifier picks the CAD from a
   library), run FoundationPose to get its 6D pose + mask, visualize it, refine that
   pose with point‑to‑plane ICP against the segmented point cloud, then track it live.
   The classified name drives which CAD the ICP node loads, so no part has to be
   selected by hand.
3. **Assembly → weld‑seam extraction → new-model export** — freeze each located
   part's CAD cloud into a static assembly model, find the joint between two
   near‑orthogonal parts by PCA curvature and publish it as a weld toolpath, and
   **rebuild the assembled parts into a single watertight CAD mesh** that can be
   uploaded back to the server as a new classifiable object for the next cycle.
4. **Hands‑free operation** — drive the whole perception cycle from hand gestures
   seen by the same RealSense, including *pointing at the part* to place the SAM2
   click. The operator never touches a keyboard on either machine.

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
                          │        ├─ foundationpose_bridge_node ──HTTP──▶ FoundationPose server (GPU)
                          │        │    → /perception/detections      │
                          │        │         │            │           │
                          │        │  detection_marker_node   icp_pose_│
                          │        │  (RViz markers + box)    refiner_ │
                          │        │                          node     │
                          │        │                    (segmented cloud
                          │        │                     + ICP metrics)│
                          │        └─ gesture_control_node             │
                          │             hand gestures → ~/capture,     │
                          │             SAM2 click, run/stop/save ICP  │
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
| `launch/point_cloud_config.rviz` | Saved RViz layout, opened by default by both pointcloud launches (`rviz_config` arg) |
| `srv/`, `action/` | Custom interfaces (`ComputeTOTG.srv`, `ExecuteDrawing.action`) |
| `config/` | UR5e description, controllers, kinematics, joint limits, SRDF |
| `urdf/`, `meshes/` | UR5e robot description |
| `models/*.ply` | CAD models for FoundationPose / ICP / PPF (e.g. `test_objv3.ply`). The ICP node resolves the classifier's answer to `models/<obj_name>.ply`, so names here must match the server's CAD library. |
| `world/welding_world.usda` | Isaac Sim scene (robot + eye‑in‑hand RealSense) |
| `notebooks/` | Hand‑eye calibration outputs (`T_tcp_to_cam.npy`), intrinsics, experiments, and the gesture sandbox (`gesture_control_sandbox.ipynb` + `gesture_recognizer.task`) |
| `notes/*.md` | Design notes behind each algorithm (see [Algorithms](#algorithms-and-the-maths-behind-them)) |
| `scripts/rgb_depth_to_send/` | Transfer dir: last `rgb.png` / `depth.png` / `camera.json` sent to the pose server |
| `scripts/foundationpose_results/` | **Live** pose-server outputs (`detection_pem.json` — now carries `obj_name`, `detection_ism.npz`, `mask.png`, `vis_pose.png`, `object_name.txt`) **and** assembly state (`static_env.ply/.npy`, `assembly.json`, `welding_points.ply/.npy`, `assembly_mesh.ply`). The ICP node's `results_dir` default. |
| `scripts/{fp_server,ppf_classifier,build_ppf_library,ppf_selftest,server}.py` | **Reference copies** of the GPU-host code (the FoundationPose Flask server + its PPF classifier and library tools). These run on the pose-server machine, *not* in the ROS graph; they are tracked here so the client and server contracts stay in sync. Edit on the host, then mirror back. |
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
   trigger ▶ foundationpose_bridge_node ──HTTP POST rgb+depth+camera+click──▶ FoundationPose server (fp_server.py, GPU)
                 │        (click → SAM2 mask → PPF classifies the CAD → register)
                 │  ◀── pose 4x4 [m] + object_name + classification table
                 │       + artifacts (detection_pem.json[obj_name], detection_ism.npz, mask.png, vis_pose.png, object_name.txt)
                 ▼
        /perception/detections   (Detection3DArray; class_id = classified CAD name)
        /perception/object_name  (std_msgs/String, latched)
                 │                                   │
                 ▼                                   ▼
   detection_marker_node                    icp_pose_refiner_node
   → /perception/detection_markers          Phase 1 (~/run_icp, once): read obj_name →
     (pose triads, labels, and an             load models/<obj_name>.ply → mask ∩ cloud
      oriented CAD wireframe box on            → point-to-plane ICP → seed current_pose
      the highest-scoring detection)         Phase 2 (timer, no pose server): dynamic
                                               CropBox around current_pose → Fast-
                                               ICP (Anderson-accel. point-to-plane)
                                               → update pose every frame
                                               → /perception/icp/scene_cloud (white)
                                               → /perception/icp/model_cloud (green)
                                               → /perception/icp/refined_pose + TF
                                               → /perception/icp/crop_box (Marker)
```

The server no longer needs to be told which part is on the table: a **PPF
classifier** (`ppf_classifier.py`) matches the masked depth cloud against a library
built from every CAD in `CAD_DIR`, and the winning *name* selects the mesh
FoundationPose registers against ([Algorithms §12](#12-ppf-object-classification)).
The bridge republishes that name — as the detection's `class_id` and on the latched
`/perception/object_name` — and the ICP node resolves it to `models/<obj_name>.ply`,
so the whole chain follows the classified part automatically.

**Who supplies the click.** FoundationPose has no detector, so something must say
which pixels are the object. Either the client sends the points, or the server
opens a window on the GPU host and blocks until an operator clicks it. Sending
them from the client is the normal path (and lets the server container run
without X11 forwarding); the window is the fallback and still works untouched.

A click only means something against the exact image it was picked on, so the
capture is split in two ([Gesture control](#e-gesture-control) drives both):

```
~/capture ─▶ bridge freezes one synced RGB-D pair, writes it to the transfer dir,
             republishes the colour image on /perception/frozen_rgb (latched)
                 │
                 ▼   operator picks points on THAT image
             /perception/sam2_click  (std_msgs/String, JSON)
                 {"stamp_ns": <frozen frame stamp>, "points": [[u,v], …],
                  "labels": [1,0,…]}       1 = object, 0 = not-object
                 │
                 ▼
~/trigger  ─▶ POSTs the frozen frame + click + click_labels; SAM2 runs headless
```

`stamp_ns` must match the frame the bridge is holding — that check is what makes
"did the operator point at the picture we are about to send?" an assertion rather
than a hope. Without it the click silently drifts onto a later frame, which is
invisible on a static scene and wrong the moment the arm (and with it the
eye‑in‑hand camera) moves. `~/clear_click` drops both the points and the frame;
`~/trigger` with nothing frozen falls back to grab‑and‑ask‑the‑host.

Negative points let SAM2 be told what *not* to grab, which is how you stop it
swallowing a neighbouring part pushed flush against the target. At least one
positive point is required — negatives only carve, they cannot start a mask.

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
        ~/run_icp (part B) → track → ~/save_object ─┤  (CAD auto-selected per part
                                                    │   from the classifier)
                                                    ▼
                    SEPC = CAD(A) ∪ CAD(B)  in base_link
                    → /perception/icp/static_env  (orange)
                    → foundationpose_results/static_env.ply + assembly.json
                          │                                 │
           ~/welding_points│                   ~/export_mesh│
                          ▼                                 ▼
     radius-PCA curvature → cross-object       re-instantiate each part's CAD at its
     filter → voxel merge                      stored pose_static → boolean-union
     → /perception/icp/welding_points (red)    → watertight faced mesh (mm)
     → foundationpose_results/welding_points.ply → foundationpose_results/assembly_mesh.ply
                                                        │
                                    set bridge model_ply_path + ~/add_model
                                                        ▼
                                    server indexes it as a new classifiable object
                                                        │
                                          ~/reset_environment
                                                        ▼
                                    SEPC dropped, assembly archived, next cycle starts clean
```

**Starting the next assembly.** Once the exported mesh is uploaded, the assembly
*is* one classifiable part and every object in the SEPC is stale — and a stale
SEPC keeps subtracting itself out of the live crop, so ICP goes blind exactly
where the next part gets placed. `~/reset_environment` drops the SEPC, the
saved‑object list and the tracking state, blanks the latched RViz clouds, and
moves `static_env.*` / `assembly.json` / `welding_points.*` / `assembly_mesh.ply`
into `<save_dir>/previous_assemblies/<timestamp>/`.

The files have to *move*, not just be forgotten: `welding_points` and
`export_mesh` reload them from disk whenever the in‑memory SEPC is empty (that
lazy reload is a feature — it survives a node restart), so leaving them in place
would resurrect the old assembly at the next call. They are archived rather than
deleted because you have usually just exported a mesh from them.

`assembly.json` records each frozen part's CAD filename **and** its refined 4×4
`pose_static`, which is what makes `~/export_mesh` possible: rather than
reconstructing a mesh from the faceless SEPC points, it reloads the *original* CADs,
places them at those poses, and boolean-unions them into a CAD-exact watertight mesh
([Algorithms §13](#13-assembly-mesh-export)). That mesh is the one artifact the
server's `/add_model` will accept — the raw `static_env.ply` is a point cloud with no
faces and is rejected.

Every tracking tick, the live crop is transformed into `base_link` and points
within `bg_subtract_dist_m` of the SEPC are deleted, so part B's ICP cannot snap
onto part A when the two are pushed flush ([`notes/model_based_background.md`](notes/model_based_background.md)).
The same transform applies the **ground cut** (`z <= ground_z_m` is the bench;
[`notes/ground_removal.md`](notes/ground_removal.md)).

### E. Gesture control

`gesture_control_node.py` runs MediaPipe's gesture recognizer on the **RealSense
colour stream** — not a separate webcam — and turns one‑shot gestures into the
service calls above.

Using the same camera is what makes pointing work at all. The hand and the
workpiece are then in one image, so the index fingertip lands on a RealSense
pixel *directly*: no calibration between two cameras, no coordinate mapping, no
mirroring. The cursor appears where the finger physically is, over the thing
being pointed at.

```
/camera/color/image_raw ─▶ GestureRecognizer ─▶ GestureLatch ─▶ Trigger clients
                        └▶ index fingertip ──▶ DwellCursor ──▶ /perception/sam2_click
/perception/frozen_rgb  ─▶ the image drawn on and picked against
```

| Gesture | Idle | Segmenting (a frame is frozen) |
|---|---|---|
| `ILoveYou` | `~/capture` — freeze a frame | re‑capture (replaces a bad freeze) |
| `Pointing_Up` | — | move the cursor; hold still `dwell_sec` → *pending* point |
| `Thumb_Down` | `~/clear_click` | commit pending as **negative** — "not this one" |
| `Thumb_Up` | — | commit pending as **positive**, publish, `~/trigger` |
| `Closed_Fist` | `~/run_icp` | `~/run_icp` |
| `Open_Palm` | `~/stop_tracking` | `~/stop_tracking` |
| `Victory` | `~/save_object` | `~/save_object` |

`Thumb_Down` is the one gesture that means different things in the two modes,
because a negative click is only meaningful while points are being picked. The
current mode is drawn in the window so it is never ambiguous.

**Two details worth knowing before tuning it.**

*Debouncing.* Raw per‑frame gestures flicker during transitions and a held pose
repeats at frame rate, so `GestureLatch` requires `stable_frames` consecutive
identical frames and fires once on the rising edge. What re‑arms it matters more
than it looks: passing through neutral does, but neutral cannot be *required* —
`Pointing_Up → Thumb_Up` are two poses one finger apart and the recognizer
frequently slides straight between them without ever reporting `None`. A
neutral‑only latch swallows every thumbs‑up that follows a point, which is the
one gesture the segmentation flow depends on. So a stable run of a *different*
gesture re‑arms it too, and `cooldown_sec` stays short (0.4 s) because repeats
are already blocked by tracking which pose last fired.

*Occlusion.* The gesture that triggers the capture needs the hand in view, so the
hand is in the frozen frame. Keep it to the side of the part when signing
`ILoveYou`. The window shows the frozen frame itself, so a hand across the
workpiece is visible immediately — sign `ILoveYou` again to re‑freeze.

**Orientation sensitivity.** MediaPipe's landmark stage tolerates rotation well,
but the *gesture classifier* is deliberately orientation‑dependent: `Thumb_Up`
and `Thumb_Down` have identical hand geometry and differ only by orientation in
the image, so a rotation‑invariant classifier could not separate them at all. It
was trained on roughly upright, camera‑facing hands. On a wrist‑mounted camera
"upright" moves with the arm, so present the hand upright relative to the image,
or retrain the classifier head with MediaPipe Model Maker for your viewing angle.

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
| `foundationpose_bridge_node.py` | **The pose bridge in use.** `~/capture` freezes one synced RGB‑D pair and republishes its colour image on latched `/perception/frozen_rgb` for the operator to pick points on; `~/trigger` POSTs that frozen frame — plus any click points received on `/perception/sam2_click` — to `fp_server.py`, and republishes the reply as `Detection3DArray` **plus the classified object name** on latched `/perception/object_name`. A click whose `stamp_ns` does not match the held frame is refused, so a pixel picked on one image can never be segmented on another. `~/trigger` with nothing frozen grabs the latest pair and sends no click, leaving the server to ask a human (hence `request_timeout_sec` = 300). The pose comes back as a 4×4 **in metres** — no mm→m division, unlike SAM‑6D. Saves server artifacts to `foundationpose_results/`. `~/add_model` uploads the `.ply` at `model_ply_path` to the server's `/add_model` so a newly-generated part becomes classifiable without a restart. | in: color+depth, `/perception/sam2_click`; srv: `~/capture`, `~/trigger`, `~/clear_click`, `~/add_model`; out: `/perception/detections`, `/perception/object_name`, `/perception/frozen_rgb` (all latched) |
| `gesture_control_node.py` | **Hands‑free operator console.** Runs MediaPipe's gesture recognizer on the RealSense colour stream, turns the 7 built‑in gestures into `Trigger` calls on the bridge and the ICP node, and turns the index fingertip into a SAM2 click via dwell‑to‑latch. Draws the frozen frame with the cursor and picked points so what the server will see is exactly what the operator sees. Needs `gesture_recognizer.task` (`model_path`). See [Data flow §E](#e-gesture-control). | in: `/camera/color/image_raw`, `/perception/frozen_rgb`; out: `/perception/sam2_click`; calls: bridge `~/capture`/`~/trigger`/`~/clear_click`, ICP `~/run_icp`/`~/stop_tracking`/`~/save_object` |
| `fp_server.py` | The **FoundationPose Flask server** (GPU machine, not the robot). `POST /predict_pose` with rgb/depth/camera → SAM2 mask from a click → **PPF classifier names the CAD** → `reset_object()` to that mesh → `register()`. Has **no detector of its own**: send a `mask` file, or `click` (`{"u":…,"v":…}` or `[[u,v],…]`) with optional parallel `click_labels` (1 = object, 0 = not‑object), to skip the window entirely — with a click it never needs `DISPLAY`, so the container can run without X11 forwarding. Click points are validated against the frame they arrived with, so a client that latched a pixel on a differently‑sized image is rejected (400) rather than quietly segmenting the wrong thing. Also serves `POST /classify` (classification only, ~1 s, for bring-up) and `POST /add_model` (index a new `.ply` into the live library and persist it). Re‑emits the result in SAM‑6D's on‑disk format (now with `obj_name`) so the ICP node needs no changes. Set `PPF_ENABLE=0` to pin it to a single `MESH_PATH` like before. | HTTP `:5000/{predict_pose,classify,add_model,health}` |
| `sam6d_bridge_node.py` | *Superseded by the FoundationPose bridge.* Same trigger→POST→publish cycle against `server.py`, parsing a list of detections (`R`, `t` mm→m). Still installed so an old SAM‑6D setup can be run for comparison. | in: color+depth; srv: `~/trigger`; out: `/perception/detections` (latched) |
| `server.py` | *Superseded by `fp_server.py`.* The SAM‑6D Flask server: instance seg (FastSAM/SAM) + pose estimation against `CAD_PATH`, re‑running `demo.sh` per request. | HTTP `:5000/predict_pose` |
| `detection_marker_node.py` | Converts `Detection3DArray` → RViz `MarkerArray` (RViz has no native `vision_msgs` display). Draws a pose triad + sphere + label per detection, and an **oriented CAD wireframe box** on the highest‑scoring detection. | in: `/perception/detections`; out: `/perception/detection_markers` |
| `icp_pose_refiner_node.py` | **Real‑time object tracker + assembly/weld model** (`notes/realtime_icp.md`). *Phase 1* `~/run_icp`: reads `obj_name` from the detection, loads `models/<obj_name>.ply` (falling back to `model_path`), uses the mask (`detection_ism.npz`) to segment the object, places the CAD at the FoundationPose pose, runs ICP, and seeds `current_pose`. *Phase 2* (timer at `tracking_rate_hz`, no pose server): builds a **dynamic CropBox** (model AABB + `crop_margin_m`) around `current_pose` to isolate the object, applies **ground removal** + **model‑based background subtraction**, runs **Fast‑ICP** (Anderson‑accelerated, Welsch‑robust point‑to‑plane) from `current_pose`, updates it, and publishes the CropBox as a Marker. Pauses if fitness < `lost_fitness`. *Assembly* `~/save_object`: bakes the CAD at its refined pose into the **SEPC** in `static_frame`, persists it (with each part's `pose_static`), clears tracking for the next part. *Weld* `~/welding_points`: extracts the joint between the SEPC's parts and publishes it red. *Export* `~/export_mesh`: boolean-unions the saved parts' CADs at their poses into a watertight `assembly_mesh.ply` for upload as a new model. *Reset* `~/reset_environment`: drops the SEPC, the saved-object list and the tracking state, blanks the latched clouds, and archives the on-disk assembly into `previous_assemblies/<timestamp>/` so the next cycle starts clean without a restart. `scene_from` (init only) = `pointcloud` or `depth_png`. With `use_open3d` (default), the per‑frame voxel downsample + normals run on the *cropped* cloud via Open3D (crop‑first is the key speedup); without Open3D it falls back to NumPy. | in: `/camera/depth/color/points`, TF; srv: `~/run_icp`, `~/stop_tracking`, `~/start_tracking`, `~/save_object`, `~/welding_points`, `~/export_mesh`, `~/reset_environment`; out: `/perception/icp/{scene_cloud,model_cloud,refined_pose,crop_box,static_env,welding_points}`, TF `sam6d_object` |
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
| `assembly_mesh.py` | Rebuilds a watertight CAD mesh of an assembled scene for `~/export_mesh`: `load_assembly` (read `assembly.json`), `build_assembly_mesh` (re-instantiate each CAD at its `pose_static`, scale to metres, **boolean-union**, re-centre, emit in mm), `build_and_write`. Needs `trimesh` + `manifold3d` (`pip install trimesh manifold3d`) — used only by that one service, so the rest of the stack has no new deps. |

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

### 12. PPF object classification

The server is handed a masked object with no label. **Point Pair Features** (PPF,
Drost et al. 2010, via OpenCV's `cv2.ppf_match_3d`) name it by matching the masked
depth cloud against a library of CAD models. A PPF is the 4‑tuple of a point pair
`(p₁, n₁), (p₂, n₂)`:

```
F(p₁, p₂) = ( ‖d‖,  ∠(n₁, d),  ∠(n₂, d),  ∠(n₁, n₂) ),   d = p₂ − p₁
```

It is invariant to rigid motion, so a model is summarised offline as a hash table
over the quantised features of all its point pairs, and a scene is recognised by
letting its pairs vote (a generalised Hough scheme) for a model + pose. Here the
pose is a means, not the end: the classifier computes one *only* to score how well
each CAD explains the cloud, then discards it — FoundationPose re‑estimates the pose
from scratch on the chosen mesh.

Two guards make the "which part" answer trustworthy:

- **Extent pre‑filter.** A model whose physical diameter is far from the scene
  cloud's is skipped before any matching, so a wrong FreeCAD export unit (a 1000×
  mis‑scale) shows up as "unclassifiable" rather than a silent mismatch.
- **Margin + verification.** Every candidate's inliers are counted at radius
  `PPF_TAU` (your depth noise, ~8 mm), and the winner's lead over the runner‑up is
  the **margin**. Two thin plates of similar size are genuinely indistinguishable
  from one view; `PPF_MIN_MARGIN` (with `PPF_STRICT`) makes the server *refuse*
  rather than guess when the margin is thin. The full score table always comes back
  in the reply, and the bridge logs it — the margin is the number to watch during
  bring‑up (`POST /classify` returns it in ~1 s without spending GPU on a pose).

The library is built once (`build_ppf_library.py`) from every `.ply` in `CAD_DIR`;
`POST /add_model` adds one incrementally (each model owns its own detector, so
nothing else retrains) and persists it. Details live on the host in
`ppf_classifier.py`; `ppf_selftest.py` scores each model against itself to find the
per‑model sampling step a thin curved part needs.

### 13. Assembly mesh export

`~/export_mesh` turns an assembled scene back into a single CAD mesh — the input a
*new* PPF model needs. The naive route is surface reconstruction from the fused SEPC
points (Delaunay + graph‑cut + manifold repair; see `notes/triangulation_idea.md`),
but that is lossy at ~2500 pts/part and needs a heavy stack. It is also
unnecessary: every SEPC part was *sampled from a known CAD at a known pose*, both
recorded in `assembly.json`. So instead we **re‑instantiate the originals**:

```
for each saved part:  M ← load CAD (faces intact)
                      M ← (M · mesh_scale)     # mm → m, the frame pose_static expects
                      M ← pose_static · M      # place in base_link
mesh ← ⋃ boolean-union(parts)                  # manifold3d backend, via trimesh
mesh ← recenter(mesh);  mesh ← mesh · 1000     # m → mm library convention
```

The result is CAD‑exact (no reconstruction error), watertight by construction (a
union of watertight solids is watertight; disjoint parts return as multiple
components in one mesh), and written in millimetres so the host's `MESH_SCALE=0.001`
reads it at true size like any other library `.ply`. The build validates
`is_watertight`/`volume` and warns if the union came back open. This is exactly the
"boolean union of the original CAD B‑reps" that `notes/triangulation_idea.md` names
as its own ground truth — available directly because the CADs and poses were kept.

---

## Interfaces

| Interface | Type | Summary |
|---|---|---|
| `execute_drawing` | action `ExecuteDrawing` | Goal: Cartesian `waypoints` + `orientation` (base_link). Feedback: `current_phase`, `drawing_progress`. Result: `success`, `message`. |
| `compute_totg` | service `ComputeTOTG` | Request: flattened joint waypoints + per‑joint vel/accel limits + `path_tolerance` + `resample_dt`. Response: timed positions/velocities + timestamps. |
| `~/capture` (foundationpose_bridge) | `std_srvs/Trigger` | Freeze one synced RGB‑D pair and republish its colour image on `/perception/frozen_rgb`. Nothing is sent. Re‑capturing replaces a bad freeze and drops the points picked on the old image. |
| `~/trigger` (foundationpose_bridge) | `std_srvs/Trigger` | POST the frozen frame (+ any matching click) → FoundationPose → publish. With no frame frozen and no click, grabs the latest pair and blocks until the object is clicked on the server host. Reply names the part (PPF), republished on `/perception/object_name`. |
| `~/clear_click` (foundationpose_bridge) | `std_srvs/Trigger` | Drop the frozen frame and the points picked on it. |
| `~/add_model` (foundationpose_bridge) | `std_srvs/Trigger` | Upload the `.ply` at the `model_ply_path` param to the server's `/add_model`, indexing it into the live PPF library. Must be a **faced mesh** (a point cloud is rejected). |
| `~/run_icp` (icp_pose_refiner) | `std_srvs/Trigger` | Phase‑1 init: segment (mask) + ICP‑refine the best detection, seed the tracker, and (if `auto_track`) start Phase‑2 tracking. |
| `~/stop_tracking` / `~/start_tracking` (icp_pose_refiner) | `std_srvs/Trigger` | Pause / resume the Phase‑2 tracking loop. |
| `~/save_object` (icp_pose_refiner) | `std_srvs/Trigger` | Freeze the tracked CAD at its refined pose into the SEPC; persist `static_env.*` + `assembly.json`; clear tracking for the next part. |
| `~/welding_points` (icp_pose_refiner) | `std_srvs/Trigger` | Extract the weld seam from the SEPC, publish it red, persist `welding_points.*`. Needs ≥ 2 saved objects. |
| `~/export_mesh` (icp_pose_refiner) | `std_srvs/Trigger` | Boolean-union the saved parts' CADs at their poses → watertight `assembly_mesh.ply` (mm), ready to feed the bridge's `~/add_model`. Needs `trimesh`+`manifold3d`. |
| `~/reset_environment` (icp_pose_refiner) | `std_srvs/Trigger` | Start a new assembly: drop the SEPC + saved objects + tracking state, blank the latched SEPC/weld clouds, and archive the on-disk assembly into `previous_assemblies/<timestamp>/`. Call it after `add_model`, or the stale SEPC keeps subtracting itself out of the live crop. |
| `~/go` (move_to_object) | `std_srvs/Trigger` | Execute the standoff move (safety gate). |

### Key topics

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/joint_states` / `/isaac_joint_states` | `sensor_msgs/JointState` | robot/Isaac → robot_state_publisher |
| `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/color/camera_info` | `Image`/`CameraInfo` | camera node → pointcloud + bridge |
| `/camera/depth/color/points` | `PointCloud2` | depth_image_proc → ICP / RViz |
| `/perception/detections` | `vision_msgs/Detection3DArray` | foundationpose_bridge → markers / ICP / move_to_object (`class_id` = classified CAD name) |
| `/perception/object_name` | `std_msgs/String` (latched) | foundationpose_bridge → ICP / trackers (which `.ply` the classifier picked) |
| `/perception/frozen_rgb` | `sensor_msgs/Image` bgr8 (latched) | foundationpose_bridge → gesture_control / RViz (the held frame points are picked on; its `header.stamp` is the handle the click quotes back) |
| `/perception/sam2_click` | `std_msgs/String` (JSON) | gesture_control → foundationpose_bridge (`{"stamp_ns":…, "points":[[u,v],…], "labels":[1,0,…]}`) |
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

**RViz layout.** Both launches open RViz with `-d launch/point_cloud_config.rviz`
(robot model, the point cloud, the ICP scene/model clouds, the orange SEPC, the
red weld seam, detection markers, and the live + frozen colour images). Override
with `rviz_config:=/path/to/other.rviz`, or `rviz_config:=''` for a bare RViz. A
missing file logs a warning and falls back to RViz's default layout rather than
failing the launch.

The file is installed by the existing `install(DIRECTORY launch …)` rule, so it
resolves under `install/`, not `src/`. This workspace is built with
`--symlink-install`, so re‑saving the layout over
`src/admittance_control/launch/point_cloud_config.rviz` takes effect on the next
launch — just make sure RViz's *Save Config As* points at the `src/` path.
Without `--symlink-install` you need a `colcon build` after each save.

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
# Point the ICP node at the CAD folder once; each part's CAD is then chosen
# automatically from the classifier (models/<obj_name>.ply). model_path is only
# a fallback for when classification is off.
ros2 param set /icp_pose_refiner model_path <ws>/src/admittance_control/models/test_objv2_base.ply

# --- part A ---
ros2 service call /foundationpose_bridge/trigger        std_srvs/srv/Trigger   # classify + pose
ros2 service call /icp_pose_refiner/run_icp    std_srvs/srv/Trigger   # loads the classified CAD, seed + track
ros2 service call /icp_pose_refiner/save_object std_srvs/srv/Trigger  # freeze into the SEPC

# --- part B (place it against A, then locate it) — no model_path change needed ---
ros2 service call /foundationpose_bridge/trigger        std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/run_icp    std_srvs/srv/Trigger
ros2 service call /icp_pose_refiner/save_object std_srvs/srv/Trigger

# --- the joint between them, in red ---
ros2 service call /icp_pose_refiner/welding_points std_srvs/srv/Trigger
# → "99 welding points from 314 edge points (of 5000 SEPC points)"

# --- export the assembly as a new CAD model and teach the server ---
ros2 service call /icp_pose_refiner/export_mesh std_srvs/srv/Trigger   # → foundationpose_results/assembly_mesh.ply
ros2 param set /foundationpose_bridge model_ply_path \
  <ws>/src/admittance_control/scripts/foundationpose_results/assembly_mesh.ply
ros2 service call /foundationpose_bridge/add_model std_srvs/srv/Trigger # server indexes it; classifiable next cycle

# --- start the next assembly with a clean environment (no restart needed) ---
ros2 service call /icp_pose_refiner/reset_environment std_srvs/srv/Trigger
```

**Do not skip the reset.** The assembly is now a single classifiable part, so the
individual objects in the SEPC are stale — and a stale SEPC keeps subtracting
itself out of the live crop, so ICP goes blind exactly where the next part gets
placed. Restarting the node is *not* an equivalent workaround: it clears the
in‑memory SEPC (so background subtraction does stop), but `welding_points` and
`export_mesh` reload `static_env.npy` + `assembly.json` from disk, so the first
call after a restart quietly resurrects the previous assembly.

The classified `obj_name` must resolve to `models/<obj_name>.ply` on the ROS side,
which must be the **same CAD** the server holds in its library. The seam is written
to `foundationpose_results/welding_points.ply` and published on
`/perception/icp/welding_points` in `base_link`. `export_mesh` needs `trimesh` +
`manifold3d` in the ROS‑side Python env (`pip install trimesh manifold3d`).

`welding_points` reloads `static_env.npy` + `assembly.json` from `save_dir` if
the node was restarted, so you can re‑run the extraction (with different `R` or
threshold) without re‑locating the parts:

```bash
ros2 param set /icp_pose_refiner weld_radius_m 0.006          # must exceed the gap...
ros2 param set /icp_pose_refiner weld_curvature_thresh 0.03   # ...and stay under part thickness
ros2 service call /icp_pose_refiner/welding_points std_srvs/srv/Trigger
```

### Run the cycle by hand gestures

One‑time: fetch MediaPipe's recognizer bundle.

```bash
wget -O notebooks/gesture_recognizer.task \
  https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task
pip install mediapipe
```

With the perception stack already up:

```bash
ros2 run admittance_control gesture_control_node.py --ros-args \
  -p model_path:=<ws>/src/admittance_control/notebooks/gesture_recognizer.task
```

A window opens on the RealSense stream. Then, entirely by hand:

```
ILoveYou                     freeze a frame  (keep the hand clear of the part)
Pointing_Up + hold still     cursor → pending point (hollow yellow)
  Thumb_Down                 → mark it "not this one" (red), repeat as needed
  Pointing_Up + hold still   → move to the real target
Thumb_Up                     → positive point (green) + segment: SAM2 → PPF → FoundationPose
Closed_Fist                  run ICP        Open_Palm   stop tracking
Victory                      save object into the SEPC
```

The status line at the bottom of the window echoes every service reply, so a
failed call is visible without watching the logs. `welding_points`,
`export_mesh` and `reset_environment` stay service calls — there are only 7
built‑in gestures and they are spent. See [Data flow §E](#e-gesture-control).

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
- **Different object?** Put its CAD `.ply` in `models/` (ROS side) **and** in the
  server's `CAD_DIR` under the **same filename** — the classifier returns that stem
  as `obj_name` and the ICP node loads `models/<obj_name>.ply`. Rebuild the PPF
  library (`build_ppf_library.py`) or upload live via the bridge's `~/add_model`. To
  bypass classification, set `PPF_ENABLE=0` on the server and use ICP `model_path`.
- **Generated a new part (an assembly)?** `~/export_mesh` writes a watertight
  `assembly_mesh.ply`; feed it to the bridge's `~/add_model`. Do **not** upload the
  raw `static_env.ply` — it is a faceless point cloud and the server rejects it.
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
- **ICP still shows the previous object's CAD after the part changed?** The running
  node loaded the old module at startup — a source edit (or `--symlink-install`)
  doesn't hot‑reload it. **Restart `icp_pose_refiner`.** Then confirm the log line
  `model <obj_name>.ply (classified <obj_name>)`; if it fell back to `model_path`,
  `models/<obj_name>.ply` is missing or misnamed.
- **`add_model` returns HTTP 500 `NoneType * NoneType` (or 400 "0 faces")?** You
  uploaded a point cloud, not a mesh — almost always `static_env.ply`. Run
  `~/export_mesh` and upload the resulting `assembly_mesh.ply` instead.
- **`export_mesh` fails to import / "needs manifold3d"?** `pip install trimesh
  manifold3d` into the **ROS‑side** Python env (not the server's).
- **Classifier keeps confusing two parts?** Their diameters are within the extent
  pre‑filter's tolerance, so it comes down to the verification margin. Watch the
  score table the bridge logs; if the margin is genuinely thin from that viewpoint,
  set `PPF_MIN_MARGIN` + `PPF_STRICT` on the server so it refuses rather than guesses
  (§12).
- **Second assembly's ICP goes blind where you put the part?** The previous
  assembly is still in the SEPC, subtracting itself out of the live crop. Call
  `~/reset_environment` after `add_model`. Restarting the node is not equivalent —
  it clears memory but `welding_points`/`export_mesh` reload the old
  `static_env.npy` from disk.
- **Bridge logs "ignoring click … the client is pointing at a stale image"?** The
  click's `stamp_ns` does not match the frozen frame — usually a `~/capture` landed
  between picking the points and triggering. Re‑capture and pick again. This is the
  check working: the alternative is segmenting a pixel nobody looked at.
- **Server returns 400 "click … is outside the WxH frame"?** The client latched a
  pixel on a differently‑sized image than it sent. In the gesture node that means
  the live and frozen streams disagree on resolution (someone reconfigured the
  camera mid‑run); it logs an error and suppresses the cursor when it detects this.
- **Gestures only recognized in one hand orientation?** Expected. MediaPipe's
  gesture classifier is deliberately orientation‑dependent — `Thumb_Up` and
  `Thumb_Down` are the same hand geometry and differ *only* by orientation, so it
  cannot be rotation‑invariant. Present the hand upright relative to the image, or
  retrain the head with Model Maker for your camera angle ([§E](#e-gesture-control)).
- **A gesture fires but the next one is swallowed?** `cooldown_sec` is longer than
  the gap between them. The `Pointing_Up → dwell → Thumb_Up` sequence puts ~1 s
  between two fires, so anything above that eats the thumbs‑up. Repeats are already
  blocked by the latch tracking which pose last fired; the cooldown only has to
  absorb recognizer jitter.
- **`QFontDatabase: Cannot find font directory` from the gesture node?** Cosmetic.
  cv2's bundled Qt ships no fonts; `cv2.putText` doesn't use them. Ignore it.
