# Sand Drawer — UR5e Planar Drawing System

Draw shapes on a flat surface using a UR5e robot arm, with support for **Isaac Sim** (virtual) and **real UR5e** hardware. The system captures a drawing plane from 4 physical points, plans collision-free paths in joint space, and follows dense Cartesian trajectories on the surface.

Current action-mode version supports dynamic TCP tools. Tool faces are selected
around wrist-3 with `active_tool`:
- `fork` = 0°
- `pointy` = +90°
- `empty` = -90°
- `spatula` = 180°
- `orthogonal` = 0° with configurable tip length via `orthogonal_tool_length_m`

Action mode computes a dynamic wrist pose from each tip waypoint during IK,
which improves robustness on larger drawings.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [File Map](#file-map)
3. [Coordinate Frames](#coordinate-frames)
4. [The Plane JSON — Central Data Contract](#the-plane-json--central-data-contract)
5. [Motion Pipeline — Phase by Phase](#motion-pipeline--phase-by-phase)
6. [Kinematics Engine (ur5e_rrt_planner.py)](#kinematics-engine-ur5e_rrt_plannerpy)
7. [Action Mode — Drawing Server + Dispatcher](#action-mode--drawing-server--dispatcher)
8. [TOTG Service Node (totg_service_node.cpp)](#totg-service-node-totg_service_nodecpp)
9. [Cartesian Controller (cartesian_square_controller.py)](#cartesian-controller-cartesian_square_controllerpy)
10. [Velocity Controller (planar_servo_controller.py)](#velocity-controller-planar_servo_controllerpy)
11. [Jacobian IK Node (jacobian_calculator_node.cpp)](#jacobian-ik-node-jacobian_calculator_nodecpp)
12. [Plane Capture — Simulation vs Real Robot](#plane-capture--simulation-vs-real-robot)
13. [Real Robot Bridging](#real-robot-bridging)
14. [Safety Systems](#safety-systems)
15. [Parameter Reference](#parameter-reference)
16. [Launch Modes & Usage](#launch-modes--usage)
17. [Tuning Guide](#tuning-guide)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         LAUNCH FILE                                     │
│                   sand_drawer.launch.py                                  │
│                                                                         │
│   Picks ONE mode and spawns the right nodes:                            │
│     mode=action      → ActionServer + TOTG + Dispatcher (recommended)   │
│     mode=trajectory  → PlanarServoController + JacobianCalc + RSP       │
│     mode=teleop      → PlanarServoController (teleop) + JacobianCalc    │
│     mode=cartesian   → CartesianDrawController + RSP                    │
│     mode=capture     → PlaneSolverNode + RSP  (sim, red ball)           │
│     mode=freedrive_capture → FreedrivePlaneCapture + RSP  (real robot)  │
└──────────────────────────────┬──────────────────────────────────────────┘
                               │
           ┌───────────────────┼───────────────────────┐
           ▼                   ▼                       ▼
   ┌───────────────┐  ┌────────────────┐  ┌──────────────────────┐
   │ Plane Capture  │  │   Controller   │  │  Kinematics Engine   │
   │                │  │  (one of two)  │  │  ur5e_rrt_planner.py │
   │ plane_solver   │  │                │  │                      │
   │ _node.py  (sim)│  │ cartesian_sq…  │  │  • FK  (base_link    │
   │       OR       │  │ _controller.py │  │        → tool0)      │
   │ freedrive_     │  │       OR       │  │  • IK  (damped LS)   │
   │ plane_capture  │  │ planar_servo_  │  │  • RRT-Connect       │
   │ .py    (real)  │  │ controller.py  │  │  • Path smoothing    │
   │                │  │                │  │  • Bezier splines    │
   └───────┬────────┘  └───────┬────────┘  └──────────┬───────────┘
           │                   │                       │
           │  writes           │  reads                │  imported by
           ▼                   ▼                       │  controllers
   ┌───────────────────────────────────┐               │
   │     sand_drawer_plane.json        │◄──────────────┘
   │  (plane, rectangle, waypoints,    │
   │   orientations, UV path)          │
   └───────────────────────────────────┘
```

### Data Flow (Cartesian mode — `mode=cartesian`)

```
                    ┌──────────────────────┐
                    │  sand_drawer_plane    │
                    │       .json           │
                    └──────────┬────────────┘
                               │ load at startup
                               ▼
┌─────────────┐    ┌──────────────────────────┐    ┌─────────────┐
│ Isaac Sim   │───►│  CartesianDrawController  │───►│ Isaac Sim   │
│ /isaac_     │    │                            │    │ /isaac_     │
│ joint_states│    │  FK/IK/RRT from            │    │ joint_cmds  │
└─────────────┘    │  ur5e_rrt_planner.py       │    └─────────────┘
                   │                            │
┌─────────────┐    │  (real_robot=true also:)   │    ┌─────────────┐
│ UR Driver   │───►│  /joint_states feedback    │───►│ UR Driver   │
│ /joint_     │    │                            │    │ /scaled_    │
│ states      │    │                            │    │ joint_traj… │
└─────────────┘    └────────────────────────────┘    └─────────────┘
```

### Data Flow (Velocity mode — `mode=trajectory`)

```
                    ┌──────────────────────┐
                    │  sand_drawer_plane    │
                    │       .json           │
                    └──────────┬────────────┘
                               │
                               ▼
┌─────────────┐    ┌──────────────────────────┐    ┌──────────────────┐
│  TF tree    │───►│  PlanarServoController    │───►│ /end_effector_   │
│ (RSP + TF2) │    │  (PD velocity control)    │    │  velocity (Twist)│
└─────────────┘    │                            │    └────────┬─────────┘
                   │  FK/IK/RRT from            │             │
                   │  ur5e_rrt_planner.py       │             ▼
                   │  (for RETRACT/RRT phases)  │    ┌──────────────────┐
                   └────────────────────────────┘    │ JacobianCalc     │
                                                     │ (C++ MoveIt)     │
┌─────────────┐                                      │ damped J⁻¹ →    │
│ Isaac Sim   │◄─────────────────────────────────────│ /isaac_joint_cmds│
│ /isaac_     │                                      └──────────────────┘
│ joint_states│
└─────────────┘
```

### Data Flow (Action mode — `mode=action`, **recommended**)

```
                    ┌──────────────────────┐
                    │  sand_drawer_plane    │
                    │       .json           │
                    └──────────┬────────────┘
                               │ read by both
                ┌──────────────┼──────────────────────┐
                ▼                                      ▼
┌──────────────────────┐             ┌───────────────────────────────┐
│  DrawingDispatcher   │──(action)──►│   DrawingActionServer         │
│  Reads JSON / params │  Goal:      │   (pre-computes & executes)   │
│  Generates waypoints │  waypoints  │                               │
│  Sends goals via     │  + orient.  │  FK/IK/RRT from               │
│  ExecuteDrawing      │             │  ur5e_rrt_planner.py          │
│  action client       │◄─feedback───│                               │
└──────────────────────┘  progress   │  Joint-space timing via:      │
                                     │  ┌─────────────────────────┐  │
                                     │  │ TOTG Service (C++)      │  │
                                     │  │ compute_totg            │  │
                                     │  │ (Kunz & Stilman 2012)   │  │
                                     │  └─────────────────────────┘  │
                                     └──────────┬────────────────────┘
                                                │
                              ┌─────────────────┼──────────────────┐
                              ▼                                    ▼
                  ┌─────────────────────┐            ┌─────────────────────┐
                  │ Isaac Sim           │            │ UR Driver           │
                  │ /isaac_joint_cmds   │            │ /scaled_joint_      │
                  │                     │            │  trajectory_ctrl    │
                  └─────────────────────┘            └─────────────────────┘
```

**Key differences between modes:**
- **Action mode** decouples path generation (dispatcher) from execution (action server). Supports sequential multi-shape drawing via continuous dispatch. Uses MoveIt 2 TOTG for time-optimal, jerk-free joint-space timing on RRT and homing phases.
- **Cartesian mode** does its own IK internally (via `ur5e_rrt_planner.py`). Single monolithic run.
- **Velocity mode** sends Twist commands and relies on the C++ Jacobian node to resolve them into joint space.

### Action mode tool behavior (`active_tool`)

| Tool | Wrist angle | Tip length from wrist axis |
|------|-------------|----------------------------|
| `fork` | 0° | 0.13 m |
| `pointy` | +90° | 0.15 m |
| `empty` | -90° | 0.13 m |
| `spatula` | 180° | 0.13 m |
| `orthogonal` | 0° | `orthogonal_tool_length_m` (default 0.13 m) |

In action mode, the dispatcher sends tip paths and the action server computes
per-waypoint dynamic wrist poses before IK.

---

## File Map

```
sand_drawer/
├── action/
│   └── ExecuteDrawing.action           # Action interface for sequential drawing
│
├── srv/
│   └── ComputeTOTG.srv                 # Service interface for TOTG timing
│
├── scripts/
│   ├── ur5e_rrt_planner.py             # FK, IK, RRT, path smoothing (LIBRARY)
│   ├── drawing_action_server.py        # Action-based drawing controller (ROS NODE)
│   ├── drawing_dispatcher.py           # Action client / drawing feeder (ROS NODE)
│   ├── cartesian_square_controller.py  # Position-controlled drawing (ROS NODE)
│   ├── planar_servo_controller.py      # Velocity-controlled drawing (ROS NODE)
│   ├── plane_solver_node.py            # Plane capture — simulation (ROS NODE)
│   ├── freedrive_plane_capture.py      # Plane capture — real robot (ROS NODE)
│   ├── plane_frame_broadcaster.py      # Static TF broadcaster (ROS NODE)
│   ├── plane_teleop_keyboard.py        # Keyboard velocity input (ROS NODE)
│   └── pd_auto_tuner.py               # Auto-tune PD gains (standalone)
│
├── src/
│   ├── jacobian_calculator_node.cpp    # MoveIt Jacobian IK engine (C++ NODE)
│   └── totg_service_node.cpp           # MoveIt TOTG trajectory timing (C++ NODE)
│
├── launch/
│   └── sand_drawer.launch.py           # Main launch file (all modes)
│
├── generated_planes/
│   └── sand_drawer_plane.json          # Plane definition (output of capture)
│
├── config/                             # Robot SRDF, kinematics config
├── meshes/ur5e/                        # Collision & visual meshes
├── urdf/                               # UR5e URDF/xacro
└── world/                              # Isaac Sim USD scenes
```

### Who imports what

| File | Imports from `ur5e_rrt_planner.py` | Purpose |
|------|-----------------------------------|---------|
| `drawing_action_server.py` | `ur5e_fk`, `ik_solve`, `rrt_connect`, `smooth_path`, `bezier_smooth_path`, `ur5e_jacobian` | All phases: FK, IK, RRT, Cartesian + joint-space planning |
| `cartesian_square_controller.py` | `ur5e_fk`, `ik_solve`, `rrt_connect`, `smooth_path`, `bezier_smooth_path`, `ur5e_jacobian` | All phases: FK for position checks, IK for Cartesian→joint, RRT for free-space planning |
| `planar_servo_controller.py` | `ur5e_fk`, `ik_solve`, `rrt_connect`, `smooth_path`, `bezier_smooth_path` | RETRACT/RRT/DESCENT phases only (SERVO phase uses the C++ Jacobian node) |
| Standalone test scripts | `ur5e_fk`, `ik_solve` | Quick verification |

---

## Coordinate Frames

The UR5e URDF defines several frames. Understanding them is **critical** because the real robot's driver and our FK/IK use different conventions:

```
world / base_link  (URDF root — REP-103: X+ forward, Y+ left, Z+ up)
    │
    ├──[Rz(π)]──► base_link_inertia  (internal UR frame, X+ backward)
    │                 │
    │                 ├──► shoulder_link ──► upper_arm ──► forearm
    │                 │    ──► wrist_1 ──► wrist_2 ──► wrist_3
    │                 │    ──► flange ──► tool0
    │                 │
    │                 └── (FK chain starts here with Rz(π) from base_link)
    │
    └──[Rz(π)]──► base  (UR controller's "Base" frame, same orientation as base_link_inertia)
```

### The 180° Rotation Issue

| Frame | Convention | Used by |
|-------|-----------|---------|
| `base_link` | REP-103 (X+ forward) | URDF, TF tree, our `ur5e_fk()` output, Isaac Sim |
| `base` | UR internal (X+ backward) = `Rz(π)` from `base_link` | UR driver's `tcp_pose_broadcaster`, freedrive capture |

**Our FK function `ur5e_fk(q)` returns poses in `base_link` frame** (the URDF root). But the UR driver's TCP broadcaster reports in the `base` frame. These differ by `Rz(π)` — a 180° rotation around Z, which **negates X and Y**.

**How we handle it:** When loading the plane JSON, both controllers check `target_frame`. If it says `"base"` (freedrive capture), all positions get `[x,y,z] → [-x,-y,z]` and quaternions get the equivalent rotation. This happens in `_load_plane_json()` before any other processing.

---

## The Plane JSON — Central Data Contract

Every controller reads the same JSON file. Here's the schema:

```jsonc
{
  "target_frame": "base",              // "base" (freedrive) or "base_link" (sim)
  "source_topic": "...",               // provenance info
  "points_source": "freedrive_capture",// or "manual" or "red_ball"

  "captured_points_base": [            // raw 4 captured XYZ positions
    [x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]
  ],

  "plane": {
    "origin": [x, y, z],              // first captured point (P1)
    "x_axis": [dx, dy, dz],           // unit vector along rectangle width
    "y_axis": [dx, dy, dz],           // unit vector along rectangle height
    "normal": [nx, ny, nz]            // surface normal (cross product)
  },

  "rectangle_corners": [              // 4 corners of the bounding rectangle
    [x, y, z], ...
  ],

  "square_trajectory": [              // inscribed square (scaled by square_scale)
    {"position": [x,y,z], "orientation_xyzw": [x,y,z,w]},
    ...  // 5 points (closed loop)
  ],

  "vector_path_uv": [                 // UV polyline (pairs of u,v in 0→1)
    0.2, 0.2,  0.8, 0.2, ...          // default: rectangle inset path
  ],

  "projected_vector_trajectory": [     // UV path mapped to 3D world positions
    {"position": [x,y,z], "orientation_xyzw": [x,y,z,w]},
    ...
  ]
}
```

### UV Coordinates

The rectangle surface is parameterised as UV where `(0,0)` = corner 0 and `(1,1)` = corner 3. The `trajectory_key` parameter selects which trajectory to follow:

| Key | Source | Typical use |
|-----|--------|-------------|
| `line` | Generated from `line_u_start/v_start → line_u_end/v_end` params | Simple line drawing |
| `square_trajectory` | From JSON (inscribed square) | Border test |
| `projected_vector_trajectory` | From JSON (UV polyline mapped to 3D) | Complex paths |

---

## Motion Pipeline — Phase by Phase

Both controllers follow the same high-level state machine. The cartesian controller is fully position-controlled; the velocity controller switches to Twist-based PD control during the on-surface phase.

```
 ┌────────────┐     ┌──────────────┐     ┌────────────┐
 │ RETRACT_UP │────►│ RETRACT_HOME │────►│ HOME_HOLD  │
 │ Cartesian  │     │ Joint-space  │     │ 2s settle  │
 │ lift +Z    │     │ cosine interp│     │            │
 └────────────┘     └──────────────┘     └──────┬─────┘
                                                │
                                                ▼
                    ┌──────────────┐     ┌────────────┐
                    │  EXECUTING   │◄────│  PLANNING  │
                    │  Follow RRT  │     │  RRT-Connect│
                    │  joint path  │     │  + smooth  │
                    └──────┬───────┘     └────────────┘
                           │
                           ▼
                    ┌──────────────┐
                    │  DESCENDING  │
                    │  Cartesian   │
                    │  straight ↓  │
                    └──────┬───────┘
                           │
         ┌─────────────────┼────────────────────┐
         │ (cartesian)     │                    │ (velocity)
         ▼                 │                    ▼
  ┌──────────────┐         │            ┌──────────────┐
  │   DRAWING    │         │            │    SERVO     │
  │  Cartesian   │         │            │  PD velocity │
  │  IK per wp   │         │            │  via Jacobian│
  └──────┬───────┘         │            └──────┬───────┘
         │                 │                   │
         ▼                 │                   ▼
  ┌──────────────┐         │            ┌──────────────┐
  │  ASCENDING   │         │            │    DONE      │
  │  Cartesian ↑ │         │            │  or TELEOP   │
  └──────┬───────┘         │            └──────────────┘
         │                 │
         ▼                 │
  ┌──────────────┐         │
  │    DONE      │         │
  └──────────────┘         │
```

### Phase Details

#### 1. RETRACT_UP (both controllers)
**Purpose:** Safely lift the end-effector straight up before any other motion.
**How it works:**
- Read current joint angles → compute FK → get current EE position
- Generate a vertical lift path: current Z → current Z + `retract_height` (0.15 m)
- Dense Cartesian interpolation using trapezoidal velocity profile (`_interpolate_cartesian_smooth`)
- Solve IK for each Cartesian waypoint, publish joint commands one per tick
- **Real robot:** Batch-solve all IK → compute trapezoidal timing → send one multi-point `JointTrajectory` message → monitor convergence

#### 2. RETRACT_HOME (both controllers)
**Purpose:** Move from retracted position to the known home configuration.
**How it works:**
- Cosine-interpolated joint-space path from current → home (~2 seconds at execution_hz)
- `α_smooth = 0.5 × (1 - cos(π × t/T))` gives smooth ease-in/ease-out
- **Real robot:** Full trajectory with trapezoidal timing sent once

#### 3. HOME_HOLD (both controllers)
**Purpose:** Hold for 2 seconds at home to verify stability and let the real robot settle.

#### 4. PLANNING (both controllers)
**Purpose:** Find a collision-free joint-space path from home to the approach pose (above the first drawing point).
**How it works — this is where `ur5e_rrt_planner.py` is called:**

```python
from ur5e_rrt_planner import (ur5e_fk, rrt_connect, smooth_path, bezier_smooth_path)

# Step 1: Compute approach position (above first waypoint along surface normal)
approach_pos = draw_positions[0] - approach_height * plane_normal

# Step 2: Constrained IK — find elbow-up goal joint configuration
#   Tries 30+ seeds biased toward elbow-up (shoulder_lift < 0, elbow < -0.3)
#   Picks solution closest to home position
q_goal = _constrained_ik_for_pose(T_approach)

# Step 3: RRT-Connect in joint space
raw_path = rrt_connect(q_current, q_goal, step_size=0.2, max_iter=10000)

# Step 4: Shortcut smoothing (random rewiring, 200 attempts)
smoothed = smooth_path(raw_path, max_attempts=200)

# Step 5: Catmull-Rom spline interpolation (C1-continuous)
path = bezier_smooth_path(smoothed, max_step=0.02)
```

#### 5. EXECUTING (both controllers)
**Purpose:** Play back the planned RRT joint-space path.
- **Sim:** Step through one waypoint per tick
- **Real robot:** One multi-point `JointTrajectory` with trapezoidal timing, then monitor convergence by comparing real joint state to closest path waypoint

#### 6. DESCENDING (both controllers)
**Purpose:** Lower from approach height to the drawing surface along a straight Cartesian line.
**How:** Dense Cartesian interpolation → per-waypoint IK (or batch IK → full trajectory for real robot).

#### 7. DRAWING (cartesian controller) / SERVO (velocity controller)
**Cartesian controller:** Dense Cartesian waypoints with trapezoidal velocity profile, each solved via damped-least-squares IK with joint-jump rejection.
**Velocity controller:** PD control loop computes a Twist command from position error, publishes to `/end_effector_velocity`, and the C++ Jacobian node resolves it to joint commands.

#### 8. ASCENDING (cartesian controller only)
**Purpose:** Lift off the surface after drawing.

---

## Kinematics Engine (ur5e_rrt_planner.py)

This is a **pure Python library** (no ROS dependencies). Both controllers import functions from it at runtime.

### Forward Kinematics — `ur5e_fk(q)`

Computes the 4×4 homogeneous transform from `base_link` to `tool0`.

```
base_link
  └─[Rz(π)]──► base_link_inertia
      └─[Tz(0.1625) Rz(q₁)]──► shoulder
          └─[Rx(π/2) Rz(q₂)]──► upper_arm
              └─[Tx(-0.425) Rz(q₃)]──► forearm
                  └─[Tx(-0.3922) Tz(0.1333) Rz(q₄)]──► wrist_1
                      └─[Rx(π/2) Ty(-0.0997) Rz(q₅)]──► wrist_2
                          └─[Rx(π/2)Ry(π)Rz(π) Ty(0.0996) Rz(q₆)]──► wrist_3
                              └─[Ry(-π/2)Rz(-π/2)]──► flange
                                  └─[Rx(π/2)Rz(π/2)]──► tool0
```

The chain starts with `Rz(π)` because the URDF `base_link` (REP-103, X-forward) differs from the UR internal frame (`base_link_inertia`, X-backward) by 180° around Z.

**Returns:** 4×4 matrix. Position = `T[:3, 3]`, orientation = `T[:3, :3]`.

### Jacobian — `ur5e_jacobian(q)`

6×6 numerical Jacobian via central finite differences (ε = 1e-6). Rows 0-2 are linear velocity, rows 3-5 are angular velocity.

### Inverse Kinematics — `ik_solve(T_target, q_seed)`

Damped-least-squares (Levenberg-Marquardt style) iterative solver:

```
for each iteration:
    err = pose_error(FK(q), T_target)     # 6-vector [pos_err; orient_err]
    if |pos_err| < 0.5mm and |orient_err| < 1e-3 rad:
        return q (wrapped to joint limits)
    J = jacobian(q)
    dq = Jᵀ (JJᵀ + λ²I)⁻¹ err           # damped pseudoinverse
    q += clamp(dq, 0.3)                   # max step 0.3 rad
```

**Parameters:**
- `max_iter=200` — convergence attempts
- `pos_tol=1e-4` — position tolerance (m)
- `orient_tol=1e-3` — orientation tolerance (rad)
- `damping=0.05` — λ in the damped pseudoinverse

### RRT-Connect — `rrt_connect(q_start, q_goal)`

Bidirectional rapidly-exploring random tree in 6D joint space:

1. **Tree A** grows from `q_start`, **Tree B** from `q_goal`
2. Each iteration: sample random config (with 10% goal bias), extend Tree A toward it
3. If Tree A extended successfully, try to **connect** Tree B to the new node (greedy)
4. If trees connect → extract path. Otherwise swap trees and repeat.
5. Joint validity = within joint limits (no geometric collision model)

**Parameters:**
- `step_size=0.2` — max joint-space step per extend
- `max_iter=5000` — planning attempts
- `goal_bias=0.1` — probability of sampling the goal directly

### Path Smoothing — Two Stages

**Stage 1: Shortcut smoothing — `smooth_path(path)`**

Random shortcutting: pick two random waypoints i, j; if the direct edge is valid (checked at 0.02 rad resolution), remove all intermediate waypoints. Repeat up to 200 times.

This removes unnecessary zigzags from the RRT output.

**Stage 2: Catmull-Rom spline — `bezier_smooth_path(path)`**

Despite the name "bezier", this actually uses **centripetal Catmull-Rom splines** (α=0.5):

- Creates a **C1-continuous** curve through all remaining waypoints
- Phantom endpoints are reflected from the first/last two points
- Each segment is densely sampled with `max_step=0.02` rad spacing
- If any interpolated point violates joint limits, that segment falls back to linear interpolation

The result is a smooth, jerk-free joint-space path that passes exactly through every post-shortcut waypoint.

**Why not Bézier?** Catmull-Rom is an **interpolating** spline (passes through control points), while cubic Bézier is **approximating** (pulled toward but doesn't pass through). For robot paths we need exact waypoint hitting.

### High-Level Planning — `plan_to_pose(q_current, T_target)`

Convenience function: IK (multi-seed) → RRT-Connect → smooth → Bezier → return dense path. The controllers call the individual functions directly for more control.

---

## Action Mode — Drawing Server + Dispatcher

**Launch mode:** `mode=action` (recommended for new development)

The action mode decouples **what to draw** from **how to draw it**. A dispatcher node generates drawing paths and sends them as goals to an action server, which pre-computes the entire motion and executes it.

### Architecture

```
┌─────────────────────────┐    ExecuteDrawing     ┌──────────────────────────┐
│   DrawingDispatcher      │───────action──────────►│  DrawingActionServer     │
│                          │                        │                          │
│ • Reads plane JSON       │◄───feedback────────────│ • Pre-computes all       │
│ • Generates UV waypoints │  (phase + progress)    │   phases in joint-space  │
│ • Maps to 3D positions   │                        │ • TOTG for joint timing  │
│ • Sends goals            │◄───result──────────────│ • Executes sequentially  │
│ • (continuous mode:      │  (success/failure)     │ • Safe shutdown: retract │
│    re-sends after each)  │                        │   + home on Ctrl-C       │
└─────────────────────────┘                        └──────────────────────────┘
                                                             │
                                                    ┌────────┴────────┐
                                                    ▼                 ▼
                                            /isaac_joint_   /scaled_joint_
                                            commands (sim)  trajectory_ctrl
                                                            (real robot)
```

### Action Interface — `ExecuteDrawing.action`

```
# Goal
geometry_msgs/Point[]      waypoints       # Cartesian drawing path (base_link frame)
geometry_msgs/Quaternion   orientation     # Fixed EE orientation during drawing

# Result
bool     success
string   message

# Feedback
string   current_phase        # RETRACT_UP, HOMING, RRT, DESCENT, DRAWING, ASCENT
float32  drawing_progress     # 0.0 → 1.0 (meaningful during DRAWING phase)
```

### Drawing Action Server (`drawing_action_server.py`)

**Node name:** `drawing_action_server`

Receives goals via the `execute_drawing` action and pre-computes the full motion before any movement begins. Each goal executes through:

```
RETRACT_UP → [HOMING] → RRT → DESCENT → DRAWING → ASCENT
```

The first goal includes HOMING (retract home + hold); subsequent goals skip it since the robot is already hovering from the previous ascent.

#### Pre-Computation Pipeline

For each incoming goal:
1. **Dynamic TCP pose generation** — Convert each tip waypoint to a tool-aware wrist pose (dynamic yaw + tool offset)
2. **Cartesian IK** — Solve IK for all drawing waypoints + descent/ascent segments
3. **RRT planning** — Joint-space path from current hover to approach pose via `rrt_connect` + `smooth_path` + `bezier_smooth_path`
4. **Phase timing** — Each phase gets time-stamped:
   - **Joint-space phases** (retract_home, rrt, home_hold): sent to the TOTG service for time-optimal timing with continuous acceleration
   - **Cartesian phases** (retract_up, descent, drawing, ascent): trapezoidal velocity profile in Cartesian space, then per-waypoint IK
5. **Concatenation** — All phases joined into a single timed trajectory (positions + velocities + timestamps)

#### Execution

- **Sim:** Plays back pre-computed trajectory at wall-clock rate using `time.monotonic()` (avoids dependency on `/clock` topic)
- **Real robot:** Sends phase trajectories as multi-point `JointTrajectory` messages to the UR controller's `scaled_joint_trajectory_controller`, then monitors convergence

#### Key Design Decisions

| Decision | Why |
|----------|-----|
| Pre-compute everything before moving | Eliminates mid-execution IK failures |
| TOTG for joint-space timing | Guarantees continuous acceleration — no speed jumps |
| Spin-free service calls (`future.done()` polling) | Avoids nested executor deadlocks in MultiThreadedExecutor |
| Wall-clock execution timing | Works regardless of sim time / `/clock` availability |
| TOTG fallback to IPTP | If the TOTG service is unavailable, uses a Python fallback |

### Drawing Dispatcher (`drawing_dispatcher.py`)

**Node name:** `drawing_dispatcher`

Reads the plane JSON, generates drawing waypoints based on the `trajectory_key` parameter, and sends them to the action server.

**Modes:**
- **Single** (default): Sends one drawing goal, waits for result, exits.
- **Continuous** (`continuous:=true`): After each drawing completes, sends the next goal. Currently replays the same path; extensible to random patterns, GUI input, etc.

**Resilience:**
- Waits for the action server with exponential backoff
- Retries on transient failure with a 0.5s dispatch delay
- Graceful shutdown on Ctrl-C

---

## TOTG Service Node (totg_service_node.cpp)

**Node name:** `totg_service_node`
**Service:** `compute_totg` (`sand_drawer/srv/ComputeTOTG`)

A lightweight C++ node that wraps MoveIt 2's **Time-Optimal Trajectory Generation** algorithm (Kunz & Stilman, 2012, Georgia Tech). It takes raw joint-space waypoints and returns a fully time-parameterized trajectory with positions, velocities, and timestamps.

### Why TOTG?

The RRT planner produces geometrically smooth paths (via Catmull-Rom splines), but has **no timing information**. Naïve constant-speed playback or simple trapezoidal timing causes:
- Speed jumps at waypoint transitions
- Unnecessary slowdowns on straight segments
- Jerky acceleration profiles

TOTG solves a constrained optimization problem that finds the **mathematically fastest** traversal of the geometric path while respecting per-joint velocity and acceleration limits. The output has **continuous acceleration** — no speed jumps by construction.

### Algorithm

Uses the low-level `trajectory_processing::Path` + `trajectory_processing::Trajectory` API:

```
1. Parse flattened waypoints into Eigen vectors
2. Filter near-duplicate consecutive waypoints (norm < 1e-6)
3. Path::create(waypoints, path_tolerance)     // corner blending
4. Trajectory::create(path, max_vel, max_acc)  // time-optimal parameterisation
5. Guard against NaN/invalid duration
6. Resample at resample_dt intervals           // uniform output timestep
7. Return positions + velocities + timestamps
```

**No robot model required** — the algorithm operates purely on joint-space geometry with user-supplied velocity/acceleration limits.

### Service Interface — `ComputeTOTG.srv`

```
# Request
float64[]  waypoints_flat      # [q0_j0, q0_j1, ..., q0_j5, q1_j0, ...]
uint32     num_joints           # 6 for UR5e
float64[]  max_velocity         # per-joint rad/s limits
float64[]  max_acceleration     # per-joint rad/s² limits
float64    path_tolerance       # corner blending radius (rad), default 0.1
float64    resample_dt          # output timestep (s), default 0.01

# Response
float64[]  timed_positions_flat  # resampled positions (same layout)
float64[]  timed_velocities_flat # resampled velocities
float64[]  timestamps            # seconds from trajectory start
uint32     num_output_points
bool       success
string     message
```

### Robustness Features

- **Near-duplicate filtering:** Removes consecutive waypoints closer than 1e-6 rad (prevents `Path::create` from failing on degenerate segments)
- **NaN/invalid duration guard:** If TOTG produces NaN or non-positive duration (can happen with degenerate geometry), returns an error instead of crashing
- **Degenerate path handling:** Single-waypoint or all-identical paths return a single-point result

---

## Cartesian Controller (cartesian_square_controller.py)

**Node name:** `cartesian_draw_controller`
**Mode:** `mode=cartesian` in launch file
**Main idea:** Every motion phase is resolved to joint-space positions via our own IK. No dependency on MoveIt at runtime.

### Key Internal Functions

| Function | Called during | Purpose |
|----------|-------------|---------|
| `_load_plane_json()` | Startup | Reads JSON, applies Rz(π) frame correction if needed, builds draw_positions list |
| `_retract_up()` | RETRACT_UP | Cartesian vertical lift via IK |
| `_retract_home()` | RETRACT_HOME | Cosine-interpolated joint path to home |
| `_plan()` | PLANNING | Constrained IK + RRT + smooth + Bezier → joint path |
| `_execute()` | EXECUTING | Play RRT path (one-by-one or full trajectory) |
| `_descend()` | DESCENDING | Cartesian descent to surface |
| `_draw()` | DRAWING | Dense Cartesian path following with IK |
| `_ascend()` | ASCENDING | Cartesian lift off surface |
| `_ik(T, q_seed)` | All Cartesian phases | IK with **joint-jump rejection** + **elbow-up enforcement** |
| `_constrained_ik_for_pose(T)` | PLANNING | Multi-seed IK (30+ seeds), returns best elbow-up solution closest to home |
| `_config_ok(q)` | IK validation | Checks shoulder_lift and elbow within safe ranges |
| `_pub_joints(q)` | Every tick | Velocity-clamped publish (sim) or single-point trajectory (real) |
| `_ik_solve_cartesian_path(wps, seed)` | Real-robot batch solve | Solves IK for entire Cartesian path at once |
| `_send_cartesian_as_trajectory(wps, seed)` | Real-robot trajectory | Batch IK → trapezoidal timing → one JointTrajectory message |
| `_compute_joint_path_timing(path)` | All full-trajectory sends | Trapezoidal velocity profile timestamps |
| `_send_full_trajectory(path, times)` | Real robot | Publishes multi-point JointTrajectory |
| `_interpolate_cartesian_smooth(...)` | Cartesian phase setup | Trapezoidal velocity profile in Cartesian space |

### Cartesian Interpolation — Trapezoidal Velocity Profile

For each Cartesian segment (point A → point B), the interpolator uses a **trapezoidal velocity profile**:

```
velocity
   ▲
   │     ┌──────────┐
 v_max ──│          │
   │    /│          │\
   │   / │          │ \
   │  /  │          │  \
   │ /   │          │   \
   │/    │          │    \
   ├─────┼──────────┼─────►  time
   0   t_accel   t_cruise  t_total
```

- **Acceleration phase:** ramp up to `v_max` at rate `a_max`
- **Cruise phase:** hold `v_max`
- **Deceleration phase:** ramp down to 0
- **Short segments:** If distance < `v_max²/a_max`, uses a **triangle profile** (never reaches full speed)

Parameters: `max_linear_vel` (0.05 m/s default), `max_linear_accel` (0.05 m/s²)

### IK Safety: Joint-Jump Rejection

Each IK solution is checked against the previous joint state:

```python
if max(|q_new - q_seed|) > max_joint_step (0.15 rad ≈ 8.6°):
    REJECT  # likely configuration flip → collision risk
```

This prevents the IK from suddenly jumping to an elbow-down or shoulder-flipped solution.

### IK Safety: Elbow-Up Constraints

```python
shoulder_lift_min (-2.5) ≤ q[1] ≤ shoulder_lift_max (0.0)
elbow_min (-3.14)        ≤ q[2] ≤ elbow_max (-0.3)
```

These keep the arm in an elbow-up posture to avoid the table/workpiece.

---

## Velocity Controller (planar_servo_controller.py)

**Node name:** `planar_servo_controller`
**Mode:** `mode=trajectory` (default) or `mode=teleop`

### Phase Differences from Cartesian Controller

Phases RETRACT_UP through DESCENDING work **identically** to the Cartesian controller (same FK/IK/RRT pipeline). The difference is the on-surface phase:

**SERVO phase:** PD velocity control
```
position_error = target_waypoint - current_EE_position
velocity_cmd = Kp × error + Kd × d(error)/dt
z_correction = z_correction_gain × off_plane_distance
```

The resulting velocity is published as a `Twist` on `/end_effector_velocity`, which the C++ Jacobian calculator node converts to joint commands.

**TELEOP phase:** Same PD loop but the target comes from keyboard input on `/teleop_plane_vel` instead of pre-defined waypoints.

### Key Differences from Cartesian Controller

| Aspect | Cartesian Controller | Velocity Controller |
|--------|---------------------|-------------------|
| On-surface motion | Position IK per waypoint | PD velocity via Jacobian node |
| Requires `jacobian_calculator_node` | No | Yes |
| Drawing accuracy | Higher (direct IK) | Lower (velocity integration drift) |
| Singularity handling | IK damping only | Jacobian damping + nullspace optimization |
| Teleop support | No | Yes |

---

## Jacobian IK Node (jacobian_calculator_node.cpp)

**Only used with the velocity controller** (`mode=trajectory` or `mode=teleop`).

Subscribes to `/end_effector_velocity` (Twist) and `/isaac_joint_states` (joint feedback). Publishes `/isaac_joint_commands` (joint positions — position mode).

### Algorithm

```
1. Get current Jacobian J from MoveIt's RobotState
2. Compute manipulability μ = √det(JJᵀ)
3. Adaptive damping: λ = λ_ref × (1 - μ/μ_ref)  when μ < μ_ref
4. Pseudoinverse: J† = Jᵀ(JJᵀ + λ²I)⁻¹
5. Joint velocities: dq = J† × twist
6. Nullspace projection: N = (I - J†J)
7. Nullspace term 1: manipulability gradient ascent (push away from singularities)
8. Nullspace term 2: posture optimization (drive joints toward mid-range)
9. dq_total = dq + N × (w₁ × manip_gradient + w₂ × posture_correction)
10. Clamp to max_joint_velocity, integrate to positions, publish
```

**Singularity slowdown:** When μ < `slowdown_mu_threshold`, all velocities are scaled by μ/threshold, providing graceful degradation near singularities.

---

## Plane Capture — Simulation vs Real Robot

### Simulation: `plane_solver_node.py` (`mode=capture`)

- Subscribes to `/red_ball/ground_truth` (PointStamped from Isaac Sim)
- TF-transforms each point from `world` to `base_link`
- Service `~/capture_point` records the latest point
- After 4 points: auto-solves plane and saves JSON
- `target_frame` in JSON = `"base_link"` (matches FK directly)

### Real Robot: `freedrive_plane_capture.py` (`mode=freedrive_capture`)

- Enables freedrive (hand-guidance) mode via UR controller manager
- Reads TCP pose from `/tcp_pose_broadcaster/pose`
- Service `~/capture_point` records current TCP position
- After 4 points: auto-solves plane and saves JSON
- `target_frame` in JSON = `"base"` (UR driver's frame — **180° rotated from base_link**)

### Plane Fitting Algorithm (identical in both)

Given 4 points P1, P2, P3, P4:
1. `x_raw = P2 - P1` (first edge), `y_raw = P4 - P1` (second edge)
2. `normal = normalize(x_raw × y_raw)` (surface normal)
3. `x_axis = normalize(x_raw)`, `y_axis = normalize(normal × x_axis)` (orthogonalised)
4. Project all 4 points onto x_axis/y_axis to find bounding rectangle
5. Inscribe a square at `square_scale` (default 0.8 = 80% of the bounded area)
6. Map `vector_path_uv` onto the rectangle to get world-space trajectory

---

## Real Robot Bridging

When `real_robot:=true`:

### Feedback Flow
```
Real UR5e ──/joint_states──► controller (primary feedback source)
                                │
                                ├──► IK uses real joints as seed
                                ├──► convergence checking against real
                                └──► mirror real→sim: publish real joints
                                     to /isaac_joint_commands so Isaac Sim
                                     tracks the physical robot
```

### Command Flow — Full Trajectory Approach

**Problem solved:** Sending individual single-point JointTrajectory messages at 25-100 Hz causes constant start-stop on the UR controller, creating acceleration spikes that trigger safety faults (C306A0).

**Solution:** For every motion phase, the controller:
1. Pre-computes the entire path (Cartesian IK batch or joint interpolation)
2. Computes **trapezoidal timing** for each segment (respecting speed + accel limits)
3. Sends **one multi-point JointTrajectory** message to `/scaled_joint_trajectory_controller/joint_trajectory`
4. Then monitors convergence by comparing the real robot's joint state to the path's final waypoint

### Trapezoidal Timing (`_compute_joint_path_timing`)

For each consecutive pair of waypoints, the maximum joint displacement determines the segment time:

```python
delta = max(|q[i+1] - q[i]|)  # worst-case joint

if delta ≤ v_max²/a_max:
    # Triangle profile (can't reach full speed)
    dt = 2 × √(delta / a_max)
else:
    # Trapezoidal profile
    dt = delta/v_max + v_max/a_max
```

This guarantees every segment respects both `max_joint_speed_deg` (45°/s) and `max_joint_accel_deg` (40°/s²).

### Convergence Gating

The controller does NOT advance to the next phase until:
```python
max(|target_q - real_q|) < 2°  (real_converge_tol)
```

This ensures the physical robot has actually reached the target before the controller assumes the motion is complete.

### Sim Mirroring

While the real robot executes, `_mirror_real_to_sim()` continuously publishes the real robot's actual joint positions to `/isaac_joint_commands`, keeping the Isaac Sim visualization in sync.

---

## Safety Systems

### 1. Safe Homing Sequence (RETRACT_UP → RETRACT_HOME)
- Before any homing motion, the EE lifts straight up by `retract_height` (0.15 m)
- This prevents the arm from sweeping through the workspace or hitting the table
- Only then does it interpolate to the home configuration

### 2. Joint Speed Limiting (45°/s default)
Every joint command is velocity-clamped:
```python
max_allowed_delta = max_joint_speed_rad × dt  # per tick
if max(|q_new - q_prev|) > max_allowed_delta:
    scale down all joints proportionally
```

### 3. Joint Acceleration Limiting (40°/s² default)
Built into the trapezoidal timing for full trajectories. The UR controller itself also enforces acceleration limits.

### 4. Elbow-Up Constraints
IK solutions where `shoulder_lift > 0` or `elbow > -0.3` are rejected. This prevents table collisions.

### 5. Joint-Jump Rejection
IK solutions where any joint moves > `max_joint_step` (0.15 rad ≈ 8.6°) from the seed are rejected. This prevents configuration flips.

### 6. IK Failure Skip
If IK fails for a waypoint, the controller logs a warning and skips to the next waypoint rather than stopping.

---

## Parameter Reference

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `mode` | `trajectory` | `action` / `trajectory` / `teleop` / `cartesian` / `capture` / `freedrive_capture` |
| `real_robot` | `false` | Enable dual sim+real bridging |
| `loop` | `false` | Loop the drawing trajectory (cartesian/trajectory modes) |
| `continuous` | `false` | Continuous drawing dispatch (action mode) |
| `active_tool` | `pointy` | Action mode tool face (`fork` / `pointy` / `empty` / `spatula` / `orthogonal`) |
| `orthogonal_tool_length_m` | `0.13` | Tool length used when `active_tool=orthogonal` |
| `trajectory_key` | `projected_vector_trajectory` | Which JSON key to follow (`line` / `square_trajectory` / `projected_vector_trajectory`) |
| `plane_json` | `<auto>` | Path to plane JSON file |
| `use_sim_time` | `true` | Use `/clock` topic (auto-disabled when `real_robot=true`) |
| `max_joint_speed_deg` | `45.0` | Max joint speed (°/s) for all joints |
| `max_joint_accel_deg` | `40.0` | Max joint acceleration (°/s²) for trapezoidal timing |
| `line_u_start` | `0.5` | Line start U coordinate (0→1) |
| `line_v_start` | `0.3` | Line start V coordinate (0→1) |
| `line_u_end` | `0.5` | Line end U coordinate (0→1) |
| `line_v_end` | `0.7` | Line end V coordinate (0→1) |
| `kp_linear` | `1.5` | Position P-gain (velocity controller only) |
| `kd_linear` | `0.0` | Position D-gain (velocity controller only) |
| `kp_angular` | `1.5` | Orientation P-gain (velocity controller only) |
| `kd_angular` | `0.0` | Orientation D-gain (velocity controller only) |

### Cartesian Controller Parameters (set in launch file)

| Parameter | Default | Where to change | Effect |
|-----------|---------|-----------------|--------|
| `approach_height` | 0.08 m | launch file | Height above surface for approach/ascent |
| `max_linear_vel` | 0.05 m/s | launch file | Cruising speed for Cartesian interpolation |
| `max_linear_accel` | 0.05 m/s² | launch file | Acceleration for trapezoidal Cartesian profile |
| `ik_damping` | 0.05 | launch file | λ in damped-least-squares IK |
| `execution_hz` | 100 Hz | launch file | Control loop rate |
| `max_joint_step` | 0.15 rad | launch file | Joint-jump rejection threshold |
| `shoulder_lift_max` | 0.0 rad | launch file | Upper bound for shoulder_lift (elbow-up) |
| `shoulder_lift_min` | -2.5 rad | launch file | Lower bound for shoulder_lift |
| `elbow_max` | -0.3 rad | launch file | Upper bound for elbow (elbow-up) |
| `elbow_min` | -3.14 rad | launch file | Lower bound for elbow |
| `ik_num_seeds` | 30 | launch file | Number of random IK seeds in constrained solver |

### Action Server Parameters (set in launch file)

Inherits all Cartesian Controller parameters above, plus:

| Parameter | Default | Where to change | Effect |
|-----------|---------|-----------------|--------|
| `max_linear_vel` | 0.07 m/s | launch file | Drawing Cartesian speed (slightly faster than cartesian mode) |
| `max_linear_accel` | 0.03 m/s² | launch file | Drawing Cartesian acceleration |
| `approach_linear_vel` | 0.04 m/s | launch file | Approach/retract Cartesian speed |
| `approach_linear_accel` | 0.03 m/s² | launch file | Approach/retract Cartesian acceleration |
| `max_joint_speed_deg` | 90.0 °/s | launch file | Per-joint speed limit for TOTG and trapezoidal timing |
| `max_joint_accel_deg` | 40.0 °/s² | launch file | Per-joint acceleration limit for TOTG |
| `totg_path_tolerance` | 0.1 rad | launch file | TOTG corner blending radius (~5.7° deviation allowed) |
| `totg_resample_dt` | 0.01 s | launch file | TOTG output timestep (100 Hz matches execution_hz) |
| `active_tool` | `pointy` | launch arg | Tool face used in dynamic TCP IK (`fork`/`pointy`/`empty`/`spatula`/`orthogonal`) |
| `orthogonal_tool_length_m` | `0.13` | launch arg | Tool length used for `active_tool:=orthogonal` |

### Velocity Controller Additional Parameters

| Parameter | Default | Where to change | Effect |
|-----------|---------|-----------------|--------|
| `kp_linear` | 1.5 | launch arg | Position error → velocity gain |
| `kd_linear` | 0.0 | launch arg | Velocity error damping |
| `kp_angular` | 1.5 | launch arg | Orientation error → angular velocity |
| `kd_angular` | 0.0 | launch arg | Angular velocity damping |
| `max_linear_vel` | 0.25 m/s | launch file | Max EE speed |
| `max_angular_vel` | 0.60 rad/s | launch file | Max EE rotation speed |
| `plane_z_correction_gain` | 2.0 | launch file | How hard EE is pushed back onto plane |
| `waypoint_threshold` | 0.03 m | launch file | Distance to consider waypoint reached |
| `descent_step` | 0.002 m | launch file | Per-tick descent distance |
| `teleop_speed` | 0.10 m/s | launch file | Teleop velocity per keypress |

### Jacobian Calculator Parameters (C++ node)

| Parameter | Default | Where to change | Effect |
|-----------|---------|-----------------|--------|
| `max_joint_velocity` | 1.5 rad/s | launch file | Per-joint velocity cap |
| `damping_mu_reference` | 0.02 | launch file | Manipulability below which damping activates |
| `slowdown_mu_threshold` | 0.01 | launch file | Below this μ, velocities are scaled down |
| `manipulability_gain` | 0.4 | launch file | Nullspace manipulability gradient weight |
| `posture_gain` | 0.4 | launch file | Nullspace posture optimization weight |
| `w2_manipulability` | 1.0 | launch file | Overall manipulability cost weight |
| `control_mode` | `position` | launch file | `position` (integrates to joint targets) or `velocity` |

---

## Launch Modes & Usage

### Build

```bash
cd ~/welding_cell_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sand_drawer --symlink-install
source install/setup.bash
```

### Mode 1: Action-Based Drawing (recommended)

```bash
# Simulation — single drawing
ros2 launch sand_drawer sand_drawer.launch.py mode:=action

# Simulation — select tool face
ros2 launch sand_drawer sand_drawer.launch.py mode:=action active_tool:=pointy
ros2 launch sand_drawer sand_drawer.launch.py mode:=action active_tool:=fork
ros2 launch sand_drawer sand_drawer.launch.py mode:=action active_tool:=empty
ros2 launch sand_drawer sand_drawer.launch.py mode:=action active_tool:=spatula

# Simulation — continuous drawing loop
ros2 launch sand_drawer sand_drawer.launch.py mode:=action continuous:=true

# Real robot — single drawing
ros2 launch sand_drawer sand_drawer.launch.py mode:=action real_robot:=true

# Draw a specific line
ros2 launch sand_drawer sand_drawer.launch.py mode:=action \
    trajectory_key:=line line_u_start:=0.2 line_v_start:=0.5 line_u_end:=0.8 line_v_end:=0.5
```

**What launches:** 3 nodes
1. `drawing_action_server` — pre-computes and executes trajectories
2. `totg_service_node` — C++ TOTG timing service (via MoveIt 2)
3. `drawing_dispatcher` — generates goals and sends them via the action interface

### Mode 2: Cartesian Drawing

```bash
# Simulation
ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian

# Real robot
ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian real_robot:=true

# Draw a line instead of the full trajectory
ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian trajectory_key:=line \
    line_u_start:=0.2 line_v_start:=0.5 line_u_end:=0.8 line_v_end:=0.5

# Loop continuously
ros2 launch sand_drawer sand_drawer.launch.py mode:=cartesian loop:=true
```

### Mode 3: Velocity Trajectory Following

```bash
ros2 launch sand_drawer sand_drawer.launch.py mode:=trajectory
ros2 launch sand_drawer sand_drawer.launch.py mode:=trajectory loop:=true
```

### Mode 4: Teleop (keyboard control)

```bash
# Terminal 1
ros2 launch sand_drawer sand_drawer.launch.py mode:=teleop

# Terminal 2
ros2 run sand_drawer plane_teleop_keyboard.py
```

### Mode 5: Plane Capture — Simulation

```bash
ros2 launch sand_drawer sand_drawer.launch.py mode:=capture
# Then call 4 times:
ros2 service call /plane_solver_node/capture_point std_srvs/srv/Trigger
```

### Mode 6: Plane Capture — Real Robot (Freedrive)

```bash
# Prerequisite: UR driver must be running
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.8.4 launch_rviz:=false

# Then in another terminal:
ros2 launch sand_drawer sand_drawer.launch.py mode:=freedrive_capture

# Hand-guide robot to each corner and call:
ros2 service call /freedrive_plane_capture/capture_point std_srvs/srv/Trigger
# After 4th point, plane auto-solves and saves.
```

---

## Tuning Guide

### TOTG timing produces speed jumps or jerky RRT motion (action mode)

This should not happen — TOTG guarantees continuous acceleration by construction. If you see jerkiness:
- Check that the TOTG service node is actually running (`ros2 service list | grep totg`)
- If the log shows `[IPTP-fallback]` instead of `[TOTG]`, the TOTG service failed to respond. Check the C++ node logs.
- Reduce `totg_path_tolerance` (try 0.05) for tighter path following at the cost of sharper corners
- Reduce `max_joint_speed_deg` / `max_joint_accel_deg` for more conservative motion

### TOTG corner blending is too aggressive

`totg_path_tolerance=0.1` rad ≈ 5.7° of corner deviation. This allows smooth blending through waypoints but the path may deviate from the original. Lower values (0.01–0.05) give tighter corners but may produce higher accelerations at transitions.

### Sim shows small vibrations that real robot doesn't

This is expected. Isaac Sim's physics timestep introduces jitter that the real UR controller's internal interpolator smooths out. The real robot has hardware-level trajectory smoothing. No tuning needed.

### Drawing is wobbly or drifting off the plane (velocity mode)

Increase `kp_linear` (try 2.0-3.0) and/or add `kd_linear` (try 0.1-0.3). If oscillating, reduce `kp_linear` or increase `kd_linear`.

### IK keeps failing during DRAWING

- The arm may be near a singularity. Try a different `approach_height` or change the plane orientation.
- Increase `ik_num_seeds` to find more solutions.
- Widen `shoulder_lift_max` / `elbow_max` constraints (but watch for table collisions).
- Reduce `max_joint_step` stricter (0.10) if jumps are the problem.

### Real robot triggers safety fault (C306A0)

- Reduce `max_joint_speed_deg` (try 30°/s)
- Reduce `max_joint_accel_deg` (try 25°/s²)
- These are conservative defaults but older UR firmware may have lower thresholds.

### RRT planning takes too long or fails

- The approach pose may be unreachable with elbow-up constraints. Try `shoulder_lift_max=0.3` or `elbow_max=0.0` temporarily.
- Increase `max_iter` in the code (currently 10000).
- Check that the plane isn't behind the robot or below the table.

### Drawing is 180° rotated from capture points

- This happens when freedrive capture writes `target_frame: "base"` and the controller doesn't correct it. Both controllers now auto-detect this and apply Rz(π) correction. If you have an old JSON file, either re-capture or manually change `"target_frame"` to `"base_link"` and negate X,Y of all positions.

### Home position

| Joint | Degrees | Radians |
|-------|---------|---------|
| shoulder_pan | -45° | -0.7854 |
| shoulder_lift | -25° | -0.4363 |
| elbow | -145° | -2.5307 |
| wrist_1 | -10° | -0.1745 |
| wrist_2 | 0° | 0.0 |
| wrist_3 | 0° | 0.0 |

To change the home position, edit `_home_positions` / `_home_joint_positions` in both controller scripts.