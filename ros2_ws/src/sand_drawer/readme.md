# Sand Drawer — UR5e Planar Servoing on Isaac Sim

Draw shapes on a flat surface using a UR5e robot arm in Isaac Sim. The end-effector (`tool0`) is constrained to a previously-defined rectangular plane while a Jacobian-based IK engine handles singularity avoidance and manipulability optimization.

## Project Structure

```
sand_drawer/
├── config/                  # Robot SRDF, kinematics, joint limits
├── generated_planes/
│   └── sand_drawer_plane.json   # Plane definition (from plane_solver_node)
├── launch/
│   └── sand_drawer.launch.py    # Main launch file
├── meshes/ur5e/             # Collision & visual meshes
├── scripts/
│   ├── plane_solver_node.py         # Stage 0: capture 4 points → plane JSON
│   ├── plane_frame_broadcaster.py   # Static TF: base_link → drawing_plane
│   ├── planar_servo_controller.py   # Trajectory / teleop controller
│   └── plane_teleop_keyboard.py     # Keyboard teleop for manual control
├── src/
│   └── jacobian_calculator_node.cpp # Twist → joint commands (Jacobian IK)
├── urdf/                    # UR5e URDF/xacro
└── world/                   # Isaac Sim USD scenes
```

## Data Flow

```
Isaac Sim ──/isaac_joint_states──► robot_state_publisher ──► TF tree
                                   plane_frame_broadcaster ──► TF: base_link → drawing_plane

planar_servo_controller (reads JSON + TF + /teleop_plane_vel)
    │  /end_effector_velocity (Twist)
    ▼
jacobian_calculator_node (damped pseudoinverse IK)
    │  /isaac_joint_commands (JointState)
    ▼
Isaac Sim
```

## Prerequisites

- Isaac Sim 4.5+ running with the welding world scene
- ROS 2 Jazzy (inside the dev container)
- The following Isaac Sim topics must be active:
  - `/isaac_joint_states` — `sensor_msgs/JointState`
  - `/isaac_joint_commands` — `sensor_msgs/JointState`
  - `/clock` — `rosgraph_msgs/Clock`

## Home Position (Isaac Sim)

Set these joint angles in the Isaac Sim Articulation properties before pressing Play:

| Joint               | Degrees | Radians  |
|---------------------|---------|----------|
| shoulder_pan_joint  |  -45    | -0.7854  |
| shoulder_lift_joint |  -25    | -0.4363  |
| elbow_joint         | -145    | -2.5307  |
| wrist_1_joint       |  -10    | -0.1745  |
| wrist_2_joint       |    0    |  0.0000  |
| wrist_3_joint       |    0    |  0.0000  |

The servoing pipeline works regardless of starting position though these angles give a good manipulability configuration for the working plane.

## Usage

Build:
```bash
cd ~/welding_cell_ws/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sand_drawer --symlink-install
source install/setup.bash
```

### Mode 1: Trajectory Following (default)

Follows the `projected_vector_trajectory` waypoints from the plane JSON:
```bash
ros2 launch sand_drawer sand_drawer.launch.py
```

Loop the trajectory continuously:
```bash
ros2 launch sand_drawer sand_drawer.launch.py loop:=true
```

Use the wider square trajectory:
```bash
ros2 launch sand_drawer sand_drawer.launch.py trajectory_key:=square_trajectory
```

### Mode 2: Teleop (manual keyboard control)

Launch the servoing pipeline in teleop mode — the EE moves to the center of the plane then waits for keyboard commands:
```bash
# Terminal 1 — launch the pipeline
ros2 launch sand_drawer sand_drawer.launch.py mode:=teleop

# Terminal 2 — send keyboard commands
ros2 run sand_drawer plane_teleop_keyboard.py
```

Keyboard controls:
```
   W           ↑ move +Y (plane Y axis)
 A S D       ← ↓ → move along plane X/Y axes
   Q           quit
```

### Mode 3: Plane Capture (one-time setup)

Capture 4 points using the red ball marker to define the plane:
```bash
ros2 launch sand_drawer sand_drawer.launch.py mode:=capture
```
Then call `ros2 service call /plane_solver_node/capture_point std_srvs/srv/Trigger` four times.

## Effective Parameters

### Planar Servo Controller

| Parameter                 | Default | Effect |
|---------------------------|---------|--------|
| `kp_linear`               | 0.5     | Position error gain → EE velocity. Higher = faster but risk overshoot |
| `kp_angular`              | 1.0     | Orientation correction gain |
| `max_linear_vel`          | 0.10    | Max EE speed in m/s |
| `max_angular_vel`         | 0.30    | Max angular speed in rad/s |
| `plane_z_correction_gain` | 2.0     | How hard the EE is pushed back onto the plane surface |
| `approach_height`         | 0.08    | Hover distance above the plane during APPROACH phase |
| `waypoint_threshold`      | 0.015   | Distance (m) to consider a waypoint reached |
| `boundary_margin`         | 0.01    | Safety inset (m) from the rectangle edges |
| `teleop_speed`            | 0.05    | Teleop velocity (m/s) per keypress |

### Jacobian Calculator (IK Engine)

| Parameter                | Default | Effect |
|--------------------------|---------|--------|
| `max_joint_velocity`     | 0.5     | Per-joint velocity limit (rad/s). Main safety cap |
| `damping_mu_reference`   | 0.05    | Below this manipulability μ, Jacobian damping activates |
| `slowdown_mu_threshold`  | 0.04    | Below this μ, joint velocities are scaled down proportionally |
| `w2_manipulability`      | 1.0     | Cost weight for manipulability optimization in nullspace |
| `manipulability_gain`    | 0.4     | Gain for nullspace manipulability gradient ascent |
| `posture_gain`           | 0.4     | Gain for nullspace mid-range posture optimization |
| `control_mode`           | position| `position` integrates to joint targets; `velocity` sends raw velocities |

----------

Current modes are: mode:= teleop, loop:=true, mode:= capture

"ros2 launch sand_drawer sand_drawer.launch.py mode:=teleop

