# Sand Drawer Pipeline (v1)

## Goal
Generate task-space trajectories on a plane extracted from simulation points (red ball), expressed in robot base frame.

## Stage 1: Plane capture and solve
Node: `plane_solver_node.py`

### Inputs
- Topic: `/red_ball/ground_truth` (`geometry_msgs/PointStamped`) by default
- TF: source frame (default `world`) to target frame (default `base_link`)

### Capture protocol
- Capture 4 points using service: `/plane_solver_node/capture_point`
- Point 1,2,3 define plane axes and normal
- Point 4 defines rectangle size in the same plane

### Services
- `/plane_solver_node/capture_point` (`std_srvs/Trigger`)
- `/plane_solver_node/solve_plane` (`std_srvs/Trigger`)
- `/plane_solver_node/reset_points` (`std_srvs/Trigger`)
- `/plane_solver_node/save_last` (`std_srvs/Trigger`)

### Outputs
JSON file (default `/tmp/sand_drawer_plane.json`) with:
- Captured points in base frame
- Plane basis (`x_axis`, `y_axis`, `normal`)
- Rectangle corners
- Closed square trajectory (5 waypoints) with orientation (`xyzw`) where EE z-axis is plane normal

## Stage 2: Task-space trajectory generation (next)
Use Lula Task Space Trajectory Generator:
1. Read `square_trajectory` from JSON
2. Build `target_positions` + fixed `target_orientations`
3. Call `compute_task_space_trajectory_from_points(...)`
4. If trajectory fails (`None`), report reachability/singularity issue

## Stage 3: Config-space safety shaping (next)
Apply limits/filters before execution:
- Joint velocity caps
- Joint acceleration/jerk constraints (from UR config)
- Optional waypoint densification near corners to reduce peak joint speeds
- Abort/fallback when IK discontinuity is detected

## Stage 4: Execution (next)
- Approach first waypoint from home via planner (RRT)
- Execute Lula-generated trajectory
- Return to safe pose
