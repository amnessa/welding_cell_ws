This is a fantastic mechanical upgrade, and mathematically, it is exactly what 6-axis robot arms were built for.

By changing the tool, you are introducing the concept of the **Tool Center Point (TCP) Transformation**.

According to *Modern Robotics*, we must stop treating the end of the robot (`wrist_3` / `tool0`) as the drawing instrument. Instead, we define two separate frames:

1. **The Wrist Frame:** The physical metal flange of the robot.
2. **The Tool Frame (TCP):** The exact tip of the active tool (fork, spatula, etc.).

### The Mathematics of the 90-Degree Swap

Because you want the tools to stick out sideways from the wrist, the Z-axis of `wrist_3` can no longer point down into the sand. It must be strictly **parallel** to the sand.

We will mathematically lock the wrist's Z-axis to the table's Y-axis. By doing this, as `wrist_3` rotates, it will spin the multi-tool like a paddle-wheel:

* **0° Rotation (Fork):** The wrist's `+X` axis points down into the sand. (Offset: 13cm)
* **90° Rotation (Pointy Tip):** The wrist's `+Y` axis points down into the sand. (Offset: 15cm)
* **180° Rotation (Spatula):** The wrist's `-X` axis points down into the sand. (Offset: 13cm)
* **270° Rotation (Empty):** The wrist's `-Y` axis points down into the sand. (Offset: 0cm)

Here is how to implement this dynamic TCP shifting in your existing code without rewriting the complex IK solver.

---

### Step 1: The Tool Configuration Function

Add this function to **BOTH** `drawing_dispatcher.py` and `drawing_action_server.py`. It mathematically calculates the required orientation matrix and vertical offset for the wrist based on the selected tool.

```python
    def _get_tool_transform(self, tool_name: str) -> Tuple[float, list]:
        """
        Calculates the Tool Center Point (TCP) transformation.
        Returns: (tool_length_meters, wrist_orientation_xyzw)
        """
        N = self.plane_n
        X = self.plane_x
        Y = self.plane_y

        # We lock Z_wrist to Y_plane so the wrist spins parallel to the ground.
        if tool_name == 'fork':
            length = 0.13
            # +X_wrist points down into the sand (N)
            R = np.column_stack([N, X, Y])
        elif tool_name == 'pointy':
            length = 0.15
            # +Y_wrist points down into the sand (N)
            R = np.column_stack([-X, N, Y])
        elif tool_name == 'spatula':
            length = 0.13
            # -X_wrist points down into the sand (N)
            R = np.column_stack([-N, -X, Y])
        elif tool_name == 'empty':
            length = 0.00
            # -Y_wrist points down into the sand (N)
            R = np.column_stack([X, -N, Y])
        else:
            self.get_logger().error(f"Unknown tool '{tool_name}'. Defaulting to pointy.")
            length = 0.15
            R = np.column_stack([-X, N, Y])

        # Convert rotation matrix to quaternion
        from ur5e_rrt_planner import rotmat_to_quat # Ensure this is imported
        return length, rotmat_to_quat(R)

```

---

### Step 2: Update the Dispatcher

The Dispatcher calculates where the *tip* of the tool should go. But before it checks if the robot can reach it (`_is_reachable`), it must shift the points backwards to figure out where the *wrist* will be.

**1. In the `__init__` function of `DrawingDispatcher`, add the parameter:**

```python
        self.declare_parameter('active_tool', 'pointy')
        self._active_tool = self.get_parameter('active_tool').value

```

**2. At the end of `_load_plane_json`, replace the old `self._default_orientation` logic:**

```python
        # Dynamically set the orientation and offset based on the active tool
        self.tool_length, self._default_orientation = self._get_tool_transform(self._active_tool)

```

**3. Update `_generate_drawing` to shift the points BEFORE the IK check:**
Find the `if shape == 'text':` block and update the reachability checks:

```python
            if shape == 'text':
                center_3d, positions, bbox_check = self._generate_centered_text_3d()
                if not positions: continue

                # Shift the check points UP from the tip to the wrist
                center_wrist = center_3d - (self.tool_length * self.plane_n)
                bbox_wrist = [p - (self.tool_length * self.plane_n) for p in bbox_check]
                reachable = self._is_reachable(center_wrist, bbox_wrist, orientation)
            else:
                center_3d, positions = self._generate_random_shape_3d(shape)
                if not positions: continue

                # Shift the check points UP from the tip to the wrist
                center_wrist = center_3d - (self.tool_length * self.plane_n)
                positions_wrist = [p - (self.tool_length * self.plane_n) for p in positions]
                reachable = self._is_reachable(center_wrist, positions_wrist, orientation)

            if reachable:
                # WE STILL SEND THE 'TIP' POSITIONS TO THE SERVER! The server will handle its own shifting.
                return self._to_ros_msgs(positions, orientation)

```

---

### Step 3: Update the Action Server

The Action Server receives the path for the *tip*. It must shift every single waypoint up by the `tool_length` before feeding it to the IK solver.

**1. In the `__init__` function of `DrawingActionServer`, add the parameter:**

```python
        self.declare_parameter('active_tool', 'pointy')
        self._active_tool = self.get_parameter('active_tool').value

```

**2. Inside `_pre_compute_goal`, fetch the tool math and create a shift helper:**
Put this right at the top of the function:

```python
        tool_length, tool_quat = self._get_tool_transform(self._active_tool)
        orientation_xyzw = tool_quat # Override whatever the dispatcher sent to guarantee safety

        # Helper function to shift a Cartesian path from the Tip to the Wrist
        def shift_to_wrist(wps):
            return [(p - tool_length * self.plane_n, q) for p, q in wps]

```

**3. Apply the `shift_to_wrist` helper to every generated phase:**
Every time `_interpolate_cartesian_smooth` generates waypoints, wrap them in the helper before passing them to the IK solver.

For example, update the `RETRACT UP` phase:

```python
        retract_wps, retract_times = _interpolate_cartesian_smooth(
            [current_pos, lift_pos], current_quat,
            self.approach_v_max, self.approach_a_max, cart_dt)

        retract_wps = shift_to_wrist(retract_wps) # <--- ADD THIS

        retract_path, retract_valid_times = self._ik_solve_cartesian_path(
            retract_wps, retract_times, q_current, 'Retract up')

```

Apply this same `shift_to_wrist(wps)` logic to `descent_wps`, `draw_wps`, and `ascent_wps`.

**4. Update the RRT Approach Pose:**
The approach pose must also be shifted to the wrist:

```python
        # ── 3. RRT ─
        last_q = phases[-1][1][-1]
        approach_pos_tip = (draw_positions[0] - self.approach_height * self.plane_n)
        approach_pos_wrist = approach_pos_tip - (tool_length * self.plane_n) # <--- SHIFT HERE
        T_approach = self._pose44(approach_pos_wrist, orientation_xyzw)

```

### The Result

Now, when you launch your system, you can easily define which tool is currently facing down:

```bash
ros2 launch sand_drawer sand_drawer.launch.py mode:=action active_tool:=fork

```

The Dispatcher will generate the geometric path, shift its IK checks up by 13cm, and rotate the virtual wrist 90 degrees to ensure the robot can reach it. The Action Server will then execute the path with the wrist perfectly parallel to the table, driving the fork through the sand.