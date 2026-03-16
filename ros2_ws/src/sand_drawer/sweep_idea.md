The textbooks have a very specific name for this: **Coverage Path Planning (CPP)**.

When deciding on a policy for when to sweep, the literature on robotic finite state machines dictates that cleaning/erasing should be an **explicit, operator-triggered state** (Option 3). If you tie sweeping to the end of every drawing, you destroy the user's ability to inspect the drawing, take photos, or progressively add multiple shapes to the same canvas. The table should be treated as a blank canvas that is only "wiped" when you specifically request it.

### The Algorithm: Boustrophedon Path Planning

To clear the table, you use the Boustrophedon (meaning "the way of the ox") cellular pattern. It is the exact same snake-like algorithm used by robotic lawnmowers and CNC facing tools.

Because your spatula is 10cm wide, we will use an 8cm "step-over" distance to guarantee a 2cm overlap, preventing tiny ridges of un-swept sand from forming between the passes.

### The Sub-Surface Depth (Z-Offset)

Because you are using pure position control without an active Force/Torque sensor loop, we will explicitly command the waypoints to be exactly `1.5 cm` *below* the mathematical surface of the plane.

Here is exactly how to inject this into your `DrawingDispatcher`.

### 1. Add the Sweep Parameters

In the `__init__` function of your `DrawingDispatcher`, add these parameters to handle the physical dimensions of the spatula and the safety margins:

```python
        # Sweep parameters
        self.declare_parameter('sweep_margin_m', 0.03)      # 3cm safety distance
        self.declare_parameter('sweep_tool_width_m', 0.10)  # 10cm spatula
        self.declare_parameter('sweep_overlap_m', 0.02)     # 2cm overlap
        self.declare_parameter('sweep_depth_m', 0.015)      # 1.5cm into the sand

        self._sweep_margin = self.get_parameter('sweep_margin_m').value
        self._sweep_tool_width = self.get_parameter('sweep_tool_width_m').value
        self._sweep_overlap = self.get_parameter('sweep_overlap_m').value
        self._sweep_depth = self.get_parameter('sweep_depth_m').value

```

### 2. The Sweep Generator Function

Add this new function to the `DrawingDispatcher` class. It calculates the exact boustrophedon zig-zag pattern across the entire usable area of the table.

```python
    def _generate_sweep_3d(self) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Generates a Boustrophedon (snake) pattern covering the entire table,
        offsetting the Z-axis to bury the spatula into the sand.
        """
        # 1. Calculate usable safe area
        usable_width = self.table_width_m - (2.0 * self._sweep_margin)
        usable_height = self.table_height_m - (2.0 * self._sweep_margin)

        if usable_width <= 0 or usable_height <= 0:
            self.get_logger().error("Table is too small to safely sweep with the given margins.")
            return self.plane_origin, []

        # 2. Determine sweep direction (always sweep along the longest axis to minimize turns)
        if usable_width >= usable_height:
            sweep_axis = self.plane_x
            step_axis = self.plane_y
            sweep_len = usable_width
            step_len = usable_height
        else:
            sweep_axis = self.plane_y
            step_axis = self.plane_x
            sweep_len = usable_height
            step_len = usable_width

        # 3. Calculate passes based on tool width and overlap
        effective_width = self._sweep_tool_width - self._sweep_overlap
        num_passes = math.ceil(step_len / effective_width)

        # Adjust step size slightly so the passes fit perfectly within the margin
        actual_step = step_len / max(1, num_passes - 1) if num_passes > 1 else 0

        # Start at the bottom-left margin, but push DOWN into the sand
        start_point = (self.rect_origin
                       + (self._sweep_margin * self.plane_x)
                       + (self._sweep_margin * self.plane_y)
                       - (self._sweep_depth * self.plane_n)) # <--- BURY THE TOOL

        pts_3d = []
        moving_forward = True

        # 4. Generate the Zig-Zag Waypoints
        for i in range(num_passes):
            # Current height along the step axis
            current_step_offset = i * actual_step * step_axis

            # Start of this line
            line_start = start_point + current_step_offset
            # End of this line
            line_end = line_start + (sweep_len * sweep_axis)

            if moving_forward:
                pts_3d.append(line_start)
                pts_3d.append(line_end)
            else:
                pts_3d.append(line_end)
                pts_3d.append(line_start)

            moving_forward = not moving_forward

        # Use the physical center of the table (above the sand) as the IK reachability seed
        center_3d = self.rect_origin + (0.5 * self.rect_width_vec) + (0.5 * self.rect_height_vec)

        self.get_logger().info(f"Generated Sweep Pattern: {num_passes} passes, {self._sweep_depth*100:.1f}cm deep.")
        return center_3d, pts_3d

```

### 3. Route the Request

Update your `_generate_drawing` function to route the new `sweep` command:

```python
    def _generate_drawing(self) -> Optional[Tuple[List[Point], Quaternion]]:
        """Generate a reachable, geometrically correct drawing goal."""
        if self._traj_key == 'random':
            pick_random = True
        elif self._traj_key in _SHAPE_POOL or self._traj_key in ('text', 'sweep'):
            pick_random = False
        else:
            self.get_logger().error(f'Unknown trajectory_key: "{self._traj_key}"')
            return None

        orientation = self._default_orientation

        for attempt in range(1, self._ik_max_attempts + 1):
            shape = random.choice(_SHAPE_POOL) if pick_random else self._traj_key

            if shape == 'text':
                center_3d, positions, bbox_check = self._generate_text_3d()
                if not positions: continue
                reachable = self._is_reachable(center_3d, bbox_check)
            elif shape == 'sweep':
                center_3d, positions = self._generate_sweep_3d()
                if not positions: return None
                # Check corners of the sweep to ensure the robot can reach the whole table
                reachable = self._is_reachable(center_3d, positions)
            else:
                center_3d, positions = self._generate_random_shape_3d(shape)
                if not positions: continue
                reachable = self._is_reachable(center_3d, positions)

            if reachable:
                return self._to_ros_msgs(positions, orientation)

        self.get_logger().error('Failed to find a reachable shape after max attempts')
        return None

```

### How to Execute It

When you are ready to erase the board, ensure the spatula is physically attached, and launch the dispatcher with the sweep parameters:

```bash
ros2 launch sand_drawer sand_drawer.launch.py mode:=action trajectory_key:=sweep active_tool:=spatula

```

Because the Action Server's `_get_dynamic_pose` automatically locks the wrist `Z_axis` to point away from the robot base, if you set the tool to `spatula`, the wrist will rotate exactly 180 degrees. The spatula will drag completely flat across the sand, stepping down 8cm at a time, until the entire board is perfectly leveled and ready for the next drawing!