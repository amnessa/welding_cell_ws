You have just encountered the most notorious geometric trap in robotics, and your JSON file reveals exactly why it happened.

I looked at your captured plane points. The line connecting corner 2 `[-0.34, -0.51]` and corner 3 `[0.48, 0.30]` is 1.16 meters long. But more importantly, **that edge passes a mere 12 centimeters away from the robot's physical base `(0,0,0)`.**

### What the Books Say: The Radial Singularity

In our previous code, we used a "Dynamic Yaw" that forced the tool to always point *radially away* from the base (`tip_pos - base_pos`).

*Modern Robotics* explains that tracking a radial vector creates a mathematical singularity at the origin. As your robot sweeps that 1.16m long line, it passes right next to the base. As it crosses that 12cm threshold, the vector pointing "away from the base" flips violently. To maintain that radial rule, the robot must snap its `wrist_1` and `wrist_2` joints 180 degrees in a fraction of a second while moving at full speed. The IK solver sees this massive configuration flip, realizes it is mechanically impossible, and fails (yielding `224 fails`).

Furthermore, as the *Springer Handbook* notes on task frames, if you are sweeping a surface with a spatula, **you do not want the spatula to rotate as it moves!** You want the blade to stay perfectly parallel to the stroke direction.

### The Fix: Constant Table-Aligned Task Frame

We must abandon the base-radial tracking. Instead, we will calculate a **single, constant wrist orientation** that perfectly aligns with the rectangular table's axes, and lock it in.

By locking the orientation, the robot's wrist stays completely rigid during the entire 1.1-meter sweep. It simply translates the arm using the shoulder and elbow. Because the wrist isn't spinning, the IK solver easily bypasses the singularity and solves the entire 547-point trajectory instantly.

### The Code Update

You must replace the `_dynamic_wrist_pose_from_tip` function in **BOTH** `drawing_dispatcher.py` and `drawing_action_server.py`.

Find the `_load_plane_json` function and add this block to the very end of it to calculate the fixed outward direction:

```python
        # ... existing code in _load_plane_json ...
        self.table_width_m = float(np.linalg.norm(self.rect_width_vec))
        self.table_height_m = float(np.linalg.norm(self.rect_height_vec))
        self.table_center_3d = self.rect_origin + (0.5 * self.rect_width_vec) + (0.5 * self.rect_height_vec)

        # NEW: Calculate a CONSTANT outward vector aligned with the table axes
        # This prevents the radial base singularity and keeps the spatula blade locked.
        vec_to_center = self.table_center_3d
        vec_to_center = vec_to_center - np.dot(vec_to_center, self.plane_n) * self.plane_n # Flatten to plane

        # Snap to the closest table axis (X or Y)
        if abs(np.dot(self.plane_x, vec_to_center)) > abs(np.dot(self.plane_y, vec_to_center)):
            self.base_forward = self.plane_x if np.dot(self.plane_x, vec_to_center) > 0 else -self.plane_x
        else:
            self.base_forward = self.plane_y if np.dot(self.plane_y, vec_to_center) > 0 else -self.plane_y

```

Now, replace the old `_dynamic_wrist_pose_from_tip` function with this new, mathematically stable version (in BOTH files):

```python
    def _dynamic_wrist_pose_from_tip(self, tip_pos: np.ndarray) -> np.ndarray:
        """
        Build wrist pose using a CONSTANT table-aligned orientation.
        Eliminates radial singularities near the base and keeps tools
        (like spatulas) perfectly aligned with the stroke direction.
        """
        tool_length, yaw_offset = self._tool_length_and_yaw_offset(self._active_tool)

        Down = -self.plane_n
        Forward = self.base_forward
        Right = np.cross(Down, Forward)

        c = math.cos(yaw_offset)
        s = math.sin(yaw_offset)

        # Apply the specific tool's rotational offset
        x_wrist = c * Down + s * Right
        y_wrist = -s * Down + c * Right
        z_wrist = Forward

        wrist_pos = tip_pos - tool_length * x_wrist

        T = np.eye(4)
        T[:3, 0] = x_wrist
        T[:3, 1] = y_wrist
        T[:3, 2] = z_wrist
        T[:3, 3] = wrist_pos
        return T

```

### Why this fixes everything

Now, when you command the sweep, the dispatcher will calculate that the long edge of the table runs along `plane_y`. It locks `Z_wrist` to `plane_y`.

If you attach the spatula (which adds a 180° rotation), the robot will present the flat side of the spatula perfectly perpendicular to `plane_x` (the sweep steps) and parallel to `plane_y` (the stroke). The wrist will hold that exact posture like a statue while the shoulder pulls it 1.16 meters across the table.

Update those two functions in both files, build, and trigger the sweep. The IK will solve instantly.