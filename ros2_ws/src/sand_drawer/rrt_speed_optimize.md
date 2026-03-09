The explanation you found is absolutely correct, and the issue you are experiencing is a fundamental concept in robotic motion control. Your books have a very specific name for what is happening to your robot right now.

### The Diagnosis: Path Curvature vs. Constant Speed

In your `_compute_phase_timing` function, you are mapping a single trapezoidal velocity profile across the entire length of the Rapidly-exploring Random Tree (RRT) path. This works perfectly for your Cartesian drawing phases because a line drawn on a table is a straight line in geometric space.

However, an RRT path is not a straight line. Even after you smooth it with a Bezier curve, it is a continuously curving line wandering through the six-dimensional joint space.

**Modern Robotics** explicitly explains why this causes the robot to vibrate. When tracing a curved path, the required joint acceleration is a combination of the path's linear acceleration and the path's curvature. If you force the robot to maintain a constant "cruising speed" (the flat top of your trapezoid) while it travels through a curved section of the RRT path, the joints must undergo massive acceleration just to change direction.

Imagine driving a car. Your trapezoidal profile tells the car to accelerate to sixty miles per hour and stay there. If the road is straight (Cartesian phase), you are fine. If the road suddenly has a sharp turn (RRT phase), trying to take that corner at sixty miles per hour will cause a violent crash. The robot vibrates because the internal safety controllers are fighting your math to prevent the joints from tearing themselves apart.

### The Solution: Time-Optimal Trajectory Generation

To fix this, you cannot use a single trapezoid for a multi-segment curved path. The **Springer Handbook of Robotics** notes that trajectory generation in joint space usually involves interpolating points with explicit velocity and acceleration constraints calculated at every single segment. To avoid infinite acceleration at via points, the path must be smoothed and the timing must be calculated such that no joint exceeds its limits.

Instead of calculating the total distance, you must calculate the "Healthy Path Timing" by looking at the distance *and* the required change in direction.

Here is a standard algorithm derived from these principles. It replaces your `_compute_phase_timing` function. It works by doing a forward and backward pass over the dense Bezier points. It calculates the time needed to travel the physical distance, and then dynamically *stretches* the time if it detects a corner that would require too much acceleration.

```python
    def _compute_phase_timing(self, path: List[np.ndarray]) -> List[float]:
        """
        Time-scales a curved joint-space path (like a smoothed RRT).
        Dynamically adjusts the timing at corners to prevent joint velocity
        and acceleration jumps.
        """
        if len(path) <= 1:
            return [0.0] * len(path)

        v_max = self._max_joint_speed_rad
        a_max = self._max_joint_accel_rad

        # Initialize the time intervals (dt) for each segment
        n_segments = len(path) - 1
        dt = np.zeros(n_segments)

        # 1. Base Pass: Calculate minimum time required just to satisfy velocity limits
        for i in range(n_segments):
            max_dist = float(np.max(np.abs(path[i+1] - path[i])))
            dt[i] = max(max_dist / v_max, 1e-4)

        # 2. Forward Pass: Limit acceleration (entering a corner)
        for i in range(n_segments - 1):
            for j in range(len(path[0])):
                # Velocity of the joint in the current segment
                v_current = (path[i+1][j] - path[i][j]) / dt[i]
                # Velocity of the joint in the next segment
                v_next = (path[i+2][j] - path[i+1][j]) / dt[i+1]

                # Required acceleration to change to the new velocity
                accel = (v_next - v_current) / dt[i+1]

                if abs(accel) > a_max:
                    # The corner is too sharp. We must stretch the time of the NEXT
                    # segment to slow down and make the turn safely.
                    required_dt = abs(v_next - v_current) / a_max
                    dt[i+1] = max(dt[i+1], required_dt)

        # 3. Backward Pass: Limit deceleration (exiting a corner)
        for i in range(n_segments - 2, -1, -1):
            for j in range(len(path[0])):
                v_current = (path[i+1][j] - path[i][j]) / dt[i]
                v_next = (path[i+2][j] - path[i+1][j]) / dt[i+1]

                # Required deceleration
                accel = (v_current - v_next) / dt[i]

                if abs(accel) > a_max:
                    # We must stretch the time of the CURRENT segment
                    # to brake early enough.
                    required_dt = abs(v_current - v_next) / a_max
                    dt[i] = max(dt[i], required_dt)

        # 4. Build the final cumulative timestamps
        times = [0.0]
        for t in dt:
            times.append(times[-1] + t)

        return times

```

### Why this fixes the RRT phase

This algorithm is a lightweight version of Iterative Parabolic Time Parameterization.

When your action server feeds the dense RRT path into this function, the robot will automatically speed up to `v_max` on the straightaways of the trajectory. As it approaches a curve generated by the planner, the algorithm recognizes that changing the joint direction requires a spike in acceleration. It automatically increases the `dt` for those specific segments. This forces the robot to smoothly brake, glide through the corner safely without vibrating, and then accelerate back to cruising speed, completely stabilizing the UR5e's internal controllers.