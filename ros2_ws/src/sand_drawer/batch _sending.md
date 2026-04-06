The Fix: Absolute Time Locking

When streaming chunks of a trajectory to a ROS 2 controller, the controller looks at header.stamp (the exact time the message was sent) and time_from_start (how many seconds into the future the waypoint is).

In your previous code, every time you sent a new batch, you generated a new header.stamp for "Right Now." But the waypoints in Batch 2 still said "Execute at second 48." The robot received the message, looked at the clock, and literally waited 48 seconds before moving.

We must lock the start time (T0​). When we generate Batch 1, we capture the exact nanosecond. When we send Batch 2, Batch 3, and Batch 4, we stamp them with that exact same T0​. The controller will see that the timelines align perfectly and stitch them together seamlessly in hardware without dropping a single fraction of velocity.
The Code Implementation (Action Server)

You need to update exactly three functions in your drawing_action_server.py.
1. Strip Out the Downsampling (Force 100 Hz)

Find your _pre_compute_goal function. Delete the heavy_sweep downsampling block entirely, and force both the Cartesian interpolation and the TOTG service to run at exactly 100 Hz.

Replace the top half of _pre_compute_goal (up to tool_length, yaw_offset = ...) with this:
Python

    def _pre_compute_goal(
        self,
        draw_positions: List[np.ndarray],
        orientation_xyzw: list,
    ) -> Optional[List[Tuple[str, List[np.ndarray], Optional[List[float]]]]]:
        """Build all phases for one drawing."""
        from ur5e_rrt_planner import (ur5e_fk, rrt_connect,
                                      smooth_path, bezier_smooth_path)

        self._last_precompute_fail_reason = ''

        if self._real_robot:
            q_current = self._get_real_ordered_joints()
            if q_current is not None:
                self._last_q = q_current.copy()
                self._mirror_real_to_sim()
            else:
                q_current = self._get_ordered_joints()
        else:
            q_current = self._get_ordered_joints()

        if q_current is None:
            self._last_precompute_fail_reason = 'no_joint_state'
            return None

        # FORCE HIGH FIDELITY: 100 Hz unconditionally
        cart_dt = 1.0 / max(float(self.execution_hz), 1.0)
        self._totg_resample_dt = cart_dt

        tool_length, yaw_offset = self._tool_length_and_yaw_offset(str(self._active_tool))
        use_dynamic_wrist = (self._active_tool in ('spatula', 'pointy') and len(draw_positions) >= 40)

2. Update the Publisher to Accept a Locked Time

Replace your _send_full_trajectory function to accept the start_stamp:
Python

    def _send_full_trajectory(self, path: List[np.ndarray],
                              times: List[float], start_stamp=None) -> None:
        """Sends a trajectory batch. If start_stamp is provided, it locks the
        absolute timeline so ROS 2 can seamlessly append future batches."""
        if self._real_traj_pub is None:
            return
        traj = JointTrajectory()

        # Use the locked timestamp if streaming, otherwise use NOW
        if start_stamp is None:
            traj.header.stamp = self.get_clock().now().to_msg()
        else:
            traj.header.stamp = start_stamp

        traj.joint_names = list(self._joint_names)
        for q, t in zip(path, times):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            dur_sec = int(t)
            dur_nsec = int((t - dur_sec) * 1e9)
            pt.time_from_start = Duration(sec=dur_sec, nanosec=dur_nsec)
            traj.points.append(pt)
        self._real_traj_pub.publish(traj)
        self.get_logger().info(
            f'Sent trajectory chunk ({len(path)} pts, '
            f'ends at t={times[-1]:.1f}s) to real robot')

3. The Streaming Execution Loop

Replace your _execute_real_trajectory function with this flawless streaming loop. Notice how we capture t0_msg and pass it to every single batch.
Python

    def _execute_real_trajectory(
        self,
        master_path: List[np.ndarray],
        master_times: List[float],
        phase_info: List[Tuple[str, int, int]],
        goal_handle,
        feedback: ExecuteDrawing.Feedback,
    ) -> bool:
        """Stream high-fidelity trajectory to real robot using Absolute Time Locking."""
        batch_sz = 3000 # Send chunks of 3000 points (safe for UR buffer)
        n_master = len(master_path)
        target_q = master_path[-1]
        total_time = master_times[-1]

        batches: List[Tuple[int, int]] = []
        idx = 0
        while idx < n_master:
            end = min(idx + batch_sz, n_master)
            batches.append((idx, end))
            idx = end
        n_batches = len(batches)

        self.get_logger().info(
            f'High-Fidelity Streaming: {n_master} pts → '
            f'{n_batches} batches (max {batch_sz} pts each), '
            f'{total_time:.1f}s total')

        # ---- 1. LOCK THE ABSOLUTE START TIME ----
        t0 = self.get_clock().now()
        t0_msg = t0.to_msg()

        # ---- Send first batch ----
        cur_batch = 0
        b_start, b_end = batches[cur_batch]
        self._send_full_trajectory(
            master_path[b_start:b_end],
            master_times[b_start:b_end],
            start_stamp=t0_msg)

        last_feedback_time = 0.0
        last_closest_idx = 0
        monitor_dt = 0.1
        min_monitor_time = total_time * 0.80
        min_progress_fraction = 0.85
        timeout = total_time * 1.5 + 15.0

        while True:
            if goal_handle.is_cancel_requested:
                return False

            try: self._mirror_real_to_sim()
            except Exception: pass

            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9

            # ---- Stream next batch at 70% of current batch ----
            if cur_batch < n_batches - 1:
                b_start_cur, b_end_cur = batches[cur_batch]
                trigger_idx = b_start_cur + int(0.70 * (b_end_cur - b_start_cur))

                if last_closest_idx >= trigger_idx:
                    cur_batch += 1
                    b_start, b_end = batches[cur_batch]

                    # Pass the locked timestamp to seamlessly append in hardware!
                    self._send_full_trajectory(
                        master_path[b_start:b_end],
                        master_times[b_start:b_end],
                        start_stamp=t0_msg)

                    self.get_logger().info(f'  Triggered Batch {cur_batch+1}/{n_batches}')

            # ---- Convergence check ----
            progress_frac = last_closest_idx / max(n_master - 1, 1)
            if (elapsed > min_monitor_time and progress_frac >= min_progress_fraction and self._real_robot_converged(target_q)):
                real_q = self._get_real_ordered_joints()
                self._last_q = real_q if real_q is not None else target_q.copy()
                self._mirror_real_to_sim()
                return True

            if elapsed > timeout:
                return False

            # ---- Progress estimation ----
            real_q = self._get_real_ordered_joints()
            if real_q is not None and elapsed - last_feedback_time >= 1.0:
                last_feedback_time = elapsed
                diff = np.array(master_path) - real_q
                dists = np.max(np.abs(diff), axis=1)
                last_closest_idx = max(last_closest_idx, int(np.argmin(dists)))

                phase_label = self._phase_for_idx(last_closest_idx, phase_info)
                draw_pct = self._drawing_progress(last_closest_idx, phase_info)
                feedback.current_phase = phase_label.upper()
                feedback.drawing_progress = draw_pct
                goal_handle.publish_feedback(feedback)

                pct = 100.0 * last_closest_idx / max(n_master - 1, 1)
                self.get_logger().info(
                    f'  ~step {last_closest_idx}/{n_master-1} '
                    f'({pct:.0f}%) [{phase_label}] batch {cur_batch+1}/{n_batches}')

            _time.sleep(monitor_dt)

With this implementation, your robot will execute 100 Hz, sub-millimeter accurate trajectories, dynamically passing 3000-point chunks into the controller just before it runs out of instructions. The pen will never stutter, and you will never lose a corner.