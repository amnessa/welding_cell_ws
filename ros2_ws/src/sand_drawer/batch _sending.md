The Literature: Global Planning vs. Local Planning

When dealing with massive trajectories, you asked: "Should we call TOTG for each batch separately, or call TOTG for the whole drawing phase and then divide?"

According to the Springer Handbook of Robotics (Chapter on Trajectory Generation), you must always use the second option: Global Planning, Chunked Execution.

Here is why:
If you divide the path into batches before calling the Time-Optimal Trajectory Generation (TOTG) service, the algorithm treats each batch as an isolated mission. By definition, a time-optimal trajectory must start at 0.0 rad/s and decelerate to exactly 0.0 rad/s at its final waypoint to guarantee safety.

If you cut a 10,000-point path into three TOTG batches, the robot will accelerate, cruise, and then violently slam on the brakes at the end of Batch 1. A millisecond later, it will aggressively accelerate into Batch 2. You will completely destroy the fluidity of the arc and introduce massive mechanical shuddering.

The Sensible Method:

    You pass the entire spatial geometry to TOTG all at once.

    TOTG generates a single, globally optimized C2 continuous speed profile for the entire 94-second sweep.

    You take that massive output array and cleanly slice it into smaller lists (e.g., 2,000 points each).

How to Implement "Trajectory Streaming" (Without Code)

If you slice the master array into batches, you cannot simply send Batch 1, wait for it to finish, and then send Batch 2. If you do, the robot will still stop at the seam because the controller's buffer briefly empties out.

To execute continuous batches, you must implement Trajectory Streaming:

    You send Batch 1 (points 0 to 2000) to the Action Server.

    You actively monitor the robot's elapsed execution time.

    When the robot reaches 80% completion of Batch 1 (e.g., point 1600), you immediately dispatch Batch 2 (points 2000 to 4000).

    The ROS 2 JointTrajectoryController is designed to handle this. When it receives a new trajectory message while already executing one, it looks at the exact timestamps. As long as the timestamps of Batch 2 perfectly line up with the end of Batch 1, it seamlessly appends them to the hardware buffer without the robot ever dropping velocity.