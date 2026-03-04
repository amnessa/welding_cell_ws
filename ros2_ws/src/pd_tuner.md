The Proposed Architecture: The Auto-Tuner System

To make this work, you will leave your existing planar_servo_controller mostly alone (just adding the Derivative term to its math) and create a brand new master script called the Auto-Tuner.
1. The Components

    The Simulation: Isaac Sim running your UR5e robot, listening to velocity commands.

    The Node: Your planar_servo_controller, which takes a target trajectory point, calculates the error, applies the Proportional and Derivative gains, and outputs velocity commands.

    The Auto-Tuner (New): A separate Python script that runs the Coordinate Descent algorithm. It acts as the puppet master.

2. The Auto-Tuner's Job (The Loop)

The Auto-Tuner script will execute this sequence repeatedly:

    Launch: It starts your sand_drawer.launch.py process, passing in the current guess for the Proportional and Derivative gains as launch arguments.

    Monitor: It subscribes End Effector's physical position and tracks the commanded speeds.

    Record: As the robot attempts to follow the discretized square trajectory, the Auto-Tuner records the distance to the target point and the change in velocity at every time step.

    Grade: Once the trajectory is finished, the Auto-Tuner kills the launch process and calculates the final cost.

    Adjust: It uses the Coordinate Descent rules to tweak the Proportional or Derivative gains up or down based on whether the final score improved or worsened.

    Repeat: It launches the simulation again with the new gains.

3. The Grading Rubric (The Cost Function)

When the run finishes, the Auto-Tuner calculates the final score using the composite logic we discussed. A lower score is better.

    Tracking Penalty: Calculate the Root Mean Square Error of the physical distance between the End Effector and the moving target point over the entire run. This ensures the robot actually stays on the path.

    Jiggle Penalty: Calculate the rate of change (acceleration) of the commands published to /end_effector_velocity at every step, square them, and sum them up. This heavily penalizes high-frequency oscillations.

    The Total Score: Cost = (Tracking Penalty) + (Weight * Jiggle Penalty)

        Note: You will need a weight multiplier for the jiggle penalty because acceleration values might be numerically much larger or smaller than your position error values.

4. The Constraints (The Failsafes)

While monitoring the run, if the Auto-Tuner detects that the End Effector has left the defined rectangular plane boundaries, or if the manipulability index drops too low, it immediately aborts the run and assigns a massive score (e.g., Cost = 99999). This tells the algorithm, "Never try these gains again."