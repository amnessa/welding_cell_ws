Here is the comprehensive development plan to transition your current `sand_drawer` architecture to the new discrete, action-based sequence.

This plan is structured to keep the geometric path planning strictly separated from the time parameterization, while using the operational space framing (following Oussama Khatib's methodology) to map these task-level commands directly to the robot's behavior.

### Phase 1: Define the Action Interface

Before touching the Python scripts, you need to define the exact communication contract between your trajectory generator and the controller.

1. **Create the Action File:** Create a new folder named `action` in your package and create a file named `ExecuteDrawing.action`.
2. **Define the Structure:**
* **Goal:** The geometric path. This should be an array of Cartesian poses or a specialized path message containing the 2D plane coordinates.
* **Result:** A boolean indicating successful completion of the sequence (ending at the "Ascent" phase).
* **Feedback:** The current phase of the state machine (e.g., "Rapidly-exploring Random Tree Planning", "Descent", "Drawing") and the percentage completion of the drawing itself.



### Phase 2: Refactor the Main Controller into an Action Server

Transform your existing planar controller (likely `planar_servo_controller.py` or `cartesian_square_controller.py`) from a continuous loop into a Robot Operating System Action Server.

1. **Initialize the Server:** Set up the Action Server to listen for `ExecuteDrawing` goals.
2. **Implement the Sequential State Machine:** Restructure your internal loop to match your exact specified sequence:
* **State 1 (Wait):** Idle, waiting for a goal.
* **State 2 (Retract Up):** Move the end effector safely above the sand/drawing plane.
* **State 3 (Home Hold / Rapidly-exploring Random Tree):** Plan the collision-free path to the starting coordinate of the newly received drawing geometry.
* **State 4 (Descent):** Lower the tool to the operational plane.
* **State 5 (Drawing):** Execute the time-parameterized trajectory.
* **State 6 (Ascent):** Lift the tool off the plane and return a `SUCCESS` result to the Action Client. Loop back to State 1.



### Phase 3: Decouple Geometry and Time Parameterization

When the Action Server receives the goal (the geometric path), it must process it before executing State 5.

1. **Geometric Ingestion:** Read the incoming array of points. This represents pure spatial geometry in the operational space.
2. **Apply the Time Law:** Write a function to generate a velocity profile (such as a trapezoidal velocity profile) over that spatial geometry. This function will assign a timestamp, velocity, and acceleration to each point, ensuring the robot respects its physical limits without distorting the shape of the drawing.
3. **Execution:** Pass this fully parameterized trajectory to the lower-level Cartesian position controller during the "Drawing" phase.

### Phase 4: Build the Client Dispatcher

To achieve continuous, automated fabrication, you need a node that feeds drawings to the robot.

1. **Create the Dispatcher Node:** Write a new Python script that acts as the Action Client.
2. **Queueing Logic:** This node should hold the list of all drawings (or read them from your generated files).
3. **Continuous Dispatch:** The client sends Drawing 1. It waits asynchronously. The moment the Action Server returns the `SUCCESS` result (meaning the "Ascent" phase is complete and the robot is safely hovering), the client immediately dispatches Drawing 2.

### Phase 5: Homing and Teardown

1. **Graceful Shutdown:** Implement a standard service call or a specific kill signal handler in the Action Server.
2. **Final Sequence:** When this signal is received, the state machine should force a final "Retract Up", followed by a "Homing" phase to park the robot safely before shutting down the controller nodes.

Would you like to start by writing the `ExecuteDrawing.action` definition, or would you prefer to look at refactoring the state machine inside your current Python controller first?