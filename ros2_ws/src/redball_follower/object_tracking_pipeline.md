Project: UR10 Real-Time Object Tracking with Isaac Sim & ROS 2

This plan outlines the steps to create a system where a UR10 robot, simulated in NVIDIA's Isaac Sim, uses its end-effector camera to visually track a moving object. We will use ROS 2 Humble as the middleware, MoveIt2 for motion planning to a starting pose, and a real-time visual servoing controller for the tracking behavior.
Phase 1: Environment Setup & Sanity Checks

The goal of this phase is to get all the software components installed and communicating with each other. This is the "Hello, World!" of your setup.

1. Docker Environment Setup:

    Action: Create a Docker environment for ROS 2 Humble. This isolates your dependencies and ensures reproducibility.

    Details:

        Pull the official ROS 2 Humble image: docker pull osrf/ros:humble-desktop.

✅        Create a docker-compose.yml file to manage the container. This makes it easy to handle X11 forwarding (for GUI apps like RViz) and network settings.

        Ensure your Docker container is on the same host network as Isaac Sim for easy communication (network_mode: "host").

✅ create an alias version of this and test

2. Isaac Sim & ROS 2 Bridge:

    Action: Install Isaac Sim and configure it to talk to ROS 2.

    Details:

        Install Isaac Sim via the Omniverse Launcher.

        Inside Isaac Sim, go to Window > Extensions and enable omni.isaac.ros2_bridge. This extension translates simulation data into ROS 2 messages and vice-versa.

        In the ROS 2 Bridge settings, ensure the "ROS Domain ID" matches your ROS 2 environment and the middleware is set correctly (the default for Humble is CycloneDDS) -> changed to fast .

3. ROS 2 Workspace & Dependencies:

    Action: Inside your Docker container, create a colcon workspace to hold your code and necessary robot packages.

    Details:

        Create a workspace directory (e.g., ~/ros2_ws/src).

        Clone the official Universal Robots ROS 2 description: git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git. This gives you the UR10's URDF (robot model).

✅(not needed)        Clone the ROS 2 drivers: git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git. We will use the "fake hardware" mode for simulation.

        Build your workspace with colcon build.

✅ 4. MoveIt2 Configuration:

    Action: Generate the configuration files that MoveIt2 needs to plan for the UR10.

    Details:

        Run the MoveIt2 Setup Assistant: ros2 launch moveit_setup_assistant setup_assistant.launch.py.

        Load your UR10 URDF file (it will be a .xacro file from the description package).

        Crucial Steps in Setup Assistant:

            Self-Collisions: Generate a comprehensive self-collision matrix. This prevents the robot from hitting itself.

            Planning Group: Create a planning group for the arm, typically named ur_manipulator or manipulator, that includes all 6 arm joints from base_link to wrist_3_link.

            Robot Poses: Define a "Home" or "Ready" named pose. This is a default, safe configuration.

            End-Effectors: You can define an end-effector group for the camera later if you wish.

        Generate the configuration package and place it in your src folder.

5. Initial Sanity Check:

    Action: Verify that all components are connected and working.

    Checklist:

        In Isaac Sim, load the UR10 robot asset.

        In your Docker terminal, launch the simulation driver and MoveIt2: ros2 launch ur_moveit_config ur_sim_control.launch.py ur_type:=ur10 use_fake_hardware:=true launch_rviz:=true.
✅ cyclonedds-cpp wont render robot model but fastdds does

        In RViz: You should see the UR10 model. In the left panel, under MotionPlanning, ensure the "Scene Geometry" and "Planned Path" displays are active.

        Test Connection: Move the robot joints manually in Isaac Sim. The robot model in RViz should move identically.

        Test MoveIt2: In the RViz MotionPlanning panel, select your ur_manipulator group. Drag the interactive orange sphere (the end-effector goal) to a new position. Click Plan. You should see a planned trajectory. Click Execute. The robot in RViz and Isaac Sim should perform the motion.

✅ Phase 2: Finding the Optimal Starting Pose

Before tracking, the robot should move to a "ready" pose that gives it the best ability to follow the object. This is a pose of high manipulability, far from singularities.

✅ 1. The Concept: Manipulability

    The Jacobian matrix (J) maps joint velocities to end-effector velocities.

    At a singularity, the robot loses the ability to move its end-effector in one or more directions (e.g., a fully stretched-out arm can't move further outwards).

    The manipulability index (μ = sqrt(det(J * J^T))) is a number that quantifies this. A high value means the robot can move easily in all directions. A value of 0 means it's at a singularity.

✅ 2. Implementation Plan:

    Action: Create a Python node that samples different robot configurations, calculates their manipulability, and finds the best one.

    Steps:

        Create a new ROS 2 package and a Python node within it.

        The node will use the tf2_ros library and robot kinematics (you can use a library like pykdl_utils or the kinematics solvers within MoveIt2) to calculate the Jacobian.

        Write a function that takes a set of joint angles, computes the Jacobian for the end-effector, and returns the manipulability index.

        Create a service that, when called, iterates through a grid of reachable poses, calculates the manipulability for each, and determines the joint configuration with the highest score.

        This "optimal" configuration becomes your go-to starting pose for the tracking task.

✅ Phase 3: Object Simulation & Perception

The robot needs to "see" the object to track it. We will simulate this entire perception pipeline.

1. Simulating the Target:

    Action: Create a moving ball in Isaac Sim.

    Details:

        Add a primitive shape (e.g., a Sphere) to the Isaac Sim stage. Give it a bright, unique color (e.g., red).

        Use Isaac Sim's Python scripting API to define a trajectory for the ball (e.g., moving in a circle on a plane).

2. Simulated Perception:

    Action: Add a camera to the robot and create a perception node to find the ball in the camera image.

    Details:

        Add Camera: Modify the UR10's URDF file to include a camera. Attach it rigidly to the wrist_3_link (the end-effector link).

        Publish Camera Data: In Isaac Sim, add the ROS 2 Camera helper to the camera object. Configure it to publish the /image_raw and /camera_info topics.

        Perception Node: Create a new Python node in your ROS 2 workspace.

            It will subscribe to /image_raw.

            Use OpenCV (cv_bridge to convert ROS images) to do simple color thresholding to find the red ball.

            Use OpenCV's moments function to find the center of the detected color blob.

            This node will publish the ball's (x, y) pixel coordinates to a new topic, e.g., /target_position.

✅ Phase 4: Implementing the Real-Time Tracking Controller

This is the core of the project. We will implement an Image-Based Visual Servoing (IBVS) controller. We will NOT use MoveIt for this part. MoveIt is for planning, not for high-frequency, reactive control.

1. The Concept: Visual Servoing
The goal is to drive the error between the object's current pixel position (p) and the desired position (p*, the center of the image) to zero.

    Control Law: v_c = -λ * L_p^+ * (p - p*)

        v_c: The desired velocity of the camera (and thus end-effector).

        λ: A proportional gain (a tuning parameter).

        L_p^+: The pseudo-inverse of the Image Jacobian, which relates changes in pixel coordinates to camera velocity. This matrix depends on the current pixel coordinates and an estimated depth to the object.

    Robot Control: Once we have v_c, we use the robot's geometric Jacobian (J) to convert it to the joint velocities (q_dot) needed to achieve that end-effector motion: q_dot = J^-1 * v_c.

2. Implementation Plan:

    Action: Create a "tracker" node that calculates joint velocities and sends them directly to the robot's controllers.

    Steps:

        Controller Setup: In your ur_sim_control.launch.py, ensure you load and start a joint velocity controller, like forward_velocity_controller.

        Tracker Node:

            Subscribes to /target_position (from Phase 3) and /joint_states.

            Continuously calculates the pixel error (p - p*).

            Estimates the Image Jacobian. Start with a fixed, reasonable guess for the object's depth (Z).

            Calculates the robot's geometric Jacobian using the current /joint_states.

            Computes the final required joint velocities q_dot using the control law.

            Publishes these velocities as a Float64MultiArray message directly to the topic the forward_velocity_controller is listening on.

TODO: Phase 5: Integration & Visualization

This final phase ties everything together into a cohesive application.

1. Master Launch File:

    Create a top-level launch file that starts everything in the correct order:

        The Isaac Sim driver and ros2_control with the velocity controller.

        MoveIt2 (so we can use it for the initial move).

        The perception node.

        The tracking node.

2. Operational Procedure & RViz Dashboard:

    Your final workflow will be:

        Launch Isaac Sim and your master launch file.

        In RViz, use the MotionPlanning panel to command the robot to its optimal starting pose (from Phase 2). Click Plan and Execute.

        Once the robot is in position, use a ROS 2 service call to "activate" your tracking node.

        The tracking node takes over, and the robot's end-effector should now follow the moving ball in Isaac Sim.

    RViz Configuration:

        Display the robot model and planning scene.

        Add an Image display to view the robot's camera feed in real-time.

        Add a Marker display to visualize the detected target position being published by your perception node. This is invaluable for debugging.