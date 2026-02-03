# UR10 Mobile Manipulator Ball Tracking Project

This project contains the ROS2 pipeline for controlling a UR10 arm mounted on a mobile base to track a red ball.

## System Architecture

The system is composed of the following ROS2 nodes:

* **`ball_tracker_node`**: Subscribes to an image topic from a camera mounted on the UR10's end-effector. It uses OpenCV to detect a red ball and publishes the ball's 3D coordinates relative to the camera frame.
* **`jacobian_calculator_node`**: Subscribes to the robot's joint states and calculates the Jacobian matrix. It provides a service that, given a desired end-effector velocity, returns the required joint velocities.
* **`ur10_arm_controller_node`**: This is the main control node. It subscribes to the ball's 3D coordinates and the mobile base's odometry. It calculates the desired end-effector velocity to keep the camera pointed at the ball and calls the `jacobian_calculator_node` to get the necessary joint velocities, which it then sends to the robot.
* **`mobile_base_controller`**: A simple node that listens for keyboard input and publishes `geometry_msgs/Twist` messages to move the mobile base.
* **`teleop_twist_keyboard`**: A standard ROS2 package for keyboard control of the mobile base.

## Getting Started

### Prerequisites

* ROS2 Humble installed in a Docker container.
* Isaac Sim with ROS2 bridge enabled.
* A UR10 robot model with a camera and a mobile base set up in Isaac Sim.

### Build Instructions

1.  Create a new ROS2 workspace:
    ```bash
    mkdir -p ~/ur10_ws/src
    cd ~/ur10_ws/src
    ```

2.  Clone this repository into the `src` directory.

3.  Build the workspace:
    ```bash
    cd ~/ur10_ws
    colcon build
    ```

4.  Source the workspace:
    ```bash
    source ~/ur10_ws/install/setup.bash
    ```

### Running the Pipeline

1.  Start the Isaac Sim simulation with the UR10 on the mobile base.

2.  In your ROS2 Docker container, launch the pipeline:
    ```bash
    ros2 launch ur10_ball_tracker track_and_follow.launch.py
    ```

3.  In a new terminal in the ROS2 Docker container, run the keyboard controller for the mobile base:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
    ```

Now you can drive the mobile base with the arrow keys, and the UR10 arm will automatically adjust to keep the red ball in the center of the camera's view.

