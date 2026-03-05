#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
import math
import sys

# Set to True to bypass speed scaling check and use the non-scaled controller
USE_UNSCALED_CONTROLLER = False

class UR5eTestMoveNode(Node):
    def __init__(self):
        super().__init__('ur5e_test_move')

        # Choose controller based on mode
        if USE_UNSCALED_CONTROLLER:
            topic = '/joint_trajectory_controller/joint_trajectory'
            self.get_logger().warn("Using NON-SCALED controller (bypasses teach pendant speed slider!)")
        else:
            topic = '/scaled_joint_trajectory_controller/joint_trajectory'
            self.get_logger().info("Using scaled controller (respects teach pendant speed)")

        # Publisher to the UR driver's trajectory controller
        self.publisher_ = self.create_publisher(
            JointTrajectory, topic, 10)

        # Subscriber to get current physical positions
        self.subscription = self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10)

        # Monitor speed scaling to warn the user
        self.speed_scaling = None
        self.speed_sub = self.create_subscription(
            Float64,
            '/speed_scaling_state_broadcaster/speed_scaling',
            self.speed_scaling_callback, 10)

        self.current_positions = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.command_sent = False

    def speed_scaling_callback(self, msg):
        self.speed_scaling = msg.data

    def joint_state_callback(self, msg):
        if self.current_positions is None and not self.command_sent:
            # Map the incoming joints to our expected order
            try:
                pos_dict = dict(zip(msg.name, msg.position))
                self.current_positions = [pos_dict[name] for name in self.joint_names]

                self.get_logger().info("Got current joint states:")
                for name, pos in zip(self.joint_names, self.current_positions):
                    self.get_logger().info(f"  {name}: {math.degrees(pos):.2f} deg")

                # Check speed scaling before proceeding
                if self.speed_scaling is not None and self.speed_scaling == 0.0 and not USE_UNSCALED_CONTROLLER:
                    self.get_logger().error("=" * 60)
                    self.get_logger().error("SPEED SCALING IS 0%! Robot will NOT move.")
                    self.get_logger().error("The External Control URCap program is likely NOT running.")
                    self.get_logger().error("On the teach pendant:")
                    self.get_logger().error("  1. Switch to Remote Control mode")
                    self.get_logger().error("  2. Load & play the External Control program")
                    self.get_logger().error("Or set USE_UNSCALED_CONTROLLER = True in this script")
                    self.get_logger().error("=" * 60)
                    raise SystemExit
                elif self.speed_scaling is not None:
                    self.get_logger().info(f"Speed scaling: {self.speed_scaling * 100:.0f}%")

                self.get_logger().info("READY ON THE E-STOP! Moving in 2 seconds...")

                # Trigger the move after a short safety delay
                self.create_timer(2.0, self.send_movement_command)
                self.command_sent = True
            except KeyError:
                pass # Still waiting for all 6 joints to be published

    def send_movement_command(self):
        if self.current_positions is None:
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Copy current positions, add 10 degrees to the Base joint (index 0)
        target_positions = list(self.current_positions)
        move_deg = 10.0
        target_positions[0] += math.radians(move_deg)

        point.positions = target_positions

        # Tell the robot it has 6 full seconds to complete this move (very slow/safe)
        point.time_from_start.sec = 6
        point.time_from_start.nanosec = 0

        msg.points = [point]

        self.get_logger().info(f"Publishing JointTrajectory to move Base Joint +{move_deg} degrees.")
        self.publisher_.publish(msg)

        self.get_logger().info("Command sent! You can Ctrl+C to exit.")
        # Cancel the timer so it only fires once
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = UR5eTestMoveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()