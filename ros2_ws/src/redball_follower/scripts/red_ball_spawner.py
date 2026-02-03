#!/usr/bin/env python3
"""
Red Ball Spawner - Dummy script for Isaac Sim

This script creates and moves a red ball in Isaac Sim for the UR5e robot to track.
It publishes the ball's position and can be controlled via ROS 2 topics.

TODO: Implement actual Isaac Sim integration using omni.isaac.core
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
import math


class RedBallSpawner(Node):
    """
    Dummy red ball spawner that simulates a moving red ball.
    In production, this will interface with Isaac Sim to spawn and move
    an actual red ball in the simulation.
    """

    def __init__(self):
        super().__init__('red_ball_spawner')

        # Parameters
        self.declare_parameter('radius', 0.05)  # Ball radius in meters
        self.declare_parameter('orbit_radius', 0.4)  # Orbit radius around robot
        self.declare_parameter('orbit_speed', 0.3)  # Radians per second
        self.declare_parameter('orbit_center_x', 0.5)  # Center X position
        self.declare_parameter('orbit_center_y', 0.0)  # Center Y position
        self.declare_parameter('orbit_center_z', 0.5)  # Center Z position (height)
        self.declare_parameter('publish_rate', 30.0)  # Hz

        self.ball_radius = self.get_parameter('radius').value
        self.orbit_radius = self.get_parameter('orbit_radius').value
        self.orbit_speed = self.get_parameter('orbit_speed').value
        self.center = Point()
        self.center.x = self.get_parameter('orbit_center_x').value
        self.center.y = self.get_parameter('orbit_center_y').value
        self.center.z = self.get_parameter('orbit_center_z').value

        # Publisher for ball ground truth position
        self.ball_position_pub = self.create_publisher(
            PointStamped,
            '/red_ball/ground_truth',
            10
        )

        # Timer for position updates
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.update_ball_position)

        self.start_time = self.get_clock().now()
        self.get_logger().info(
            f'Red Ball Spawner started - Orbit: radius={self.orbit_radius}m, '
            f'speed={self.orbit_speed}rad/s, center=({self.center.x}, {self.center.y}, {self.center.z})'
        )

    def update_ball_position(self):
        """Update and publish the ball's position along a circular orbit."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        angle = elapsed * self.orbit_speed

        # Circular motion in XY plane
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x = self.center.x + self.orbit_radius * math.cos(angle)
        msg.point.y = self.center.y + self.orbit_radius * math.sin(angle)
        msg.point.z = self.center.z + 0.1 * math.sin(angle * 2)  # Slight Z oscillation

        self.ball_position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RedBallSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
