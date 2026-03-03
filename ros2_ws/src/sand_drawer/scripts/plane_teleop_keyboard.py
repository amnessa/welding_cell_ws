#!/usr/bin/env python3
"""
Keyboard Teleop for Sand Drawer Plane Servoing.

Publishes geometry_msgs/Twist on /teleop_plane_vel to drive the end-effector
along the drawing plane axes.  The planar_servo_controller (in teleop mode)
subscribes to this topic and maps velocities into the plane frame with
boundary clamping and Z-drift correction.

Keys
----
  W / ↑  :  +Y on the plane (forward in plane Y axis)
  S / ↓  :  -Y on the plane
  A / ←  :  -X on the plane
  D / →  :  +X on the plane
  Q      :  quit

The Twist encodes the plane-frame velocity:
  twist.linear.x → plane X velocity
  twist.linear.y → plane Y velocity
  (all other fields zero)

Run in a separate terminal:
  ros2 run sand_drawer plane_teleop_keyboard.py
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP_TEXT = """
╔═══════════════════════════════════════════╗
║   Sand Drawer — Keyboard Teleop          ║
║                                           ║
║         W / ↑   → plane +Y (forward)     ║
║   A / ←         D / → → plane ±X (L/R)  ║
║         S / ↓   → plane -Y (backward)   ║
║                                           ║
║   Speed: {speed:.3f} m/s                  ║
║   +/- to change speed  (0.01 – 0.30)     ║
║   Q  → quit                               ║
╚═══════════════════════════════════════════╝
"""

# Arrow key escape sequences: ESC [ A/B/C/D
ARROW_UP    = '\x1b[A'
ARROW_DOWN  = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT  = '\x1b[D'


def get_key(settings):
    """Read a single keypress (handles arrow key escape sequences)."""
    tty.setraw(sys.stdin.fileno())
    try:
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch + ch2 + ch3
        return ch
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class PlaneTeleopKeyboard(Node):
    def __init__(self):
        super().__init__('plane_teleop_keyboard')
        self.declare_parameter('speed', 0.10)
        self.speed = self.get_parameter('speed').value

        self.pub = self.create_publisher(Twist, '/teleop_plane_vel', 10)
        self.get_logger().info(f'Teleop ready — speed={self.speed:.3f} m/s')

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(HELP_TEXT.format(speed=self.speed))

        try:
            while True:
                key = get_key(settings)
                tw = Twist()

                if key in ('q', 'Q', '\x03'):  # q or Ctrl-C
                    # Publish zero before quitting
                    self.pub.publish(tw)
                    break

                if key in ('w', 'W', ARROW_UP):
                    tw.linear.y = self.speed
                elif key in ('s', 'S', ARROW_DOWN):
                    tw.linear.y = -self.speed
                elif key in ('a', 'A', ARROW_LEFT):
                    tw.linear.x = -self.speed
                elif key in ('d', 'D', ARROW_RIGHT):
                    tw.linear.x = self.speed
                elif key == '+':
                    self.speed = min(self.speed + 0.01, 0.30)
                    print(f'Speed: {self.speed:.3f} m/s')
                elif key in ('-', '_'):
                    self.speed = max(self.speed - 0.01, 0.01)
                    print(f'Speed: {self.speed:.3f} m/s')
                else:
                    # Unknown key → publish zero (stop)
                    pass

                self.pub.publish(tw)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            # Ensure stop
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PlaneTeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
