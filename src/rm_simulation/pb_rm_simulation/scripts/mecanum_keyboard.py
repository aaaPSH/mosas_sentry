#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Mecanum keyboard teleop
-----------------------
Move:
  w/s : forward/back
  a/d : strafe left/right
  q/e : rotate left/right

Speed:
  r/f : increase/decrease linear speed
  t/g : increase/decrease angular speed
  +/= : increase both speeds
  -/_ : decrease both speeds

Stop:
  space or x

Quit:
  Ctrl-C
"""


class MecanumKeyboard(Node):
    def __init__(self):
        super().__init__("mecanum_keyboard")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_chassis")
        self.declare_parameter("linear_speed", 0.8)
        self.declare_parameter("angular_speed", 1.5)
        self.declare_parameter("linear_speed_step", 0.1)
        self.declare_parameter("angular_speed_step", 0.1)
        self.declare_parameter("key_timeout", 0.15)

        topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.linear_speed_step = float(self.get_parameter("linear_speed_step").value)
        self.angular_speed_step = float(self.get_parameter("angular_speed_step").value)
        self.key_timeout = float(self.get_parameter("key_timeout").value)

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f"Publishing mecanum cmd_vel to '{topic}'")

    def make_twist(self, x=0.0, y=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = yaw
        return msg

    def publish_stop(self):
        self.publisher.publish(self.make_twist())

    def log_speed(self):
        self.get_logger().info(
            f"Speed linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}"
        )

    def handle_key(self, key):
        linear = self.linear_speed
        angular = self.angular_speed

        if key == "w":
            return self.make_twist(x=linear)
        if key == "s":
            return self.make_twist(x=-linear)
        if key == "a":
            return self.make_twist(y=linear)
        if key == "d":
            return self.make_twist(y=-linear)
        if key == "q":
            return self.make_twist(yaw=angular)
        if key == "e":
            return self.make_twist(yaw=-angular)
        if key in (" ", "x"):
            return self.make_twist()
        if key == "r":
            self.linear_speed += self.linear_speed_step
            self.log_speed()
            return None
        if key == "f":
            self.linear_speed = max(0.0, self.linear_speed - self.linear_speed_step)
            self.log_speed()
            return None
        if key == "t":
            self.angular_speed += self.angular_speed_step
            self.log_speed()
            return None
        if key == "g":
            self.angular_speed = max(0.0, self.angular_speed - self.angular_speed_step)
            self.log_speed()
            return None
        if key in ("+", "="):
            self.linear_speed += self.linear_speed_step
            self.angular_speed += self.angular_speed_step
            self.log_speed()
            return None
        if key in ("-", "_"):
            self.linear_speed = max(0.0, self.linear_speed - self.linear_speed_step)
            self.angular_speed = max(0.0, self.angular_speed - self.angular_speed_step)
            self.log_speed()
            return None
        return None


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return ""


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = MecanumKeyboard()

    print(HELP)
    tty.setraw(sys.stdin.fileno())

    try:
        while rclpy.ok():
            key = read_key(node.key_timeout)
            if key == "\x03":
                break

            msg = node.handle_key(key) if key else node.make_twist()
            if msg is not None:
                node.publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.publish_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
