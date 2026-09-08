#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__("keyboard_teleop")

        self.publisher = self.create_publisher(
            TwistStamped,
            "/ackermann_steering_controller/reference",
            10
        )

        self.speed = 0.5
        self.turn = 0.35

    def publish(self, speed, steering):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = speed
        msg.twist.angular.z = steering
        self.publisher.publish(msg)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())

    key = sys.stdin.read(1)

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings
    )

    return key


def main():
    rclpy.init()

    node = KeyboardTeleop()

    settings = termios.tcgetattr(sys.stdin)

    print()
    print("Mars Rover Ackermann Teleop")
    print("---------------------------")
    print("W : forward")
    print("S : reverse")
    print("A : steer left")
    print("D : steer right")
    print("X : stop")
    print("Q : quit")
    print()

    speed = 0.0
    steering = 0.0

    try:
        while rclpy.ok():

            key = get_key(settings)

            if key.lower() == "w":
                speed = node.speed
            elif key.lower() == "s":
                speed = -node.speed
            elif key.lower() == "a":
                steering = node.turn
            elif key.lower() == "d":
                steering = -node.turn
            elif key.lower() == "x":
                speed = 0.0
                steering = 0.0
            elif key.lower() == "q":
                break

            node.publish(speed, steering)
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish(0.0, 0.0)
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
