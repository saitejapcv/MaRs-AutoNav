#!/usr/bin/env python3
"""
Velocity Command Relay for Caterpillar Challenge
=================================================
Relays standard Nav2 velocity commands from /cmd_vel to the ros2_control
diff_drive_base_controller input topic (/diff_drive_base_controller/cmd_vel_unstamped).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        self.pub_controller = self.create_publisher(
            Twist, '/diff_drive_base_controller/cmd_vel_unstamped', 10
        )
        self.get_logger().info('cmd_vel relay initialized: /cmd_vel -> /diff_drive_base_controller/cmd_vel_unstamped')

    def cmd_callback(self, msg: Twist):
        self.pub_controller.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
