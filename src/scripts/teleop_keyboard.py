#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
============================================================
   Shaastra 2027 Caterpillar Challenge — Rover Teleop
============================================================
  Drive Controls:
       [w] : Forward
  [a]  [s]  [d] : Turn Left | Backward | Turn Right
       
  Curved Arcs:
  [q] : Forward-Left       [e] : Forward-Right
  [z] : Reverse-Left       [c] : Reverse-Right
  
  Speed Control:
  [i] / [k] : Increase / Decrease Linear Speed  (+/- 0.05 m/s)
  [o] / [l] : Increase / Decrease Angular Speed (+/- 0.10 rad/s)
  
  [SPACE]   : Instant Stop
  [Ctrl+C]  : Quit
============================================================
"""

KEY_MAPPINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 0.8),
    'e': (1.0, -0.8),
    'z': (-1.0, -0.8),
    'c': (-1.0, 0.8),
    ' ': (0.0, 0.0),
}

class CaterpillarTeleop(Node):
    def __init__(self):
        super().__init__('caterpillar_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        self.linear_speed = 0.40   # m/s (sand-safe speed)
        self.angular_speed = 0.80  # rad/s

        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz publish loop
        self.get_logger().info("Caterpillar Teleop initialized on /cmd_vel")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.target_angular
        self.pub.publish(msg)

    def print_status(self):
        sys.stdout.write(f"\r[STATUS] Linear Speed: {self.linear_speed:.2f} m/s | "
                         f"Angular Speed: {self.angular_speed:.2f} rad/s | "
                         f"Command: (vx={self.target_linear:.2f}, wz={self.target_angular:.2f})   ")
        sys.stdout.flush()

def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    teleop = CaterpillarTeleop()
    
    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    teleop.print_status()

    try:
        while rclpy.ok():
            rclpy.spin_once(teleop, timeout_sec=0.01)
            key = get_key(settings, timeout=0.05)
            
            if key in KEY_MAPPINGS:
                lin_factor, ang_factor = KEY_MAPPINGS[key]
                teleop.target_linear = lin_factor * teleop.linear_speed
                teleop.target_angular = ang_factor * teleop.angular_speed
                teleop.print_status()
            elif key == 'i':
                teleop.linear_speed = min(0.80, teleop.linear_speed + 0.05)
                teleop.print_status()
            elif key == 'k':
                teleop.linear_speed = max(0.10, teleop.linear_speed - 0.05)
                teleop.print_status()
            elif key == 'o':
                teleop.angular_speed = min(1.50, teleop.angular_speed + 0.10)
                teleop.print_status()
            elif key == 'l':
                teleop.angular_speed = max(0.20, teleop.angular_speed - 0.10)
                teleop.print_status()
            elif key == '\x03': # Ctrl+C
                break
                
    except Exception as e:
        print(f"\nTeleop Exception: {e}")
    finally:
        # Send zero velocity on exit
        stop_msg = Twist()
        teleop.pub.publish(stop_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.destroy_node()
        rclpy.shutdown()
        print("\nTeleop stopped safely.")

if __name__ == '__main__':
    main()
