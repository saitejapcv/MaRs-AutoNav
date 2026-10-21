#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from rclpy.time import Time
import time

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        
        # --- 1. ROVER DIMENSIONS (Updated to CAD Specs) ---
        self.wheel_radius = 0.15
        self.L = 0.8850  # Wheelbase (from new suspension_length)
        self.W = 0.76    # Track width (base_width 0.6 + our 8cm padding on each side)
        
        # Wheel coordinates relative to chassis center: (x, y)
        self.wheels = {
            'FL': (self.L/2, self.W/2),
            'FR': (self.L/2, -self.W/2),
            'RL': (-self.L/2, self.W/2),
            'RR': (-self.L/2, -self.W/2)
        }

        # --- 2. PUBLISHERS & SUBSCRIBERS ---
        # Publish commands to ros2_control
        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_position_controller/commands', 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_velocity_controller/commands', 10)
        
        # Publish Odometry & TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Listen to target velocities and actual joint states
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # --- 3. ODOMETRY TRACKING VARIABLES ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.last_steer = [0.0] * 4
        self.joint_prefix = {'FL': 'front_left', 'FR': 'front_right', 'RL': 'rear_left', 'RR': 'rear_right'}

        self.get_logger().info("Swerve Kinematics Controller Online!")

    def cmd_vel_callback(self, msg):
        """ Inverse Kinematics: cmd_vel -> Wheel Angles & Speeds """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        
        steer_cmds = []
        drive_cmds = []
        
        # Calculate target vector for each wheel
        for key in ['FL', 'FR', 'RL', 'RR']:
            x, y = self.wheels[key]
            
            # The kinematic equations for holonomic swerve drive
            wheel_vx = vx - (omega * y)
            wheel_vy = vy + (omega * x)
            
            # Calculate required wheel speed (rad/s) and steering angle (rad)
            speed = math.sqrt(wheel_vx**2 + wheel_vy**2) / self.wheel_radius
            angle = math.atan2(wheel_vy, wheel_vx)
            
            # --- THE FIX: ANGLE OPTIMIZATION ---
            # Keep the steering between -90 and +90 degrees. 
            # If it tries to go past 90, flip the wheel straight and reverse the motor.
            if angle > (math.pi / 2.0):
                angle -= math.pi
                speed *= -1.0
            elif angle < -(math.pi / 2.0):
                angle += math.pi
                speed *= -1.0
            # -----------------------------------
                
            steer_cmds.append(angle)
            drive_cmds.append(speed)

        if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(omega) < 1e-3:
            steer_cmds = list(self.last_steer)   # hold the wheel angles
            drive_cmds = [0.0] * 4
        else:
            self.last_steer = list(steer_cmds)

        # Publish the arrays (Order matches the ros2_controllers.yaml: FL, FR, RL, RR)
        self.steer_pub.publish(Float64MultiArray(data=steer_cmds))
        self.drive_pub.publish(Float64MultiArray(data=drive_cmds))

    def joint_state_callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_time is None:
            self.last_time = stamp
            return
        dt = (stamp - self.last_time).nanoseconds / 1e9
        self.last_time = stamp
        if dt <= 0.0 or dt > 0.5:
            return

        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            sum_vx = sum_vy = moment = 0.0
            for key, prefix in self.joint_prefix.items():
                v = msg.velocity[idx[f'{prefix}_drive_joint']] * self.wheel_radius
                a = msg.position[idx[f'{prefix}_steering_joint']]
                wvx, wvy = v * math.cos(a), v * math.sin(a)
                x, y = self.wheels[key]
                sum_vx += wvx
                sum_vy += wvy
                moment += x * wvy - y * wvx
        except (KeyError, IndexError):
            self.get_logger().warn('joint_states missing expected joints', throttle_duration_sec=5.0)
            return

        vx = sum_vx / 4.0
        vy = sum_vy / 4.0
        omega = moment / (self.L ** 2 + self.W ** 2)

        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta = math.atan2(math.sin(self.theta + omega * dt),
                                math.cos(self.theta + omega * dt))
        self.publish_odometry(vx, vy, omega, msg.header.stamp)

    def publish_odometry(self, vx, vy, omega, stamp):
        # Convert Euler yaw to Quaternion
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)

        # Publish Odom Message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (Quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # Velocities
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.02
        odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.05
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()