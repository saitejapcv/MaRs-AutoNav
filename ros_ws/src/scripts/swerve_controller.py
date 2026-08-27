#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
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
        self.last_time = self.get_clock().now()

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

            # If the rover isn't moving, keep wheels facing forward to avoid snapping back
            if vx == 0.0 and vy == 0.0 and omega == 0.0:
                angle = 0.0
                
            steer_cmds.append(angle)
            drive_cmds.append(speed)

        # Publish the arrays (Order matches the ros2_controllers.yaml: FL, FR, RL, RR)
        self.steer_pub.publish(Float64MultiArray(data=steer_cmds))
        self.drive_pub.publish(Float64MultiArray(data=drive_cmds))

    def joint_state_callback(self, msg):
        """ Forward Kinematics: Joint States -> Odometry Pose """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0: return

        try:
            # Extract current speeds and angles from the encoders
            # Note: You may need to verify the exact index mapping based on your Gazebo output
            v_fl = msg.velocity[msg.name.index('front_left_drive_joint')] * self.wheel_radius
            v_fr = msg.velocity[msg.name.index('front_right_drive_joint')] * self.wheel_radius
            v_rl = msg.velocity[msg.name.index('rear_left_drive_joint')] * self.wheel_radius
            v_rr = msg.velocity[msg.name.index('rear_right_drive_joint')] * self.wheel_radius
            
            a_fl = msg.position[msg.name.index('front_left_steering_joint')]
            a_fr = msg.position[msg.name.index('front_right_steering_joint')]
            a_rl = msg.position[msg.name.index('rear_left_steering_joint')]
            a_rr = msg.position[msg.name.index('rear_right_steering_joint')]
            
            # Convert wheel vectors back to chassis velocities
            vx_est = (v_fl*math.cos(a_fl) + v_fr*math.cos(a_fr) + v_rl*math.cos(a_rl) + v_rr*math.cos(a_rr)) / 4.0
            vy_est = (v_fl*math.sin(a_fl) + v_fr*math.sin(a_fr) + v_rl*math.sin(a_rl) + v_rr*math.sin(a_rr)) / 4.0
            
            # Estimate angular velocity
            omega_est = 0.0 # Simplified for brevity, usually averaged across torque vectors
            
            # Integrate to find new position
            self.x += (vx_est * math.cos(self.theta) - vy_est * math.sin(self.theta)) * dt
            self.y += (vx_est * math.sin(self.theta) + vy_est * math.cos(self.theta)) * dt
            self.theta += omega_est * dt
            
            self.publish_odometry(vx_est, vy_est, omega_est)
            
        except ValueError:
            pass # Joint states haven't fully populated yet

    def publish_odometry(self, vx, vy, omega):
        # Convert Euler to Quaternion
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        q = [0.0, 0.0, sy, cy] # [x, y, z, w]
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odom Message
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()