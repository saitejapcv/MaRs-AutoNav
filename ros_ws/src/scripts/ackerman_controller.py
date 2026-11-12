#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class AckermanController(Node):
    def __init__(self):
        super().__init__('ackerman_controller')
        
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

        # Node parameters
        self.declare_parameter('publish_tf', False)
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self.get_logger().info(f"Ackerman Kinematics Controller Online! (publish_tf={self.publish_tf})")

    def cmd_vel_callback(self, msg):
        """ Inverse Kinematics: cmd_vel -> Wheel Angles & Speeds """
        vx = msg.linear.x
        omega = msg.angular.z
        
        steer_cmds = []
        drive_cmds = []
        
        # if the rover is stopped
        if abs(vx) < 1e-4 and abs(omega) < 1e-4:
            self.steer_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
            self.drive_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
            return
            
        # if the rover is moving in straight line
        if abs(omega) < 1e-4:
            drive_speed = vx/self.wheel_radius
            self.steer_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
            self.drive_pub.publish(Float64MultiArray(data=[drive_speed, drive_speed, drive_speed, drive_speed]))
            return
        
        # Calculate target vector for each wheel
        for key in ['FL', 'FR', 'RL', 'RR']:
            x, y = self.wheels[key]
            
            # The kinematic equations for ackerman drive
            wheel_vx = vx - (omega * y)
            wheel_vy = omega * x

            # speed an steer angle in (rad/s) and (rad) respectively
            linear_speed = math.hypot(wheel_vx, wheel_vy)
            speed = linear_speed/self.wheel_radius
            angle = math.atan2(wheel_vy, wheel_vx)
            
            # Angle optimization: keep steering angle within [-90, +90] degrees.
            # If the vector points backwards, flip the wheel 180 deg and reverse motor direction.
            if angle > (math.pi / 2.0):
                angle -= math.pi
                speed *= -1.0
            elif angle < -(math.pi / 2.0):
                angle += math.pi
                speed *= -1.0
                
            steer_cmds.append(angle)
            drive_cmds.append(speed)

        # Publish the arrays (Order matches the ros2_controllers.yaml: FL, FR, RL, RR)
        self.steer_pub.publish(Float64MultiArray(data=steer_cmds))
        self.drive_pub.publish(Float64MultiArray(data=drive_cmds))

    def get_joint_val(self, msg, joint_name, is_velocity=False):
        try:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
            else:
                idx = next((i for i, n in enumerate(msg.name) if n.endswith(joint_name)), -1)
                if idx == -1:
                    return 0.0

            arr = msg.velocity if is_velocity else msg.position
            if idx < len(arr):
                val = arr[idx]
                if not math.isnan(val):
                    return val
        except (ValueError, IndexError):
            pass
        return 0.0

    def joint_state_callback(self, msg):
        """ Forward Kinematics: Joint States -> Odometry Pose """
        # Ignore messages without velocity information (e.g. from static/model bridges)
        if len(msg.velocity) == 0:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.02  # Default fallback step if clock jump occurs

        try:
            # Safely extract current speeds and angles from the encoders
            v_fl = self.get_joint_val(msg, 'front_left_drive_joint', is_velocity=True) * self.wheel_radius
            v_fr = self.get_joint_val(msg, 'front_right_drive_joint', is_velocity=True) * self.wheel_radius
            v_rl = self.get_joint_val(msg, 'rear_left_drive_joint', is_velocity=True) * self.wheel_radius
            v_rr = self.get_joint_val(msg, 'rear_right_drive_joint', is_velocity=True) * self.wheel_radius
            
            a_fl = self.get_joint_val(msg, 'front_left_steering_joint', is_velocity=False)
            a_fr = self.get_joint_val(msg, 'front_right_steering_joint', is_velocity=False)
            a_rl = self.get_joint_val(msg, 'rear_left_steering_joint', is_velocity=False)
            a_rr = self.get_joint_val(msg, 'rear_right_steering_joint', is_velocity=False)
            
            # Convert wheel vectors back to chassis velocities
            vx_est = (v_fl*math.cos(a_fl) + v_fr*math.cos(a_fr) + v_rl*math.cos(a_rl) + v_rr*math.cos(a_rr)) / 4.0

            # Lateral velocities of each wheel
            vy_fl = v_fl * math.sin(a_fl)
            vy_fr = v_fr * math.sin(a_fr)
            vy_rl = v_rl * math.sin(a_rl)
            vy_rr = v_rr * math.sin(a_rr)
            vy_est = (vy_fl + vy_fr + vy_rl + vy_rr) / 4.0

            # Estimating angular velocity omega from difference in front and rear lateral speeds
            vy_front = (vy_fl + vy_fr) / 2.0
            vy_rear = (vy_rl + vy_rr) / 2.0
            omega_est = (vy_front - vy_rear) / self.L

            # Integrate to find new position
            self.x += (vx_est * math.cos(self.theta) - vy_est * math.sin(self.theta)) * dt 
            self.y += (vx_est * math.sin(self.theta) + vy_est * math.cos(self.theta)) * dt
            self.theta += omega_est * dt
            
            self.publish_odometry(vx_est, vy_est, omega_est)
            
        except Exception as e:
            self.get_logger().warn_throttle(2.0, f"Odometry calculation error: {e}")

    def publish_odometry(self, vx, vy, omega):
        # Convert Euler to Quaternion
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        q = [0.0, 0.0, sy, cy] # [x, y, z, w]
        
        now = self.get_clock().now().to_msg()

        # Publish TF only if enabled (EKF publishes odom -> base_footprint when active)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            self.tf_broadcaster.sendTransform(t)

        # Publish Odom Message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Pose Covariance (2D motion with high uncertainty for unobserved z/roll/pitch)
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.1

        # Twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # Twist Covariance for EKF fusion
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = AckermanController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
