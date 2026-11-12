#!/usr/bin/env python3
"""
4WD Skid-Steer Drive Controller & Odometry Estimator
Caterpillar Autonomy Challenge - Shaastra 2027

Features:
- Subscribes to /cmd_vel (geometry_msgs/msg/Twist)
- 4WD Skid-Steer Inverse Kinematics with empirical sand slip multiplier (W_eff = alpha * W_track)
- Watchdog timer: automatically stops motors if cmd_vel stream drops
- Publishes /wheel_speeds (std_msgs/msg/Float64MultiArray: [FL, RL, FR, RR] in rad/s)
- Computes dead-reckoning odometry from wheel joint encoders (/joint_states)
- Publishes /skid_odom (nav_msgs/msg/Odometry) with proper covariance matrices
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import tf2_ros

class SkidSteerController(Node):
    def __init__(self):
        super().__init__('skid_steer_controller')

        # Declare configurable parameters
        self.declare_parameter('wheel_radius', 0.15)         # 30 cm diameter wheels
        self.declare_parameter('track_width', 0.76)          # 76 cm axle-to-axle track width
        self.declare_parameter('wheelbase', 0.88)            # 88 cm wheelbase
        self.declare_parameter('slip_multiplier', 1.35)      # Effective track width multiplier for sand
        self.declare_parameter('cmd_timeout', 0.5)           # Watchdog timeout in seconds
        self.declare_parameter('publish_tf', False)          # EKF (robot_localization) handles odom->base_footprint
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        # Read parameters
        self.r = self.get_parameter('wheel_radius').value
        self.w = self.get_parameter('track_width').value
        self.l = self.get_parameter('wheelbase').value
        self.alpha = self.get_parameter('slip_multiplier').value
        self.timeout = self.get_parameter('cmd_timeout').value
        self.pub_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value

        # Effective track width (kinematic skid-steer correction)
        self.w_eff = self.alpha * self.w

        # Internal state
        self.last_cmd_time = self.get_clock().now()
        self.current_cmd = Twist()
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        # Joint tracking
        self.wheel_joint_names = [
            'front_left_wheel_joint',
            'rear_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]
        self.last_wheel_positions = {name: None for name in self.wheel_joint_names}

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            20
        )

        # Publishers
        self.wheel_speed_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_speeds',
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            '/skid_odom',
            20
        )

        if self.pub_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Control loop & Watchdog Timer (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f'4WD Skid-Steer Controller initialized: R={self.r}m, Track={self.w}m, '
            f'SlipMultiplier={self.alpha:.2f} (W_eff={self.w_eff:.3f}m)'
        )

    def cmd_vel_callback(self, msg: Twist):
        self.current_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

        # Watchdog: if no command received within timeout, stop
        if dt_cmd > self.timeout:
            vx = 0.0
            wz = 0.0
        else:
            vx = self.current_cmd.linear.x
            wz = self.current_cmd.angular.z

        # Inverse Kinematics for 4WD Paired Skid-Steer:
        # Left side and right side wheel tangential speeds
        v_left = vx - (wz * self.w_eff / 2.0)
        v_right = vx + (wz * self.w_eff / 2.0)

        omega_left = v_left / self.r
        omega_right = v_right / self.r

        # [FL, RL, FR, RR]
        speeds_msg = Float64MultiArray()
        speeds_msg.data = [omega_left, omega_left, omega_right, omega_right]
        self.wheel_speed_pub.publish(speeds_msg)

    def joint_state_callback(self, msg: JointState):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            self.last_odom_time = now
            return

        # Extract current wheel positions
        current_positions = {}
        for i, name in enumerate(msg.name):
            if name in self.wheel_joint_names and i < len(msg.position):
                current_positions[name] = msg.position[i]

        if any(name not in current_positions for name in self.wheel_joint_names):
            return

        # If first run, initialize baseline positions
        if any(self.last_wheel_positions[n] is None for n in self.wheel_joint_names):
            for n in self.wheel_joint_names:
                self.last_wheel_positions[n] = current_positions[n]
            self.last_odom_time = now
            return

        # Calculate joint deltas
        d_fl = current_positions['front_left_wheel_joint'] - self.last_wheel_positions['front_left_wheel_joint']
        d_rl = current_positions['rear_left_wheel_joint'] - self.last_wheel_positions['rear_left_wheel_joint']
        d_fr = current_positions['front_right_wheel_joint'] - self.last_wheel_positions['front_right_wheel_joint']
        d_rr = current_positions['rear_right_wheel_joint'] - self.last_wheel_positions['rear_right_wheel_joint']

        # Update last positions
        for n in self.wheel_joint_names:
            self.last_wheel_positions[n] = current_positions[n]
        self.last_odom_time = now

        # Average left side and right side angular displacements
        d_theta_left = (d_fl + d_rl) / 2.0
        d_theta_right = (d_fr + d_rr) / 2.0

        # Linear displacements
        ds_left = d_theta_left * self.r
        ds_right = d_theta_right * self.r

        # Forward kinematics
        ds = (ds_left + ds_right) / 2.0
        d_yaw = (ds_right - ds_left) / self.w_eff

        # Runge-Kutta 2nd Order pose integration
        mid_yaw = self.yaw + (d_yaw / 2.0)
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw += d_yaw

        # Velocities
        vx = ds / dt
        wz = d_yaw / dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        # Pose covariance (estimated uncertainty for loose sand)
        odom.pose.covariance[0] = 0.005   # x
        odom.pose.covariance[7] = 0.005   # y
        odom.pose.covariance[14] = 1e6    # z (unobserved)
        odom.pose.covariance[21] = 1e6    # roll
        odom.pose.covariance[28] = 1e6    # pitch
        odom.pose.covariance[35] = 0.02   # yaw

        # Twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        # Twist covariance
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 1e6
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)

        if self.pub_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SkidSteerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
