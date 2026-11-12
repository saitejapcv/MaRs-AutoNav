import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('caterpillar_challenge')
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. robot_localization Extended Kalman Filter (EKF) Node
    # Fuses forward linear velocity (vx) from wheel odometry with angular velocity (omega_z) from IMU
    # Magnetometer strictly disabled per competition rules.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odometry/local')
        ]
    )

    # 2. ArUco Visual Localizer Node
    # Detects passive wall markers on Starting Zone perimeter.
    # Broadcasts map -> odom transform and locks initial pose.
    aruco_localizer_node = Node(
        package='caterpillar_challenge',
        executable='aruco_localizer.py',
        name='aruco_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'camera_frame': 'camera_optical_link',
            'marker_size': 0.192,
            'publish_debug_image': True,
            'tf_broadcast_rate': 30.0,
            'ema_alpha': 0.20,
            'lock_threshold_count': 3
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_node,
        aruco_localizer_node
    ])
