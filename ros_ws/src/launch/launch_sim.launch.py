import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'autonavtest'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    
    # 1. Process the URDF XACRO
    xacro_file = os.path.join(pkg_path, 'description', 'rover.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'world.sdf')
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': robot_description_config.toxml()}
    
    # 2. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc, {'use_sim_time': True}]
    )
    
    # 3. Start Gazebo Simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )
    
    # 4. Spawn the Rover
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ares_nova', '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'],
        output='screen'
    )
    
    # 5. The Data Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/depth_camera/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/gps/fix@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 6. ros2_control Spawners
    spawn_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    spawn_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_velocity_controller', '--controller-manager', '/controller_manager'],
    )
    spawn_steer = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_position_controller', '--controller-manager', '/controller_manager'],
    )

    # 7. Our Custom Swerve Brain
    swerve_brain = Node(
        package=package_name,
        executable='swerve_controller.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. CHAINING THE SPAWNERS (The Fix for the Race Condition)
    delay_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_broadcaster,
            on_exit=[spawn_drive],
        )
    )

    delay_steer_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_drive,
            on_exit=[spawn_steer],
        )
    )

    delay_swerve_brain = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_steer,
            on_exit=[swerve_brain],
        )
    )

    ekf_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ekf.yaml'
    )

    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'publish_filtered_gps': True,
            'base_link_frame': 'base_footprint',
            'use_odometry_yaw': True,             # <--- FIX 1: Bypasses Gazebo IMU bugs
            'transform_timeout': 0.5              # <--- FIX 2: Allows TF tree time to sync
        }],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_footprint',        # Anchors the map to the rover's base
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,        # We are feeding it the dense 3D point cloud
            'approx_sync': True,
            'wait_for_transform': 0.5,
            'sync_queue_size': 20,
            'topic_queue_size': 20,
            'Grid/NormalsSegmentation': 'true',
            'Grid/MaxGroundAngle': '20.0',       # Slopes steeper than 20 deg are classified as obstacles
            'Grid/NormalK': '20',                # K=20 gives smoother normal estimation and reduces surface jitter
            'Grid/MinClusterSize': '15',         # Minimum cluster size to retain obstacle points (filters speckles)
            'Grid/MaxObstacleHeight': '2.0',     # Ignores points higher than 2.0m
            'Grid/RangeMax': '5.0',              # Clips far-range depth distortion and boundary ray artifacts
            'Grid/RangeMin': '0.4',              # Clips near-field points / chassis reflections
            'Grid/NoiseFilteringRadius': '0.15', # Rejects outlier points with radius search
            'Grid/NoiseFilteringMinNeighbors': '5', # Minimum neighbors required in filtering radius
            'Grid/PreVoxelFiltering': 'true',    # Downsamples cloud to CellSize before normal calculation
            'Grid/CellSize': '0.05',             # 5cm grid resolution
            'Grid/RayTracing': 'false',          # Disables aggressive clearing inside depressions
            'Grid/From3D': 'true',               # Projects 3D scan cloud directly into 2D occupancy grid
            'Grid/Sensor': '0',
            'RGBD/CreateOccupancyGrid': 'true',  # Creates and saves local occupancy grids in memory
            'RGBD/ProximityPathMaxNeighbors': '0'
        }],
        remappings=[
            ('odom', '/odometry/filtered'),
            ('scan_cloud', '/depth_camera/image_raw/points')
        ],
        arguments=['-d'] # Deletes the old map database on every fresh boot
    )

    # 9. RViz2 Node
    rviz_config_file = os.path.join(pkg_path, 'config', 'autonavtest.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # 10. Launch Everything!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2'
        ),
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        TimerAction(period=3.0, actions=[spawn_broadcaster]),
        delay_drive_spawner,
        delay_steer_spawner,
        delay_swerve_brain,
        start_ekf_node,
        TimerAction(period=5.0, actions=[navsat_transform]),
        TimerAction(period=7.0, actions=[rtabmap_node]),
        TimerAction(period=8.5, actions=[rviz_node])
    ])