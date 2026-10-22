import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
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
        arguments=['-topic', 'robot_description', '-name', 'ares_nova', '-x', '0', '-y', '3', '-z', '0.25'],
        output='screen'
    )
    
    # 5. The Data Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/depth_camera/image_raw/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/image_raw/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/model/ares_nova/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 6. ros2_control Spawners
    spawn_broadcaster = Node(
        package='controller_manager', 
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'])

    delay_broadcaster = RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[spawn_broadcaster]))
    
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

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # 10. RGBD Sync — packages matching RGB, depth and camera-info messages
    # into one RGBDImage.  The Gazebo bridge may deliver the RGB and depth
    # messages up to 132 ms apart even though they came from the same sensor
    # update, so a 100 ms matching window drops valid frames.
    # ros_gz_bridge processes color/depth callbacks at different sim-clock moments,
    # causing up to 132ms header skew. This node fixes that before rtabmap sees the data.
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.2,
            'queue_size': 60,
        }],
        remappings=[
            ('rgb/image',        '/depth_camera/image_raw/image'),
            ('depth/image',      '/depth_camera/image_raw/depth_image'),
            ('rgb/camera_info',  '/depth_camera/image_raw/camera_info'),
            # output: /rgbd_image  (consumed by rtabmap via rgbd_topic argument)
        ]
    )

    # 11. RTAB-Map must consume the combined RGBDImage, not the three raw
    # bridge topics.  Subscribing to the raw topics recreates the timestamp
    # mismatch above and produces repeated "Did not receive data" warnings.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_footprint',
            'odom_topic': '/odometry/global',
            'subscribe_rgbd': True,
            'queue_size': 60,
            'sync_queue_size': 60,
            'topic_queue_size': 60,
            # Keep the existing one-Hz mapping cadence; this is not the
            # camera publishing rate.
            'Rtabmap/DetectionRate': '1.0',
            # 2D navigation grid tuned for the 640x480, downward-looking
            # simulated RGB-D camera.  The defaults preserve too much sparse
            # depth noise while leaving observed free space unknown.
            'Grid/Sensor': '1',                 # build the grid from depth
            'Grid/3D': 'false',                 # 2D occupancy grid for navigation
            'Grid/CellSize': '0.08',            # 8 cm cells; stable at rover scale
            'Grid/DepthDecimation': '2',        # retain more depth detail than default 4x
            'Grid/RangeMin': '0.4',             # reject near-clip artifacts
            'Grid/RangeMax': '6.0',             # ignore unreliable distant depth
            'Grid/RayTracing': 'true',          # mark space up to observations free
            'Grid/NoiseFilteringRadius': '0.15',
            'Grid/NoiseFilteringMinNeighbors': '3',
            'GridGlobal/FootprintRadius': '0.55', # remove the rover from its map
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            # rtabmap_ros takes odometry as the "odom" topic; a parameter
            # named odom_topic does not remap that subscription.
            ('odom', '/odometry/global'),
        ]
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_footprint',
            'odom_topic': '/odometry/global',
            'subscribe_rgbd': True,
            'queue_size': 60,
            'sync_queue_size': 60,
            'topic_queue_size': 60,
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('odom', '/odometry/global'),
        ]
    )

    # 9. Launch Everything!
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        delay_broadcaster,
        delay_drive_spawner,
        delay_steer_spawner,
        delay_swerve_brain,
        ekf_local,
        rgbd_sync,
        rtabmap,
        rtabmap_viz,
    ])
