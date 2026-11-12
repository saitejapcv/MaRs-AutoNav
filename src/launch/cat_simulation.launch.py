import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('caterpillar_challenge')
    
    # 0. Set Ignition Gazebo Resource Paths so all meshes and models are resolved
    workspace_root = '/home/yashvant/Desktop/MaRs-Caterpillar'
    current_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    resource_paths = [
        workspace_root,
        os.path.join(workspace_root, 'src'),
        pkg_path,
        os.path.dirname(pkg_path),
        os.path.join(pkg_path, 'description'),
        os.path.join(pkg_path, 'worlds'),
        '/home/yashvant/Desktop/MaRs-AutoNav'
    ]
    if current_ign_path:
        resource_paths.append(current_ign_path)
    combined_resource_path = ':'.join(resource_paths)
    
    set_ign_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', combined_resource_path)
    
    # 1. Process Rover URDF Xacro
    xacro_file = os.path.join(pkg_path, 'description', 'cat_rover.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'cat_arena.sdf')
    rviz_file = os.path.join(pkg_path, 'config', 'cat_rover.rviz')
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': robot_description_config.toxml()}
    
    # 2. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc, {'use_sim_time': True}]
    )
    
    # 3. Ignition Gazebo Simulator (Loading 9m x 5m Arena World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r ' + world_file}.items()
    )
    
    # Launch Configurations for flexible rover spawn position
    spawn_x = LaunchConfiguration('spawn_x', default='1.0')
    spawn_y = LaunchConfiguration('spawn_y', default='4.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.25')
    spawn_yaw = LaunchConfiguration('spawn_yaw', default='0.0')

    declare_x = DeclareLaunchArgument('spawn_x', default_value='1.0', description='Rover spawn X')
    declare_y = DeclareLaunchArgument('spawn_y', default_value='4.0', description='Rover spawn Y')
    declare_z = DeclareLaunchArgument('spawn_z', default_value='0.25', description='Rover spawn Z')
    declare_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Rover spawn Yaw (rad)')

    # 4. Spawn Rover inside Arena
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'cat_rover',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
        output='screen'
    )
    
    # 5. ROS-Gazebo Data Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/depth_camera/image_raw/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/depth_camera/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/depth_camera/image_raw/image', '/camera/image_raw'),
            ('/depth_camera/image_raw/camera_info', '/camera/camera_info'),
            ('/depth_camera/image_raw/points', '/depth_camera/points')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # 6. ros2_control Spawners
    spawn_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Chaining the controller spawners to prevent race conditions
    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_broadcaster,
            on_exit=[spawn_diff_drive],
        )
    )
    
    # 7. State Estimation & Localization (Phase 3: EKF + ArUco)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'cat_localization.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_localization'))
    )

    declare_localization = DeclareLaunchArgument(
        'launch_localization',
        default_value='true',
        description='Whether to launch EKF and ArUco state estimation'
    )

    # 8. Terrain Hazards & Perception (Phase 4: Boulders, Craters, Wall Exclusion)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'cat_perception.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_perception'))
    )

    declare_perception = DeclareLaunchArgument(
        'launch_perception',
        default_value='true',
        description='Whether to launch terrain hazard perception'
    )

    # 9. Nav2 Navigation Stack (Phase 5: Path Planning & Tracking)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'cat_navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_nav2'))
    )

    declare_nav2 = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Whether to launch Nav2 navigation stack'
    )

    # 10. RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        set_ign_env,
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz2 for visualization'
        ),
        declare_localization,
        declare_perception,
        declare_nav2,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        TimerAction(period=3.5, actions=[spawn_broadcaster]),
        delay_diff_drive_spawner,
        TimerAction(period=4.2, actions=[localization_launch]),
        TimerAction(period=4.5, actions=[perception_launch]),
        TimerAction(period=6.0, actions=[navigation_launch]),
        TimerAction(period=6.5, actions=[rviz_node])
    ])
