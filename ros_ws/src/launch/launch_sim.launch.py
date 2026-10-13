import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'autonavtest'
    pkg_path = os.path.join(get_package_share_directory(package_name))

    # Resource paths so Gazebo always finds meshes (e.g. map2.stl) regardless of working directory
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_path, 'worlds'), ':',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ]
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_path, 'worlds'), ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )
    
    # 1. Process the URDF XACRO with dynamically resolved controller config path
    xacro_file = os.path.join(pkg_path, 'description', 'rover.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'world.sdf')
    controllers_file = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')
    
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'params_file': controllers_file}
    )
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
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 9. Launch Everything!
    return LaunchDescription([
        ign_resource_path,
        gz_resource_path,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        delay_broadcaster,
        delay_drive_spawner,
        delay_steer_spawner,
        delay_swerve_brain,
        ekf_local,
    ])
