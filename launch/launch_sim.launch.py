import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'ares_nova_v2'
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
        arguments=['-topic', 'robot_description', '-name', 'ares_nova'],
        output='screen'
    )
    
    # 5. The Data Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/my_world/model/ares_nova/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/depth_camera/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/world/my_world/model/ares_nova/joint_state', '/joint_states')
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

    # 9. Launch Everything!
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        TimerAction(period=3.0, actions=[spawn_broadcaster]),
        delay_drive_spawner,
        delay_steer_spawner,
        delay_swerve_brain
    ])