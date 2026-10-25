import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_autonavtest = get_package_share_directory('autonavtest')
    pkg_rtabmap_launch = get_package_share_directory('rtabmap_launch')

    # RViz config: use package's rviz/default.rviz
    rviz_cfg_path = os.path.join(pkg_autonavtest, 'rviz', 'default.rviz')

    # Launch Configurations & Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    localization = LaunchConfiguration('localization')
    args = LaunchConfiguration('args')

    # Shared Grid parameters (defined once, reused across RTAB-Map SLAM and obstacles_detection)
    grid_params = {
        'Grid/RayTracing': 'false',
        'Grid/MaxGroundAngle': '29',
        'Grid/RangeMax': '5.0',
        'Grid/RangeMin': '0.4',
        'Grid/FlatObstacleDetected': 'false',
        'Grid/NormalK': '30',
        'Grid/NoiseFilteringRadius': '0.15',
        'Grid/NoiseFilteringMinNeighbors': '8',
        'Grid/MinClusterSize': '30',
        'Grid/MinGroundHeight': '-0.20',
    }
    grid_args_str = ' '.join([f'--{k} {v}' for k, v in grid_params.items()])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Disable rtabmap standalone GUI by default
    declare_rtabmap_viz = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map standalone GUI (disabled by default)'
    )

    # Enable RViz2 by default
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 (enabled by default)'
    )

    declare_rviz_cfg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=rviz_cfg_path,
        description='Path to RViz configuration file'
    )

    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Set to true for localization mode (mapping disabled)'
    )

    declare_args = DeclareLaunchArgument(
        'args',
        default_value=f'--delete_db_on_start --GridGlobal/Eroded true {grid_args_str}',
        description='Custom RTAB-Map core arguments'
    )

    # Point Cloud Generation from Depth Image
    node_point_cloud_xyz = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        output='screen',
        parameters=[{
            'decimation': 4,
            'min_depth': 0.4,
            'max_depth': 5.0,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('depth/image', '/depth_camera/image_raw/depth_image'),
            ('depth/camera_info', '/depth_camera/image_raw/camera_info'),
            ('cloud', '/camera/cloud'),
        ]
    )

    # Obstacle vs Ground Segmentation Node
    node_obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        parameters=[
            grid_params,
            {
                'frame_id': 'base_footprint',
                'map_frame_id': 'map',
                'wait_for_transform': 0.2,
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('cloud', '/camera/cloud'),
            ('obstacles', '/obstacles'),
            ('ground', '/ground'),
        ]
    )

    # Include RTAB-Map main launch with configured parameters
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rtabmap_launch, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'rgb_topic': '/depth_camera/image_raw/image',
            'depth_topic': '/depth_camera/image_raw/depth_image',
            'camera_info_topic': '/depth_camera/image_raw/camera_info',
            'odom_topic': '/odometry/local',
            'visual_odometry': 'false',
            'approx_sync': 'true',
            'rtabmap_viz': rtabmap_viz,
            'rviz': rviz,
            'rviz_cfg': rviz_cfg,
            'localization': localization,
            'args': args,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rtabmap_viz,
        declare_rviz,
        declare_rviz_cfg,
        declare_localization,
        declare_args,
        node_point_cloud_xyz,
        node_obstacles_detection,
        rtabmap_launch
    ])
