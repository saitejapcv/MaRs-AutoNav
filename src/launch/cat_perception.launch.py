import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Terrain Hazard Filter Node
    # Extracts boulders (>+8cm), craters (<-8cm), purges walls, and publishes virtual boundary
    terrain_hazard_filter_node = Node(
        package='caterpillar_challenge',
        executable='terrain_hazard_filter.py',
        name='terrain_hazard_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_cloud_topic': '/depth_camera/points',
            'base_frame': 'base_footprint',
            'map_frame': 'map',
            'boulder_threshold_z': 0.08,
            'crater_threshold_z': -0.08,
            'min_range': 0.40,
            'max_range': 3.50,
            'voxel_size': 0.04,
            'arena_length_x': 9.0,
            'arena_width_y': 5.0,
            'wall_crop_margin': 0.25,
            'wall_height_ceiling': 0.40,
            'virtual_wall_buffer': 0.45
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        terrain_hazard_filter_node
    ])
