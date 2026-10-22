from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Runs in parallel: /map, /map_cleared and Nav2 are untouched.
    return LaunchDescription([
        Node(
            package='ardubot_projected_map_clearing',
            executable='map_clearing_node',
            name='ardubot_projected_map_clearing',
            output='screen',
            parameters=[{
                'map_topic': '/projected_map',
                'output_map_topic': '/projected_map_cleared',
                'map_frame': 'map',
                'robot_frame': 'base_link',
                'fov_degrees': 62.8,
                'clear_range': 2.5,
                'camera_forward_offset': 0.085,
                'tf_timeout': 0.08,
                'source_obstacle_threshold': 99,
                'source_free_max_value': 0,
                'map_resolution': 0.025,
                'initial_size_m': 16.0,
                'expand_step_m': 2.0,
                'publish_period': 0.25,
            }],
        )
    ])
