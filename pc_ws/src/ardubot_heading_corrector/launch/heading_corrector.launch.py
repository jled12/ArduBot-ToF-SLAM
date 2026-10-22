from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'heading_profile', default_value='B2', choices=['B2', 'C1']
        ),
        DeclareLaunchArgument(
            'input_cmd_vel_topic', default_value='/cmd_vel_smoothed'
        ),
        DeclareLaunchArgument(
            'output_cmd_vel_topic', default_value='/cmd_vel'
        ),
        Node(
            package='ardubot_heading_corrector',
            executable='heading_corrector',
            name='ardubot_heading_corrector',
            output='screen',
            parameters=[{
                'heading_profile': LaunchConfiguration('heading_profile'),
                'input_cmd_vel_topic': LaunchConfiguration('input_cmd_vel_topic'),
                'output_cmd_vel_topic': LaunchConfiguration('output_cmd_vel_topic'),
            }],
        ),
    ])
