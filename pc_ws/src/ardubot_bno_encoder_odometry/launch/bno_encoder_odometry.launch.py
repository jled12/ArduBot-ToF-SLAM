import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ardubot_bno_encoder_odometry')
    params = os.path.join(share, 'config', 'bno_encoder_odometry.yaml')
    return LaunchDescription([
        Node(
            package='ardubot_bno_encoder_odometry',
            executable='bno_encoder_odometry',
            name='ardubot_bno_encoder_odometry',
            output='screen',
            parameters=[params],
        )
    ])
