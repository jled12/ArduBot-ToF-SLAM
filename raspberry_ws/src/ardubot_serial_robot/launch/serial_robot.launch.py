import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ardubot_serial_robot')
    params = os.path.join(pkg_share, 'config', 'serial_robot.yaml')

    return LaunchDescription([
        Node(
            package='ardubot_serial_robot',
            executable='serial_robot',
            name='serial_robot',
            output='screen',
            parameters=[params],
        ),
    ])
