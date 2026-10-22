import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ardubot_pc_bringup_pointcloud')
    params = os.path.join(share, 'config', 'slam.yaml')
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                params,
                {
                    'use_sim_time': False,
                    'map_frame': 'map',
                    'odom_frame': 'odom_bno',
                    'base_frame': 'base_link',
                    'scan_topic': '/scan',
                    'min_laser_range': 0.10,
                    'max_laser_range': 3.0,
                },
            ],
        )
    ])
