import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ardubot_full_robot_bringup')
    serial_share = get_package_share_directory('ardubot_serial_robot')
    return LaunchDescription([
        DeclareLaunchArgument('bridge_config', default_value=os.path.join(share, 'config', 'bridge_with_pc.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(serial_share, 'launch', 'serial_robot.launch.py'))),
        Node(package='arducam', executable='tof_pointcloud', output='screen'),
        Node(package='domain_bridge', executable='domain_bridge', name='bridge_with_pc',
             arguments=[LaunchConfiguration('bridge_config')], output='screen'),
    ])
