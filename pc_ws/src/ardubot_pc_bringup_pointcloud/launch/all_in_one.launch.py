import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ardubot_pc_bringup_pointcloud')
    desc = get_package_share_directory('ardubot_description')
    heading = get_package_share_directory('ardubot_heading_corrector')
    clearing = get_package_share_directory('ardubot_projected_map_clearing')
    with open(os.path.join(desc, 'urdf', 'ardubot.urdf')) as f:
        robot_description = f.read()

    def include(filename, condition=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)), condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_3d', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        # Stable mapping configuration retained from October: no scan-matching correction.
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_bno_static',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom_bno'], output='screen'),
        include('localization.launch.py'),
        include('slam.launch.py'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(heading, 'launch', 'heading_corrector.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('use_nav2'))),
        TimerAction(period=5.0, actions=[include('navigation.launch.py', IfCondition(LaunchConfiguration('use_nav2')))]),
        Node(package='octomap_server', executable='octomap_server_node', name='octomap_server',
             condition=IfCondition(LaunchConfiguration('use_3d')), output='screen',
             remappings=[('cloud_in', '/point_cloud2')],
             parameters=[{'frame_id': 'map', 'resolution': 0.025, 'latch': False,
                          'sensor_model.max_range': 2.5, 'filter_ground_plane': False,
                          'filter_speckles': True, 'occupancy_min_z': 0.0, 'occupancy_max_z': 0.06,
                          'sensor_model.hit': 0.65, 'sensor_model.miss': 0.40}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(clearing, 'launch', 'map_clearing.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('use_3d'))),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'ardubot.rviz')], output='screen'),
    ])
