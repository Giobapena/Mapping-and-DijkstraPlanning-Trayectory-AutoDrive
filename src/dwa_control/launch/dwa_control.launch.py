import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('dwa_control'), 'config', 'dwa_params.yaml')

    default_csv = os.path.expanduser(
        '~/Mapping-and-DijkstraPlanning-Trayectory-AutoDrive/src/global_planner/'
        'waypoints/dijkstra_waypoints_smooth.csv')

    return LaunchDescription([
        DeclareLaunchArgument('path_csv', default_value=default_csv),
        DeclareLaunchArgument('v_max', default_value='2.0'),
        DeclareLaunchArgument('total_laps', default_value='10'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='dwa_control',
            executable='dwa_controller',
            name='dwa_controller',
            output='screen',
            emulate_tty=True,
            parameters=[params, {
                'path_csv': LaunchConfiguration('path_csv'),
                'v_max': LaunchConfiguration('v_max'),
                'total_laps': LaunchConfiguration('total_laps'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
