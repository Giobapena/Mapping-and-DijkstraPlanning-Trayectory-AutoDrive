import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'global_planner'
REPO = os.path.join(os.path.expanduser('~'),
                    'Mapping-and-DijkstraPlanning-Trayectory-AutoDrive')
WP = os.path.join(REPO, 'src', PKG, 'waypoints')


def generate_launch_description():
    share = get_package_share_directory(PKG)
    use_sim = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml',
                              default_value=os.path.join(share, 'maps', 'F1tenth_Map.yaml')),
        DeclareLaunchArgument('smooth_csv',
                              default_value=os.path.join(WP, 'dijkstra_waypoints_smooth.csv')),
        DeclareLaunchArgument('raw_csv',
                              default_value=os.path.join(WP, 'dijkstra_waypoints.csv')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'),
                          'topic_name': 'saved_map',
                          'frame_id': 'map',
                          'use_sim_time': use_sim}]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server'],
                          'use_sim_time': use_sim}]),

        Node(package=PKG, executable='global_path_publisher',
             name='global_path_publisher', output='screen',
             parameters=[{'smooth_csv': LaunchConfiguration('smooth_csv'),
                          'raw_csv': LaunchConfiguration('raw_csv'),
                          'frame_id': 'map', 'use_sim_time': use_sim}]),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'trajectory.rviz')],
             parameters=[{'use_sim_time': use_sim}]),
    ])
