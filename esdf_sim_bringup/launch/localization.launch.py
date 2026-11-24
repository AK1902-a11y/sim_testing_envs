#!/usr/bin/env python3
"""Localization: simulation + map_server + AMCL on a saved map.

  ros2 launch esdf_sim_bringup localization.launch.py [map:=<name or path .yaml>] [world:=simple_room]
`map` defaults to the map named like the world, from esdf_sim_bringup/maps/<name>.yaml.
Needs nav2_amcl and nav2_lifecycle_manager (source ~/nav2_ws/install/setup.bash).
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    bringup = get_package_share_directory('esdf_sim_bringup')
    params = LaunchConfiguration('params').perform(context)
    m = LaunchConfiguration('map').perform(context) or LaunchConfiguration('world').perform(context)
    map_yaml = m if m.endswith('.yaml') else join(bringup, 'maps', f'{m}.yaml')
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[params]),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('esdf_sim_bringup')
    # Scoped: the nested launch files rewrite the `world` launch argument (to a full SDF path);
    # without the group that leaks out and breaks the map-name lookup below.
    sim = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'lidar_noise': LaunchConfiguration('lidar_noise'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_room'),
        DeclareLaunchArgument('map', default_value='', description='Map name or .yaml path (default: world name)'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('lidar_noise', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='localization.rviz'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('params', default_value=join(bringup, 'config', 'amcl.yaml')),
        sim,
        OpaqueFunction(function=_nodes),
    ])
