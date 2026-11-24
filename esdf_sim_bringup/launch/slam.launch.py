#!/usr/bin/env python3
"""Mapping: simulation + SLAM Toolbox (online async). Drive with teleop, then save the map.

  ros2 launch esdf_sim_bringup slam.launch.py [world:=simple_room] [headless:=false]
Params: config/slam_toolbox_mapping.yaml.  Save maps into maps/ (see README / chat).
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('esdf_sim_bringup')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'lidar_noise': LaunchConfiguration('lidar_noise'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'slam.rviz',
        }.items())

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[LaunchConfiguration('slam_params'), {'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_room'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('lidar_noise', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam_params',
                              default_value=join(bringup, 'config', 'slam_toolbox_mapping.yaml')),
        sim, slam,
    ])
