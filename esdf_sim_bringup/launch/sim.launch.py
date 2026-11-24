#!/usr/bin/env python3
"""Simulation entry point: Gazebo + esdf_bot (+ RViz).

  ros2 launch esdf_sim_bringup sim.launch.py                    # simple_room, GUI, RViz
  ros2 launch esdf_sim_bringup sim.launch.py world:=empty rviz:=false headless:=true
`world` is a name resolved to esdf_sim_worlds/worlds/<name>.sdf (or esdf_sim_description's empty.sdf).
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _robot(context):
    desc = get_package_share_directory('esdf_sim_description')
    worlds = get_package_share_directory('esdf_sim_worlds')
    name = LaunchConfiguration('world').perform(context)
    path = name if name.endswith('.sdf') else join(worlds, 'worlds', f'{name}.sdf')
    if name == 'empty':
        path = join(desc, 'worlds', 'empty.sdf')
    args = {k: LaunchConfiguration(k).perform(context)
            for k in ('headless', 'lidar_noise', 'x', 'y', 'yaw')}
    args['world'] = path
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(desc, 'launch', 'robot.launch.py')),
        launch_arguments=args.items())]


def generate_launch_description():
    bringup = get_package_share_directory('esdf_sim_bringup')
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', PathJoinSubstitution([bringup, 'rviz', LaunchConfiguration('rviz_config')])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_room',
                              description='World name, or a path to an .sdf file'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='sim.rviz',
                              description='File in esdf_sim_bringup/rviz/'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('lidar_noise', default_value='0.0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_robot),
        rviz,
    ])
