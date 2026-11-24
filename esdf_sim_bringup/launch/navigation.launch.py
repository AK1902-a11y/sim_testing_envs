#!/usr/bin/env python3
"""Full navigation: simulation + localization (map_server, AMCL) + Nav2 servers.

  ros2 launch esdf_sim_bringup navigation.launch.py [world:=simple_room] [map:=<name|path.yaml>]
Controller: Regulated Pure Pursuit (config/nav2_params.yaml). Send goals with RViz "2D Goal Pose",
or let the route_runner node drive the goal list of a testing_env:
  ros2 launch esdf_sim_bringup navigation.launch.py world:=testing_env2 env:=testing_env2 [route_laps:=2 route_auto_start:=true]
  ros2 service call /route_runner/start  std_srvs/srv/Trigger {}      # /route_runner/cancel to stop
Needs the Nav2 packages built in ~/nav2_ws (source ~/nav2_ws/install/setup.bash first).
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('esdf_sim_bringup')
    params = LaunchConfiguration('nav_params')

    # Scoped: nested launches rewrite launch arguments such as `world` (see localization.launch.py).
    localization = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bringup, 'launch', 'localization.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'headless': LaunchConfiguration('headless'),
            'lidar_noise': LaunchConfiguration('lidar_noise'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': 'navigation.rviz',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())])

    def nav2_node(pkg, exe, name, remaps=None):
        return Node(package=pkg, executable=exe, name=name, output='screen',
                    parameters=[params], remappings=remaps or [])

    nodes = [
        nav2_node('nav2_controller', 'controller_server', 'controller_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        nav2_node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation'),
    ]

    # Idle until /route_runner/start is called (or route_auto_start:=true). With env:='' it has no route and does nothing.
    route_runner = Node(
        package='esdf_sim_bringup', executable='route_runner.py', name='route_runner', output='screen',
        parameters=[{
            'use_sim_time': True,
            'env': LaunchConfiguration('env'),
            'laps': ParameterValue(LaunchConfiguration('route_laps'), value_type=int),
            'auto_start': ParameterValue(LaunchConfiguration('route_auto_start'), value_type=bool),
            'status_file': LaunchConfiguration('route_status_file'),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('env', default_value='', description='testing_env whose goals route_runner drives'),
        DeclareLaunchArgument('route_laps', default_value='1'),
        DeclareLaunchArgument('route_auto_start', default_value='false'),
        DeclareLaunchArgument('route_status_file', default_value='', description='JSON status file for the GUI'),
        DeclareLaunchArgument('world', default_value='simple_room'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('lidar_noise', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('nav_params', default_value=join(bringup, 'config', 'nav2_params.yaml')),
        localization,
        *nodes,
        route_runner,
    ])
