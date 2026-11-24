#!/usr/bin/env python3
"""Launch Gazebo Fortress, spawn esdf_bot and bridge its topics to ROS 2.

Stack: ros_gz_sim (gz_args, gz_version=6 -> `ign gazebo`) + robot_state_publisher
       + ros_gz_sim create + ros_gz_bridge (ignition.msgs.* types, Humble).
"""
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('esdf_sim_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    headless = LaunchConfiguration('headless')
    lidar_noise = LaunchConfiguration('lidar_noise')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')
    robot = 'esdf_bot'

    robot_description = ParameterValue(
        Command(['xacro ', join(pkg, 'urdf', 'esdf_bot.urdf.xacro'),
                 ' lidar_noise:=', lidar_noise]),
        value_type=str)

    # `-r` = run immediately; `-s` = server only, no GUI (headless)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world, TextSubstitution(text=' -r')],
                          'gz_version': '6'}.items(),
        condition=UnlessCondition(headless))
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world, TextSubstitution(text=' -r -s --headless-rendering')],
                          'gz_version': '6'}.items(),
        condition=IfCondition(headless))

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', robot, '-allow_renaming', 'false',
                   '-x', x, '-y', y, '-z', '0.02', '-Y', yaw])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ['/world/', world_name, '/model/', robot,
             '/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'],
        ],
        remappings=[
            (['/world/', world_name, '/model/', robot, '/joint_state'], '/joint_states'),
        ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=join(pkg, 'worlds', 'empty.sdf'),
                              description='SDF world file (must be named per world_name)'),
        DeclareLaunchArgument('world_name', default_value='default',
                              description='<world name=...> inside the SDF'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server without GUI'),
        DeclareLaunchArgument('lidar_noise', default_value='0.0',
                              description='Lidar gaussian noise stddev [m]; 0 = noise free'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_sim, gz_sim_headless, rsp, spawn, bridge,
    ])
