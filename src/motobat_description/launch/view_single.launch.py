#!/usr/bin/env python3
"""
MotoBat — view ONE arm in isolation.

    ros2 launch motobat_description view_single.launch.py arm:=gp4
    ros2 launch motobat_description view_single.launch.py arm:=gp7
    ros2 launch motobat_description view_single.launch.py arm:=hc10dt

Useful when checking a single robot's meshes or joint limits without the
visual clutter of the full line.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm = LaunchConfiguration('arm')

    # Each vendor package ships a standalone top-level xacro named <arm>.xacro
    xacro_path = PathJoinSubstitution([
        FindPackageShare(['motoman_', arm, '_support']), 'urdf', [arm, '.xacro']
    ])

    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='gp4',
                              choices=['gp4', 'gp7', 'gp8', 'hc10', 'hc10dt'],
                              description='Which Yaskawa arm to display'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', xacro_path]), value_type=str)}],
        ),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', output='screen'),
    ])
