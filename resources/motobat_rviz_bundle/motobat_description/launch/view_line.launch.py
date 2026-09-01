#!/usr/bin/env python3
"""
MotoBat — view the full four-station line in RViz.

    ros2 launch motobat_description view_line.launch.py

Brings up:
  robot_state_publisher   turns /joint_states + URDF into the TF tree
  joint_state_publisher_gui  sliders for all 24 joints
  rviz2                   the display

Drag the sliders and the arms move. This is pure kinematics -- no physics,
no controllers, no simulator. It is the fastest possible check that the
descriptions, TF tree, and meshes are all correct.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('motobat_description')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Show the joint_state_publisher slider GUI')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz')

    # xacro is expanded at launch time, so editing the .xacro and relaunching
    # is all it takes -- no rebuild needed for description-only changes.
    robot_description = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([pkg, 'urdf', 'motobat_line.urdf.xacro'])
        ])
    }

    return LaunchDescription([
        gui_arg,
        rviz_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'motobat_line.rviz'])],
        ),
    ])
