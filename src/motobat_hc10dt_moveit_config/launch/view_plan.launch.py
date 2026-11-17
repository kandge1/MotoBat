"""
RViz for the 2.5 view plan: robot_state_publisher + RViz, nothing else.

    ros2 launch motobat_hc10dt_moveit_config view_plan.launch.py

Normally started by scripts/view_plan.py, which also starts the module and the player.

show_view_plan.py publishes /joint_states and the module's place, so there is no
move_group, no controller and no joint_state_publisher here (two joint-state
sources would fight). Light enough to run beside a dataset job: no Gazebo.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare('motobat_hc10dt_moveit_config')
    urdf = PathJoinSubstitution([pkg, 'config', 'motobat_inspector.urdf.xacro'])
    description = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution([pkg, 'config', 'view_plan.rviz'])]),
    ])
