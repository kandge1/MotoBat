"""
Station 3, the Inspector (HC10DT), in Gazebo Harmonic, driven by MoveIt. (task 2.2)

    ros2 launch motobat_bringup inspector_gazebo.launch.py              # Gazebo GUI + RViz
    ros2 launch motobat_bringup inspector_gazebo.launch.py gui:=false rviz:=false   # headless

The control path is the one the real arm will use (project_plan.md §6):

    MoveIt (move_group) --FollowJointTrajectory--> inspector_arm_controller
        --position commands--> gz_ros2_control/GazeboSimSystem --> Gazebo joints
    Gazebo joints --> joint_state_broadcaster --> /joint_states --> MoveIt

Compared with demo.launch.py, only the hardware plugin changes (xacro sim:=gazebo),
and Gazebo's plugin hosts the controller_manager instead of ros2_control_node.
Isaac Sim will swap the plugin again; hardware swaps it for MotoROS2.

One simulation at a time across both worktree lanes (16 GB, ROS_DOMAIN_ID=42).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

ROBOT = 'motobat_inspector'
MOVEIT_PKG = 'motobat_hc10dt_moveit_config'
# Packages whose package:// meshes Gazebo has to find.
MESH_PACKAGES = ['motoman_hc10_support', 'motoman_resources', 'motobat_description']


def _nodes(context):
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    moveit_config = (MoveItConfigsBuilder(ROBOT, package_name=MOVEIT_PKG)
                     .robot_description(mappings={'sim': 'gazebo'})
                     .to_moveit_configs())
    sim_time = {'use_sim_time': True}

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        # -r: run on start. -s: server only (no GUI). empty.sdf: ground plane, sun, physics.
        launch_arguments={'gz_args': ('-r empty.sdf' if gui else '-r -s empty.sdf'),
                          'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[moveit_config.robot_description, sim_time])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', ROBOT])

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # Controllers start once the robot exists in Gazebo, i.e. once its plugin
    # has created the controller_manager.
    controllers = [Node(package='controller_manager', executable='spawner', output='screen',
                        arguments=[c, '--controller-manager', '/controller_manager'])
                   for c in ('joint_state_broadcaster', 'inspector_arm_controller')]
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=controllers))

    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[moveit_config.to_dict(), sim_time])

    rviz = Node(package='rviz2', executable='rviz2', output='log',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', os.path.join(get_package_share_directory(MOVEIT_PKG), 'config', 'moveit.rviz')],
                parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                            moveit_config.joint_limits, sim_time])

    return [gz, rsp, spawn, clock, after_spawn, move_group, rviz]


def generate_launch_description():
    resource_paths = [AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory(p)))
        for p in MESH_PACKAGES]
    return LaunchDescription(resource_paths + [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Gazebo GUI (false = server only)'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_nodes),
    ])
