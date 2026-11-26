"""
Station 3, the Inspector (HC10DT), in Gazebo Harmonic, driven by MoveIt. (task 2.2)

    ros2 launch motobat_bringup inspector_gazebo.launch.py              # Gazebo GUI + RViz
    ros2 launch motobat_bringup inspector_gazebo.launch.py gui:=false rviz:=false   # headless
    ros2 launch motobat_bringup inspector_gazebo.launch.py world:=line   # with the line (2.6)

world:=line loads worlds/inspection_line.sdf: the conveyor, the fixed line-scan gate and a
defectgen module on a pallet (model://motobat_pallet, generated into $MOTOBAT_ROOT/data/gazebo/
models by scripts/gazebo_module.py), and bridges the sensors and the pallet to ROS:
    /zivid/points     sensor_msgs/PointCloud2   wrist depth camera (s3_zivid_gz)
    /ljx/scan         sensor_msgs/LaserScan     wrist laser line (s3_ljx_gz)
    /ranger3/scan     sensor_msgs/LaserScan     the fixed gate's laser line (frame ranger3)
    /pallet/x         std_msgs/Float64          pallet position command, m (0 = at the Inspector)
    /pallet/joint_states sensor_msgs/JointState  where the pallet is

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
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
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
WORLD_LINE = os.path.join(get_package_share_directory('motobat_bringup'), 'worlds', 'inspection_line.sdf')
GENERATED_MODELS = os.path.join(os.environ.get('MOTOBAT_ROOT', os.getcwd()), 'data', 'gazebo', 'models')


def _nodes(context):
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    line = LaunchConfiguration('world').perform(context) == 'line'
    moveit_config = (MoveItConfigsBuilder(ROBOT, package_name=MOVEIT_PKG)
                     .robot_description(mappings={'sim': 'gazebo'})
                     .to_moveit_configs())
    sim_time = {'use_sim_time': True}

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        # -r: run on start. -s: server only (no GUI). empty.sdf: ground plane, sun, physics.
        launch_arguments={'gz_args': ('-r ' + ('' if gui else '-s ') + (WORLD_LINE if line else 'empty.sdf')),
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
                arguments=['-d', os.path.join(get_package_share_directory('motobat_bringup'), 'rviz', 'line_demo.rviz')
                           if line else os.path.join(get_package_share_directory(MOVEIT_PKG), 'config', 'moveit.rviz')],
                parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                            moveit_config.joint_limits, sim_time])

    extra = []
    if line:
        world = 'inspection_line'
        extra.append(Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                          name='line_bridge', parameters=[sim_time], arguments=[
                              '/zivid/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                              '/ljx/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                              '/ranger3/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                              '/pallet/x@std_msgs/msg/Float64]gz.msgs.Double',
                              f'/world/{world}/model/motobat_pallet/joint_state'
                              '@sensor_msgs/msg/JointState[gz.msgs.Model'],
                          remappings=[(f'/world/{world}/model/motobat_pallet/joint_state',
                                       '/pallet/joint_states')]))
        # The gate's sensor frame, as placed in inspection_line.sdf (gate_x in line_layout.xacro).
        extra.append(Node(package='tf2_ros', executable='static_transform_publisher', output='log',
                          parameters=[sim_time], arguments=[
                              '--x', '0.659', '--y', '0', '--z', '1.501', '--roll', '0', '--pitch', '1.5707963',
                              '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'ranger3']))
    return [gz, rsp, spawn, clock, after_spawn, move_group, rviz] + extra


def generate_launch_description():
    resource_paths = [AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory(p)))
        for p in MESH_PACKAGES]
    resource_paths.append(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', GENERATED_MODELS))
    # Hybrid-graphics laptops (25 Nov 2026): Gazebo renders its sensors on the integrated GPU by
    # default, at a real-time factor of 0.09 with the wrist and gate sensors. PRIME render
    # offload to the NVIDIA GPU runs the same world at 1.0. MOTOBAT_GZ_NVIDIA=0 turns it off.
    nvidia = []
    if shutil.which('nvidia-smi') and os.environ.get('MOTOBAT_GZ_NVIDIA', '1') != '0':
        nvidia = [SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
                  SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
                  SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                                         '/usr/share/glvnd/egl_vendor.d/10_nvidia.json')]
    return LaunchDescription(resource_paths + nvidia + [
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Gazebo GUI (false = server only)'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='empty', choices=['empty', 'line'],
                              description='empty: the arm alone (2.2); line: conveyor, gate, module (2.6)'),
        OpaqueFunction(function=_nodes),
    ])
