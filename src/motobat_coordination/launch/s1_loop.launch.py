"""The S1 correction loop on mock hardware: GP7 MoveIt stack + S1 station + correction node.

    ros2 launch motobat_coordination s1_loop.launch.py gains:="[0.5,0.5,0.5]" modules:=3 label:=closed_k05
    ros2 launch motobat_coordination s1_loop.launch.py gains:="[0,0,0]" label:=open

Everything stops when the correction node has finished its modules (data/loop/<label>.json).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    station = Node(package='motobat_coordination', executable='s1_station.py', output='screen',
                   parameters=[{'seed': int(lc('seed')), 'image_mode': lc('image_mode'),
                                'image_reliable': lc('image_reliable').lower() == 'true'}])
    loop = Node(package='motobat_coordination', executable='correction_node.py', output='screen',
                parameters=[{'modules': int(lc('modules')), 'gains': [float(v) for v in yaml.safe_load(lc('gains'))],
                             'label': lc('label'), 'out': lc('out')}])
    acts = [TimerAction(period=8.0, actions=[station]), TimerAction(period=10.0, actions=[loop]),
            RegisterEventHandler(OnProcessExit(target_action=loop, on_exit=[EmitEvent(event=Shutdown())]))]
    if lc('bag'):
        # task 3.6: everything the latency chain needs, including the action's (hidden) status topic,
        # whose ACCEPTED entries mark when each placement goal starts
        acts.insert(0, ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', lc('bag'), '--include-hidden-topics',
                                           '/s1/stack_camera/image', '/s1/defect_report', '/s1/correction',
                                           '/correction_cmd/_action/status'], output='screen'))
    return acts


def generate_launch_description():
    gp7 = get_package_share_directory('motobat_gp7_moveit_config')
    return LaunchDescription([
        DeclareLaunchArgument('gains', default_value='[0.5, 0.5, 0.5]', description='[Kx, Ky, Kyaw]; [0,0,0] = open loop'),
        DeclareLaunchArgument('modules', default_value='3'),
        DeclareLaunchArgument('label', default_value='run'),
        DeclareLaunchArgument('out', default_value=os.path.join(os.getcwd(), 'data/loop')),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('bag', default_value='', description='record the latency topics to this rosbag2 path'),
        DeclareLaunchArgument('image_mode', default_value='roi', description='roi | full (a 10 MB frame)'),
        DeclareLaunchArgument('image_reliable', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gp7, 'launch/demo.launch.py')),
                                 launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()),
        OpaqueFunction(function=_nodes),
    ])
