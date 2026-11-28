"""moveit_rviz for one GP4 station:  ros2 launch motobat_gp4_moveit_config moveit_rviz.launch.py station:=s4a|s5"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_moveit_rviz_launch

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from gp4_config import config  # noqa: E402


def _launch(context):
    return [generate_moveit_rviz_launch(config(LaunchConfiguration('station').perform(context)))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('station', default_value='s4a',
                                                    description='s4a (busbar carrier) or s5 (CMC and cover)'),
                              OpaqueFunction(function=_launch)])
