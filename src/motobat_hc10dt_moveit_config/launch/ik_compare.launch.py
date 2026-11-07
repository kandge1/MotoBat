"""
move_group alone, with a chosen kinematics file. Used by scripts/compare_ik.py.

    ros2 launch motobat_hc10dt_moveit_config ik_compare.launch.py \
        kinematics:=config/ik_candidates/trac_ik_speed.yaml

`kinematics` is relative to this package's share directory, or absolute.
No controllers, no RViz: /compute_ik, /compute_fk and /check_state_validity
take an explicit robot state, so nothing has to publish /joint_states.
robot_state_publisher is still needed: move_group resolves a request's frame
through TF, and the fixed world -> s3_base_link mount is a static transform
only robot_state_publisher broadcasts.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _move_group(context):
    kinematics = LaunchConfiguration('kinematics').perform(context)
    config = (MoveItConfigsBuilder('motobat_inspector', package_name='motobat_hc10dt_moveit_config')
              .robot_description_kinematics(file_path=kinematics)
              .to_moveit_configs())
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[config.robot_description]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[config.to_dict()]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('kinematics', default_value='config/kinematics.yaml'),
        OpaqueFunction(function=_move_group),
    ])
