"""The GP4 MoveIt configuration for one station: s4a (busbar carrier) or s5 (CMC and cover).

Robot identity lives in a launch argument, not in a package (project_structure.md §3.4): both
stations share the kinematics, planners and RViz setup; the files that name joints are per
station (config/<station>.srdf, <station>_joint_limits.yaml, <station>_moveit_controllers.yaml,
<station>_initial_positions.yaml). ros2_controllers.yaml holds both stations' controllers,
because moveit_configs_utils' demo launch reads that one file by path; the spawner starts only
the controller this station's moveit_controllers names.
"""
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

STATIONS = ('s4a', 's5')


def config(station):
    if station not in STATIONS:
        raise ValueError(f'station must be one of {STATIONS}, got {station!r}')
    share = get_package_share_directory('motobat_gp4_moveit_config')
    return (MoveItConfigsBuilder('motobat_gp4_station', package_name='motobat_gp4_moveit_config')
            .robot_description(file_path='config/motobat_gp4_station.urdf.xacro',
                               mappings={'station': station,
                                         'initial_positions_file': f'{share}/config/{station}_initial_positions.yaml'})
            .robot_description_semantic(file_path=f'config/{station}.srdf')
            .robot_description_kinematics(file_path='config/kinematics.yaml')
            .joint_limits(file_path=f'config/{station}_joint_limits.yaml')
            .trajectory_execution(file_path=f'config/{station}_moveit_controllers.yaml')
            .to_moveit_configs())
