import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def lekiwi_moveit_config():
    # The LeKiwi model (config/lekiwi.urdf.xacro, found through .setup_assistant) and SRDF from
    # this package; the arm's kinematics, joint limits and controller mapping from
    # so101_moveit_config, since the arm and its controllers are the same.
    so101_config = os.path.join(get_package_share_directory('so101_moveit_config'), 'config')
    return (
        MoveItConfigsBuilder('lekiwi', package_name='lekiwi_moveit_config')
        .robot_description_kinematics(file_path=os.path.join(so101_config, 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(so101_config, 'joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.join(so101_config, 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
