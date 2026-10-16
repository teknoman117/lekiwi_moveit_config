import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def lekiwi_moveit_config(package='lekiwi_moveit_config', robot_name='lekiwi',
                         urdf='config/lekiwi.urdf.xacro', srdf='config/lekiwi.srdf.xacro'):
    """MoveIt config for the LeKiwi, or for a robot built on it.

    The model and SRDF come from 'package' (paths relative to it); the arm's kinematics, joint
    limits and controller mapping from so101_moveit_config, since the arm and its controllers are
    the same. A robot that adds parts to the LeKiwi passes its own package, URDF (its robot plus
    config/ground.xacro) and SRDF (config/lekiwi_semantic.xacro plus its own collision pairs).
    """
    so101_config = os.path.join(get_package_share_directory('so101_moveit_config'), 'config')
    return (
        MoveItConfigsBuilder(robot_name, package_name=package)
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path=srdf)
        .robot_description_kinematics(file_path=os.path.join(so101_config, 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(so101_config, 'joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.join(so101_config, 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
