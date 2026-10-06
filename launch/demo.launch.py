import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Everything on one host: the LeKiwi's ros2_control stack (mock hardware by default), move_group,
    # and RViz with the MotionPlanning panel.
    pkg_lekiwi_bringup = get_package_share_directory('lekiwi_bringup')
    pkg_lekiwi_moveit_config = get_package_share_directory('lekiwi_moveit_config')

    # Launch arguments
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    port = LaunchConfiguration('port')
    rviz = LaunchConfiguration('rviz')

    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock_components/GenericSystem instead of the real servos'
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port of the servo bus'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz with the MotionPlanning panel'
    )

    # Hardware, robot_state_publisher and controllers
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lekiwi_bringup, 'launch', 'hardware_control.launch.py')),
        launch_arguments={'use_mock_hardware': use_mock_hardware, 'port': port}.items(),
    )

    # MoveIt
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lekiwi_moveit_config, 'launch', 'move_group.launch.py')),
    )

    # RViz
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lekiwi_moveit_config, 'launch', 'moveit_rviz.launch.py')),
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_use_mock_hardware,
        declare_port,
        declare_rviz,
        hardware,
        move_group,
        moveit_rviz,
    ])
