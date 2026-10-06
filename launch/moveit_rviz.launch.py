import os
import sys

from moveit_configs_utils.launches import generate_moveit_rviz_launch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_config import lekiwi_moveit_config  # noqa: E402


def generate_launch_description():
    # RViz with the MotionPlanning panel. It talks to a running move_group, so it can run on a
    # workstation, with move_group on the workstation or the robot.
    return generate_moveit_rviz_launch(lekiwi_moveit_config())
