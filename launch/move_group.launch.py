import os
import sys

from moveit_configs_utils.launches import generate_move_group_launch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_config import lekiwi_moveit_config  # noqa: E402


def generate_launch_description():
    # move_group for the LeKiwi: plans for the arm and checks collisions with the base and the
    # ground. It needs the robot's /joint_states and the controllers' actions, so it can run on a
    # workstation whose clock is synchronized with the robot's, or on the robot.
    return generate_move_group_launch(lekiwi_moveit_config())
