# lekiwi_moveit_config

MoveIt 2 configuration for the LeKiwi mobile manipulator: plans for the SO-101 arm and checks it
for collisions with the LeKiwi base and the ground. For the arm on its own, use
`so101_moveit_config`.

## What is shared with so101_moveit_config

The planning groups (`arm`, `arm_position_only`, `gripper`), named poses, kinematics
(`kinematics.yaml`), joint limits and controller mapping are the arm's, so
`launch/moveit_config.py` loads those files from `so101_moveit_config`. This package adds:

- `config/lekiwi.urdf.xacro`: `lekiwi_bringup/urdf/lekiwi.urdf.xacro` plus a `ground` link (a
  2 m x 2 m box whose top is at z = 0) fixed to `base_footprint`, so the floor moves with the robot.
  Only move_group and the MoveIt RViz panel load it; `robot_state_publisher` and ros2_control keep
  the bringup URDF. The joints are the same.
- `config/lekiwi.srdf`: the arm's groups, and the disabled collision pairs for the whole robot.
  Every arm link from `shoulder_link` down stays checked against the base parts and the ground.
  Disabled: links joined by a joint, pairs in contact at the zero pose (the motor mounts and the
  upper plate), and pairs of links that do not move relative to each other (base, wheels, arm
  mount, ground) and never collided in 2000 random poses.

The base's collision shapes are simple primitives from `lekiwi_description` (cylinders for the
plates and wheels, boxes for the motor mounts), not its meshes.

## Launch files

| Launch | Starts |
|---|---|
| `demo.launch.py` | the LeKiwi ros2_control stack (`lekiwi_bringup/hardware_control.launch.py`, mock hardware by default), `move_group`, RViz |
| `move_group.launch.py` | `move_group` only |
| `moveit_rviz.launch.py` | RViz with the MotionPlanning panel only (fixed frame `base_footprint`) |

The robot runs the hardware and controllers; planning runs on the workstation (same
`ROS_DOMAIN_ID`):

```bash
# robot
ros2 launch lekiwi_bringup hardware_control.launch.py
# workstation
ros2 launch lekiwi_moveit_config move_group.launch.py
ros2 launch lekiwi_moveit_config moveit_rviz.launch.py
```

The host that runs move_group needs a clock synchronized with the robot's (for example chrony on
both): before it executes a trajectory, MoveIt waits up to 1 s for a `/joint_states` message
stamped at or after its own current time, and aborts the execution if none comes. Without clock
sync, run move_group on the robot instead.
