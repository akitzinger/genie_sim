# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""Bring up the dual-arm pick-and-place task node.

Assumes `move_group` is already running (`ros2 launch genie_sim_moveit
wbc.launch.py`) — this node is a client of the existing action server.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _launch_setup(context):
    # Only set the `pose_config` parameter when the launch arg is non-empty
    # — otherwise let the node fall back to its packaged default.
    pose_config = context.launch_configurations.get("pose_config", "")
    params = {"move_group_timeout_sec": 30.0}
    if pose_config:
        params["pose_config"] = pose_config

    return [
        Node(
            package="genie_sim_dual_arm_manip",
            executable="dual_arm_pick_place_task.py",
            name="dual_arm_pick_place_task",
            output="screen",
            parameters=[params],
        )
    ]


def generate_launch_description():
    pose_config_arg = DeclareLaunchArgument(
        "pose_config",
        default_value="",
        description=(
            "Path to a pick_place_poses.yaml. Defaults to the package's own "
            "config/pick_place_poses.yaml when left empty."
        ),
    )

    return LaunchDescription(
        [
            pose_config_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
