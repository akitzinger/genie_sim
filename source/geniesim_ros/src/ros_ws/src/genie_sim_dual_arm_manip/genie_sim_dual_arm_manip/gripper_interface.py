# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""Minimal /joint_command wrapper for the two parallel-jaw grippers.

Same joint-name caveat as genie_sim_bringup/scripts/gripper_cmds.py: verify
`left_joint` / `right_joint` in config/pick_place_poses.yaml against the
bringup console log ("[genie_sim_engine] joints ...") for the robot/gripper
combo you launched — URDF->USD imports prefix joint names, hand-authored
USDs don't.
"""
from __future__ import annotations

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

from genie_sim_dual_arm_manip.pose_config import GripperConfig


class GripperInterface:
    def __init__(self, node: Node, config: GripperConfig):
        self._node = node
        self._config = config
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._pub = node.create_publisher(JointState, "/joint_command", qos)

    def _send(self, joint_name: str, position: float) -> None:
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = [joint_name]
        msg.position = [position]
        self._pub.publish(msg)

    def apply(self, side: str, command: str) -> None:
        """`command` is one of "open", "close", "hold" (no-op, keep current)."""
        if command == "hold":
            return
        if command not in ("open", "close"):
            raise ValueError(f"unknown gripper command '{command}' (want open/close/hold)")
        position = self._config.open_position if command == "open" else self._config.close_position
        joint = self._config.left_joint if side == "left" else self._config.right_joint
        self._send(joint, position)
