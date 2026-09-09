# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0
"""Stub perception hook — the extension point for camera-based target poses.

For every state with `perception_override: true` in the pose config, this
subscribes to `~/target_pose/<state>/left` and `~/target_pose/<state>/right`
and latches the most recent `geometry_msgs/PoseStamped` per side. No
perception is implemented here: until an external grasp-pose estimator
publishes on these topics, `get_pose(...)` returns None and callers fall
back to the configured joint-space target.
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from genie_sim_dual_arm_manip.task_states import TaskState

_Key = Tuple[TaskState, str]


class PerceptionOverride:
    def __init__(self, node: Node, overridable_states: Dict[TaskState, bool]):
        self._node = node
        self._poses: Dict[_Key, Optional[PoseStamped]] = {}
        self._subs = []
        for state, enabled in overridable_states.items():
            if not enabled:
                continue
            for side in ("left", "right"):
                key: _Key = (state, side)
                self._poses[key] = None
                topic = f"~/target_pose/{state.value}/{side}"
                self._subs.append(
                    node.create_subscription(
                        PoseStamped, topic, self._make_callback(key), qos_profile_sensor_data
                    )
                )
                node.get_logger().info(f"perception override enabled for '{state.value}'/{side} on {topic}")

    def _make_callback(self, key: _Key):
        def _cb(msg: PoseStamped) -> None:
            self._poses[key] = msg

        return _cb

    def get_pose(self, state: TaskState, side: str) -> Optional[PoseStamped]:
        return self._poses.get((state, side))

    def has_any_override(self, state: TaskState) -> bool:
        return self.get_pose(state, "left") is not None or self.get_pose(state, "right") is not None
