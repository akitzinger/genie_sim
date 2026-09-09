# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0
"""Thin synchronous wrapper around the moveit_msgs/MoveGroup action.

Talks directly to the already-running `move_group` node's `/move_action`
server (see genie_sim_moveit/launch/wbc.launch.py). This package does not
start its own planning-scene monitor / robot-model copy (unlike moveit_py),
so it stays decoupled from whichever MoveIt config launched move_group.
"""
from __future__ import annotations

import threading
from typing import List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

MOVEIT_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS


class MoveGroupError(RuntimeError):
    pass


class MoveGroupActionClient:
    def __init__(self, node: Node, group_name: str, callback_group: Optional[CallbackGroup] = None):
        self._node = node
        self.group_name = group_name
        self._client = ActionClient(node, MoveGroup, "/move_action", callback_group=callback_group)

    def wait_for_server(self, timeout_sec: float = 10.0) -> None:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            raise MoveGroupError("move_group action server '/move_action' not available")

    @staticmethod
    def _joint_constraints(joint_names: List[str], positions: List[float], tolerance: float) -> Constraints:
        constraints = Constraints()
        for name, pos in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    @staticmethod
    def _pose_constraint(
        link_name: str, pose: PoseStamped, pos_tolerance: float, angle_tolerance: float
    ) -> Constraints:
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = link_name
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tolerance]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = link_name
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = angle_tolerance
        oc.absolute_y_axis_tolerance = angle_tolerance
        oc.absolute_z_axis_tolerance = angle_tolerance
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return constraints

    def _send(self, constraints: Constraints, timeout_sec: float) -> None:
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.goal_constraints = [constraints]
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        done_event = threading.Event()
        outcome: dict = {}

        def _on_result(result_future) -> None:
            result = result_future.result()
            outcome["status"] = result.status
            outcome["error_code"] = result.result.error_code.val
            done_event.set()

        def _on_goal_response(goal_future) -> None:
            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                outcome["error"] = f"move_group rejected the goal for group '{self.group_name}'"
                done_event.set()
                return
            goal_handle.get_result_async().add_done_callback(_on_result)

        self._client.send_goal_async(goal).add_done_callback(_on_goal_response)

        if not done_event.wait(timeout_sec):
            raise MoveGroupError(f"move_group goal for group '{self.group_name}' timed out after {timeout_sec}s")

        if "error" in outcome:
            raise MoveGroupError(outcome["error"])

        if outcome["status"] != GoalStatus.STATUS_SUCCEEDED or outcome["error_code"] != MOVEIT_SUCCESS:
            raise MoveGroupError(
                f"move_group goal for group '{self.group_name}' failed: "
                f"status={outcome['status']} error_code={outcome['error_code']}"
            )

    def move_to_joint_targets(
        self,
        joint_names: List[str],
        positions: List[float],
        tolerance: float = 0.01,
        timeout_sec: float = 30.0,
    ) -> None:
        self._send(self._joint_constraints(joint_names, positions, tolerance), timeout_sec)

    def move_to_poses(
        self,
        link_poses: List[Tuple[str, PoseStamped]],
        pos_tolerance: float = 0.01,
        angle_tolerance: float = 0.05,
        timeout_sec: float = 30.0,
    ) -> None:
        """`link_poses`: one (link_name, PoseStamped) pair per end-effector to constrain."""
        if not link_poses:
            raise ValueError("move_to_poses requires at least one (link_name, pose) pair")
        combined = Constraints()
        for link_name, pose in link_poses:
            sub = self._pose_constraint(link_name, pose, pos_tolerance, angle_tolerance)
            combined.position_constraints.extend(sub.position_constraints)
            combined.orientation_constraints.extend(sub.orientation_constraints)
        self._send(combined, timeout_sec)
