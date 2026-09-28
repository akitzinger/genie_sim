# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""Thin synchronous wrapper around the moveit_msgs/MoveGroup action.

Talks directly to the already-running `move_group` node's `/move_action`
server (see genie_sim_moveit/launch/wbc.launch.py). This package does not
start its own planning-scene monitor / robot-model copy (unlike moveit_py),
so it stays decoupled from whichever MoveIt config launched move_group.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive

MOVEIT_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS

# Latched (transient-local) so an RViz "Pose" display added after the goal was
# sent still picks up the last requested pose for that link.
_DEBUG_POSE_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class MoveGroupError(RuntimeError):
    pass


@dataclass
class OrientationPathConstraint:
    """One `moveit_msgs/OrientationConstraint` to hold along the whole planned path.

    `MotionPlanRequest.path_constraints` is a single `Constraints` (not
    `PathConstraints` -- that message type doesn't exist in moveit_msgs), so
    multiple of these get merged into one `Constraints.orientation_constraints`
    list by `MoveGroupActionClient._path_constraints`.
    """

    link_name: str
    frame_id: str
    orientation_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    x_tolerance: float = 0.1
    y_tolerance: float = 0.1
    z_tolerance: float = 3.14159
    weight: float = 1.0


class MoveGroupActionClient:
    def __init__(self, node: Node, group_name: str, callback_group: Optional[CallbackGroup] = None):
        self._node = node
        # Default group, used when a call site doesn't pass its own `group_name`.
        self.group_name = group_name
        self._client = ActionClient(node, MoveGroup, "/move_action", callback_group=callback_group)
        # One latched PoseStamped publisher per constrained link, created on first
        # use -- add an RViz "Pose" display on `~/debug/target_pose/<link_name>` to
        # see the Cartesian goal a move_to_poses/move_to_targets call is requesting.
        self._debug_pose_pubs: dict = {}

    def wait_for_server(self, timeout_sec: float = 10.0) -> None:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            raise MoveGroupError("move_group action server '/move_action' not available")

    def _publish_debug_pose(self, link_name: str, pose: PoseStamped) -> None:
        pub = self._debug_pose_pubs.get(link_name)
        if pub is None:
            pub = self._node.create_publisher(PoseStamped, f"~/debug/target_pose/{link_name}", _DEBUG_POSE_QOS)
            self._debug_pose_pubs[link_name] = pub
        pub.publish(pose)

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

    @staticmethod
    def _path_constraints(path_constraints: Optional[List[OrientationPathConstraint]]) -> Constraints:
        constraints = Constraints()
        for pc in path_constraints or []:
            oc = OrientationConstraint()
            oc.header.frame_id = pc.frame_id
            oc.link_name = pc.link_name
            oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = pc.orientation_xyzw
            oc.absolute_x_axis_tolerance = pc.x_tolerance
            oc.absolute_y_axis_tolerance = pc.y_tolerance
            oc.absolute_z_axis_tolerance = pc.z_tolerance
            oc.weight = pc.weight
            constraints.orientation_constraints.append(oc)
        return constraints

    def _send(self, constraints: Constraints, path_constraints: Constraints, timeout_sec: float, group_name: Optional[str] = None) -> None:
        resolved_group = group_name or self.group_name

        req = MotionPlanRequest()
        req.group_name = resolved_group
        req.goal_constraints = [constraints]
        req.path_constraints = path_constraints
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

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
                outcome["error"] = f"move_group rejected the goal for group '{resolved_group}'"
                done_event.set()
                return
            goal_handle.get_result_async().add_done_callback(_on_result)

        self._client.send_goal_async(goal).add_done_callback(_on_goal_response)

        if not done_event.wait(timeout_sec):
            raise MoveGroupError(f"move_group goal for group '{resolved_group}' timed out after {timeout_sec}s")

        if "error" in outcome:
            raise MoveGroupError(outcome["error"])

        if outcome["status"] != GoalStatus.STATUS_SUCCEEDED or outcome["error_code"] != MOVEIT_SUCCESS:
            raise MoveGroupError(
                f"move_group goal for group '{resolved_group}' failed: "
                f"status={outcome['status']} error_code={outcome['error_code']}"
            )

    def move_to_targets(
        self,
        joint_names: List[str],
        positions: List[float],
        link_poses: List[Tuple[str, PoseStamped]] = (),
        tolerance: float = 0.01,
        pos_tolerance: float = 0.03,
        angle_tolerance: float = 0.2,
        timeout_sec: float = 30.0,
        group_name: Optional[str] = None,
        path_constraints: Optional[List[OrientationPathConstraint]] = None,
    ) -> None:
        """Send one MoveGroup goal mixing joint-space targets (`joint_names`/`positions`) with
        Cartesian targets (`link_poses`, one (link_name, PoseStamped) pair per constrained
        end-effector) -- lets different sub-groups of the same planning group each use
        whichever goal type suits them (e.g. torso via joints, an arm via a Cartesian pose).
        """
        if not joint_names and not link_poses:
            raise ValueError("move_to_targets requires at least one joint target or (link_name, pose) pair")
        goal_constraints = self._joint_constraints(joint_names, positions, tolerance)
        for link_name, pose in link_poses:
            self._publish_debug_pose(link_name, pose)
            sub = self._pose_constraint(link_name, pose, pos_tolerance, angle_tolerance)
            goal_constraints.position_constraints.extend(sub.position_constraints)
            goal_constraints.orientation_constraints.extend(sub.orientation_constraints)
        self._send(goal_constraints, self._path_constraints(path_constraints), timeout_sec, group_name)

    def move_to_joint_targets(
        self,
        joint_names: List[str],
        positions: List[float],
        tolerance: float = 0.01,
        timeout_sec: float = 30.0,
        group_name: Optional[str] = None,
        path_constraints: Optional[List[OrientationPathConstraint]] = None,
    ) -> None:
        self.move_to_targets(
            joint_names,
            positions,
            tolerance=tolerance,
            timeout_sec=timeout_sec,
            group_name=group_name,
            path_constraints=path_constraints,
        )

    def move_to_poses(
        self,
        link_poses: List[Tuple[str, PoseStamped]],
        pos_tolerance: float = 0.01,
        angle_tolerance: float = 0.05,
        timeout_sec: float = 30.0,
        group_name: Optional[str] = None,
        path_constraints: Optional[List[OrientationPathConstraint]] = None,
    ) -> None:
        """`link_poses`: one (link_name, PoseStamped) pair per end-effector to constrain."""
        if not link_poses:
            raise ValueError("move_to_poses requires at least one (link_name, pose) pair")
        self.move_to_targets(
            [],
            [],
            link_poses,
            pos_tolerance=pos_tolerance,
            angle_tolerance=angle_tolerance,
            timeout_sec=timeout_sec,
            group_name=group_name,
            path_constraints=path_constraints,
        )
