#!/usr/bin/env python3
# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""Dual-arm pick-and-place task orchestrator.

Drives a configurable MoveIt planning group (see genie_sim_moveit) —
`wbc_fixed_headless` by default, i.e. torso + both arms — through a fixed
sequence of named states — default, pick_ready, pick, pick_hold, place_hold,
place_ready, place — sending one MoveGroup goal per transition and toggling the
grippers over /joint_command. Each transition is also exposed as its own
std_srvs/Trigger service so it can be triggered individually (e.g. from
RViz's "Service Caller" panel) instead of only as a scripted sequence.

Each state configures a full whole-body pose (body + head + both arms) in
config/pick_place_poses.yaml, and each state may pick its own `group_name`
(falling back to the top-level default) — only the sub-groups actually
spanned by that group are sent as goal constraints, see
pose_config.PosePlan.goal_joint_target.

Transitions are only allowed in sequence — default -> pick_ready -> pick ->
pick_hold -> place_hold -> place_ready -> place — one step at a time; `go_default` is always
allowed regardless of the current state (see `task_states.ORDER` and
`_check_transition`). Calling e.g. `go_pick` before `go_pick_ready` fails
with a Trigger `success=false` response.

Requires `move_group` to already be running (`ros2 launch genie_sim_moveit
wbc.launch.py`) — this node is a client of the existing action server, it
does not start its own planning-scene monitor.

Target poses per state come from config/pick_place_poses.yaml. States
flagged `perception_override: true` will prefer a live PoseStamped on
`~/target_pose/<state>/{left,right}` when available (see
perception_interface.PerceptionOverride) and fall back to the configured
joint-space target otherwise — this is the integration point for a future
camera-based grasp-pose estimator.

Usage
-----
    ros2 launch genie_sim_dual_arm_manip manipulation.launch.py

    # trigger one state transition
    ros2 service call /dual_arm_pick_place_task/go_pick std_srvs/srv/Trigger {}

    # run the full default->...->place->default sequence once
    ros2 service call /dual_arm_pick_place_task/run_cycle std_srvs/srv/Trigger {}
"""
from __future__ import annotations

import os
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

from genie_sim_dual_arm_manip.gripper_interface import GripperInterface
from genie_sim_dual_arm_manip.moveit_action_client import MoveGroupActionClient, MoveGroupError
from genie_sim_dual_arm_manip.perception_interface import PerceptionOverride
from genie_sim_dual_arm_manip.pose_config import PosePlan, load_pose_plan
from genie_sim_dual_arm_manip.task_states import ORDER, SEQUENCE, TaskState


class DualArmPickPlaceTask(Node):
    def __init__(self):
        super().__init__("dual_arm_pick_place_task")

        default_config = os.path.join(
            get_package_share_directory("genie_sim_dual_arm_manip"),
            "config",
            "pick_place_poses.yaml",
        )
        self.declare_parameter("pose_config", default_config)
        self.declare_parameter("move_group_timeout_sec", 30.0)
        self.declare_parameter("planning_scene_frame", "base_link")

        config_path = self.get_parameter("pose_config").get_parameter_value().string_value
        self._plan: PosePlan = load_pose_plan(config_path)
        self._timeout_sec = float(self.get_parameter("move_group_timeout_sec").value)

        # Serializes transitions (both the `_current_state` order check and
        # the execution itself) so two concurrently-triggered services can't
        # race each other into an inconsistent state.
        self._transition_lock = threading.Lock()
        # Assumed pose at node startup — matches the scene's `init_joint_pos`
        # (this package's `default` state should mirror it). There is no way
        # to ask MoveIt "what named state is the robot currently in", so this
        # is a bookkeeping assumption, not a read of true robot state.
        self._current_state = TaskState.DEFAULT

        # ReentrantCallbackGroup + MultiThreadedExecutor (see main()) lets a
        # Trigger service callback block waiting on a MoveGroup goal without
        # starving the executor thread that will deliver that goal's result.
        self._cb_group = ReentrantCallbackGroup()
        self._move_group = MoveGroupActionClient(self, self._plan.group_name, callback_group=self._cb_group)
        self._gripper = GripperInterface(self, self._plan.gripper)
        self._perception = PerceptionOverride(
            self,
            {state: target.perception_override for state, target in self._plan.states.items()},
        )

        self.get_logger().info(f"waiting for move_group action server (group '{self._plan.group_name}')...")
        self._move_group.wait_for_server(timeout_sec=30.0)
        self.get_logger().info("move_group action server ready")
        self._apply_collision_objects()

        self._services = []
        for state in TaskState:
            self._services.append(
                self.create_service(
                    Trigger,
                    f"~/go_{state.value}",
                    self._make_state_service(state),
                    callback_group=self._cb_group,
                )
            )
        self._services.append(
            self.create_service(Trigger, "~/run_cycle", self._run_cycle_cb, callback_group=self._cb_group)
        )

    def _apply_collision_objects(self) -> None:
        planning_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not planning_scene_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt planning-scene service '/apply_planning_scene' not available")

        object_ids = ["table_1", "table_2", "table_3"]
        dimensions = [[0.8, 1.1, 0.75], [0.8, 1.1, 0.75], [0.8, 1.1, 0.75]]
        positions = [[0.825, 0.0, 0.35], [0.15, -0.95, 0.35], [0.15, 0.95, 0.35]]
        orientations = [
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.7071068, 0.7071068],
            [0.0, 0.0, 0.7071068, 0.7071068],
        ]

        scene = PlanningScene()
        scene.is_diff = True
        for object_id, dim, pos, ori in zip(object_ids, dimensions, positions, orientations):
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(d) for d in dim]

            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.x = float(ori[0])
            pose.orientation.y = float(ori[1])
            pose.orientation.z = float(ori[2])
            pose.orientation.w = float(ori[3])

            collision_object = CollisionObject()
            collision_object.id = object_id
            collision_object.header.frame_id = str(self.get_parameter("planning_scene_frame").value)
            collision_object.operation = CollisionObject.ADD
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
            scene.world.collision_objects.append(collision_object)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError("failed to apply the table collision object to the planning scene")
        self.get_logger().info(f"applied table collision object(s) '{', '.join(object_ids)}'")

    def _make_state_service(self, state: TaskState):
        def _cb(request, response):
            try:
                self._execute_state(state)
                response.success = True
                response.message = f"reached '{state.value}'"
            except MoveGroupError as exc:
                response.success = False
                response.message = str(exc)
            return response

        return _cb

    def _run_cycle_cb(self, request, response):
        try:
            for state in SEQUENCE:
                self._execute_state(state)
            response.success = True
            response.message = "cycle complete"
        except MoveGroupError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _check_transition(self, state: TaskState) -> None:
        """Enforce ORDER: only the next state after `_current_state`, or DEFAULT from anywhere."""
        if state == TaskState.DEFAULT:
            return
        next_state = ORDER[(ORDER.index(self._current_state) + 1) % len(ORDER)]
        if state != next_state:
            raise MoveGroupError(
                f"invalid transition '{self._current_state.value}' -> '{state.value}'; "
                f"expected '{next_state.value}' (or 'default')"
            )

    def _execute_state(self, state: TaskState) -> None:
        with self._transition_lock:
            self._check_transition(state)
            target = self._plan.states[state]
            self.get_logger().info(f"-> {state.value} (group '{target.group_name}')")

            if target.perception_override and self._perception.has_any_override(state):
                link_poses = []
                # Only constrain an arm's end-link pose when that arm is part of
                # the configured planning group — otherwise move_group rejects it.
                if self._plan.has_subgroup(target, "arm_l"):
                    pose_l = self._perception.get_pose(state, "left")
                    if pose_l is not None:
                        link_poses.append(("arm_l_end_link", pose_l))
                if self._plan.has_subgroup(target, "arm_r"):
                    pose_r = self._perception.get_pose(state, "right")
                    if pose_r is not None:
                        link_poses.append(("arm_r_end_link", pose_r))
                self._move_group.move_to_poses(
                    link_poses, timeout_sec=self._timeout_sec, group_name=target.group_name
                )
            else:
                joint_names, positions = self._plan.goal_joint_target(target)
                self._move_group.move_to_joint_targets(
                    joint_names,
                    positions,
                    tolerance=self._plan.joint_tolerance,
                    timeout_sec=self._timeout_sec,
                    group_name=target.group_name,
                )

            self._gripper.apply("left", target.gripper_l)
            self._gripper.apply("right", target.gripper_r)
            self._current_state = state


def main():
    rclpy.init()
    node = DualArmPickPlaceTask()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
