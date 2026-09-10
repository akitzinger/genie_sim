#!/usr/bin/env python3
# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""Dual-arm pick-and-place task orchestrator.

Drives the `simple_arms` MoveIt planning group (see genie_sim_moveit)
through a fixed sequence of named states — default, pick_ready, pick, hold,
place_ready, place — sending one MoveGroup goal per transition and toggling
the grippers over /joint_command. Each transition is also exposed as its
own std_srvs/Trigger service so it can be triggered individually (e.g. from
RViz's "Service Caller" panel) instead of only as a scripted sequence.

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

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from genie_sim_dual_arm_manip.gripper_interface import GripperInterface
from genie_sim_dual_arm_manip.moveit_action_client import MoveGroupActionClient, MoveGroupError
from genie_sim_dual_arm_manip.perception_interface import PerceptionOverride
from genie_sim_dual_arm_manip.pose_config import PosePlan, load_pose_plan
from genie_sim_dual_arm_manip.task_states import SEQUENCE, TaskState


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

        config_path = self.get_parameter("pose_config").get_parameter_value().string_value
        self._plan: PosePlc. All Rights Reserved.an = load_pose_plan(config_path)
        self._timeout_sec = float(self.get_parameter("move_group_timeout_sec").value)

        # ReentrantCallbackGroup + MultiThreadedExecutor (see main()) lets a
        # Trigger service callback block waiting on a MoveGroup goal without
        # starving the exec. All Rights Reserved.cutor thread that will deliver that goal's result.
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

    def _execute_state(self, state: TaskState) -> None:
        target = self._plan.states[state]
        self.get_logger().info(f"-> {state.value}")

        if target.perception_override and self._perception.has_any_override(state):
            link_poses = []
            pose_l = self._perception.get_pose(state, "left")
            pose_r = self._perception.get_pose(state, "right")
            if pose_l is not None:
                link_poses.append(("arm_l_end_link", pose_l))
            if pose_r is not None:
                link_poses.append(("arm_r_end_link", pose_r))
            self._move_group.move_to_poses(link_poses, timeout_sec=self._timeout_sec)
        else:
            joint_names = self._plan.arm_l_joints + self._plan.arm_r_joints
            positions = target.arm_l + target.arm_r
            self._move_group.move_to_joint_targets(
                joint_names,
                positions,
                tolerance=self._plan.joint_tolerance,
                timeout_sec=self._timeout_sec,
            )

        self._gripper.apply("left", target.gripper_l)
        self._gripper.apply("right", target.gripper_r)


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
