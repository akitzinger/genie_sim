# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""Loader for config/pick_place_poses.yaml — the per-state joint targets."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import yaml

from genie_sim_dual_arm_manip.task_states import TaskState


@dataclass
class GripperConfig:
    left_joint: str
    right_joint: str
    open_position: float
    close_position: float


@dataclass
class StateTarget:
    arm_l: List[float]
    arm_r: List[float]
    gripper_l: str  # "open" | "close" | "hold" (no change)
    gripper_r: str
    # If true, a live PoseStamped on ~/target_pose/<state>/{left,right} (see
    # perception_interface.PerceptionOverride) is used instead of arm_l/arm_r
    # when available; falls back to the joint-space target otherwise.
    perception_override: bool = False


@dataclass
class PosePlan:
    group_name: str
    joint_tolerance: float
    arm_l_joints: List[str]
    arm_r_joints: List[str]
    gripper: GripperConfig
    states: Dict[TaskState, StateTarget]


def load_pose_plan(path: str) -> PosePlan:
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    gripper_raw = raw["gripper"]
    gripper = GripperConfig(
        left_joint=gripper_raw["left_joint"],
        right_joint=gripper_raw["right_joint"],
        open_position=float(gripper_raw["open_position"]),
        close_position=float(gripper_raw["close_position"]),
    )

    states: Dict[TaskState, StateTarget] = {}
    for name, cfg in raw["states"].items():
        state = TaskState(name)
        states[state] = StateTarget(
            arm_l=[float(v) for v in cfg["arm_l"]],
            arm_r=[float(v) for v in cfg["arm_r"]],
            gripper_l=cfg.get("gripper_l", "hold"),
            gripper_r=cfg.get("gripper_r", "hold"),
            perception_override=bool(cfg.get("perception_override", False)),
        )

    missing = [s.value for s in TaskState if s not in states]
    if missing:
        raise ValueError(f"{path}: missing state(s) {missing} under 'states:'")

    return PosePlan(
        group_name=raw.get("group_name", "simple_arms"),
        joint_tolerance=float(raw.get("joint_tolerance", 0.01)),
        arm_l_joints=list(raw["arm_l_joints"]),
        arm_r_joints=list(raw["arm_r_joints"]),
        gripper=gripper,
        states=states,
    )
