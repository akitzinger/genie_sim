# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""Loader for config/pick_place_poses.yaml — the per-state joint targets.

Every state configures a full whole-body pose (body + head + both arms),
but only the sub-groups spanned by that state's `group_name` are actually
sent as MoveGroup goal constraints — see `_GROUP_SUBGROUPS` and
`PosePlan.goal_joint_target`. The `body` pose is split into `waist`
(idx01-03_body_joint) and `torso` (idx04-05_body_joint) sub-groups so that
atomic SRDF groups (`simple_waist`, `simple_torso`, `simple_body`) are
valid `group_name` choices, not just the composite `wbc_*` groups. Each
state may set its own `group_name` (falling back to the top-level
default), so e.g. an arm-only transition can plan with `simple_arms` while
a transition that only repositions the torso uses `simple_torso` —
without editing every state's joint values.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import yaml

from genie_sim_dual_arm_manip.task_states import TaskState

# Sub-group -> planning groups that span it, mirroring the composite group
# definitions in genie_sim_moveit/config/genie.srdf.xacro. "waist" ==
# idx01-03_body_joint (simple_waist), "torso" == idx04-05_body_joint
# (simple_torso) -- kept separate because simple_torso/simple_waist are
# themselves valid selectable groups. Extend this if you add a new
# selectable `group_name` or a new sub-group (e.g. chassis).
_GROUP_SUBGROUPS: Dict[str, Tuple[str, ...]] = {
    "simple_arms": ("arm_l", "arm_r"),
    "simple_dual_arm_l": ("arm_l", "arm_r"),
    "simple_dual_arm_r": ("arm_l", "arm_r"),
    "wbc_fixed_arms": ("arm_l", "arm_r"),
    "wbc_arms": ("arm_l", "arm_r"),
    "simple_waist": ("waist",),
    "simple_torso": ("torso",),
    "simple_body": ("waist", "torso"),
    "wbc_arm_l": ("waist", "torso", "arm_l"),
    "wbc_arm_r": ("waist", "torso", "arm_r"),
    "wbc_fixed_arm_l": ("waist", "torso", "arm_l"),
    "wbc_fixed_arm_r": ("waist", "torso", "arm_r"),
    "wbc_headless": ("waist", "torso", "arm_l", "arm_r"),
    "wbc_fixed_headless": ("waist", "torso", "arm_l", "arm_r"),
    "wbc": ("waist", "torso", "arm_l", "arm_r", "head"),
    "wbc_fixed": ("waist", "torso", "arm_l", "arm_r", "head"),
    "mobile_base_manipulator": ("waist", "torso", "arm_l", "arm_r"),
}


@dataclass
class GripperConfig:
    left_joint: str
    right_joint: str
    open_position: float
    close_position: float


@dataclass
class StateTarget:
    group_name: str
    body: List[float]
    head: List[float]
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
    body_joints: List[str]
    head_joints: List[str]
    arm_l_joints: List[str]
    arm_r_joints: List[str]
    gripper: GripperConfig
    states: Dict[TaskState, StateTarget]

    @property
    def waist_joints(self) -> List[str]:
        """idx01-03_body_joint -- the first 3 entries of `body_joints` (simple_waist)."""
        return self.body_joints[:3]

    @property
    def torso_joints(self) -> List[str]:
        """idx04-05_body_joint -- the last 2 entries of `body_joints` (simple_torso)."""
        return self.body_joints[3:]

    def has_subgroup(self, target: StateTarget, name: str) -> bool:
        """Whether `target.group_name` spans the sub-group ("waist"|"torso"|"head"|"arm_l"|"arm_r")."""
        return name in _GROUP_SUBGROUPS[target.group_name]

    def goal_joint_target(self, target: StateTarget) -> Tuple[List[str], List[float]]:
        """Joint names/positions for `target`, filtered down to `target.group_name`'s sub-groups."""
        names: List[str] = []
        positions: List[float] = []
        for sub_name, joints, values in (
            ("waist", self.waist_joints, target.body[:3]),
            ("torso", self.torso_joints, target.body[3:]),
            ("head", self.head_joints, target.head),
            ("arm_l", self.arm_l_joints, target.arm_l),
            ("arm_r", self.arm_r_joints, target.arm_r),
        ):
            if self.has_subgroup(target, sub_name):
                names += joints
                positions += values
        return names, positions


def _deg_list_to_rad(values: List[float]) -> List[float]:
    return [float(v) / 180.0 * math.pi for v in values]


def load_pose_plan(path: str) -> PosePlan:
    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    group_name = raw.get("group_name", "wbc_fixed_headless")
    if group_name not in _GROUP_SUBGROUPS:
        raise ValueError(
            f"{path}: unsupported group_name '{group_name}'; supported: {sorted(_GROUP_SUBGROUPS)}"
        )

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
        state_group_name = cfg.get("group_name", group_name)
        if state_group_name not in _GROUP_SUBGROUPS:
            raise ValueError(
                f"{path}: state '{name}' has unsupported group_name '{state_group_name}'; "
                f"supported: {sorted(_GROUP_SUBGROUPS)}"
            )
        states[state] = StateTarget(
            group_name=state_group_name,
            body=_deg_list_to_rad(cfg["body"]),
            head=_deg_list_to_rad(cfg["head"]),
            arm_l=_deg_list_to_rad(cfg["arm_l"]),
            arm_r=_deg_list_to_rad(cfg["arm_r"]),
            gripper_l=cfg.get("gripper_l", "hold"),
            gripper_r=cfg.get("gripper_r", "hold"),
            perception_override=bool(cfg.get("perception_override", False)),
        )

    missing = [s.value for s in TaskState if s not in states]
    if missing:
        raise ValueError(f"{path}: missing state(s) {missing} under 'states:'")

    return PosePlan(
        group_name=group_name,
        joint_tolerance=float(raw.get("joint_tolerance", 0.01)),
        body_joints=list(raw["body_joints"]),
        head_joints=list(raw["head_joints"]),
        arm_l_joints=list(raw["arm_l_joints"]),
        arm_r_joints=list(raw["arm_r_joints"]),
        gripper=gripper,
        states=states,
    )
