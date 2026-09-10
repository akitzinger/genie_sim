# genie_sim_dual_arm_manip — AGENTS.md

Routing + mechanism notes for agents. See [README.md](README.md) for the
human-facing overview.

## What this package is (and isn't)

- **Is**: a task-orchestration client of the already-running `move_group`
  action server. One `rclpy` node, one YAML pose config, one MoveGroup
  action wrapper, one `/joint_command` gripper wrapper.
- **Isn't**: a MoveIt config package (that's `genie_sim_moveit`), a planner
  or IK plugin (that's `genie_sim_moveit_plugins`), or a perception package
  (no camera/grasp-pose estimation is implemented here — see
  `perception_interface.py` for the stub/extension point).

## Invariants

- Planning group is configurable **per state** via `group_name` in
  `config/pick_place_poses.yaml` (falls back to the top-level `group_name`
  default when a state omits it; groups are defined in
  `genie_sim_moveit/config/genie.srdf.xacro`) — defaults to
  `wbc_fixed_headless` (torso + both arms, no head/chassis). Every state
  configures a full whole-body pose (`body`, `head`, `arm_l`, `arm_r`);
  `body`'s first 3 values are the waist (idx01-03_body_joint) and the last
  2 are the torso (idx04-05_body_joint), split so atomic SRDF groups
  (`simple_waist`, `simple_torso`, `simple_body`) are valid `group_name`
  choices too, not just the composite `wbc_*` groups. `pose_config.
  _GROUP_SUBGROUPS` maps each state's `group_name` to the sub-groups
  (`waist`/`torso`/`head`/`arm_l`/`arm_r`) it spans; `PosePlan.
  goal_joint_target` filters the goal constraints to just those.
  `moveit_action_client.py` remains group-name-agnostic (`group_name` is
  now a per-call override, not fixed at construction) — add a new entry to
  `_GROUP_SUBGROUPS` rather than hardcoding a second group name in code.
- Transitions are only legal in `task_states.ORDER` (default → pick_ready →
  pick → pick_hold → place_hold → place_ready → place), one step at a time, tracked via
  `DualArmPickPlaceTask._current_state` / `_check_transition`.
  `go_default` is always legal regardless of the current state; any other
  out-of-order `go_*` call raises `MoveGroupError` (surfaced as
  `success=false` on the Trigger service) before any MoveGroup goal is
  sent. `_current_state` is a bookkeeping assumption seeded to `DEFAULT` at
  node startup — there is no way to query MoveIt for "which named state is
  the robot currently in", so keep the scene's `init_joint_pos` in sync
  with the `default` state's pose.
- Joint names in `config/pick_place_poses.yaml` (`body_joints`,
  `head_joints`, `arm_l_joints`, `arm_r_joints`, `gripper.left_joint`,
  `gripper.right_joint`) are robot/gripper-combo specific (URDF→USD import
  prefixes differ from hand-authored USDs — same caveat as
  `genie_sim_bringup/scripts/gripper_cmds.py`). Don't assume the shipped
  defaults are correct for a robot_model/arm/gripper combo other than
  `g2`/`crsB`/`swiftpicker`.
- `pose_config.load_pose_plan` requires all 6 `TaskState` values to be
  present under `states:` — it raises `ValueError` listing what's missing
  rather than defaulting silently.

## Extending

- **A new selectable `group_name`**: add its body/head/arm_l/arm_r span to
  `pose_config._GROUP_SUBGROUPS` (mirroring the composite group definition
  in `genie_sim_moveit/config/genie.srdf.xacro`) — no other code change
  needed; `load_pose_plan` validates `group_name` against this map.
- **More states**: add to `task_states.TaskState` and `SEQUENCE`, then add
  a matching block under `states:` in every pose config in use. The
  Trigger-service loop in `dual_arm_pick_place_task.py` iterates
  `TaskState` automatically — no service wiring needed per new state.
- **Cartesian (not joint-space) targets for static states**: currently only
  `perception_override` states use `move_to_poses` (Cartesian). To make a
  static state Cartesian too, extend `StateTarget` with an optional pose
  field and branch in `_execute_state` the same way.
- **Real perception**: implement a node that publishes
  `geometry_msgs/PoseStamped` on `~/target_pose/<state>/{left,right}`
  (relative to the `dual_arm_pick_place_task` node's namespace) — no
  change needed in this package.
- **RViz panel**: build it in `genie_sim_rviz_plugins` calling the
  `std_srvs/Trigger` services this node already exposes; don't duplicate
  the state machine there.

## Gotchas

- `MoveGroupActionClient._send` blocks the calling thread on a
  `threading.Event`, not `rclpy.spin_until_future_complete`. This is
  intentional — nesting a second executor inside an already-spinning
  node's callback is unsafe. The `ReentrantCallbackGroup` +
  `MultiThreadedExecutor(num_threads=4)` in `main()` is what lets the
  action-client's own response/result callbacks run on a different thread
  while a Trigger service callback blocks. If you ever swap to a
  `SingleThreadedExecutor`, every Trigger call will deadlock.
- `move_to_poses` builds constraints for whichever links have a
  latched perception pose (left, right, or both) — it does **not**
  require both arms to have an override to proceed. Callers still fall
  back entirely to joint-space targets if neither side has one yet.
