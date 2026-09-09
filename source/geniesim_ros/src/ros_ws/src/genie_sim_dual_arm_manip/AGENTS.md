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

- Planning group is `simple_arms` (defined in
  `genie_sim_moveit/config/genie.srdf.xacro`) — both 7-DoF arms, no
  torso/chassis. If you need the mobile base or torso in the loop, add a
  new pose-plan config with `group_name: wbc_arms` (or similar) rather
  than hardcoding a second group name in code — `moveit_action_client.py`
  is already group-name-agnostic.
- Talks to `/move_action` (the default, unnamespaced `move_group` action
  name). If a scene ever launches `move_group` in a non-default namespace,
  update `MoveGroupActionClient.__init__`'s hardcoded action name.
- Joint names in `config/pick_place_poses.yaml` (`arm_l_joints`,
  `arm_r_joints`, `gripper.left_joint`, `gripper.right_joint`) are
  robot/gripper-combo specific (URDF→USD import prefixes differ from
  hand-authored USDs — same caveat as
  `genie_sim_bringup/scripts/gripper_cmds.py`). Don't assume the shipped
  defaults are correct for a robot_model/arm/gripper combo other than
  `g2`/`crsB`/`swiftpicker`.
- `pose_config.load_pose_plan` requires all 6 `TaskState` values to be
  present under `states:` — it raises `ValueError` listing what's missing
  rather than defaulting silently.

## Extending

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
