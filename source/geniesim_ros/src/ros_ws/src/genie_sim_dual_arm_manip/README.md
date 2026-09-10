# genie_sim_dual_arm_manip

High-level task orchestration for dual-arm pick-and-place on Genie G2 —
the layer above MoveIt that turns "plan/execute a trajectory" into
"go pick up the thing". Drives the `simple_arms` planning group (see
[genie_sim_moveit](../genie_sim_moveit)) through a small named-pose state
machine and toggles the parallel-jaw grippers over `/joint_command`.

Source: [source/geniesim_ros/src/ros_ws/src/genie_sim_dual_arm_manip/](.)
License: [MIT](LICENSE)

---

## States

```
default → pick_ready → pick → hold → place_ready → place → default
```

Each state has a target joint pose per arm (`config/pick_place_poses.yaml`)
plus an open/close/hold command for each gripper. `pick` and `place` can
optionally be driven by a live camera-estimated pose instead of the
configured joint target — see "Perception hook" below.

## Prerequisites

`move_group` must already be running:

```bash
ros2 launch genie_sim_moveit wbc.launch.py
```

This package is a pure **client** of that action server (`/move_action`);
it does not start its own planning-scene monitor or duplicate the MoveIt
config.

## Run

```bash
ros2 launch genie_sim_dual_arm_manip manipulation.launch.py

# trigger one state transition
ros2 service call /dual_arm_pick_place_task/go_pick_ready std_srvs/srv/Trigger {}
ros2 service call /dual_arm_pick_place_task/go_pick std_srvs/srv/Trigger {}
ros2 service call /dual_arm_pick_place_task/go_hold std_srvs/srv/Trigger {}
ros2 service call /dual_arm_pick_place_task/go_place_ready std_srvs/srv/Trigger {}
ros2 service call /dual_arm_pick_place_task/go_place std_srvs/srv/Trigger {}
ros2 service call /dual_arm_pick_place_task/go_default std_srvs/srv/Trigger {}

# or run the whole cycle in one call
ros2 service call /dual_arm_pick_place_task/run_cycle std_srvs/srv/Trigger {}
```

RViz2 integration today: use the built-in **Service Caller** panel
(Panels → Add New Panel) against the services above — no custom plugin
needed to drive the state machine manually. A native RViz panel with one
button per state can be added later under
[genie_sim_rviz_plugins](../genie_sim_rviz_plugins) once the state set is
stable; it would just call the same Trigger services.

## Configuring poses

Edit [`config/pick_place_poses.yaml`](config/pick_place_poses.yaml). The
shipped values are safe placeholders — capture real joint angles per state
by driving the arm (RViz drag handles or teleop) into position and reading
`ros2 topic echo /joint_states --once`.

## Perception hook

States with `perception_override: true` (`pick`, `place` by default)
prefer a live `geometry_msgs/PoseStamped` on
`~/target_pose/<state>/{left,right}` over the configured joint target, via
[`perception_interface.PerceptionOverride`](genie_sim_dual_arm_manip/perception_interface.py).
No perception is implemented in this package — wire a grasp-pose estimator
(e.g. consuming `/gripper_l_camera_rgb/image_raw`,
`/gripper_r_camera_rgb/image_raw`) to publish on those topics and the state
machine will pick it up automatically; until then it falls back to the
YAML joint target.

## Layout

```
genie_sim_dual_arm_manip/
├── genie_sim_dual_arm_manip/        ← importable Python package
│   ├── task_states.py               ← the 6 states + demo sequence order
│   ├── pose_config.py               ← pick_place_poses.yaml loader
│   ├── moveit_action_client.py      ← moveit_msgs/MoveGroup action wrapper
│   ├── gripper_interface.py         ← /joint_command wrapper
│   └── perception_interface.py      ← camera target-pose stub (see above)
├── scripts/
│   └── dual_arm_pick_place_task.py  ← the task node (Trigger services)
├── config/
│   └── pick_place_poses.yaml
└── launch/
    └── manipulation.launch.py
```
