/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/kinematic_constraints/utils.hpp>

#include <sstream>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "simple_dual_arm_l"; 

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.hpp>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  RCLCPP_INFO(
    LOGGER,
    "MoveGroupInterface created for planning group '%s'",
    PLANNING_GROUP.c_str());

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.hpp>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "odom", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link emplace_backfor this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  const auto group_names = move_group.getJointModelGroupNames();
  for (const auto& group : group_names)
  {
    RCLCPP_INFO(LOGGER, "  - %s", group.c_str());
  }

  // Planning Frame
  const std::string PLANNING_FRAME = move_group.getPlanningFrame();

  // Endeffector link
  const std::string END_EFFECTOR_LINK = "arm_l_end_link";

  // Poses given to setPoseTarget()/computeCartesianPath() (and returned by
  // getCurrentPose()) are interpreted relative to this frame instead of the
  // default planning frame ("odom" here, from the SRDF virtual_joint).
  const std::string POSE_REFERENCE_FRAME = "arm_base_link";
  move_group.setPoseReferenceFrame(POSE_REFERENCE_FRAME);

  // visual_tools draws in "odom" frame, so poses
  // given in the POSE_REFERENCE_FRAME must be converted before
  // being passed to publishAxisLabeled()/publishCuboid() or they'll be
  // drawn as if POSE_REFERENCE_FRAME == odom.
  auto toPlanningFrame = [&](const geometry_msgs::msg::Pose& pose_in_ref) {
    Eigen::Isometry3d pose_eigen;
    tf2::fromMsg(pose_in_ref, pose_eigen);
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    const Eigen::Isometry3d ref_frame_tf =
        current_state->getGlobalLinkTransform(move_group.getPoseReferenceFrame());
    return tf2::toMsg(ref_frame_tf * pose_eigen);
  };

  // Get current state
  moveit::core::RobotState start_state(*move_group.getCurrentState());

  // Defined start pose from current state
  const Eigen::Isometry3d& KI_T_I1 = start_state.getGlobalLinkTransform(END_EFFECTOR_LINK);
  const Eigen::Isometry3d& KI_T_I0 = start_state.getGlobalLinkTransform(POSE_REFERENCE_FRAME);
  const auto R_I0 = KI_T_I0.rotation();
  const auto R_01 = KI_T_I0.rotation().inverse() * KI_T_I1.rotation();

  // pose in reference frame 
  const Eigen::Vector3d t = R_I0.inverse() * (KI_T_I1.translation() - KI_T_I0.translation());
  const Eigen::Quaterniond q(R_01);

  {
    std::ostringstream start_state_positions;
    start_state.printStatePositions(start_state_positions);
    RCLCPP_INFO(LOGGER, "Start state joint positions: %s", start_state_positions.str().c_str());

    RCLCPP_INFO(LOGGER, "Start_state arm_l_end_link pose: xyz=[%.3f, %.3f, %.3f] xyzw=[%.3f, %.3f, %.3f, %.3f]",
                t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());

  }

  // Set start pose
  geometry_msgs::msg::Pose start_pose1;
  start_pose1.orientation.x = q.x();
  start_pose1.orientation.y = q.y();
  start_pose1.orientation.z = q.z();
  start_pose1.orientation.w = q.w();
  start_pose1.position.x = t.x();
  start_pose1.position.y = t.y();
  start_pose1.position.z = t.z();

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();
  target_pose1.position.x = t.x()-0.2;
  target_pose1.position.y = t.y()+0.2;
  target_pose1.position.z = t.z()+0.1;

  // move_group.setStartState(start_state);
  // move_group.setPoseTarget(target_pose1, END_EFFECTOR_LINK);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(toPlanningFrame(start_pose1), "start");
  visual_tools.publishAxisLabeled(toPlanningFrame(target_pose1), "goal");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the ``move()`` function. Note that1
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
  /* move_group.move(); */


  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = END_EFFECTOR_LINK;
  ocm.header.frame_id = POSE_REFERENCE_FRAME;
  ocm.orientation.x = start_pose1.orientation.x;
  ocm.orientation.y = start_pose1.orientation.y;
  ocm.orientation.z = start_pose1.orientation.z;
  ocm.orientation.w = start_pose1.orientation.w;
  ocm.absolute_x_axis_tolerance = 0.6;
  ocm.absolute_y_axis_tolerance = 0.6;
  ocm.absolute_z_axis_tolerance = 0.6;
  ocm.weight = 1.0;

  // Position Box Constraints
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.link_name = END_EFFECTOR_LINK;
  box_constraint.header.frame_id = POSE_REFERENCE_FRAME;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.8, 1.0, 1.0 };
  box_constraint.constraint_region.primitives.push_back(box);

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.55;
  box_pose.position.y = 0;
  box_pose.position.z = 0.2;
  box_pose.orientation.x = 0.0;
  box_pose.orientation.y = 0.0;
  box_pose.orientation.z = 0.0;
  box_pose.orientation.w = 1.0;
  box_constraint.constraint_region.primitive_poses.push_back(box_pose);
  box_constraint.weight = 1.0;

  // Visualize Box in Rviz
  visual_tools.publishCuboid(toPlanningFrame(box_pose), box.dimensions[0], box.dimensions[1], box.dimensions[2]);
  visual_tools.trigger();

  // Now, set it as the path constraint for the group.
  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.orientation_constraints.push_back(ocm);
  path_constraints.position_constraints.push_back(box_constraint);
  move_group.setPathConstraints(path_constraints);

  // // pose constraints
  // geometry_msgs::msg::PoseStamped target_pose_stamped;
  // target_pose_stamped.header.frame_id = POSE_REFERENCE_FRAME;
  // target_pose_stamped.pose.position.x = start_pose1.position.x;
  // target_pose_stamped.pose.position.y = start_pose1.position.y;
  // target_pose_stamped.pose.position.z = start_pose1.position.z;
  // target_pose_stamped.pose.orientation.x = start_pose1.orientation.x;
  // target_pose_stamped.pose.orientation.y = start_pose1.orientation.y;
  // target_pose_stamped.pose.orientation.z = start_pose1.orientation.z;
  // target_pose_stamped.pose.orientation.w = start_pose1.orientation.w;

  // moveit_msgs::msg::Constraints pose_constraints =
  //   kinematic_constraints::constructGoalConstraints(
  //       END_EFFECTOR_LINK,           // link name
  //       target_pose_stamped,  // geometry_msgs::PoseStamped
  //       1.0,              // position tolerance (m)
  //       0.1             // orientation tolerance (rad)
  //   );
  // move_group.setPathConstraints(pose_constraints);

  // Workspace Bounds
  moveit_msgs::msg::WorkspaceParameters wp;
  wp.header.frame_id = POSE_REFERENCE_FRAME;
  wp.min_corner.x = -2.0;
  wp.min_corner.y = -2.0;
  wp.min_corner.z = -2.0;
  wp.max_corner.x = 2.0;
  wp.max_corner.y = 2.0;
  wp.max_corner.z = 2.0;
  move_group.setWorkspace(wp.min_corner.x, wp.min_corner.y, wp.min_corner.z,
                         wp.max_corner.x, wp.max_corner.y, wp.max_corner.z);

  // Enforce Planning in Joint Space
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Depending on the planning problem MoveIt chooses between
  // ``joint space`` and ``cartesian space`` for problem representation.
  // Setting the group parameter ``enforce_joint_model_state_space:true`` in
  // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
  //
  // By default, planning requests with orientation path constraints
  // are sampled in ``cartesian space`` so that invoking IK serves as a
  // generative sampler.
  //
  // By enforcing ``joint space``, the planning process will use rejection
  // sampling to find valid requests. Please note that this might
  // increase planning time considerably.
  //
  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So we need to set the start
  // state to a new pose.

  // Now, we will plan to the earlier pose target from the new
  // start state that we just created.
  move_group.setStartState(start_state);
  move_group.setPoseTarget(target_pose1, END_EFFECTOR_LINK);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(300.0);

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan with constraints %s", success ? "" : "FAILED");
  // move_group.move();

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(toPlanningFrame(start_pose1), "start");
  visual_tools.publishAxisLabeled(toPlanningFrame(target_pose1), "goal");
  visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // When done with the path constraint, be sure to clear it.
  move_group.clearPathConstraints();

  // END_TUTORIAL0
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
