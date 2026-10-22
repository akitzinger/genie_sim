
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
  static const std::string PLANNING_GROUP = "sync_dual_arm_l"; 
  static const std::string PLANNING_GROUP_OPT = "sync_dual_arm_l_opt"; 

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

  // Second instance publishes the solution of the second group on its own RobotState topic.
  moveit_visual_tools::MoveItVisualTools visual_tools_opt(move_group_node, "odom", "move_group_tutorial_opt",
                                                          move_group.getRobotModel());
  visual_tools_opt.loadRobotStatePub("display_robot_state_opt");

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

  //
  // Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::msg::Pose start_pose1;
  start_pose1.orientation.x = q.x();
  start_pose1.orientation.y = q.y();
  start_pose1.orientation.z = q.z();
  start_pose1.orientation.w = q.w();
  start_pose1.position.x = t.x();
  start_pose1.position.y = t.y();
  start_pose1.position.z = t.z();

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();
  target_pose1.position.x = t.x()-0.2;
  target_pose1.position.y = t.y()+0.2;
  target_pose1.position.z = t.z()+0.1;


  // Solves IK for the target pose with the solver of the given group and shows the result with the given tools.
  auto solveAndShow = [&](const std::string& group_name, moveit_visual_tools::MoveItVisualTools& state_tools,
                          rvt::Colors color, double label_z) {
    const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(group_name);
    auto solver = jmg ? jmg->getSolverInstance() : nullptr;
    if (!solver)
    {
      RCLCPP_ERROR(LOGGER, "No IK solver for group '%s'", group_name.c_str());
      return;
    }

    // IK query in the base frame of the solver, for the solver's tip frame
    const std::string& tip_frame = solver->getTipFrame();
    const Eigen::Isometry3d eef_to_tip = start_state.getGlobalLinkTransform(END_EFFECTOR_LINK).inverse() *
                                         start_state.getGlobalLinkTransform(tip_frame);
    RCLCPP_INFO(LOGGER, "[%s] solver base frame '%s', tip frame '%s'", group_name.c_str(),
                solver->getBaseFrame().c_str(), tip_frame.c_str());
    Eigen::Isometry3d target_in_planning;
    tf2::fromMsg(toPlanningFrame(target_pose1), target_in_planning);
    const geometry_msgs::msg::Pose ik_query = tf2::toMsg(
        start_state.getFrameTransform(solver->getBaseFrame()).inverse() * target_in_planning * eef_to_tip);

    std::vector<double> group_values;
    start_state.copyJointGroupPositions(jmg, group_values);
    const std::vector<size_t>& bijection = jmg->getKinematicsSolverJointBijection();
    std::vector<double> seed(bijection.size());
    for (std::size_t i = 0; i < bijection.size(); ++i)
      seed[i] = group_values[bijection[i]];

    // The coupled-arm plugin needs the start state as context_state (setFromIK() does not pass it).
    std::vector<double> ik_solution;
    moveit_msgs::msg::MoveItErrorCodes error;
    const rclcpp::Time t0 = move_group_node->now();
    const bool found = solver->searchPositionIK(
        std::vector<geometry_msgs::msg::Pose>{ ik_query }, seed, jmg->getDefaultIKTimeout(), std::vector<double>(),
        ik_solution, kinematics::KinematicsBase::IKCallbackFn(), error, kinematics::KinematicsQueryOptions(),
        &start_state);
    const double solve_time = (move_group_node->now() - t0).seconds();

    if (!found)
    {
      RCLCPP_WARN(LOGGER, "[%s] no IK solution found (%.3f s, error code %d)", group_name.c_str(), solve_time,
                  error.val);

      // Sanity check: the unchanged start pose must be solvable if the setup is correct.
      Eigen::Isometry3d start_in_planning;
      tf2::fromMsg(toPlanningFrame(start_pose1), start_in_planning);
      const geometry_msgs::msg::Pose start_query = tf2::toMsg(
          start_state.getFrameTransform(solver->getBaseFrame()).inverse() * start_in_planning * eef_to_tip);
      std::vector<double> sanity_solution;
      const bool start_ok = solver->searchPositionIK(
          std::vector<geometry_msgs::msg::Pose>{ start_query }, seed, jmg->getDefaultIKTimeout(),
          std::vector<double>(), sanity_solution, kinematics::KinematicsBase::IKCallbackFn(), error,
          kinematics::KinematicsQueryOptions(), &start_state);
      RCLCPP_WARN(LOGGER, "[%s] IK for the unchanged start pose: %s (solver base frame '%s')", group_name.c_str(),
                  start_ok ? "solvable, target is probably out of reach" : "FAILED, setup is wrong",
                  solver->getBaseFrame().c_str());
      return;
    }

    moveit::core::RobotState solution_state(start_state);
    std::vector<double> solution(bijection.size());
    for (std::size_t i = 0; i < bijection.size(); ++i)
      solution[bijection[i]] = ik_solution[i];
    solution_state.setJointGroupPositions(jmg, solution);
    solution_state.update();

    const Eigen::Isometry3d reached = solution_state.getGlobalLinkTransform(END_EFFECTOR_LINK);
    RCLCPP_INFO(LOGGER, "[%s] solved in %.3f s, position error %.4f m", group_name.c_str(), solve_time,
                (reached.translation() - target_in_planning.translation()).norm());

    Eigen::Isometry3d label_pose = text_pose;
    label_pose.translation().z() = label_z;
    visual_tools.publishText(label_pose, "IK_" + group_name, color, rvt::XLARGE);
    state_tools.publishRobotState(solution_state, color);
    visual_tools.trigger();
  };

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.publishAxisLabeled(toPlanningFrame(start_pose1), "start");
  visual_tools.publishAxisLabeled(toPlanningFrame(target_pose1), "goal");
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  solveAndShow(PLANNING_GROUP, visual_tools, rvt::GREEN, 1.2);


  // Compare with second planning group
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  solveAndShow(PLANNING_GROUP_OPT, visual_tools_opt, rvt::RED, 1.4);

  visual_tools.prompt("Showing both IK solutions (green: " + PLANNING_GROUP + ", red: " + PLANNING_GROUP_OPT +
                      "). Press 'next' to finish");

  // END_TUTORIAL0
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.hideRobot();
  visual_tools_opt.hideRobot();

  rclcpp::shutdown();
  return 0;
}
