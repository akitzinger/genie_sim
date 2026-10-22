#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <random_numbers/random_numbers.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik_solver_comparison");

namespace
{
// Result of one IK call (plus quality metrics of the solution).
struct TrialResult
{
  bool ok = false;
  int error_code = 0;
  double time = 0.0;        // wall-clock solve time [s]
  double pos_err = NAN;     // max position error over both tips [m]
  double rot_err = NAN;     // max orientation error over both tips [rad]
  double w_master = NAN;    // position manipulability sqrt(det(J J^T)) of master arm
  double w_slave = NAN;     // same for slave arm
  double joint_dist = NAN;  // joint-space distance of solution to the current state [rad]
  std::shared_ptr<moveit::core::RobotState> state;
};

double mean(const std::vector<double>& v)
{
  return v.empty() ? NAN : std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("ik_solver_comparison", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // ---------------------------------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------------------------------
  static const std::string PLANNING_GROUP = "sync_dual_arm_l";
  static const std::string PLANNING_GROUP_OPT = "sync_dual_arm_l_opt";

  // Master/slave links and the single-arm groups of the two arms (used for manipulability).
  static const std::string MASTER_LINK = "arm_l_end_link";
  static const std::string SLAVE_LINK = "arm_r_end_link";
  static const std::string MASTER_ARM_GROUP = "simple_arm_l";
  static const std::string SLAVE_ARM_GROUP = "simple_arm_r";

  constexpr int NUM_TRIALS = 20;              // number of random seeds per solver
  constexpr unsigned int RNG_SEED = 42;       // fixed -> identical seeds for both solvers and across runs
  constexpr double MIN_SEED_DISTANCE = 1.0;   // min. joint-space distance of a seed to the current state [rad]

  // Offset of the IK target w.r.t. the current master pose, in the planning frame ("odom").
  // Zero -> target is exactly the current pose, which is known to be reachable.
  const Eigen::Vector3d TARGET_OFFSET = Eigen::Vector3d::Zero();

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getRobotModel()->getJointModelGroup(PLANNING_GROUP);
  if (!joint_model_group)
  {
    RCLCPP_ERROR(LOGGER, "Unknown planning group '%s'", PLANNING_GROUP.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // ---------------------------------------------------------------------------------------------
  // Visualization
  // ---------------------------------------------------------------------------------------------
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "odom", "move_group_tutorial",
                                                      move_group.getRobotModel());
  moveit_visual_tools::MoveItVisualTools visual_tools_opt(move_group_node, "odom", "move_group_tutorial_opt",
                                                          move_group.getRobotModel());
  visual_tools_opt.loadRobotStatePub("display_robot_state_opt");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "IK_comparison", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // ---------------------------------------------------------------------------------------------
  // ONE state snapshot. Everything below (context state, target, relative pose, seeds) is derived
  // from this snapshot, so it cannot become inconsistent if the joint states change meanwhile.
  // ---------------------------------------------------------------------------------------------
  moveit::core::RobotStatePtr current = move_group.getCurrentState(5.0);
  if (!current)
  {
    RCLCPP_ERROR(LOGGER, "Could not get the current robot state (no /joint_states?)");
    rclcpp::shutdown();
    return 1;
  }
  moveit::core::RobotState start_state(*current);
  start_state.update();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  {
    std::ostringstream ss;
    start_state.printStatePositions(ss);
    RCLCPP_INFO(LOGGER, "Start state joint positions: %s", ss.str().c_str());
  }

  // Target: current master pose (+ optional offset); slave target keeps the current relative pose.
  Eigen::Isometry3d target_master = start_state.getGlobalLinkTransform(MASTER_LINK);
  target_master.translation() += TARGET_OFFSET;
  const Eigen::Isometry3d relative_pose =
      start_state.getGlobalLinkTransform(MASTER_LINK).inverse() * start_state.getGlobalLinkTransform(SLAVE_LINK);
  const Eigen::Isometry3d target_slave = target_master * relative_pose;

  // ---------------------------------------------------------------------------------------------
  // Seeds: random configurations that are far away from the current state, identical for both solvers
  // ---------------------------------------------------------------------------------------------
  random_numbers::RandomNumberGenerator rng(RNG_SEED);
  std::vector<moveit::core::RobotState> seeds;
  for (int attempts = 0; static_cast<int>(seeds.size()) < NUM_TRIALS && attempts < 1000 * NUM_TRIALS; ++attempts)
  {
    moveit::core::RobotState s(start_state);
    s.setToRandomPositions(joint_model_group, rng);
    s.update();
    if (s.distance(start_state, joint_model_group) >= MIN_SEED_DISTANCE)
      seeds.push_back(s);
  }
  RCLCPP_INFO(LOGGER, "Generated %zu random seeds (min. distance to current state: %.2f rad)", seeds.size(),
              MIN_SEED_DISTANCE);

  // ---------------------------------------------------------------------------------------------
  // Single IK call + evaluation
  // ---------------------------------------------------------------------------------------------
  auto manipulability = [](const moveit::core::RobotState& s, const std::string& arm_group,
                           const std::string& link) -> double {
    const moveit::core::JointModelGroup* g = s.getRobotModel()->getJointModelGroup(arm_group);
    const moveit::core::LinkModel* l = s.getRobotModel()->getLinkModel(link);
    if (!g || !l)
      return NAN;
    Eigen::MatrixXd J;
    s.getJacobian(g, l, Eigen::Vector3d::Zero(), J);
    const Eigen::Matrix3d A = J.topRows<3>() * J.topRows<3>().transpose();
    return std::sqrt(std::max(0.0, A.determinant()));
  };

  auto solveOnce = [&](const std::string& group_name, const moveit::core::RobotState& seed_state) -> TrialResult {
    TrialResult r;
    const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(group_name);
    auto solver = jmg ? jmg->getSolverInstance() : nullptr;
    if (!solver)
    {
      RCLCPP_ERROR(LOGGER, "No IK solver for group '%s'", group_name.c_str());
      return r;
    }

    // IK query in the base frame of the solver, for the solver's tip frame
    const Eigen::Isometry3d eef_to_tip = start_state.getGlobalLinkTransform(MASTER_LINK).inverse() *
                                         start_state.getGlobalLinkTransform(solver->getTipFrame());
    const geometry_msgs::msg::Pose ik_query =
        tf2::toMsg(start_state.getFrameTransform(solver->getBaseFrame()).inverse() * target_master * eef_to_tip);

    // Seed in the joint order of the solver
    std::vector<double> group_values;
    seed_state.copyJointGroupPositions(jmg, group_values);
    const std::vector<size_t>& bijection = jmg->getKinematicsSolverJointBijection();
    std::vector<double> seed(bijection.size());
    for (std::size_t i = 0; i < bijection.size(); ++i)
      seed[i] = group_values[bijection[i]];

    // The coupled-arm plugin needs the start state as context_state (setFromIK() does not pass it).
    std::vector<double> ik_solution;
    moveit_msgs::msg::MoveItErrorCodes error;
    const auto t0 = std::chrono::steady_clock::now();  // wall clock: independent of use_sim_time
    r.ok = solver->searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ ik_query }, seed,
                                    jmg->getDefaultIKTimeout(), std::vector<double>(), ik_solution,
                                    kinematics::KinematicsBase::IKCallbackFn(), error,
                                    kinematics::KinematicsQueryOptions(), &start_state);
    r.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    r.error_code = error.val;
    if (!r.ok)
      return r;

    // Evaluate the solution (joints outside the group, e.g. chassis, stay at the start state)
    std::vector<double> solution(bijection.size());
    for (std::size_t i = 0; i < bijection.size(); ++i)
      solution[bijection[i]] = ik_solution[i];
    r.state = std::make_shared<moveit::core::RobotState>(start_state);
    r.state->setJointGroupPositions(jmg, solution);
    r.state->update();

    auto pose_error = [](const Eigen::Isometry3d& reached, const Eigen::Isometry3d& target, double& pos,
                         double& rot) {
      pos = (reached.translation() - target.translation()).norm();
      rot = Eigen::AngleAxisd(target.rotation().transpose() * reached.rotation()).angle();
    };
    double p1, r1, p2, r2;
    pose_error(r.state->getGlobalLinkTransform(MASTER_LINK), target_master, p1, r1);
    pose_error(r.state->getGlobalLinkTransform(SLAVE_LINK), target_slave, p2, r2);
    r.pos_err = std::max(p1, p2);
    r.rot_err = std::max(r1, r2);
    r.w_master = manipulability(*r.state, MASTER_ARM_GROUP, MASTER_LINK);
    r.w_slave = manipulability(*r.state, SLAVE_ARM_GROUP, SLAVE_LINK);
    r.joint_dist = r.state->distance(start_state, jmg);
    return r;
  };

  auto runGroup = [&](const std::string& group_name) {
    // Sanity check: seeded with the current state, the unchanged pose must be solvable.
    const TrialResult sanity = solveOnce(group_name, start_state);
    RCLCPP_INFO(LOGGER, "[%s] sanity (seed = current state): %s (%.3f s, error code %d)", group_name.c_str(),
                sanity.ok ? "OK" : "FAILED -> setup is wrong", sanity.time, sanity.error_code);

    std::vector<TrialResult> results;
    for (std::size_t i = 0; i < seeds.size(); ++i)
    {
      results.push_back(solveOnce(group_name, seeds[i]));
      const TrialResult& r = results.back();
      if (r.ok)
        RCLCPP_INFO(LOGGER,
                    "[%s] trial %2zu: OK   %.3f s | pos %.4f m rot %.4f rad | w_master %.4f w_slave %.4f | dq %.2f rad",
                    group_name.c_str(), i, r.time, r.pos_err, r.rot_err, r.w_master, r.w_slave, r.joint_dist);
      else
        RCLCPP_WARN(LOGGER, "[%s] trial %2zu: FAIL %.3f s (error code %d)", group_name.c_str(), i, r.time,
                    r.error_code);
    }

    std::vector<double> t, pe, re, wm, ws, dq;
    std::size_t n_ok = 0;
    for (const auto& r : results)
    {
      if (!r.ok)
        continue;
      ++n_ok;
      t.push_back(r.time);
      pe.push_back(r.pos_err);
      re.push_back(r.rot_err);
      wm.push_back(r.w_master);
      ws.push_back(r.w_slave);
      dq.push_back(r.joint_dist);
    }
    RCLCPP_INFO(LOGGER,
                "[%s] SUMMARY: success %zu/%zu | mean time %.3f s | mean pos err %.4f m, rot err %.4f rad | "
                "mean w_master %.4f, w_slave %.4f | mean dq %.2f rad",
                group_name.c_str(), n_ok, results.size(), mean(t), mean(pe), mean(re), mean(wm), mean(ws),
                mean(dq));
    return results;
  };

  // ---------------------------------------------------------------------------------------------
  // Run the comparison
  // ---------------------------------------------------------------------------------------------
  visual_tools.publishAxisLabeled(tf2::toMsg(target_master), "master goal");
  visual_tools.publishAxisLabeled(tf2::toMsg(target_slave), "slave goal");
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the comparison");

  const std::vector<TrialResult> res = runGroup(PLANNING_GROUP);
  const std::vector<TrialResult> res_opt = runGroup(PLANNING_GROUP_OPT);

  // Show the first seed for which both solvers found a solution
  for (std::size_t i = 0; i < res.size() && i < res_opt.size(); ++i)
  {
    if (res[i].ok && res_opt[i].ok)
    {
      RCLCPP_INFO(LOGGER, "Showing trial %zu (green: %s, red: %s)", i, PLANNING_GROUP.c_str(),
                  PLANNING_GROUP_OPT.c_str());
      Eigen::Isometry3d label = text_pose;
      label.translation().z() = 1.2;
      visual_tools.publishText(label, "IK_" + PLANNING_GROUP, rvt::GREEN, rvt::XLARGE);
      label.translation().z() = 1.4;
      visual_tools.publishText(label, "IK_" + PLANNING_GROUP_OPT, rvt::RED, rvt::XLARGE);
      visual_tools.publishRobotState(*res[i].state, rvt::GREEN);
      visual_tools_opt.publishRobotState(*res_opt[i].state, rvt::RED);
      visual_tools.trigger();
      break;
    }
  }

  visual_tools.prompt("Press 'next' to finish");

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.hideRobot();
  visual_tools_opt.hideRobot();

  rclcpp::shutdown();
  return 0;
}
