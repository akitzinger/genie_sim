#include <moveit/planning_interface/planning_request_adapter.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/utils/logger.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <Eigen/Geometry>

namespace robin_moveit_plugins
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.robin_moveit_plugins.default_path_constraints_adapter");
}

constexpr char GROUP_NAME[] = "sync_dual_arm_l";
constexpr char CONSTRAINT_FRAME[] = "arm_base_link";
constexpr char CONSTRAINED_LINK[] = "arm_l_end_link";
}  // namespace

class DefaultPathConstraintsAdapter : public planning_interface::PlanningRequestAdapter
{
public:

  DefaultPathConstraintsAdapter() = default;

  ~DefaultPathConstraintsAdapter() override = default;

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("DefaultPathConstraintsAdapter");
  }

  [[nodiscard]] moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    planning_interface::MotionPlanRequest& req) const override
  {
    RCLCPP_DEBUG(getLogger(), "Request BEFORE adapter:\n%s",
            moveit_msgs::msg::to_yaml(req).c_str());

    if (req.group_name == GROUP_NAME && req.goal_constraints.size() == 1)
    {
      auto& goal = req.goal_constraints[0];
      const bool is_joint_goal = !goal.joint_constraints.empty() &&
                                goal.position_constraints.empty() &&
                                goal.orientation_constraints.empty();
      if (is_joint_goal)
      {
        moveit::core::RobotState goal_state(*planning_scene->getCurrentStateUpdated(req.start_state));
        for (const auto& jc : goal.joint_constraints)
          goal_state.setVariablePosition(jc.joint_name, jc.position);
        goal_state.updateLinkTransforms();

        // pose of the constrained link relative to the reference frame
        const Eigen::Isometry3d T_base = goal_state.getGlobalLinkTransform(CONSTRAINT_FRAME);
        const Eigen::Isometry3d T_rel  = T_base.inverse() * goal_state.getGlobalLinkTransform(CONSTRAINED_LINK);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = CONSTRAINT_FRAME;
        pose.pose = tf2::toMsg(T_rel);

        goal = kinematic_constraints::constructGoalConstraints(
            CONSTRAINED_LINK, pose, 1e-3 /*pos tol [m]*/, 1e-2 /*ori tol [rad]*/);

        RCLCPP_INFO(getLogger(), "Converted joint goal to pose goal.");
      }
    }

    if (req.group_name == GROUP_NAME && 
      req.path_constraints.position_constraints.empty() && 
      req.path_constraints.orientation_constraints.empty())
    {
      // Get current state
      moveit::core::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);
      start_state->updateLinkTransforms();

      // Defined start pose from current state
      const Eigen::Isometry3d& KI_T_I1 = start_state->getGlobalLinkTransform(CONSTRAINED_LINK);
      const Eigen::Isometry3d& KI_T_I0 = start_state->getGlobalLinkTransform(CONSTRAINT_FRAME);
      const auto R_I0 = KI_T_I0.rotation();
      const auto R_01 = KI_T_I0.rotation().inverse() * KI_T_I1.rotation();

      // Pose in reference frame 
      const Eigen::Vector3d t = R_I0.inverse() * (KI_T_I1.translation() - KI_T_I0.translation());
      const Eigen::Quaterniond q(R_01);

      RCLCPP_DEBUG(getLogger(), "Start pose: position=(%f, %f, %f), orientation=(%f, %f, %f, %f)",
                   t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());

      // Set start pose
      geometry_msgs::msg::Pose start_pose1;
      start_pose1.orientation.x = q.x();
      start_pose1.orientation.y = q.y();
      start_pose1.orientation.z = q.z();
      start_pose1.orientation.w = q.w();
      start_pose1.position.x = t.x();
      start_pose1.position.y = t.y();
      start_pose1.position.z = t.z();

      // Orientation Constraints
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = CONSTRAINED_LINK;
      ocm.header.frame_id = CONSTRAINT_FRAME;
      ocm.orientation.x = start_pose1.orientation.x;
      ocm.orientation.y = start_pose1.orientation.y;
      ocm.orientation.z = start_pose1.orientation.z;
      ocm.orientation.w = start_pose1.orientation.w;
      ocm.absolute_x_axis_tolerance = 3.14;
      ocm.absolute_y_axis_tolerance = 3.14;
      ocm.absolute_z_axis_tolerance = 3.14;
      ocm.weight = 1.0;

      // Position Box Constraints
      moveit_msgs::msg::PositionConstraint pcm;
      pcm.link_name = CONSTRAINED_LINK;
      pcm.header.frame_id = CONSTRAINT_FRAME;
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = { 0.8, 1.0, 1.0 };
      pcm.constraint_region.primitives.push_back(box);
      pcm.weight = 1.0;

      geometry_msgs::msg::Pose bp;
      bp.position.x = 0.55;
      bp.position.y = 0.2;
      bp.position.z = 0.0;
      bp.orientation.x = 0.0;
      bp.orientation.y = 0.0;
      bp.orientation.z = 0.0;
      bp.orientation.w = 1.0;
      pcm.constraint_region.primitive_poses.push_back(bp);
      pcm.weight = 1.0;

      req.path_constraints.orientation_constraints.push_back(ocm);
      req.path_constraints.position_constraints.push_back(pcm);

      RCLCPP_INFO(getLogger(), "Default path constraints applied for group %s.", req.group_name.c_str());

    }

    RCLCPP_DEBUG(getLogger(), "Request AFTER adapter:\n%s",
                moveit_msgs::msg::to_yaml(req).c_str());

    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, std::string(""), getDescription());
  }
};

}  // namespace robin_moveit_plugins

PLUGINLIB_EXPORT_CLASS(robin_moveit_plugins::DefaultPathConstraintsAdapter,
                       planning_interface::PlanningRequestAdapter)
