#include "robin_moveit_plugins/kinematics_base_plugin/kinematics_base_plugin.hpp"

#include <bio_ik/goal_types.h>
#include <bio_ik/bio_ik.h>

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace bio_ik
{
  const BioIKKinematicsQueryOptions *toBioIKKinematicsQueryOptions(const void *ptr);
}

namespace robin_moveit_plugins
{
  static rclcpp::Logger getLogger()
  {
    return moveit::getLogger("robin_moveit_plugins");
  }

  class RobinIKPlugin : public kinematics::KinematicsBase
  {
  public:
    RobinIKPlugin() = default;

    ~RobinIKPlugin() override
    {
      inner_.reset();
      loader_.reset();
    }

    bool initialize(
        const rclcpp::Node::SharedPtr &node,
        const moveit::core::RobotModel &robot_model,
        const std::string &group_name,
        const std::string &base_frame,
        const std::vector<std::string> &tip_frames,
        double search_discretization) override
    {
      node_ = node;
      storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

      try
      {
        loader_ = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
            "moveit_core", "kinematics::KinematicsBase");
        inner_ = loader_->createSharedInstance("bio_ik/BioIKKinematicsPlugin");
      }
      catch (const pluginlib::PluginlibException &exception)
      {
        if (node)
        {
          RCLCPP_ERROR(node->get_logger(), "Failed to load bio_ik: %s", exception.what());
        }
        return false;
      }

      if (!inner_->initialize(
              node, robot_model, group_name, base_frame, tip_frames, search_discretization))
      {
        if (node)
        {
          RCLCPP_ERROR(node->get_logger(), "bio_ik initialization failed");
        }
        return false;
      }

      first_tip_link_ = getParameterString(node, "kinematics_solver_master_link");

      // Default fallbacks based on group name conventions if parameters are omitted
      if (first_tip_link_.empty())
      {
        if (group_name == "simple_dual_arm_l")
        {
          first_tip_link_ = "arm_l_end_link";
          second_tip_link_ = "arm_r_end_link";
        }
        else if (group_name == "simple_dual_arm_r")
        {
          first_tip_link_ = "arm_r_end_link";
          second_tip_link_ = "arm_l_end_link";
        }
      }
      else
      {
        if (first_tip_link_ == "arm_l_end_link")
        {
          second_tip_link_ = "arm_r_end_link";
        }
        else if (first_tip_link_ == "arm_r_end_link")
        {
          second_tip_link_ = "arm_l_end_link";
        }
      }

      // The first tip link is considered the primary end-effector for the coupled group.
      // The underlying BioIK plugin auto-discovers both arm end-effectors, while rviz 
      // should only see the primary one.
      tip_frames_.clear();
      if (!first_tip_link_.empty())
      {
        tip_frames_.push_back(first_tip_link_);
      }
      else
      {
        tip_frames_ = tip_frames;
      }

      if (node)
      {
        RCLCPP_INFO(
            node->get_logger(),
            "[RobinBioIK] Initialized for '%s' (master='%s', tips=%zu)",
            group_name.c_str(), first_tip_link_.c_str(), tip_frames_.size());
      }

      return true;
    }

    bool supportsGroup(
        const moveit::core::JointModelGroup *, std::string *error_text_out = nullptr) const override
    {
      if (error_text_out)
      {
        error_text_out->clear();
      }
      return true;
    }

    bool searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const override
    {
      RCLCPP_DEBUG(getLogger(), "searchPositionIK [1] called with pose: %f %f %f %f %f %f %f",
                  ik_pose.position.x, ik_pose.position.y, ik_pose.position.z,
                  ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w);

      return searchPositionIK(
          std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state, timeout,
          std::vector<double>(), solution, IKCallbackFn(), error_code, options);
    }

    bool searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const override
    {
      RCLCPP_DEBUG(getLogger(), "searchPositionIK [2] called with pose: %f %f %f %f %f %f %f",
            ik_pose.position.x, ik_pose.position.y, ik_pose.position.z,
            ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w);

      return searchPositionIK(
          std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state, timeout,
          consistency_limits, solution, IKCallbackFn(), error_code, options);
    }

    bool searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const override
    {
      RCLCPP_DEBUG(getLogger(), "searchPositionIK [3] called with pose: %f %f %f %f %f %f %f",
            ik_pose.position.x, ik_pose.position.y, ik_pose.position.z,
            ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w);

      return searchPositionIK(
          std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state, timeout,
          std::vector<double>(), solution, solution_callback, error_code, options);
    }

    bool searchPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const override
    {
      RCLCPP_DEBUG(getLogger(), "searchPositionIK [4] called with pose: %f %f %f %f %f %f %f",
            ik_pose.position.x, ik_pose.position.y, ik_pose.position.z,
            ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w);

      return searchPositionIK(
          std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state, timeout,
          consistency_limits, solution, solution_callback, error_code, options);
    }

    bool searchPositionIK(
        const std::vector<geometry_msgs::msg::Pose> &ik_poses,
        const std::vector<double> &ik_seed_state,
        double timeout,
        const std::vector<double> &consistency_limits,
        std::vector<double> &solution,
        const IKCallbackFn &solution_callback,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions(),
        const moveit::core::RobotState *context_state = nullptr) const override
    {
      RCLCPP_DEBUG(getLogger(), "searchPositionIK [5] called with pose: %f %f %f %f %f %f %f",
            ik_poses[0].position.x, ik_poses[0].position.y, ik_poses[0].position.z,
            ik_poses[0].orientation.x, ik_poses[0].orientation.y, ik_poses[0].orientation.z, ik_poses[0].orientation.w);
      
      if (ik_poses.empty())
      {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
      }

      // Caller-supplied BioIK options take precedence over the coupled goal stack.
      if (ik_poses.size() == 1 && !bio_ik::toBioIKKinematicsQueryOptions(&options))
      {
        std::vector<geometry_msgs::msg::Pose> poses;
        auto bio_options = makeCoupledOptions(
            ik_poses[0], ik_seed_state, context_state, poses);
        if (bio_options)
        {
          return inner_->searchPositionIK(
              poses, ik_seed_state, timeout, consistency_limits, solution,
              solution_callback, error_code, *bio_options);
        }
      }

      RCLCPP_WARN(getLogger(),
          "Multiple poses in searchPositionIK!");

      return inner_->searchPositionIK(
          ik_poses, ik_seed_state, timeout, consistency_limits, solution,
          solution_callback, error_code, options);
    }

    bool getPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const override
    {
      const bool success = searchPositionIK(
          ik_pose, ik_seed_state, 0.0, solution, error_code, options);
      if (!success && error_code.val == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION)
      {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
      }
      return success;
    }

    bool getPositionIK(
        const std::vector<geometry_msgs::msg::Pose> &ik_poses,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const
    {
      return searchPositionIK(
          ik_poses, ik_seed_state, 0.0, std::vector<double>(), solution,
          IKCallbackFn(), error_code, options);
    }

    bool getPositionFK(
        const std::vector<std::string> &link_names,
        const std::vector<double> &joint_angles,
        std::vector<geometry_msgs::msg::Pose> &poses) const override
    {
      return inner_->getPositionFK(link_names, joint_angles, poses);
    }

    const std::vector<std::string> &getJointNames() const override
    {
      return inner_->getJointNames();
    }

    const std::vector<std::string> &getLinkNames() const override
    {
      return inner_->getLinkNames();
    }

    const std::vector<std::string> &getTipFrames() const override
    {
      return tip_frames_.empty() ? inner_->getTipFrames() : tip_frames_;
    }

  private:
    std::string getParameterString(
        const rclcpp::Node::SharedPtr &node, const std::string &name) const
    {
      if (!node)
      {
        return {};
      }

      std::string value;
      const std::string scoped_name =
          "robot_description_kinematics." + getGroupName() + "." + name;
      const std::string group_scoped_name = getGroupName() + "." + name;

      const std::vector<std::string> candidates = {scoped_name, group_scoped_name, name};
      for (const auto &cand : candidates)
      {
        if (node->has_parameter(cand))
        {
          if (node->get_parameter(cand, value) && !value.empty())
          {
            return value;
          }
        }
      }

      for (const auto &cand : candidates)
      {
        try
        {
          node->declare_parameter(cand, std::string{});
        }
        catch (...)
        {
        }
        if (node->has_parameter(cand))
        {
          if (node->get_parameter(cand, value) && !value.empty())
          {
            return value;
          }
        }
      }
      return {};
    }

    std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> makeCoupledOptions(
        const geometry_msgs::msg::Pose &single_tip_pose,
        const std::vector<double> &ik_seed_state,
        const moveit::core::RobotState *context_state,
        std::vector<geometry_msgs::msg::Pose> &poses_out) const
    {
      if (!robot_model_ || first_tip_link_.empty() || second_tip_link_.empty())
      {
        return nullptr;
      }

      // relative pose of the second tip link with respect to the first tip link
      Eigen::Isometry3d first_tip_target;
      tf2::fromMsg(single_tip_pose, first_tip_target);
      const Eigen::Isometry3d second_tip_target = first_tip_target * relative_initial_pose;

      // change pose gaol msg
      const geometry_msgs::msg::Pose first_tip_pose = tf2::toMsg(first_tip_target);
      const geometry_msgs::msg::Pose second_tip_pose = tf2::toMsg(second_tip_target);
      poses_out = {first_tip_pose, second_tip_pose};

      RCLCPP_DEBUG(getLogger(),
                   "First tip pose: %f %f %f, Second tip pose: %f %f %f",
                   poses_out[0].position.x, poses_out[0].position.y, poses_out[0].position.z,
                   poses_out[1].position.x, poses_out[1].position.y, poses_out[1].position.z);

      RCLCPP_DEBUG(getLogger(),
                   "Relative pose position: %f %f %f orientation: %f %f %f",
                   relative_initial_pose.translation().x(),
                   relative_initial_pose.translation().y(),
                   relative_initial_pose.translation().z(),
                   relative_initial_pose.rotation().eulerAngles(0, 1, 2).x(),
                   relative_initial_pose.rotation().eulerAngles(0, 1, 2).y(),
                   relative_initial_pose.rotation().eulerAngles(0, 1, 2).z());

      // BioIK options
      auto options = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
      options->replace = false;
      addCustomGoals(*options);
      return options;
    }

    // extra goals applied on top of bio_ik's default goals
    void addCustomGoals(bio_ik::BioIKKinematicsQueryOptions &options) const
    {

    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader_;
    std::shared_ptr<kinematics::KinematicsBase> inner_;
    
    std::string first_tip_link_ = "arm_l_end_link";
    std::string second_tip_link_ = "arm_r_end_link";
    const Eigen::Isometry3d relative_initial_pose =
        Eigen::Translation3d(0.029433, 0.000006, 0.329343) *
        Eigen::AngleAxisd(0.002945, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(-2.965842, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-3.140918, Eigen::Vector3d::UnitZ());
      };

} // namespace robin_moveit_plugins

PLUGINLIB_EXPORT_CLASS(
    robin_moveit_plugins::RobinIKPlugin,
    kinematics::KinematicsBase)
