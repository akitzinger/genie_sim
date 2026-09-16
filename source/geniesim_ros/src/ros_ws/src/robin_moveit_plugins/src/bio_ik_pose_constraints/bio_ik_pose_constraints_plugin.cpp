#include <bio_ik/goal_types.h>
#include <bio_ik/bio_ik.h>

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>
#include <string>
#include <utility>
#include <vector>

#include "robin_moveit_plugins/pose_constraints.hpp"

namespace robin_moveit_plugins
{

  class BioIKPlugin : public kinematics::KinematicsBase
  {
  public:
    BioIKPlugin() = default;

    ~BioIKPlugin() override
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
          RCLCPP_ERROR(node->get_logger(), "[RobinBioIK] Failed to load bio_ik: %s", exception.what());
        }
        return false;
      }

      if (!inner_->initialize(
              node, robot_model, group_name, base_frame, tip_frames, search_discretization))
      {
        if (node)
        {
          RCLCPP_ERROR(node->get_logger(), "[RobinBioIK] bio_ik initialization failed");
        }
        return false;
      }

      master_link_ = getParameterString(node, "kinematics_solver_master_link");
      slave_link_ = getParameterString(node, "kinematics_solver_slave_link");
      master_pose_weight_ = getParameterDouble(
          node, "kinematics_solver_master_pose_weight", 1.0);
      slave_pose_weight_ = getParameterDouble(
          node, "kinematics_solver_slave_pose_weight", 1.0);

      // Default fallbacks based on group name conventions if parameters are omitted
      if (master_link_.empty())
      {
        if (group_name == "simple_dual_arm_l")
        {
          master_link_ = "arm_l_end_link";
          slave_link_ = "arm_r_end_link";
        }
        else if (group_name == "simple_dual_arm_r")
        {
          master_link_ = "arm_r_end_link";
          slave_link_ = "arm_l_end_link";
        }
        else if (tip_frames_.size() >= 2)
        {
          master_link_ = tip_frames_[0];
          slave_link_ = tip_frames_[1];
        }
      }

      // The coupled group's public IK tip must be the configured master. The
      // underlying BioIK plugin auto-discovers both arm end-effectors, while
      // RViz uses this list to attach the interactive marker. Exposing the
      // incoming composite-group order here can attach a marker to the slave
      // arm, especially when both groups have the same SRDF subgroup order.
      // Keep the slave private to the coupled goal stack below.
      tip_frames_.clear();
      if (!master_link_.empty())
      {
        tip_frames_.push_back(master_link_);
      }
      else
      {
        tip_frames_ = tip_frames;
      }

      if (node)
      {
        RCLCPP_INFO(
            node->get_logger(),
            "[RobinBioIK] Initialized for '%s' (master='%s' weight=%.3f, slave='%s' weight=%.3f, tips=%zu)",
            group_name.c_str(), master_link_.c_str(), master_pose_weight_,
            slave_link_.c_str(), slave_pose_weight_, tip_frames_.size());
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
      auto bio_options = makeCoupledOptions(ik_pose, ik_seed_state);
      return inner_->searchPositionIK(
          ik_pose, ik_seed_state, timeout, solution, error_code,
          bio_options ? *bio_options : options);
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
      auto bio_options = makeCoupledOptions(ik_pose, ik_seed_state);
      return inner_->searchPositionIK(
          ik_pose, ik_seed_state, timeout, consistency_limits, solution, error_code,
          bio_options ? *bio_options : options);
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
      auto bio_options = makeCoupledOptions(ik_pose, ik_seed_state);
      return inner_->searchPositionIK(
          ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
          bio_options ? *bio_options : options);
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
      auto bio_options = makeCoupledOptions(ik_pose, ik_seed_state);
      return inner_->searchPositionIK(
          ik_pose, ik_seed_state, timeout, consistency_limits, solution,
          solution_callback, error_code, bio_options ? *bio_options : options);
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
      if (ik_poses.empty())
      {
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
      }

      if (ik_poses.size() == 1)
      {
        auto bio_options = makeCoupledOptions(ik_poses[0], ik_seed_state, context_state);
        return inner_->searchPositionIK(
            ik_poses, ik_seed_state, timeout, consistency_limits, solution,
            solution_callback, error_code, bio_options ? *bio_options : options, context_state);
      }

      return inner_->searchPositionIK(
          ik_poses, ik_seed_state, timeout, consistency_limits, solution,
          solution_callback, error_code, options, context_state);
    }

    bool getPositionIK(
        const geometry_msgs::msg::Pose &ik_pose,
        const std::vector<double> &ik_seed_state,
        std::vector<double> &solution,
        moveit_msgs::msg::MoveItErrorCodes &error_code,
        const kinematics::KinematicsQueryOptions &options =
            kinematics::KinematicsQueryOptions()) const override
    {
      return searchPositionIK(
          ik_pose, ik_seed_state, default_timeout_, solution, error_code, options);
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
          ik_poses, ik_seed_state, default_timeout_, std::vector<double>(), solution,
          IKCallbackFn(), error_code, options);
    }

    bool getPositionFK(
        const std::vector<std::string> &link_names,
        const std::vector<double> &joint_angles,
        std::vector<geometry_msgs::msg::Pose> &poses) const override
    {
      if (!robot_model_ || joint_angles.size() != getJointNames().size())
      {
        return false;
      }

      moveit::core::RobotState state(robot_model_);
      state.setToDefaultValues();
      size_t cursor = 0;
      for (const auto &joint_name : getJointNames())
      {
        const auto *joint_model = robot_model_->getJointModel(joint_name);
        if (!joint_model)
        {
          return false;
        }
        for (size_t variable = 0; variable < joint_model->getVariableCount(); ++variable)
        {
          state.setVariablePosition(
              joint_model->getFirstVariableIndex() + variable, joint_angles[cursor++]);
        }
      }
      state.update();

      poses.clear();
      poses.reserve(link_names.size());
      for (const auto &link_name : link_names)
      {
        const auto *link_model = robot_model_->getLinkModel(link_name);
        if (!link_model)
        {
          return false;
        }
        poses.push_back(tf2::toMsg(state.getGlobalLinkTransform(link_model)));
      }
      return true;
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
    double getParameterDouble(
        const rclcpp::Node::SharedPtr &node, const std::string &name, double default_value) const
    {
      if (!node)
      {
        return default_value;
      }

      const std::string scoped_name =
          "robot_description_kinematics." + getGroupName() + "." + name;
      const std::string group_scoped_name = getGroupName() + "." + name;
      const std::vector<std::string> candidates = {scoped_name, group_scoped_name, name};

      for (const auto &candidate : candidates)
      {
        if (node->has_parameter(candidate))
        {
          double value = default_value;
          if (node->get_parameter(candidate, value))
          {
            return value;
          }
        }
      }

      for (const auto &candidate : candidates)
      {
        try
        {
          node->declare_parameter(candidate, default_value);
        }
        catch (...)
        {
        }
        if (node->has_parameter(candidate))
        {
          double value = default_value;
          if (node->get_parameter(candidate, value))
          {
            return value;
          }
        }
      }
      return default_value;
    }

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
        const geometry_msgs::msg::Pose &master_pose,
        const std::vector<double> &ik_seed_state,
        const moveit::core::RobotState *context_state = nullptr) const
    {
      if (master_link_.empty() || slave_link_.empty() || !robot_model_)
      {
        return nullptr;
      }

      const auto *master_model = robot_model_->getLinkModel(master_link_);
      const auto *slave_model = robot_model_->getLinkModel(slave_link_);
      if (!master_model || !slave_model)
      {
        if (node_)
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "[RobinBioIK] Cannot build coupled goals: link '%s' or '%s' is missing",
              master_link_.c_str(), slave_link_.c_str());
        }
        return nullptr;
      }

      moveit::core::RobotState seed_state(robot_model_);
      if (context_state)
      {
        seed_state = *context_state;
      }
      else
      {
        seed_state.setToDefaultValues();
      }

      size_t cursor = 0;
      for (const auto &joint_name : getJointNames())
      {
        const auto *joint_model = robot_model_->getJointModel(joint_name);
        if (!joint_model)
        {
          continue;
        }
        for (size_t variable = 0; variable < joint_model->getVariableCount(); ++variable)
        {
          if (cursor < ik_seed_state.size())
          {
            seed_state.setVariablePosition(
                joint_model->getFirstVariableIndex() + variable, ik_seed_state[cursor]);
          }
          ++cursor;
        }
      }
      seed_state.update();

      // Base transform: converts poses from getBaseFrame() to robot model root frame
      const Eigen::Isometry3d base_transform =
          seed_state.getFrameTransform(getBaseFrame());

      Eigen::Isometry3d master_pose_in_base;
      tf2::fromMsg(master_pose, master_pose_in_base);
      const Eigen::Isometry3d master_target_model =
          base_transform * master_pose_in_base;

      // Current global link transforms in model root frame
      const Eigen::Isometry3d master_current_model =
          seed_state.getGlobalLinkTransform(master_model);
      const Eigen::Isometry3d slave_current_model =
          seed_state.getGlobalLinkTransform(slave_model);

      // Relative transform from master to slave link
      const Eigen::Isometry3d relative_slave =
          master_current_model.inverse() * slave_current_model;

      // Target slave pose in model root frame
      const Eigen::Isometry3d slave_target_model =
          master_target_model * relative_slave;

      auto options = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
      options->replace = true;
      options->goals.push_back(
          makePoseGoal(master_link_, master_target_model, master_pose_weight_));
      options->goals.push_back(
          makePoseGoal(slave_link_, slave_target_model, slave_pose_weight_));
      options->goals.push_back(
          std::make_unique<bio_ik::MinimalDisplacementGoal>(0.5, true));
      options->goals.push_back(
          std::make_unique<bio_ik::AvoidJointLimitsGoal>(0.2, true));
      options->goals.push_back(
          std::make_unique<bio_ik::CenterJointsGoal>(0.1, true));
      return options;
    }

    static std::unique_ptr<bio_ik::PoseGoal> makePoseGoal(
        const std::string &link_name, const Eigen::Isometry3d &pose, double weight)
    {
      tf2::Vector3 position(
          pose.translation().x(), pose.translation().y(), pose.translation().z());
      Eigen::Quaterniond orientation(pose.rotation());
      tf2::Quaternion rotation(
          orientation.x(), orientation.y(), orientation.z(), orientation.w());
      auto goal = std::make_unique<bio_ik::PoseGoal>(link_name, position, rotation, weight);
      goal->setRotationScale(0.5);
      return goal;
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader_;
    std::shared_ptr<kinematics::KinematicsBase> inner_;
    std::string master_link_;
    std::string slave_link_;
    double master_pose_weight_{1.0};
    double slave_pose_weight_{1.0};
  };

} // namespace robin_moveit_plugins

PLUGINLIB_EXPORT_CLASS(
    robin_moveit_plugins::BioIKPlugin,
    kinematics::KinematicsBase)
