#include <custom_sampler/custom_sampler.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robin_moveit_plugins
{

  // All source files that use ROS logging should define a file-specific
  // static const rclcpp::Logger named LOGGER, located at the top of the file
  // and inside the namespace with the narrowest scope (if there is one)
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

  bool CustomSampler::configure(const moveit_msgs::msg::Constraints &constr)
  {
    RCLCPP_INFO(LOGGER, "Configuring CustomSampler with given constraints.");

    // Parse constr (joint/position/orientation/visibility constraints), store what you need,
    // set is_valid_ = true on success.
    is_valid_ = true;
    return is_valid_;
  }

  bool CustomSampler::sample(moveit::core::RobotState &state,
                             const moveit::core::RobotState &,
                             unsigned int max_attempts)
  {
    for (unsigned int i = 0; i < max_attempts; ++i)
    {
      RCLCPP_INFO(LOGGER, "Attempting to sample for the custom sampler, attempt %u", i);

      // fill joint values for jmg_ (the JointModelGroup), e.g. state.setToRandomPositions(jmg_, random_number_generator_);
      // then apply your custom logic (IK, biasing, etc.)
      state.update();
      if (/* satisfies your constraint */ true)
        return true;
    }

    return false;
  }

  constraint_samplers::ConstraintSamplerPtr
  CustomSamplerAllocator::alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
                                const moveit_msgs::msg::Constraints &constr)
  {
    auto s = std::make_shared<CustomSampler>(scene, group_name);
    return s->configure(constr) ? s : nullptr;
  }

  bool CustomSamplerAllocator::canService(const planning_scene::PlanningSceneConstPtr &, const std::string &group_name,
                                          const moveit_msgs::msg::Constraints &constr) const
  {
    RCLCPP_INFO(LOGGER, "Checking if CustomSamplerAllocator can service group: %s", group_name.c_str());

    // Return true only for the constraints you want to handle
    return group_name == "simple_dual_arm_l" && !constr.position_constraints.empty();
  }
} // namespace robin_moveit_plugins

PLUGINLIB_EXPORT_CLASS(robin_moveit_plugins::CustomSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)
