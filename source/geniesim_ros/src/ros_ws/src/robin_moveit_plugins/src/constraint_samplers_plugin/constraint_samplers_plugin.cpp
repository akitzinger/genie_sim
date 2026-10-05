#include <robin_moveit_plugins/constraint_samplers_plugin/constraint_samplers_plugin.hpp>
#include <robin_moveit_plugins/constraint_samplers_plugin/constraint_sampler_manager.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <moveit/utils/logger.hpp>

namespace robin_moveit_plugins
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("robin_moveit_plugins.custom_sampler");
}
}  // namespace

SamplerAdapter::SamplerAdapter(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
                               ConstraintSamplerPtr impl)
  : constraint_samplers::ConstraintSampler(scene, group_name), impl_(std::move(impl))
{
  is_valid_ = impl_->isValid();
  frame_depends_ = impl_->getFrameDependency();
}

bool SamplerAdapter::configure(const moveit_msgs::msg::Constraints& constr)
{
  is_valid_ = impl_->configure(constr);
  frame_depends_ = impl_->getFrameDependency();
  return is_valid_;
}

bool SamplerAdapter::sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference_state,
                            unsigned int max_attempts)
{
  impl_->setGroupStateValidityCallback(group_state_validity_callback_);
  return impl_->sample(state, reference_state, max_attempts);
}

void SamplerAdapter::setVerbose(bool verbose)
{
  constraint_samplers::ConstraintSampler::setVerbose(verbose);
  impl_->setVerbose(verbose);
}

const std::string& SamplerAdapter::getName() const
{
  return impl_->getName();
}

constraint_samplers::ConstraintSamplerPtr
CustomSamplerAllocator::alloc(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
                              const moveit_msgs::msg::Constraints& constr)
{
  ConstraintSamplerPtr impl = ConstraintSamplerManager::selectDefaultSampler(scene, group_name, constr);
  if (!impl)
  {
    RCLCPP_WARN(getLogger(), "No sampler could be constructed for group '%s'", group_name.c_str());
    return nullptr;
  }
  return std::make_shared<SamplerAdapter>(scene, group_name, std::move(impl));
}

bool CustomSamplerAllocator::canService(const planning_scene::PlanningSceneConstPtr& scene,
                                        const std::string& group_name,
                                        const moveit_msgs::msg::Constraints& constr) const
{
  return (group_name == "simple_dual_arm_l" || group_name == "simple_dual_arm_r") && scene->getRobotModel()->hasJointModelGroup(group_name) &&
         (!constr.joint_constraints.empty() || !constr.position_constraints.empty() ||
          !constr.orientation_constraints.empty());
}

}  // end of namespace robin_moveit_plugins

PLUGINLIB_EXPORT_CLASS(robin_moveit_plugins::CustomSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)
