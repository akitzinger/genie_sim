
#pragma once

#include <robin_moveit_plugins/constraint_samplers_plugin/constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>
#include <string>

namespace robin_moveit_plugins
{

// Exposes a robin_moveit_plugins sampler through the upstream MoveIt sampler interface.
class SamplerAdapter : public constraint_samplers::ConstraintSampler
{
public:
  SamplerAdapter(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
                 ConstraintSamplerPtr impl);

  bool configure(const moveit_msgs::msg::Constraints& constr) override;

  bool sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference_state,
              unsigned int max_attempts) override;

  void setVerbose(bool verbose) override;

  const std::string& getName() const override;

private:
  ConstraintSamplerPtr impl_;
};

class CustomSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:
  constraint_samplers::ConstraintSamplerPtr
  alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
        const moveit_msgs::msg::Constraints &constr) override;

  bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name,
                  const moveit_msgs::msg::Constraints &constr) const override;
};

}  // namespace robin_moveit_plugins
