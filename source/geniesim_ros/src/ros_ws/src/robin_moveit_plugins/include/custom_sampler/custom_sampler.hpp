#pragma once
#include <moveit/constraint_samplers/constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>

namespace robin_moveit_plugins
{
  class CustomSampler : public constraint_samplers::ConstraintSampler
  {
  public:
    CustomSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name)
        : ConstraintSampler(scene, group_name) {}

    bool configure(const moveit_msgs::msg::Constraints &constr) override;
    bool sample(moveit::core::RobotState &state, const moveit::core::RobotState &reference_state,
                unsigned int max_attempts) override;
    const std::string &getName() const override
    {
      static const std::string n = "CustomSampler";
      return n;
    }
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
} // namespace robin_moveit_plugins
