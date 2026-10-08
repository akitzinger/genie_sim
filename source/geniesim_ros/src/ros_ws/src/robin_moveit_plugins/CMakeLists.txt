cmake_minimum_required(VERSION 3.22)
project(robin_moveit_plugins)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(bio_ik REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenMP)
find_package(ompl REQUIRED)

# ===========================================================================
# BioIK + Pose Constraints Plugin
# ===========================================================================
add_library(robin_moveit_plugins_kinematics_base_plugin SHARED
  src/kinematics_base_plugin/kinematics_base_plugin.cpp
)
target_include_directories(robin_moveit_plugins_kinematics_base_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robin_moveit_plugins_kinematics_base_plugin
  Eigen3::Eigen
  yaml-cpp
  bio_ik::bio_ik
)
ament_target_dependencies(robin_moveit_plugins_kinematics_base_plugin
  rclcpp
  moveit_core
  pluginlib
  tf2_eigen
  geometry_msgs
  moveit_msgs
  bio_ik
  ament_index_cpp
)

# ===========================================================================
# Custom Constraint Sampler Plugin
# ===========================================================================
add_library(robin_moveit_plugins_constraint_samplers_plugin SHARED
  src/constraint_samplers_plugin/constraint_samplers_plugin.cpp
  src/constraint_samplers_plugin/constraint_sampler.cpp
  src/constraint_samplers_plugin/constraint_sampler_manager.cpp
  src/constraint_samplers_plugin/default_constraint_samplers.cpp
  src/constraint_samplers_plugin/union_constraint_sampler.cpp
)
target_include_directories(robin_moveit_plugins_constraint_samplers_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robin_moveit_plugins_constraint_samplers_plugin
  moveit_core
  pluginlib
  moveit_msgs
)

# ===========================================================================
# Moveit Planner Plugin
# ===========================================================================
add_library(robin_moveit_plugins_planner_manager_plugin SHARED
  src/ompl_interface/ompl_interface.cpp
  src/ompl_interface/planning_context_manager.cpp
  src/ompl_interface/model_based_planning_context.cpp
  src/ompl_interface/parameterization/model_based_state_space.cpp
  src/ompl_interface/parameterization/model_based_state_space_factory.cpp
  src/ompl_interface/parameterization/joint_space/constrained_planning_state_space.cpp
  src/ompl_interface/parameterization/joint_space/constrained_planning_state_space_factory.cpp
  src/ompl_interface/parameterization/joint_space/joint_model_state_space.cpp
  src/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.cpp
  src/ompl_interface/parameterization/work_space/pose_model_state_space.cpp
  src/ompl_interface/parameterization/work_space/pose_model_state_space_factory.cpp
  src/ompl_interface/detail/ompl_constraints.cpp
  src/ompl_interface/detail/threadsafe_state_storage.cpp
  src/ompl_interface/detail/state_validity_checker.cpp
  src/ompl_interface/detail/projection_evaluators.cpp
  src/ompl_interface/detail/goal_union.cpp
  src/ompl_interface/detail/constraints_library.cpp
  src/ompl_interface/detail/constrained_sampler.cpp
  src/ompl_interface/detail/constrained_goal_sampler.cpp)

  target_include_directories(robin_moveit_plugins_planner_manager_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robin_moveit_plugins_planner_manager_plugin
  PUBLIC
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  pluginlib
  tf2_eigen
  tf2_ros
  Boost)


# ===========================================================================
# Plugin description exports
# ===========================================================================
pluginlib_export_plugin_description_file(moveit_core kinematics_base_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core constraint_samplers_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_planners ompl_interface_plugin_description.xml)

# ===========================================================================
# Install
# ===========================================================================
install(
  TARGETS
    robin_moveit_plugins_kinematics_base_plugin
    robin_moveit_plugins_constraint_samplers_plugin
    robin_moveit_plugins_planner_manager_plugin
  EXPORT  ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})

ament_package()
