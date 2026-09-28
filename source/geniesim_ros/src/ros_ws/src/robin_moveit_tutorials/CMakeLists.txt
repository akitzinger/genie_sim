cmake_minimum_required(VERSION 3.22)
project(robin_moveit_tutorials)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

find_package(ament_cmake)
find_package(Boost CONFIG REQUIRED)
find_package(control_msgs)
find_package(Eigen3)
find_package(interactive_markers)
find_package(moveit_core)
find_package(moveit_ros_planning)
find_package(moveit_ros_planning_interface)
find_package(moveit_ros_trajectory_cache)
find_package(moveit_servo)
find_package(moveit_task_constructor_core)
find_package(moveit_visual_tools)
find_package(pluginlib)
find_package(rclcpp)
find_package(rclcpp_action)
find_package(rviz_visual_tools)
find_package(tf2_ros)
find_package(warehouse_ros_sqlite)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(examples/move_group_interface)


ament_export_include_directories(include)

ament_package()
