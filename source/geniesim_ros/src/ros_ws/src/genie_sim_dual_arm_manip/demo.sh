#!/bin/bash

# chmod +x source/geniesim_ros/src/ros_ws/src/genie_sim_dual_arm_manip/demo.sh
# ./source/geniesim_ros/src/ros_ws/src/genie_sim_dual_arm_manip/demo.sh

geniesim ros build dev
source devel/setup.bash
tmuxinator start -p source/geniesim_ros/src/ros_ws/src/genie_sim_dual_arm_manip/demo.yml
