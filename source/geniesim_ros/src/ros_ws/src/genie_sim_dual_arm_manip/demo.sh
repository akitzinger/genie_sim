#!/bin/bash

# chmod +x source/geniesim_ros/src/ros_ws/src/genie_sim_dual_arm_manip/demo.sh
# ./source/geniesim_ros/src/ros_ws/src/genie_sim_dual_arm_manip/demo.sh

# sudo apt update
# sudo apt install tmuxinator

geniesim ros build dev
source devel/setup.bash
tmuxinator start -p source/geniesim_ros/src/ros_ws/src/genie_sim_dual_arm_manip/demo.yml

# Str+B, :kill-session
