#!/bin/bash

tmux new -d -s project11_sim_robot_core
tmux setenv ROS_MASTER_URI http://localhost:11312
tmux send-keys "ROS_MASTER_URI=http://localhost:11312 roscore -p 11312" C-m

#tmux new -d -s project11_sim_robot
#tmux setenv ROS_MASTER_URI http://localhost:11312
export ROS_MASTER_URI=http://localhost:11312
rosrun rosmon rosmon project11_simulation sim_robot.launch
#tmux send-keys "rosrun rosmon rosmon project11_simulation sim_robot.launch" C-m
#tmux attach -t project11_sim_robot
