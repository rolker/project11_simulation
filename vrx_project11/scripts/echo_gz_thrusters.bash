#!/bin/bash

tmux new-session -d -s echo_gz_thrusters
tmux send-keys "gz topic -e -t /wamv/thrusters/left/thrust" C-m
tmux split-window -h
tmux send-keys "gz topic -e -t /wamv/thrusters/right/thrust" C-m
tmux attach-session -t echo_gz_thrusters
