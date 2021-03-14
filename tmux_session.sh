#!/bin/bash

# print help
dir=$(dirname "$0")
bash "$dir/./tmux_help.sh"

# TMUX setup: 3 tabs: bash, roscore, gazebo
session="ROS"
tmux new-session -d -s "$session"
# tmux rename-window -t 0 'Main'
tmux new-window -t $session:0 -n 'ROScore'
tmux send-keys -t $session:ROScore 'roscore' C-m

sleep 1 # give roscore some time to start up

tmux new-window -t $session:2 -n 'Main'
tmux send-keys -t $session:Main C-m 'clear' C-m 'roslaunch teeterbot_listener teeterbot_listener.launch' C-m
tmux split-pane -t $session:Main -p 50 
tmux send-keys -t $session:Main.2 'roslaunch teeterbot_controller teeterbot_controller.launch' C-m

tmux new-window -t $session:3 -n 'Gazebo'
tmux send-keys -t $session:Gazebo 'roslaunch teeterbot_gazebo teeterbot_empty_world.launch' C-m

tmux set -g mouse on

tmux attach-session -t $SESSION:2

