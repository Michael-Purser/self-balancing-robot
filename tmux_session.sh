#!/bin/bash

echo """
TMUX basic commands: see https://github.com/gpakosz/.tmux
- commands require a <prefix>, then a key: Ctrl+a = <prefix>
- a 'window' is like a tab.
- a 'pane' is a subdivision of current terminal, like tiling
You can use keybindings below or click with mouse to select active pane/window, resize etc

Windows
  <prefix> C-h and <prefix> C-l let you navigate windows
  <prefix> Tab brings you to the last active window

Panes
  <prefix> - splits the current pane vertically
  <prefix> _ splits the current pane horizontally
  <prefix> h, <prefix> j, <prefix> k and <prefix> l let you navigate panes ala Vim
  <prefix> H, <prefix> J, <prefix> K, <prefix> L let you resize panes
  <prefix> < and <prefix> > let you swap panes
  <prefix> + maximizes the current pane to a new window

Other
  if you shutdown the terminal, the session keeps running (like screen). Reattach with 'tmux list-sessions' and 'tmux attach-session -t <session_name>'
"""

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

