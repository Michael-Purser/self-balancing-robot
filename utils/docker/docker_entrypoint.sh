#!/bin/bash
set -e

cd $HOME/self_balancing_robot

catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin init && catkin build

echo "source $HOME/self_balancing_robot/devel/setup.bash" >> $HOME/.bashrc
echo "source $HOME/self_balancing_robot/src/utils/scripts/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

clear
echo "to start pre-configured tmux session with 3 terminals set up for ROS, use 'bash src/tmux_session.sh'"
echo "for a TMUX quick summary, use 'bash src/tmux_help.sh'"
echo "alternatively, you can start any terminal (like terminator) via terminal and run commands manually"
exec "$@" 
