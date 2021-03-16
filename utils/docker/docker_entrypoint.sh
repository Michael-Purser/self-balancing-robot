#!/bin/bash
set -e

# files mounted from host system, so need to source here instead of in Dockerfile
echo "source $HOME/self_balancing_robot/devel/setup.bash" >> $HOME/.bashrc
echo "source $HOME/self_balancing_robot/src/utils/scripts/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

cd $HOME/self_balancing_robot/
catkin build

clear

cd $HOME/self_balancing_robot/src
echo "to start pre-configured tmux session set up for ROS, Gazebo and our current software, use 'bash tmux_session.sh'"
echo "for a TMUX quick summary, use 'bash tmux_help.sh'"
echo "alternatively, you can start any terminal (like terminator) via terminal and run commands manually"
exec "$@" 
