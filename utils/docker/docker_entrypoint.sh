#!/bin/bash
set -e

cd $HOME/self_balancing_robot

DOCKER_SETUP_DONE="$HOME/self_balancing_robot/.docker_setup_done"
if [ ! -f "${DOCKER_SETUP_DONE}" ]; then
    catkin clean -y
    touch ${DOCKER_SETUP_DONE}
fi

catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin init && catkin build

echo "source $HOME/self_balancing_robot/devel/setup.bash" >> $HOME/.bashrc
echo "source $HOME/self_balancing_robot/src/utils/scripts/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

clear
echo "to start pre-configured tmux session with 3 terminals set up for ROS, use 'bash tmux_session.sh'"
echo "alternatively, you can start any terminal (like terminator) and run commands manually"
exec "$@" 
