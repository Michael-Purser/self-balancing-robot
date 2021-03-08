#!/bin/bash
set -e

# mkdir -p /self_balancing_robot/src
# git clone git@github.com:Michael-Purser/self_balancing_robot.git /teeterbot/src
# git submodule update --init --recursive

# If not using git above, you need to mount project dir in host at /self_balancing_robot for the following to work
# `docker run --it <self_balancing_robot_img> -v <host_self_balancing_robot_dir>:/self_balancing_robot`
cd /self_balancing_robot
# catkin clean -y  # TODO need to run one time to fix paths to docker; afterwards can commend for much faster launch
catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin init && catkin build

echo "source /self_balancing_robot/devel/setup.bash" >> /root/.bashrc
echo "source /self_balancing_robot/src/utils/scripts/setup.bash" >> /root/.bashrc
source /root/.bashrc

# # TODO need next line for rosdep init to prevent crashing on 'already exist' error?
# rm /etc/ros/rosdep/sources.list.d/20-default.list
# rosdep init -y && rosdep update 
# rosdep install --from-paths src --ignore-src -y

exec "$@"