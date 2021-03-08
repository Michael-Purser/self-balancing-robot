FROM osrf/ros:melodic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim git gcc g++ cmake wget less tmux
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# # catkin tools
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y python-catkin-tools
RUN apt-get update && apt-get install -q -y ros-melodic-hector-gazebo-plugins

ENV WS=/self_balancing_robot/

RUN source /opt/ros/melodic/setup.bash

EXPOSE 11345

COPY ./docker_entrypoint.sh /

# default command that's run when Docker container is started
ENTRYPOINT ["/bin/bash", "/docker_entrypoint.sh"]

CMD ["bash"]