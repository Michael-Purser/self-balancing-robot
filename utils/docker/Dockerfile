FROM osrf/ros:melodic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# For correct permissions, build with `docker build -t <image_name> --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) .`
ARG USER_ID
ARG GROUP_ID

RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim git gcc g++ cmake wget less tmux dbus-x11 terminator openssh-client
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && apt-get install -y python-catkin-tools
RUN apt-get update && apt-get install -q -y ros-melodic-hector-gazebo-plugins

RUN source /opt/ros/melodic/setup.bash

# quick Gazebo shutdown
RUN sed -i 's/_TIMEOUT_SIGINT  = 15.0/_TIMEOUT_SIGINT  = 0.5/' /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
RUN sed -i 's/_TIMEOUT_SIGTERM = 15.0/_TIMEOUT_SIGTERM = 0.5/' /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py

# if we still want root access (eg apt-get install something), use `su` command and 'abc123' as root password
# disable b/c popential security risk? Though can only modify files mounted in Docker...
# RUN echo 'root:abc123' | chpasswd

# create new user that matches the host user
RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user

# create a base directory we'll work from
RUN mkdir -p /home/user/self_balancing_robot
RUN chown -R $USER_ID:$GROUP_ID /home/user/self_balancing_robot
ENV WS=/home/user/self_balancing_robot/
WORKDIR /home/user/self_balancing_robot/

USER user

# nice bash and tmux prompt, w/ colors and Git support
RUN cd $HOME && git clone https://github.com/gpakosz/.tmux.git && ln -s -f .tmux/.tmux.conf && cp .tmux/.tmux.conf.local .
RUN (cd $HOME && ([[ -d sexy-bash-prompt ]] || git clone --depth 1 --config core.autocrlf=false https://github.com/twolfson/sexy-bash-prompt) && cd sexy-bash-prompt && make install) && source ~/.bashrc

COPY ./docker_entrypoint.sh /tmp/
ENTRYPOINT ["/bin/bash", "/tmp/docker_entrypoint.sh"]
CMD ["bash"]
