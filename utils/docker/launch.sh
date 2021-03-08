#!/bin/bash

# Main docker launch script

source ~/.bashrc

# 1. start docker container
# capture id with sed (sed -n Xp gets the X'th line of output)
echo "launching container. This takes about 20s on first call b/c 'catkin clean' (see docker_entrypoint.sh)"
dockerid=$(bash _launch_container.sh | sed -n 1p)
echo "started container ${dockerid}"

# 2. start 3 terminals connected to the container
# TODO TMUXINATOR setup?
# docker exec -it "${dockerid}" /bin/bash -c "roscore"
# docker exec -it "${dockerid}" /bin/bash -c "roslaunch teeterbot_gazebo teeterbot_empty_world.launch"
# docker exec -it "${dockerid}" /bin/bash -c "roslaunch teeterbot_listener teeterbot_listener.launch"
