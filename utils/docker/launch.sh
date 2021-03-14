#!/bin/bash

# required for access to display using X
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    sudo chmod a+r $XAUTH
fi

if [[ $1 == "" ]]
then
    echo "You need to pass the workspace directory as argument (parent dir of src/)!"
    exit
fi

# attach display, audio, GPUs, network, fix permissions and mount data
docker run -it --rm \
    --name="teeterbot_dev_container" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --user $(id -u):$(id -g) \
    --device /dev/snd \
    --ipc=host --gpus=all \
    --privileged --net=host \
    --volume="$1:/home/user/self_balancing_robot:rw" \
    --volume="$HOME/.gitconfig:/home/user/.gitconfig" \
    --volume="$HOME/.ssh/:/home/user/.ssh/" \
    --volume="$HOME/.bash_aliases:/home/user/.bash_aliases" \
    self_balancing_robot_image \
    bash