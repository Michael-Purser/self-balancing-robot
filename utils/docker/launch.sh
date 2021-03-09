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
    chmod a+r $XAUTH
fi

if [[ $WORKSPACE == "" ]]
then
    echo "You need to run 'export WORKSPACE=<self_balancing_robot_root>!"
    exit
fi

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
    --volume="${WORKSPACE}:/home/user/self_balancing_robot:rw" \
    self_balancing_robot_image \
    bash
