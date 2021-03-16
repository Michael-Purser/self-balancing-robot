## Setup

1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/): `curl https://get.docker.com | sh && sudo systemctl --now enable docker`
2. Fix Docker permissions:
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker ${USER}
    su -s ${USER} # if this fails with 'authentication failure', log out and log back in (or reboot)
    ```
3. Test with `docker run hello-world`. This should print "Hello from docker! ... For more examples visit docs.docker.com"


### Optional: GPU acceleration

The simulator (Gazebo) can use GPUs to increase speed and reduce system load significantly. 
For our super simple empty world, this isn't really required though.
 
1. Install latest nvidia drivers
    * use `nvidia-smi` to find version of installed driver
    * go to "software and updates" -> "additional drivers" -> select nvidia-driver-460. You might need to restart.
2. install [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) for GPU access
3. Test with `sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi`. 
    This should print your GPU model, and the container's driver and CUDA version similar to running `nvidia-smi` on the host system

----
## Teeterbot: Docker quickstart

```
cd src/

# compile the container (it gives a few red messages, that's ok)
docker build -t self_balancing_robot_image --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) ./utils/docker/

# only if you have a pre-existing catkin workspace
`catkin clean -y`

# launch the container
bash utils/docker/launch.sh $(realpath ..)
```

In the container, you can start a pre-configured tmux session with Gazebo, Roscore etc with `bash tmux_session.sh`.  
You can get a summary of useful TMUX commands with `bash tmux_help.sh`.

In the container, you're the same user as on the host system.
---- 

## Explanation: Docker Basics
Docker image consists of read-only layers, each of which represents a Dockerfile instruction. 
The layers are stacked and each one is a delta of the changes from the previous layer.
Starting from some Docker image like a clean Ubuntu, we can add layers in a Dockerfile, then compile (`docker build`) to obtain a Docker image.
Then we can run this image; a running Docker image is called a _container_.

#### Basic Example

```Docker
FROM ubuntu:18.04
COPY . /app
RUN make /app
RUN apt-get update && apt-get install -y vim git
CMD python /app/app.py
```

Each instruction creates one layer:

    * FROM creates a layer from the ubuntu:18.04 Docker image.
    * COPY adds files from your Docker client’s current directory.
    * RUN executes a system command in the Docker. Is compiled into the image, so always
    * ENTRYPOINT/CMD:
        execute a cmd in the started container.
        * CMD: default arguments/cmd, can be overwritten when starting with `docker run $image $args`. If multiple CMD lines, only last one is considered.
        * ENTRYPOINT: like CMD, but always executed.
        So you should use ENTRYPOINT for calling executable and CMD for arguments that can be overwritten at container start.

#### Other layers

* COPY: copy from current directory into somewhere in the Docker image
* WORKDIR: set active directory inside the image. Next like in Dockerfile will run from this WORKDIR, and when the container starts terminal opens here.

---- 
### Starting/running the container

* Build/compile the Docker image: `docker build --tag <image_name> .`: compiles Dockerfile in current directory, and names it <image_name>. Don't forget the `.` at the end!
    * the `docker_entrypoint.sh` file is executed when the container starts, so has access to the host file system via mounts (see launch.sh). It will set up ROS/catkin workspace.

* Basic command: `docker run <image>`.
    To go into interactive terminal inside the container, use `-it` argument (interactive).

* start container in the background, return container id: `docker run -d` -> use example `docker_id=$(docker run -d -it | sed -n 2p)`
* starting multiple terminals connected to the same container: `docker exec -it <container> bash`
* attach terminal into running container: `docker attach ${docker_id}`

* mount local directory inside the container (very useful for developing):
    `docker run <image> -v <local_dir>:<container_dir>`

    * by default Docker uses a root user. I create a user with UID/GID of your host user, so files created from inside Docker have correct permissions. For details check `Dockerfile` and `launch.sh`.

* get access to host display inside container: `docker run -it --env="DISPLAY=$DISPLAY"` and some messy X stuff for permissions. See `launch.sh`

* get access to host network: `--net=host`

* get access to host GPU inside container: `docker run -it --gpus all --ipc=host`
    * You might need to install [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

* you should put all these things in a bash script to launch the container, like `launch.sh`. 
    * use an argument to the script to make it more flexible: `bash launch.sh <your_root_dir>`
    * if you know the path the user is in when calling the script, you can automatically specify the root dir. 
    Eg in our case user is in the `src/` folder: `bash utils/docker/launch.sh $(realpath ..)`
    * alternatively, you could use an environment variable  

---- 
### Tips 'n Tricks

* `docker image prune / docker prune`: reclaim storage space.

* Docker caches layers that don't change. However if one layers changes with 10 following layers that don't change, everything will be recompiled.
    So put layers you're likely to change often at the bottom

*  bash inside docker is by default ugly; I use sexy-bash-prompt, which has coloring and Git support etc: https://github.com/twolfson/sexy-bash-prompt

* it's nice to use TMUX to have multiple terminals inside Docker. 
    * Fancy setup + nice defaults: https://github.com/gpakosz/.tmux
    * see `tmux_session.sh`
  
* alternatively, in your container, you can install/start any program, like a terminal (terminator)

----
### References
https://docs.docker.com/develop/develop-images/dockerfile_best-practices/  
https://goinbigdata.com/docker-run-vs-cmd-vs-entrypoint/


