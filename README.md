## System Requirements

We provide a Docker container for convenience (see utils/docker/); the below requirements are if you want to install on your system directly.

 - Ubuntu 18.04 LTS.
 - [ROS Melodic](https://wiki.ros.org/melodic).
   - Install `ros-melodic-desktop-full` to also install the necessary ROS packages to communicate with Gazebo.
 - [Gazebo](http://www.gazebosim.org/tutorials?tut=install_ubuntu&cat=install) 9.0.
   - Follow the step-by-step installation guide, and install both `gazebo9` and `libgazebo9-dev`.
 - [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) 0.6.1.
 - [GCC](https://gcc.gnu.org/releases.html) 10.1.<br>
   A guide to updating GCC/G++ on linux systems can be found [here](https://azrael.digipen.edu/~mmead/www/mg/update-compilers/index.html).
 - [CMake](https://cmake.org/) 3.10.2.

Gazebo takes a long time to shutdown. For faster iteration, edit
`sudo edit /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py` and change TIMEOUTs in lines 57/58.

## Developer Setup

If using the Docker container, you can use Git inside the container. However you'll need to perform the initial pull on your host system.

1. **Choose a local workspace location**<br>
   Choose a location on your machine where to clone the repository.<br>
   This location will be called `<DIR>` in this guide.

2. **Clone the repository**<br>
   ```
   $ cd <DIR>
   $ mkdir -p ./self_balancing_robot/src/
   $ git clone git@github.com:Michael-Purser/self_balancing_robot.git ./self_balancing_robot/src/
   ```

   If you cloned the code into a directory normally owned by root, you need to set the permissions for that directory (if you cloned the code into a directory owned by the local user, you can ignore this step):
   ```
   $ sudo chmod 755 <DIR>/self_balancing_robot
   $ sudo chown $USER <DIR>/self_balancing_robot
   $ sudo chown :$USER <DIR>/self_balancing_robot
   ```

3. **Initialize, fetch and checkout the git submodules**<br>
   ```
   $ cd <DIR>/self_balancing_robot/src
   $ git submodule update --init --recursive
   ```

4. Now, you can choose to  use Docker, or set up your host system.
   For Docker, see `utils/docker/README.md`, and skip to `Running the code` below.
   For the host setup, continue with steps 5-7.

5. **Initialize and build the workspace**<br>
   Execute the commands below to configure the catkin workspace:
   ```
   $ cd <DIR>/self_balancing_robot
   $ catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```
   Explanation of the options:
    - `--extend /opt/ros/melodic`:
      - Extends the workspace to include the ROS melodic workspace.
    - `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`:
      - Builds the workspace in Release mode with Debug symbols (instead of the default Debug mode). See [here](https://cmake.org/pipermail/cmake/2001-October/002479.html) for information about the `RelDebWithInfo` mode.

   Now initialize the workspace and build it:
   ```
   $ catkin init
   $ catkin build
   ```
   The build should succeed and create the `build/`, `devel/` and `logs/` directories within `<DIR>/self_balancing_robot/`.

6. **Automatically source the setup files**<br>
   In your `~/.bashrc` file, add the following lines.<br>
   *These must always be the bottom-most lines in your .bashrc file!*
   ```
   source <DIR>/self_balancing_robot/devel/setup.bash
   source <DIR>/self_balancing_robot/src/utils/scripts/setup.bash
   ```

   Verify the setup files are sourced and no errors occur by closing the current terminal, opening a new terminal and executing:
   ```
   $ echo $ROS_WORKSPACE
   ```
   This should return `<DIR>/self_balancing_robot/src`.

   You can now change to the project source directory by using the alias `roscd`.

7. **Install the package dependencies**<br>
   Package dependencies are not automatically installed when you clone the repository.

   In case you have not initialized rosdep yet, please run:
   ```
   sudo rosdep init
   ```

   To install the dependencies, open a new terminal or resource your ~/.bashrc file, then enter the following commands:
   ```
   cd <DIR>/self_balancing_robot/
   rosdep update
   rosdep install --from-paths src --ignore-src -y
   ```

## Running the code

1. Start a ROS master:
   ```
   $ roscore
   ```

2. In another terminal, start the teeterbot simulation with Gazebo:
   ```
   $ roslaunch teeterbot_gazebo teeterbot_empty_world.launch
   ```

3. In other terminals start your own nodes, for example a motion controller:
   ```
   $ roslaunch <your_node> <your_node>.launch
   ```
