## System Requirements

This repository is developed and tested with the following system requirements and versions.
There are no guarantees that this will work on another system configuration.
 - Ubuntu 18.04 LTS
 - [ROS Melodic](https://wiki.ros.org/melodic).
   - Install `ros-melodic-desktop-full` to also install the necessary ROS packages to communicate with Gazebo.
 - [Gazebo 9.0 or later](http://www.gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
   - Follow the step-by-step installation guide, and install both `gazebo9` and `libgazebo9-dev`.
 - [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).


## Developer Setup

1. Choose a location on your machine where to clone the repository.<br>
   This location will be called `<DIR>` in this guide.

2. Clone the repository to your local machine:
   ```
   $ cd <DIR>
   $ mkdir -p ./self_balancing_robot/src/
   $ git clone git@github.com:Michael-Purser/self_balancing_robot.git ./self_balancing_robot/src/
   ```

3. Set permissions for the workspace.<br>
   *If you cloned the code into a directory normally owned by root, you need to perform this step. If you cloned the code into a directory owned by the local user, you can ignore this step.*
   ```
   $ sudo chmod 755 <DIR>/self_balancing_robot
   $ sudo chown $USER <DIR>/self_balancing_robot
   $ sudo chown :$USER <DIR>/self_balancing_robot
   ```

4. Initialize and build the workspace:
   ```
   $ cd <DIR>/self_balancing_robot/src
   $ catkin init
   $ catkin build
   ```
   The build should succeed and create the `build/`, `devel/` and `logs/` directories within `<DIR>/self_balancing_robot/`.

5. In your `~/.bashrc` file, add the following lines
   (**these must always be the bottom-most lines in your .bashrc file**):
   ```
   source <DIR>/self_balancing_robot/devel/setup.bash
   source <DIR>/self_balancing_robot/src/utils/scripts/setup.bash
   ```

6. Close the current terminal, open a new terminal and make sure `ROS_WORKSPACE` has been set:
   ```
   $ echo $ROS_WORKSPACE
   ```
   This should return `<DIR>/self_balancing_robot/src`
   You can now change to the project source directory by using the alias `roscd`.

7. Initialize, fetch and checkout the git submodules:
   ```
   $ cd <DIR>/self_balancing_robot/src           # Note: you can now use also roscd for this
   $ git submodule update --init --recursive
   ```
