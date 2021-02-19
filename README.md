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

3. **Initialize and build the workspace**<br>
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

4. **Automatically source the setup files**<br>
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

5. **Initialize, fetch and checkout the git submodules**<br>
   ```
   $ cd <DIR>/self_balancing_robot/src           # Note: you can now use also roscd for this
   $ git submodule update --init --recursive
   ```
