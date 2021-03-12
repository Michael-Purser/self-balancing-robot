cmake_minimum_required(VERSION 3.1)
project(self_balancing_robot_msgs)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_CATKIN_DEPS
  std_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PROJECT_CATKIN_DEPS}
    message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
    ControlError.msg
    ControllerStatus.msg
    PIDControllerInfo.msg
    PIDControlSignals.msg
)

generate_messages(
  DEPENDENCIES
    ${PROJECT_CATKIN_DEPS}
)

catkin_package(
  CATKIN_DEPENDS
    ${PROJECT_CATKIN_DEPS}
    message_runtime
)
