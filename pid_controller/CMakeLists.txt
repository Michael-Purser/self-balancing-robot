cmake_minimum_required(VERSION 3.10.2)
project(pid_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PROJECT_CATKIN_DEPENDENCIES
    roscpp
    sensor_msgs
    tf
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PROJECT_CATKIN_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PROJECT_CATKIN_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/${PROJECT_NAME})

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
