cmake_minimum_required(VERSION 3.0.2)
project(teeterbot_listener)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_CATKIN_DEPENDENCIES
    roscpp
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
