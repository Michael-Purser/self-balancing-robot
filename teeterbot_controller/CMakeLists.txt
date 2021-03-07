cmake_minimum_required(VERSION 3.0.2)
project(teeterbot_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(PROJECT_CATKIN_DEPENDENCIES
    roscpp
    sensor_msgs
    tf
)

find_package(catkin REQUIRED COMPONENTS ${PROJECT_CATKIN_DEPENDENCIES})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES teeterbot_controller
 CATKIN_DEPENDS ${PROJECT_CATKIN_DEPENDENCIES}
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/teeterbot_controller.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
