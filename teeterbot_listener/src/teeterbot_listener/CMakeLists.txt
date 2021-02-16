file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

add_executable(
  ${PROJECT_NAME}
  ${SOURCES}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
