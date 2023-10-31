cmake_minimum_required(VERSION 3.0.2)
project(your_keyboard_control_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp mavros_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_control_node src/keyboard_control_node.cpp)

target_link_libraries(keyboard_control_node
  ${catkin_LIBRARIES}
)

install(TARGETS keyboard_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
