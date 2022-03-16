cmake_minimum_required(VERSION 3.0.2)
project(move_waypoint)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  move_base_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs move_base_msgs nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES move_waypoint
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs roscpp std_msgs tf2 tf2_ros nav_msgs
#  DEPENDS system_lib
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(move_waypoint src/move_waypoint.cpp src/move_waypoint_node.cpp)
target_link_libraries(move_waypoint ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
