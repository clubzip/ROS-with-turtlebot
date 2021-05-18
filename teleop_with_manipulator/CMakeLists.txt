cmake_minimum_required(VERSION 3.0.2)
project(teleop_with_manipulator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teleop_with_manipulator
  CATKIN_DEPENDS geometry_msgs moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
