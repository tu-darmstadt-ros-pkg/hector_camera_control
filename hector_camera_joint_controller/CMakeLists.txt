cmake_minimum_required(VERSION 2.8.3)
project(hector_camera_joint_controller)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  roscpp
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_camera_joint_controller
#  CATKIN_DEPENDS control_msgs geometry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


