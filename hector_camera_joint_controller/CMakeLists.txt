cmake_minimum_required(VERSION 2.8.3)
project(hector_camera_joint_controller)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  roscpp
  tf
  cmake_modules
  actionlib
  moveit_msgs
  hector_perception_msgs
)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS} include)
add_definitions(${EIGEN_DEFINITIONS})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_camera_joint_controller
#  CATKIN_DEPENDS control_msgs geometry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(cam_joint_trajectory_controller src/cam_joint_trajectory_controller.cpp)

target_link_libraries(cam_joint_trajectory_controller
  ${catkin_LIBRARIES}
)


