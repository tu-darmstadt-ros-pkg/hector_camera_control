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
  moveit_core
  moveit_ros_planning
)

find_package(Eigen3 REQUIRED)
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

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS cam_joint_trajectory_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

