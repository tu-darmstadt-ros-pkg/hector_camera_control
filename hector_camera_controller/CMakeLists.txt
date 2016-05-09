cmake_minimum_required(VERSION 2.8.3)
project(hector_camera_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  monstertruck_msgs
  sensor_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_camera_controller
#  CATKIN_DEPENDS geometry_msgs monstertruck_msgs sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_control src/camera_control.cpp)

target_link_libraries(camera_control
  ${catkin_LIBRARIES}
)

