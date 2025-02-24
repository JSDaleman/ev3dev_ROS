cmake_minimum_required(VERSION 3.0.2)
project(ev3_launch_pkg)


find_package(catkin REQUIRED COMPONENTS
  controls
  gui
  mqtt
  ugv_description
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ev3_launch_pkg
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Instalar archivos launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
