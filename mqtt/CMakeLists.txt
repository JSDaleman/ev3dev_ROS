cmake_minimum_required(VERSION 3.0.2)
project(mqtt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mqtt
#   CATKIN_DEPENDS ugv_description
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/mqtt_messenger.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
