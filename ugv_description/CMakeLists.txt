cmake_minimum_required(VERSION 3.0.2)
project(ugv_description)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ugv_description
  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

