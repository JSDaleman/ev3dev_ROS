cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED
    rospy
    std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controls
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

