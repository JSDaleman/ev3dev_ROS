cmake_minimum_required(VERSION 3.0.2)
project(gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rqt_gui
  rqt_gui_cpp
  rqt_gui_py
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gui
#  CATKIN_DEPENDS mqtt ugv_description
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/app.py
   scripts/rqt_app.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  resource/rqt_control_gui_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

