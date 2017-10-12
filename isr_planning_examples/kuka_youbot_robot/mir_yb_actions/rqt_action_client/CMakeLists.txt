cmake_minimum_required(VERSION 2.8.3)
project(rqt_action_client)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui_py
)
catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/rqt_action_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
