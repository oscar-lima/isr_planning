cmake_minimum_required(VERSION 2.8.3)
project(mbot_planner_executor)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)
