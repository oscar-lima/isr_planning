cmake_minimum_required(VERSION 2.8.3)
project(mir_planner_executor)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)