cmake_minimum_required(VERSION 2.8.3)
project(mcr_cost_generators)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   roslint
   std_msgs
   geometry_msgs
   nav_msgs
   tf
)

catkin_package(
    CATKIN_DEPENDS 
    std_msgs
)

include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES
add_executable(navigation_cost_generator_node ros/src/navigation_cost_generator_node.cpp)
target_link_libraries(navigation_cost_generator_node ${catkin_LIBRARIES})

add_executable(perception_cost_generator_node ros/src/perception_cost_generator_node.cpp)
target_link_libraries(perception_cost_generator_node ${catkin_LIBRARIES})

### TESTS
if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(ros/launch)
endif()

roslint_cpp()

### INSTALLS

install(TARGETS navigation_cost_generator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS perception_cost_generator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
