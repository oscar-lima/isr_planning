cmake_minimum_required(VERSION 2.8.3)
project(mbot_actions)

find_package(catkin REQUIRED COMPONENTS
    rospy
    mbot_action_msgs
)

catkin_package( CATKIN_DEPENDS mbot_action_msgs )
catkin_python_setup()
