cmake_minimum_required(VERSION 2.8.3)
project(mbot_action_msgs)

find_package(catkin REQUIRED 
    genmsg 
    actionlib_msgs 
    actionlib
    geometry_msgs
)

add_action_files(DIRECTORY action FILES 
    MoveBaseSafe.action
    Explain.action
    PickObject.action
    PerceiveLocation.action
    Follow.action
    Place.action
    AskQuestion.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package()
