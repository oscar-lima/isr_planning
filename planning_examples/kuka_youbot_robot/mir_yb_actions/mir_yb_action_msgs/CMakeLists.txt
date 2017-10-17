cmake_minimum_required(VERSION 2.8.3)
project(mir_yb_action_msgs)

find_package(catkin REQUIRED 
    genmsg 
    actionlib_msgs 
    actionlib
    geometry_msgs
)

add_action_files(DIRECTORY action FILES 
    MoveBaseSafe.action
    StageObject.action
    UnStageObject.action
    InsertObject.action
    PickObject.action
    PickObjectWBC.action
    PerceiveLocation.action
    GetPlate.action
    PlaceObject.action
    PickPlate.action
    DrillPlate.action
    ComputeBaseShift.action
    AlignWithObject.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package()
