#! /usr/bin/env python
import rospy
import roslib
import actionlib

from mir_yb_action_msgs.msg import StageObjectAction, StageObjectGoal

if __name__ == '__main__':
    rospy.init_node('stage_object_client_tester')
    client = actionlib.SimpleActionClient('stage_object_server', StageObjectAction)
    client.wait_for_server()
    goal = StageObjectGoal()
    #goal.robot_platform = "platform_middle"
    #goal.robot_platform = "platform_left"
    goal.robot_platform = "platform_right"
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
    print client.get_result()
