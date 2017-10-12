#! /usr/bin/env python
import rospy
import roslib
import actionlib

from mir_yb_action_msgs.msg import UnStageObjectAction, UnStageObjectGoal

if __name__ == '__main__':
    rospy.init_node('unstage_object_client_tester')
    client = actionlib.SimpleActionClient('unstage_object_server', UnStageObjectAction)
    client.wait_for_server()
    goal = UnStageObjectGoal()
    #goal.robot_platform = "platform_middle"
    #goal.robot_platform = "platform_left"
    goal.robot_platform = "platform_right"
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
    print client.get_result()
