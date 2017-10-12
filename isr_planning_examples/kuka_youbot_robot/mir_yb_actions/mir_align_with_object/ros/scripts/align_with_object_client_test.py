#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mir_yb_action_msgs.msg import AlignWithObjectAction, AlignWithObjectGoal

if __name__ == '__main__':
    rospy.init_node('align_with_object_client')
    client = actionlib.SimpleActionClient('align_with_object_server', AlignWithObjectAction)
    client.wait_for_server()
    goal = AlignWithObjectGoal()
    goal.object = str(sys.argv[1])
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(50.0))
    print client.get_result()
