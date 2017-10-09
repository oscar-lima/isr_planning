#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mir_yb_action_msgs.msg import ComputeBaseShiftAction, ComputeBaseShiftGoal

if __name__ == '__main__':
    rospy.init_node('compute_base_shift_client_tester')
    client = actionlib.SimpleActionClient('compute_base_shift_server', ComputeBaseShiftAction)
    client.wait_for_server()
    goal = ComputeBaseShiftGoal()
    goal.source_location = 'location'
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(50.0))
    print client.get_result()