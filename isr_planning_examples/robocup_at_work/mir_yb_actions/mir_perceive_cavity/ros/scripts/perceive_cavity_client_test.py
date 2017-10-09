#! /usr/bin/env python
import rospy
import roslib
import actionlib

from mir_yb_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal

if __name__ == '__main__':
    rospy.init_node('perceive_cavity_client_tester')
    client = actionlib.SimpleActionClient('perceive_cavity_server', PerceiveLocationAction)
    client.wait_for_server()
    goal = PerceiveLocationGoal()
    goal.location = "sh-03"
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
    print client.get_result()
