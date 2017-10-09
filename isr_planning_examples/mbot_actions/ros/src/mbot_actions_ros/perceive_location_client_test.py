#! /usr/bin/env python
import rospy
import actionlib

from mbot_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal, PerceiveLocationResult


def start_perceive_location_client_test():
    client = actionlib.SimpleActionClient('perceive_location_server', PerceiveLocationAction)
    client.wait_for_server()
    goal = PerceiveLocationGoal()
    goal.location = "dummy_location"
    rospy.set_param('/world_model/' + goal.location + '/height', 0.4)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
    result = client.get_result()

    if result.success:
        rospy.loginfo('Objects detected')
    else:
        rospy.loginfo('No objects found')

    return


def main():
    rospy.init_node('perceive_location_client_tester', anonymous=False)
    start_perceive_location_client_test()
