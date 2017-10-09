#! /usr/bin/env python
import rospy
import roslib
import actionlib

from mbot_action_msgs.msg import PlaceAction, PlaceGoal

def start_explain_client_test():
    client = actionlib.SimpleActionClient('place_server', PlaceAction)
    client.wait_for_server()
    rospy.loginfo('server is up !')
    goal = PlaceGoal()
    goal.shelf = "folded"
    timeout = 60.0
   
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    if client.get_result():
        rospy.loginfo("Server responded with success")
    else:
        rospy.loginfo("Server responded with error")
        
def main():
    rospy.init_node('place_client_tester', anonymous=False)
    start_explain_client_test()
