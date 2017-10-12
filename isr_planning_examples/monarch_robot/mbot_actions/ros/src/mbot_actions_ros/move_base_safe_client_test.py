#! /usr/bin/env python
import rospy
import roslib
import actionlib

from mbot_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal

def start_explain_client_test():
    client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
    client.wait_for_server()
    rospy.loginfo('server is up !')
    goal = MoveBaseSafeGoal()
    goal.arm_safe_position = "folded"
    goal.source_location = "start"
    #goal.destination_location = "testbed"
    goal.destination_location = "BEDROOM"
    goal.destination_orientation = ""
    goal.use_destination_pose = False
    timeout = 500.0 #Timeout handled by the server
    rospy.loginfo('Sending action lib goal to move_base_safe_server, destination : ' + goal.destination_location)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    if client.get_result().success:
        rospy.loginfo("Server responded with success")
    else:
        rospy.loginfo("Server responded with error")
        
def main():
    rospy.init_node('move_base_safe_client_tester', anonymous=False)
    start_explain_client_test()
