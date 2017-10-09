#! /usr/bin/env python
import rospy
import roslib
import actionlib

from mbot_action_msgs.msg import ExplainAction, ExplainGoal

def start_explain_client_test():
    client = actionlib.SimpleActionClient('explain_server', ExplainAction)
    client.wait_for_server()
    goal = ExplainGoal()
    goal.question = "hello robot"
    timeout = 60.0
    rospy.loginfo('Sending action lib goal to explain_server, question : ' + goal.question)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    if client.get_result():
        rospy.loginfo("Server responded with success")
    else:
        rospy.logwarn("Server responded with error")
        
def main():
    rospy.init_node('explain_client_tester', anonymous=False)
    start_explain_client_test()
