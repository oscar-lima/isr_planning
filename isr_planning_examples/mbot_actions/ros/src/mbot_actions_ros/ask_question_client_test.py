#! /usr/bin/env python
import rospy
import roslib
import actionlib

from mbot_action_msgs.msg import AskQuestionAction, AskQuestionGoal

def start_ask_question_client_test():
    client = actionlib.SimpleActionClient('ask_question_server', AskQuestionAction)
    client.wait_for_server()
    goal = AskQuestionGoal()
    goal.question = 'would you like a coffee' # the question that the robot will ask to the user
    goal.success_response = 'ok' # if the user speaks any of this, the robot will interpret as a ok!
    goal.timeout = 8.0 # the time in seconds that the robot will wait for response
    
    goal.positive_speech = 'your answer was yes'
    goal.negative_speech = 'your answer was no'
    goal.timeout_speech = 'it looks like no one is here'
    
    timeout = 60.0
    rospy.loginfo('Sending action lib goal to ask_question_server, question : ' + goal.question)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
    print client.get_result()
        
def main():
    rospy.init_node('ask_question_client_tester', anonymous=False)
    start_ask_question_client_test()
