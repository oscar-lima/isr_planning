#!/usr/bin/env python
import rospy
import smach
        
# for send and receive event combined
#import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mbot_action_msgs.msg import ExplainAction
from mbot_action_msgs.msg import ExplainFeedback
from mbot_action_msgs.msg import ExplainResult

import mbot_states.common_states as cs

from std_msgs.msg import String

class getActionLibParams(smach.State):
    '''
    Get parameters for the action to be executed from action lib client
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],
                                   input_keys=['explain_goal'], 
                                   output_keys=['explain_feedback', 'explain_result', 'question'])

    def execute(self, userdata):
        userdata.question = userdata.explain_goal.question
        # giving feedback to the user
        feedback = ExplainFeedback()
        feedback.current_state = 'GET_ACTION_LIB_PARAMS'
        feedback.text='[explain] get question from action lib'
        userdata.explain_feedback = feedback
        return 'success'

class answerQuestion(smach.State):
    '''
    Get parameters for the action to be executed from action lib client
    '''
    def __init__(self, topic):
        smach.State.__init__(self, outcomes=['success'],
                                   input_keys=['explain_goal', 'question'], 
                                   output_keys=['explain_feedback', 'explain_result'])
        self.pub_question = rospy.Publisher(topic, String, queue_size=1)
        # give some time for the publisher to register in the network
        rospy.sleep(0.2)

    def execute(self, userdata):
        # HACK harcode some answers
        if userdata.question == 'hello robot':
            response = 'hello visitor, my name is mbot, i am a service robot at your service'
        elif userdata.question == 'how are you':
            response = 'i am a robot, i do not feel anything, but thanks for asking'
        # publish to a speech syntethizer component for the robot to speak
        self.pub_question.publish(String(response))
        # giving feedback to the user
        feedback = ExplainFeedback()
        feedback.current_state = 'ANSWERING QUESTION'
        feedback.text='[explain] anwering question by hardoding some fixed answers'
        userdata.explain_feedback = feedback
        return 'success'

def start_explain_server():
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED','OVERALL_PREEMPTED'],
            input_keys = ['explain_goal'],
            output_keys = ['explain_feedback', 'explain_result'])
    with sm:
        # get arguments from action lib
        smach.StateMachine.add('GET_ACTION_LIB_PARAMS', getActionLibParams(),
                transitions={'success': 'ANSWER_QUESTION'})
        
        # Answer the question of the human
        smach.StateMachine.add('ANSWER_QUESTION', answerQuestion('say'),
                transitions={'success': 'SET_ROBOT_FACE_TO_HAPPINNESS'})
        
        # set the robot's face to Happiness, the robot is happy to explain you something
        smach.StateMachine.add('SET_ROBOT_FACE_TO_HAPPINNESS', cs.PublishString('led_emotions_interface/emotions','Happiness'),
                transitions={'success': 'WAIT_UNTIL_QUESTON_IS_ANSWERED'})
        
        # wait for some time for the robot to answer the question
        smach.StateMachine.add('WAIT_UNTIL_QUESTON_IS_ANSWERED', cs.Sleep(7.0),
                transitions={'success': 'SET_ROBOT_FACE_TO_NEUTRAL'})
        
        # after answering the question set a neutral expression on the robot's face
        smach.StateMachine.add('SET_ROBOT_FACE_TO_NEUTRAL', cs.PublishString('led_emotions_interface/emotions','Neutral'),
                transitions={'success': 'SET_ACTION_LIB_SUCCESS'})
        
        # set the outcome of the server to success
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', cs.SetActionLibResult(True, ExplainResult, ExplainFeedback, ['explain_goal'], ['explain_feedback', 'explain_result']),
                               transitions={'success':'OVERALL_SUCCESS'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'explain_server', 
        action_spec = ExplainAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['OVERALL_PREEMPTED'],
        goal_key     = 'explain_goal',
        feedback_key = 'explain_feedback',
        result_key   = 'explain_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

    
def main():
    rospy.init_node('explain_server', anonymous=False)
    start_explain_server()
