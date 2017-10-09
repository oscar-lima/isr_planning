#!/usr/bin/env python
import rospy
import smach

# action lib
from smach_ros import ActionServerWrapper
from mbot_action_msgs.msg import AskQuestionAction, AskQuestionFeedback, AskQuestionResult

import mbot_states.common_states as cs

from std_msgs.msg import String
import os

class answerQuestion(smach.State):
    '''
    Get parameters for the action to be executed from action lib client
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['positive_response', 'negative_response','timeout'],
                                   input_keys=['ask_question_goal'], 
                                   output_keys=['ask_question_feedback', 'ask_question_result'])
        # subscribe speech recognition topic
        rospy.Subscriber("recognized_speech", String, self.speechCallBack)
        # init variables
        self.speech_received = None
        self.recognized_speech = None
        # give some time for subscribers to register in the network
        rospy.sleep(0.1)

    def speechCallBack(self, msg):
        rospy.loginfo('speech received!!')
        self.speech_received = True
        self.recognized_speech = msg.data

    def execute(self, userdata):
        # giving feedback to the user
        feedback = AskQuestionFeedback()
        feedback.current_state = 'GET_ACTION_LIB_PARAMS_AND_ASK_QUESTION'
        feedback.text='[ask_question] get question from action lib and ask it to the user through robot speakers'
        userdata.ask_question_feedback = feedback
        # ask the question to the user
        speech = userdata.ask_question_goal.question
        os.system('espeak -s 120 -v en-us "' + speech + '"')
        # wait for speech
        start_time = rospy.Time.now()
        timeout = rospy.Duration.from_sec(userdata.ask_question_goal.timeout)
        result = AskQuestionResult() # prepare result to be filled later
        self.speech_received = False
        while(rospy.Time.now() - start_time < timeout):
            # sleep a small amount of time to allow to listen to callbacks
            rospy.sleep(0.01)
            if self.speech_received:
                break

        if not self.speech_received:
            result.response = False
            result.timeout = True
            result.recognized_speech = ""
            userdata.ask_question_result = result
            # no speech input was received within timeout
            rospy.loginfo('no response received within timeout')
            speech = userdata.ask_question_goal.timeout_speech
            os.system('espeak -s 120 -v en-us "' + speech + '"')
            # reset flag for next to be ready for future calls
            self.speech_received = False
            return 'timeout'
        #speech received!
        rospy.loginfo('response from the user was received!')
        # old code that takes a vector of possible success reponses (uncomment after lucia summer school to increase flexibility)
        #for each_posible_positive_responses in userdata.ask_question_goal.success_responses:
            #if each_posible_positive_responses == self.recognized_speech:
                ## user response is yes
                #rospy.loginfo('user responded : Yes!')
                #speech = 'your answer was yes'
                #os.system('espeak -s 120 -v en-us "' + speech + '"')
                ## fill result msg
                #result.response = True
                #result.timeout = False
                #result.recognized_speech = self.recognized_speech
                #userdata.ask_question_result = result
                ## reset flag for next to be ready for future calls
                #self.speech_received = False
                #return 'positive_response'
        # process received speech
        if userdata.ask_question_goal.success_response == self.recognized_speech:
            speech = userdata.ask_question_goal.positive_speech
            os.system('espeak -s 120 -v en-us "' + speech + '"')
            ## fill result msg
            result.response = True
            result.timeout = False
            result.recognized_speech = self.recognized_speech
            userdata.ask_question_result = result
            return 'positive_response'
        # user response is no
        rospy.loginfo('user responded : No!')
        speech = userdata.ask_question_goal.negative_speech
        os.system('espeak -s 120 -v en-us "' + speech + '"')
        result.response = False
        result.timeout = False
        result.recognized_speech = self.recognized_speech
        userdata.ask_question_result = result
        # reset flag for next to be ready for future calls
        self.speech_received = False
        return 'negative_response'

def start_ask_question_server():
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED','OVERALL_PREEMPTED'],
            input_keys = ['ask_question_goal'],
            output_keys = ['ask_question_feedback', 'ask_question_result'])
    with sm:
        # get arguments from action lib, ask question to user and get response from speech recognition
        sm.add('ANSWER_QUESTION', answerQuestion(),
                transitions={'positive_response': 'SET_ROBOT_FACE_TO_HAPPINNESS',
                             'negative_response':'SET_ROBOT_FACE_TO_ANGRY',
                             'timeout':'OVERALL_SUCCESS'})

        ## set the robot's face to Happiness, the robot is happy because user said yes!
        smach.StateMachine.add('SET_ROBOT_FACE_TO_HAPPINNESS', cs.PublishString('led_emotions_interface/emotions','Happiness'),
                transitions={'success': 'OVERALL_SUCCESS'})
        
        ## set the robot's face to Angry, the robot is happy because user said yes!
        smach.StateMachine.add('SET_ROBOT_FACE_TO_ANGRY', cs.PublishString('led_emotions_interface/emotions','Anger'),
                transitions={'success': 'OVERALL_SUCCESS'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'ask_question_server', 
        action_spec = AskQuestionAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['OVERALL_PREEMPTED'],
        goal_key     = 'ask_question_goal',
        feedback_key = 'ask_question_feedback',
        result_key   = 'ask_question_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

    
def main():
    rospy.init_node('ask_question_server', anonymous=False)
    start_ask_question_server()
