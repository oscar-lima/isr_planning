#!/usr/bin/python
import sys
import rospy
import smach
import smach_ros

from std_msgs.msg import String
from mcr_manipulation_msgs.msg import GripperCommand  

import mbot_states.manipulation_states as gms
import mcr_states.common.basic_states as gbs
import mbot_states.common_states as cs

# action lib
from smach_ros import ActionServerWrapper
from mbot_action_msgs.msg import PlaceAction, PlaceFeedback, PlaceResult
        
#==============================================================================
        
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['place_goal'], 
                                    output_keys=['place_feedback', 'place_result'])
        self.result = result

    def execute(self, userdata):
        result = PlaceResult()
        result.success = self.result
        userdata.place_result = result
        return 'succeeded'

#===============================================================================

class Speak(smach.State):
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

#===============================================================================

def main():
    # Open the container
    rospy.init_node('place_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['place_goal'],
            output_keys = ['place_feedback', 'place_result'])
    with sm:
        # move arm to place position
        smach.StateMachine.add('SPEAK', cs.PublishString('say','please stand in front of me at two meters to start following'),
                               transitions={'succeeded':'CALL_SERVICE_TO_START','failed':'MOVE_ARM_TO_PREPLACE'})

        # call service
        smach.StateMachine.add('CALL_SERVICE_TO_START', call_service(True),
            transitions={'succeeded':'WAIT_FOR_STOP_SIGNAL'})
        
        # trigger object selector, to publish object pose
            smach.StateMachine.add('WAIT_FOR_STOP_SIGNAL', gbs.send_and_wait_events_combined(
                event_in_list=[('/dummy_topic','bla_bla')],
                event_out_list=[('/recognized_speech','here is the car', True)],
                timeout_duration=1.0),
                transitions={'success':'RECONFIGURE_PREGRASP_PARAMS',
                             'timeout':'SET_ROBOT_FACE_TO_ANGER',
                             'failure':'SET_ROBOT_FACE_TO_ANGER'})

        #Wait until gripper in open
        smach.StateMachine.add('WAIT_UNTIL_GRIPPER_IS_OPEN', call_service(False),
            transitions={'success': 'SET_ACTION_LIB_SUCCESS'})

# open gripper
smach.StateMachine.add('CLOSE_GRIPPER', cs.PublishtoTopic('/left_arm_gripper/gripper_command', GripperCommand, GripperCommand(1)),
    transitions={'success':'WAIT_UNTIL_GRIPPER_IS_CLOSED'})

#Wait until gripper in open
smach.StateMachine.add('WAIT_UNTIL_GRIPPER_IS_CLOSED', cs.Sleep(5.0),
    transitions={'success': 'MOVE_ARM_TO_PREPL'})

# move arm to place position
smach.StateMachine.add('MOVE_ARM_TO_PREPL', gms.move_arm("preplace"),
    transitions={'succeeded':'MOVE_ARM_TO_MOVING','failed':'MOVE_ARM_TO_PREPL'})

# move arm to place position
smach.StateMachine.add('MOVE_ARM_TO_MOVING', gms.move_arm("moving"),
    transitions={'succeeded':'SET_ROBOT_FACE_TO_HAPPINESS','failed':'MOVE_ARM_TO_MOVING'})

# set a Happiness face on the robot (robot is happy to achieve the goal)
smach.StateMachine.add('SET_ROBOT_FACE_TO_HAPPINESS', cs.PublishString('led_emotions_interface/emotions','Happiness'),
    transitions={'success':'SET_ACTION_LIB_SUCCESS'})

# set a Anger face on the robot (robot is angry because goal could not be achieved but will try again)
smach.StateMachine.add('SET_ROBOT_FACE_TO_ANGER', cs.PublishString('led_emotions_interface/emotions','Anger'),
    transitions={'success':'SET_ACTION_LIB_FAILURE'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
            transitions={'succeeded':'OVERALL_SUCCESS'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
            transitions={'succeeded':'OVERALL_FAILED'})

    # smach viewer
    sis = smach_ros.IntrospectionServer('place_smach_viewer', sm, '/PLACE_SMACH_VIEWER')
    sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'place_server', 
        action_spec = PlaceAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'place_goal',
        feedback_key = 'place_feedback',
        result_key   = 'place_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
    main()
