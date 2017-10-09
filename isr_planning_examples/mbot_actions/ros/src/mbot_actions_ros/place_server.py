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
        smach.StateMachine.add('MOVE_ARM_TO_PREPLACE', gms.move_arm("preplace"),
                               transitions={'succeeded':'MOVE_ARM_TO_PLACE','failed':'MOVE_ARM_TO_PREPLACE'})

        #
        smach.StateMachine.add('MOVE_ARM_TO_PLACE', gms.move_arm("place"),
            transitions={'succeeded':'OPEN_GRIPPER','failed':'MOVE_ARM_TO_PLACE'})
        
        # open gripper
        smach.StateMachine.add('OPEN_GRIPPER', cs.PublishtoTopic('/left_arm_gripper/gripper_command', GripperCommand, GripperCommand(0)),
            transitions={'success': 'WAIT_UNTIL_GRIPPER_IS_OPEN'})

        #Wait until gripper in open
        smach.StateMachine.add('WAIT_UNTIL_GRIPPER_IS_OPEN', cs.Sleep(5.0),
            transitions={'success': 'CLOSE_GRIPPER'})

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
