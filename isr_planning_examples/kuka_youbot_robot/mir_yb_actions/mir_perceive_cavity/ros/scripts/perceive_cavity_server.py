#!/usr/bin/python

import rospy
import smach
import smach_ros

# import of generic states
import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms
import mcr_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import PerceiveLocationAction
from mir_yb_action_msgs.msg import PerceiveLocationFeedback
from mir_yb_action_msgs.msg import PerceiveLocationResult

#===============================================================================

class getGoal(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], 
                                   input_keys=['perceive_cavity_goal'], 
                                   output_keys=['perceive_cavity_feedback', 'perceive_cavity_result'])
        self.counter = 0

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = PerceiveLocationResult()
        result.success = False
        userdata.perceive_cavity_result = result
        # get arm goal from actionlib
        platform = userdata.perceive_cavity_goal.location
        # giving feedback to the user
        feedback = PerceiveLocationFeedback()
        feedback.current_state = 'GET_GOAL'
        feedback.text='[perceive_cavity] Perceiving location : ' + platform
        userdata.perceive_cavity_feedback = feedback
        return 'succeeded'
        
#===============================================================================
        
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['perceive_cavity_goal'], 
                                    output_keys=['perceive_cavity_feedback', 'perceive_cavity_result'])
        self.result = result

    def execute(self, userdata):
        result = PerceiveLocationResult()
        result.success = self.result
        userdata.perceive_cavity_result = result
        return 'succeeded'
        
#===============================================================================

def main():
    rospy.init_node('perceive_cavity_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['perceive_cavity_goal'],
            output_keys = ['perceive_cavity_feedback', 'perceive_cavity_result'])
    sm.userdata.next_arm_pose_index = 0
    # Open the container
    with sm:
        # approach to platform
        smach.StateMachine.add('GET_GOAL', getGoal(),
            transitions={'succeeded':'PUBLISH_REFERENCE_FRAME_FOR_WBC'})

        # generates a pose based on the previous string object topic received
        smach.StateMachine.add('PUBLISH_REFERENCE_FRAME_FOR_WBC', gbs.send_event([('/static_transform_publisher_node/event_in', 'e_start')]),
                transitions={'success':'START_POSE_SELECTOR'})

        # start the cavity pose selector to accumulate the poses
        smach.StateMachine.add('START_POSE_SELECTOR', gbs.send_event([('/mcr_perception/cavity_pose_selector/event_in', 'e_start')]),
                transitions={'success':'SELECT_NEXT_LOOK_POSE'})

        # select a look_at_workspace pose
        smach.StateMachine.add('SELECT_NEXT_LOOK_POSE', gms.select_arm_pose(['look_straight_at_workspace_left','look_straight_at_workspace_right']),
            transitions={'succeeded': 'LOOK_AROUND', # next pose selected
                         'completed': 'SET_ACTION_LIB_SUCCESS', # we've run out of poses to select, which means we've gone through the list
                         'failed': 'SET_ACTION_LIB_SUCCESS'}) # we've run out of poses to select, which means we've gone through the list

        # move arm to selected pose
        smach.StateMachine.add('LOOK_AROUND', gms.move_arm(),
            transitions={'succeeded': 'RECOGNIZE_CAVITIES',
                         'failed': 'LOOK_AROUND'})

        # trigger perception pipeline
        smach.StateMachine.add('RECOGNIZE_CAVITIES', gps.find_cavities(retries=1),
            transitions={'cavities_found': 'SELECT_NEXT_LOOK_POSE',
                        'no_cavities_found':'SELECT_NEXT_LOOK_POSE'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                               transitions={'succeeded':'OVERALL_SUCCESS'})
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                               transitions={'succeeded':'OVERALL_FAILED'})
    # smach viewer
    sis = smach_ros.IntrospectionServer('perceive_cavity_smach_viewer', sm, '/PERCEIVE_CAVITY_SMACH_VIEWER')
    sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'perceive_cavity_server', 
        action_spec = PerceiveLocationAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'perceive_cavity_goal',
        feedback_key = 'perceive_cavity_feedback',
        result_key   = 'perceive_cavity_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
    main()
