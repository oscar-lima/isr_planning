#!/usr/bin/env python
import rospy
import smach


# for send and receive event combined
import mcr_states.common.basic_states as gbs
# arm motions
import mir_states.common.manipulation_states as gms
# object recognition
import mcr_states.common.perception_states as gps

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import ComputeBaseShiftAction
from mir_yb_action_msgs.msg import ComputeBaseShiftFeedback
from mir_yb_action_msgs.msg import ComputeBaseShiftResult

#===============================================================================

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['compute_base_shift_goal'],
                                    output_keys=['compute_base_shift_feedback', 'compute_base_shift_result'])
        self.result = result

    def execute(self, userdata):
        result = ComputeBaseShiftResult()
        result.success = self.result
        feedback = ComputeBaseShiftFeedback()
        userdata.compute_base_shift_result = result
        userdata.compute_base_shift_feedback = feedback
        return 'succeeded'


#===============================================================================




def main():
    rospy.init_node('compute_base_shift_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['compute_base_shift_goal'],
            output_keys = ['compute_base_shift_feedback', 'compute_base_shift_result'])
    sm.userdata.next_arm_pose_index = 0
    sm.userdata.move_arm_to = None
    with sm:
        #add states to the container
        # remove accumulated base poses and start non loop dependant components
        smach.StateMachine.add('CLEAR_BASE_POSES', gbs.send_event([
			('/move_base_relative/pose_selector/event_in','e_forget'),
                        ('/move_base_relative/random_obj_selector_transformer_republisher/event_in','e_start'),
                        ('/move_base_relative/modified_pose_transformer/event_in','e_start')]),
                transitions={'success':'SELECT_NEXT_LOOK_POSE'})

        # select a look_at_workspace pose
        smach.StateMachine.add('SELECT_NEXT_LOOK_POSE', gms.select_arm_pose(['look_at_workspace']),
            transitions={'succeeded': 'LOOK_AROUND', # next pose selected
                          'completed': 'STOP_REMAINING_COMPONENTS_WITH_SUCCESS',
                         'failed': 'STOP_REMAINING_COMPONENTS_WITH_SUCCESS'}) # we've run out of poses to select, which means we've gone through the list

        # move arm to selected pose
        smach.StateMachine.add('LOOK_AROUND', gms.move_arm(),
            transitions={'succeeded': 'RECOGNIZE_OBJECTS',
                         'failed': 'LOOK_AROUND'})

        smach.StateMachine.add('RECOGNIZE_OBJECTS', gps.find_objects(retries=1),
            transitions={'objects_found': 'SELECT_OBJECT',
                        'no_objects_found':'SELECT_NEXT_LOOK_POSE'},
            remapping={'found_objects':'recognized_objects'})

        # here the object selector is set up to select objects randomly
        # it will publish object name (goes to pose selector) and object pose (goes to compute base shift pipeline)
	# relative displacement calculator pose republisher in odom frame is also triggered here, before doing computations
        smach.StateMachine.add('SELECT_OBJECT', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/random_object_selector/event_in','e_trigger')],
                event_out_list=[('/mcr_perception/random_object_selector/event_out','e_selected', True)],
                timeout_duration=10),
                transitions={'success':'SELECT_OBJECT',
                             'timeout':'SELECT_NEXT_LOOK_POSE',
                             'failure':'SELECT_NEXT_LOOK_POSE'}) # this means we're done computing poses for all recognized objects from this pose


        # stop relative displacement calculator pose republisher and after that go to success
        smach.StateMachine.add('STOP_REMAINING_COMPONENTS_WITH_SUCCESS', gbs.send_event([
                                ('/move_base_relative/random_obj_selector_transformer_republisher/event_in','e_stop'),
                                ('/move_base_relative/modified_pose_transformer/event_in','e_stop')]),
                transitions={'success':'SET_ACTION_LIB_SUCCESS'})

        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                               transitions={'succeeded':'OVERALL_SUCCESS'})

        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False),
                               transitions={'succeeded':'OVERALL_FAILED'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'compute_base_shift_server',
        action_spec = ComputeBaseShiftAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'compute_base_shift_goal',
        feedback_key = 'compute_base_shift_feedback',
        result_key   = 'compute_base_shift_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
