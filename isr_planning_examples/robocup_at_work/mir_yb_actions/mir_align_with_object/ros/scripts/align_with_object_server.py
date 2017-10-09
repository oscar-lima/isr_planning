#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String

# for send and receive event combined
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import AlignWithObjectAction
from mir_yb_action_msgs.msg import AlignWithObjectFeedback
from mir_yb_action_msgs.msg import AlignWithObjectResult

# to publish pose stamped from param server
from geometry_msgs.msg import PoseStamped


class select_object(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['align_with_object_goal'], 
                                    output_keys=['align_with_object_feedback', 'align_with_object_result'])
        # create publisher
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        result = AlignWithObjectResult()
        result.success = False
        userdata.align_with_object_result = result
        # give feedback
        feedback = AlignWithObjectFeedback()
        feedback.current_state = 'SELECT_OBJECT'
        userdata.align_with_object_feedback = feedback
        # receive parameters from actionlib
        # TODO: why is this being set to all caps? 
        object_to_pick = userdata.align_with_object_goal.object.upper()
        # creating string message
        msg = String()
        # filling message
        msg.data = object_to_pick
        self.publisher.publish(msg)
        # do not kill the node so fast, let the topic to survive for some time
        rospy.loginfo('publishing on ' + self.topic_name + ' : ' + object_to_pick)
        rospy.sleep(0.2)
        return 'success'

#===============================================================================

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['align_with_object_goal'],
                                    output_keys=['align_with_object_feedback', 'align_with_object_result'])
        self.result = result

    def execute(self, userdata):
        result = AlignWithObjectResult()
        result.success = self.result
        feedback = AlignWithObjectFeedback()
        userdata.align_with_object_result = result
        userdata.align_with_object_feedback = feedback
        return 'succeeded'


#===============================================================================

def main():
    rospy.init_node('align_with_object_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['align_with_object_goal'],
            output_keys = ['align_with_object_feedback', 'align_with_object_result'])
    with sm:
        #add states to the container
        # publish name of object to which we want to align
        smach.StateMachine.add('PUBLISH_OBJECT_NAME', select_object('/move_base_relative/pose_selector/selected_object_name'),
            transitions={'success':'SELECT_POSE_FOR_OBJECT'})

        # trigger pose selector. this will use the object name to output the corresponding pose (in map/odom frame)
        smach.StateMachine.add('SELECT_POSE_FOR_OBJECT', gbs.send_and_wait_events_combined(
                event_in_list=[('/move_base_relative/pose_selector/event_in','e_trigger')],
                event_out_list=[('/move_base_relative/pose_selector/event_out','e_selected', True)],
                timeout_duration=10),
                transitions={'success':'START_DIRECT_BASE_CONTROLLER',
                             'timeout':'SET_ACTION_LIB_FAILURE',
                             'failure':'SET_ACTION_LIB_FAILURE'}) # object is not in the list

        # call direct base controller
        # Note: the pose out from pose selector needs to be mapped to input pose of direct base controller
        # time out of 20 seconds is set
        smach.StateMachine.add('START_DIRECT_BASE_CONTROLLER', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_start')],
                event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_success', True), # if this happens it has succeeded
                                ('/mcr_navigation/collision_velocity_filter/event_out', 'e_zero_velocities_forwarded', False)], # if this happens we stop
                timeout_duration=20),
                transitions={'success':'STOP_CONTROLLER_WITH_SUCCESS',
                             'timeout':'STOP_CONTROLLER_WITH_FAILURE',
                             'failure':'STOP_CONTROLLER_WITH_FAILURE'}) # this means we're done computing poses for all recognized objects from this pose


        # stop controller with success
        smach.StateMachine.add('STOP_CONTROLLER_WITH_SUCCESS', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop')],
                event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_stopped', True)],
                timeout_duration=1),
                transitions={'success':'SET_ACTION_LIB_SUCCESS',
                             'timeout':'SET_ACTION_LIB_SUCCESS',
                             'failure':'SET_ACTION_LIB_FAILURE'})

        # stop controller with failure
        smach.StateMachine.add('STOP_CONTROLLER_WITH_FAILURE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop')],
                event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_stopped', True)],
                timeout_duration=1),
                transitions={'success':'SET_ACTION_LIB_FAILURE',
                             'timeout':'SET_ACTION_LIB_FAILURE',
                             'failure':'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                               transitions={'succeeded':'OVERALL_SUCCESS'})

        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False),
                               transitions={'succeeded':'OVERALL_FAILED'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'align_with_object_server',
        action_spec = AlignWithObjectAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'align_with_object_goal',
        feedback_key = 'align_with_object_feedback',
        result_key   = 'align_with_object_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
