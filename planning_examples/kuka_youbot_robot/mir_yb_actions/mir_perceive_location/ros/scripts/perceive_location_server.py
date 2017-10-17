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
                                   input_keys=['perceive_location_goal'], 
                                   output_keys=['perceive_location_feedback', 'perceive_location_result'])
        self.counter = 0

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = PerceiveLocationResult()
        result.success = False
        userdata.perceive_location_result = result
        # get arm goal from actionlib
        platform = userdata.perceive_location_goal.location
        # giving feedback to the user
        feedback = PerceiveLocationFeedback()
        feedback.current_state = 'GET_GOAL'
        feedback.text='[perceive_location] Perceiving location : ' + platform
        userdata.perceive_location_feedback = feedback
        return 'succeeded'
        
#===============================================================================
        
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['perceive_location_goal'], 
                                    output_keys=['perceive_location_feedback', 'perceive_location_result'])
        self.result = result

    def execute(self, userdata):
        result = PerceiveLocationResult()
        result.success = self.result
        userdata.perceive_location_result = result
        return 'succeeded'
        
#===============================================================================

def main():
    rospy.init_node('perceive_location_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['perceive_location_goal'],
            output_keys = ['perceive_location_feedback', 'perceive_location_result'])
    # Open the container
    with sm:
        # approach to platform
        smach.StateMachine.add('GET_GOAL', getGoal(),
            transitions={'succeeded':'PUBLISH_REFERENCE_FRAME'})
            #transitions={'succeeded':'START_OBJECT_LIST_MERGER'})

        # generates a pose based on the previous string object topic received
        smach.StateMachine.add('PUBLISH_REFERENCE_FRAME', gbs.send_event([('/static_transform_publisher_node/event_in', 'e_start')]),
                transitions={'success':'START_OBJECT_LIST_MERGER'})

        smach.StateMachine.add('START_OBJECT_LIST_MERGER', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_start')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_started', True)],
                timeout_duration=5),
                transitions={'success': 'LOOK_AT_WORKSPACE_LEFT',
                             'timeout': 'SET_ACTION_LIB_FAILURE',
                             'failure': 'SET_ACTION_LIB_FAILURE'})

        # send arm to a position in which the depth camera can see the platformsmach.StateMachine.add('LOOK_AT_WORKSPACE_LEFT', gms.move_arm('look_at_workspace_LEFT'),
        smach.StateMachine.add('LOOK_AT_WORKSPACE_LEFT', gms.move_arm('look_at_workspace_left'),
            transitions={'succeeded': 'SUBSCRIBE_TO_POINT_CLOUD',
                            'failed': 'LOOK_AT_WORKSPACE_LEFT'})

        smach.StateMachine.add('SUBSCRIBE_TO_POINT_CLOUD', gbs.send_event([('/mcr_perception/mux_pointcloud/select', '/arm_cam3d/depth_registered/points')]),
                transitions={'success':'RECOGNIZE_OBJECTS_LEFT'})

        # trigger perception pipeline
        smach.StateMachine.add('RECOGNIZE_OBJECTS_LEFT', gps.find_objects(retries=1),
            transitions={'objects_found': 'LOOK_AT_WORKSPACE_STRAIGHT',
                        'no_objects_found':'LOOK_AT_WORKSPACE_STRAIGHT'},
            #transitions={'objects_found': 'SUBSCRIBE_TO_POINT_CLOUD_2',
            #            'no_objects_found':'SUBSCRIBE_TO_POINT_CLOUD_2'},
            remapping={'found_objects':'recognized_objects'})
        
        smach.StateMachine.add('SUBSCRIBE_TO_POINT_CLOUD_2', gbs.send_event([('/mcr_perception/mux_pointcloud/select', '/arm_cam3d/depth_registered/points')]),
                transitions={'success':'LOOK_AT_WORKSPACE_STRAIGHT'})

        # send arm to a position in which the depth camera can see the platform
        smach.StateMachine.add('LOOK_AT_WORKSPACE_STRAIGHT', gms.move_arm('look_at_workspace'),
            transitions={'succeeded': 'RECOGNIZE_OBJECTS_STRAIGHT',
                            'failed': 'LOOK_AT_WORKSPACE_STRAIGHT'})

        # trigger perception pipeline
        smach.StateMachine.add('RECOGNIZE_OBJECTS_STRAIGHT', gps.find_objects(retries=1),
            transitions={'objects_found': 'LOOK_AT_WORKSPACE_RIGHT',
                        'no_objects_found':'LOOK_AT_WORKSPACE_RIGHT'},
            #transitions={'objects_found': 'SUBSCRIBE_TO_POINT_CLOUD_3',
            #            'no_objects_found':'SUBSCRIBE_TO_POINT_CLOUD_3'},
            remapping={'found_objects':'recognized_objects'})

        smach.StateMachine.add('SUBSCRIBE_TO_POINT_CLOUD_3', gbs.send_event([('/mcr_perception/mux_pointcloud/select', '/arm_cam3d/depth_registered/points')]),
                transitions={'success':'LOOK_AT_WORKSPACE_RIGHT'})

        # send arm to a position in which the depth camera can see the platform
        smach.StateMachine.add('LOOK_AT_WORKSPACE_RIGHT', gms.move_arm('look_at_workspace_right'),
            transitions={'succeeded': 'RECOGNIZE_OBJECTS_RIGHT',
                            'failed': 'LOOK_AT_WORKSPACE_RIGHT'})

        # trigger perception pipeline
        smach.StateMachine.add('RECOGNIZE_OBJECTS_RIGHT', gps.find_objects(retries=1),
            transitions={'objects_found': 'STOP_OBJECT_LIST_MERGER',
                        'no_objects_found':'STOP_OBJECT_LIST_MERGER'},
            remapping={'found_objects':'recognized_objects'})

        smach.StateMachine.add('STOP_OBJECT_LIST_MERGER', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_stop')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_stopped', True)],
                timeout_duration=5),
                transitions={'success': 'UNSUBSCRIBE_FROM_POINT_CLOUD',
                             'timeout': 'UNSUBSCRIBE_FROM_POINT_CLOUD',
                             'failure': 'UNSUBSCRIBE_FROM_POINT_CLOUD'})

        smach.StateMachine.add('UNSUBSCRIBE_FROM_POINT_CLOUD', gbs.send_event([('/mcr_perception/mux_pointcloud/select', '/emtpty_topic')]),
                transitions={'success':'PUBLISH_MERGED_OBJECT_LIST'})

        smach.StateMachine.add('PUBLISH_MERGED_OBJECT_LIST', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_list_merger/event_in', 'e_trigger')],
                event_out_list=[('/mcr_perception/object_list_merger/event_out', 'e_done', True)],
                timeout_duration=5),
                transitions={'success': 'SET_ACTION_LIB_SUCCESS',
                             'timeout': 'SET_ACTION_LIB_FAILURE',
                             'failure': 'SET_ACTION_LIB_FAILURE'})
        
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                               transitions={'succeeded':'OVERALL_SUCCESS'})
                               
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                               transitions={'succeeded':'OVERALL_FAILED'})
    # smach viewer
    sis = smach_ros.IntrospectionServer('perceive_location_smach_viewer', sm, '/PERCEIVE_LOCATION_SMACH_VIEWER')
    sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'perceive_location_server', 
        action_spec = PerceiveLocationAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'perceive_location_goal',
        feedback_key = 'perceive_location_feedback',
        result_key   = 'perceive_location_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
    main()
