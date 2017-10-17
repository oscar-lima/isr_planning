#!/usr/bin/python

import rospy
import smach
import smach_ros

# message definitions
from monarch_msgs.msg import HeadControlSemantic

# import of generic states
import mcr_states.common.perception_states as gps

# action lib
from smach_ros import ActionServerWrapper
from mbot_action_msgs.msg import PerceiveLocationAction
from mbot_action_msgs.msg import PerceiveLocationFeedback
from mbot_action_msgs.msg import PerceiveLocationResult

# mbot state imports
import mbot_states.common_states as mcs
import mbot_states.hri_states as mhs
import mbot_states.perception_states as mps

# ===============================================================================


class GetGoal(smach.State):  # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'location_unknown'],
                             input_keys=['perceive_location_goal'],
                             output_keys=['perceive_location_feedback', 'perceive_location_result'])

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
        feedback.text = '[perceive] Perceiving location : ' + platform
        userdata.perceive_location_feedback = feedback
        # check if location exists
        if not rospy.has_param('/world_model/' + platform):
            rospy.logwarn('[GetGoal] location ' + platform + ' not found in world_model/' + platform)
            return 'location_unknown'
        return 'succeeded'

# ===============================================================================


def main():
    rospy.init_node('perceive_location_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILED'],
            input_keys=['perceive_location_goal'],
            output_keys=['perceive_location_feedback', 'perceive_location_result'])
    # Open the container
    with sm:
        # approach to platform
        smach.StateMachine.add('GET_GOAL', GetGoal(),
                               transitions={'succeeded': 'ROTATE_HEAD_LEFT',
                                            'location_unknown': 'OVERALL_FAILED'})

        # rotate to desired direction
        smach.StateMachine.add('ROTATE_HEAD_LEFT',
                               mhs.rotate_head(timeout=2,
                                               send_topic='semantic_head_controller_node/cmd', wait_topic='cmd_head',
                                               direction='W', speed=HeadControlSemantic.NORMAL,
                                               sleep_duration=rospy.Duration.from_sec(1.0)),
                               transitions={'success': 'DECIDE_CAMERA',
                                            'timeout': 'SET_ACTION_LIB_FAILURE'})

        # decide which camera to use
        smach.StateMachine.add('DECIDE_CAMERA',
                               mps.decide_camera(overload_parameter='camera/overload'),
                               transitions={'camera_neck': 'SUBSCRIBE_TO_POINT_CLOUD',
                                            'camera_head': 'TILT_HEAD_SERVO',
                                            'failure': 'ROTATE_HEAD_FRONT_WITH_FAILURE'})

        # tilt head servo
        smach.StateMachine.add('TILT_HEAD_SERVO',
                               mps.tilt_servo(timeout=2,
                                              send_topic='cmd_tilt', wait_topic='servo_position'),
                               transitions={'success': 'SUBSCRIBE_TO_POINT_CLOUD',
                                            'timeout': 'ROTATE_HEAD_FRONT_WITH_FAILURE'})

        smach.StateMachine.add('SUBSCRIBE_TO_POINT_CLOUD',
                               mcs.PublishString('/mcr_perception/mux_pointcloud/select',
                                                '/camera/depth_registered/points'),
                               transitions={'success': 'RECOGNIZE_OBJECTS'})

        # trigger perception pipeline
        smach.StateMachine.add('RECOGNIZE_OBJECTS', gps.find_objects(retries=1),
                               transitions={'objects_found': 'UNSUBSCRIBE_FROM_POINT_CLOUD_WITH_SUCCESS',
                                            'no_objects_found': 'UNSUBSCRIBE_FROM_POINT_CLOUD_WITH_FAILURE'},
                               remapping={'found_objects': 'recognized_objects'})

        smach.StateMachine.add('UNSUBSCRIBE_FROM_POINT_CLOUD_WITH_SUCCESS',
                               mcs.PublishString('/mcr_perception/mux_pointcloud/select', '/empty_topic'),
                               transitions={'success': 'ROTATE_HEAD_FRONT_WITH_SUCCESS'})

        smach.StateMachine.add('UNSUBSCRIBE_FROM_POINT_CLOUD_WITH_FAILURE',
                               mcs.PublishString('/mcr_perception/mux_pointcloud/select', '/empty_topic'),
                               transitions={'success': 'ROTATE_HEAD_FRONT_WITH_FAILURE'})

        # rotate head back to front
        smach.StateMachine.add('ROTATE_HEAD_FRONT_WITH_SUCCESS',
                               mhs.rotate_head(timeout=2,
                                               send_topic='semantic_head_controller_node/cmd', wait_topic='cmd_head',
                                               direction='N', speed=HeadControlSemantic.NORMAL),
                               transitions={'success': 'SET_ACTION_LIB_SUCCESS',
                                            'timeout': 'SET_ACTION_LIB_FAILURE'})

        # rotate head back to front
        smach.StateMachine.add('ROTATE_HEAD_FRONT_WITH_FAILURE',
                               mhs.rotate_head(timeout=2,
                                               send_topic='semantic_head_controller_node/cmd', wait_topic='cmd_head',
                                               direction='N', speed=HeadControlSemantic.NORMAL),
                               transitions={'success': 'SET_ACTION_LIB_FAILURE',
                                            'timeout': 'SET_ACTION_LIB_FAILURE'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS',
                               mcs.SetActionLibResult(
                                   True, PerceiveLocationResult, PerceiveLocationFeedback,
                                   ['perceive_location_goal'],
                                   ['perceive_location_feedback', 'perceive_location_result']),
                               transitions={'success': 'OVERALL_SUCCESS'})
                               
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE',
                               mcs.SetActionLibResult(
                                   False, PerceiveLocationResult, PerceiveLocationFeedback,
                                   ['perceive_location_goal'],
                                   ['perceive_location_feedback', 'perceive_location_result']),
                               transitions={'success': 'OVERALL_FAILED'})
    # smach viewer
    sis = smach_ros.IntrospectionServer('perceive_location_smach_viewer', sm, '/perceive_location_SMACH_VIEWER')
    sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name='perceive_location_server',
        action_spec=PerceiveLocationAction,
        wrapped_container=sm,
        succeeded_outcomes=['OVERALL_SUCCESS'],
        aborted_outcomes=['OVERALL_FAILED'],
        preempted_outcomes=['PREEMPTED'],
        goal_key='perceive_location_goal',
        feedback_key='perceive_location_feedback',
        result_key='perceive_location_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
    main()
