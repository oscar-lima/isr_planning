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
from mbot_action_msgs.msg import PickObjectAction, PickObjectFeedback, PickObjectResult

#===============================================================================

class select_object(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['pick_object_goal'], 
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
        # create publisher
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        result = PickObjectResult()
        result.success = False
        userdata.pick_object_result = result
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'SELECT_OBJECT'
        userdata.pick_object_feedback = feedback
        # receive parameters from actionlib
        object_to_pick = userdata.pick_object_goal.object.upper()
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

class send_event(smach.State):
    '''
    This class publishes e_start on topic_name argument
    '''
    def __init__(self, topic_name, event):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['pick_object_goal'], 
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
        # create publisher
        self.topic_name = topic_name
        self.event = event
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        # give feedback
        feedback = PickObjectFeedback()
        feedback.current_state = 'SEND_EVENT'
        userdata.pick_object_feedback = feedback
        # creating string message
        msg = String()
        # filling message
        msg.data = self.event
        # publish
        self.publisher.publish(msg)
        rospy.loginfo('publishing on ' + self.topic_name + ' ' + self.event)
        # wait, dont kill the node so quickly
        rospy.sleep(0.2)
        return 'success'
        
#===============================================================================
        
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['pick_object_goal'], 
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
        self.result = result

    def execute(self, userdata):
        result = PickObjectResult()
        result.success = self.result
        userdata.pick_object_result = result
        return 'succeeded'
        
        
#===============================================================================
        
class grasp_monitor_mockup(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['pick_object_goal'], 
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
    
    def execute(self, userdata):
        # mockup graps monitor, will always return success -> replace with padmaja stuff later on
        return 'succeeded'
        
#===============================================================================

def main(mockup=False):
    # Open the container
    rospy.init_node('pick_object_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['pick_object_goal'],
            output_keys = ['pick_object_feedback', 'pick_object_result'])
    with sm:
        if not mockup:
            # publish object as string to mcr_perception_selectors -> object_selector, this component then publishes
            # pose in base_link reference frame when e_trigger is sent (next state)
            smach.StateMachine.add('SELECT_OBJECT', select_object('/mcr_perception/object_selector/input/object_name'),
                transitions={'success':'SET_ROBOT_FACE_TO_SURPRISE'})

            # set a Surprise face on the robot (just for fun)
            smach.StateMachine.add('SET_ROBOT_FACE_TO_SURPRISE', cs.PublishString('led_emotions_interface/emotions','Surprise'),
                transitions={'success': 'OPEN_GRIPPER'})

            # open gripper
            smach.StateMachine.add('OPEN_GRIPPER', cs.PublishtoTopic('/left_arm_gripper/gripper_command', GripperCommand, GripperCommand(0)),
                transitions={'success': 'MOVE_ARM_TO_CANDLE'})

            # move arm to candle position as a workaround for not hitting the arm with the table
            smach.StateMachine.add('MOVE_ARM_TO_CANDLE', gms.move_arm("candle"),
                transitions={'succeeded':'MOVE_ARM_TO_PREGRASP', 'failed':'MOVE_ARM_TO_HOLD'})

            # move arm to pregrasp position to be ready to grasp object
            smach.StateMachine.add('MOVE_ARM_TO_PREGRASP', gms.move_arm("pregrasp_side_table"),
                transitions={'succeeded':'GENERATE_OBJECT_POSE', 'failed':'MOVE_ARM_TO_HOLD'})

            # trigger object selector, to publish object pose
            smach.StateMachine.add('GENERATE_OBJECT_POSE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/object_selector/event_in','e_trigger')],
                event_out_list=[('/mcr_perception/object_selector/event_out','e_selected', True)],
                timeout_duration=1.0),
                transitions={'success':'RECONFIGURE_PREGRASP_PARAMS',
                             'timeout':'SET_ROBOT_FACE_TO_ANGER',
                             'failure':'SET_ROBOT_FACE_TO_ANGER'})

        # reconfigure pregrasp planner for pick
        smach.StateMachine.add('RECONFIGURE_PREGRASP_PARAMS', gbs.set_named_config('front_small_table'),
            transitions={'success':'PLAN_ARM_MOTION',
                         'failure':'SET_ROBOT_FACE_TO_ANGER',
                         'timeout': 'RECONFIGURE_PREGRASP_PARAMS'})
 
        # based on a published pose, calls pregrasp planner to generate a graspable pose and wait for the result of the pregrasp planner
        smach.StateMachine.add('PLAN_ARM_MOTION', gbs.send_and_wait_events_combined(
            event_in_list=[('/mir_manipulation/pregrasp_planner_pipeline/event_in','e_start')],
            event_out_list=[('/mir_manipulation/pregrasp_planner_pipeline/event_out','e_success', True)],
            timeout_duration=60),
            transitions={'success':'STOP_PLAN_ARM_MOTION',
                              'timeout':'STOP_PLAN_ARM_MOTION_WITH_FAILURE',
                              'failure':'STOP_PLAN_ARM_MOTION_WITH_FAILURE'})

        # pregrasp planner was successful, so lets stop it since its work is done
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION', send_event('/pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'MOVE_ARM_TO_OBJECT'})

        # pregrasp planner failed or timeout, stop the component and then return overall failure
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION_WITH_FAILURE', send_event('/pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'SET_ROBOT_FACE_TO_ANGER'})

        # move arm to pregrasp planned pose and wait for its response
        smach.StateMachine.add('MOVE_ARM_TO_OBJECT', gbs.send_and_wait_events_combined(
            event_in_list=[('/move_arm_planned_motion/event_in','e_start')],
            event_out_list=[('/move_arm_planned_motion/event_out','e_success', True)],
            timeout_duration=7),
            transitions={'success':'STOP_MOVE_ARM_TO_OBJECT',
                          'timeout':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE',
                          'failure':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE'})
        
        # send stop event_in to arm motion component and return failure
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE', send_event('/move_arm_planned/event_in','e_stop'),
            transitions={'success':'SET_ROBOT_FACE_TO_ANGER'})

        # send stop event_in to arm motion component
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT', send_event('/move_arm_planned/event_in','e_stop'),
            transitions={'success':'WAIT_FOR_ARM_TO_OBJECT'})

	# sleep for a bit to allow time for arm to get to final pose
	smach.StateMachine.add('WAIT_FOR_ARM_TO_OBJECT', cs.Sleep(2),
	    transitions={'success':'CLOSE_GRIPPER'})

        smach.StateMachine.add('CLOSE_GRIPPER', cs.PublishtoTopic('/left_arm_gripper/gripper_command', GripperCommand, GripperCommand(1)),
            transitions={'success': 'MOVE_ARM_TO_HOLD'})

        # move arm to HOLD position
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("pregrasp_side_table"),
            transitions={'succeeded':'SET_ROBOT_FACE_TO_HAPPINESS', 'failed':'MOVE_ARM_TO_HOLD'})

        # set a Happiness face on the robot (robot is happy to achieve the goal)
        smach.StateMachine.add('SET_ROBOT_FACE_TO_HAPPINESS', cs.PublishString('led_emotions_interface/emotions','Happiness'),
                transitions={'success': 'SET_ACTION_LIB_SUCCESS'})

        # set a Anger face on the robot (robot is angry because goal could not be achieved but will try again)
        smach.StateMachine.add('SET_ROBOT_FACE_TO_ANGER', cs.PublishString('led_emotions_interface/emotions','Anger'),
                transitions={'success': 'SET_ACTION_LIB_FAILURE'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
            transitions={'succeeded':'OVERALL_SUCCESS'})

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
            transitions={'succeeded':'OVERALL_FAILED'})

    # smach viewer
    sis = smach_ros.IntrospectionServer('pick_object_smach_viewer', sm, '/PICK_OBJECT_SMACH_VIEWER')
    sis.start()

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'pick_object_server', 
        action_spec = PickObjectAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'pick_object_goal',
        feedback_key = 'pick_object_feedback',
        result_key   = 'pick_object_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
    if len(sys.argv) > 1:
       main(mockup=(sys.argv[1]=='mockup'))
    else:
       main(mockup=False)
