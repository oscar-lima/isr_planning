#!/usr/bin/python
import sys
import rospy
import smach
import smach_ros

from std_msgs.msg import String

# import of generic states
import mir_states.common.manipulation_states as gms

# for dynamic reconfigure node + grasp monitor
import mcr_states.common.basic_states as gbs
# action lib
from smach_ros import ActionServerWrapper

 
# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import PlaceObjectAction
from mir_yb_action_msgs.msg import PlaceObjectFeedback
from mir_yb_action_msgs.msg import PlaceObjectResult
        
#===============================================================================

class select_object(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['place_object_goal'],
                                    output_keys=['place_object_feedback', 'place_object_result'])
        # create publisher
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        result = PlaceObjectResult()
        result.success = False
        userdata.place_object_result = result
        # give feedback
        feedback = PlaceObjectFeedback()
        feedback.current_state = 'SELECT_OBJECT'
        userdata.place_object_feedback = feedback
        # receive parameters from actionlib
        object_to_place = userdata.place_object_goal.object.upper()
        # creating string message
        msg = String()
        # filling message
        msg.data = object_to_place
        self.publisher.publish(msg)
        # do not kill the node so fast, let the topic to survive for some time
        rospy.loginfo('publishing on ' + self.topic_name + ' : ' + object_to_place)
        rospy.sleep(0.2)
        return 'success'
#===============================================================================

class send_event(smach.State):
    '''
    This class publishes e_start on topic_name argument
    '''
    def __init__(self, topic_name, event):
        smach.State.__init__(self,  outcomes=['success'],
                                    input_keys=['place_object_goal'],
                                    output_keys=['place_object_feedback', 'place_object_result'])
        # create publisher
        self.topic_name = topic_name
        self.event = event
        self.publisher = rospy.Publisher(self.topic_name, String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        # give feedback
        feedback = PlaceObjectFeedback()
        feedback.current_state = 'SEND_EVENT'
        userdata.place_object_feedback = feedback
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

class wait_for_event(smach.State):
    '''
    This state will take a event name as input and waits for the event to
    be published.
    '''
    def __init__(self, topic_name, timeout_duration):
        smach.State.__init__(self,  outcomes=['success', 'failure', 'timeout'],
                                    input_keys=['place_object_goal'],
                                    output_keys=['place_object_feedback', 'place_object_result'])
        rospy.Subscriber(topic_name, String, self.event_cb)
        self.callback_msg_ = None
        self.message_received = False
        self.timeout = rospy.Duration.from_sec(timeout_duration)

    def event_cb(self, callback_msg):
        self.callback_msg_ = callback_msg
        self.message_received = True

    def getResult(self):
        if self.callback_msg_ is None:
            rospy.logerr('event out message not received in the specified time')
            return 'timeout'
        elif self.callback_msg_.data == 'e_failure':
            self.callback_msg_ = None
            return 'failure'
        elif self.callback_msg_.data == 'e_success' or self.callback_msg_.data == 'e_selected':
            self.callback_msg_ = None
            return 'success'
        else:
            print('[wait for event] : no response, or response message "{}" is not known.'.format(self.callback_msg_.data))
            self.callback_msg_ = None
            return 'failure'
        
    def execute(self, userdata):
        # give feedback
        feedback = PlaceObjectFeedback()
        feedback.current_state = 'WAIT_FOR_FEEDBACK'
        userdata.place_object_feedback = feedback
        # reset flag of received to false
        print "waiting for node response..."
        self.message_received = False
        start_time = rospy.Time.now()
        rate = rospy.Rate(10) # 10hz
        # wait for message to arrive
        while((rospy.Time.now() - start_time < self.timeout) and not(self.message_received) and not(rospy.is_shutdown())):
            rate.sleep()
        # dont kill the node so quickly, wait for handshake of nodes
        print("(rospy.Time.now() - start_time < self.timeout) = {}".format((rospy.Time.now() - start_time < self.timeout)))
        print("self.message_received = {}".format((self.message_received)))
        rospy.sleep(0.2)  
        return self.getResult()

        
#===============================================================================
        
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['place_object_goal'],
                                    output_keys=['place_object_feedback', 'place_object_result'])
        self.result = result

    def execute(self, userdata):
        result = PlaceObjectResult()
        result.success = self.result
        userdata.place_object_result = result
        return 'succeeded'
        
        
#===============================================================================
        
class grasp_monitor_mockup(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded'],
                                    input_keys=['place_object_goal'],
                                    output_keys=['place_object_feedback', 'place_object_result'])
    
    def execute(self, userdata):
        # mockup graps monitor, will always return success -> replace with padmaja stuff later on
        return 'succeeded'

#===============================================================================

def main(mokeup=False):
    # Open the container
    rospy.init_node('place_object_in_cavity_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['place_object_goal'],
            output_keys = ['place_object_feedback', 'place_object_result'])
    with sm:

        # publish object as string to mcr_perception_selectors -> cavity, this component then publishes
        # pose in base_link reference frame
        smach.StateMachine.add('SELECT_OBJECT', select_object('/mcr_perception/cavity_pose_selector/object_name'),
            transitions={'success':'CHECK_IF_OBJECT_IS_AVAILABLE'})
        

        # waits for object selector response, if failure this means that the string published previously was not found in object list
        #smach.StateMachine.add('CHECK_IF_OBJECT_IS_AVAILABLE', wait_for_event('/mcr_perception/cavity_pose_selector/event_out', 10.0),
        #    transitions={'success':'SELECT_PREGRASP_PLANNER_INPUT',
        #                    'timeout': 'SET_ACTION_LIB_FAILURE',
        #                    'failure':'SET_ACTION_LIB_FAILURE'})
    
        # waits for object selector response, if failure this means that the string published previously was not found in object list
        smach.StateMachine.add('CHECK_IF_OBJECT_IS_AVAILABLE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_perception/cavity_pose_selector/event_in','e_trigger')],
                event_out_list=[('/mcr_perception/cavity_pose_selector/event_out','e_success', True)],
                timeout_duration=10),
                transitions={'success':'SELECT_PREGRASP_PLANNER_INPUT',
                            'timeout':'SET_ACTION_LIB_FAILURE',
                            'failure':'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('SELECT_PREGRASP_PLANNER_INPUT', send_event('/mir_manipulation/mux_pregrasp_pose/select',
                                                                            '/whole_body_motion_calculator_pipeline/pose_out'),
                transitions={'success':'PLAN_WHOLE_BODY_MOTION'})
 
        smach.StateMachine.add('PLAN_WHOLE_BODY_MOTION', send_event('/whole_body_motion_calculator_pipeline/event_in','e_start'),
            transitions={'success':'WAIT_PLAN_WHOLE_BODY_MOTION'})

        # wait for the result of the pregrasp planner
        smach.StateMachine.add('WAIT_PLAN_WHOLE_BODY_MOTION', wait_for_event('/whole_body_motion_calculator_pipeline/event_out', 15.0),
            transitions={'success':'PLAN_ARM_MOTION',
                            'timeout': 'STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE',
                            'failure':'STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE'})

        # pregrasp planner failed or timeout, stop the component and then return overall failure
        smach.StateMachine.add('STOP_PLAN_WHOLE_BODY_MOTION_WITH_FAILURE', send_event('/whole_body_motion_calculator_pipeline/event_in','e_stop'),
            transitions={'success':'STOP_PLAN_ARM_MOTION_WITH_FAILURE'})
        
        # based on a published pose, calls pregrasp planner to generate a graspable pose
        smach.StateMachine.add('PLAN_ARM_MOTION', send_event('/pregrasp_planner_pipeline/event_in','e_start'),
            transitions={'success':'WAIT_PLAN_ARM_MOTION'})
        
        # wait for the result of the pregrasp planner
        smach.StateMachine.add('WAIT_PLAN_ARM_MOTION', wait_for_event('/pregrasp_planner_pipeline/event_out', 15.0),
            transitions={'success':'CHECK_BASE_COLLISION',
                            'timeout': 'PLAN_WHOLE_BODY_MOTION',
                            'failure':'PLAN_WHOLE_BODY_MOTION'})
        
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION', send_event('/pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'CHECK_BASE_COLLISION'})
        
        # pregrasp planner failed or timeout, stop the component and then return overall failure
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION_WITH_FAILURE', send_event('/pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('CHECK_BASE_COLLISION', gbs.send_and_wait_events_combined(
                event_in_list=[('/base_motion_calculator/event_in','e_start')],
                                #('/base_collision_checker_pipeline/base_collision_checker_node/event_in','e_start')],
                event_out_list=[('/base_motion_calculator/event_out','e_success', True)],
                                #('/base_collision_checker_pipeline/base_collision_checker_node/event_out','e_success', True)],
                timeout_duration=30),
                transitions={'success':'STOP_PLAN_WHOLE_BODY_MOTION',
                             'timeout':'STOP_MOVE_BASE_TO_OBJECT',
                             'failure':'PLAN_WHOLE_BODY_MOTION'}) # select next pose from the sampled poses

        # wbc pipeline  was successful, so lets stop it since its work is done
        smach.StateMachine.add('STOP_PLAN_WHOLE_BODY_MOTION', send_event('/whole_body_motion_calculator_pipeline/event_in','e_stop'),
            transitions={'success':'MOVE_ROBOT_TO_OBJECT'})

        #execute robot motion
        smach.StateMachine.add('MOVE_ROBOT_TO_OBJECT', gbs.send_and_wait_events_combined(
                event_in_list=[('/move_arm_planned/event_in','e_start'),
                                ('/wbc/mcr_navigation/direct_base_controller/coordinator/event_in','e_start')],
                event_out_list=[('/move_arm_planned/event_out','e_success', True),
                                ('/mcr_navigation/collision_velocity_filter/event_out','e_zero_velocities_forwarded', False),
                                ('/wbc/mcr_navigation/direct_base_controller/coordinator/event_out','e_success', True)],
                timeout_duration=10),
                transitions={'success':'STOP_MOVE_BASE_TO_OBJECT',
                             'timeout':'STOP_MOVE_BASE_TO_OBJECT',
                             'failure':'STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE'})
        
        # send stop event_in to arm motion component and return failure
        smach.StateMachine.add('STOP_MOVE_ROBOT_TO_OBJECT_WITH_FAILURE', send_event('/move_arm_planned/event_in','e_stop'),
            transitions={'success':'OPEN_GRIPPER'})
        
        # send stop event_in to arm motion component
        smach.StateMachine.add('STOP_MOVE_BASE_TO_OBJECT', 
                        send_event('/wbc/mcr_navigation/direct_base_controller/coordinator/event_in','e_stop'),
            transitions={'success':'STOP_MOVE_ARM_TO_OBJECT'})

        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT', send_event('/move_arm_planned/event_in','e_stop'),
            transitions={'success':'OPEN_GRIPPER'})
        
        # open gripper
        smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
            transitions={'succeeded': 'MOVE_ARM_TO_HOLD'})
            
        '''
        smach.StateMachine.add('VERIFY_GRIPPER_CLOSED', gbs.send_and_wait_events_combined(
                event_in_list=[('/gripper_state_monitor/event_in','e_trigger')],
                event_out_list=[('/gripper_state_monitor/event_out','e_gripper_closed', True)],
                timeout_duration=5),
                transitions={'success':'MOVE_ARM_TO_HOLD',
                             'timeout':'CLOSE_GRIPPER',
                             'failure':'CLOSE_GRIPPER'})
        '''
        # move arm to HOLD position
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_SUCCESS', 
                                            'failed':'MOVE_ARM_TO_HOLD'})
        
        '''
        smach.StateMachine.add('VERIFY_GRASPED', gbs.send_and_wait_events_combined(
                        event_in_list=[('/gripper_controller/grasp_monitor/event_in','e_trigger')],
                        event_out_list=[('/gripper_controller/grasp_monitor/event_out','e_object_grasped', True)],
                        timeout_duration=10),
                transitions={'success':'SET_ACTION_LIB_SUCCESS',
                             'timeout':'SET_ACTION_LIB_FAILURE',
                             'failure':'SET_ACTION_LIB_FAILURE'})
        '''

        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                                transitions={'succeeded':'RESET_PREGRASP_PLANNER_INPUT_SUCCESS'})

        smach.StateMachine.add('RESET_PREGRASP_PLANNER_INPUT_SUCCESS', send_event('/mir_manipulation/mux_pregrasp_pose/select',
                                                                   '/mcr_perception/object_selector/output/object_pose'),
                                transitions={'success':'OVERALL_SUCCESS'})
                               
        # set action lib result
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                               transitions={'succeeded':'RESET_PREGRASP_PLANNER_INPUT_FAILURE'})

        smach.StateMachine.add('RESET_PREGRASP_PLANNER_INPUT_FAILURE', send_event('/mir_manipulation/mux_pregrasp_pose/select',
                                                           '/mcr_perception/object_selector/output/object_pose'),
                        transitions={'success':'OVERALL_FAILED'})
    
    # smach viewer
    sis = smach_ros.IntrospectionServer('place_object_smach_viewer', sm, '/PLACE_OBJECT_SMACH_VIEWER')
    sis.start()
    
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'place_object_in_cavity_server',
        action_spec = PlaceObjectAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'place_object_goal',
        feedback_key = 'place_object_feedback',
        result_key   = 'place_object_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()
        
if __name__ == '__main__':
    if len(sys.argv) > 1:
       main(mokeup=(sys.argv[1]=='mokeup'))
    else:
       main(mokeup=False)
