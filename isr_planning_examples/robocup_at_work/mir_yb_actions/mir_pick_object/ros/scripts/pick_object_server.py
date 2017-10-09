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
from mir_yb_action_msgs.msg import PickObjectAction
from mir_yb_action_msgs.msg import PickObjectFeedback
from mir_yb_action_msgs.msg import PickObjectResult
        
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

class wait_for_event(smach.State):
    '''
    This state will take a event name as input and waits for the event to
    be published.
    '''
    def __init__(self, topic_name, timeout_duration):
        smach.State.__init__(self,  outcomes=['success', 'failure', 'timeout'],
                                    input_keys=['pick_object_goal'], 
                                    output_keys=['pick_object_feedback', 'pick_object_result'])
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
        feedback = PickObjectFeedback()
        feedback.current_state = 'WAIT_FOR_FEEDBACK'
        userdata.pick_object_feedback = feedback
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

def main(mokeup=False):
    # Open the container
    rospy.init_node('pick_object_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['pick_object_goal'],
            output_keys = ['pick_object_feedback', 'pick_object_result'])
    with sm:
        if not mokeup:
            # publish object as string to mcr_perception_selectors -> object_selector, this component then publishes
            # pose in base_link reference frame when e_trigger is sent (next state)
            smach.StateMachine.add('SELECT_OBJECT', select_object('/mcr_perception/object_selector/input/object_name'),
                transitions={'success':'OPEN_GRIPPER'})
            
            # open gripper
            smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded': 'GENERATE_OBJECT_POSE'})
            
            # generates a pose based on the previous string object topic received
            smach.StateMachine.add('GENERATE_OBJECT_POSE', send_event('/mcr_perception/object_selector/event_in', 'e_trigger'),
                transitions={'success':'CHECK_IF_OBJECT_IS_AVAILABLE'})

            # waits for object selector response, if failure this means that the string published previously was not found in object list
            smach.StateMachine.add('CHECK_IF_OBJECT_IS_AVAILABLE', wait_for_event('/mcr_perception/object_selector/event_out', 1.0),
                transitions={'success':'RECONFIGURE_PREGRASP_PARAMS',
                                'timeout': 'SET_ACTION_LIB_FAILURE',
                                'failure':'SET_ACTION_LIB_FAILURE'})
     
            # reconfigure pregrasp planner for pick
            smach.StateMachine.add('RECONFIGURE_PREGRASP_PARAMS', gbs.set_named_config('pick_normal_pregrasp'),
                transitions={'success':'PLAN_ARM_MOTION',
                             'failure':'SET_ACTION_LIB_FAILURE',
                             'timeout': 'RECONFIGURE_PREGRASP_PARAMS'})
                            
        # based on a published pose, calls pregrasp planner to generate a graspable pose and wait for the result of the pregrasp planner
        smach.StateMachine.add('PLAN_ARM_MOTION', gbs.send_and_wait_events_combined(
                event_in_list=[('/pregrasp_planner_pipeline/event_in','e_start')],
                event_out_list=[('/pregrasp_planner_pipeline/event_out','e_success', True)],
                timeout_duration=10),
                transitions={'success':'STOP_PLAN_ARM_MOTION',
                             'timeout':'STOP_PLAN_ARM_MOTION_WITH_FAILURE',
                             'failure':'STOP_PLAN_ARM_MOTION_WITH_FAILURE'})
        
        # pregrasp planner was successful, so lets stop it since its work is done
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION', send_event('/pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'MOVE_ARM_TO_OBJECT'})
        
        # pregrasp planner failed or timeout, stop the component and then return overall failure
        smach.StateMachine.add('STOP_PLAN_ARM_MOTION_WITH_FAILURE', send_event('/pregrasp_planner_pipeline/event_in','e_stop'),
            transitions={'success':'SET_ACTION_LIB_FAILURE'})
                            
        # move arm to pregrasp planned pose and wait for its response
        smach.StateMachine.add('MOVE_ARM_TO_OBJECT', gbs.send_and_wait_events_combined(
                event_in_list=[('/move_arm_planned/event_in','e_start')],
                event_out_list=[('/move_arm_planned/event_out','e_success', True)],
                timeout_duration=7),
                transitions={'success':'STOP_MOVE_ARM_TO_OBJECT',
                             'timeout':'STOP_MOVE_ARM_TO_OBJECT',
                             'failure':'STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE'})
        
        # send stop event_in to arm motion component and return failure
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT_WITH_FAILURE', send_event('/move_arm_planned/event_in','e_stop'),
            transitions={'success':'SET_ACTION_LIB_FAILURE'})
        
        # send stop event_in to arm motion component
        smach.StateMachine.add('STOP_MOVE_ARM_TO_OBJECT', send_event('/move_arm_planned/event_in','e_stop'),
            transitions={'success':'CLOSE_GRIPPER'})
        
        # close gripper
        smach.StateMachine.add('CLOSE_GRIPPER', gms.control_gripper('close'),
                transitions={'succeeded': 'MOVE_ARM_TO_HOLD'})
                
        # check if gripper changed in position, if not then close again
#        smach.StateMachine.add('VERIFY_GRIPPER_CLOSED', gbs.send_and_wait_events_combined(
#                event_in_list=[('/gripper_state_monitor/event_in','e_trigger')],
#                event_out_list=[('/gripper_state_monitor/event_out','e_gripper_closed', True)],
#                timeout_duration=5),
#                transitions={'success':'MOVE_ARM_TO_HOLD',
#                             'timeout':'CLOSE_GRIPPER',
#                             'failure':'CLOSE_GRIPPER'})
        
        # move arm to HOLD position
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_SUCCESS', 
                                            'failed':'MOVE_ARM_TO_HOLD'})

        # optocoupler check if robot has a object in the gripper, if not then pick failed
#        smach.StateMachine.add('VERIFY_OBJECT_GRASPED_OPTOCOUPLER', gbs.send_and_wait_events_combined(
#                event_in_list=[('/optocoupler_gripper_status/event_in','e_trigger')],
#                event_out_list=[('/optocoupler_gripper_status/event_out','e_grasped', True)],
#                timeout_duration=5),
#                transitions={'success':'SET_ACTION_LIB_SUCCESS',
#                             'timeout':'SET_ACTION_LIB_FAILURE',
#                             'failure':'SET_ACTION_LIB_FAILURE'})
 
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
       main(mokeup=(sys.argv[1]=='mokeup'))
    else:
       main(mokeup=False)
