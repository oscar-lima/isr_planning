#!/usr/bin/env python
import rospy
import smach

# move the arm and base
import mir_states.common.manipulation_states as gms # move the arm
        
# for send and receive event combined
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import MoveBaseSafeAction
from mir_yb_action_msgs.msg import MoveBaseSafeFeedback
from mir_yb_action_msgs.msg import MoveBaseSafeResult

# to publish pose stamped from param server
from geometry_msgs.msg import PoseStamped
import param_server_utils

#===============================================================================

class SetupMoveArm(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded','failed'], 
                                    input_keys=['move_base_safe_goal'], 
                                    output_keys=['move_base_safe_feedback', 'move_base_safe_result', 'move_arm_to'])

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = MoveBaseSafeResult()
        result.success = False
        userdata.move_base_safe_result = result
        # get arm goal from actionlib
        #arm_goal = userdata.move_base_safe_goal.arm_safe_position
        # harcoded for now, just to be safe
        #arm_goal = 'folded'
        arm_goal = 'folded'
        # giving feedback to the user
        feedback = MoveBaseSafeFeedback()
        feedback.current_state = 'MOVE_ARM'
        feedback.text='[move_base_safe] Moving the arm to ' + arm_goal
        userdata.move_base_safe_feedback = feedback
        userdata.move_arm_to = arm_goal
        return 'succeeded'
        
#===============================================================================        
        
class SetupMoveBase(smach.State): # inherit from the State base class
    """
    Obtains pre nav goal from action lib and publishes a pose stamped to the specified topic
    """
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['succeeded','failed','preempted'], 
                                    input_keys=['move_base_safe_goal'], 
                                    output_keys=['move_base_safe_feedback', 'move_base_safe_result'])
        self.pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        # give some time for the publisher to get registered, remove if not used
        #rospy.sleep(0.1)

    def execute(self, userdata):
        # Check for preempt                                                                                             
        if self.preempt_requested():
            rospy.logwarn('preemption requested!!!')
            # reset preemption flag for next request
            self.recall_preempt()
            return 'preempted'
        # get base goal from actionlib
        base_goal = userdata.move_base_safe_goal.destination_location
        base_orientation = userdata.move_base_safe_goal.destination_orientation
        # converting base goal to uppercase since mercury transforms everything to lower case
        base_goal = base_goal.upper()
        base_orientation = base_orientation.upper()
        
        pose = None
        if userdata.move_base_safe_goal.use_destination_pose:
            pose = userdata.move_base_safe_goal.destination_pose
        else:
            pose = param_server_utils.get_pose_from_param_server(base_goal)
            if base_orientation != "":
                pose.pose.orientation = param_server_utils.get_orientation_from_param_server(base_orientation)        

        # giving feedback to the user
        feedback = MoveBaseSafeFeedback()
        feedback.current_state = 'MOVE_BASE'
        feedback.text = '[move_base_safe] moving the base to ' + base_goal
        userdata.move_base_safe_feedback = feedback
        if pose:
            #publish pose stamped nav goal
            self.pub.publish(pose)
            return 'succeeded'
        else:
            return 'failed' 
        
       
#=============================================================================== 
       
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['move_base_safe_goal'], 
                                    output_keys=['move_base_safe_feedback', 'move_base_safe_result'])
        self.result = result

    def execute(self, userdata):
        result = MoveBaseSafeResult()
        result.success = self.result
        userdata.move_base_safe_result = result
        return 'succeeded'
        
        
#===============================================================================

def main():
    rospy.init_node('move_base_safe_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED','OVERALL_PREEMPTED'],
            input_keys = ['move_base_safe_goal'],
            output_keys = ['move_base_safe_feedback', 'move_base_safe_result'])
    with sm:
        #add states to the container
        smach.StateMachine.add('SETUP_MOVE_ARM', SetupMoveArm(),
                transitions={'succeeded': 'MOVE_ARM',
                             'failed': 'SETUP_MOVE_ARM'})

        smach.StateMachine.add('MOVE_ARM', gms.move_arm(),
                transitions={'succeeded': 'SETUP_MOVE_BASE',
                             'failed': 'MOVE_ARM'})

        # get pose from action lib as string, convert to pose stamped and publish
        smach.StateMachine.add('SETUP_MOVE_BASE', SetupMoveBase('/move_base_wrapper/pose_in'),
                transitions={'succeeded': 'MOVE_BASE',
                             'failed': 'SET_ACTION_LIB_FAILURE',
                             'preempted':'OVERALL_PREEMPTED'})
        
        # send event_in to move base to a pose
        smach.StateMachine.add('MOVE_BASE', gbs.send_and_wait_events_combined(
                event_in_list=[('/move_base_wrapper/event_in','e_start')],
                event_out_list=[('/move_base_wrapper/event_out','e_success', True)],
                timeout_duration=50),
                transitions={'success':'ADJUST_BASE',
                             'timeout':'SET_ACTION_LIB_FAILURE',
                             'failure':'SETUP_MOVE_BASE'})
        
        # call direct base controller to fine adjust the base to the final desired pose 
        # (navigation tolerance is set to a wide tolerance)
        smach.StateMachine.add('ADJUST_BASE', gbs.send_and_wait_events_combined(
                event_in_list=[('/mcr_navigation/direct_base_controller/coordinator/event_in','e_start')],
                event_out_list=[('/mcr_navigation/direct_base_controller/coordinator/event_out','e_success', True)],
                timeout_duration=5), # this is a tradeoff between speed and accuracy, set a higher value for accuracy increase
                transitions={'success':'STOP_CONTROLLER_WITH_SUCCESS',
                             'timeout':'STOP_CONTROLLER_WITH_SUCCESS',
                             'failure':'STOP_CONTROLLER_WITH_FAILURE'})
        
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
                               #transitions={'succeeded':'RESET_STATIC_TRANSFORM_FOR_PERCEPTION'})
                               transitions={'succeeded':'OVERALL_SUCCESS'})

        # generates a pose based on the previous string object topic received
        #smach.StateMachine.add('RESET_STATIC_TRANSFORM_FOR_PERCEPTION', gbs.send_event([('/static_transform_publisher_node/event_in', 'e_start')]),
        #         transitions={'success':'OVERALL_SUCCESS'})
                               
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                               transitions={'succeeded':'OVERALL_FAILED'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'move_base_safe_server', 
        action_spec = MoveBaseSafeAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['OVERALL_PREEMPTED'],
        goal_key     = 'move_base_safe_goal',
        feedback_key = 'move_base_safe_feedback',
        result_key   = 'move_base_safe_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
