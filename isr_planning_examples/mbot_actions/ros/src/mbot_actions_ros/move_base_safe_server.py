#!/usr/bin/env python
import rospy
import smach
        
# for send and receive event combined
import mcr_states.common.basic_states as gbs

# common states
import mbot_states.common_states as cs

# action lib
from smach_ros import ActionServerWrapper
from mbot_action_msgs.msg import MoveBaseSafeAction
from mbot_action_msgs.msg import MoveBaseSafeFeedback
from mbot_action_msgs.msg import MoveBaseSafeResult

# to publish pose stamped from param server
from geometry_msgs.msg import PoseStamped
import mbot_actions_ros.param_server_utils as param_server_utils

from std_msgs.msg import String, UInt8

class SetupMoveBase(smach.State): # inherit from the State base class
    """
    Obtains pre nav goal from action lib and publishes a pose stamped to the specified topic
    """
    def __init__(self, topic_name):
        smach.State.__init__(self,  outcomes=['succeeded','failed','preempted'], 
                                    input_keys=['move_base_safe_goal'], 
                                    output_keys=['move_base_safe_feedback', 'move_base_safe_result'])
        self.pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
        # to speak about which pose is the robot visiting
        self.pub_text_to_speech = rospy.Publisher("say", String, queue_size=1)
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
        # speak each time the robot calls move_base_safe server
        self.pub_text_to_speech.publish(userdata.move_base_safe_goal.stucked_speech)
        rospy.sleep(1.0)
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

class CounterHandler(smach.State):
    def __init__(self, reset):
        smach.State.__init__(self,  outcomes=['retry','noretry'])
        self.counter = 0
        self.reset = reset
        rospy.set_param('move_base_retry', 5)
        global counter
        counter = UInt8()
        counter = 0
    def execute(self, userdata):
        global counter
        limit = rospy.get_param('/move_base_retry')
        if self.reset:
            counter = 0
            return 'noretry'
        else:
            if counter >= limit:
                counter = 0
                return 'noretry'
            else:
                counter = counter +1
                rospy.loginfo("retry number " + str(counter))
                return 'retry'
            
def start_move_base_safe_server():
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED','OVERALL_PREEMPTED'],
            input_keys = ['move_base_safe_goal'],
            output_keys = ['move_base_safe_feedback', 'move_base_safe_result'])
    with sm:
        # get pose from action lib as string, convert to pose stamped and publish
        smach.StateMachine.add('SETUP_MOVE_BASE', SetupMoveBase('move_base_wrapper/pose_in'),
                transitions={'succeeded': 'SET_ROBOT_FACE_TO_SURPRISE',
                             'failed': 'SET_ACTION_LIB_FAILURE',
                             'preempted':'OVERALL_PREEMPTED'})
        
        # set a Surprise face on the robot (just for fun)
        smach.StateMachine.add('SET_ROBOT_FACE_TO_SURPRISE', cs.PublishString('led_emotions_interface/emotions','Surprise'),
                transitions={'success': 'MOVE_BASE'})
        
        # send event_in to move base to a pose
        smach.StateMachine.add('MOVE_BASE', gbs.send_and_wait_events_combined(
                event_in_list=[('move_base_wrapper/event_in','e_start')],
                event_out_list=[('move_base_wrapper/event_out','e_success', True)],
                timeout_duration=50),
                transitions={'success':'SET_ROBOT_FACE_TO_HAPPINESS',
                             'timeout':'SET_ROBOT_FACE_TO_FEAR',
                             'failure':'SET_ROBOT_FACE_TO_ANGER'})
        
        # set a Happiness face on the robot (robot is happy to achieve the goal)
        smach.StateMachine.add('SET_ROBOT_FACE_TO_HAPPINESS', cs.PublishString('led_emotions_interface/emotions','Happiness'),
                transitions={'success': 'SUCCESS_HANDLE_COUNTER'})
        
        # set a Anger face on the robot (robot is angry because goal could not be achieved but will try again)
        smach.StateMachine.add('SET_ROBOT_FACE_TO_ANGER', cs.PublishString('led_emotions_interface/emotions','Anger'),
                transitions={'success': 'FAILURE_HANDLE_COUNTER'})

        smach.StateMachine.add('FAILURE_HANDLE_COUNTER', CounterHandler(False),
                transitions={'retry': 'SETUP_MOVE_BASE',
                             'noretry': 'SET_ROBOT_FACE_TO_FEAR'})

        smach.StateMachine.add('SUCCESS_HANDLE_COUNTER', CounterHandler(True),
                transitions={'retry': 'SET_ACTION_LIB_FAILURE',
                             'noretry': 'SET_ACTION_LIB_SUCCESS'})
                                
        # set a Fear face on the robot (robot is scared because goal could not be achieved)
        smach.StateMachine.add('SET_ROBOT_FACE_TO_FEAR', cs.PublishString('led_emotions_interface/emotions','Fear'),
                transitions={'success': 'SET_ACTION_LIB_FAILURE'})
        
        # set the outcome of the server to success
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', cs.SetActionLibResult(True, MoveBaseSafeResult, MoveBaseSafeFeedback, ['move_base_safe_goal'], ['move_base_safe_feedback', 'move_base_safe_result']),
                               transitions={'success':'OVERALL_SUCCESS'})
                               
        # set the outcome of the server to failure
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', cs.SetActionLibResult(False, MoveBaseSafeResult, MoveBaseSafeFeedback, ['move_base_safe_goal'], ['move_base_safe_feedback', 'move_base_safe_result']),
                               transitions={'success':'OVERALL_FAILED'})

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

def main():
    rospy.init_node('move_base_safe_server', anonymous=False)
    start_move_base_safe_server()
    global counter

    
