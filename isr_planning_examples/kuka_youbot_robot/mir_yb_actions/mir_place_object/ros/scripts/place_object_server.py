#!/usr/bin/env python
import rospy
import smach

import smach_ros
import std_msgs.msg

import mir_states.common.navigation_states as gns
import mir_states.common.manipulation_states as gms # move the arm, and gripper
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import PlaceObjectAction
from mir_yb_action_msgs.msg import PlaceObjectFeedback
from mir_yb_action_msgs.msg import PlaceObjectResult
        
#=============================================================================== 
class GetPoseToPlaceOject(smach.State): # inherit from the State base class
    def __init__(self, topic_name_pub, topic_name_sub, event_sub, timeout_duration):
        smach.State.__init__(self, outcomes=['succeeded','failed'], 
                                   input_keys=['place_object_goal'], 
                                   output_keys=['place_object_feedback', 'place_object_result', 'move_arm_to'])
        
        self.timeout = rospy.Duration.from_sec(timeout_duration)
        # create publisher
        self.topic_name_pub = topic_name_pub
        self.event_sub = event_sub
        self.platform_name_pub = rospy.Publisher(
            self.topic_name_pub, std_msgs.msg.String, queue_size=10)
        self.topic_name_sub = topic_name_sub
        rospy.Subscriber(self.topic_name_sub, std_msgs.msg.String, self.pose_cb)
        rospy.Subscriber(self.event_sub, std_msgs.msg.String, self.event_cb)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)
        self.place_pose = None
        self.message_received = False
        self.status_received = False
        self.status = None

    def pose_cb(self, msg):
        self.place_pose = msg
        self.message_received = True

    def event_cb(self, msg):
        self.status = msg
        self.status_received = True

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = PlaceObjectResult()
        result.success = False
        userdata.place_object_result = result
        # get arm goal from actionlib
        obj = userdata.place_object_goal.object
        location = userdata.place_object_goal.location
        object_location = std_msgs.msg.String() 
        object_location.data = location
        self.platform_name_pub.publish(object_location)
        rospy.loginfo('publishing on ' + self.topic_name_pub + ' : ' + object_location.data)
        # giving feedback to the user
        feedback = PlaceObjectFeedback()
        feedback.current_state = 'GET_POSE_TO_PLACE_OBJECT'
        rospy.sleep(0.2)
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10) # 10hz
        # wait for message to arrive
        while((rospy.Time.now() - start_time < self.timeout) and not(self.message_received) 
            and not(self.status_received) and not(rospy.is_shutdown())):
            rate.sleep()
        # dont kill the node so quickly, wait for handshake of nodes
        print("(rospy.Time.now() - start_time < self.timeout) = {}".format((rospy.Time.now() - start_time < self.timeout)))
        print("self.message_received = {}".format((self.message_received)))
        print("self.Status received = {}".format((self.status_received)))
        #rospy.sleep(0.2)  
        if self.message_received and self.status.data == 'e_success':
            userdata.move_arm_to = self.place_pose.data
            return 'succeeded'
        else:
            return 'failed'

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
        self.publisher = rospy.Publisher(self.topic_name, std_msgs.msg.String, queue_size=10)
        # giving some time to the publisher to register in ros network
        rospy.sleep(0.1)

    def execute(self, userdata):
        # give feedback
        feedback = PlaceObjectFeedback()
        feedback.current_state = 'SEND_EVENT'
        userdata.place_object_feedback = feedback
        # creating string message
        msg = std_msgs.msg.String()
        # filling message
        msg.data = self.event
        # publish
        self.publisher.publish(msg)
        rospy.loginfo('publishing on ' + self.topic_name + ' ' + self.event)
        # wait, dont kill the node so quickly
        rospy.sleep(0.2)
        return 'success'
        
#===============================================================================
        
#===============================================================================        

class UpdateKnowledgeBase(smach.State): # inherit from the State base class
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded','failed'], 
                                    input_keys=['place_object_goal'], 
                                    output_keys=['place_object_feedback', 'place_object_result'])
        self.counter = 0

    def execute(self, userdata):
        # updating result
        result = PlaceObjectResult()
        result.success = True
        userdata.place_object_result = result
        return 'succeeded'

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

def main():
    rospy.init_node('place_object_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['place_object_goal'],
            output_keys = ['place_object_feedback', 'place_object_result'])
    with sm:
        #add states to the container
        smach.StateMachine.add('START_PLACE_POSE_SELECTOR', send_event
            ('/mcr_perception/place_pose_selector/event_in','e_start'),
                transitions={'success':'GET_POSE_TO_PLACE_OBJECT'})

        smach.StateMachine.add('GET_POSE_TO_PLACE_OBJECT', GetPoseToPlaceOject(
            '/mcr_perception/place_pose_selector/platform_name',
            '/mcr_perception/place_pose_selector/place_pose', 
            '/mcr_perception/place_pose_selector/event_out',15.0),
                transitions={'succeeded': 'MOVE_ARM_TO_PLACE_OBJECT',
                             'failed': 'SET_ACTION_LIB_FAILURE'})

        smach.StateMachine.add('MOVE_ARM_TO_PLACE_OBJECT', gms.move_arm(),
                transitions={'succeeded': 'OPEN_GRIPPER',
                             'failed': 'OPEN_GRIPPER'})
                             
        smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open'),
                transitions={'succeeded': 'MOVE_ARM_TO_HOLD'})
                             
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_SUCCESS', 
                                            'failed':'MOVE_ARM_TO_HOLD'})
                                            
        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                               transitions={'succeeded':'OVERALL_SUCCESS'})
        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False), 
                               transitions={'succeeded':'OVERALL_FAILED'})
    
    # smach viewer
    sis = smach_ros.IntrospectionServer(
        'place_object_smach_viewer', sm, '/STAGE_OBJECT_SMACH_VIEWER')
    sis.start()
                                            
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'place_object_server', 
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
    main()
