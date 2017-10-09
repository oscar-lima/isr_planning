#!/usr/bin/env python
import rospy
import smach

import smach_ros

import mir_states.common.manipulation_states as gms # move the arm, and gripper
        
# knowledge base update
#from knowledge_base_sm import UpdateKnowledgeBase

# action lib
from smach_ros import ActionServerWrapper
from mir_yb_action_msgs.msg import StageObjectAction
from mir_yb_action_msgs.msg import StageObjectFeedback
from mir_yb_action_msgs.msg import StageObjectResult
        
#===============================================================================        
        
class SetupMoveArm(smach.State): # inherit from the State base class
    def __init__(self, arm_target):
        smach.State.__init__(self, outcomes=['succeeded','failed'], 
                                   input_keys=['stage_object_goal'], 
                                   output_keys=['stage_object_feedback', 'stage_object_result', 'move_arm_to'])
        self.counter = 0
        self.arm_target = arm_target

    def execute(self, userdata):
        # updating result (false until finished all actions)
        result = StageObjectResult()
        result.success = False
        userdata.stage_object_result = result
        # get arm goal from actionlib
        platform = userdata.stage_object_goal.robot_platform
        # giving feedback to the user
        feedback = StageObjectFeedback()
        feedback.current_state = 'MOVE_ARM'
        if self.arm_target == 'pre':
            platform = platform + "_pre"
        feedback.text='[stage_object] Moving the arm to ' + platform
        userdata.move_arm_to = platform
        userdata.stage_object_feedback = feedback
        return 'succeeded'

#=============================================================================== 
       
class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self,  outcomes=['succeeded'], 
                                    input_keys=['stage_object_goal'], 
                                    output_keys=['stage_object_feedback', 'stage_object_result'])
        self.result = result

    def execute(self, userdata):
        result = StageObjectResult()
        result.success = self.result
        userdata.stage_object_result = result
        return 'succeeded'
        
        
#===============================================================================
            
def main():
    rospy.init_node('stage_object_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS','OVERALL_FAILED'],
            input_keys = ['stage_object_goal'],
            output_keys = ['stage_object_feedback', 'stage_object_result'])
    with sm:
        #add states to the container
        smach.StateMachine.add('SETUP_MOVE_ARM_PRE_STAGE', SetupMoveArm('pre'),
                transitions={'succeeded': 'MOVE_ARM_PRE_STAGE',
                             'failed': 'SETUP_MOVE_ARM_PRE_STAGE'})

        smach.StateMachine.add('MOVE_ARM_PRE_STAGE', gms.move_arm(),
                transitions={'succeeded': 'SETUP_MOVE_ARM_STAGE',
                             'failed': 'MOVE_ARM_PRE_STAGE'})
                             
        smach.StateMachine.add('SETUP_MOVE_ARM_STAGE', SetupMoveArm('final'),
                transitions={'succeeded': 'MOVE_ARM_STAGE',
                             'failed': 'SETUP_MOVE_ARM_STAGE'})

        smach.StateMachine.add('MOVE_ARM_STAGE', gms.move_arm(),
                transitions={'succeeded': 'OPEN_GRIPPER',
                             'failed': 'MOVE_ARM_STAGE'})
                             
        smach.StateMachine.add('OPEN_GRIPPER', gms.control_gripper('open_release'),
                transitions={'succeeded': 'MOVE_ARM_TO_HOLD'})
                             
        smach.StateMachine.add('MOVE_ARM_TO_HOLD', gms.move_arm("look_at_turntable"), 
                               transitions={'succeeded':'SET_ACTION_LIB_SUCCESS', 
                                            'failed':'MOVE_ARM_TO_HOLD'})

        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True), 
                               transitions={'succeeded':'OVERALL_SUCCESS'})

    # smach viewer
    sis = smach_ros.IntrospectionServer('stage_object_smach_viewer', sm, '/STAGE_OBJECT_SMACH_VIEWER')
    sis.start()
                                            
    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name = 'stage_object_server', 
        action_spec = StageObjectAction, 
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes   = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key     = 'stage_object_goal',
        feedback_key = 'stage_object_feedback',
        result_key   = 'stage_object_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()
