#! /usr/bin/env python
import rospy
import actionlib

# /plan topic
from rosplan_dispatch_msgs.msg import CompletePlan

from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mir_yb_action_msgs.msg import StageObjectAction, StageObjectGoal
from mir_yb_action_msgs.msg import UnStageObjectAction, UnStageObjectGoal
from mir_yb_action_msgs.msg import InsertObjectAction, InsertObjectGoal
from mir_yb_action_msgs.msg import PickObjectAction, PickObjectGoal
from mir_yb_action_msgs.msg import PlaceObjectAction, PlaceObjectGoal
from mir_yb_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal
from mir_yb_action_msgs.msg import PickObjectWBCAction, PickObjectWBCGoal

import rosplan_knowledge_msgs.msg as rosplan_knowledge
import rosplan_knowledge_msgs.srv as rosplan_service

from diagnostic_msgs.msg import KeyValue

from std_msgs.msg import String

rospy.init_node('planner_executor_node')
event_out_pub = rospy.Publisher('~event_out', String, queue_size=10)
wiggle_base_pub = rospy.Publisher('/mcr_navigation/wiggle_base/event_in', String)

# using sys.args
import sys

#if sys.argv[1].lower() == 'true':
#    mockup = True
#else:
mockup = False
    
def action_outcome_mockup(client_name):
    """
    Used for planner executor mockup, user is asked with desired action outcome which
    is the input from keyboard
    """
    print('Executing action ' + client_name + 'mockup!!!')
    response = raw_input('Do you want the action to succed or fail (success s , failed f) :')
    if response.startswith('s'):
        print('Outcome = ' + str(response) + ' succeded !!!')
        return True
    else:
        print('Outcome = ' + str(response) + ' failed !!!')
        return False
    rospy.logerr('This message should not ever appear, something is really wrong with the code')
    return None


def rosplan_update_knowledge(knowledge_type, instance_type, instance_name, attribute_name, values, function_value=0.0, update_type='ADD_KNOWLEDGE'):
    """
    rosplan_sc_add_knowledge = rosplan service call add knowledge
    
    This function receives knowledge parameters and does a service call
    to rosplan knowledge base (mongodb)
    
    example 1:
    # upload instance : youbot has one dynamixel gripper (dynamixel - gripper)
    rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0 # ADD_KNOWLEDGE = 0
        knowledge:
        knowledge_type: 0 # INSTANCE = 0
        instance_type: 'gripper'
        instance_name: 'dynamixel'
        attribute_name: ''
        values:
        -{}
        function_value: 0.0";
    # (on o4 S3)
    
    example 2: object4 (o4) is on location S3
    # upload fact : 
    rosservice call /kcl_rosplan/update_knowledge_base "update_type: 0 # ADD_KNOWLEDGE = 0
        knowledge:
        knowledge_type: 1 # FACT = 1
        instance_type: ''
        instance_name: ''
        attribute_name: 'on'
        values:
        - {key: 'o', value: 'o4'}
        - {key: 'l', value: 'S3'}
        function_value: 0.0";
    
    """
    msg = rosplan_knowledge.KnowledgeItem()
    if knowledge_type == 0:
        # instance
        msg.knowledge_type = 0 # INSTANCE (rosplan_knowledge_msgs/msg/KnowledgeItem.msg)
        msg.instance_type = instance_type
        msg.instance_name = instance_name
        msg.attribute_name = ''
    elif knowledge_type == 1:
        # fact
        msg.knowledge_type = 1 # FACT (rosplan_knowledge_msgs/msg/KnowledgeItem.msg)
        msg.instance_type = ''
        msg.instance_name = ''
        msg.attribute_name = attribute_name
        for each in values:
            msg.values.append(KeyValue(each[0], each[1]))
        #remove
        print '--------- updating knowledge : ------------'
        print update_type + ':'
        print attribute_name
        print msg.values
        print '---------------------'
    else:
        rospy.logerr('error: no information will be uploaded, Knowledge type should be INSTANCE = 0, or FACT = 1')
        return False
    msg.function_value = function_value
    rospy.logdebug('======msg=========')
    rospy.logdebug(msg)
    rospy.logdebug('================')
    rospy.loginfo('Waiting for /kcl_rosplan/update_knowledge_base to become available...')
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    try:
        update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', rosplan_service.KnowledgeUpdateService)
        #get response
        if update_type == 'ADD_KNOWLEDGE':
            response = update_kb(rosplan_service.KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, msg)
        elif update_type == 'ADD_GOAL':
            response = update_kb(rosplan_service.KnowledgeUpdateServiceRequest.ADD_GOAL, msg)
        elif update_type == 'REMOVE_KNOWLEDGE':
            response = update_kb(rosplan_service.KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE, msg)
        elif update_type == 'REMOVE_GOAL':
            response = update_kb(rosplan_service.KnowledgeUpdateServiceRequest.REMOVE_GOAL, msg)
        else:
            rospy.logerr('Error : Unknown update_type, admisible values are ADD_KNOWLEDGE, ADD_GOAL, REMOVE_KNOWLEDGE and REMOVE_GOAL')
            rospy.logwarn('Knowledge base not updated...!')
            return False
    except rospy.ServiceException, e:
        rospy.logerr('Service call failed: %s'%e)
    if(response.success):
        rospy.loginfo('Knowledge base successfully updated')
        # service call succesful
        return True
    else:
        rospy.logerror('Could not update world model, failed')
        # service call failed
        return False
        

def replan():
    rospy.logwarn('Some component reported failure, therefore replan is needed')
    msg = String()
    msg.data = 'e_failure'
    event_out_pub.publish(msg)
    return False

    
def publish_success():
    rospy.loginfo('Plan succesfully completed, everyone is happy now : )')
    msg = String()
    msg.data = 'e_success'
    event_out_pub.publish(msg)
    return False
    
    
def move_base_safe(arm_safe_position, current_robot_location, destination, duration=150.0):
    """
    example:
    arm_safe_position : "folded"
    current_robot_location : "START"
    destination : "S1"
    """
    print('move_base_safe ' + current_robot_location + ' ' + destination)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('move_base_safe')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
        client.wait_for_server()
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = arm_safe_position
        goal.source_location = current_robot_location
        goal.destination_location = destination
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True
     

def stage_object(robot_platform, duration=150.0):
    """
    robot_platform can be:
    platform_left
    platform_middle
    platform_right
    """
    print('stage_object ' + robot_platform)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('stage_object')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('stage_object_server', StageObjectAction)
        client.wait_for_server()
        goal = StageObjectGoal()
        goal.robot_platform = robot_platform
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True


def unstage_object(robot_platform, duration=150.0):
    """
    robot_platform can be:
    platform_left
    platform_middle
    platform_right
    """
    print('unstage_object ' + robot_platform)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('unstage_object')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('unstage_object_server', UnStageObjectAction)
        client.wait_for_server()
        goal = UnStageObjectGoal()
        goal.robot_platform = robot_platform
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True


def insert_object(o1, o2, duration=150.0):
    """
    o1, o2 are strings, they describe the objects which needs to be inserted
    inserts object1 into object2
    """
    print('insert_object ' + o1 + ' in ' + o2)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('insert_object')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('insert_object_server', InsertObjectAction)
        client.wait_for_server()
        goal = InsertObjectGoal()
        goal.peg = o1.upper()
        goal.hole = o2.upper()
        goal.hole += "-insert"
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True


def pick_object(obj, duration=150.0):
    """
    obj is a string describing the object which needs to be picked
    """
    print('pick_object ' + obj)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('pick_object')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('pick_object_server', PickObjectAction)
        client.wait_for_server()
        goal = PickObjectGoal()
        goal.object = obj
        #goal.object += "-pick"
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True


def pick_object_wbc(obj, duration=150.0):
    """
    obj is a string describing the object which needs to be picked
    """
    print('pick_object ' + obj)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('pick_object')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('wbc_pick_object_server', PickObjectWBCAction)
        client.wait_for_server()
        goal = PickObjectWBCGoal()
        goal.object = obj
        #goal.object += "-pick"
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True

def perceive_location(location, duration=150.0):
    """
    location
    """
    print('perceive_location ' + location)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('perceive_location')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('perceive_location_server', PerceiveLocationAction)
        client.wait_for_server()
        goal = PerceiveLocationGoal()
        goal.location = location
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True
        
        
def place_object(obj, location, duration=150.0):
    """
    obj is to be placed on location
    """
    print('place_object ' + obj)
    # mockup begins----
    if mockup:
        mockup_result = action_outcome_mockup('place_object')
        if mockup_result:
            return True
        else:
            return replan()
    else:
        client = actionlib.SimpleActionClient('place_object_server', PlaceObjectAction)
        client.wait_for_server()
        goal = PlaceObjectGoal()
        goal.object = obj
        goal.location = location
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(duration))
        rospy.loginfo('action server finished with the following response : ' + str(client.get_result().success))
        if client.get_result().success == False:
            return replan()
        else:
            return True
        
            
def update_kb_move_base_safe(source, destination, robot, success=True):
    """
    1.  youbot is no longer at start if accion succeded removing this predicate from knowledge base
    2.  robot acomplished the given goal removing this goal from pending goals (in case it is there because
        it might be the case that this is a sub goal, but if you try to remove something that is not there nothing happens)
    3.  add knowledge, now youbot is at destination
    4.  when you move the base the perception is lost, this must be updated to knowledge base
        (not (perceived ?source))
    """
    if success:
        params = [['r', robot],['l', source]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'at', params, update_type='REMOVE_KNOWLEDGE')
        params = [['r', robot],['l', destination]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'at', params, update_type='REMOVE_GOAL')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'at', params, update_type='ADD_KNOWLEDGE')
        params = [['l', source]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='REMOVE_KNOWLEDGE')
        params = [['l', destination]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='REMOVE_KNOWLEDGE')
    else:
        #TODO!!
        pass
    
def update_kb_pick_object(obj, location, gripper, success=True):
    if success:
        params = [['g', gripper],['o', obj]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params, update_type='REMOVE_GOAL')
        params = [['o', obj],['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params, update_type='REMOVE_KNOWLEDGE')
        params = [['g', gripper]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='REMOVE_KNOWLEDGE')
        params = [['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='REMOVE_KNOWLEDGE')
    else:
        params = [['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='REMOVE_KNOWLEDGE')
        wiggle_base_pub.publish('e_trigger')
    
def update_kb_insert_object(peg, hole, gripper, location, success=True):
    if success:
        params = [['g', gripper],['o', peg]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params, update_type='REMOVE_KNOWLEDGE')
        params = [['g', gripper]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='REMOVE_GOAL')
        params = [['peg', peg],['hole', hole]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'in', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'in', params, update_type='REMOVE_GOAL') # knowledge_type, instance_type, instance_name, attribute_name, values, function_value=0.0, update_type='ADD_KNOWLEDGE'
        params = [['peg', peg],['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params, update_type='REMOVE_GOAL')
        params = [['peg', peg]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'heavy', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'heavy', params, update_type='REMOVE_GOAL')
        params = [['hole', hole]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'heavy', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'heavy', params, update_type='REMOVE_GOAL')
    else:
        params = [['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='REMOVE_KNOWLEDGE')
        wiggle_base_pub.publish('e_trigger')
    
def update_kb_stage_object(obj, robot_platform, gripper, success=True):
    if success:
        params = [['g', gripper],['o', obj]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params, update_type='REMOVE_KNOWLEDGE')
        params = [['g', gripper]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='REMOVE_GOAL')
        params = [['o', obj],['rp', robot_platform]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'stored', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'stored', params, update_type='REMOVE_GOAL')
        params = [['rp', robot_platform]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'occupied', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'occupied', params, update_type='REMOVE_GOAL')
    else:
        #TODO!!
        pass
    
def update_kb_unstage_object(obj, robot_platform, gripper, success=True):
    if success:
        params = [['g', gripper]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='REMOVE_KNOWLEDGE')
        params = [['o', obj],['rp', robot_platform]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'stored', params, update_type='REMOVE_KNOWLEDGE')
        params = [['rp', robot_platform]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'occupied', params, update_type='REMOVE_KNOWLEDGE')
        params = [['g', gripper], ['o', obj]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params, update_type='REMOVE_GOAL')
    else:
        #TODO!!
        pass

def update_kb_perceive(location, success=True):
    if success:
        params = [['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='REMOVE_GOAL')
    else:
        wiggle_base_pub.publish('e_trigger')
        client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
        client.wait_for_server()
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'
        try:
            goal.source_location = "dummy_source"
            goal.destination_location = location
            timeout = 15.0
            rospy.loginfo('Sending action lib goal to move_base_safe_server, source : ' + goal.source_location + ' , destination : ' + goal.destination_location)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
            print client.get_result()
        except:
            pass
    
def update_kb_place_object(obj, location, gripper, success=True):
    if success:
        params = [['o', obj],['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params, update_type='REMOVE_GOAL')
        params = [['g', gripper],['o', obj]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params, update_type='REMOVE_KNOWLEDGE')
        params = [['g', gripper]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='ADD_KNOWLEDGE')
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'gripper_is_free', params, update_type='REMOVE_GOAL')
        params = [['l', location]]
        rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='REMOVE_KNOWLEDGE')
    else:
        #TODO!!
        pass
    
def planCallBack(complete_plan):
    rospy.loginfo('plan received, lets execute it!')
    for action in complete_plan.plan:
        if action.name == 'move_base_safe' or action.name == 'move_base_safe_to_pick' or action.name == 'move_base_safe_to_deliver':
            rospy.loginfo('requesting move_base_safe action from actionlib server : move_base_safe')
            if move_base_safe('folded', action.parameters[0].value, action.parameters[1].value, duration=150.0):
                rospy.loginfo('action succeded !')
                update_kb_move_base_safe(action.parameters[0].value, action.parameters[1].value, action.parameters[2].value)
            else:
                rospy.logerr('action failed ! , aborting the execution...')
                return
        elif action.name=='perceive_location':
            rospy.loginfo('requesting perceive_location action from actionlib server : perceive_location')
            if perceive_location(action.parameters[0].value, duration=150.0):
                rospy.loginfo('action succeded !')
                update_kb_perceive(action.parameters[0].value)
            else:
                update_kb_perceive(action.parameters[0].value, False)
                rospy.logerr('perceive action failed')
                return
        elif action.name=='pick':
            rospy.loginfo('pick action')
            if pick_object(action.parameters[0].value, duration=150.0):
                rospy.loginfo('action succeded !')
                update_kb_pick_object(action.parameters[0].value, action.parameters[1].value, action.parameters[3].value)
            else:
                rospy.logwarn('pick action failed!')
                update_kb_pick_object(action.parameters[0].value, action.parameters[1].value, action.parameters[3].value, False)
                return
        elif action.name=='pick_wbc':
            rospy.loginfo('pick wbc_action')
            if pick_object_wbc(action.parameters[0].value, duration=150.0):
                rospy.loginfo('pick wbc action succeded !')
                update_kb_pick_object(action.parameters[0].value, action.parameters[1].value, action.parameters[3].value)
            else:
                rospy.logwarn('pick wbc action failed!')
                update_kb_pick_object(action.parameters[0].value, action.parameters[1].value, action.parameters[3].value, False)
                return
        elif action.name=='stage':
            rospy.loginfo('stage action')
            if stage_object(action.parameters[1].value, duration=150.0):
                rospy.loginfo('action succeded !')
                update_kb_stage_object(action.parameters[0].value, action.parameters[1].value, action.parameters[2].value)
            else:
                rospy.logerr('action failed ! , aborting the execution...')
                return
        elif action.name=='unstage':
            rospy.loginfo('unstage action')
            if unstage_object(action.parameters[1].value, duration=150.0):
                rospy.loginfo('action succeded !')
                update_kb_unstage_object(action.parameters[0].value, action.parameters[1].value, action.parameters[2].value)
            else:
                rospy.logerr('action failed ! , aborting the execution...')
                return
        elif action.name=='insert':
            rospy.loginfo('insert action')
            if insert_object(action.parameters[0].value, action.parameters[1].value, duration=150.0):
                rospy.loginfo('action succeded !')
                update_kb_insert_object(action.parameters[0].value, action.parameters[1].value, action.parameters[2].value, action.parameters[4].value)
            else:
                rospy.logwarn('insert action failed!')
                update_kb_insert_object(action.parameters[0].value, action.parameters[1].value, action.parameters[2].value, action.parameters[4].value, False)
                return
        elif action.name=='place':
            rospy.loginfo('place_object')
            if place_object(action.parameters[0].value, action.parameters[1].value, duration=150.0):
                rospy.loginfo('action succeded !')
                update_kb_place_object(action.parameters[0].value, action.parameters[1].value, action.parameters[3].value)
            else:
                rospy.logerr('action failed ! , aborting the execution...')
                return
        elif action.name=='drill_object':
            rospy.loginfo('drill action')
            rospy.logerr('Not implemented yet')
            #TODO!! implement drill action
        else:
            rospy.logerr('Error: action "{0}" not recognized, skipping.'.format(action.name))
            rospy.logerr('aborting the execution...')
            return
    publish_success()


def main():
    rospy.loginfo('Initializing planner executor node')
    if mockup:
        print('Warning !!! Using mockup for planner executor!!!')
    else:
        print('Not using mockup! so everything is fine : )')
    rospy.Subscriber("/task_planning/dispatch_msg_generator/plan", CompletePlan, planCallBack)
    rospy.spin()


if __name__ == '__main__':
    main()
