#! /usr/bin/env python
import rospy
import actionlib
import rosplan_knowledge_msgs.msg as rosplan_knowledge
import rosplan_knowledge_msgs.srv as rosplan_service

from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import CompletePlan
from mbot_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mbot_action_msgs.msg import PickObjectAction, PickObjectGoal
from mbot_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal
from mbot_action_msgs.msg import PlaceAction, PlaceGoal



# functions to upload facts, goals into the knowledge base
from mcr_knowledge_ros import upload_knowledge

class MbotPlannerExecutor(object):
    '''
    Listens to ~plan topic and executes the actions contained in it one by one
    by creating action clients (actionlib) assuming that action servers are up and running.
    '''
    def __init__(self):
        rospy.loginfo('Initializing planner executor node')
        # class variables
        self.plan_msg = None
        self.plan_received = False
        self.speech_sring_received = True # HACK
        self.speech_string = None # HACK
        # parameters
        self.mockup = rospy.get_param('~mockup', False)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 5.0))
        # publishers
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
        # subcribers
        rospy.Subscriber("~plan", CompletePlan, self.planCallBack)
        rospy.Subscriber("~speech_recognition", String, self.speechCallback) # HACK, remove : listens also to speech topic to pass as argument to explain action
        # Inform user about the parameters that will be used
        rospy.loginfo("The node will run with the following parameters :")
        if self.mockup:
            rospy.loginfo('mockup is set to true')
            rospy.logwarn('Warning!!! : Using mockup for planner executor!!!')
        else:
            rospy.loginfo('mockup is set to false')
        rospy.loginfo("-----------------")
        
    # HACK: speechCallback function, should be queried by world model perhaps...
    def speechCallback(self, msg):
        # receives a string with the speech recognition sentence
        self.speech_string = msg.data
        self.speech_sring_received = True

    def action_outcome_mockup(self, client_name):
        '''
        Used for planner executor mockup, user is asked with desired action outcome which
        is the input from keyboard
        '''
        rospy.loginfo('Executing action ' + client_name + 'mockup!!!')
        response = raw_input('Do you want the action to succed or fail (success s , failed f) :')
        if response.startswith('s'):
            rospy.loginfo('Outcome = ' + str(response) + ' succeded !!!')
            return True
        else:
            rospy.loginfo('Outcome = ' + str(response) + ' failed !!!')
            return False
        rospy.logerr('This message should not ever appear, something is really wrong with the code')
        return None


    def replan(self):
        rospy.logwarn('Some component reported failure, therefore replan is needed')
        msg = String()
        msg.data = 'e_failure'
        self.event_out_pub.publish(msg)
        return False


    def publish_success(self):
        rospy.loginfo('Plan succesfully completed, everyone is happy now : )')
        self.event_out_pub.publish(String('e_success'))
        
        
    def move_base_safe(self, arm_safe_position, current_robot_location, destination, duration=150.0):

        rospy.loginfo('calling action client move_base_safe ' + current_robot_location + ' ' + destination)
        # mockup begins----
        if self.mockup:
            mockup_result = self.action_outcome_mockup('move_base_safe')
            if mockup_result:
                return True
            else:
                return self.replan()
        else:
            client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
            client.wait_for_server()
            goal = MoveBaseSafeGoal()
            goal.arm_safe_position = arm_safe_position
            goal.source_location = current_robot_location
            goal.destination_location = destination
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(duration))
            rospy.loginfo('action server finished execution with the following response : ' + str(client.get_result().success))
            if client.get_result().success == False:
                return self.replan()
            else:
                return True

    def perceive_location(self, location, duration=150.0):

        rospy.loginfo("calling action client perceive location " + location)
        # mockup begins
        if self.mockup:
            mockup_result = self.action_outcome_mockup('perceive location')
            if mockup_result:
                return True
            else:
                return self.replan()
        else:
            self.speech_sring_received = False
            client = actionlib.SimpleActionClient('perceive_location_server', PerceiveLocationAction)
            client.wait_for_server()
            goal = PerceiveLocationGoal()
            goal.location = location
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(duration))
            rospy.loginfo("action server finished execution with the following response : " + str(client.get_result().success))
            if client.get_result().success == False:
                return self.replan()
            else:
                return True

    def pick_object(self, obj, duration=150.0):

        rospy.loginfo("calling action client pick_object " + obj)
        # mockup begins
        if self.mockup:
            mockup_result = self.action_outcome_mockup('pick_object')
            if mockup_result:
                return True
            else:
                return self.replan()
        else:
            client = actionlib.SimpleActionClient('pick_object_server', PickObjectAction)
            client.wait_for_server()
            goal = PickObjectGoal()
            goal.obj = obj
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(duration))
            rospy.loginfo(
                "action server finished execution with the following response : " + str(client.get_result().success))
            if client.get_result().success == False:
                return self.replan()
            else:
                return True

    def place(self, duration=150.0):

        rospy.loginfo("calling action client place " + obj + location)
        # mockup begins
        if self.mockup:
            mockup_result = self.action_outcome_mockup('place')
            if mockup_result:
                return True
            else:
                return self.replan()
        else:
            client = actionlib.SimpleActionClient('place_server', place)
            client.wait_for_server()
            goal = PlaceGoal()
            goal.shelf = 'folded'
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(duration))
            rospy.loginfo("action server finished execution with the following response : " + str(client.get_result().success))
            if client.get_result().success == False:
                return self.replan()
            else:
                return True


    def update_kb_move_base_safe(self, source, destination, robot, success=True):
        """
        1.  robot is no longer at start if accion succeded removing this predicate from knowledge base
        2.  robot acomplished the given goal removing this goal from pending goals (in case it is there because
            it might be the case that this is a sub goal, but if you try to remove something that is not there nothing happens)
        3.  add knowledge, now robot is at destination
        """
        if success:
            params = [['r', robot],['l', source]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'at', params, update_type='REMOVE_KNOWLEDGE')
            params = [['r', robot],['l', destination]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'at', params, update_type='ADD_KNOWLEDGE')
        else:
            pass

    def update_kb_perceive_location(self, obj, furniture, success=True):

        if success:
            params = [['f', furniture]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params, update_type='ADD_KNOWLEDGE')
            params = [['obj', obj]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'obj_perceived', params, update_type='ADD_KNOWLEDGE')

        else:
            pass

    def update_kb_pick_object(self, obj, furniture, gripper, success=True):

        if success:
            params = [['g', gripper]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'loaded', params,
                                                      update_type='ADD_KNOWLEDGE')
            params = [['obj', obj], ['f', furniture]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params,
                                                      update_type='REMOVE_KNOWLEDGE')
            params = [['obj', obj], ['g', gripper]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params,
                                                      update_type='ADD_KNOWLEDGE')
            params = [['f', furniture]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'perceived', params,
                                                      update_type='REMOVE_KNOWLEDGE')
        else:
            pass

    def update_kb_place(self, obj, furniture, gripper, success=True):

        if success:
            params = [['obj', obj], ['f', furniture]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params,
                                                      update_type='ADD_KNOWLEDGE')
            params = [['g', gripper]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'loaded', params,
                                                  update_type='REMOVE_KNOWLEDGE')
            params = [['obj', obj], ['g', gripper]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'holding', params,
                                                      update_type='ADD_KNOWLEDGE')
            params = [['obj', obj], ['f', furniture]]
            upload_knowledge.rosplan_update_knowledge(1, 'n/a', 'n/a', 'on', params,
                                                  update_type='REMOVE_GOAL')
        else:
            pass

    def planCallBack(self, plan_msg):
        self.plan_msg = plan_msg
        self.plan_received = True
        rospy.loginfo('plan received !')
        
        
    def execute_plan(self):
        rospy.loginfo('Executing plan')
        for action in self.plan_msg.plan:
            if action.name == 'move_base_safe':
                rospy.loginfo('requesting move_base_safe action from actionlib server : move_base_safe')
                if self.move_base_safe('folded', action.parameters[0].value, action.parameters[1].value, duration=150.0):
                    rospy.loginfo('action succeded !')
                    self.update_kb_move_base_safe(action.parameters[0].value, action.parameters[1].value, action.parameters[2].value, success=True)
                else:
                    rospy.logerr('action failed ! , aborting the execution...')
                    return

            elif action.name == 'perceive_location':
                rospy.loginfo('requesting perceive location action from actionlib server : perceive_location')
                if self.perceive_location(action.parameters[2].value, duration=150.0):
                    self.update_kb_perceive_locations(action.parameters[0].value, action.parameters[1].value, success=True)
                    rospy.loginfo('action succeded !')
                else:
                    rospy.logerr('explain action failed')
                    return
            elif action.name == 'pick_object':
                rospy.loginfo('requesting pick_object action from actionlib server : pick_object')
                if self.pick_object(action.parameters[0].value, duration=150.0):
                    self.update_kb_pick_object(action.parameters[0].value, action.parameters[1].value, action.parameters[3].value,
                                               success=True)
                    rospy.loginfo('action succeded !')
                else:
                    rospy.logerr('explain action failed')
                    return
            elif action.name == 'place':
                rospy.loginfo('requesting place action from actionlib server : place')
                if self.place(duration=150.0):
                    self.update_kb_place(action.parameters[0].value, action.parameters[1].value, action.parameters[2].value,
                                         success=True)
                    rospy.loginfo('action succeded !')
                else:
                    rospy.logerr('explain action failed')
                    return
            else:
                rospy.logerr('Error: action "{0}" not recognized, skipping.'.format(action.name))
                rospy.logerr('aborting the execution...')
                return
            # check if a new plan was received
            if self.plan_received == True:
                rospy.logwarn('A new plan was received while executing the previous one, aborting execution execution')
                self.plan_received = False
                return
        self.publish_success()
        
    def start_mbot_planner_executor(self):
        while not rospy.is_shutdown():
            if self.plan_received == True:
                # lower flag
                self.plan_received = False
                # execute plan
                self.execute_plan()
            self.loop_rate.sleep()


def main():
    rospy.init_node('mbot_planner_executor', anonymous=False)
    mbot_planner_executor = MbotPlannerExecutor()
    mbot_planner_executor.start_mbot_planner_executor()
