#!/usr/bin/env python
"""

"""
#-*- encoding: utf-8 -*-
__author__ = 'dnair2s'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import atwork_ros_msgs.msg

import roslib
import actionlib
import sys
from mir_yb_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mir_yb_action_msgs.msg import MoveBaseSafeResult

from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.srv import GetDomainAttributeService
from rosplan_knowledge_msgs.srv import GetDomainTypeService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

class RefboxParser(object):
    """
    World model for storing the objects
    """
    def __init__(self):

        #inventory message
        self._inventory_message = None
        #order messge
        self._tasks = None

        #list for storing the parameters from domain 
        self._predicate_param_type_list = {}
        self._predicate_param_label_list = {}

        # model view
        self._type_list = []

        #list to remember the naming given to models
        self._model_number = {}

        #The dictinary to store name of the dommain as per pddl
        #NOTE container is of type object as per the pddl designed 
        self.domain_name = {'object':'object', 'location':'location', 'container':'object', 'destination':'location'}

        #The enum value for rosplan_knowledge_msgs/KnowledgeUpdateService
        self.knowledge_update_service = {'ADD_KNOWLEDGE':0, 'ADD_GOAL':1, 'REMOVE_KNOWLEDGE':2, 'REMOVE_GOAL':3}

        #Enum of object types from messages rocking
        #https://github.com/rockin-robot-challenge/at_work_central_factory_hub/blob/rockin/rockin/msgs/Inventory.proto
        self.object_type = {1:'F20_20_B', 2:'F20_20_G', 3:'S40_40_B',
                            4:'S40_40_G' , 5:'M20_100', 6:'M20',
                            7:'M30' , 8:'R20', 9:'BEARING_BOX',
                            10:'BEARING' , 11:'AXIS', 12:'DISTANCE_TUBE',
                            13:'MOTOR', 14:'CONTAINER_B', 15:'CONTAINER_R'}

        self.location_type = {1:'SH', 2:'WS', 3:'CB', 4:'WP', 5:'PP', 6:'ROBOT'}

        self.orientation = {1:'NORTH', 2:'EAST', 3:'SOUTH', 4:'WEST'}

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 0.5))

        #Subscriber -> subscribe to refbox Inventory message
        rospy.Subscriber("~refbox", atwork_ros_msgs.msg.Inventory, self.inventory_callback, queue_size=10)

        #Subscriber -> subscribe to refbox for Order message
        rospy.Subscriber("~refbox_task", atwork_ros_msgs.msg.TaskInfo, self.task_callback, queue_size=10)

        #event in subscriber
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.eventInCallBack, queue_size=10)

        # even inventory flag
        self.start_inventory_flag = True

        # event of completion of inventory update for starting task update
        self.inventory_update_complete_flag = False

        # even order flag
        self.start_order_flag = True
        #publisher
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        bnt =  rospy.get_param("~bnt", False)

        if bnt:
            rospy.loginfo('Running BNT from refbox. Not waiting for the database world model')
        else:
            rospy.loginfo('waiting for update world model service')
            rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')

            self.update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base',
                                            KnowledgeUpdateService)

            # populate goal combo boxes
            rospy.wait_for_service('/kcl_rosplan/get_domain_predicates')
            try:
                predicates_client = rospy.ServiceProxy('/kcl_rosplan/get_domain_predicates', GetDomainAttributeService)
                resp = predicates_client()
                for pred in resp.items:
                    param_list = []
                    label_list = []
                    for param in pred.typed_parameters:
                        param_list.append(param.value)
                        label_list.append(param.key)
                    self._predicate_param_type_list[pred.name] = param_list
                    self._predicate_param_label_list[pred.name] = label_list

                print self._predicate_param_type_list
                print self._predicate_param_label_list
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            # populate type combo box
            rospy.wait_for_service('/kcl_rosplan/get_domain_types')
            try:
                type_client = rospy.ServiceProxy('/kcl_rosplan/get_domain_types', GetDomainTypeService)
                resp = type_client()
                for typename in resp.types:
                    self._type_list.append(typename)

                print self._type_list
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        rospy.loginfo("Started node Refbox")
    def eventInCallBack(self, msg):
        if msg.data == 'e_trigger':
            rospy.loginfo('Received trigger updating knowledge base based on order and invetory data')
            self.start_inventory_flag = True
            self.start_order_flag = True

    def start(self):
        while not rospy.is_shutdown():
            if self.start_inventory_flag :
                self.load_inventory_to_knowledge_base()
                #Resetting the namings
                self._model_number = {}
            if self.start_order_flag:
                self.process_task_message()
                #Resetting the namings
                self._model_number = {}
            self.loop_rate.sleep()

    '''
    Callback on inventory message

    Inventory message has a list of "Items".
    Each Item has the following structure.


    object                      ->description of the object
        type
        type_id
        instance_id(optional)   ->If provided then dont check quantity
        description(optional)
    quantity(optional)          ->If instance_id not provided check this
    container(optional)         ->The container in which the item is stored
        type
        type_id
        instance_id(optional)
        description(optional)
    location(optional)          ->Location where the object is stored
        type
        instance_id
        description(optional)
    '''
    def inventory_callback(self, msg):
        self._inventory_message = atwork_ros_msgs.msg.Inventory(msg.items)
	pass

    def load_instance_message(self, type, domain, item, count=-1 ):
        #Creating the message
        instance_msg = KnowledgeItem()
        #uploading the instance of object

        instance_msg.knowledge_type = type
        instance_msg.instance_type = self.domain_name[domain]
        instance_msg.instance_name = self.msg_to_instance_name(item, domain, count)
        instance_msg.attribute_name = ''
        instance_msg.function_value = 0.0

        self.write_to_knowledge_base(instance_msg, self.knowledge_update_service['ADD_KNOWLEDGE'])
        return instance_msg.instance_name

    def load_inventory_to_knowledge_base(self):

        if (None == self._inventory_message):
            return

        #Creating the message
        instance_msg = KnowledgeItem()


        for item in self._inventory_message.items:
            #Attribute on which the fact is written
            attribute = None
            object_instance_list = []

            #################################
            ## Uploading INSTANCE of Objects
            #################################
            #check if the instance_id is present
            if (0 != item.object.instance_id.data):
                #uploading the instance of object
                name = self.load_instance_message(KnowledgeItem.INSTANCE, 'object', item )
                object_instance_list.append(name)

            else:
                #if not then
                #check quantity and loop on quantity
                if (0 != item.quantity.data):
                    for count in range(item.quantity.data):
                        #uploading the instance of object
                        name = self.load_instance_message(KnowledgeItem.INSTANCE, 'object', item, count)
                        object_instance_list.append(name)
                else:
                    rospy.loginfo("Instance ID not provided as well as quantity ")
                    #uploading the instance of object without ID and quantity 
                    name = self.load_instance_message(KnowledgeItem.INSTANCE, 'object', item)
                    object_instance_list.append(name)
                    self.event_out.publish("e_failure")

            #Flag for identifying if location or container
            instance_type_identifyer = None

            #################################
            ## Uploading INSTANCE of locations
            #################################
            #check if location exist
            if (0 != item.location.type.data):
                name = self.load_instance_message(KnowledgeItem.INSTANCE, 'location', item)

                #fill fact "Object" ON "location"
                attribute = 'on'
                instance_type_identifyer = 'location'

            #################################
            ## Uploading INSTANCE of CONTAINER
            #################################
            elif (0 != item.container.type.data):
                name = self.load_instance_message(KnowledgeItem.INSTANCE, 'container', item)
                #upload fact "Object" IN "container"
                attribute = 'in'
                instance_type_identifyer = 'container'

            #################################
            ## Uploading FACTS
            #################################

            if (attribute):
                for uploaded_instance in object_instance_list:
                    fact_msg = KnowledgeItem()
                    fact_msg.knowledge_type = KnowledgeItem.FACT
                    fact_msg.instance_type = ''
                    fact_msg.instance_name = ''
                    fact_msg.attribute_name = attribute
                    fact_msg.function_value = 0.0
                    output_value = KeyValue()
                    output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[0]
                    output_value.value = uploaded_instance
                    fact_msg.values.append(output_value)
                    output_value = KeyValue()
                    output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[1]
                    output_value.value = self.msg_to_instance_name(item, \
                                                        instance_type_identifyer)
                    fact_msg.values.append(output_value)
                    self.write_to_knowledge_base(fact_msg, self.knowledge_update_service['ADD_KNOWLEDGE'])

                attribute = None

            #Resetting the flag when the inventory contained some data
            rospy.loginfo('Received inventory data updated knowledge base')
            self.start_inventory_flag = False
            # Triggering the completion of the inventory update
            self.inventory_update_complete_flag = True

        #clearning inventory message
        self._inventory_message = None


        self.event_out.publish("e_done")


    def write_to_knowledge_base(self, msg, type):
        try:
            #get response
            rospy.loginfo("Type :%d, msg %d , %s ,%s, %s", type, msg.knowledge_type, msg.instance_type,
                    msg.instance_name, msg.attribute_name)

            resp1 = self.update_kb(type, msg)
            success = resp1.success
        except rospy.ServiceException, e:
            rospy.logerr( "Service call failed: %s" ,e )
            success = None

        if(success):
            rospy.loginfo('World model successfully updated')
            return 'success'
        else:
            rospy.logerr('Could not update world model, failed')
            self.event_out.publish("e_failure")
            return 'fail'

    def msg_to_instance_name(self, item, domain_name, count=-1):

        if domain_name == 'object':
            text = self.object_type[item.object.type.data] 
            if -1 == count:
                text = text
            else:
                #if the count is available in the list then add 
                if self._model_number.has_key(text):
                    count = self._model_number[text] + 1
                    self._model_number[text] = count
                else:
                    self._model_number[text] = 0
                text = text + '-count-' + str(count).zfill(2)

        if domain_name == 'container':
            text = self.object_type[item.container.type.data] 
            if -1 == count:
                text = text
            else:
                text = text + '-count-' + str(count).zfill(2)

        if domain_name == 'location':
            text = self.location_type[item.location.type.data] + \
                    '-' + str(item.location.instance_id.data).zfill(2)

        if domain_name == 'destination':
            text = self.location_type[item.destination.type.data] + \
                    '-' + str(item.destination.instance_id.data).zfill(2)
        return text

    '''
    Order callback for taking the order
    '''
    def task_callback(self, msg):
        self._tasks = atwork_ros_msgs.msg.TaskInfo(msg.tasks)
	pass

    ####################################################################
    # Navigation task
    ####################################################################
    def execute_navigation_task(self):
        client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
        rospy.loginfo("Waiting for Movebase actionlib server")
        client.wait_for_server()
        rospy.loginfo("Got Movebase actionlib server sending goals")
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'

        for task in self._tasks.tasks:
            #check if the task is navigation or transportation

            if atwork_ros_msgs.msg.Task.NAVIGATION == task.type.data:
                print self.location_type[task.navigation_task.location.type.data] + \
                                              str(task.navigation_task.location.instance_id.data ).zfill(2)
                try:
                    goal.source_location = ""
                    goal.destination_location = self.location_type[task.navigation_task.location.type.data] + \
                                                 str(task.navigation_task.location.instance_id.data ).zfill(2)
                    goal.destination_orientation = self.orientation[task.navigation_task.orientation.data]
                    timeout = 60.0
                    rospy.loginfo('Sending action lib goal to move_base_safe_server'
                                     + ' , destination : ' + goal.destination_location 
                                     + ' , orientation : '+ goal.destination_orientation)
                    client.send_goal(goal)
                    client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                    result = client.get_result()
                    if(result):
                        if True == result.success:
                            d = rospy.Duration(task.navigation_task.wait_time.data.secs, 0)
                            rospy.sleep(d)
                        else:
                            rospy.logerr('Navigation task failure. TODO to retry')
                    else:
                        client.cancel_goal()
                        rospy.logerr('No Result from client')
                except:
                    rospy.logerr("FAILED . Action server MOVE_BASE_SAFE to desitnaton %s ", goal.destination_location)

        #Exiting when the action is done move to EXIT
        goal.destination_location = "EXIT"
        timeout = 60.0
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        client.cancel_goal()
        rospy.loginfo('Reached EXIT ')



    def process_task_message(self):
        if(None == self._tasks):
            return
        if atwork_ros_msgs.msg.Task.NAVIGATION == self._tasks.tasks[0].type.data:
            self.execute_navigation_task()
        elif atwork_ros_msgs.msg.Task.TRANSPORTATION == self._tasks.tasks[0].type.data:
            #check if the inventory data was written otherwise wait for inventory update
            if False == self.inventory_update_complete_flag :
                #Return without resetting the message and order flag
                rospy.loginfo("The inventory is not updated. Waiting ")
                return
            self.load_order_to_knowledge_base()
        else:
            rospy.logerr("NO VALID TASK PRESENT")

        #Resetting flag when order came and it contined data
        self.start_order_flag = False
        #order messge
        self._tasks = None

    def load_order_to_knowledge_base(self):

        rospy.loginfo("Loading task info to knowledge base")
        #Creating the message
        instance_msg = KnowledgeItem()
        for task in self._tasks.tasks:
            ####################################################################
            # Transportation task
            ####################################################################
            if atwork_ros_msgs.msg.Task.TRANSPORTATION == task.type.data:
                # The task is transportation
                # find the object, its destination

                if (0 != task.transportation_task.container.type_id.data):
                    #container is available
                    self.fill_order(task.transportation_task, 'in', 'object', 'container')
                elif (0 != task.transportation_task.destination.instance_id.data):
                    #container not available but destination available
                    self.fill_order(task.transportation_task, 'on', 'object', 'destination')
                else:
                    #both container and destination not available
                    rospy.logerr("No Goal set")

                if (0 != task.transportation_task.container.type_id.data) and \
                (0 != task.transportation_task.destination.instance_id.data):
                    #Both container and destination available
                    self.fill_order(task.transportation_task, 'on', 'container', 'destination' )


        rospy.loginfo("Task info upload completed")



    def fill_order(self, order, attribute, first, second, count=-1):
        #In transportationtask sometimes the destination is not there in inventory
        #Needs to add fact about the inventory
        if 'destination' == second:
            name = self.load_instance_message(KnowledgeItem.INSTANCE, 'destination', order)
        fact_msg = KnowledgeItem()
        fact_msg.knowledge_type = KnowledgeItem.FACT
        fact_msg.instance_type = ''
        fact_msg.instance_name = ''
        fact_msg.attribute_name = attribute
        fact_msg.function_value = 0.0
        output_value = KeyValue()
        output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[0]
        output_value.value = self.msg_to_instance_name(order, first, count)

        fact_msg.values.append(output_value)
        output_value = KeyValue()
        output_value.key = (self._predicate_param_label_list[fact_msg.attribute_name])[1]
        #in order the container or location doesnt have the count
        output_value.value = self.msg_to_instance_name(order, second )
        fact_msg.values.append(output_value)
        self.write_to_knowledge_base(fact_msg, self.knowledge_update_service['ADD_GOAL'] )

def main():
    rospy.init_node('refbox_parser', anonymous=True)
    refbox_parser = RefboxParser()
    refbox_parser.start()

