#!/usr/bin/env python
import rospy

# for reading arguments comming into this function, in this case is for receiving
# the path to pddl problem file
import sys

# pddl parser, reads pddl file and outputs pddl vector
import mercury_planner.pddl as pddl

# create dictionary out of pddl vector
import isr_knowledge_base_ros.update_knowledge_utils as utils

# for rosplan service calls
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

# to ADD_FACTS
from rosplan_knowledge_msgs.srv import rosplan_knowledge_msgs

# for the event_in callback
from std_msgs.msg import String

class UploadPDDLKnowledge(object):
    '''
    This class reads a PDDL problem instance file and uploads its contents to rosplan KB via service calls

    It uses an existing PDDL parser (taken from downward code inside mercury planner code) that transforms PDDL syntax into a PDDL vector:
    Example:
    The third element of this vector looks like this:

    pddl_objects = [':objects', 'dynamixel', '-', 'gripper', 's1', 's2', 's3', 's4', 'start', 'exit', 'cb_ramp', 'cb_trash', 'drill_loc',
                    'force_fitting', 'assembly_station', '-', 'location', 'o1', 'o2', 'o3', 'o4', 'o5', 'o6', 'faulty_plate',
                    'fixable_plate', 'filecard', 'tray', 'blue_box', 'drill', 'trash', '-', 'object', 'youbot-brsu-3', '-', 'robot',
                    'platform_middle', 'platform_left', 'platform_right', '-', 'robot_platform']

    This vector is then converted into a dictinary by using the external ros independant function utils() called by the local
    pddl_object_list_to_dict() function

    We iterate over the above mentioned dictionary to perform individual service calls via rosplan_update_knowledge() function
    '''
    def __init__(self):
        # member variables
        # flag to indicate that a event msg was received in the callback
        self.event_in_received = False
        # get the path to the pddl file from param server
        self.pddl_file_path = rospy.get_param('~pddl_file_path', '/home/user/my_pddl_problem.pddl')
        # get from param server the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # subscribe to event_in std_msgs/String topic to start uploading the knowledge
        rospy.Subscriber("~event_in", String, self.eventInCallBack, queue_size=1)
        # to publish feedback on event_out topic
        self.even_out_pub = rospy.Publisher('~even_out', String, queue_size=1)
        rospy.loginfo('Upload PDDL knowledge node initialized... waiting for event_in to upload knowledge')

    def rosplan_update_knowledge(self, knowledge_type, instance_type, instance_name, attribute_name, values, function_value=0.0, update_type='ADD_KNOWLEDGE'):
        '''
        This generic function receives knowledge parameters and does a service call
        to rosplan knowledge base (mongodb)

        rosplan_update_knowledge() = rosplan service call add knowledge

        Example 1:
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

        '''
        msg = KnowledgeItem()
        if knowledge_type == 0:
            # instance
            msg.knowledge_type = KnowledgeItem.INSTANCE # INSTANCE (rosplan_knowledge_msgs/msg/KnowledgeItem.msg)
            msg.instance_type = instance_type
            msg.instance_name = instance_name
            msg.attribute_name = ''
        elif knowledge_type == 1:
            # fact
            msg.knowledge_type = KnowledgeItem.FACT # FACT (rosplan_knowledge_msgs/msg/KnowledgeItem.msg)
            msg.instance_type = ''
            msg.instance_name = ''
            msg.attribute_name = attribute_name
            for each in values:
                msg.values.append(KeyValue(each[0], each[1]))
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
            update_kb = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base', KnowledgeUpdateService)
            # get response
            if update_type == 'ADD_KNOWLEDGE':
                response = update_kb(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, msg)
            elif update_type == 'ADD_GOAL':
                response = update_kb(KnowledgeUpdateServiceRequest.ADD_GOAL, msg)
            elif update_type == 'REMOVE_KNOWLEDGE':
                response = update_kb(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE, msg)
            elif update_type == 'REMOVE_GOAL':
                response = update_kb(KnowledgeUpdateServiceRequest.REMOVE_GOAL, msg)
            else:
                rospy.logerr('Error : Unknown update_type, admisible values are ADD_KNOWLEDGE, ADD_GOAL, REMOVE_KNOWLEDGE and REMOVE_GOAL')
                rospy.logwarn('Knowledge base not updated...!')
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s'%e)
            return False
        if(response.success):
            rospy.loginfo('Knowledge base successfully updated')
            # service call succesful
            return True
        else:
            rospy.logerror('Could not update world model, failed')
            # service call failed
            return False

    def upload_instances(self, pddl_instances_as_dictionary):
        '''
        Upload PDDL instances helper function, will iterate over PDDL vector making service calls to rosplan KB
        '''
        rospy.loginfo("Uploading pddl objects from specified problem pddl to knowledge base")
        # iterate over PDDL vector (previously converted to a dictionary)
        for each in pddl_instances_as_dictionary:
            for element in each:
                rospy.logdebug('-------- uploading unit knowledge ---------')
                update_type = element.get('update_type')
                knowledge = element.get('knowledge')
                rospy.logdebug(update_type)
                rospy.logdebug(knowledge)
                rospy.logdebug(knowledge.get('instance_name'))
                rospy.logdebug(knowledge.get('instance_type'))
                rospy.logdebug(knowledge.get('attribute_name'))
                rospy.logdebug(knowledge.get('knowledge_type'))
                rospy.logdebug(knowledge.get('function_value'))
                rospy.logdebug('-----------------')
                # remove i_ from instances : example r_youbot-brsu-3 -> youbot-brsu-3
                try:
                    rospy.logdebug(knowledge.get('instance_name').split('--')[1])
                    self.rosplan_update_knowledge(0, knowledge.get('instance_type'), knowledge.get('instance_name').split('--')[1], '', [])
                except:
                    rospy.logerr('Error ocurred, please make sure that you are correctly using key--object and that no negative preconditions are present')
                    return


    def compute_key_values(self, args):
        '''
        splits names by using character -- to get the keys
        this is needed by rosplan KB to be able to upload facts
        example:
            l--locA ; key is "l"
            (at r--ghost l--locA) ; keys are "r" and "l"

        '''
        key_value_list = []
        for each in args[1:]:
            try:
                key_value_list.append(each.split('--'))
            except:
                rospy.logerr('Error ocurred, please make sure that you are correctly using key--object and that no negative preconditions are present')
                return
        return key_value_list


    def upload_facts(self, pddl_init_vector):
        '''
        Upload PDDL facts helper function, will iterate over PDDL vector making service calls to rosplan KB
        '''
        rospy.loginfo("Uploading init predicates from minimum_required_info_problem.pddl file into knowledge base")
        # iterate over pddl vector facts
        for each in pddl_init_vector[1:]:
            key_values = self.compute_key_values(each)
            # sending for service call
            self.rosplan_update_knowledge(1, '', '', each[0], key_values)


    def upload_goals(self, pddl_goal_vector):
        '''
        Upload PDDL goals helper function, will iterate over PDDL vector making service calls to rosplan KB
        '''
        rospy.loginfo("Uploading goal predicates from minimum_required_info_problem.pddl file into knowledge base")
        for each in pddl_goal_vector[1][1:]:
            key_values = self.compute_key_values(each)
            self.rosplan_update_knowledge(1, '', '', each[0], key_values, update_type='ADD_GOAL')


    def upload_knowledge(self, pddl_vector):
        '''
        generic function that upload knowledge to KB
        '''
        # used later on to identify cathegory (pddl_vector[0] is either objects, init or goals)
        pddl_cathegory = pddl_vector[0]
        # check if knowledge is empty
        if len(pddl_vector) == 1:
            rospy.logwarn(pddl_cathegory + " knowledge is empty, while uploading pddl knowledge")
        else:
            if pddl_cathegory == ':objects':
                self.upload_instances(self.pddl_object_list_to_dict(pddl_vector))
            elif pddl_cathegory == ':init':
                self.upload_facts(pddl_vector)
            elif pddl_cathegory ==':goal':
                self.upload_goals(pddl_vector)
            else:
                rospy.logerr('Error : could not identify cathegory, admissible values are :objects, :init and :goal')


    def pddl_object_list_to_dict(self, pddl_objects):
        '''
        transforms the output of the pddl parser into a dictionary, that later on will be used
        to upload the knowledge via service calls
        '''
        categories, objects = utils.parse_objects(pddl_objects)

        pddl_problem_dict = []
        for ii, category in enumerate(categories):
            pddl_problem_dict.append([
                utils.create_knowledge_unit_dict(0, utils.create_knowledge_dict(instance_type=category, instance_name=obj))
                for obj in objects[ii]
            ])

        return pddl_problem_dict


    def getPddlProblem(self, pddl_file_path):
        '''
        Converts PDDL file into PDDL vector using helper functions from downward planner
        '''
        pddl_problem_vector = None
        try:
            # inform the user about the file that will be used
            rospy.loginfo('Using problem.pddl file : ' + pddl_file_path)
            # convert pddl file into ppdl vector
            pddl_problem_vector = pddl.pddl_file.parse_pddl_file('problem', pddl_file_path)
        except:
            rospy.logerr('Could not open .pddl file !')
        return pddl_problem_vector


    def UploadNowPDDLKnowledge(self, pddl_file_path):
        '''
        This method uses all above class methods to upload PDDL knowledge to KB
        '''
        # read pddl file and convert into PDDL vector
        pddl_problem_vector = self.getPddlProblem(pddl_file_path)
        # ensure that operation was succesful
        if pddl_problem_vector == None:
            return False
        # pddl_problem_vector[3] -> objects, pddl_problem_vector[4] -> init, pddl_problem_vector[5] -> goals
        # upload instances
        self.upload_knowledge(pddl_problem_vector[3])
        # upload facts
        self.upload_knowledge(pddl_problem_vector[4])
        # upload goals
        self.upload_knowledge(pddl_problem_vector[5])
        return True


    def eventInCallBack(self, msg):
        '''
        This fuction will get executed upon receiving a msg on event_in topic
        '''
        self.event_in_msg = msg
        self.event_in_received = True


    def start_knowledge_uploader(self):
        '''
        PDDL knowledge uploader main loop function
        '''
        while not rospy.is_shutdown():
            if self.event_in_received == True:
                # lower flag
                self.event_in_received = False
                # check msg content
                if self.event_in_msg.data == 'e_start':
                    # upload PDDL knowledge to KB and publish feedback about the result
                    if self.UploadNowPDDLKnowledge(self.pddl_file_path):
                        self.even_out_pub.publish(String('e_success'))
                    else:
                        self.even_out_pub.publish(String('e_failure'))
                else:
                    rospy.logerr('even_in msg not supported, admissible values are : e_start')
                    self.even_out_pub.publish(String('e_failure'))
            self.loop_rate.sleep()


def main():
    rospy.init_node('upload_pddl_knowledge_node', anonymous=False)
    pddl_knowledge_uploader = UploadPDDLKnowledge()
    pddl_knowledge_uploader.start_knowledge_uploader()
