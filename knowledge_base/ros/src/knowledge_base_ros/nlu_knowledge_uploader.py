#!/usr/bin/env python
import rospy

# for reading arguments comming into this function, in this case is for receiving
# the path to pddl problem file
import sys

# # pddl parser, reads pddl file and outputs pddl vector
# import mercury_planner.pddl as pddl

# # create dictionary out of pddl vector
# import knowledge_base_ros.update_knowledge_utils as utils

# # for rosplan service calls
# from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
# from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
# from rosplan_knowledge_msgs.msg import KnowledgeItem
# from diagnostic_msgs.msg import KeyValue

# # to ADD_FACTS
# from rosplan_knowledge_msgs.srv import rosplan_knowledge_msgs

# for the event_in callback
from std_msgs.msg import String

import knowledge_base_ros.upload_knowledge as upld


class nlu_knowledge_upload(object):
    def __init__(self):
        # rospy.init_node('nlu_knowledge_uploader', anonymous=False)
        #subscribe to mbot_nlu_node -> natural_language_processing
        #rospy.Subscriber("natural_language_processing", String, self.nluCallback)

        # self.intention_list = ('go', 'grasp', 'bring')
        # self.intention_dict = {}
        # self.z = 0
        # for x in self.intention_list: # construct the dict based on the intention list
        #     self.intention_dict[x] = self.z
        #     self.z = self.z+1

        # self.slot_list = ('kitchen', 'coke', 'dinner_table')
        # self.slot_dict = {}
        # self.z = 0
        # for x in self.slot_list: # construct the dict based on the intention list
        #     self.slot_dict[x] = self.z
        #     self.z = self.z+1


        # self.intention_dict = {'go':'move_base' , 'grasp':'grasp', 'put':'place', 'find':'find_person', 'introduce':'introduce', 'guide':'guide', '':'answer_question', '':'ask_name'}
        #self.intention_to_action = {'go':'move_base' , 'grasp':'grasp', 'put':'place', 'bring':'place', 'find':'find_person', 'introduce':'introduce', 'guide':'guide'}
        self.intention_to_action = {'go':'move_base' , 'grasp':'grasp', 'meet':'introduce' , 'take':'place' , 'guide':'guide' , 'find':'find_person' , 'tell':'tell' , 'answer':'answer_question' }
        self.action_to_predicate = {'move_base':'at_r' , 'grasp':'holding', 'place':'on', 'find_person':'found', 'introduce':'known_p', 'guide':'at_p', 'answer_question':'iluminated' , 'tell':'told'}
        #? - nove_base, guide, place

        self.slot_to_type = {'b':'location', 'kitchen':'location' , 'dinner_table':'location', 'entrance':'location', 'coke':'object', 'milk':'object', 'mbot':'robot', 'John':'person', 'Lucy':'person'}
        # self.slot_list_object = ('coke', 'energy drink', 'dinner_table')
        


    def map_nlu_to_predicates(self, reconized_intention, reconized_slot):
        if(reconized_intention in self.intention_to_action):
            if(self.intention_to_action[reconized_intention] in self.action_to_predicate):
                self.attribute_name = self.action_to_predicate[self.intention_to_action[reconized_intention]]


        self.value = []
        for argument in reconized_slot:
            if(argument in self.slot_to_type):
                self.value.append( [self.slot_to_type[argument] , argument] )

                if (self.attribute_name == 'holding'): # hardcode while issue #80 is not solved - after this line can be removed
                    self.value.append( ['robot', 'mbot'] )

        return self.attribute_name, self.value

    # def nluCallback(self, msg):
    #     '''
    #     This fuction will get executed each time a msg is published in natural_language_processing topic
    #     '''

    #     '''
    #     move to the kitchen counter, grasp the Energy drink and bring it to the dinner table

    #     nlu current output
    #         recognized_action: go
    #         slot: ['kitchen counter']
    #         -
    #         recognized_action: grasp
    #         slot: ['Energy drink']
    #         -
    #         recognized_action: bring
    #         slot: ['energy drink','dinner table']
    #     '''

    #     # self.reconized_intention = msg.action
    #     # self.reconized_slot = msg.arguments #list of arguments

    #     #map  actions/arguments (from nlu) to predicates (ppdl domain)
    #     attribute_name, value = self.map_nlu_to_predicates(msg.action, msg.arguments)
        
    #     print attribute_name
    #     print value
    #     #publish these in the knowledgebase

    # def start_nlu_uploader(self):

# def compute_key_values(args):
#     '''
#     splits names by using character -- to get the keys
#     this is needed by rosplan KB to be able to upload facts
#     example:
#         l--locA ; key is "l"
#         (at r--ghost l--locA) ; keys are "r" and "l"

#     '''
#     key_value_list = []
#     for each in args[1:]:
#         try:
#             key_value_list.append(each.split('--'))
#         except:
#             rospy.logerr('Error ocurred, please make sure that you are correctly using key--object and that no negative preconditions are present')
#             return
#     return key_value_list


def main():
    print 'Hey'
    rospy.init_node('upload_pddl_knowledge_node', anonymous=False)
    print 'end'
    nlu_knowledge_uploader = upld.UploadPDDLKnowledge()
    print 'upld'

    sentence_recognized = [ ['go' , ['mbot', 'b', 'madsa']] , ['grasp', ['coke']] , ['introduce', ['']]]

    map_test = nlu_knowledge_upload()   


    for phrase in sentence_recognized:
        attribute_name, value = map_test.map_nlu_to_predicates(phrase[0], phrase[1])

        #DEBUG
        nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', attribute_name, value, 0.0, 'ADD_KNOWLEDGE')

        # if (is a GOAL):
        #     nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', attribute_name, value, 0.0, 'ADD_GOAL')
        # elif(is a FACT):
        #     nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', attribute_name, value, 0.0, 'ADD_KNOWLEDGE')

    # print attribute_name
    # print value


        
    print 'update kb'

# rosservice call /kcl_rosplan/update_knowledge_base "update_type: 1
# knowledge:
#   knowledge_type: 1
#   instance_type: ''
#   instance_name: ''
#   attribute_name: 'at'
#   values:
#   - {key: 'o', value: 'mbot'}
#   - {key: 'l', value: 'b'}
#   function_value: 0.0
#   is_negative: false" 

#     (knowledge_type, instance_type, instance_name, attribute_name, values, function_value=0.0, update_type='ADD_KNOWLEDGE')

    # rospy.init_node('upload_pddl_knowledge_node', anonymous=False)
    # pddl_knowledge_uploader = UploadPDDLKnowledge()
    # pddl_knowledge_uploader.start_knowledge_uploader()