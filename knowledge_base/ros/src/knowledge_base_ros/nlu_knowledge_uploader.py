#!/usr/bin/env python
import rospy

# for reading arguments comming into this function, in this case is for receiving
# the path to pddl problem file
import sys

# pddl parser, reads pddl file and outputs pddl vector
import mercury_planner.pddl as pddl

# create dictionary out of pddl vector
import knowledge_base_ros.update_knowledge_utils as utils

# for rosplan service calls
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

# to ADD_FACTS
from rosplan_knowledge_msgs.srv import rosplan_knowledge_msgs

# for the event_in callback
from std_msgs.msg import String

import UploadPDDLKnowledge as upld

# class nlu_knowledge_upload(object):
#     def __init__(self):

    #subscribe to mbot_nlu_node -> natural_language_processing
    #map  actions/arguments (from nlu) to predicates (ppdl domain)
    #publish these in knowledgebase



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
    nlu_knowledge_uploader = upld()
    upld.start_knowledge_uploader()
    upld.rosplan_update_knowledge(1, '', '', 'at', [['o', 'mbot'], ['l', 'b']], 0.0, 'ADD_GOAL')

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