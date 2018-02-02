#!/usr/bin/env python
import rospy
import sys

# for the msg_received in the nluCallback
from mbot_nlu.msg import ActionSlotArray

import knowledge_base_ros.upload_knowledge as upld

# import mapping from slot to the available types
from mbot_world_model_ros.gpsr_dict import slots_dict


class nlu_knowledge_upload(object):
    def __init__(self):
        # flag to indicate that a event msg was received in the callback
        self.msg_received = False

        # instatiate the UploadPDDLKnowledge so we can make useo use the rosplan_update_knowledge upload method
        self.nlu_knowledge_uploader = upld.UploadPDDLKnowledge()

        # subscribe to recognized_intention topic that is publiched by natural_language_understanding node
        rospy.Subscriber("natural_language_understanding/recognized_intention", ActionSlotArray, self.nluCallback)

        # get from param server the frequency at which this node will pool incoming messages
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # dictionary that translates between the nlu intention output and the corresponding actions on the pddl domain
        self.intention_to_action = {'go':'move_base' , 'grasp':'grasp', 'meet':'introduce' , 'take':'place' , 'guide':'guide' , 'find':'find_person' , 'tell':'tell' , 'answer':'answer_question' }
        
        # dictionary that translates between the actions on the pddl domain and the corresponding predicates on the pddl domain
        self.action_to_predicate = {'move_base':'at_r' , 'grasp':'holding', 'place':'on', 'find_person':'found', 'introduce':'known_p', 'guide':'at_p', 'answer_question':'iluminated' , 'tell':'told'}
        
        # dictionary that translates between the nlu slot output and the corresponding types on the pddl domain
        self.slot_to_type = slots_dict

    def map_nlu_to_pddl_domain(self, reconized_intention, reconized_slot):
        # mapping the recognized intention (output from the nlu) to the corresponding predicate of the pddl domain
        if(reconized_intention in self.intention_to_action):
            if(self.intention_to_action[reconized_intention] in self.action_to_predicate):
                self.attribute_name = self.action_to_predicate[self.intention_to_action[reconized_intention]]

            # mapping the recognized slots (output from the nlu) to the corresponding types of the pddl domain
            self.value = []
            for argument in reconized_slot:
                if(argument in self.slot_to_type):
                    self.value.append( [self.slot_to_type[argument] , argument] )
                elif(argument == 'yourself'):
                    self.value.append( [self.slot_to_type['person'] , 'person'] )

                    #if (self.attribute_name == 'holding'): # hardcode while issue #80 is not solved - after this line can be removed
                        #self.value.append( ['robot', 'mbot'] )

        return self.attribute_name, self.value


    def nluCallback(self, msg):
        '''
        This fuction will get executed upon receiving a msg on natural_language_processing topic

        Example of nlu output msg:
        recognized_action: take
        slot: ['coke is an object', 'kitchen is a destination']
        recognized_action: go
        slot: ['living room is a destination']
        recognized_action: find
        slot: ['person and is an object']
        recognized_action: tell
        slot: ['what time is what to tell']
        '''
        self.sentence_recognized = msg.sentence_recognition
        self.msg_received = True # this flag is set true everytime a new message is published on the recognized_intention topic


    def start_nlu_translate_upload(self):
        while not rospy.is_shutdown():
            if self.msg_received == True:
                # reset the flag
                self.msg_received = False
                
                # parse the message in phrases so each phrase can be translated into the pddl attributes and uploaded
                for phrase in self.sentence_recognized:
                    slot = []
                    for arg in phrase.slot:
                        slot.append(arg.split(' is')[0]) # split each argument by the ' is' keyword and use only the first block e.g. in ['living room is a destination'] we just retrieve 'living room'

                    # DEBUG prints
                    # print ("intention and slot !!")
                    # print phrase.recognized_action
                    # print slot
                    # print "------"
                    
                    if (not not slot):
                        # map the intentions and slots to predicates and types
                        attribute_name, value = self.map_nlu_to_pddl_domain(phrase.recognized_action, slot)

                        # upload the translated goals to the knowledgebase
                        # the 'ADD_GOAL' flag sets these attributes to be uploaded as goals. Use the flag 'ADD_KNOWLEDGE' to upload as facts.
                        # use 'REMOVE_KNOWLEDGE' and 'REMOVE_GOAL' to remove facts and goals, respectively.
                        self.nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', attribute_name, value, function_value=0.0, update_type='ADD_GOAL')

            # sleep the node for a predefined period of time to decrease the node resources load
            self.loop_rate.sleep()


def main():
    # launch the upload_pddl_knowledge_node so we are able to upload knowledge to the knowledgbase
    rospy.init_node('upload_pddl_knowledge_node', anonymous=False)

    # instantiate the nlu_knowledge_upload class in the nlu_knowledge_upload_obj object
    nlu_knowledge_upload_obj = nlu_knowledge_upload()

    # trigger the translation and upload to the knowledgebase process
    nlu_knowledge_upload_obj.start_nlu_translate_upload()
