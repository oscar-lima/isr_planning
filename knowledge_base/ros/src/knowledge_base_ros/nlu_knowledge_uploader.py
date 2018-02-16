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
        rospy.Subscriber("~intention_and_args", ActionSlotArray, self.nluCallback)

        # get from param server the frequency at which this node will pool incoming messages
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # dictionary that translates between the nlu intention output and the corresponding actions on the pddl domain
        self.intention_to_action = {'go':'move_base' , 'grasp':'grasp', 'meet':'introduce' , 'take':'place' , 'guide':'guide' , 'find':'find_person' , 'tell':'tell' , 'answer':'answer_question', 'follow':'follow' }
        
        # dictionary that translates between the actions on the pddl domain and the corresponding predicates on the pddl domain
        self.action_to_predicate = {'move_base':'at_r' , 'grasp':'holding', 'place':'on', 'find_person':'found', 'introduce':'known_p', 'guide':'at_p', 'answer_question':'iluminated' , 'tell':'told', 'follow':'following'}
        
        # dictionary that translates between the nlu slot output and the corresponding types on the pddl domain
        self.slot_to_type = slots_dict


    def map_arguments_to_type(self, reconized_arguments):
        # mapping the recognized arguments (output from the nlu) to the corresponding types of the pddl domain
        self.arguments_types = []
        for arg in reconized_arguments:
            if (arg in self.slot_to_type):
                self.arguments_types.append([self.slot_to_type[arg], arg])
            elif (arg == 'yourself' or arg == 'me'):
                self.arguments_types.append([self.slot_to_type['person'], 'person'])
            elif (arg == 'noperson'):
                self.arguments_types.append([self.slot_to_type['person'], 'noperson'])

        return self.arguments_types

    def split_slot(self, reconized_slot):
        self.arguments = []
        self.description = []
        for arg in reconized_slot:
            self.arguments.append(arg.split(' is ')[0])
            self.description.append(arg.split(' is ')[-1].split()[-1])

        # concatenate with and underscore arguments or intentions that are separated by spaces  
        for i in range(len(self.arguments)):
            self.arguments[i] = self.arguments[i].replace(' ', '_')

        return self.arguments, self.description

    def map_nlu_to_pddl_domain(self, reconized_intention, reconized_slot):

        slot_arguments, slot_description = self.split_slot(reconized_slot)
        print slot_arguments
        print slot_description

        goal_intention = []
        goal_arguments = []
        fact_intention = []
        fact_arguments = []

        if reconized_intention == 'go':
            for i in range(len(slot_description)):
                if 'destination' == slot_description[i]:
                    goal_intention = 'at_r'
                    goal_arguments.append(slot_arguments[i])

            goal_arguments = self.map_arguments_to_type(goal_arguments)


        elif reconized_intention == 'grasp':
            for i in range(len(slot_description)):
                if 'object' == slot_description[i]:
                    goal_intention = 'holding'
                    object_name = slot_arguments[i]
                    goal_arguments.append(object_name)

            if (goal_arguments):
                for i in range(len(slot_description)):
                    if 'source' == slot_description[i] or 'destination' == slot_description[i]:
                        fact_intention = 'on'
                        fact_arguments.append(object_name)
                        fact_arguments.append(slot_arguments[i])

                goal_arguments = self.map_arguments_to_type(goal_arguments)

                if (fact_arguments):
                    fact_arguments = self.map_arguments_to_type(fact_arguments)


        elif reconized_intention == 'meet':
            for i in range(len(slot_description)):
                if 'person' == slot_description[i]:
                    goal_intention = 'known_p'
                    person_name = slot_arguments[i]
                    goal_arguments.append(person_name)

            if (goal_arguments):
                for i in range(len(slot_description)):
                    if 'destination' == slot_description[i]:
                        fact_intention = 'at_p'
                        fact_arguments.append(person_name)
                        fact_arguments.append(slot_arguments[i])

                goal_arguments = self.map_arguments_to_type(goal_arguments)


                if (fact_arguments):
                    fact_arguments = self.map_arguments_to_type(fact_arguments)


        elif reconized_intention == 'guide':
            for i in range(len(slot_description)):
                if 'person' == slot_description[i]:
                    goal_intention = 'at_p'
                    person_name = slot_arguments[i]
                    goal_arguments.append(person_name)

            if (goal_arguments):
                for i in range(len(slot_description)):
                    if 'destination' == slot_description[i]:
                        goal_arguments.append(slot_arguments[i])

                    elif 'source' == slot_description[i]:
                        fact_intention = 'at_p'
                        fact_arguments.append(person_name)
                        fact_arguments.append(slot_arguments[i])

                goal_arguments = self.map_arguments_to_type(goal_arguments)

                if (fact_arguments):
                    fact_arguments = self.map_arguments_to_type(fact_arguments)


        print 'upload goals to the knowledgebase'
        print "Att: ", goal_intention
        print "Val: ", goal_arguments

        print 'upload facts to the knowledgebase'
        print "Att: ", fact_intention
        print "Val: ", fact_arguments
        
        return goal_intention, goal_arguments, fact_intention, fact_arguments


        '''
        # mapping the recognized intention (output from the nlu) to the corresponding predicate of the pddl domain
        if(reconized_intention in self.intention_to_action):
            if(self.intention_to_action[reconized_intention] in self.action_to_predicate):
                self.attribute_name = self.action_to_predicate[self.intention_to_action[reconized_intention]]

            # mapping the recognized slots (output from the nlu) to the corresponding types of the pddl domain
            self.value = []
            for argument in reconized_slot:
                if(argument in self.slot_to_type):
                    self.value.append( [self.slot_to_type[argument] , argument] )
                elif(argument == 'yourself' or argument == 'me'):
                    self.value.append( [self.slot_to_type['person'] , 'person'] )

                    #if (self.attribute_name == 'holding'): # hardcode while issue #80 is not solved - after this line can be removed
                        #self.value.append( ['robot', 'mbot'] )
        '''
        #print "DEBUG"
        #print "***Att: ", self.attribute_name
        #print "***Val: ", self.value
        #self.attribute_name = 'aa'
        #self.value = 'bb'
        #return self.attribute_name, self.value


    def nluCallback(self, msg):
        print "----NEW DATA---"
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

                print '...........................'
                print self.sentence_recognized
                print '...........................'

                # parse the message in phrases so each phrase can be translated into the pddl attributes and uploaded
                for phrase in self.sentence_recognized:                    
                    if (phrase.slot):
                        # map the intentions and slots to predicates and types
                        goal_intention, goal_arguments, fact_intention, fact_arguments = self.map_nlu_to_pddl_domain(phrase.recognized_action, phrase.slot)

                        if(goal_arguments):
                            # upload the translated goals to the knowledgebase
                            self.nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', goal_intention, goal_arguments, function_value=0.0, update_type='ADD_GOAL')

                            if(fact_arguments):
                                # upload the translated goals to the knowledgebase
                                self.nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', fact_intention, fact_arguments, function_value=0.0, update_type='ADD_KNOWLEDGE')

                        # the 'ADD_GOAL' flag sets these attributes to be uploaded as goals. Use the flag 'ADD_KNOWLEDGE' to upload as facts.
                        # use 'REMOVE_KNOWLEDGE' and 'REMOVE_GOAL' to remove facts and goals, respectively.


            # sleep the node for a predefined period of time to decrease the node resources load
            self.loop_rate.sleep()


def main():
    print "----------------Begin---------------------"
    # launch the upload_pddl_knowledge_node so we are able to upload knowledge to the knowledgbase
    rospy.init_node('upload_pddl_knowledge_node', anonymous=False)

    # instantiate the nlu_knowledge_upload class in the nlu_knowledge_upload_obj object
    nlu_knowledge_upload_obj = nlu_knowledge_upload()

    # trigger the translation and upload to the knowledgebase process
    nlu_knowledge_upload_obj.start_nlu_translate_upload()
