#!/usr/bin/env python
import rospy
import sys

# for the msg_received in the nluCallback
from mbot_nlu.msg import ActionSlotArray

import knowledge_base_ros.upload_knowledge as upld

# import mapping from slot to the available types
from mbot_world_model_ros.gpsr_dict import slots_dict

# import mbot class
from mbot_robot_class_ros import mbot as mbot_class

from rosplan_knowledge_msgs.srv import GetAttributeService

#TODO: Test sentence and fix: "guide alice from the beddrom to the bathroom"
#nlu output: recognized_action: "guide"
#            slot: [alice is a person, bed into is a source, bathroom is a source]
#uploaded goal: goal:(at_p alice) fact:(at_p alice alice bathroom)

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
        #self.intention_to_action = {'go':'move_base' , 'grasp':'grasp', 'meet':'introduce' , 'take':'place' , 'guide':'guide' , 'find':'find_person' , 'tell':'tell' , 'answer':'answer_question', 'follow':'follow' }
        
        # dictionary that translates between the actions on the pddl domain and the corresponding predicates on the pddl domain
        #self.action_to_predicate = {'move_base':'at_r' , 'grasp':'holding', 'place':'on', 'find_person':'found', 'introduce':'known_p', 'guide':'at_p', 'answer_question':'iluminated' , 'tell':'told', 'follow':'following'}
        
        # dictionary that translates between the nlu slot output and the corresponding types on the pddl domain
        self.slot_to_type = slots_dict

        # Instantiating mbotRobot object
        self.mbot = mbot_class.mbotRobot(disabled={'perception': True, 'people_following': True, 'yolo': True, 'misc': True, 'hri': False, 'manipulation': True, 'navigation': True})

        self.knowledge_service = rospy.ServiceProxy('/kcl_rosplan/get_current_knowledge', GetAttributeService)

    def map_arguments_to_type(self, reconized_arguments):
        successful_mapping = True
        # mapping the recognized arguments (output from the nlu) to the corresponding types of the pddl domain
        arguments_types = []
        for arg in reconized_arguments:
            if (arg in self.slot_to_type):
                arguments_types.append([self.slot_to_type[arg], arg])
            elif (arg == 'yourself' or arg == 'me'):
                arguments_types.append([self.slot_to_type['person'], 'person'])
            else:
                print "BREAK"
                arguments_types = []
                successful_mapping = False
                break
            # elif (arg == 'noperson'):
            #     arguments_types.append([self.slot_to_type['person'], 'noperson'])

        print "Argtypes: ", arguments_types
        return arguments_types

    def split_slot(self, reconized_slot):
        arguments = []
        description = []
        for arg in reconized_slot:
            arguments.append(arg.split(' is ')[0])
            description.append(arg.split(' is ')[-1].split()[-1])

        # concatenate with and underscore arguments or intentions that are separated by spaces  
        for i in range(len(arguments)):
            arguments[i] = arguments[i].replace(' ', '_')

        return arguments, description

    def map_nlu_to_pddl_domain(self, reconized_intention, reconized_slot):

        slot_arguments, slot_description = self.split_slot(reconized_slot)
        print slot_arguments
        print slot_description

        goal_intention = []
        goal_arguments = []
        fact_intention = []
        fact_arguments = []

#--------------------------------------MOVE-------------------------------------------------#
#-------------------------------------------------------------------------------------------#
        if reconized_intention == 'go':
            for i in range(len(slot_description)):
                if 'destination' == slot_description[i]:
                    goal_intention = 'at_r'
                    goal_arguments.append(slot_arguments[i])

            if (goal_arguments):
                goal_arguments = self.map_arguments_to_type(goal_arguments)

#---------------------------------------GRASP-----------------------------------------------#
#-------------------------------------------------------------------------------------------#
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

#---------------------------------------KNOW------------------------------------------------#
#-------------------------------------------------------------------------------------------#
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

#---------------------------------------PLACE-----------------------------------------------#
#-------------------------------------------------------------------------------------------#
        elif reconized_intention == 'take':
            for i in range(len(slot_description)):
                if 'object' == slot_description[i]:
                    goal_intention = 'on'
                    goal_arguments.append(slot_arguments[i])

            if (goal_arguments):
                for i in range(len(slot_description)):
                    if 'destination' == slot_description[i]:
                        goal_arguments.append(slot_arguments[i])

            goal_arguments = self.map_arguments_to_type(goal_arguments)

#---------------------------------------GUIDE-----------------------------------------------#
#-------------------------------------------------------------------------------------------#
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


#-------------------------------FIND PERSON OR OBJECT---------------------------------------#
#-------------------------------------------------------------------------------------------#
        elif reconized_intention == 'find':
            for i in range(len(slot_description)):
                if 'person' == slot_description[i]:
                    goal_intention = 'found_p'
                    person_name = slot_arguments[i]
                    goal_arguments.append(person_name)

                elif 'object' == slot_description[i]:
                    goal_intention = 'found_obj'
                    object_name = slot_arguments[i]
                    goal_arguments.append(object_name)

            if (goal_arguments):
                for i in range(len(slot_description)):
                    if 'source' == slot_description[i] or 'destination' == slot_description[i]:
                        if (goal_intention == 'found_p'):
                            fact_intention = 'at_p'
                            fact_arguments.append(person_name)
                            fact_arguments.append(slot_arguments[i])
                        else:
                            fact_intention = 'on'
                            fact_arguments.append(object_name)
                            fact_arguments.append(slot_arguments[i])

                goal_arguments = self.map_arguments_to_type(goal_arguments)

                if (fact_arguments):
                    fact_arguments = self.map_arguments_to_type(fact_arguments)


#---------------------------------------FOLLOW----------------------------------------------#
#-------------------------------------------------------------------------------------------#
        elif reconized_intention == 'follow':
            for i in range(len(slot_description)):
                if 'person' == slot_description[i]:
                    goal_intention = 'following'
                    person_name = slot_arguments[i]
                    goal_arguments.append(person_name)

            if (goal_arguments):
                goal_arguments = self.map_arguments_to_type(goal_arguments)

                try:
                    robot_location = self.knowledge_service('at_r').attributes[0].values[0].value
                    fact_intention = 'at_p'
                    fact_arguments.append(person_name)
                    fact_arguments.append(robot_location)
                    fact_arguments = self.map_arguments_to_type(fact_arguments)

                except KeyError as e:
                    pass

        print "GOAL_INTENTION: ", goal_intention
        print "GOAL_ARGS: ", goal_arguments

        print "FACT_INTENTION: ", fact_intention
        print "FACT_ARGS: ", fact_arguments
        
        return goal_intention, goal_arguments, fact_intention, fact_arguments


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
                if (self.sentence_recognized):
                    for phrase in self.sentence_recognized:  
                        successful_upload = False

                        if (phrase.slot):
                            # map the intentions and slots to predicates and types
                            goal_intention, goal_arguments, fact_intention, fact_arguments = self.map_nlu_to_pddl_domain(phrase.recognized_action, phrase.slot)

                            if(goal_arguments):
                                successful_upload = True

                                # upload the translated goals to the knowledgebase
                                self.nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', goal_intention, goal_arguments, function_value=0.0, update_type='ADD_GOAL')

                                if(fact_arguments):
                                    # upload the translated goals to the knowledgebase
                                    self.nlu_knowledge_uploader.rosplan_update_knowledge(1, '', '', fact_intention, fact_arguments, function_value=0.0, update_type='ADD_KNOWLEDGE')

                            # the 'ADD_GOAL' flag sets these attributes to be uploaded as goals. Use the flag 'ADD_KNOWLEDGE' to upload as facts.
                            # use 'REMOVE_KNOWLEDGE' and 'REMOVE_GOAL' to remove facts and goals, respectively.

                    if (not successful_upload):
                        #Say sorry for not being able to upload a goal
                        self.mbot.hri.say('I am sorry I cannot do it')

                else:
                    #Say sorry for not being able to upload a goal
                    self.mbot.hri.say('I am sorry I cannot do it')


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
