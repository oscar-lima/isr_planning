#!/usr/bin/env python
import rospy

import sys

# for the msg_received in the nluCallback
from mbot_nlu.msg import ActionSlotArray

import knowledge_base_ros.upload_knowledge as upld


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
        self.slot_to_type = {'summer': 'person', 'madison': 'person', 'freddie': 'person', 'bananas': 'object', 'office': 'location',
                            'lewis': 'person', 'laptop': 'object', 'charles': 'person', 'toby': 'person', 'thomas': 'person',
                            'joshua': 'person', 'peaches': 'object', 'iced tea': 'object', 'logan': 'person', 'bedside': 'location',
                            'harrison': 'person', 'bedroom': 'location', 'faith': 'person', 'milk': 'object', 'emily': 'person',
                            'blake': 'person', 'blueberries': 'object', 'pens': 'object', 'ham': 'object', 'tablet': 'object',
                            'matthew': 'person', 'peanut': 'object', 'noah': 'person', 'josh': 'person', 'sugar': 'object',
                            'TV stand': 'location', 'center table': 'location', 'ethan': 'person', 'orange': 'object',
                            'peanuts': 'object', 'manju': 'object', 'scarlett': 'person', 'theo': 'person', 'plate': 'object',
                            'coffee': 'object', 'rose': 'person', 'blackberries': 'object', 'food': 'object', 'luke': 'person',
                            'samuel': 'person', 'fridge': 'location', 'oranges': 'object', 'amy': 'person', 'barbara': 'person',
                            'dryer': 'object', 'watermelon': 'object', 'jack': 'person', 'desk': 'location', 'jacob': 'person',
                            'bar': 'l', 'chips': 'object', 'bread': 'object', 'towel': 'object', 'onion': 'object',
                            'sofa': 'location', 'dining_table': 'location', 'cider': 'object', 'alice': 'person', 'candy': 'object',
                            'eleanor': 'person', 'arthur': 'person', 'rosie': 'person', 'knife': 'object', 'james': 'person',
                            'biscuits': 'object', 'glasses': 'object', 'louis': 'person', 'cookies': 'object', 'isabelle': 'person',
                            'knifes': 'object', 'tea': 'object', 'almond': 'object', 'lucy': 'person', 'bowls': 'object',
                            'daisy': 'person', 'sponge': 'object', 'burger': 'object', 'almonds': 'object', 'ryan': 'person',
                            'erika': 'person', 'noodles': 'object', 'william': 'person', 'container': 'object',
                            'chewing gums': 'object', 'closet': 'location', 'harvey': 'person', 'alex': 'person', 'emma': 'person',
                            'pen': 'object', 'daniel': 'person', 'living table': 'location', 'yogurt': 'object', 'forks': 'object',
                            'choth': 'object', 'mints': 'object', 'blackberry': 'object', 'ken': 'person', 'pepper': 'object',
                            'living_room': 'location', 'rum': 'object', 'harry': 'person', 'lotion': 'object',
                            'toothbrush': 'object', 'cake': 'object', 'snack': 'object', 'cookie': 'object', 'sink': 'location',
                            'grapes': 'object', 'brooke': 'person', 'comb': 'object', 'sophie': 'person',
                            'chocolate tablet': 'object', 'kitchen': 'location', 'water': 'object', 'bedroom chair': 'location',
                            'tommy': 'person', 'tray': 'object', 'biscuit': 'object', 'jackie': 'person', 'magazine': 'object',
                            'bookcase': 'location', 'donuts': 'object', 'emilia': 'person', 'amber': 'person', 'cheese': 'object',
                            'henry': 'person', 'apple': 'object', 'sideshelf': 'location', 'charlotte': 'person',
                            'sienna': 'person', 'michael': 'person', 'magazines': 'object', 'chewing gum': 'object',
                            'apples': 'object', 'table': 'location', 'kitchen chair': 'location', 'florence': 'person',
                            'trays': 'object', 'katie': 'person', 'sarah': 'person', 'bathroom': 'location', 'oliver': 'person',
                            'isaac': 'person', 'sake': 'object', 'toothpaste': 'object', 'aaron': 'person', 'beer': 'object',
                            'edward': 'person', 'amelie': 'person', 'jane': 'person', 'john': 'person', 'blueberry': 'object',
                            'containers': 'object', 'fork': 'object', 'pencil': 'object', 'paprika': 'object', 'door': 'location',
                            'sprite': 'object', 'cereals bar': 'object', 'oscar': 'person', 'coke': 'object', 'jamie': 'person',
                            'cabinet': 'location', 'glass': 'object', 'strawberries': 'object', 'lemons': 'object',
                            'amelia': 'person', 'grace': 'person', 'whisky': 'object', 'hot chocolate': 'object',
                            'nathan': 'person', 'drawer': 'location', 'hanna': 'person', 'shampoo': 'object', 'bed': 'location',
                            'banana': 'object', 'will': 'person', 'strawberry': 'object', 'books': 'object', 'lemonade': 'object',
                            'can': 'object', 'charlie': 'person', 'newspaper': 'object', 'crackers': 'object',
                            'newspapers': 'object', 'erik': 'person', 'toilet': 'location', 'vodka': 'object', 'chloe': 'person',
                            'snacks': 'object', 'candies': 'object', 'sushi': 'object', 'senbei': 'object', 'pie': 'object',
                            'juice': 'object', 'cans': 'object', 'onions': 'object', 'ivy': 'person', 'cream': 'object',
                            'peter': 'person', 'lemon': 'object', 'zoe': 'person', 'peach': 'object', 'bowl': 'object',
                            'couch': 'location', 'brian': 'person', 'book': 'object', 'corridor': 'location', 'plates': 'object',
                            'martha': 'person', 'lily': 'person', 'soap': 'object', 'pringles': 'object', 'pizza': 'object',
                            'cupboard': 'location', 'chocolate bar': 'object', 'samantha': 'person', 'ella': 'person',
                            'drink': 'object', 'pear': 'object', 'dylan': 'person', 'chocolate egg': 'object', 'tyler': 'person',
                            'notebook': 'object', 'evan': 'person', 'toilet paper': 'object', 'connor': 'person',
                            'bottles': 'object', 'red bull': 'object', 'max': 'person', 'joseph': 'person', 'skyler': 'person',
                            'cloth': 'object', 'charger': 'object', 'toiletries': 'object', 'counter': 'location', 'seth': 'person',
                            'adam': 'person', 'bottle': 'object', 'olivia': 'person', 'elliot': 'person', 'paige': 'person',
                            'salt': 'object', 'wine': 'object', 'nightstand':'location', 'hallway':'location', 'wardrobe':'location',
                            'bookshelf':'location', 'coffee table':'location', 'sidetable':'location', 'dining_room':'location',
                            'kitchen_table':'location', 'kitchen_cabinet':'location', 'rice':'object',
                            'kleenex':'object', 'whiteboard_cleaner':'object', 'cup':'object', 'drink':'object', 'food':'object',
                            'cleaning_stuff':'object', 'mia':'person', 'liam':'person', 'sophia':'person', 'abigail':'person',
                            'mason':'person', 'alexander':'person', 'isabella':'person', 'ava':'person'}


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

                if (self.attribute_name == 'holding'): # hardcode while issue #80 is not solved - after this line can be removed
                    self.value.append( ['robot', 'mbot'] )

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
