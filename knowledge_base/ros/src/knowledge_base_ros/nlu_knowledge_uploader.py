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
        rospy.Subscriber("~intention_and_args", ActionSlotArray, self.nluCallback)

        # get from param server the frequency at which this node will pool incoming messages
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # dictionary that translates between the nlu intention output and the corresponding actions on the pddl domain
        self.intention_to_action = {'go':'move_base' , 'grasp':'grasp', 'meet':'introduce' , 'take':'place' , 'guide':'guide' , 'find':'find_person' , 'tell':'tell' , 'answer':'answer_question' }
        
        # dictionary that translates between the actions on the pddl domain and the corresponding predicates on the pddl domain
        self.action_to_predicate = {'move_base':'at_r' , 'grasp':'holding', 'place':'on', 'find_person':'found', 'introduce':'known_p', 'guide':'at_p', 'answer_question':'iluminated' , 'tell':'told'}
        
        # dictionary that translates between the nlu slot output and the corresponding types on the pddl domain
        self.slot_to_type = {'summer': 'p', 'madison': 'p', 'freddie': 'p', 'bananas': 'obj', 'office': 'l',
                                    'lewis': 'p', 'laptop': 'obj', 'charles': 'p', 'toby': 'p', 'thomas': 'p',
                                    'joshua': 'p', 'peaches': 'obj', 'iced tea': 'obj', 'logan': 'p', 'bedside': 'l',
                                    'harrison': 'p', 'bedroom': 'l', 'faith': 'p', 'milk': 'obj', 'emily': 'p',
                                    'blake': 'p', 'blueberries': 'obj', 'pens': 'obj', 'ham': 'obj', 'tablet': 'obj',
                                    'matthew': 'p', 'peanut': 'obj', 'noah': 'p', 'josh': 'p', 'sugar': 'obj',
                                    'TV stand': 'l', 'center table': 'l', 'ethan': 'p', 'orange': 'obj',
                                    'peanuts': 'obj', 'manju': 'obj', 'scarlett': 'p', 'theo': 'p', 'plate': 'obj',
                                    'coffee': 'obj', 'rose': 'p', 'blackberries': 'obj', 'food': 'obj', 'luke': 'p',
                                    'samuel': 'p', 'fridge': 'l', 'oranges': 'obj', 'amy': 'p', 'barbara': 'p',
                                    'dryer': 'obj', 'watermelon': 'obj', 'jack': 'p', 'desk': 'l', 'jacob': 'p',
                                    'bar': 'l', 'chips': 'obj', 'bread': 'obj', 'towel': 'obj', 'onion': 'obj',
                                    'sofa': 'l', 'dining_table': 'l', 'cider': 'obj', 'alice': 'p', 'candy': 'obj',
                                    'eleanor': 'p', 'arthur': 'p', 'rosie': 'p', 'knife': 'obj', 'james': 'p',
                                    'biscuits': 'obj', 'glasses': 'obj', 'louis': 'p', 'cookies': 'obj', 'isabelle': 'p',
                                    'knifes': 'obj', 'tea': 'obj', 'almond': 'obj', 'lucy': 'p', 'bowls': 'obj',
                                    'daisy': 'p', 'sponge': 'obj', 'burger': 'obj', 'almonds': 'obj', 'ryan': 'p',
                                    'erika': 'p', 'noodles': 'obj', 'william': 'p', 'container': 'obj',
                                    'chewing gums': 'obj', 'closet': 'l', 'harvey': 'p', 'alex': 'p', 'emma': 'p',
                                    'pen': 'obj', 'daniel': 'p', 'living table': 'l', 'yogurt': 'obj', 'forks': 'obj',
                                    'choth': 'obj', 'mints': 'obj', 'blackberry': 'obj', 'ken': 'p', 'pepper': 'obj',
                                    'living_room': 'l', 'rum': 'obj', 'harry': 'p', 'lotion': 'obj',
                                    'toothbrush': 'obj', 'cake': 'obj', 'snack': 'obj', 'cookie': 'obj', 'sink': 'l',
                                    'grapes': 'obj', 'brooke': 'p', 'comb': 'obj', 'sophie': 'p',
                                    'chocolate tablet': 'obj', 'kitchen': 'l', 'water': 'obj', 'bedroom chair': 'l',
                                    'tommy': 'p', 'tray': 'obj', 'biscuit': 'obj', 'jackie': 'p', 'magazine': 'obj',
                                    'bookcase': 'l', 'donuts': 'obj', 'emilia': 'p', 'amber': 'p', 'cheese': 'obj',
                                    'henry': 'p', 'apple': 'obj', 'sideshelf': 'l', 'charlotte': 'p',
                                    'sienna': 'p', 'michael': 'p', 'magazines': 'obj', 'chewing gum': 'obj',
                                    'apples': 'obj', 'table': 'l', 'kitchen chair': 'l', 'florence': 'p',
                                    'trays': 'obj', 'katie': 'p', 'sarah': 'p', 'bathroom': 'l', 'oliver': 'p',
                                    'isaac': 'p', 'sake': 'obj', 'toothpaste': 'obj', 'aaron': 'p', 'beer': 'obj',
                                    'edward': 'p', 'amelie': 'p', 'jane': 'p', 'john': 'p', 'blueberry': 'obj',
                                    'containers': 'obj', 'fork': 'obj', 'pencil': 'obj', 'paprika': 'obj', 'door': 'l',
                                    'sprite': 'obj', 'cereals bar': 'obj', 'oscar': 'p', 'coke': 'obj', 'jamie': 'p',
                                    'cabinet': 'l', 'glass': 'obj', 'strawberries': 'obj', 'lemons': 'obj',
                                    'amelia': 'p', 'grace': 'p', 'whisky': 'obj', 'hot chocolate': 'obj',
                                    'nathan': 'p', 'drawer': 'l', 'hanna': 'p', 'shampoo': 'obj', 'bed': 'l',
                                    'banana': 'obj', 'will': 'p', 'strawberry': 'obj', 'books': 'obj', 'lemonade': 'obj',
                                    'can': 'obj', 'charlie': 'p', 'newspaper': 'obj', 'crackers': 'obj',
                                    'newspapers': 'obj', 'erik': 'p', 'toilet': 'l', 'vodka': 'obj', 'chloe': 'p',
                                    'snacks': 'obj', 'candies': 'obj', 'sushi': 'obj', 'senbei': 'obj', 'pie': 'obj',
                                    'juice': 'obj', 'cans': 'obj', 'onions': 'obj', 'ivy': 'p', 'cream': 'obj',
                                    'peter': 'p', 'lemon': 'obj', 'zoe': 'p', 'peach': 'obj', 'bowl': 'obj',
                                    'couch': 'l', 'brian': 'p', 'book': 'obj', 'corridor': 'l', 'plates': 'obj',
                                    'martha': 'p', 'lily': 'p', 'soap': 'obj', 'pringles': 'obj', 'pizza': 'obj',
                                    'cupboard': 'l', 'chocolate bar': 'obj', 'samantha': 'p', 'ella': 'p',
                                    'drink': 'obj', 'pear': 'obj', 'dylan': 'p', 'chocolate egg': 'obj', 'tyler': 'p',
                                    'notebook': 'obj', 'evan': 'p', 'toilet paper': 'obj', 'connor': 'p',
                                    'bottles': 'obj', 'red bull': 'obj', 'max': 'p', 'joseph': 'p', 'skyler': 'p',
                                    'cloth': 'obj', 'charger': 'obj', 'toiletries': 'obj', 'counter': 'l', 'seth': 'p',
                                    'adam': 'p', 'bottle': 'obj', 'olivia': 'p', 'elliot': 'p', 'paige': 'p',
                                    'salt': 'obj', 'wine': 'obj', 'nightstand':'l', 'hallway':'l', 'wardrobe':'l',
                                    'bookshelf':'l', 'coffee_table':'l', 'sidetable':'l', 'dining_room':'l',
                                    'kitchen_table':'l', 'kitchen_cabinet':'l', 'rice':'obj',
                                    'kleenex':'obj', 'whiteboard cleaner':'obj', 'cup':'obj', 'drink':'obj', 'food':'obj',
                                    'cleaning_stuff':'obj', 'mia':'p', 'liam':'p', 'sophia':'p', 'abigail':'p',
                                    'mason':'p', 'alexander':'p', 'isabella':'p', 'ava':'p', 'person':'p'}

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
