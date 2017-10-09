#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# function to upload knowledge to knowledge base
from mcr_knowledge_ros import upload_knowledge

# for service call to find out the location of the robot
from rosplan_knowledge_msgs.srv import GetAttributeService

class SpeechToKnowledge(object):
    '''
    Trigger the autom param tuning coordinator and configures some parameters for gmapping
    then listens to the quality score published by pcl ICP (iterative closest point) and outputs a log file
    with information about the run.
    '''
    def __init__(self):
        # class variables
        self.speech_string = None
        self.speech_received = False
        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # inform user about node initialization
        rospy.loginfo("Initializing speech to knowledge node...")
        # create subscriber
        rospy.Subscriber("~speech_recognition", String, self.speechCallback)
        # create publisher for the robot to speak
        self.pub_talk = rospy.Publisher("say", String, queue_size=1)
        # create publisher to get angry when you command to go to the same spot
        self.pub_emotion = rospy.Publisher("led_emotions_interface/emotions", String, queue_size=1)
        # give some time for subscriber to register in the network
        rospy.sleep(0.2)

    def robot_is_at(self, location):
        '''
        query the knowledge base to confirm if the robot is already at the location
        
        Example on how to call the service from terminal:
        
        rosservice call /kcl_rosplan/get_current_knowledge "predicate_name: 'at'"
        '''
        rospy.loginfo('query from KB the location of the robot')
        rospy.wait_for_service('kcl_rosplan/get_current_knowledge')
        rospy.loginfo('service kcl_rosplan/get_current_knowledge is available')
        try:
            get_location = rospy.ServiceProxy('kcl_rosplan/get_current_knowledge', GetAttributeService)
            resp1 = get_location('at')
            robot_location = resp1.attributes[0].values[1].value
            if robot_location == location:
                return True
            else:
                return False
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def identify_string_then_upload_knowledge(self):
        '''
        Upload knowledge based upon received string in speech topic
        '''
        if self.speech_string == "hello robot":
            rospy.loginfo("uploading fact human_is_puzzled(unknown_visitor) and goal human_is_answered(unknown_visitor)")
            upload_knowledge.rosplan_update_knowledge(1, '', '', 'human_is_puzzled', [['h', 'unknown_visitor']], update_type='ADD_KNOWLEDGE')
            upload_knowledge.rosplan_update_knowledge(1, '', '', 'human_is_answered', [['h', 'unknown_visitor']], update_type='ADD_GOAL')
        elif self.speech_string == "robot go to the testbed":
            if self.robot_is_at("testbed"):
                rospy.logwarn("robot is already at the testbed, doing nothing")
                self.pub_talk.publish(String("I am already at the testbed, do not try to fool me"))
                self.pub_emotion.publish(String("Anger"))
                return
            rospy.loginfo("uploading goal at(robot testbed)")
            upload_knowledge.rosplan_update_knowledge(1, '', '', 'at', [['r', 'monarch'], ['l','testbed']], update_type='ADD_GOAL')
        elif self.speech_string == "robot go to the coffee machine":
            if self.robot_is_at("coffee_machine"):
                rospy.logwarn("robot is already at the coffee machine, doing nothing")
                self.pub_talk.publish(String("I am already at the coffee machine, yummi what a nice coffee"))
                self.pub_emotion.publish(String("Fear"))
                return
            rospy.loginfo("uploading goal at(robot coffee_machine)")
            upload_knowledge.rosplan_update_knowledge(1, '', '', 'at', [['r', 'monarch'], ['l','coffee_machine']], update_type='ADD_GOAL')
        else:
            rospy.logerr("speech sentence not supported")
        #elif self.speech_string == "robot follow me":
        #    upload_knowledge.rosplan_update_knowledge(1, '', '', 'todo', [['key', 'value']], update_type='ADD_GOAL')

    def speechCallback(self, msg):
        self.speech_string = msg.data
        self.speech_received = True
        rospy.loginfo("speech input has been received !")

    def start(self):
        # wait for speech input
        while not rospy.is_shutdown():
            if self.speech_received:
                self.speech_received = False
                self.identify_string_then_upload_knowledge()
            else:
                self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('speech_to_knowledge', anonymous=False)
    speech_to_knowledge = SpeechToKnowledge()
    speech_to_knowledge.start()
