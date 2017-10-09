#!/usr/bin/env python
import rospy
from rosplan_knowledge_msgs.srv import GetAttributeService
from std_msgs.msg import String

"""
1.
performs a service call to:
rosservice call /kcl_rosplan/get_current_goals "predicate_name: ''"
check if it is empty, this means there are no unfinished goals

2. performs a service call to:
rosservice call /kcl_rosplan/get_current_knowledge "predicate_name: ''"
stores this value in memory as string
if this string value does not change with respect to the previous request
then it means knowledge base has not changed. In the scenario in which the planner
fails to create a plan, it makes no sense to try to plan again if  the knowledge base
does not change. This component is useful for that purpose
"""

class KnowledgeBaseAnalizer(object):
    """
    Analyzes if there are new goals in the knowledge base

    """

    def __init__(self):
        # Subscribers
        rospy.Subscriber("~pending_goals/event_in", String, self.pending_goals_event_in_cb)
        rospy.Subscriber("~new_knowledge/event_in", String, self.new_knowledge_event_in_cb)
        # Publishers
        self.pending_goals_event_out = rospy.Publisher('~pending_goals/event_out', String)
        self.new_knowledge_event_out = rospy.Publisher('~new_knowledge/event_out', String)
        # for keeping memory of the information inside the knowledge base
        self.knowledge_base_info = 'attributes: []'
        

    def pending_goals_event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        rospy.loginfo('Pending goal request received...')
        rospy.loginfo('Waiting for service: /kcl_rosplan/get_current_goals')
        rospy.wait_for_service('/kcl_rosplan/get_current_goals')
        rospy.loginfo('Service /kcl_rosplan/get_current_goals is available, proceeding')
        event_out = String()
        event_out.data = 'e_failure'
        try:
            pending_goals = rospy.ServiceProxy('/kcl_rosplan/get_current_goals', GetAttributeService)
            response = pending_goals('')
            if str(response) == 'attributes: []':
                rospy.loginfo('There are no pending goals in the knowledge base')
                event_out.data = 'e_failure'
            else:
                rospy.loginfo('There are pending goals in the knowledge base')
                event_out.data = 'e_success'
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s'%e)
        rospy.loginfo('Publishing pending goals response')
        self.pending_goals_event_out.publish(event_out)
        
        
    def new_knowledge_event_in_cb(self, msg):
        """
        Gets information from database and informs if new knowledge has been received
        """
        rospy.loginfo('New knowledge request received...')
        rospy.loginfo('Waiting for service: /kcl_rosplan/get_current_knowledge')
        rospy.wait_for_service('/kcl_rosplan/get_current_knowledge')
        rospy.loginfo('Service /kcl_rosplan/get_current_knowledge is available, proceeding')
        event_out = String()
        event_out.data = 'e_failure'
        try:
            knowledge_request = rospy.ServiceProxy('/kcl_rosplan/get_current_knowledge', GetAttributeService)
            response = knowledge_request('')
            if str(response) != self.knowledge_base_info:
                rospy.loginfo('There is new knowledge stored in the knowledge base')
                event_out.data = 'e_success'
            else:
                rospy.logwarn('There is no new knowledge stored in the knowledge base')
                event_out.data = 'e_failure'
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s'%e)
        self.knowledge_base_info = str(response)
        rospy.loginfo('Publishing new knowledge response')
        self.new_knowledge_event_out.publish(event_out)
        
        
    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo('Pending_goals_analyzer node initilized...')
        rospy.spin()
    
    
def main():
    rospy.init_node('pending_goals_analyzer')
    knowledge_base_analyzer = KnowledgeBaseAnalizer()
    knowledge_base_analyzer.start()
    