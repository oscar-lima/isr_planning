#!/usr/bin/env python
import rospy
import rosservice
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
fails to create a plan, it makes no sense to try to plan again if the knowledge base
does not change. This component is useful for that purpose.
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
        self.pending_goals_event_out = rospy.Publisher('~pending_goals/event_out', String, queue_size=1)
        self.new_knowledge_event_out = rospy.Publisher('~new_knowledge/event_out', String, queue_size=1)
        # for keeping memory of the information inside the knowledge base
        self.knowledge_base_info = 'attributes: []'
        # The maximum amount of time to wait for the rosplan knowledge base service to show
        self.kb_service_timeout = rospy.get_param('~kb_service_timeout', 3.0)
        # to control the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))
        # inform the user that the node has finished initialization
        rospy.loginfo('knowledge base analyzer node initilized... ready to accept requests')
        # member variables
        self.is_new_knowledge_request_received = False
        self.new_knowledge_request_msg = None
        self.is_goals_available_request_received = False


    def pending_goals_event_in_cb(self, msg):
        """
        Callback indicating a request to know if goals are available in the KB
        """
        rospy.logdebug('Pending goal request received...')
        self.is_goals_available_request_received = True
        self.goals_available_request_msg = msg


    def new_knowledge_event_in_cb(self, msg):
        """
        Callback indicating a request to know if new knowledge is available in the KB
        """
        rospy.logdebug('New knowledge request received...')
        self.is_new_knowledge_request_received = True
        self.new_knowledge_request_msg = msg


    def wait_for_service(self, service_name, timeout):
        """
        Wait for service existance within a timeout, handling possible errors that might rise
        """
        rospy.loginfo('Waiting for service: ' + service_name)
        try:
            resp1 = rospy.wait_for_service(service_name, timeout)
        except rospy.ROSException as exc:
            rospy.logerr('Exception while calling ' + service_name + ' service, does it exist?')
            return False
        rospy.logdebug('Service ' + service_name + ' is available, proceeding')
        return True


    def look_for_unfinished_goals(self):
        """
        Query KB to see if there are unfinished goals
        """
        if not self.wait_for_service('/kcl_rosplan/get_current_goals', self.kb_service_timeout):
            # rosplan kb service most likely not available
            return False
        # call rosplan KB service for analysis
        try:
            pending_goals = rospy.ServiceProxy('/kcl_rosplan/get_current_goals', GetAttributeService)
            response = pending_goals('')
            if str(response) == 'attributes: []':
                rospy.logdebug('There are no pending goals in the knowledge base')
                return False
            else:
                rospy.logdebug('There are pending goals in the knowledge base')
                return True
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s'%e)


    def look_for_new_knowledge(self):
        """
        Query KB to see if there is new knowledge available
        """
        if not self.wait_for_service('/kcl_rosplan/get_current_knowledge', self.kb_service_timeout):
            # rosplan kb service most likely not available
            return False
        # call rosplan KB service for analysis
        try:
            knowledge_request = rospy.ServiceProxy('/kcl_rosplan/get_current_knowledge', GetAttributeService)
            response = knowledge_request('')
            if str(response) != self.knowledge_base_info:
                rospy.loginfo('There is new knowledge stored in the knowledge base')
                # backup last KB content
                self.knowledge_base_info = str(response)
                return True
            else:
                rospy.logwarn('There is no new knowledge stored in the knowledge base')
                return False
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s'%e)


    def start_knowledge_base_analyzer(self):
        """
        knowledge base analyzer main loop
        """
        while not rospy.is_shutdown():
            if self.is_new_knowledge_request_received == True:
                # lower flag
                self.is_new_knowledge_request_received = False
                # analyze msg content
                if self.new_knowledge_request_msg.data != 'e_start':
                    rospy.logerr('event not supported, admissible values are : e_start')
                    self.new_knowledge_event_out.publish(String('e_failure'))
                    continue
                if self.look_for_new_knowledge():
                    self.new_knowledge_event_out.publish(String('e_new_knowledge'))
                else:
                    self.new_knowledge_event_out.publish(String('e_no_new_knowledge'))
            elif self.is_goals_available_request_received == True:
                # lower flag
                self.is_goals_available_request_received = False
                # analyze msg content
                if self.goals_available_request_msg.data != 'e_start':
                    rospy.logerr('event not supported, admissible values are : e_start')
                    self.pending_goals_event_out.publish(String('e_failure'))
                    continue
                if self.look_for_unfinished_goals():
                    rospy.loginfo('There are unfinished goals in the KB')
                    self.pending_goals_event_out.publish(String('e_new_goals'))
                else:
                    self.pending_goals_event_out.publish(String('e_no_new_goals'))
            self.loop_rate.sleep()


def main():
    rospy.init_node('knowledge_base_analyzer_node')
    knowledge_base_analyzer = KnowledgeBaseAnalizer()
    knowledge_base_analyzer.start_knowledge_base_analyzer()
