#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import CompletePlan
from diagnostic_msgs.msg import KeyValue
# from plan_parse_dispatch.plan_parser_utils import convert_IPC2014_plan_file_to_list
# from plan_parse_dispatch.plan_parser_utils import Neutral
import plan_parse_dispatch.plan_parser_utils

class PlanParseDispatch(object):
    '''
    reads plan from ~/.ros/mercury.plan , parses it and generates
    dispatch msgs that are published in /plan topic
    '''
    def __init__(self):
        # subscriptions
        rospy.Subscriber("~event_in", String, self.eventInCallBack, queue_size=1)
        # publications
        self.pub_event_out = rospy.Publisher('~event_out', String, queue_size=1)
        self.pub_plan = rospy.Publisher('~plan', CompletePlan, queue_size=1)
        # member variables
        self.event_in_received = False
        # fetch parameter from param server
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        self.plan_path = rospy.get_param('~plan_path', '/home/user/mercury.plan')
        self.plan_parser = rospy.get_param('~plan_parser', 'IPC2014')
        rospy.loginfo('plan_parse_dispatch_msg_node node initialized...')


    def genDispatchMsg(self, plan_as_list):
        '''
        Receives list of lists containing the plan, iterates over each element on the list
        constructing dispatch msgs
        '''
        complete_plan = CompletePlan()
        for ii, action in enumerate(plan_as_list):
            msg = ActionDispatch()
            msg.action_id = ii
            msg.name = action.pop(0)
            rospy.logdebug(action)
            for jj, parameter in enumerate(action):
                msg.parameters.append(KeyValue(str(jj + 1), parameter))
            complete_plan.plan_as_list.append(msg)
        self.pub_plan.publish(complete_plan)
        rospy.loginfo('Complete plan was published to ~plan topic')
        rospy.logdebug(str(complete_plan))
        

    def eventInCallBack(self, msg):
        '''
        To receive the request of parsing a plan
        '''
        self.event_in_msg = msg
        self.event_in_received = True


    def convert_plan_file_to_list(self):
        '''
        Convert a plan text file to a python list
        Add more parsers here if needed
        '''
        if self.plan_parser == 'IPC2014':
            return convert_IPC2014_plan_file_to_list(self.plan_path)
        else:
            rospy.logerr('Plan parser not supported, admissible values are : IPC2014')
            return None


    def parse_and_dispatch(self):
        '''
        parse a plan file convert into a ROS msg and publish
        '''
        # convert plan file to list
        plan_as_list = self.convert_plan_file_to_list()
        if plan_as_list == None:
            rospy.logerr('Error while parsing plan file, does it exist? : path -> ' + str(self.plan_path))
            return False
        self.genDispatchMsg(plan_as_list)
        return True


    def start_parse_dispatch(self):
        '''
        plan parse dispatch main loop
        '''
        while not rospy.is_shutdown():
            if self.event_in_received == True:
                rospy.loginfo('Generate dispatch msgs request received !')
                # lower flag
                self.event_in_received = False
                # analyze msg content
                if self.event_in_msg.data != 'e_start':
                    rospy.logerr('event not supported, admissible values are : e_start')
                    self.pub_event_out.publish(String('e_failure'))
                    continue
                if self.parse_and_dispatch():
                    self.pub_event_out.publish(String('e_success'))
                else:
                    self.pub_event_out.publish(String('e_failure'))
            self.loop_rate.sleep()


def main():
    rospy.init_node('plan_parse_dispatch_msg_node', anonymous=False)
    plan_parse_dispatch = PlanParseDispatch()
    plan_parse_dispatch.start_parse_dispatch()
