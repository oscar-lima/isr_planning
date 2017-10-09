#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ipc_2014_plan_parser import ParseMercuryOutput
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_dispatch_msgs.msg import CompletePlan
from diagnostic_msgs.msg import KeyValue

'''
reads plan from ~/.ros/mercury.plan , parses it and generates
dispatch msgs that are published in /plan topic
'''

rospy.init_node('mercury_dispatch_msg_generator')
pub = rospy.Publisher('~plan', CompletePlan, queue_size=2)

def genDispatchMsg(plan):
    """
    Receives list of lists containing the plan, iterates over each element on the list
    constructs dispatch msg out of this list
    """
    complete_plan = CompletePlan()
    for ii, action in enumerate(plan):
        msg = ActionDispatch()
        msg.action_id = ii
        msg.name = action.pop(0)
        #print action
        for jj, parameter in enumerate(action):
            msg.parameters.append(KeyValue(str(jj + 1), parameter))
        #pub.publish(msg)
        complete_plan.plan.append(msg)
    #publish complete plan
    pub.publish(complete_plan)
    rospy.loginfo('Complete plan was published to ~plan topic')
    rospy.logdebug(str(complete_plan))
    

def genDispatchMsgCallBack(data):
    rospy.loginfo('Generate dispatch msgs request received !')
    dispatch_msg = genDispatchMsg(ParseMercuryOutput())
    rospy.loginfo('Done with dispatch generation... ready for new request')
    
    
def main():
    rospy.Subscriber("~event_in", String, genDispatchMsgCallBack)
    rospy.loginfo('mercury dispatch msg generator node initialized...')
    rospy.spin()


if __name__ == '__main__':
    main()
