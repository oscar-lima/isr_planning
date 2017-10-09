#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import String
from file_changed import FileChanged

filename = os.getenv("HOME") + '/.ros/mercury.plan'

rospy.init_node('mercury_plan_success_analyzer')
event_out_pub = rospy.Publisher('~event_out', String, queue_size=2)

def analizeCallBack(data):
    rospy.loginfo('Request received for analyzing the success of mercury planner')
    msg = String()
    if FileChanged(filename):
        rospy.loginfo("mercury successfully created new plan (plan file has changed)")
        msg.data = 'e_success'
    else:
        rospy.logerr("mercury planner failed to produce a plan (time property of file " + filename + " did not change)")
        rospy.logwarn('is mercury planner running?, is problem.pddl + domain.pddl consistent?, is problem solvable at all?')
        msg.data = 'e_failure'
    rospy.loginfo('Publishing event out message')
    event_out_pub.publish(msg)
    
def main():
    rospy.loginfo('mercury plan success analyzer node initialized...')
    rospy.Subscriber("~event_in", String, analizeCallBack)
    rospy.spin()

if __name__ == '__main__':
    main()