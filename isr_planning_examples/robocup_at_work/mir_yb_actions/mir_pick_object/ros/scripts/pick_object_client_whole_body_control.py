#! /usr/bin/env python
import rospy
import roslib
import actionlib
import sys

from mir_yb_action_msgs.msg import PickObjectWBCAction
from mir_yb_action_msgs.msg import PickObjectWBCGoal

if __name__ == '__main__':
    rospy.init_node('pick_object_client_tester')
    client = actionlib.SimpleActionClient('wbc_pick_object_server', PickObjectWBCAction)
    client.wait_for_server()
    goal = PickObjectWBCGoal()
    if len(sys.argv) == 2: # 1 arguments was received : ok proceed
        try:
            goal.object = str(sys.argv[1])
            timeout = 15.0
            rospy.loginfo('Sending action lib goal to pick_object_server : ' + goal.object)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
            print client.get_result()
        except:
            pass
    else:
        rospy.logerr('Arguments were not received in the proper format !')
        rospy.loginfo('usage : pick OBJECT_NAME')
