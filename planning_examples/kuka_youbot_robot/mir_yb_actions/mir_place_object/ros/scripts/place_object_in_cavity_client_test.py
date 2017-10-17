#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mir_yb_action_msgs.msg import PlaceObjectAction, PlaceObjectGoal

if __name__ == '__main__':
    rospy.init_node('place_object_client_tester')
    client = actionlib.SimpleActionClient('place_object_in_cavity_server', PlaceObjectAction)
    client.wait_for_server()
    goal = PlaceObjectGoal()
    if len(sys.argv) == 3: # 2 arguments were received : ok proceed
        try:
            goal.object = str(sys.argv[1])
            goal.location = str(sys.argv[2])
            timeout = 15.0
            rospy.loginfo('Sending action lib goal to place_object_server : ' + goal.object + ' ' + goal.location)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
            print client.get_result()
        except:
            pass
    else:
        rospy.logerr('Arguments were not received in the proper format !')
        rospy.loginfo('usage : place OBJECT_NAME LOCATION')
