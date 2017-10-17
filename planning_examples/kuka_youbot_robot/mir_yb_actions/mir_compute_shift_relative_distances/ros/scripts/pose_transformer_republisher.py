#!/usr/bin/env python
'''
   This component transforms a give pose into specified target frame and
   republishes the output pose without orientation and without x
'''
#-*- encoding: utf-8 -*-
__author__ = 'shehzad'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf

from geometry_msgs.msg import PoseStamped

class PoseTransformer(object):
    def __init__(self):
        # params
        self.monitor_event = None
        self.pose_in = None
        self.transformed_pose = None
        self.listener = tf.TransformListener()

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # target frame to transform pose
        self.target_frame = rospy.get_param('~target_frame', 'odom')

        #maximum distance to shift is 20 cm, otherwise we risk collision with side walls (if any)
        self.max_shift_distance = rospy.get_param('~max_shift_distance', 0.2)

        # publishers
        self.transform_pose_pub = rospy.Publisher(
            '~transformed_pose',  geometry_msgs.msg.PoseStamped, queue_size=1
        )
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=1)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~pose_in', geometry_msgs.msg.PoseStamped, self.pose_cb)

    def start(self):
        """
        Starts the state machine

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()
            elif state == 'PROCESSING':
                state = self.processing_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event

        """
        self.monitor_event = msg.data

    def pose_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.pose_in = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.pose_in:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            self.transformed_pose = self.transform_pose(self.pose_in, self.target_frame)
            if self.transformed_pose:
                self.event_out.publish('e_success')
                return 'PROCESSING'
            else:
                self.event_out.publish('e_failure')
                self.reset_component_data()
                return 'INIT'

    def processing_state(self):
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            if self.transformed_pose:
                modified_pose = PoseStamped()
                modified_pose.header = self.transformed_pose.header
                modified_pose.pose.position.x = 0.0
		# clamp to maximum shift value not to collide with possible side walls
		if self.transformed_pose.pose.position.y > self.max_shift_distance:
		    self.transformed_pose.pose.position.y = self.max_shift_distance
		elif self.transformed_pose.pose.position.y < -self.max_shift_distance:
		    self.transformed_pose.pose.position.y = -self.max_shift_distance
                modified_pose.pose.position.y = self.transformed_pose.pose.position.y
                modified_pose.pose.position.z = 0.0
                modified_pose.pose.orientation.x = 0.0
                modified_pose.pose.orientation.y = 0.0
                modified_pose.pose.orientation.z = 0.0
                modified_pose.pose.orientation.w = 1.0
                self.transform_pose_pub.publish(modified_pose)
                self.pose_in = None
                return 'IDLE'
            else:
                return 'INIT'

            return 'PROCESSING'

    def transform_pose(self, reference_pose, target_frame):
        """
        Transforms a given pose into the target frame.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_frame: The name of the taget frame.
        :type target_frame: String

        :return: The pose in the target frame.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            common_time = self.listener.getLatestCommonTime(
                target_frame, reference_pose.header.frame_id
            )

            self.listener.waitForTransform(
                target_frame, reference_pose.header.frame_id,
                common_time, rospy.Duration(self.wait_for_transform)
            )
            reference_pose.header.stamp = common_time

            transformed_pose = self.listener.transformPose(
                target_frame, reference_pose,
            )

            return transformed_pose

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.monitor_event = None
        self.pose_in = None
        self.transformed_pose = None


def main():
    rospy.init_node('pose_transformer_node', anonymous=True)
    pose_transformer = PoseTransformer()
    pose_transformer.start()

if __name__ == '__main__':
    main()
