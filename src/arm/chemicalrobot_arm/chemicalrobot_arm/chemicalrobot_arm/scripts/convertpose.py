#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf


def callback(camera_pose):
    listener = tf.TransformListener()
    base_pose = PoseStamped()
    base_pose = listener.transformPose('base_link', camera_pose)
    # base_pose = tfBuffer.transform(
    #     camera_pose, 'base_link', timeout=rospy.Duration(5))
    print(base_pose.pose.position)


if __name__ == '__main__':
    rospy.init_node('tf_listener')
    rospy.Subscriber("aruco_tracker/pose", PoseStamped, callback)
    rospy.spin()
