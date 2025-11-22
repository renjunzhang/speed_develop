#!/usr/bin/env python  
# -*- coding: UTF-8 -*-  
import rospy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import math
import tf2_ros
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

v_ori = [0.7071067812,0.7071067812,0,0]
h_ori = [0.5,0.5,0.5,0.5]

def callback(camera_pose):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    base_pose = PoseStamped()
    base_pose = tfBuffer.transform(camera_pose, 'base_link')
    print(base_pose)

if __name__ == '__main__':
    rospy.init_node('tf2_listener')
    # robot=moveit_commander.RobotCommander()
    # group=moveit_commander.MoveGroupCommander('manipulator')
    # group.setplannerID('PTP')
    # listener = rospy.Subscriber("point_coordinate1", PoseStamped, callback)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    camera_pose = PoseStamped()
    base_pose = PoseStamped()
    current_pose = PoseStamped()
    while(not rospy.is_shutdown()):
        camera_pose = rospy.wait_for_message(
            "/aruco_tracker/pose", PoseStamped)
        base_pose = tfBuffer.transform(
            camera_pose, 'base_link', timeout=rospy.Duration(2))
        print(base_pose.pose.position)
        print('\n')
        # camera_pose.pose.orientation = tfBuffer.transform(current_pose, 'camera_link', timeout=rospy.Duration(1.0)).pose.orientation

        # if you want to the arm move to the target position,do not comment on the following code.
        # raw_input("press any key to move")
        # base_pose.pose.orientation.x = v_ori[0]
        # base_pose.pose.orientation.y = v_ori[1]
        # base_pose.pose.orientation.z = v_ori[2]
        # base_pose.pose.orientation.w = v_ori[3]
        # base_pose.pose.position.z += 0.4
        # group.set_pose_target(base_pose)
        # plan = group.plan()
        # group.execute(plan)