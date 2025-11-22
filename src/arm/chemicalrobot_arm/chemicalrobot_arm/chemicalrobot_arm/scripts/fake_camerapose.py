#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

pose = {
    #离心孔
    21:[],
    # 起始样品架 回收样品架
    31: [0.0519513, 0.072757, 0.344525, 0.00903074, 0.958919, 0.283527, -0.00174161],
    # 固体进样站
    32: [0.043543, 0.0524347, 0.285287, 0.00708047, 0.981061, 0.19353, 0.00376903],
    # 液体进样站
    33: [0.0316957,0.0132278,0.328192,0.405873,0.804829,0.399252,-0.167673],
    # 磁力搅拌站
    34: [-0.005480, 0.005845, 0.256296, 0.999614, -0.00294, 0.027027, 0.00573],
    # 吸液工作站
    35: [0.0358826, 0.0261339, 0.307057, 0.00982211, 0.979416, 0.199381, 0.0284861],
    # 封装工作站
    36: [0.03106, 0.0467866, 0.375338, 0.1608, 0.971009, 0.135368, 0.113842],
    # 烘干工作站
    37: [0.0256908, 0.0347757, 0.292075, 0.000378387, 0.967242, 0.253832, 0.00117185],
    # 离心工作站
    38: [0.0302286, 0.0471779, 0.278239, 0.0180617, 0.999411, 0.0220313, -0.0191411],
    # 光催化工作站
    39: [0.0581403,0.041354,0.290857,0.0177457,0.984336,0.175145,-0.00854007],
    # 滴液制样
    40: [0.0116146, 0.0373973, 0.316417, 0.0121663, 0.988678, 0.149499, 0.00309841],
    # 电催化工作站
    41: [0.0508806, 0.0639343, 0.352043, 0.0197691, 0.999779, -0.00191067, -0.00693704],
    # 荧光光谱工作站
    42:[],
    # 紫外光谱工作站
    43:[0.0367819,0.0542168,0.291506,-0.00965843,0.996997,0.0765719,-0.00434934],
    # 气相色谱工作站
    44: [0.00181012, 0.0394782, 0.361146, 0.0176989, 0.965792, 0.257809, 0.0215342],
    # LIBS工作站
    45: [0.0236582, 0.0913173, 0.391587, -0.00641119, 0.945763, 0.324134, 0.0206137],
    # LIBS样品架
    # 46: [-0.0242439, 0.0660424, 0.379687, -0.678972, 0.683317, 0.184846, 0.194692],
	# XRD工作站
	46:[0.0240136,0.0349034,0.292881,-0.00334781,0.952007,0.305974,0.00713825],
}


def callback(msg):
    camera_pose = PoseStamped()
    camera_pose.header.frame_id = "camera_color_frame"
    [
        camera_pose.pose.position.x,
        camera_pose.pose.position.y,
        camera_pose.pose.position.z,
    ] = pose[msg.data][0:3]
    [
        camera_pose.pose.orientation.x,
        camera_pose.pose.orientation.y,
        camera_pose.pose.orientation.z,
        camera_pose.pose.orientation.w,
    ] = pose[msg.data][3:]
    pub = rospy.Publisher("point_coordinate", PoseStamped, queue_size=1)
    rospy.sleep(1)
    pub.publish(camera_pose)
    print("published pose")


def listener():
    rospy.init_node("fake_camerapose", anonymous=True)
    rospy.Subscriber("camera_operation", Int32, callback)
    print("waiting")
    rospy.spin()


if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
