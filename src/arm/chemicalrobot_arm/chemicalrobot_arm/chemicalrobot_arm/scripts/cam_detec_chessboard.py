#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
import cv2
import math
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped,Pose,PoseStamped,PoseWithCovariance
# from aruco_msgs.msg import Marker,MarkerArray


rospy.init_node('opencv_pose_publisher')
br = tf2_ros.TransformBroadcaster()
t = TransformStamped()
t.header.frame_id = "camera_link"
t.child_frame_id = "marker_link"
t.header.stamp = rospy.Time.now()

Camera_intrinsic = np.array([[384.46809334, 0.0, 319.27157691], [0.0, 383.71464068, 243.85360831], [0.0, 0.0, 1]])
Camera_distortion = np.array([[-0.04565919, 0.02096258, -0.00106443, 0.00160538, 0.05729466]])

objp = np.zeros((5 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:5].T.reshape(-1, 2)  # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
objp = 27 * objp   # 打印棋盘格一格的边长为27mm
obj_points = objp   # 存储3D点
img_points = []     # 存储2D点

# 从摄像头获取视频图像
cap = cv2.VideoCapture(4)
# 设置分辨率为1280*720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

while not rospy.is_shutdown():
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, (8, 5), None)
    if ret:    # 画面中有棋盘格
        img_points = np.array(corners)
        cv2.drawChessboardCorners(frame, (8, 5), corners, ret)
        # rvec: 旋转向量 tvec: 平移向量
        _, rvec, tvec = cv2.solvePnP(obj_points, img_points, Camera_intrinsic, Camera_distortion)    # 解算位姿
        # distance = math.sqrt(tvec[0]**2+tvec[1]**2+tvec[2]**2)  # 计算距离
        rvec_matrix = cv2.Rodrigues(rvec)[0]    # 旋转向量->旋转矩阵
        
        proj_matrix = np.hstack((rvec_matrix, tvec))    # hstack: 水平合并
        # print(proj_matrix)
        eulerAngles = cv2.decomposeProjectionMatrix(proj_matrix)[6]  # 欧拉角
        pitch, yaw, roll = eulerAngles[0], eulerAngles[1], eulerAngles[2]
        
        # 测量得到的数据:
        t.transform.translation.x = tvec[0]
        t.transform.translation.y = tvec[1]
        t.transform.translation.z = tvec[2]
        # RPY转四元数
        q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        # tfBuffer = tf2_ros.Buffer()
        # tf2listener = tf2_ros.TransformListener(tfBuffer)
        # listener = rospy.Subscriber("markers", MarkerArray, callback)
        
        cv2.putText(frame, "dist: %.3fmm, yaw: %.3f, pitch: %.3f, roll: %.3f" % (distance, yaw, pitch, roll), (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == 27: # 按ESC键退出
            break
    else:   # 画面中没有棋盘格
        cv2.putText(frame, "Unable to Detect Chessboard", (20, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3) 
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == 27: # 按ESC键退出
            break
cv2.destroyAllWindows()


