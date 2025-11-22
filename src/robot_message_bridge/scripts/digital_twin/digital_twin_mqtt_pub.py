#!/usr/bin/env python3

import time
import paho.mqtt.client as mqtt
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import threading


def on_connect(client, userdata, flags, rc):
    print("Connected with result code: " + str(rc))


joints_ros_topic_name = "/robot_joints"
pose_ros_topic_name = "/amcl_pose_tf"
gripped_status_ros_topic_name = "/gripped_status"
gripped_target_ros_topic_name = "/gripped_target"
robot_client_name = "digital_twin_robot"
gripped_status_mqtt_topic_name = "gripped_status"
robot_pose_mqtt_topic_name = "robot_status"

client = mqtt.Client(robot_client_name)  # 客户端名称digital_twin_robot
client.on_connect = on_connect
client.connect(host="192.168.1.229", port=1883, keepalive=60)
client.loop_start()
f = open("./data.txt", "w")
mutex = threading.Lock()


def PubPoseDatas():
    """{ "joints" : "-160.17190,-58.88362,127.21296,-68.65324,20.05318,-179.55076"
    ,  "time_stamp" : "1693292431945"}"""
    while not rospy.is_shutdown():
        try:
            joints = json.loads(rospy.wait_for_message(
                joints_ros_topic_name, String).data)
            pose_msg = rospy.wait_for_message(
                pose_ros_topic_name, PoseWithCovarianceStamped)

            x = pose_msg.pose.pose.position.x
            y = pose_msg.pose.pose.position.y
            # z=pose_msg.pose.position.z
            Qx = pose_msg.pose.pose.orientation.x
            Qy = pose_msg.pose.pose.orientation.y
            Qz = pose_msg.pose.pose.orientation.z
            Qw = pose_msg.pose.pose.orientation.w
            theta = math.atan2(2 * (Qw * Qz + Qx * Qy),
                               1 - 2 * (Qy ** 2 + Qz ** 2))

            datas = {"time_stamp": str(int(time.time()*1000)),
                     "platform_pose": str(x)+","+str(y)+","+str(theta),
                     "joints": joints["joints"]}

            client.publish(topic=robot_pose_mqtt_topic_name,
                           payload=json.dumps(datas), qos=0)
            # print("robot_pose_datas%r" % datas)
            # with mutex:
            #     f.write(json.dumps(datas)+'\r')
            #     f.flush()
        except Exception as e:
            print(e)
        time.sleep(0.1)
    # f.close()


def PubGrippedStatus():
    while not rospy.is_shutdown():
        gripped_target_datas = json.loads(rospy.wait_for_message(
            gripped_target_ros_topic_name, String).data)

        gripped_status_datas = json.loads(rospy.wait_for_message(
            gripped_status_ros_topic_name, String).data)
        datas = {"time_stamp": str(int(time.time()*1000)),
                 "gripped_status": gripped_status_datas["gripped_status"],
                 "target": gripped_target_datas["gripped_target"]}

        try:
            client.publish(topic=gripped_status_mqtt_topic_name,
                           payload=json.dumps(datas), qos=0)
            print("gripped_status_datas%r" % datas)
            # with mutex:
            #     f.write(json.dumps(datas)+'\r')
            #     f.flush()
        except Exception as e:
            print(e)
    # f.close()


# def on_pose_aquired(pose):

 #   pass


if __name__ == '__main__':
    rospy.init_node('digital_twin_robot_node',
                    anonymous=True, disable_signals=True)

    threading.Thread(target=PubGrippedStatus).start()
    PubPoseDatas()
