#!/usr/bin/env python3
import json
import os
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import requests
import rosnode
import time

from aichem_msg_srv.srv import *

from communication_rs485.msg import platformInfo

import logging
import threading
import traceback
from datetime import datetime
from wrapped_socket import get_socket
from robot_mqtt_host import RobotMqttHost
from robot_flask_server import RobotFlaskSever
from robot_common_log import Log
from func_timeout import func_set_timeout
import rospkg
import socket
import sys

from pprint import pprint

def GetIP():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception as e:
        logger.info("获取ip失败")
        sys.exit()
    finally:
        s.close()
    return ip


class RobotMsgManager:
    def __init__(self):
        param_file_path = (
            rospkg.RosPack().get_path("robot_message_bridge") + "/config/param.json"
        )
        with open(param_file_path, encoding="utf-8") as f:
            self.config_param = json.load(f)

        unique_param_path = os.path.expanduser("~/speed_develop/config/params.json")
        with open(unique_param_path, encoding="utf-8") as f:
            self.unique_config_param = json.load(f)

        self.headers = {"content-type": "application/json"}  # DMS接收的header
        self.Navigation_topic_node_name = "/obsNavigation_out"
        self.Operation_topic_node_name = "/obsOperation_out"
        self.Navigation_serve_node_name = "/inquiryNavigation_out"
        self.Operation_serve_node_name = "/robot"

        self.node_list = {
            "/aruco_to_point_and_talk": "相机节点",
            "/sick_front/sick": "前雷达",
            "/sick_rear/sick": "后雷达",
            # '/hf_platform/joy_node',  # 手柄
            "/hf_platform/rs485_control": "底盘",
            "/map_server": "地图",
            "/platform_communication_node": "底盘通信节点",
            # '/chemicalrobot_arm': '机械臂',
            # '/ur5e', #机器臂的控制程
            # "/rq_sensor": "力矩传感器",
            "/chemicalrobot_arm_new": "机械臂",
            # '/pgitest' #夹爪
        }

        self.operate_code_id = 123

        self.navi_topic_feedback_msg = dict()
        self.oper_topic_feedback_msg = dict()
        self.navi_serve_feedback_msg = dict()
        self.oper_serve_feedback_msg = dict()
        self.DMS_msg = dict()

        self.robot_heartbeat_msg = {
            "stamp": "",
            "status": "",
            "detailMsg": "",
            "currentStation": "",
            "electricityQuantity": 80.0,
            "machineCode": self.unique_config_param["robotCode"],  # 机器人id
            "detail": None,  # 机器人不用，为固定值
            "ip": GetIP(),
            "port": 3030,
            "uri": "/",
        }

        self.robot_msg = {
            "id": "",
            "stamp": "",
            "status": "",
            "current_workstation": "",
            "state": "",
            "detail": {"navi": "", "oper": ""},
        }
        self.query_msg = {"stamp": "", "query": "true"}

        self.navi_msg = {"id": "", "stamp": "", "destination": "", "action": "move"}
        self.oper_msg = {"id": "", "stamp": "", "destination": "", "operations": ""}

        self.oper_reset_msg = {
            "id": "",
            "destination": "",
            "operations": [{"operation": "reset"}],
        }
        self.oper_relocation_msg = {
            "id": "",
            "destination": "",
            "operations": [{"operation": "relocation"}],
        }
        # self.oper_reset_msg = {
        #     "id": '', "destination": "", "operation": "reset"}
        # self.oper_relocation_msg = {
        #     "id": '', "destination": "", "operation": "relocation"}
        self.now_workstation = "charge_station"
        self.libs_fire_flag = False
        self.heartbeat_status = "init"

        self.station_name_mutex = threading.Lock()
        self.callback_fun_mutex = threading.Lock()

        self.InitMessage()

    def InitMessage(self):

        self.dms_url = self.config_param["dms_url"]  # DMS接收的url
        self.robot_flask_sever = RobotFlaskSever(
            self.dms_callback,
            self.NavLockServer,
            self.GetStatusAlarmMsg,
            3030,
            False,
            "0.0.0.0",
        )

        self.mqtt = RobotMqttHost(
            ip=self.config_param["mqtt_ip"],
            port=self.config_param["mqtt_port"],
            client_name=self.unique_config_param["robotCode"],
        )

        self.voide_broadcast_pub = rospy.Publisher(
            "/voide_broadcast_msg", String, queue_size=10
        )
        self.navi_pub = rospy.Publisher("/obsNavigation_in", String, queue_size=10)
        self.oper_pub = rospy.Publisher("/obsOperation_in", String, queue_size=10)
        self.nav_lock_pub = rospy.Publisher(
            "/hf_platform/pause_navigation", Bool, queue_size=1
        )

        self.pub_nav_vel = rospy.Publisher("/hf_platform/joy_vel", Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber(
            "/hf_platform/joy", Joy, self.ChageStationName, queue_size=10
        )

    def PubDigitalTwinMqtt(self):
        data = {"state": "done"}
        self.digital_twin_mqtt.MqttPublish(
            "digital_twin_robot_state", json.dumps(data), 1
        )

    def ChageStationName(self, joy_msg):
        if (
            abs(joy_msg.axes[0] + joy_msg.axes[1] + joy_msg.axes[2]) > 1e-4
        ):  # 0x速度，1y速度，2角速度
            # logger.info("手柄操作，改变站点")
            with self.station_name_mutex:
                self.now_workstation = "charge_station"

    def OperAction(self, oper_msg, generate_msg_flag):
        self.oper_pub.publish(String(json.dumps(oper_msg)))

        self.oper_topic_feedback_msg = json.loads(
            rospy.wait_for_message(self.Operation_topic_node_name, String).data
        )

        if generate_msg_flag:
            if "interrupt" == self.oper_topic_feedback_msg["state"]:
                logger.info("interrupt return")
                self.operate_code_id = 123
                return
            self.robot_msg_gen(
                self.navi_topic_feedback_msg, self.oper_topic_feedback_msg
            )
            self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)

    def TryRelocation(self, relocation_number=0):

        while "relocation_error" == self.oper_topic_feedback_msg["state"]:
            if relocation_number > 10:
                return
            # 机械臂复位
            self.oper_reset_msg["destination"] = self.now_workstation
            self.oper_pub.publish(String(json.dumps(self.oper_reset_msg)))
            relocation_oper_topic_feedback_msg = json.loads(
                rospy.wait_for_message(self.Operation_topic_node_name, String).data
            )
            # 底盘移动
            if relocation_oper_topic_feedback_msg["state"] == "done":
                self.navi_pub.publish(String(json.dumps(self.navi_msg)))
                relocation_navi_topic_feedback_msg = json.loads(
                    rospy.wait_for_message(self.Navigation_topic_node_name, String).data
                )
            # 机械臂复位
            else:
                return
            if relocation_navi_topic_feedback_msg["state"] == "done":
                with self.station_name_mutex:
                    self.now_workstation = self.DMS_msg["destination"]

                self.oper_reset_msg["destination"] = self.now_workstation
                self.oper_pub.publish(String(json.dumps(self.oper_reset_msg)))
                relocation_oper_topic_feedback_msg = json.loads(
                    rospy.wait_for_message(self.Operation_topic_node_name, String).data
                )
            else:
                return
            if relocation_oper_topic_feedback_msg["state"] == "done":
                self.oper_relocation_msg["destination"] = self.DMS_msg["destination"]
                self.oper_pub.publish(String(json.dumps(self.oper_relocation_msg)))
                self.oper_topic_feedback_msg = json.loads(
                    rospy.wait_for_message(self.Operation_topic_node_name, String).data
                )
                relocation_number += 1
            else:
                return

    def NavLockServer(self, data):
        if "lock" == data["nav"]:
            logger.info("底盘锁定")
            self.nav_lock_pub.publish(Bool(True))
            vel_msg = Twist()
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            for i in range(10):
                self.pub_nav_vel.publish(vel_msg)
        elif "unlock" == data["nav"]:
            logger.info("底盘解锁")
            self.nav_lock_pub.publish(Bool(False))

    def dms_callback(self, operate_datas):
        with self.callback_fun_mutex:
            time.sleep(1)  # 留给状态改变的时间
            self.DMS_msg = operate_datas
            if self.operate_code_id == self.DMS_msg["id"]:
                return
            else:
                self.operate_code_id = self.DMS_msg["id"]
            if "query" in self.DMS_msg:
                logger.info("DMS问询")
                self.navi_serve_callback()
                self.oper_serve_callback()
                self.robot_msg_gen(
                    self.navi_serve_feedback_msg, self.oper_serve_feedback_msg
                )

                self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                logger.info("成功答复")
                return

            self.oper_msg_gen()
            logger.info("开始操作指令：")
            logger.info(self.DMS_msg)

            try:
                # 动作前解除充电
                tmp_operation = self.DMS_msg["operations"][0]["operation"]
                tmp_platform_status = self.navi_serve_callback(return_flage=1)

                if "charging" == tmp_platform_status["state"]:
                    if "uncharge" != tmp_operation and "charge" != tmp_operation:
                        logger.info("解除充电")
                        self.navi_msg["action"] = "uncharge"
                        self.navi_msg["id"] = self.DMS_msg["id"]
                        self.navi_msg["stamp"] = self.DMS_msg["stamp"]
                        self.navi_msg["destination"] = "charge_station"
                        self.navi_pub.publish(String(json.dumps(self.navi_msg)))
                        self.navi_topic_feedback_msg = json.loads(
                            rospy.wait_for_message(
                                self.Navigation_topic_node_name, String
                            ).data
                        )

                        if "done" != self.navi_topic_feedback_msg["state"]:
                            logger.info("解除充电异常")
                            self.robot_msg_gen(
                                self.navi_topic_feedback_msg,
                                self.oper_topic_feedback_msg,
                            )
                            self.mqtt.MqttPublish(
                                "robot-to-dms", json.dumps(self.robot_msg), 1
                            )
                        logger.info("解除充电成功")

                # 充电
                if (
                    "charge" == self.DMS_msg["operations"][0]["operation"]
                    or "uncharge" == self.DMS_msg["operations"][0]["operation"]
                ):

                    with self.station_name_mutex:
                        self.now_workstation = self.DMS_msg["destination"]

                    self.navi_msg["id"] = self.DMS_msg["id"]

                    self.oper_reset_msg["destination"] = self.now_workstation
                    self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                    if self.oper_topic_feedback_msg["state"] == "done":

                        self.navi_msg["action"] = self.DMS_msg["operations"][0][
                            "operation"
                        ]
                        self.navi_msg["id"] = self.DMS_msg["id"]
                        self.navi_msg["stamp"] = self.DMS_msg["stamp"]
                        self.navi_msg["destination"] = self.DMS_msg["destination"]
                        self.navi_pub.publish(String(json.dumps(self.navi_msg)))
                        localtime = time.asctime(time.localtime(time.time()))
                        print(
                            "@本地时间:",
                            localtime,
                            " 移动至站点:",
                            self.DMS_msg["destination"],
                        )
                        self.navi_topic_feedback_msg = json.loads(
                            rospy.wait_for_message(
                                self.Navigation_topic_node_name, String
                            ).data
                        )

                        self.robot_msg_gen(
                            self.navi_topic_feedback_msg, self.oper_topic_feedback_msg
                        )
                        self.mqtt.MqttPublish(
                            "robot-to-dms", json.dumps(self.robot_msg), 1
                        )

                    else:
                        self.navi_serve_callback()
                        self.robot_msg_gen(
                            self.navi_serve_feedback_msg, self.oper_topic_feedback_msg
                        )
                        self.mqtt.MqttPublish(
                            "robot-to-dms", json.dumps(self.robot_msg), 1
                        )
                    return
                
                # SPEED ADD
                self.now_workstation = self.navi_serve_callback_only_station()
                print("机器人当前站点：" + self.now_workstation)
                
                if (
                    self.DMS_msg["destination"] == self.now_workstation
                ):  # 如果当前站点就是所需工作站点,继续执行动作
                    if "check" == self.DMS_msg["operations"][0]["operation"]:
                        self.oper_topic_feedback_msg["id"] = self.DMS_msg["id"]
                        self.oper_topic_feedback_msg["state"] = "done"
                        self.robot_msg_gen(
                            self.navi_topic_feedback_msg, self.oper_topic_feedback_msg
                        )
                        self.mqtt.MqttPublish(
                            "robot-to-dms", json.dumps(self.robot_msg), 1
                        )
                        return
                    self.OperAction(self.oper_msg, generate_msg_flag=True)

                else:  # 如果当前站点不是所需站点，需要移动至所需工作站再进行操作,先复位
                    self.oper_reset_msg["destination"] = self.now_workstation
                    self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                    # 机械臂复位成功后底盘移动
                    if self.oper_topic_feedback_msg["state"] == "done":

                        self.navi_msg["id"] = self.DMS_msg["id"]
                        self.navi_msg["action"] = "move"
                        # self.navi_msg["expr_no"] = self.DMS_msg["expr_no"]
                        self.navi_msg["stamp"] = self.DMS_msg["stamp"]
                        self.navi_msg["destination"] = self.DMS_msg["destination"]
                        self.navi_pub.publish(String(json.dumps(self.navi_msg)))
                        localtime = time.asctime(time.localtime(time.time()))
                        print(
                            "@本地时间:",
                            localtime,
                            " 移动至站点:",
                            self.DMS_msg["destination"],
                        )
                        self.navi_topic_feedback_msg = json.loads(
                            rospy.wait_for_message(
                                self.Navigation_topic_node_name, String
                            ).data
                        )
                    elif "interrupt" == self.oper_topic_feedback_msg["state"]:
                        logger.info("interrupt return")
                        self.operate_code_id = 123
                        return

                    else:
                        self.navi_serve_callback()
                        self.robot_msg_gen(
                            self.navi_serve_feedback_msg, self.oper_topic_feedback_msg
                        )
                        # self.PubDigitalTwinMqtt()#发布数字孪生的
                        self.mqtt.MqttPublish(
                            "robot-to-dms", json.dumps(self.robot_msg), 1
                        )
                        return

                    # 底盘移动成功后，动作前机械臂复位
                    if self.navi_topic_feedback_msg["state"] == "done":
                        time.sleep(3)  # 空余点时间给底盘计算使用
                        with self.station_name_mutex:
                            self.now_workstation = self.DMS_msg["destination"]
                        self.oper_reset_msg["destination"] = self.now_workstation
                        logger.info("机械臂复位指令%r", self.oper_reset_msg)
                        self.OperAction(self.oper_reset_msg, generate_msg_flag=False)
                    else:
                        self.oper_serve_callback()
                        self.robot_msg_gen(
                            self.navi_topic_feedback_msg, self.oper_serve_feedback_msg
                        )
                        self.mqtt.MqttPublish(
                            "robot-to-dms", json.dumps(self.robot_msg), 1
                        )
                        return

                    # 机械臂复位成功后机械臂重定位
                    if self.oper_topic_feedback_msg["state"] == "done":
                        self.oper_relocation_msg["destination"] = self.DMS_msg[
                            "destination"
                        ]

                        self.OperAction(
                            self.oper_relocation_msg, generate_msg_flag=False
                        )

                        # 重定位失败就再进行底盘移动与重定位
                        self.TryRelocation(relocation_number=0)
                    elif "interrupt" == self.oper_topic_feedback_msg["state"]:
                        logger.info("interrupt return")
                        self.operate_code_id = 123
                        return
                    else:
                        self.navi_serve_callback()
                        self.robot_msg_gen(
                            self.navi_serve_feedback_msg, self.oper_topic_feedback_msg
                        )
                        self.mqtt.MqttPublish(
                            "robot-to-dms", json.dumps(self.robot_msg), 1
                        )
                        return

                    if self.oper_topic_feedback_msg["state"] == "done":
                        with self.station_name_mutex:
                            self.now_workstation = self.DMS_msg["destination"]
                        # 为烘干箱等设备开门做准备
                        if "check" == self.DMS_msg["operations"][0]["operation"]:
                            self.oper_topic_feedback_msg["id"] = self.DMS_msg["id"]
                            self.oper_topic_feedback_msg["state"] = "done"
                            self.robot_msg_gen(
                                self.navi_topic_feedback_msg,
                                self.oper_topic_feedback_msg,
                            )
                            self.mqtt.MqttPublish(
                                "robot-to-dms", json.dumps(self.robot_msg), 1
                            )
                            return

                        self.OperAction(self.oper_msg, generate_msg_flag=True)
                    elif "interrupt" == self.oper_topic_feedback_msg["state"]:
                        logger.info("interrupt return")
                        self.operate_code_id = 123
                        return
                    else:
                        self.navi_serve_callback()
                        self.robot_msg_gen(
                            self.navi_serve_feedback_msg, self.oper_topic_feedback_msg
                        )
                        self.mqtt.MqttPublish(
                            "robot-to-dms", json.dumps(self.robot_msg), 1
                        )

            except rospy.ROSInterruptException:
                logger.info("ROSInterruptException")
                pass

    def oper_msg_gen(self):
        self.oper_msg["id"] = self.DMS_msg["id"]
        # self.oper_msg["expr_no"] = self.DMS_msg["expr_no"]
        self.oper_msg["stamp"] = self.DMS_msg["stamp"]
        self.oper_msg["destination"] = self.DMS_msg["destination"]
        self.oper_msg["operations"] = self.DMS_msg["operations"]
        self.oper_reset_msg["id"] = self.DMS_msg["id"]
        self.oper_relocation_msg["id"] = self.DMS_msg["id"]

    # def navi_callback(self, data):  # 处理navi通过话题的反馈，话题只会反馈done和error
    #     raw_info = data.data
    #     self.navi_topic_feedback_msg = json.loads(raw_info)
    #     signal = self.navi_topic_feedback_msg['state']
    #     if signal == 'done':
    #         localtime = time.asctime(time.localtime(time.time()))
    #         print("@本地时间:", localtime, '已经达到站点')
    #     else:
    #         logger.info("底盘错误，details:")
    #         logger.info(self.navi_topic_feedback_msg['detail'])

    def SendHreartbeatMsg(self):

        with self.station_name_mutex:
            self.robot_heartbeat_msg["currentStation"] = self.now_workstation

        platform_info_datas = rospy.wait_for_message(
            "/hf_platform/platform_info", platformInfo
        )

        self.robot_heartbeat_msg["electricityQuantity"] = (
            platform_info_datas.batteryPower
        )
        self.robot_heartbeat_msg["stamp"] = round(time.time() * 1000)

        data = json.dumps(self.robot_heartbeat_msg)

        if self.heartbeat_status != self.robot_heartbeat_msg["status"]:
            self.heartbeat_status = self.robot_heartbeat_msg["status"]
            logger.info("当前机器人状态变更为:%r", self.heartbeat_status)
        try:
            response = requests.post(
                self.dms_url,
                data.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15),
            )
        # logger.info("DMS回复:")
        # logger.info(response.text)
        except Exception as e:
            logger.info(e)

    def navi_serve_callback(self, return_flage=0):
        client = rospy.ServiceProxy(self.Navigation_serve_node_name, DmsService)
        client.wait_for_service()
        req = DmsServiceRequest()
        req.data = ""  # self.DMS_msg
        resp = client.call(req.data)
        raw_info = resp.data
        if return_flage:
            return json.loads(raw_info)
        self.navi_serve_feedback_msg = json.loads(raw_info)
    
    def navi_serve_callback_only_station(self, return_flage=0):
        client = rospy.ServiceProxy(self.Navigation_serve_node_name, DmsService)
        client.wait_for_service()
        req = DmsServiceRequest()
        req.data = ""  # self.DMS_msg
        resp = client.call(req.data)
        raw_info = resp.data
        if return_flage:
            return json.loads(raw_info)
        msg = json.loads(raw_info)
        return msg["cur_station"]

    def oper_serve_callback(self):
        client = rospy.ServiceProxy(self.Operation_serve_node_name, DmsService)
        client.wait_for_service()
        req = DmsServiceRequest()
        req.data = ""  # self.DMS_msg
        resp = client.call(req.data)
        raw_info = resp.data
        # print(raw_info)
        self.oper_serve_feedback_msg = json.loads(raw_info)

    # def oper_callback(self, data):  # 处理oper通过话题的反馈，话题只会反馈done和error
    #     raw_info = data.data
    #     self.oper_topic_feedback_msg = json.loads(raw_info)
    #     signal = self.oper_topic_feedback_msg['state']
    #
    #     if signal == 'done':
    #         localtime = time.asctime(time.localtime(time.time()))
    #         print("@本地时间:", localtime, '已经完成操作')
    #     else:
    #         logger.info("机械臂错误，details:")
    #         logger.info(self.oper_topic_feedback_msg['detail'])

    def robot_msg_gen(self, navi, oper):

        self.robot_msg["current_workstation"] = self.now_workstation
        if navi["state"] in ["running", "error"]:
            self.robot_msg["id"] = navi["id"]
            # self.robot_msg["exper_no"] = navi["exper_no"]
            self.robot_msg["stamp"] = str(int(round(time.time() * 1000)))
            self.robot_msg["state"] = navi["state"]
        else:
            self.robot_msg["id"] = oper["id"]
            # self.robot_msg["exper_no"] = oper["exper_no"]
            self.robot_msg["stamp"] = str(int(round(time.time() * 1000)))
            self.robot_msg["state"] = oper["state"]
            print(datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"), oper["id"])
        if "detail" in navi:
            self.robot_msg["detail"]["navi"] = navi["detail"]
        else:
            pass

        if "detail" in oper:
            self.robot_msg["detail"]["oper"] = oper["detail"]
        else:
            pass

        if "error" == navi["state"]:
            self.voide_broadcast_pub.publish(
                String(navi["detail"].encode("utf-8").decode("utf-8"))
            )
        if "error" == oper["state"]:
            self.voide_broadcast_pub.publish(
                String(oper["detail"].encode("utf-8").decode("utf-8"))
            )

    def GetStatusAlarmMsg(self):

        return self.robot_heartbeat_msg

    def heartbeat_monitoring(self):  # 5秒一次的心跳监听
        while True:
            pinged, unpinged = rosnode.rosnode_ping_all()
            if unpinged:
                master = rosnode.rosgraph.Master(rosnode.ID)
                rosnode.cleanup_master_blacklist(master, unpinged)

            now_node_list = rosnode.get_node_names()

            if self.node_list.keys() <= set(now_node_list):
                self.query_msg["stamp"] = datetime.now().strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )
                self.navi_serve_callback()
                self.oper_serve_callback()

                robot_details = ""
                platform_status = self.navi_serve_feedback_msg["state"]
                arm_status = self.oper_serve_feedback_msg["state"]

                # platform_detail=self.navi_serve_feedback_msg["detail"]
                # arm_detail=self.oper_serve_feedback_msg["detail"]
                # if platform_detail!='':
                #     robot_details+="platform"+platform_detail+";"
                # if arm_detail!='':
                #     robot_details+="arm"+arm_detail+";"

                if platform_status == "error" or arm_status == "error":
                    self.robot_heartbeat_msg["status"] = "ERROR"
                elif platform_status == "running" or arm_status == "running":
                    self.robot_heartbeat_msg["status"] = "BUSY"
                elif platform_status == "done" or arm_status == "done":
                    self.robot_heartbeat_msg["status"] = "BUSY"
                elif platform_status == "charging":
                    self.robot_heartbeat_msg["status"] = "CHARGING"
                elif self.callback_fun_mutex.locked():
                    self.robot_heartbeat_msg["status"] = "BUSY"
                else:
                    self.robot_heartbeat_msg["status"] = "IDLE"

                self.robot_heartbeat_msg["detailMsg"] = robot_details
                self.SendHreartbeatMsg()

            else:
                inactive_node = ""
                for i in self.node_list.keys():
                    if i in now_node_list:
                        pass
                    else:
                        inactive_node += self.node_list[i]
                        logger.info(i + "节点异常")
                        self.voide_broadcast_pub.publish(String(i + "节点异常"))
                self.robot_heartbeat_msg["status"] = "ERROR"
                self.robot_heartbeat_msg["detailMsg"] = inactive_node + "节点异常"
                self.SendHreartbeatMsg()

            time.sleep(3)

    def DmsFlaskHttpSever(self):
        self.robot_flask_sever.RunSever()


if __name__ == "__main__":

    logger = Log(__name__).getlog()

    try:
        rospy.init_node("robot_msg_manager_node", anonymous=True, disable_signals=True)
        manager = RobotMsgManager()
        logger.info("initial node")

        threading.Thread(target=manager.heartbeat_monitoring).start()
        threading.Thread(target=manager.DmsFlaskHttpSever).start()

        rospy.spin()

    except:
        logger.critical(str(traceback.format_exc()))
