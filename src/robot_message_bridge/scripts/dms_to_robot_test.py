#!/usr/bin/env python3
"""
Author: LightInDust bariko@mail.ustc.edu.cn
Date: 2024-08-11 17:57:51
LastEditors: Lightindust VayWei@mail.ustc.edu.cn
LastEditTime: 2024-08-23 20:41:22
FilePath: \chem_newbase\src\robot_message_bridge\scripts\dms_to_robot.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
"""
import json
import os
import time
import requests
import logging
import threading
import traceback
import socket
import sys
from datetime import datetime
from func_timeout import func_set_timeout
from pprint import pprint

import rosnode
import rospy
import rospkg
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from aichem_msg_srv.srv import *

from wrapped_socket import get_socket
from robot_mqtt_host import RobotMqttHost
from robot_flask_server import RobotFlaskSever
from robot_common_log import Log
from agv_tasklist_manager import AgvTaskListManager
from hik_flask_server import HikFlaskServer

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


# def GetIP():
#     try:
#         s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         s.connect(("10.255.255.255", 1))
#         ip = s.getsockname()[0]
#     except Exception as e:
#         print("获取 IP 失败，使用默认 IP:", e)
#         ip = "127.0.0.1"  # 回退到本地回环地址
#     finally:
#         s.close()
#     return ip



class RobotMsgManager:
    def __init__(self):

        self.hik_system = rospy.get_param('~hiksystem', 'rcs_2000')

        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/param.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.config_param = json.load(f)

        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/station_mapping.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.station_mapping = json.load(f)

        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/status.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.agv_status = json.load(f)

        self.headers = {"content-type": "application/json"}  # DMS接收的header
        self.Operation_topic_node_name = "/obsOperation_out"
        self.Operation_serve_node_name = "/robot"

        self.node_list = {
            # "/aruco_to_point_and_talk": "相机节点",
            # "/chemicalrobot_arm_new": "机械臂"
            # "/sick_front/sick": "前雷达",
            # "/sick_rear/sick": "后雷达",
            # '/hf_platform/joy_node',  # 手柄
            # "/hf_platform/rs485_control": "底盘",
            # "/map_server": "地图",
            # "/platform_communication_node": "底盘通信节点",
            # '/chemicalrobot_arm': '机械臂',
            # '/ur5e', #机器臂的控制程
            # "/rq_sensor": "力矩传感器",
            # '/pgitest' #夹爪
        }

        self.AGV_url = self.config_param["AGV_url"]
        self.operate_code_id = self.config_param["operate_code_id"]

        self.oper_topic_feedback_msg = dict()
        self.navi_task_feedback_msg = dict()
        self.navi_serve_feedback_msg = dict()
        self.oper_serve_feedback_msg = dict()
        self.DMS_msg = dict()

        self.taskCode = ""

        self.robot_heartbeat_msg = {
            "stamp": "",
            "status": "",
            "detailMsg": "",
            "currentStation": "",
            "battery": 80.0,    # 电量
            "machineCode": self.config_param["robotCode"],  # 机器人id
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

        # base lock
        self.baselock_msg = {"agv_id": self.config_param["AGV_name"], "status": "OK"}

        self.now_workstation = "charge_station"
        self.libs_fire_flag = False
        self.heartbeat_status = "init"

        self.station_name_mutex = threading.Lock()
        self.callback_fun_mutex = threading.Lock()

        self.InitCommunication()

    def InitCommunication(self):

        self.dms_url = self.config_param["dms_url"]  # DMS接收信息的url
        self.robot_flask_sever = RobotFlaskSever(
            self.DMS_callback,
            self.NavLock_callback,
            self.GetStatusAlarmMsg,
            self.GetBaseLockMsg,
            port=3030,
            debug=False,
            host="0.0.0.0",
        )

        self.hik_flask_sever = HikFlaskServer(
            port=8801,
            debug=False,
            host="192.168.10.50",
        )

        self.agvlist = AgvTaskListManager(self.config_param, self.station_mapping,self.agv_status)

        self.mqtt = RobotMqttHost(
            ip=self.config_param["mqtt_ip"],
            port=self.config_param["mqtt_port"],
            client_name=self.config_param["robotCode"],
        )

        self.voide_broadcast_pub = rospy.Publisher("/voide_broadcast_msg", String, queue_size=10)

        self.oper_pub = rospy.Publisher("/obsOperation_in", String, queue_size=10)

    # 嘉智达底盘暂无锁定
    def NavLock_callback(self, data):
        if "lock" == data["nav"]:
            pass
            # logger.info("底盘锁定")
            # self.nav_lock_pub.publish(Bool(True))
            # vel_msg = Twist()
            # vel_msg.angular.z = 0
            # vel_msg.linear.x = 0
            # vel_msg.linear.y = 0
            # for i in range(10):
            #     self.pub_nav_vel.publish(vel_msg)
        elif "unlock" == data["nav"]:
            pass
            # logger.info("底盘解锁")
            # self.nav_lock_pub.publish(Bool(False))

    def DMS_callback(self, oper_datas):
        with self.callback_fun_mutex:
            time.sleep(1)  # 留给状态改变的时间
            self.DMS_msg = oper_datas
            if self.operate_code_id == self.DMS_msg["id"]:
                return
            else:
                self.operate_code_id = self.DMS_msg["id"]
            if "query" in self.DMS_msg:
                logger.info("DMS问询")
                self.navi_serve_callback()
                self.oper_serve_callback()
                self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_serve_feedback_msg)

                self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                logger.info("成功答复")
                return

            # SPEED: 如果是充电指令，直接return, 不用打印
            if self.DMS_msg["operations"][0]["operation"] in ("charge", "uncharge"):
                return

            self.oper_msg_gen()
            logger.info("开始操作指令：")
            logger.info(self.DMS_msg)

            try:
                # 动作前解除充电
                tmp_operation = self.DMS_msg["operations"][0]["operation"]
                tmp_platform_status = self.navi_serve_callback(return_flage=1)

                # SPEED:充电状态，不管
                # if "charging" == tmp_platform_status["Status"]:
                #     if "uncharge" != tmp_operation and "charge" != tmp_operation:
                #         logger.info("解除充电，停靠停车区")
                #         # pubflag, feedback_msg = self.PubAgvTask("uncharge", self.config_param["Uncharge_Park"])
                #         pubflag = False
                #         while not pubflag:
                #             pubflag, feedback_msg = self.PubAgvTask(
                #                 "uncharge", self.config_param["Uncharge_Park"]
                #             )
                #             time.sleep(1)
                #             logger.info("等待发布任务成功")
                #         if pubflag:
                #             taskflag, self.navi_task_feedback_msg = self.TaskPeriod(feedback_msg)
                #             if taskflag:
                #                 logger.info("解除充电成功")
                #             else:
                #                 logger.info("解除充电异常")
                #                 self.robot_msg_gen(
                #                     self.navi_task_feedback_msg,
                #                     self.oper_topic_feedback_msg,
                #                 )
                #                 self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)

                # SPEED: 充电和解除充电指令，不管
                if (
                    "charge" == self.DMS_msg["operations"][0]["operation"]
                    or "uncharge" == self.DMS_msg["operations"][0]["operation"]
                ):
                    return
                    # with self.station_name_mutex:
                    #     self.now_workstation = self.DMS_msg["destination"]

                    # self.oper_reset_msg["destination"] = self.now_workstation
                    # self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                    # if self.oper_topic_feedback_msg["state"] == "done":
                    #     pubflag = False
                    #     while not pubflag:
                    #         pubflag, feedback_msg = self.PubAgvTask(
                    #             self.DMS_msg["operations"][0]["operation"], self.DMS_msg["destination"]
                    #         )
                    #         time.sleep(1)
                    #         logger.info("等待发布任务成功")
                    #     localtime = time.asctime(time.localtime(time.time()))
                    #     print(
                    #         "@本地时间:",
                    #         localtime,
                    #         " 移动至站点:",
                    #         self.DMS_msg["destination"],
                    #     )

                    #     if pubflag:
                    #         logger.info("发布" + " 移动至站点: " + self.DMS_msg["destination"] + "任务成功")
                    #         taskflag, self.navi_task_feedback_msg = self.TaskPeriod(feedback_msg)

                    #     if taskflag:
                    #         logger.info("执行" + " 移动至站点: " + self.DMS_msg["destination"] + "任务成功")
                    #     else:
                    #         logger.info("执行" + " 移动至站点: " + self.DMS_msg["destination"] + "任务失败")

                    #     self.robot_msg_gen(self.navi_task_feedback_msg, self.oper_topic_feedback_msg)
                    #     self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)

                    # else:
                    #     self.navi_serve_callback()
                    #     self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
                    #     self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                    # return

                # SPEED ADD
                logger.info("机器人当前状态：")
                logger.info(self.navi_serve_callback_only_station())

                # SPEED: 正常指令操作
                if (
                    self.DMS_msg["destination"] == self.now_workstation
                ):  # 如果当前站点就是所需工作站点,继续执行动作

                    # SPEED: 锁定底盘
                    self.LockBase()

                    # 为烘干等设备开门做准备
                    if "check" == self.DMS_msg["operations"][0]["operation"]:
                        self.oper_topic_feedback_msg["id"] = self.DMS_msg["id"]
                        self.oper_topic_feedback_msg["state"] = "done"
                        self.navi_serve_callback()
                        self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
                        self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                        return

                    self.OperAction(self.oper_msg, generate_msg_flag=True)

                    # SPEED: 如果操作成功，且该操作为 isEnd, 机械臂复位，底盘解锁
                    if self.oper_topic_feedback_msg["state"] == "done" and self.DMS_msg.get("isEnd") == 1:
                        self.oper_reset_msg["destination"] = self.now_workstation
                        self.OperAction(self.oper_reset_msg, generate_msg_flag=False)
                        self.UnlockBase()
                        self.now_workstation = "charge_station"

                else:  # 如果当前站点不是所需站点，需要移动至所需工作站再进行操作,先复位
                    # SPEED: 锁定底盘
                    self.LockBase()

                    # 机械臂复位
                    self.oper_reset_msg["destination"] = self.now_workstation
                    self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                    # self.oper_topic_feedback_msg["state"] = "done"
                    # 机械臂复位成功后底盘移动
                    if self.oper_topic_feedback_msg["state"] == "done":
                        pubflag = False
                        while not pubflag:
                            pubflag, feedback_msg = self.PubAgvTask(
                                self.DMS_msg["operations"][0]["operation"], self.DMS_msg["destination"]
                            )
                            logger.info("等待发布任务成功")
                            time.sleep(1)
                        localtime = time.asctime(time.localtime(time.time()))
                        if pubflag:
                            logger.info(
                                "@本地时间:"
                                + localtime
                                + " 发布: 移动至站点: "
                                + self.DMS_msg["destination"]
                                + "任务成功"
                            )
                            taskflag, self.navi_task_feedback_msg = self.TaskPeriod(feedback_msg)
                        # else:
                        #     logger.info("发布" + " 移动至站点: " + self.DMS_msg["destination"] + "任务失败")

                    elif "interrupt" == self.oper_topic_feedback_msg["state"]:
                        logger.info("interrupt return")
                        self.operate_code_id = 123
                        return

                    else:
                        self.navi_serve_callback()
                        self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
                        self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                        return

                    # 底盘移动成功后，动作前机械臂复位
                    if taskflag:
                        logger.info("执行" + " 移动至站点: " + self.DMS_msg["destination"] + "任务成功")
                        time.sleep(3)  # 空余点时间给底盘计算使用
                        with self.station_name_mutex:
                            self.now_workstation = self.DMS_msg["destination"]
                        self.oper_reset_msg["destination"] = self.now_workstation
                        logger.info("机械臂复位指令%r", self.oper_reset_msg)
                        self.OperAction(self.oper_reset_msg, generate_msg_flag=False)
                    else:
                        logger.info("执行" + " 移动至站点: " + self.DMS_msg["destination"] + "任务失败")
                        self.oper_serve_callback()
                        self.robot_msg_gen(self.navi_task_feedback_msg, self.oper_serve_feedback_msg)
                        self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                        return

                    self.oper_topic_feedback_msg["state"] == "done"
                    # 机械臂复位成功后机械臂重定位
                    if self.oper_topic_feedback_msg["state"] == "done":
                        self.oper_relocation_msg["destination"] = self.DMS_msg["destination"]
                        self.OperAction(self.oper_relocation_msg, generate_msg_flag=False)

                        # 重定位失败就再进行底盘移动与重定位
                        self.TryRelocation(relocation_number=0)
                    elif "interrupt" == self.oper_topic_feedback_msg["state"]:
                        logger.info("interrupt return")
                        self.operate_code_id = 123
                        return
                    else:
                        self.navi_serve_callback()
                        self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
                        self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                        return

                    self.oper_topic_feedback_msg["state"] == "done"
                    # 重定位成功后进行取放瓶子操作
                    if self.oper_topic_feedback_msg["state"] == "done":
                        with self.station_name_mutex:
                            self.now_workstation = self.DMS_msg["destination"]
                        # 为烘干箱等设备开门做准备
                        if "check" == self.DMS_msg["operations"][0]["operation"]:
                            self.oper_topic_feedback_msg["id"] = self.DMS_msg["id"]
                            self.oper_topic_feedback_msg["state"] = "done"
                            self.robot_msg_gen(
                                self.navi_task_feedback_msg,
                                self.oper_topic_feedback_msg,
                            )
                            self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)
                            return
                        self.OperAction(self.oper_msg, generate_msg_flag=True)
                        self.oper_topic_feedback_msg["state"] == "done"
                        # SPEED: 如果操作成功，且该操作为 isEnd, 机械臂复位，底盘解锁
                        # if self.oper_topic_feedback_msg["state"] == "done" and self.DMS_msg.get("isEnd") == 1:
                        if self.oper_topic_feedback_msg["state"] == "done":
                            self.oper_reset_msg["destination"] = self.now_workstation
                            self.OperAction(self.oper_reset_msg, generate_msg_flag=False)
                            self.UnlockBase()
                            self.now_workstation = "charge_station"
                    
                    elif "interrupt" == self.oper_topic_feedback_msg["state"]:
                        logger.info("interrupt return")
                        self.operate_code_id = 123
                        return
                    else:
                        self.navi_serve_callback()
                        self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
                        self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)

            except rospy.ROSInterruptException:
                logger.info("ROSInterruptException")
                pass

    def DmsFlaskHttpSever(self):
        self.robot_flask_sever.RunSever()

    def HikFlaskHttpServer(self):
        self.hik_flask_sever.RunServer()

    def OperAction(self, oper_msg, generate_msg_flag):
        self.oper_pub.publish(String(json.dumps(oper_msg)))

        # self.oper_topic_feedback_msg = json.loads(
        #     rospy.wait_for_message(self.Operation_topic_node_name, String).data
        # )
        self.oper_topic_feedback_msg["state"] = "done"

        if generate_msg_flag:
            if "interrupt" == self.oper_topic_feedback_msg["state"]:
                logger.info("interrupt return")
                self.operate_code_id = 123
                return
            self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
            self.mqtt.MqttPublish("robot-to-dms", json.dumps(self.robot_msg), 1)

    def TryRelocation(self, relocation_number=0):
        while "relocation_error" == self.oper_topic_feedback_msg["state"]:
            if relocation_number > 10:
                return
            # 机械臂复位
            self.oper_reset_msg["destination"] = self.now_workstation
            self.oper_pub.publish(String(json.dumps(self.oper_reset_msg)))
            # relocation_oper_topic_feedback_msg = json.loads(
            #     rospy.wait_for_message(self.Operation_topic_node_name, String).data
            # )
            relocation_oper_topic_feedback_msg["state"] = "done"
            # # SPEED: 底盘往外移动一点，外移站点后缀为P
            # pubflag = False
            # while not pubflag:
            #     pubflag, feedback_msg = self.PubAgvTask(
            #         self.DMS_msg["operations"][0]["operation"], self.DMS_msg["destination"] + "P"
            #     )
            #     time.sleep(1)
            #     logger.info("等待发布重定位任务成功")
            # if pubflag:
            #     logger.info("发布" + " 移动至站点: " + self.DMS_msg["destination"] + "P" + "任务成功")
            #     taskflag, relocation_navi_topic_feedback_msg = self.TaskPeriod(feedback_msg)
            #     if taskflag:
            #         logger.info("执行" + " 移动至站点: " + self.DMS_msg["destination"] + "P" + "任务成功")
            #     else:
            #         logger.info("执行" + " 移动至站点: " + self.DMS_msg["destination"] + "P" + "任务失败")
            #         return

            # SPEED: 重新移动到原站点
            pubflag = False
            while not pubflag:
                pubflag, feedback_msg = self.PubAgvTask(
                    self.DMS_msg["operations"][0]["operation"], self.DMS_msg["destination"]
                )
                time.sleep(1)
                logger.info("等待发布重定位任务成功")
            if pubflag:
                logger.info("发布" + " 重新移动至站点: " + self.DMS_msg["destination"] + "任务成功")
                taskflag, relocation_navi_topic_feedback_msg = self.TaskPeriod(feedback_msg)
                if taskflag:
                    logger.info("执行" + " 重新移动至站点: " + self.DMS_msg["destination"] + "任务成功")
                    with self.station_name_mutex:
                        self.now_workstation = self.DMS_msg["destination"]
                    self.oper_reset_msg["destination"] = self.now_workstation
                    self.oper_pub.publish(String(json.dumps(self.oper_reset_msg)))
                    # relocation_oper_topic_feedback_msg = json.loads(
                    #     rospy.wait_for_message(self.Operation_topic_node_name, String).data
                    # )
                    relocation_oper_topic_feedback_msg = "done"
                else:
                    logger.info("执行" + " 重新移动至站点: " + self.DMS_msg["destination"] + "任务失败")
                    return
            if relocation_oper_topic_feedback_msg["state"] == "done":
                self.oper_relocation_msg["destination"] = self.DMS_msg["destination"]
                self.oper_pub.publish(String(json.dumps(self.oper_relocation_msg)))
                # self.oper_topic_feedback_msg = json.loads(
                #     rospy.wait_for_message(self.Operation_topic_node_name, String).data
                # )
                self.oper_topic_feedback_msg["state"] = "done"

                relocation_number += 1
            else:
                return

    def oper_msg_gen(self):
        self.oper_msg["id"] = self.DMS_msg["id"]
        # self.oper_msg["expr_no"] = self.DMS_msg["expr_no"]
        self.oper_msg["stamp"] = self.DMS_msg["stamp"]
        self.oper_msg["destination"] = self.DMS_msg["destination"]
        self.oper_msg["operations"] = self.DMS_msg["operations"]
        self.oper_reset_msg["id"] = self.DMS_msg["id"]
        self.oper_relocation_msg["id"] = self.DMS_msg["id"]

    def SendHreartbeatMsg(self):
        # logger.info("Send heartbeat")

        with self.station_name_mutex:
            self.robot_heartbeat_msg["currentStation"] = self.now_workstation

        agv_info_datas = self.navi_serve_callback(1)

        self.robot_heartbeat_msg["battery"] = agv_info_datas["battery"]
        self.robot_heartbeat_msg["stamp"] = round(time.time() * 1000)

        data = json.dumps(self.robot_heartbeat_msg)

        # print(self.heartbeat_status)

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
            logger.info("DMS回复:")
            logger.info(response.text)
        except Exception as e:
            logger.info(e)
    
    #发布任务
    def PubAgvTask(self, navi_command, navi_destination):
        jreq = self.agvlist.TaskList(navi_command, navi_destination)
        try:
            resp = requests.post(
                self.AGV_url + "genAgvSchedulingTask", 
                jreq.encode("utf-8"), 
                headers=self.headers, 
                timeout=(60, 15)
            )

        except Exception as e:
            logger.info(e)

        dresp = json.loads(resp.text)
        task = self.agvlist.CheckTaskFeedback(dresp)
        if task != "OK":
            logger.info("申请失败:\n" + task)
            return False, dresp
        else:
            logger.info("申请 " + " to " + str(navi_destination) + "成功")
            return True, dresp

    def TaskPeriod(self, feedback_msg, waitime=60):
        # start = time.time()
        # past = 0
        accept = False
        while True:
            time.sleep(1)
            jreq = self.agvlist.ListQuest(feedback_msg["data"])
            try:
                resp = requests.post(
                    self.AGV_url + "queryTaskStatus",
                    jreq.encode("utf-8"),
                    headers=self.headers,
                    timeout=(60, 15),
                )

            except Exception as e:
                logger.info(e)

            dresp = json.loads(resp.text)
            #logger.info(dresp)
            if dresp["data"][0]["taskStatus"] == "1":
                logger.info("订单已创建, 等待机器人接单")
            if dresp["data"][0]["taskStatus"] == "2" and not accept:
                accept = True
                logger.info("订单正在执行, 等待机器人完成订单")
            if dresp["data"][0]["taskStatus"] == "5":
                logger.info("订单被取消")
                return False, dresp
            if dresp["data"][0]["taskStatus"] == "9":
                logger.info("订单完成")
                return True, dresp
            if dresp["data"][0]["taskStatus"] == "10":
                logger.info("订单被打断")
                return False, dresp
                
            
    def navi_serve_callback(self, return_flage=0):
        jreq = self.agvlist.StatusQuest(self.hik_system)
        #logger.info(f"post message: ,{jreq}")
        try:
            resp = requests.post(
                "http://192.168.10.50:8182/rcms-dps/rest/queryAgvStatus",
                data=jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15),
            )

        except Exception as e:
            logger.info(e)
            return None

        raw_info = {}
        if resp and resp.status_code == 200:
            raw_info = json.loads(resp.text)
            return_flage = 1
            #logger.info(raw_info)
            #logger.info(return_flage)
        else:
            logger.info(f"Failed to fetch AGV status: {resp.status_code}")
            return None

        # 提取指定AGV的信息
        agv_info = self.agvlist.ExtractAgvInfo(raw_info)
        self.navi_serve_feedback_msg = agv_info
        
        if return_flage and agv_info != None:
            #logger.info("API success")
            return agv_info
        else:
            logger.info(f"API returned error: {raw_info.get('message')},{agv_info}")
    def navi_serve_callback_only_station(self, return_flage=0):
        jreq = self.agvlist.StatusQuest(self.hik_system)

        try:
            resp = requests.post(
                "http://192.168.10.50:8182/rcms-dps/rest/queryAgvStatus",
                data=jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15),
            )

        except Exception as e:
            logger.info(e)
            return None

        raw_info = {}
        if resp and resp.status_code == 200:
            return_flage = 1
            raw_info = json.loads(resp.text)
            #logger.info(raw_info)
        else:
            logger.info(f"Failed to fetch AGV status: {resp.status_code}")
            return None

        # 提取指定AGV的信息
        agv_info = self.agvlist.ExtractAgvInfo(raw_info)
        self.navi_serve_feedback_msg = agv_info
        
        if return_flage and agv_info != None:
            return agv_info["podCode"]
        else:
            logger.info(f"API returned error: {raw_info.get('message')}")

    def oper_serve_callback(self):
        client = rospy.ServiceProxy(self.Operation_serve_node_name, DmsService)
        client.wait_for_service()
        req = DmsServiceRequest()
        req.data = ""  # self.DMS_msg
        resp = client.call(req.data)
        raw_info = resp.data
        # print(raw_info)
        self.oper_serve_feedback_msg = json.loads(raw_info)

    def robot_msg_gen(self, navi, oper):
        oper["id"]="111"
        self.robot_msg["current_workstation"] = self.now_workstation
        if navi["status"] in ["Idle_task","Task_completed"]:         #任务空闲
            self.robot_msg["id"] = oper["id"]
            # self.robot_msg["exper_no"] = oper["exper_no"]
            self.robot_msg["stamp"] = str(int(round(time.time() * 1000)))
            self.robot_msg["state"] = oper["state"]
            logger.info("%s - 操作ID: %s", datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"), oper["id"])
        else:
            logger.info("else")
            self.robot_msg["id"] = navi["id"]   
            # self.robot_msg["exper_no"] = navi["exper_no"]
            self.robot_msg["stamp"] = str(int(round(time.time() * 1000)))
            self.robot_msg["state"] = navi["status"]
        if "robotCode" in navi:
            self.robot_msg["detail"]["navi"] = navi["status"]
        else:
            pass

        if "detail" in oper:
            self.robot_msg["detail"]["oper"] = oper["detail"]
        else:
            pass

        # 查询报警信息
        # 不确定靠不靠谱，海康文档里没有对AGV状态有详细的说明，如果不能用来进行判断，就用warnCallback

        if "Abnormal_task" == navi["status"]:   #任务异常 and 机器人暂停
            self.voide_broadcast_pub.publish(String(navi["status"].encode("utf-8").decode("utf-8")))
        if "error" == oper["state"]:
            self.voide_broadcast_pub.publish(String(oper["detail"].encode("utf-8").decode("utf-8")))


    def GetBaseLockMsg(self):
        self.baselock_msg["status"] = self.hik_flask_sever.agv_status
        return self.baselock_msg
        
    def LockBase(self):
        # if self.baselock_msg["status"] == "start":
        #     logger.info("底盘锁定")
        logger.info("底盘锁定")
        time.sleep(0.2)
        
    def UnlockBase(self):
        # while True:
        #     if self.baselock_msg["status"] == "end":
        #         logger.info("底盘解锁")
        #         break
        #     else:
        #         logger.info("底盘未解锁")
        logger.info("底盘解锁")
        time.sleep(0.2)  # 等待状态改变

        

    # RS-2000底盘到达站点自动锁定
    # def LockBase(self):
    #     # 不确定靠不靠谱，海康文档里没有对AGV状态有详细的说明，如果不能用来进行判断，就用agvCallback
    #     # self.hik_flask_sever.agv_status
    #     platform_status = self.navi_serve_feedback_msg["status"]
    #     if platform_status == "Robot_stopped":
    #         logger.info("底盘锁定")
    #         self.baselock_msg["status"] = "BUSY"

    # RS-2000底盘解锁需要通过接口发送解锁指令,若订单只发单个站点，不用解锁   
    # def UnlockBase(self):
    #     # 不确定靠不靠谱，海康文档里没有对AGV状态有详细的说明，如果不能用来进行判断，就用agvCallback
    #     # self.hik_flask_sever.agv_status
    #     while True:
    #         platform_status = self.navi_serve_feedback_msg["status"]
    #         if platform_status != "Robot_stopped":
    #             logger.info("底盘解锁")
    #             break  # 退出循环，因为已经解锁成功
    #         else:
    #             logger.info("底盘未解锁，尝试重新发送请求")

    #         jreq = self.agvlist.ContinueTask()
    #         try:
    #             resp = requests.post(
    #                 self.AGV_url + "continueTask", 
    #                 jreq.encode("utf-8"), 
    #                 headers=self.headers, 
    #                 timeout=(60, 15)
    #             )
    #         except Exception as e:
    #             logger.info(f"请求失败: {e}")
    #             continue  # 请求失败，继续循环重新发送请求

    #获取报警信息
    def GetStatusAlarmMsg(self):
        return self.robot_heartbeat_msg


    def heartbeat_monitoring(self):  # 5秒一次的心跳监听
        logger.info("heartbeat_monitoring_init")
        while True:
            pinged, unpinged = rosnode.rosnode_ping_all()
            if unpinged:
                master = rosnode.rosgraph.Master(rosnode.ID)
                rosnode.cleanup_master_blacklist(master, unpinged)

            now_node_list = rosnode.get_node_names()
            if len(list(self.node_list.keys())) <= len(set(now_node_list)):
                self.query_msg["stamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                self.navi_serve_callback()
                #self.oper_serve_callback()
                robot_details = ""
                if self.navi_serve_feedback_msg is None:
                    print("警告：navi_serve_feedback_msg 是 None，跳过状态检查。")
                    return 
                platform_status = self.navi_serve_feedback_msg["status"]
                #logger.info(platform_status)
                #arm_status = self.oper_serve_feedback_msg["state"]
                arm_status = "done"
                # 不确定靠不靠谱，海康文档里没有对AGV状态有详细的说明，如果不能用来进行判断，就用agvCallback
                # self.hik_flask_sever.agv_status
                if platform_status == "Abnormal_task" or arm_status == "error":
                    self.robot_heartbeat_msg["status"] = "ERROR"
                elif platform_status == "Executing_task" or arm_status == "running":
                    self.robot_heartbeat_msg["status"] = "BUSY"
                elif platform_status == "Task_completed" or platform_status == "Idle_task" or arm_status == "done":
                    self.robot_heartbeat_msg["status"] = "IDLE"
                elif platform_status == "Charging_status":
                    self.robot_heartbeat_msg["status"] = "CHARGING"
                elif self.callback_fun_mutex.locked():
                    self.robot_heartbeat_msg["status"] = "BUSY"
                else:   #其他状态怎么表示？
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

    def Run(self):
        threading.Thread(target=self.heartbeat_monitoring).start()
        threading.Thread(target=self.DmsFlaskHttpSever).start()
        #threading.Thread(target=self.HikFlaskHttpServer).start()


if __name__ == "__main__":

    logger = Log(__name__).getlog()

    try:
        rospy.init_node("robot_msg_manager_node", anonymous=True, disable_signals=True)
        manager = RobotMsgManager()
        logger.info("initial node")
        manager.Run()
        rospy.spin()
    except:
        logger.critical(str(traceback.format_exc()))
