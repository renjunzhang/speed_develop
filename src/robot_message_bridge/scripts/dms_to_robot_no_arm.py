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
        self.AGV_url2 = self.config_param["AGV_url2"]
        self.operate_code_id = self.config_param["operate_code_id"]

        self.oper_topic_feedback_msg = dict()
        self.navi_task_feedback_msg = dict()
        self.navi_serve_feedback_msg = dict()
        self.oper_serve_feedback_msg = dict()
        self.DMS_all_msg = dict()
        
        self.DMS_msg = dict()

        self.taskCode = ""

        self.robot_heartbeat_msg = {
            "identifyingCode": "hik_robot_11335",  # 机器人唯一标识码
            "ip": GetIP(),
            "port": 8080,
            "firmwareVersion": "1.0.0",  # 固件版本号，可从配置读取或硬编码
            "status": "IDLE",
            "msg": "",
            "data": {}
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
        self.oper_msg = {"id": "", "stamp": "", "destination": "", "operations": []}
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

        self.dms_url = self.config_param["dms_url"]  # 从配置文件读取指令和心跳 URL
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
            port=3030,
            debug=False,
            host="0.0.0.0",
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

    # 3*3种排列组合，屎上最强http请求！！！
    def determine_destination(self, operations):
        """
        判断底盘移动的目标工作站（destination）
        - source 和 target 都存在：
            - 如果两者都是 robotCode，返回 None。
            - 如果两者相同且不是 robotCode，返回该工作站。
            - 如果其中一个是 robotCode，返回另一个（非 robotCode）工作站。
            - 如果两者都不是 robotCode 且不同，返回 "error"。
        - source 存在，target 不存在：
            - 如果 source.workstation 是 robotCode，返回 None。
            - 否则，返回 source.workstation。
        - target 存在，source 不存在：
            - 如果 target.workstation 是 robotCode，返回 None。
            - 否则，返回 target.workstation。
        - source 和 target 都不存在，返回 None。
        """
        robot_code = self.config_param["robotCode"]
        source = operations.get("source", {})
        target = operations.get("target", {})
        source_workstation = source.get("workstation")
        target_workstation = target.get("workstation")

        # 情况1：source 和 target 都存在
        if source_workstation and target_workstation:
            if source_workstation == robot_code and target_workstation == robot_code:
                logger.info("source 和 target 都是 robotCode，不用移动")
                return None
            elif source_workstation == target_workstation:
                return source_workstation  # 两者相同且不是 robotCode
            elif source_workstation == robot_code:
                return target_workstation  # source 是 robotCode，返回 target
            elif target_workstation == robot_code:
                return source_workstation  # target 是 robotCode，返回 source
            else:
                logger.error("source 和 target 都不是 robotCode 且不同，无效目标")
                return "error"

        # 情况2：source 存在，target 不存在
        elif source_workstation and not target_workstation:
            if source_workstation == robot_code:
                logger.info("source 是 robotCode，target 不存在，不用移动")
                return None
            return source_workstation

        # 情况3：target 存在，source 不存在
        elif target_workstation and not source_workstation:
            if target_workstation == robot_code:
                logger.info("target 是 robotCode，source 不存在，不用移动")
                return None
            return target_workstation

        # 情况4：source 和 target 都不存在
        else:
            logger.info("source 和 target 都不存在，不用移动")
            return None

    def process_operation(self, operations):
        """
        根据输入的 operations 判断并生成最终的 operation。
        如果在处理过程中发现缺少必要的键，则返回 "error"。

        参数:
            operations (dict): 输入的操作字典，包含 operation, source, target 等字段。

        返回:
            str: 处理后的 operation 字符串，或 "error"（如果缺少必要键）。
        """
        # 获取 robot_code
        robot_code = self.config_param["robotCode"]

        # 获取 operation, source, target
        operation = operations.get("operation")
        source = operations.get("source")
        target = operations.get("target")

        # 只有当 source 和 target 都存在且 operation 为 "move" 时才处理
        if source and target and operation == "move":
            source_workstation = source.get("workstation")
            target_workstation = target.get("workstation")

            # 确保 source 和 target 中有 workstation 键
            if source_workstation and target_workstation:
                # 情况 1: source 是 robotCode, target 是 workstation
                if source_workstation == robot_code and target_workstation != robot_code:
                    if all(key in source for key in ["rack_slot_position", "bottle_slot_position"]) and "bottle_slot_position" in target:
                        return f"put_r-{source['rack_slot_position']}-{source['bottle_slot_position']}_w-{target['bottle_slot_position']}"
                    else:
                        return "error"  # 缺少必要键

                # 情况 2: target 是 workstation, source 是 robotCode
                elif target_workstation == robot_code and source_workstation != robot_code:
                    if "bottle_slot_position" in source and all(key in target for key in ["rack_slot_position", "bottle_slot_position"]):
                        return f"take_w-{source['bottle_slot_position']}_r-{target['rack_slot_position']}-{target['bottle_slot_position']}"
                    else:
                        return "error"  # 缺少必要键

                # 情况 3: target 和 source 都是 workstation
                elif source_workstation != robot_code and target_workstation != robot_code:
                    if "bottle_slot_position" in source and "bottle_slot_position" in target:
                        return f"move_w-{source['bottle_slot_position']}_w-{target['bottle_slot_position']}"
                    else:
                        return "error"  # 缺少必要键

                # 情况 4: target 和 source 都是 robotCode
                elif source_workstation == robot_code and target_workstation == robot_code:
                    if all(key in source for key in ["rack_slot_position", "bottle_slot_position"]) and all(key in target for key in ["rack_slot_position", "bottle_slot_position"]):
                        return f"move_r-{source['rack_slot_position']}-{source['bottle_slot_position']}_r-{target['rack_slot_position']}-{target['bottle_slot_position']}"
                    else:
                        return "error"  # 缺少必要键

        # 默认情况：source 或 target 缺失，或 operation != "move"，或其他无效情况
        return operation

    def DMS_callback(self, oper_datas):
        self.oper_topic_feedback_msg["state"] = "done"
        with self.callback_fun_mutex:
            time.sleep(1)  # 留给状态改变的时间
            self.DMS_all_msg = oper_datas

            # 检查指令 ID 是否重复
            if self.operate_code_id == self.DMS_all_msg["id"]:
                return
            else:
                self.operate_code_id = self.DMS_all_msg["id"]

            # 处理查询指令
            if "query" in self.DMS_all_msg:
                logger.info("DMS 问询")
                self.navi_serve_callback()
                # self.oper_serve_callback()
                self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_serve_feedback_msg)
                self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                logger.info("成功答复")
                return

            # 检查是否为充电/解除充电指令，如果是直接return
            if "param" in self.DMS_all_msg and self.DMS_all_msg["param"]:
                if self.DMS_all_msg["param"][0]["operation"] in ("charge", "uncharge"):
                    return 

            logger.info("开始操作指令：")
            logger.info(self.DMS_all_msg)

            try:
                # 检查当前 AGV 状态
                tmp_platform_status = self.navi_serve_callback(return_flage=1)
                logger.info("机器人当前状态：")
                logger.info(self.navi_serve_callback_only_station())

                # 遍历 "param" 中的所有操作
                for idx, operations in enumerate(self.DMS_all_msg["param"]):
                    logger.info(f"处理第 {idx+1} 个操作: {operations}")

                    # 确定目标工作站，主打一个猜
                    destination = self.determine_destination(operations)
                    real_operation = self.process_operation(operations)

                    logger.info("确定的destination=" + destination)
                    logger.info("确定的operation=" + real_operation)
                    
                    if destination == "error":
                        self.robot_msg["id"] = self.DMS_all_msg["id"]
                        self.robot_msg["state"] = "error"
                        self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 无效的工作站判断"
                        self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                        return

                    if destination and destination not in self.station_mapping:
                        logger.error(f"未知的工作站: {destination}")
                        self.robot_msg["id"] = self.DMS_all_msg["id"]
                        self.robot_msg["state"] = "error"
                        self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 未知的工作站 {destination}"
                        self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                        return

                    if real_operation == "error":
                        self.robot_msg["id"] = self.DMS_all_msg["id"]
                        self.robot_msg["state"] = "error"
                        self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 缺少必要键"
                        self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                        return
                    
                    # 生成当前操作的 oper_msg
                    operations["operation"] = real_operation
                    self.DMS_msg["id"] = f"{self.DMS_all_msg['id']}-{idx+1}"
                    self.DMS_msg["stamp"] = str(int(round(time.time() * 1000)))
                    self.DMS_msg["destination"] = destination
                    self.DMS_msg["operations"] = [operations]
                    # logger.info("DMS_msg"+self.DMS_msg)
                    self.oper_msg_gen()

                    # 2025/4/11 检查并解除充电状态,暂时没用
                    # if tmp_platform_status and "charging" == tmp_platform_status["Status"]:
                    #     if operations["operation"] not in ("charge", "uncharge"):
                    #         logger.info("解除充电，停靠停车区")
                    #         pubflag = False
                    #         while not pubflag:
                    #             pubflag, feedback_msg = self.PubAgvTask("uncharge", self.config_param["Uncharge_Park"])
                    #             time.sleep(1)
                    #             logger.info("等待发布任务成功")
                    #         if pubflag:
                    #             taskflag, self.navi_task_feedback_msg = self.TaskPeriod(feedback_msg)
                    #             if not taskflag:
                    #                 logger.info("解除充电失败")
                    #                 self.robot_msg["id"] = self.DMS_all_msg["id"]
                    #                 self.robot_msg["state"] = "error"
                    #                 self.robot_msg["detail"]["navi"] = "解除充电失败"
                    #                 self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                    #                 return

                    # 如果当前站带你就是所需工作站，继续执行操作
                    if self.DMS_msg["destination"] == self.now_workstation or self.DMS_msg["destination"] is None:
                        self.LockBase()

                        # 为烘干设备开门等特殊操作
                        if operations["operation"] == "check":
                            self.oper_topic_feedback_msg["id"] = self.DMS_all_msg["id"]
                            self.oper_topic_feedback_msg["state"] = "done"
                            self.navi_serve_callback()
                            self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
                            self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                            return

                        self.OperAction(self.oper_msg, generate_msg_flag=True)

                        if self.oper_topic_feedback_msg["state"] == "interrupt":
                            logger.info("中断返回")
                            self.operate_code_id = 123
                            return

                        if self.oper_topic_feedback_msg["state"] != "done":
                            self.robot_msg["id"] = self.DMS_all_msg["id"]
                            self.robot_msg["state"] = "error"
                            self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作失败: {json.dumps(operations)}"
                            self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                            return

                    # 如果当前站点不是所需站点，需要移动至所需工作站再进行操作,先复位
                    else:

                        self.LockBase()

                        # 移动前机械臂复位
                        self.oper_reset_msg["destination"] = self.now_workstation
                        self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                        if self.oper_topic_feedback_msg["state"] == "interrupt":
                            logger.info("中断返回")
                            self.operate_code_id = 123
                            return

                        if self.oper_topic_feedback_msg["state"] != "done":
                            self.robot_msg["id"] = self.DMS_all_msg["id"]
                            self.robot_msg["state"] = "error"
                            self.robot_msg["detail"]["oper"] = "移动前机械臂复位失败"
                            self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                            return

                        # 移动到目标站点
                        pubflag = False
                        while not pubflag:
                            pubflag, feedback_msg = self.PubAgvTask(operations["operation"], self.DMS_msg["destination"])
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
                            if taskflag:
                                logger.info("执行" + " 移动至站点: " + self.DMS_msg["destination"] + "任务成功")
                                time.sleep(3)  # 空余点时间给底盘计算使用
                                with self.station_name_mutex:
                                    self.now_workstation = self.DMS_msg["destination"]

                                # 移动后机械臂复位
                                self.oper_reset_msg["destination"] = self.now_workstation
                                self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                                if self.oper_topic_feedback_msg["state"] == "interrupt":
                                    logger.info("中断返回")
                                    self.operate_code_id = 123
                                    return

                                if self.oper_topic_feedback_msg["state"] == "done":
                                    # 机械臂重定位
                                    self.oper_relocation_msg["destination"] = self.DMS_msg["destination"]
                                    self.OperAction(self.oper_relocation_msg, generate_msg_flag=False)
                                    self.TryRelocation(relocation_number=0)

                                    if self.oper_topic_feedback_msg["state"] == "interrupt":
                                        logger.info("中断返回")
                                        self.operate_code_id = 123
                                        return

                                    if self.oper_topic_feedback_msg["state"] == "done":
                                        if operations["operation"] == "check":
                                            self.oper_topic_feedback_msg["id"] = self.DMS_all_msg["id"]
                                            self.oper_topic_feedback_msg["state"] = "done"
                                            self.robot_msg_gen(self.navi_task_feedback_msg, self.oper_topic_feedback_msg)
                                            self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                                            return

                                        self.OperAction(self.oper_msg, generate_msg_flag=True)

                                        if self.oper_topic_feedback_msg["state"] == "interrupt":
                                            logger.info("中断返回")
                                            self.operate_code_id = 123
                                            return

                                        if self.oper_topic_feedback_msg["state"] != "done":
                                            self.robot_msg["id"] = self.DMS_all_msg["id"]
                                            self.robot_msg["state"] = "error"
                                            self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作失败: {json.dumps(operations)}"
                                            self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                                            return
                                    else:
                                        self.robot_msg["id"] = self.DMS_all_msg["id"]
                                        self.robot_msg["state"] = "error"
                                        self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 重定位失败"
                                        self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                                        return
                                else:
                                    self.robot_msg["id"] = self.DMS_all_msg["id"]
                                    self.robot_msg["state"] = "error"
                                    self.robot_msg["detail"]["oper"] = "移动后机械臂复位失败"
                                    self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                                    return
                            else:
                                self.robot_msg["id"] = self.DMS_all_msg["id"]
                                self.robot_msg["state"] = "error"
                                self.robot_msg["detail"]["navi"] = "移动任务失败"
                                self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)
                                return

                # 所有操作完成后复位并上报成功
                self.oper_reset_msg["destination"] = self.now_workstation
                self.OperAction(self.oper_reset_msg, generate_msg_flag=False)
                self.UnlockBase()
                self.now_workstation = "charge_station"
                
                self.robot_msg["id"] = self.DMS_all_msg["id"]
                self.robot_msg["state"] = "done"
                self.robot_msg["detail"]["oper"] = "所有操作完成"
                self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)

            except rospy.ROSInterruptException:
                logger.info("ROSInterruptException")
                pass

    def DmsFlaskHttpSever(self):
        self.robot_flask_sever.RunSever()

    def HikFlaskHttpServer(self):
        self.hik_flask_sever.RunServer()

    def OperAction(self, oper_msg, generate_msg_flag):
        self.oper_topic_feedback_msg["state"] = "done"
        # self.oper_pub.publish(String(json.dumps(oper_msg)))

        # self.oper_topic_feedback_msg = json.loads(
        #     rospy.wait_for_message(self.Operation_topic_node_name, String).data
        # )

        # if generate_msg_flag:
        #     if "interrupt" == self.oper_topic_feedback_msg["state"]:
        #         logger.info("interrupt return")
        #         self.operate_code_id = 123
        #         return
        #     self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
        #     self.mqtt.MqttPublish("/robot-to-dms", json.dumps(self.robot_msg, ensure_ascii=False).encode("utf-8"), 1)

    def TryRelocation(self, relocation_number=0):
        relocation_oper_topic_feedback_msg = {}
        relocation_oper_topic_feedback_msg["state"] = "done"
        # while "relocation_error" == self.oper_topic_feedback_msg["state"]:
        #     if relocation_number > 10:
        #         return
        #     # 机械臂复位
        #     self.oper_reset_msg["destination"] = self.now_workstation
        #     self.oper_pub.publish(String(json.dumps(self.oper_reset_msg)))
        #     relocation_oper_topic_feedback_msg = json.loads(
        #         rospy.wait_for_message(self.Operation_topic_node_name, String).data
        #     )
        #     pubflag = False
        #     while not pubflag:
        #         pubflag, feedback_msg = self.PubAgvTask(
        #             self.DMS_msg["operations"][0]["operation"], self.DMS_msg["destination"]
        #         )
        #         time.sleep(1)
        #         logger.info("等待发布重定位任务成功")
        #     if pubflag:
        #         logger.info("发布" + " 重新移动至站点: " + self.DMS_msg["destination"] + "任务成功")
        #         taskflag, relocation_navi_topic_feedback_msg = self.TaskPeriod(feedback_msg)
        #         if taskflag:
        #             logger.info("执行" + " 重新移动至站点: " + self.DMS_msg["destination"] + "任务成功")
        #             with self.station_name_mutex:
        #                 self.now_workstation = self.DMS_msg["destination"]
        #             self.oper_reset_msg["destination"] = self.now_workstation
        #             self.oper_pub.publish(String(json.dumps(self.oper_reset_msg)))
        #             relocation_oper_topic_feedback_msg = json.loads(
        #                 rospy.wait_for_message(self.Operation_topic_node_name, String).data
        #             )
        #         else:
        #             logger.info("执行" + " 重新移动至站点: " + self.DMS_msg["destination"] + "任务失败")
        #             return
        #     if relocation_oper_topic_feedback_msg["state"] == "done":
        #         self.oper_relocation_msg["destination"] = self.DMS_msg["destination"]
        #         self.oper_pub.publish(String(json.dumps(self.oper_relocation_msg)))
        #         self.oper_topic_feedback_msg = json.loads(
        #             rospy.wait_for_message(self.Operation_topic_node_name, String).data
        #         )
        #         relocation_number += 1
        #     else:
        #         return

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
            # logger.info("DMS回复:")
            # logger.info(response.text)
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
        # logger.info(self.hik_system)

        try:
            resp = requests.post(
                # self.AGV_url2 + "queryAgvStatus",
                "http://192.168.200.31:8182/rcms-dps/rest/queryAgvStatus",
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
            return agv_info
        else:
            logger.info(f"API returned error: {raw_info.get('message')}")
    
    def navi_serve_callback_only_station(self, return_flage=0):
        jreq = self.agvlist.StatusQuest(self.hik_system)
        logger.info(self.hik_system)

        try:
            resp = requests.post(
                # self.AGV_url2 + "queryAgvStatus",
                "http://192.168.200.31:8182/rcms-dps/rest/queryAgvStatus",
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
            logger.info(raw_info)
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
        self.robot_msg["current_workstation"] = self.now_workstation
        if navi["status"] in ["Idle_task","Task_completed"]:         #任务空闲
            self.robot_msg["id"] = oper["id"]
            # self.robot_msg["exper_no"] = oper["exper_no"]
            self.robot_msg["stamp"] = str(int(round(time.time() * 1000)))
            self.robot_msg["state"] = oper["state"]
            logger.info("%s - 操作ID: %s", datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"), oper["id"])
        else:
            # logger.info("else")
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


    #获取报警信息
    def GetStatusAlarmMsg(self):
        return self.robot_heartbeat_msg


    def heartbeat_monitoring(self):  # 3秒一次的心跳监听
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
                # self.oper_serve_callback()

                robot_details = ""
                platform_status = self.navi_serve_feedback_msg["status"]
                # logger.info(platform_status)
                self.oper_serve_feedback_msg["state"] = "done"
                arm_status = self.oper_serve_feedback_msg["state"]
                # logger.info(arm_status)
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
        # threading.Thread(target=self.HikFlaskHttpServer).start()


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