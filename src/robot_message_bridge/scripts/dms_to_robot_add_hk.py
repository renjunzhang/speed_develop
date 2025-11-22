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

import logging
import threading
import traceback
from datetime import datetime
from robot_flask_server import RobotFlaskServer
from robot_common_log import Log
import rospkg
import socket
import sys

from pprint import pprint
from agv_tasklist_manager import AgvTaskListManager


import inspect
def print_function_name_and_lineno():
    # 获取当前帧对象
    frame = inspect.currentframe()
    # 获取调用栈信息
    stack = inspect.getouterframes(frame)
    # 获取当前函数名和行号
    function_name = stack[1][3]
    lineno = stack[1][2]
    # 打印函数名和行号
    print(f"Function name: {function_name}, Line number: {lineno}")


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

        param_file_path = (rospkg.RosPack().get_path("robot_message_bridge") + "/config/param.json")
        with open(param_file_path, encoding="utf-8") as f:
            self.config_param = json.load(f)

        wk_operation_file_path = (rospkg.RosPack().get_path("robot_message_bridge") + "/config/workstataion_operations.json")
        with open(wk_operation_file_path, encoding="utf-8") as f:
            self.workstation_operations = json.load(f)

        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/station_mapping.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.station_mapping = json.load(f)

        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/status.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.agv_status = json.load(f)
            

        self.headers = {"content-type": "application/json"}  # DMS接收的header
        self.Operation_topic_node_name = "/obsOperation_out"
        self.Operation_serve_node_name ="/robot"

        self.node_list = {
            "/aruco_to_point_and_talk": "相机节点",
            # "/sick_front/sick": "前雷达",
            # "/sick_rear/sick": "后雷达",
            # '/hf_platform/joy_node',  # 手柄
            # "/hf_platform/rs485_control": "底盘",
            # "/map_server": "地图",
            # "/platform_communication_node": "底盘通信节点",
            # '/chemicalrobot_arm': '机械臂',
            # '/ur5e', #机器臂的控制程
            # "/rq_sensor": "力矩传感器",
            "/chemicalrobot_arm_new": "机械臂",
            # '/pgitest' #夹爪
        }

        self.operate_code_id = 123
        self.AGV_url = self.config_param["AGV_url"]

        self.navi_topic_feedback_msg = dict()
        self.oper_topic_feedback_msg = dict()
        self.navi_serve_feedback_msg = dict()
        self.oper_serve_feedback_msg = dict()
        self.DMS_msg = dict()

        self.taskCode = ""

        self.robot_heartbeat_msg = {
            "identifyingCode": self.config_param["identifyingCode"], 
            "status": "",
            "detailMsg": "",
            "electricityQuantity": 80.0,
            "machineCode": self.config_param["robotCode"],  # 机器人id
            "detail": None,  # 机器人不用，为固定值
            "ip": GetIP(),
            "port": 3030,
            "uri": "/",
            "firmwareVersion":"1.0.0"

        }

        self.robot_msg = {
            "id": "",
            "stamp": "",
            "status": "",
            "current_workstation": "",
            "state": "",
            "detail": {
                "navi": "",
                "oper": "" 
            },
        }
        self.query_msg = {"stamp": "", "query": "true"}

        self.navi_msg = {"id": "", "stamp": "", "destination": "", "action": "move"}
        self.oper_msg = {"id": "", "stamp": "", "destination": "", "operations": ""}

        self.oper_reset_msg = {
            "id": "",
            "destination": "",
            "operations": [{
                "operation": "reset"
            }],
        }
        self.oper_relocation_msg = {
            "id": "",
            "destination": "",
            "operations": [{
                "operation": "relocation"
            }],
        }
        
        # base lock
        self.baselock_msg = {"agv_id": self.config_param["AGV_name"], "status": "OK"}


        self.now_workstation = "charge_station"
        self.libs_fire_flag = False
        self.heartbeat_status = "init"

        self.station_name_mutex = threading.Lock()
        self.callback_fun_mutex = threading.Lock()

        self.InitMessage()

    def InitMessage(self):

        self.dms_url = self.config_param["dms_url"]  # DMS接收的url
        self.response_url = self.config_param["response_url"]
        self.robot_flask_server = RobotFlaskServer(
            self.dms_callback,
            self.GetStatusAlarmMsg,
            self.GetBaseLockMsg,
            3030,
            False,
            "0.0.0.0",
        )

        self.agvlist = AgvTaskListManager(self.config_param, self.station_mapping, self.agv_status)

        self.oper_pub = rospy.Publisher("/obsOperation_in", String, queue_size=10)


    def OperAction(self, oper_msg, generate_msg_flag):
        self.oper_pub.publish(String(json.dumps(oper_msg)))

        self.oper_topic_feedback_msg = json.loads(rospy.wait_for_message(self.Operation_topic_node_name, String).data)

        get_feedback_msg_state = self.oper_topic_feedback_msg["state"]
        print(f"invoke OperAction, get oper_topic_feedback_msg[state]: {get_feedback_msg_state}")

        if generate_msg_flag and self.oper_topic_feedback_msg["state"] == "done":
            self.generate_and_send_status()
        elif generate_msg_flag and self.oper_topic_feedback_msg["state"] == "error":
            logger.error("机械臂操作失败，状态为error")
            self.generate_and_send_status(500)

        if generate_msg_flag:
            if "interrupt" == self.oper_topic_feedback_msg["state"]:
                logger.info("interrupt return")
                self.operate_code_id = 123
                self.generate_and_send_status(500)
                return
            self.robot_msg_gen(self.navi_topic_feedback_msg, self.oper_topic_feedback_msg)


    def dms_callback(self, operate_datas):
        with self.callback_fun_mutex:
            logger.info("接收到DMS信息")
            time.sleep(1)
            self.DMS_msg = operate_datas


            if self.operate_code_id == self.DMS_msg["id"]:
                logger.info("重复id, 不执行！！！")
                return
            else:
                self.operate_code_id = self.DMS_msg["id"]

            if "query" in self.DMS_msg:
                logger.info("DMS问询")
                self.navi_serve_callback()
                self.oper_serve_callback()
                self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_serve_feedback_msg)
                #todo response

                logger.info("成功答复")
                return

            cnt = 0
            while self.robot_heartbeat_msg["status"] != "IDLE":
                cnt += 1
                if cnt % 100 == 0:
                    print(f"等待机器人空闲 {cnt / 100} s, now_status: {self.robot_heartbeat_msg['status']}")
                if cnt >= 1000:
                    print("等待机器人空闲超时")
                    return
                time.sleep(0.01)

            logger.info("开始操作指令：")
            logger.info(self.DMS_msg)

            try:
                # 检查当前 AGV 状态
                tmp_platform_status = self.navi_serve_callback(return_flage=1)
                logger.info("机器人当前状态：")
                logger.info(self.navi_serve_callback_only_station())

                # 遍历 "param" 中的所有操作
                for idx, operations in enumerate(self.DMS_msg["param"]):
                    logger.info(f"处理第 {idx+1} 个操作: {operations}")

                    # 临时为305夹取样本架新增跳出循环
                    # if idx > 0:
                    #     print(f"为了305电极催化实验，发送的是取离心管，实际执行取的离心管架，所以跳过后续去离心管的操作, idx: {idx}")
                    #     break
                    # 确定目标工作站，主打一个猜

                    cur_destination = self.determine_destination(operations)
                    logger.info("确定的当前 destination=" + cur_destination)

                    cur_operation = self.process_operation(operations)
                    logger.info("确定的当前 operation=" + cur_operation)

                    if cur_destination == "error":
                        self.robot_msg["id"] = self.DMS_msg["id"]
                        self.robot_msg["state"] = "error"
                        self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 无效的工作站判断"
                        return

                    if cur_destination and cur_destination not in self.station_mapping:
                        logger.error(f"未知的工作站: {cur_destination}")
                        self.robot_msg["id"] = self.DMS_msg["id"]
                        self.robot_msg["state"] = "error"
                        self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 未知的工作站 {cur_destination}"
                        return

                    if cur_operation == "error":
                        self.robot_msg["id"] = self.DMS_msg["id"]
                        self.robot_msg["state"] = "error"
                        self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 缺少必要键"
                        return

                    # 使用当前的 destination 和 operation 构建机械臂的操作消息
                    self.oper_msg_gen(cur_destination, cur_operation)

                    self.now_workstation = self.navi_serve_callback_only_station()
                    print("机器人当前站点：" + self.now_workstation)
                    logger.info("机器人当前站点：" + self.now_workstation)

                    # SPEED: 正常指令操作
                    # 如果当前站点就是所需工作站点,继续执行动作
                    if cur_destination == self.now_workstation or cur_destination is None:
                        # self.LockBase()
                        logger.info("当前站点就是所需工作站点, 继续执行动作")

                        if cur_operation == "":
                            logger.info("无机械臂目标点, 无需复位")
                            self.oper_topic_feedback_msg["state"] = "done"
                            self.navi_serve_callback()

                            self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
                            return

                        if "check" == cur_operation:
                            self.oper_topic_feedback_msg["id"] = self.DMS_msg["id"]
                            self.oper_topic_feedback_msg["state"] = "done"
                            self.robot_msg_gen(self.navi_topic_feedback_msg, self.oper_topic_feedback_msg)
                            return
                        
                        self.oper_msg_gen(cur_destination, cur_operation)
                        self.OperAction(self.oper_msg, generate_msg_flag=True)
                        print("start move robot, operation: cur_operation")

                        if self.oper_topic_feedback_msg["state"] != "done":
                            self.robot_msg["id"] = self.DMS_msg["id"]
                            self.robot_msg["state"] = "error"
                            self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作失败: {json.dumps(operations)}"
                            self.generate_and_send_status(500)
                            return

                    else:  # 如果当前站点不是所需站点，需要移动至所需工作站再进行操作,先复位
                        # 机械臂复位
                        # self.LockBase()self.oper_reset_msg["destination"]

                        logger.info("1. 机械臂复位")
                        self.oper_reset_msg["destination"] = self.now_workstation
                        self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                        if self.oper_topic_feedback_msg["state"] == "interrupt":
                            logger.info("中断返回")
                            self.operate_code_id = 123
                            self.generate_and_send_status(500)
                            return

                        if self.oper_topic_feedback_msg["state"] != "done":
                            self.robot_msg["id"] = self.DMS_msg["id"]
                            self.robot_msg["state"] = "error"
                            self.robot_msg["detail"]["oper"] = "移动前机械臂复位失败"
                            logging("Error id: "+ self.robot_msg["id"] + ", " + self.robot_msg["detail"]["oper"])
                            self.generate_and_send_status(500)
                            return


                        # 机械臂复位成功后底盘移动
                        logger.info("2. 底盘移动")
                        logger.info(f"oper state: {self.oper_topic_feedback_msg['state']}")
                        if self.oper_topic_feedback_msg["state"] == "done":

                            pubflag = False
                            while not pubflag:
                                pubflag, feedback_msg = self.PubAgvTask(operations["operation"], cur_destination)
                                logger.info("等待发布任务成功")
                                time.sleep(1)

                            localtime = time.asctime(time.localtime(time.time()))

                            if pubflag:
                                logger.info(
                                    "@本地时间:"
                                    + localtime
                                    + " 发布: 移动至站点: "
                                    + cur_destination
                                    + "任务成功"
                                )

                                taskflag, self.navi_task_feedback_msg = self.TaskPeriod(feedback_msg)

                                if taskflag:
                                    logger.info("执行" + " 移动至站点: " + cur_destination + "任务成功")
                                    time.sleep(3)  # 空余点时间给底盘计算使用

                                    with self.station_name_mutex:
                                        self.now_workstation = cur_destination

                                    # 移动后机械臂复位
                                    self.oper_reset_msg["destination"] = cur_destination
                                    self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

                                    if self.oper_topic_feedback_msg["state"] == "interrupt":
                                        logger.info("中断返回")
                                        self.operate_code_id = 123
                                        self.generate_and_send_status(500)
                                        return

                                    if self.oper_topic_feedback_msg["state"] == "done":
                                        # 机械臂重定位
                                        self.oper_relocation_msg["destination"] = cur_destination
                                        self.OperAction(self.oper_relocation_msg, generate_msg_flag=False)
                                        # self.TryRelocation(relocation_number=1)

                                        if self.oper_topic_feedback_msg["state"] == "interrupt":
                                            logger.info("中断返回")
                                            self.operate_code_id = 123
                                            self.generate_and_send_status(500)
                                            return

                                        if self.oper_topic_feedback_msg["state"] == "done":

                                            self.oper_msg_gen(cur_destination, cur_operation)
                                            print_function_name_and_lineno()
                                            print(f"cur_destination: {cur_destination}, cur_operation: {cur_operation}")
                                            logger.info("机械臂具体操作指令%r", self.oper_msg)    
                                            self.OperAction(self.oper_msg, generate_msg_flag=True)
                                            print("nav success start move robot, operation: cur_operation")

                                            if self.oper_topic_feedback_msg["state"] == "interrupt":
                                                logger.info("中断返回")
                                                self.operate_code_id = 123
                                                self.generate_and_send_status(500)
                                                return

                                            if self.oper_topic_feedback_msg["state"] != "done":
                                                self.robot_msg["id"] = self.DMS_msg["id"]
                                                self.robot_msg["state"] = "error"
                                                self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作失败: {json.dumps(operations)}"
                                                logging("Error id: "+ self.robot_msg["id"] + ", "+self.robot_msg["detail"]["oper"])
                                                self.generate_and_send_status(500)
                                                return
                                        else:
                                            self.robot_msg["id"] = self.DMS_msg["id"]
                                            self.robot_msg["state"] = "error"
                                            self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 重定位失败"
                                            logging("Error id: "+ self.robot_msg["id"] + ", "+self.robot_msg["detail"]["oper"])
                                            self.generate_and_send_status(500)
                                            return
                                    else:
                                        self.robot_msg["id"] = self.DMS_msg["id"]
                                        self.robot_msg["state"] = "error"
                                        self.robot_msg["detail"]["oper"] = "移动后机械臂复位失败"
                                        logging("Error id: "+ self.robot_msg["id"] + ", "+self.robot_msg["detail"]["oper"])
                                        self.generate_and_send_status(500)
                                        return
                                else:
                                    self.robot_msg["id"] = self.DMS_msg["id"]
                                    self.robot_msg["state"] = "error"
                                    self.robot_msg["detail"]["navi"] = "底盘移动任务失败"
                                    logging("Error id: "+ self.robot_msg["id"] + ", "+self.robot_msg["detail"]["oper"])
                                    self.generate_and_send_status(500)
                                    return

                    # 所有操作完成后复位并上报成功
                    self.oper_reset_msg["destination"] = self.now_workstation
                    self.OperAction(self.oper_reset_msg, generate_msg_flag=False)
                    # self.UnlockBase()
                    self.now_workstation = "charge_station"
                    
                    self.robot_msg["id"] = self.DMS_msg["id"]
                    self.robot_msg["state"] = "done"
                    self.robot_msg["detail"]["oper"] = "所有操作完成"
                    logging("id: "+ self.robot_msg["id"] + ", "+self.robot_msg["detail"]["oper"])
                    # self.generate_and_send_status(200)

            except rospy.ROSInterruptException:
                logger.info("ROSInterruptException")
                pass

    
    def oper_msg_gen(self, cur_dest, cur_oper):
        self.oper_msg["id"] = self.DMS_msg["id"]
        self.oper_msg["stamp"] = self.DMS_msg["stamp"]
        self.oper_msg["destination"] = cur_dest
        operations = [{"operation": cur_oper}]
        self.oper_msg["operations"] = operations
        self.oper_reset_msg["id"] = self.DMS_msg["id"]
        self.oper_relocation_msg["id"] = self.DMS_msg["id"]

    def GetBaseLockMsg(self):
        # self.baselock_msg["status"] = self.hik_flask_sever.agv_status
        self.baselock_msg["status"] = "IDLE"
        return self.baselock_msg


    def SendHreartbeatMsg(self):

        with self.station_name_mutex:
            self.robot_heartbeat_msg["currentStation"] = self.now_workstation

        agv_info_datas = self.navi_serve_callback(1)

        self.robot_heartbeat_msg["electricityQuantity"] = agv_info_datas["battery"]
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
            # logger.info("DMS 心跳回复:")
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
            logger.info(dresp)
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
        logger.info(self.hik_system)

        try:
            resp = requests.post(
                "http://192.168.200.31:8182/rcms-dps/rest/queryAgvStatus",
                data=jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15),
            )
        
        except Exception as e:
            logger.info()
            return None
        
        raw_info = {}
        if resp and resp.status_code == 200:
            return_flage = 1
            raw_info = json.loads(resp.text)
        else:
            logger.info(f"Failed to fetch AGV status: {resp.status_code}")
            return None
        
        #提取指定AGV的信息
        agv_info=self.agvlist.ExtractAgvInfo(raw_info)
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
            self.robot_msg["stamp"] = str(int(round(time.time() * 1000)))
            self.robot_msg["state"] = oper["state"]
            logger.info("%s - 操作ID: %s", datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"), oper["id"])
        else:
            self.robot_msg["id"] = navi["id"]   
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

    def GetStatusAlarmMsg(self):
        self.robot_heartbeat_msg['workstation'] = self.now_workstation
        self.robot_heartbeat_msg["status"] = self.robot_msg["state"]
        return self.robot_heartbeat_msg

    def update_status(self):  # 5秒一次的心跳监听
        rate = rospy.Rate(10)
        cnt = 0
        while not rospy.is_shutdown():
            if cnt % 30 == 0:
                pinged, unpinged = rosnode.rosnode_ping_all()
                if unpinged:
                    master = rosnode.rosgraph.Master(rosnode.ID)
                    rosnode.cleanup_master_blacklist(master, unpinged)

                now_node_list = rosnode.get_node_names()

                if len(list(self.node_list.keys())) > len(set(now_node_list)):
                    inactive_node = ""
                    for i in self.node_list.keys():
                        if i in now_node_list:
                            pass
                        else:
                            inactive_node += self.node_list[i]
                            logger.info(i + "节点异常")
                            # self.voide_broadcast_pub.publish(String(i + "节点异常"))
                    self.robot_heartbeat_msg["status"] = "ERROR"
                    self.robot_heartbeat_msg["detailMsg"] = inactive_node + "节点异常"

            if cnt == 10 or (cnt > 10 and cnt % 300 == 0):
                if self.robot_heartbeat_msg["electricityQuantity"] > 60:
                    logger.info(f"当前电量：%s", self.robot_heartbeat_msg["electricityQuantity"])
                elif self.robot_heartbeat_msg["electricityQuantity"] > 35:
                    logger.warning(f"当前电量：%s", self.robot_heartbeat_msg["electricityQuantity"])
                elif self.robot_heartbeat_msg["electricityQuantity"] > 0:
                    logger.error(f"当前电量：%s", self.robot_heartbeat_msg["electricityQuantity"])

            if cnt > 10:
                # 更新状态
                self.query_msg["stamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                self.navi_serve_callback()
                self.oper_serve_callback()

                robot_details = ""
                platform_status = self.navi_serve_feedback_msg["state"]
                arm_status = self.oper_serve_feedback_msg["state"]
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

                # logger.error(f"plat: {platform_status}, arm: {arm_status}, status: {self.robot_heartbeat_msg['status']}")
                self.robot_heartbeat_msg["detailMsg"] = robot_details

            # sleep
            cnt += 1
            rate.sleep()

    def heartbeat_monitoring(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.SendHreartbeatMsg()
            rate.sleep()

    def DmsFlaskHttpSever(self):
        self.robot_flask_server.RunServer()

    #vic new add
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
        # print(f"determine_destination source: {source}")
        target = operations.get("target", {})
        # print(f"determine_destination target: {target}")
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
        
    def parse_positions(self, positions):
        '''解析多个 'xxx_position' 和对应的值，并拼接成字符串'''
        print_function_name_and_lineno
        parsed_positions = []
        for key, value in positions.items():
            if key.endswith('_position'):
                parsed_positions.append(f"{key[:-9]}_{value}")  #不带'_position'
        return '-'.join(parsed_positions)


    def build_operation_string(self, origin_datas, robot_code):
        '''根据不同的情况构建新的操作字符串'''
        print_function_name_and_lineno()
        print(f"build_operation_string origin_datas: {origin_datas}")
        source = origin_datas.get("source", {})
        print(f"build_operation_string source: {source}")
        target = origin_datas.get("target", {})
        print(f"build_operation_string target: {target}")
        source_workstation = source.get("workstation")
        target_workstation = target.get("workstation")
        source_positions = self.parse_positions(source)
        print(f"build_operation_string source_positions: {source_positions}")
        target_positions = self.parse_positions(target)
        print(f"build_operation_string target_positions: {target_positions}")
        print_function_name_and_lineno()

        if source_workstation == robot_code and target_workstation != robot_code:
            return f"move-r_{robot_code}-to-w_{target_workstation}-oper-{source_positions}-and-{target_positions}"
        elif target_workstation == robot_code and source_workstation != robot_code:
            return f"move-w_{source_workstation}-to-r_{robot_code}-oper-{source_positions}-and-{target_positions}"
        elif target_workstation != robot_code and source_workstation != robot_code:
            return f"move-w_{source_workstation}-to-w_{target_workstation}-oper-{source_positions}-and-{target_positions}"
        elif target_workstation == robot_code and source_workstation == robot_code:
            return f"move-r_{robot_code}-to-r_{robot_code}-oper-{source_positions}-and-{target_positions}"
        else:
            return "Invalid condition"
        
    def extract_workstation_and_operation(self, operation_string, robot_code):
        '''从新的操作字符串中提取工作站和操作'''
        parts = operation_string.split('-')
    
        # 确定工作站的索引
        if parts[2] == 'to':
            source_workstation = parts[1][2:]
            target_workstation = parts[3][2:]
            print(f"get source_workstation: {source_workstation}, target_workstation: {target_workstation}")
        else:
            source_workstation = parts[1][2:]
            target_workstation = parts[2][2:]
            print(f"get source_workstation: {source_workstation}, target_workstation: {target_workstation}")
    
        # 根据新的逻辑获取工作站的值
        if source_workstation == robot_code and target_workstation != robot_code:
            workstation = target_workstation
        elif target_workstation == robot_code and source_workstation != robot_code:
            workstation = source_workstation
        elif source_workstation == robot_code and target_workstation == robot_code:
            workstation = target_workstation 
        else:
            workstation = source_workstation if parts[2] == 'to' else target_workstation            
  
        # 获取操作字符串
        operation = operation_string
    
        return workstation, operation

    def get_specific_operation(self, workstation_operations, workstation, operation):
        '''根据工作站和操作获取具体的操作字符串'''
        print(f"workstation: {workstation}, operation: {operation}")
        for param in workstation_operations['param']:
            if param['workstation'] == workstation:
                if operation in param['operations']:
                    return param['operations'][operation]
        return "{workstation_operations}-{operation} not found"

    def process_operation(self, operations):
        print_function_name_and_lineno()
        robot_code = self.config_param["robotCode"]
        operation_string = self.build_operation_string(operations, robot_code)
        print(f"Operation string for {operation_string}")
        if operation_string:
            workstation, operation = self.extract_workstation_and_operation(operation_string, robot_code)
            specific_operation = self.get_specific_operation(self.workstation_operations, workstation, operation)
            print(f"Specific operation for : {specific_operation}")

        return specific_operation
    
    def generate_and_send_status(self, code=200):
        if code==500:
            msg = "Error"
        else:
            msg = "Success"
        status_msg = {
            "id": self.DMS_msg.get("id"),
            "stamp": str(int(round(time.time() * 1000))),
            "code": code,       
            "message": msg,
            "isRobot": True,
            "current_workstation": self.now_workstation,
            "oper_state": self.oper_topic_feedback_msg.get("state", "idle"),
            "nav_state": self.navi_topic_feedback_msg.get("state", "idle")
        }
        try:
            response = requests.post(
                self.response_url,
                json=status_msg,
                headers=self.headers,
                timeout=(60, 15),
            )
            logger.info(f"状态发送成功: {status_msg}")
        except Exception as e:
            logger.info(f"状态发送失败: {e}")


if __name__ == "__main__":

    logger = Log(__name__).getlog()

    try:
        rospy.init_node("robot_msg_manager_node", anonymous=True, disable_signals=True)
        manager = RobotMsgManager()
        logger.info("initial node")

        threading.Thread(target=manager.heartbeat_monitoring).start()
        threading.Thread(target=manager.update_status).start()
        threading.Thread(target=manager.DmsFlaskHttpSever).start()

        rospy.spin()

    except:
        logger.critical(str(traceback.format_exc()))
