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
    """
    机器人消息管理器
    负责DMS指令处理、AGV导航控制、机械臂操作协调
    """

    # ==================== 常量定义 ====================
    MAX_NAV_PUB_RETRIES = 10        # 发布导航任务最大重试次数
    MAX_NAV_RETRY_RETRIES = 3       # 重新导航时发布任务重试次数
    MAX_POSITION_VERIFY_RETRIES = 5 # 位置验证最大重试次数
    NAV_SETTLE_TIME = 3             # 导航后等待时间（秒）

    def __init__(self):

        self.hik_system = rospy.get_param('~hiksystem', 'rcs_2000')

        # 加载配置文件（带详细错误信息）
        package_path = rospkg.RosPack().get_path("robot_message_bridge")

        # 1. 加载主参数配置
        param_file_path = package_path + "/config/param.json"
        try:
            with open(param_file_path, encoding="utf-8") as f:
                self.config_param = json.load(f)
            logger.info(f"配置文件加载成功: {param_file_path}")
        except FileNotFoundError:
            logger.error(f"配置文件不存在: {param_file_path}")
            raise
        except json.JSONDecodeError as e:
            logger.error(f"配置文件JSON格式错误: {param_file_path}, 错误: {e}")
            raise

        # 2. 加载工作站操作映射配置
        wk_operation_file_path = package_path + "/config/workstataion_operations.json"
        try:
            with open(wk_operation_file_path, encoding="utf-8") as f:
                self.workstation_operations = json.load(f)
            logger.info(f"工作站操作配置加载成功: {wk_operation_file_path}")
        except FileNotFoundError:
            logger.error(f"工作站操作配置文件不存在: {wk_operation_file_path}")
            raise
        except json.JSONDecodeError as e:
            logger.error(f"工作站操作配置JSON格式错误: {wk_operation_file_path}, 错误: {e}")
            raise

        # 3. 加载站点映射配置
        station_mapping_file_path = package_path + "/config/station_mapping.json"
        try:
            with open(station_mapping_file_path, encoding="utf-8") as f:
                self.station_mapping = json.load(f)
            logger.info(f"站点映射配置加载成功: {station_mapping_file_path}")
        except FileNotFoundError:
            logger.error(f"站点映射配置文件不存在: {station_mapping_file_path}")
            raise
        except json.JSONDecodeError as e:
            logger.error(f"站点映射配置JSON格式错误: {station_mapping_file_path}, 错误: {e}")
            raise

        # 4. 加载AGV状态映射配置
        status_file_path = package_path + "/config/status.json"
        try:
            with open(status_file_path, encoding="utf-8") as f:
                self.agv_status = json.load(f)
            logger.info(f"AGV状态配置加载成功: {status_file_path}")
        except FileNotFoundError:
            logger.error(f"AGV状态配置文件不存在: {status_file_path}")
            raise
        except json.JSONDecodeError as e:
            logger.error(f"AGV状态配置JSON格式错误: {status_file_path}, 错误: {e}")
            raise

        # 5. 加载站点坐标配置
        workstation_plot_file_path = package_path + "/config/workstation_plot.json"
        try:
            with open(workstation_plot_file_path, encoding="utf-8") as f:
                self.workstation_plot = json.load(f)
            logger.info(f"站点坐标配置加载成功: {workstation_plot_file_path}")
        except FileNotFoundError:
            logger.error(f"站点坐标配置文件不存在: {workstation_plot_file_path}")
            raise
        except json.JSONDecodeError as e:
            logger.error(f"站点坐标配置JSON格式错误: {workstation_plot_file_path}, 错误: {e}")
            raise


        self.headers = {"content-type": "application/json"}  # DMS接收的header
        self.Operation_topic_node_name = "/obsOperation_out"
        self.Operation_serve_node_name ="/robot"

        self.node_list = {
            "/aruco_to_point_and_talk": "相机节点",
            "/chemicalrobot_arm_new": "机械臂",
        }

        self.statusFeedbackCode = 200
        self.operate_code_id = 123
        self.AGV_url = self.config_param["AGV_url"]
        self.AGV_url2 = self.config_param["AGV_url2"]

        # 加载电量阈值配置
        self.minimum_battery_threshold = self.config_param["minimum_battery_threshold"]
        self.medium_battery_threshold = self.config_param["medium_battery_threshold"]
        self.high_battery_threshold = self.config_param["high_battery_threshold"]

        # 验证阈值大小关系：minimum <= medium <= high
        if not (self.minimum_battery_threshold <= self.medium_battery_threshold <= self.high_battery_threshold):
            logger.error(f"电量阈值配置错误: minimum({self.minimum_battery_threshold}) <= medium({self.medium_battery_threshold}) <= high({self.high_battery_threshold}) 不成立")
            logger.error("将使用默认值: minimum=50, medium=70, high=90")
            self.minimum_battery_threshold = 50
            self.medium_battery_threshold = 70
            self.high_battery_threshold = 90

        # 检测是否为简化模式（三个阈值相等）
        self.is_simple_charge_mode = (self.minimum_battery_threshold == self.medium_battery_threshold == self.high_battery_threshold)
        if self.is_simple_charge_mode:
            logger.info(f"充电策略: 简化模式（阈值={self.minimum_battery_threshold}%）")
            logger.info(f"  - 低于{self.minimum_battery_threshold}%且空闲时充电")
            logger.info(f"  - 达到{self.minimum_battery_threshold}%后取消充电")
        else:
            logger.info(f"充电策略: 三档模式")
            logger.info(f"  - 低电量(<{self.minimum_battery_threshold}%): 强制充电，拒绝任务")
            logger.info(f"  - 中电量({self.minimum_battery_threshold}-{self.high_battery_threshold}%): 空闲时充电")
            logger.info(f"  - 高电量(>={self.high_battery_threshold}%): 取消充电")

        self.idle_time_before_charge = self.config_param["idle_time_before_charge"]
        self.obstacle_avoidance_feedback_url = self.config_param["ObstacleAvoidanceFeebback"]

        # 充电管理状态变量
        self.last_task_time = time.time()  # 上次任务完成时间
        self.is_in_low_battery_mode = False  # 是否处于低电量模式
        self.is_charging_to_full = False  # 是否正在充电且等待充满
        self.charge_command_sent_time = None  # 发送充电命令的时间
        self.charge_retry_count = 0  # 充电重试次数
        self.max_charge_retries = 3  # 最大充电重试次数
        self.charge_wait_timeout = 300  # 等待充电开始的超时时间（5分钟）

        # 状态锁变量（防止状态抖动）
        self.idle_start_time = None  # 开始检测到IDLE的时间
        self.idle_stable_duration = 5  # IDLE状态稳定期（秒）
        self.last_stable_status = "IDLE"  # 上次稳定的状态

        self.navi_topic_feedback_msg = dict()
        self.oper_topic_feedback_msg = dict()
        self.navi_serve_feedback_msg = dict()
        self.oper_serve_feedback_msg = dict()
        self.DMS_msg = dict()

        self.taskCode = ""

        self.robot_heartbeat_msg = {
            "identifyingCode": self.config_param["identifyingCode"],
            "status": "IDLE",  # 初始状态与 last_stable_status 保持同步
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
        self.last_battery_level = -1  # 上次电量值，用于检测电量变化

        self.station_name_mutex = threading.Lock()
        self.callback_fun_mutex = threading.Lock()

        self.InitMessage()

        #启动程序立即释放底盘
        logger.info("start dms first pubfreerobot")
        self.PubFreeRobot()

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
             #释放底盘
            self.PubFreeRobot()
            logger.info("invoke pubfreerobot end!")

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

    # ==================== 辅助方法：错误处理 ====================

    def _set_error_and_return(self, error_msg, error_type="oper", code=500):
        """
        统一的错误设置和状态发送方法

        参数:
            error_msg: 错误信息
            error_type: 错误类型 ("oper" 或 "navi")
            code: HTTP状态码
        """
        self.robot_msg["id"] = self.DMS_msg["id"]
        self.robot_msg["state"] = "error"
        self.robot_msg["detail"][error_type] = error_msg
        logger.error(f"Error id: {self.robot_msg['id']}, {error_msg}")
        self.generate_and_send_status(code)

    def _handle_interrupt(self):
        """处理中断状态"""
        logger.info("中断返回")
        self.operate_code_id = 123
        self.generate_and_send_status(500)

    def _check_arm_state_with_interrupt(self):
        """
        检查机械臂状态，处理中断情况

        返回:
            (bool, str): (是否成功, 状态描述)
            - (True, "done"): 操作成功
            - (False, "interrupt"): 被中断
            - (False, "error"): 操作失败
        """
        state = self.oper_topic_feedback_msg["state"]
        if state == "interrupt":
            self._handle_interrupt()
            return False, "interrupt"
        elif state == "done":
            return True, "done"
        else:
            return False, "error"

    # ==================== 辅助方法：导航相关 ====================

    def _pub_agv_task_with_retry(self, operation, destination, max_retries=None):
        """
        带重试的AGV任务发布

        返回:
            (bool, dict): (是否成功, 反馈消息)
        """
        if max_retries is None:
            max_retries = self.MAX_NAV_PUB_RETRIES

        pubflag = False
        pub_attempt = 0
        feedback_msg = None

        while not pubflag and pub_attempt < max_retries:
            pubflag, feedback_msg = self.PubAgvTask(operation, destination)
            if not pubflag:
                pub_attempt += 1
                logger.warning(f"发布导航任务失败，重试 {pub_attempt}/{max_retries}")
                time.sleep(1)

        return pubflag, feedback_msg

    def _verify_position_with_retry(self, destination, operations):
        """
        带重试的位置验证（包含重新导航逻辑）

        返回:
            bool: 是否验证成功
        """
        retry_count = 0
        is_arrival_valid = False

        while retry_count < self.MAX_POSITION_VERIFY_RETRIES:
            is_arrival_valid, arrival_info = self.check_position_match(destination)

            if is_arrival_valid:
                logger.info(f"坐标验证成功，继续执行操作")
                return True

            # 检查失败类型，决定是否重试
            error_type = arrival_info.get("error_type", "UNKNOWN")

            # 配置缺失或异常错误不应重试导航
            if error_type in ["MISSING_CONFIG", "EXCEPTION"]:
                logger.error(f"坐标验证失败且无法通过重试解决: {arrival_info.get('reason', error_type)}")
                return False

            # API错误可能是临时的，但重试次数应该有限
            if error_type == "API_ERROR":
                retry_count += 1
                logger.warning(f"API获取坐标失败 (第{retry_count}次)，等待后重试...")
                time.sleep(5)  # API错误只等待，不重新导航
                continue

            # 坐标不匹配，重试导航
            retry_count += 1
            logger.warning(f"导航后坐标验证失败 (第{retry_count}次): {arrival_info}")

            if retry_count >= self.MAX_POSITION_VERIFY_RETRIES:
                logger.error(f"已达到最大重试次数({self.MAX_POSITION_VERIFY_RETRIES})，坐标仍然不匹配")
                return False

            logger.info(f"尝试重新导航到{destination} (第{retry_count+1}/{self.MAX_POSITION_VERIFY_RETRIES}次)")

            # 重新发布导航任务
            pubflag_retry, feedback_msg_retry = self._pub_agv_task_with_retry(
                operations["operation"], destination, self.MAX_NAV_RETRY_RETRIES
            )

            if not pubflag_retry:
                logger.error(f"无法发布重新导航任务")
                return False

            taskflag_retry, _ = self.TaskPeriod(feedback_msg_retry)
            if not taskflag_retry:
                logger.error(f"重新导航到{destination}失败")
                return False

            logger.info(f"重新导航到{destination}成功")
            time.sleep(self.NAV_SETTLE_TIME)

        return False

    # ==================== 辅助方法：操作执行 ====================

    def _execute_arm_reset(self, destination):
        """
        执行机械臂复位

        返回:
            bool: 是否成功
        """
        self.oper_reset_msg["destination"] = destination
        self.OperAction(self.oper_reset_msg, generate_msg_flag=False)

        success, state = self._check_arm_state_with_interrupt()
        return success

    def _execute_arm_relocation(self, destination):
        """
        执行机械臂重定位

        返回:
            bool: 是否成功
        """
        self.oper_relocation_msg["destination"] = destination
        self.OperAction(self.oper_relocation_msg, generate_msg_flag=False)

        success, state = self._check_arm_state_with_interrupt()
        return success

    def _execute_arm_operation(self, destination, operation):
        """
        执行机械臂具体操作

        返回:
            bool: 是否成功
        """
        self.oper_msg_gen(destination, operation)
        logger.info("机械臂具体操作指令%r", self.oper_msg)
        self.OperAction(self.oper_msg, generate_msg_flag=False)

        success, state = self._check_arm_state_with_interrupt()
        return success

    # ==================== 核心业务方法 ====================

    def _handle_battery_check(self):
        """
        处理电量检查逻辑

        返回:
            bool: 是否可以继续执行任务
        """
        current_battery = float(self.robot_heartbeat_msg["electricityQuantity"])

        # ========== 简化模式：不拒绝任务，只是接单前取消充电 ==========
        if self.is_simple_charge_mode:
            agv_status_info = self.navi_serve_callback(1)
            if agv_status_info and agv_status_info.get("status") == "Charging_status":
                logger.info(f"简化模式：当前电量{current_battery}%，接受任务前取消充电")
                self.PubAgvUnChargeTask()
                time.sleep(2)
            return True

        # ========== 三档模式：低电量拒绝任务 ==========
        # 低电量模式（<minimum）：直接拒绝任务，不阻塞等待
        if current_battery < self.minimum_battery_threshold:
            logger.warning(f"电量{current_battery}%低于{self.minimum_battery_threshold}%，拒绝任务")
            self.is_in_low_battery_mode = True
            
            # 设置错误状态
            self.robot_msg["state"] = "ERROR"
            self.robot_msg["detail"]["oper"] = f"电量不足({current_battery}%)，请等待充电完成"
            
            # 返回错误给DMS
            self.generate_and_send_status(500)
            
            # 触发充电（后台处理，由 _manage_charging 在 update_status 线程中持续管理）
            self.PubAgvChargeTask()
            
            logger.info("任务已拒绝，充电由后台自动管理，等待电量恢复后可接受新任务")
            return False  # 返回False表示不能继续执行任务

        # 中高电量（minimum-high）：接受任务，如果正在充电则取消充电
        elif current_battery < self.high_battery_threshold:
            agv_status_info = self.navi_serve_callback(1)
            if agv_status_info and agv_status_info.get("status") == "Charging_status":
                logger.info(f"当前电量{current_battery}%，正在充电中，接受任务前先取消充电")
                self.PubAgvUnChargeTask()
                time.sleep(2)

        # 高电量（>=high）：正常接受任务（无需特殊处理）

        return True

    def _execute_operation_in_place(self, idx, operations, cur_destination, cur_operation):
        """
        在当前位置执行操作（无需导航）

        返回:
            bool: 是否成功
        """
        logger.info(f"当前站点验证通过，继续执行动作")

        if cur_operation == "":
            logger.info("无机械臂目标点, 无需复位")
            self.oper_topic_feedback_msg["state"] = "done"
            self.navi_serve_callback()
            self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_topic_feedback_msg)
            return True

        if cur_operation == "check":
            self.oper_topic_feedback_msg["id"] = self.DMS_msg["id"]
            self.oper_topic_feedback_msg["state"] = "done"
            self.robot_msg_gen(self.navi_topic_feedback_msg, self.oper_topic_feedback_msg)
            return True

        self.oper_msg_gen(cur_destination, cur_operation)
        self.OperAction(self.oper_msg, generate_msg_flag=False)
        print("start move robot, operation: cur_operation")

        if self.oper_topic_feedback_msg["state"] != "done":
            self._set_error_and_return(f"第 {idx+1} 个操作失败: {json.dumps(operations)}")
            return False

        return True

    def _execute_operation_with_navigation(self, idx, operations, cur_destination, cur_operation, position_info):
        """
        需要导航后执行操作

        返回:
            bool: 是否成功
        """
        # 记录坐标不匹配的情况
        now_workstation_code = (cur_destination == self.now_workstation)
        if now_workstation_code:
            logger.warning(f"站点代码匹配但坐标不匹配: 站点={cur_destination}, 触发重新导航")
            logger.warning(f"  坐标验证详情: {position_info}")

        # ===== 步骤1: 机械臂复位 =====
        logger.info("1. 机械臂复位")
        if not self._execute_arm_reset(self.now_workstation):
            if self.oper_topic_feedback_msg["state"] != "interrupt":
                self._set_error_and_return("移动前机械臂复位失败")
            return False

        # ===== 步骤2: 发布导航任务 =====
        logger.info("2. 底盘移动")
        logger.info(f"oper state: {self.oper_topic_feedback_msg['state']}")

        pubflag, feedback_msg = self._pub_agv_task_with_retry(operations["operation"], cur_destination)
        localtime = time.asctime(time.localtime(time.time()))

        if not pubflag:
            logger.error(f"发布导航任务失败，已重试{self.MAX_NAV_PUB_RETRIES}次，放弃任务")
            self._set_error_and_return(f"发布导航任务失败，已重试{self.MAX_NAV_PUB_RETRIES}次", "navi")
            return False

        logger.info(f"@本地时间:{localtime} 发布: 移动至站点: {cur_destination}任务成功")

        # ===== 步骤3: 等待导航完成 =====
        taskflag, self.navi_task_feedback_msg = self.TaskPeriod(feedback_msg)

        if not taskflag:
            self._set_error_and_return("底盘移动任务失败", "navi")
            return False

        logger.info(f"执行 移动至站点: {cur_destination}任务成功")
        time.sleep(self.NAV_SETTLE_TIME)

        with self.station_name_mutex:
            self.now_workstation = cur_destination

        # ===== 步骤4: 验证到达位置 =====
        logger.info("导航完成，验证到达位置是否正确...")
        if not self._verify_position_with_retry(cur_destination, operations):
            self._set_error_and_return(
                f"导航到{cur_destination}后坐标验证失败，已重试{self.MAX_POSITION_VERIFY_RETRIES}次",
                "navi"
            )
            return False

        # ===== 步骤5: 移动后机械臂复位 =====
        if not self._execute_arm_reset(cur_destination):
            if self.oper_topic_feedback_msg["state"] != "interrupt":
                self._set_error_and_return("移动后机械臂复位失败")
            return False

        # ===== 步骤6: 机械臂重定位 =====
        if not self._execute_arm_relocation(cur_destination):
            if self.oper_topic_feedback_msg["state"] != "interrupt":
                self._set_error_and_return(f"第 {idx+1} 个操作: 重定位失败")
            return False

        # ===== 步骤7: 执行机械臂操作 =====
        print(f"cur_destination: {cur_destination}, cur_operation: {cur_operation}")
        if not self._execute_arm_operation(cur_destination, cur_operation):
            if self.oper_topic_feedback_msg["state"] != "interrupt":
                self._set_error_and_return(f"第 {idx+1} 个操作失败: {json.dumps(operations)}")
            return False

        print("nav success start move robot, operation: cur_operation")
        return True

    def _process_single_operation(self, idx, operations):
        """
        处理单个操作

        返回:
            (bool, bool): (是否成功, 是否需要提前返回)
        """
        logger.info(f"处理第 {idx+1} 个操作: {operations}")

        cur_destination = self.determine_destination(operations)
        logger.info("确定的当前 destination=" + str(cur_destination))

        cur_operation = self.process_operation(operations)
        logger.info("确定的当前 operation=" + cur_operation)

        # 验证目标工作站
        if cur_destination == "error":
            self.robot_msg["id"] = self.DMS_msg["id"]
            self.robot_msg["state"] = "error"
            self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 无效的工作站判断"
            return False, True

        if cur_destination and cur_destination not in self.station_mapping:
            logger.error(f"未知的工作站: {cur_destination}")
            self.robot_msg["id"] = self.DMS_msg["id"]
            self.robot_msg["state"] = "error"
            self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 未知的工作站 {cur_destination}"
            return False, True

        if cur_operation == "error":
            self.robot_msg["id"] = self.DMS_msg["id"]
            self.robot_msg["state"] = "error"
            self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: 缺少必要键"
            return False, True

        # 构建机械臂操作消息
        self.oper_msg_gen(cur_destination, cur_operation)

        print("机器人当前站点：" + self.now_workstation)
        logger.info("机器人当前站点：" + self.now_workstation)

        # 检查是否需要导航
        now_workstation_code = (cur_destination == self.now_workstation)

        # 进行坐标验证
        now_workstation_true = False
        position_info = {}
        if cur_destination is not None:
            is_position_match, position_info = self.check_position_match(cur_destination)
            now_workstation_true = is_position_match
            logger.info(f"坐标验证结果: {position_info}")

            # 如果坐标配置缺失，报错退出
            if not is_position_match and position_info.get("error_type") == "MISSING_CONFIG":
                self.robot_msg["id"] = self.DMS_msg["id"]
                self.robot_msg["state"] = "error"
                self.robot_msg["detail"]["oper"] = f"第 {idx+1} 个操作: {position_info.get('reason')}"
                self.generate_and_send_status(500)
                return False, True
        else:
            now_workstation_true = True

        # 根据是否需要导航选择执行路径
        if (now_workstation_code and now_workstation_true) or cur_destination is None:
            # 无需导航，在当前位置执行
            logger.info(f"当前站点验证通过 - 站点代码匹配: {now_workstation_code}, 坐标位置匹配: {now_workstation_true}, 继续执行动作")
            success = self._execute_operation_in_place(idx, operations, cur_destination, cur_operation)
            # 对于check和空操作，需要提前返回
            if cur_operation == "" or cur_operation == "check":
                return success, True
            return success, not success
        else:
            # 需要导航后执行
            success = self._execute_operation_with_navigation(
                idx, operations, cur_destination, cur_operation, position_info
            )
            return success, not success

    def dms_callback(self, operate_datas):
        """
        DMS回调函数 - 处理来自DMS的指令

        重构说明：
        1. 提取电量检查逻辑到 _handle_battery_check
        2. 提取单个操作处理逻辑到 _process_single_operation
        3. 提取原地执行逻辑到 _execute_operation_in_place
        4. 提取导航执行逻辑到 _execute_operation_with_navigation
        """
        with self.callback_fun_mutex:
            logger.info("接收到DMS信息")
            time.sleep(1)
            self.DMS_msg = operate_datas

            # 电量检查（返回False表示电量过低，任务已被拒绝）
            if not self._handle_battery_check():
                return

            # 更新任务时间戳
            self.last_task_time = time.time()

            logger.info(f"invoke dms_callback statusFeedbackCode= {self.statusFeedbackCode}")

            # 检查重复ID
            if self.operate_code_id == self.DMS_msg["id"] and self.statusFeedbackCode == 200:
                logger.warning(f" 重复id: {self.operate_code_id}, 不能执行任务！！！ 请化学家平台排查问题，并重启机器人！")
                return
            else:
                self.operate_code_id = self.DMS_msg["id"]

            # 处理查询请求
            if "query" in self.DMS_msg:
                logger.info("DMS问询")
                self.navi_serve_callback()
                self.oper_serve_callback()
                self.robot_msg_gen(self.navi_serve_feedback_msg, self.oper_serve_feedback_msg)
                logger.info("成功答复")
                return

            logger.info("开始操作指令：")
            logger.info(self.DMS_msg)

            try:
                # 检查当前 AGV 状态
                tmp_platform_status = self.navi_serve_callback(return_flage=1)
                logger.info("机器人当前状态：")
                logger.info(self.navi_serve_callback_only_station())

                # 遍历 "param" 中的所有操作
                for idx, operations in enumerate(self.DMS_msg["param"]):
                    success, should_return = self._process_single_operation(idx, operations)
                    if should_return:
                        return

                # 所有操作完成后复位并上报成功
                self.oper_reset_msg["destination"] = self.now_workstation
                self.OperAction(self.oper_reset_msg, generate_msg_flag=True)
                self.now_workstation = "charge_station"

                self.robot_msg["id"] = self.DMS_msg["id"]
                self.robot_msg["state"] = "done"
                self.robot_msg["detail"]["oper"] = "所有操作完成"
                logger.info("id: "+ self.robot_msg["id"] + ", "+self.robot_msg["detail"]["oper"])

            except rospy.ROSInterruptException:
                logger.info("ROSInterruptException")
                self.generate_and_send_status(500)
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
        self.baselock_msg["status"] = "IDLE"
        return self.baselock_msg


    def SendHreartbeatMsg(self):

        with self.station_name_mutex:
            self.robot_heartbeat_msg["currentStation"] = self.now_workstation

        # 获取AGV状态，失败时重试
        agv_info_datas = None
        max_retries = 3
        for retry in range(max_retries):
            agv_info_datas = self.navi_serve_callback(1)
            if agv_info_datas is not None:
                break
            if retry < max_retries - 1:
                logger.warning(f"获取AGV状态失败，重试 {retry + 1}/{max_retries}")
                time.sleep(1)
        
        # 重试后仍然失败，使用上次的电量值继续发送心跳
        if agv_info_datas is None:
            logger.warning("获取AGV状态失败，使用上次电量值发送心跳")
        else:
            self.robot_heartbeat_msg["electricityQuantity"] = agv_info_datas["battery"]
            current_battery = int(float(self.robot_heartbeat_msg["electricityQuantity"]))
            
            # 只在电量变化时打印日志
            if current_battery != self.last_battery_level:
                logger.info("当前机器人电量：%d%%", current_battery)
                self.last_battery_level = current_battery
            
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
            logger.error(f"发布任务请求异常: {e}")
            return False, None

        try:
            dresp = json.loads(resp.text)
        except Exception as e:
            logger.error(f"解析任务响应失败: {e}")
            return False, None

        logger.info("调度系统返回" + json.dumps(dresp))
        task = self.agvlist.CheckTaskFeedback(dresp)

        if task != "OK":
            logger.info("申请失败:\n" + task)
            return False, dresp
        else:
            logger.info("申请 " + " to " + str(navi_destination) + "成功")
            return True, dresp


    #释放底盘
    def PubFreeRobot(self):
        logger.info("invoke pubfreerobot start")
        jreq = self.agvlist.FreeRobot()
        try:
            resp = requests.post(
                self.AGV_url + "freeRobot",
                jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15)
            )
        except Exception as e:
            logger.error(f"释放底盘请求异常: {e}")
            return False, None

        try:
            dresp = json.loads(resp.text)
        except Exception as e:
            logger.error(f"解析释放底盘响应失败: {e}")
            return False, None

        task = self.agvlist.CheckTaskFeedback(dresp)
        if task != "OK":
            logger.info("底盘释放失败:\n" + task)
            return False, dresp
        else:
            logger.info("底盘已释放")
            return True, dresp

    #主动充电
    def PubAgvChargeTask(self):
        jreq = self.agvlist.GoCharge()
        try:
            resp = requests.post(
                self.AGV_url + "agvChargeTask",
                jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15)
            )
        except Exception as e:
            logger.error(f"主动调用充电接口异常: {e}")
            return False, None

        try:
            dresp = json.loads(resp.text)
        except Exception as e:
            logger.error(f"解析充电接口响应失败: {e}")
            return False, None

        task = self.agvlist.CheckTaskFeedback(dresp)
        if task != "OK":
            logger.info("主动调用充电接口失败:\n" + task)
            return False, dresp
        else:
            logger.info("主动调用充电接口成功")
            return True, dresp

    # #取消充电
    # def PubAgvUnChargeTask(self):
    #     jreq = self.agvlist.UnCharge()
    #     try:
    #         resp = requests.post(
    #             self.AGV_url + "agvChargeTask",
    #             jreq.encode("utf-8"),
    #             headers=self.headers,
    #             timeout=(60, 15)
    #         )
    #     except Exception as e:
    #         logger.error(f"调用取消充电接口异常: {e}")
    #         return False, None

    #     try:
    #         dresp = json.loads(resp.text)
    #     except Exception as e:
    #         logger.error(f"解析取消充电接口响应失败: {e}")
    #         return False, None

    #     task = self.agvlist.CheckTaskFeedback(dresp)
    #     if task != "OK":
    #         logger.info("调用取消充电接口失败:\n" + task)
    #         return False, dresp
    #     else:
    #         logger.info("调用取消充电接口成功")
    #         return True, dresp

    def PubAgvUnChargeTask(self):
        """
        取消充电：通过移动到等待点实现
        原 RCS 取消充电 API 不可用，改为移动到指定等待点
        """
        waiting_point = self.config_param.get("charge_complete_waiting_point", "waiting_station")
        logger.info(f"取消充电：移动到等待点 {waiting_point}")
        return self.PubAgvTask("move", waiting_point)

    def TaskPeriod(self, feedback_msg, waitime=60):
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
                logger.error(f"查询任务状态请求异常: {e}")
                continue

            try:
                dresp = json.loads(resp.text)
            except Exception as e:
                logger.error(f"解析任务状态响应失败: {e}")
                continue

            task_status = dresp["data"][0]["taskStatus"]
            
            if task_status == "1":
                if not accept:
                    logger.info("订单已创建, 等待机器人接单")
            elif task_status == "2":
                if not accept:
                    accept = True
                    logger.info("订单正在执行, 等待机器人完成订单")
            elif task_status == "5":
                logger.info("订单被取消")
                return False, dresp
            elif task_status == "9":
                logger.info("订单完成")
                return True, dresp
            elif task_status == "10":
                logger.info("订单被打断")
                return False, dresp

    def check_position_match(self, workstation_name, tolerance_mm=None):
        """
        验证机器人当前坐标是否与目标站点坐标匹配
        """
        try:
            if workstation_name not in self.workstation_plot.get("stations", {}):
                logger.error(f"workstation_plot中{workstation_name}站点位置信息缺失")
                return False, {
                    "match": False,
                    "error_type": "MISSING_CONFIG",
                    "station_name": workstation_name,
                    "reason": f"workstation_plot中{workstation_name}站点位置信息缺失"
                }

            expected_station = self.workstation_plot["stations"][workstation_name]
            expected_posX = float(expected_station["posX"])
            expected_posY = float(expected_station["posY"])
            expected_robotDir = float(expected_station.get("robotDir", 0))

            if tolerance_mm is None:
                tolerance_mm = self.workstation_plot.get("tolerance", 500)
            dir_tolerance = self.workstation_plot.get("dir_tolerance", 5)

            agv_info = self.navi_serve_callback(return_flage=1)
            if agv_info is None:
                logger.error("无法获取机器人当前坐标信息")
                return False, {
                    "match": False,
                    "error_type": "API_ERROR",
                    "station_name": workstation_name,
                    "reason": "无法获取机器人坐标"
                }

            actual_posX = float(agv_info.get("posX", 0.0))
            actual_posY = float(agv_info.get("posY", 0.0))
            actual_robotDir = float(agv_info.get("robotDir", 0.0))

            while actual_robotDir > 180:
                actual_robotDir -= 360
            while actual_robotDir < -180:
                actual_robotDir += 360

            distance = ((actual_posX - expected_posX) ** 2 +
                       (actual_posY - expected_posY) ** 2) ** 0.5

            dir_diff = actual_robotDir - expected_robotDir
            while dir_diff > 180:
                dir_diff -= 360
            while dir_diff < -180:
                dir_diff += 360
            dir_diff = abs(dir_diff)

            is_pos_match = distance <= tolerance_mm
            is_dir_match = dir_diff <= dir_tolerance
            is_match = is_pos_match and is_dir_match

            if is_match:
                error_type = None
            elif not is_pos_match and not is_dir_match:
                error_type = "COORDINATE_AND_DIR_MISMATCH"
            elif not is_pos_match:
                error_type = "COORDINATE_MISMATCH"
            else:
                error_type = "DIR_MISMATCH"

            result = {
                "match": is_match,
                "error_type": error_type,
                "station_name": workstation_name,
                "expected_pos": {"posX": expected_posX, "posY": expected_posY, "robotDir": expected_robotDir},
                "actual_pos": {"posX": actual_posX, "posY": actual_posY, "robotDir": actual_robotDir},
                "distance_mm": distance,
                "tolerance_mm": tolerance_mm,
                "dir_diff_deg": dir_diff,
                "dir_tolerance_deg": dir_tolerance
            }

            if is_match:
                logger.info(f"位置验证通过: 站点={workstation_name}, 距离={distance:.2f}mm, 角度差={dir_diff:.1f}°")
            else:
                logger.warning(f"位置验证失败: 站点={workstation_name}")
                if not is_pos_match:
                    logger.warning(f"  坐标不匹配: 距离={distance:.2f}mm, 容差={tolerance_mm}mm")
                if not is_dir_match:
                    logger.warning(f"  姿态不匹配: 角度差={dir_diff:.1f}°, 容差=±{dir_tolerance}°")

            return is_match, result

        except Exception as e:
            logger.error(f"坐标验证异常: {e}")
            return False, {
                "match": False,
                "error_type": "EXCEPTION",
                "station_name": workstation_name,
                "reason": f"验证过程异常: {str(e)}"
            }

    def navi_serve_callback(self, return_flage=0):
        jreq = self.agvlist.StatusQuest(self.hik_system)

        try:
            resp = requests.post(
                self.AGV_url2 + "queryAgvStatus",
                data=jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15),
            )

        except Exception as e:
            logger.warning(f"navi_serve_callback网络请求异常: {e}")
            return None

        raw_info = {}
        if resp and resp.status_code == 200:
            return_flage = 1
            raw_info = json.loads(resp.text)
        else:
            logger.info(f"Failed to fetch AGV status: {resp.status_code}")
            return None

        agv_info=self.agvlist.ExtractAgvInfo(raw_info)
        self.navi_serve_feedback_msg = agv_info

        if return_flage and agv_info != None:
            return agv_info
        else:
            logger.info(f"API returned error: {raw_info.get('message')}")

    def navi_serve_callback_only_station(self, return_flage=0):
        """获取AGV当前站点代码"""
        jreq = self.agvlist.StatusQuest(self.hik_system)
        logger.info(self.hik_system)  # 保持与原代码一致的日志输出

        try:
            resp = requests.post(
                self.AGV_url2 + "queryAgvStatus",
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
            logger.info(raw_info)  # 保持与原代码一致的日志输出
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
        req.data = ""
        resp = client.call(req.data)
        raw_info = resp.data
        self.oper_serve_feedback_msg = json.loads(raw_info)

    def _manage_charging(self, current_battery, is_charging, is_busy, idle_time):
        """充电策略管理"""
        current_time = time.time()

        if self.is_simple_charge_mode:
            threshold = self.minimum_battery_threshold

            if current_battery < threshold:
                if is_charging:
                    self.charge_command_sent_time = None
                    self.charge_retry_count = 0
                elif not is_busy and idle_time > self.idle_time_before_charge:
                    if self.charge_command_sent_time is None:
                        logger.info(f"简化模式：电量{current_battery}%低于{threshold}%，空闲{idle_time:.0f}秒，触发充电")
                        success, _ = self.PubAgvChargeTask()
                        if success:
                            self.charge_command_sent_time = current_time
                    else:
                        elapsed = current_time - self.charge_command_sent_time
                        if elapsed > self.charge_wait_timeout:
                            if self.charge_retry_count < self.max_charge_retries:
                                self.charge_retry_count += 1
                                logger.warning(f"充电等待超时，重试 ({self.charge_retry_count}/{self.max_charge_retries})")
                                self.PubAgvUnChargeTask()
                                time.sleep(2)
                                success, _ = self.PubAgvChargeTask()
                                if success:
                                    self.charge_command_sent_time = current_time
                            else:
                                logger.warning(f"充电重试达上限，放弃本次充电")
                                self.charge_command_sent_time = None
                                self.charge_retry_count = 0
                else:
                    if is_busy and self.charge_command_sent_time is not None:
                        logger.info("检测到任务执行中，取消之前的充电命令")
                        self.PubAgvUnChargeTask()
                        self.charge_command_sent_time = None
                        self.charge_retry_count = 0

            else:
                if is_charging:
                    logger.info(f"简化模式：电量{current_battery}%达到{threshold}%，取消充电")
                    self.PubAgvUnChargeTask()
                self.charge_command_sent_time = None
                self.charge_retry_count = 0
                self.is_in_low_battery_mode = False

            return

        # 三档模式
        if current_battery < self.minimum_battery_threshold:
            if is_charging:
                self.charge_command_sent_time = None
                self.charge_retry_count = 0
                self.is_in_low_battery_mode = True
            elif self.charge_command_sent_time is None:
                logger.warning(f"低电量模式：当前电量{current_battery}%，发送充电命令")
                success, _ = self.PubAgvChargeTask()
                if success:
                    self.charge_command_sent_time = current_time
                self.is_in_low_battery_mode = True
            else:
                elapsed = current_time - self.charge_command_sent_time
                if elapsed > self.charge_wait_timeout:
                    if self.charge_retry_count < self.max_charge_retries:
                        self.charge_retry_count += 1
                        logger.warning(f"充电等待超时({elapsed:.0f}秒)，重试充电 ({self.charge_retry_count}/{self.max_charge_retries})")
                        self.PubAgvUnChargeTask()
                        time.sleep(2)
                        success, _ = self.PubAgvChargeTask()
                        if success:
                            self.charge_command_sent_time = current_time
                    else:
                        logger.error(f"充电重试次数已达上限({self.max_charge_retries})，请人工检查充电桩")
                        self.charge_command_sent_time = None
                        self.charge_retry_count = 0

        elif current_battery < self.high_battery_threshold:
            # 电量已脱离低电量区间，重置低电量模式标志
            self.is_in_low_battery_mode = False
            
            if is_charging:
                self.charge_command_sent_time = None
                self.charge_retry_count = 0
                self.is_charging_to_full = True  # 标记：充电中，等待充满
            elif not is_busy and idle_time > self.idle_time_before_charge:
                if self.charge_command_sent_time is None:
                    logger.info(f"中电量模式：电量{current_battery}%，空闲{idle_time:.0f}秒，触发充电")
                    success, _ = self.PubAgvChargeTask()
                    if success:
                        self.charge_command_sent_time = current_time
                        self.is_charging_to_full = True  # 标记：充电中，等待充满
                else:
                    elapsed = current_time - self.charge_command_sent_time
                    if elapsed > self.charge_wait_timeout:
                        if self.charge_retry_count < self.max_charge_retries:
                            self.charge_retry_count += 1
                            logger.warning(f"中电量充电等待超时，重试 ({self.charge_retry_count}/{self.max_charge_retries})")
                            self.PubAgvUnChargeTask()
                            time.sleep(2)
                            success, _ = self.PubAgvChargeTask()
                            if success:
                                self.charge_command_sent_time = current_time
                        else:
                            logger.warning(f"中电量充电重试达上限，放弃本次充电")
                            self.charge_command_sent_time = None
                            self.charge_retry_count = 0
                            self.is_charging_to_full = False
            else:
                if is_busy and self.charge_command_sent_time is not None:
                    logger.info("检测到任务执行中，取消之前的充电命令")
                    self.PubAgvUnChargeTask()
                    self.charge_command_sent_time = None
                    self.charge_retry_count = 0
                    self.is_charging_to_full = False

        # 第3档：高电量（>=high）- 根据情况决定是否取消充电
        else:
            self.is_in_low_battery_mode = False
            
            if is_charging:
                # 正在充电中
                if is_busy:
                    # 有任务要执行，立即取消充电
                    logger.info(f"高电量模式：电量{current_battery}%，有任务需要执行，取消充电")
                    self.PubAgvUnChargeTask()
                    self.is_charging_to_full = False
                elif current_battery >= 100:
                    # 已充满100%，取消充电
                    logger.info(f"高电量模式：电量已充满{current_battery}%，取消充电")
                    self.PubAgvUnChargeTask()
                    self.is_charging_to_full = False
                elif self.is_charging_to_full:
                    # 正在充电且标记为充满模式，继续充电不取消
                    pass  # 什么都不做，让它继续充电
                else:
                    # 其他情况（比如手动在充电桩上），不干预
                    pass
            else:
                # 不在充电状态，重置标记
                self.is_charging_to_full = False
            
            # 重置充电命令状态
            self.charge_command_sent_time = None
            self.charge_retry_count = 0

    def robot_msg_gen(self, navi, oper):
        self.robot_msg["current_workstation"] = self.now_workstation
        if navi["status"] in ["Idle_task","Task_completed"]:
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

        if "detail" in oper:
            self.robot_msg["detail"]["oper"] = oper["detail"]

    def _update_status_with_lock(self, new_status):
        """带状态锁的状态更新方法"""
        current_time = time.time()

        if new_status in ["BUSY", "CHARGING", "ERROR"]:
            if self.last_stable_status != new_status:
                logger.info(f"状态变更: {self.last_stable_status} → {new_status} (立即生效)")
            self.robot_heartbeat_msg["status"] = new_status
            self.last_stable_status = new_status
            self.idle_start_time = None
            return new_status

        elif new_status == "IDLE":
            if self.last_stable_status == "IDLE":
                self.robot_heartbeat_msg["status"] = "IDLE"  # 确保状态同步
                return "IDLE"
            else:
                if self.idle_start_time is None:
                    self.idle_start_time = current_time
                    logger.debug(f"检测到IDLE状态，开始稳定期计时...")
                    # 稳定期内保持旧状态，确保心跳消息同步
                    self.robot_heartbeat_msg["status"] = self.last_stable_status
                    return self.last_stable_status

                elapsed = current_time - self.idle_start_time
                if elapsed >= self.idle_stable_duration:
                    current_battery = int(float(self.robot_heartbeat_msg["electricityQuantity"]))
                    logger.info(f"状态变更: {self.last_stable_status} → IDLE (稳定{elapsed:.1f}秒后生效), 当前电量: {current_battery}%")
                    self.robot_heartbeat_msg["status"] = "IDLE"
                    self.last_stable_status = "IDLE"
                    self.idle_start_time = None
                    return "IDLE"
                else:
                    logger.debug(f"IDLE稳定期: {elapsed:.1f}/{self.idle_stable_duration}秒")
                    # 稳定期内保持旧状态，确保心跳消息同步
                    self.robot_heartbeat_msg["status"] = self.last_stable_status
                    return self.last_stable_status

        return self.last_stable_status

    def GetStatusAlarmMsg(self):
        self.robot_heartbeat_msg['workstation'] = self.now_workstation
        self.robot_heartbeat_msg["status"] = self.robot_msg["state"]
        return self.robot_heartbeat_msg

    def update_status(self):
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
                    self.robot_heartbeat_msg["status"] = "ERROR"
                    self.robot_heartbeat_msg["detailMsg"] = inactive_node + "节点异常"

            if cnt == 10 or (cnt > 10 and cnt % 300 == 0):
                current_battery = float(self.robot_heartbeat_msg["electricityQuantity"])
                current_time = time.time()
                idle_time = current_time - self.last_task_time

                agv_info = self.navi_serve_callback(1)
                is_charging = (agv_info and agv_info.get("status") == "Charging_status") if agv_info else False
                is_busy = self.callback_fun_mutex.locked()

                self._manage_charging(current_battery, is_charging, is_busy, idle_time)

            if cnt > 10:
                self.query_msg["stamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                self.navi_serve_callback()
                self.oper_serve_callback()

                robot_details = ""
                platform_status = self.navi_serve_feedback_msg.get("status", "")
                arm_status = self.oper_serve_feedback_msg.get("state", "")

                new_status = None
                if platform_status == "Abnormal_task" or arm_status == "error":
                    new_status = "ERROR"
                elif platform_status == "Executing_task" or arm_status == "running":
                    new_status = "BUSY"
                elif platform_status == "Charging_status":
                    new_status = "CHARGING"
                    self.now_workstation = "charge_station"
                elif self.callback_fun_mutex.locked():
                    new_status = "BUSY"
                elif platform_status == "Task_completed" or platform_status == "Idle_task" or arm_status == "done":
                    new_status = "IDLE"
                else:
                    new_status = "IDLE"

                self._update_status_with_lock(new_status)

                self.robot_heartbeat_msg["detailMsg"] = robot_details

            cnt += 1
            rate.sleep()

    def heartbeat_monitoring(self):
        rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            self.SendHreartbeatMsg()
            rate.sleep()

    def DmsFlaskHttpSever(self):
        self.robot_flask_server.RunServer()

    def determine_destination(self, operations):
        """判断底盘移动的目标工作站"""
        robot_code = self.config_param["robotCode"]
        source = operations.get("source", {})
        target = operations.get("target", {})
        source_workstation = source.get("workstation")
        target_workstation = target.get("workstation")

        if source_workstation and target_workstation:
            if source_workstation == robot_code and target_workstation == robot_code:
                logger.info("source 和 target 都是 robotCode，不用移动")
                return None
            elif source_workstation == target_workstation:
                return source_workstation
            elif source_workstation == robot_code:
                return target_workstation
            elif target_workstation == robot_code:
                return source_workstation
            else:
                logger.error("source 和 target 都不是 robotCode 且不同，无效目标")
                return "error"

        elif source_workstation and not target_workstation:
            if source_workstation == robot_code:
                logger.info("source 是 robotCode，target 不存在，不用移动")
                return None
            return source_workstation

        elif target_workstation and not source_workstation:
            if target_workstation == robot_code:
                logger.info("target 是 robotCode，source 不存在，不用移动")
                return None
            return target_workstation

        else:
            logger.info("source 和 target 都不存在，不用移动")
            return None

    def parse_positions(self, positions):
        '''解析多个 'xxx_position' 和对应的值，并拼接成字符串'''
        parsed_positions = []
        for key, value in positions.items():
            if key.endswith('_position'):
                parsed_positions.append(f"{key[:-9]}_{value}")
        return '-'.join(parsed_positions)


    def build_operation_string(self, origin_datas, robot_code):
        '''根据不同的情况构建新的操作字符串'''
        source = origin_datas.get("source", {})
        target = origin_datas.get("target", {})
        source_workstation = source.get("workstation")
        target_workstation = target.get("workstation")
        source_positions = self.parse_positions(source)
        target_positions = self.parse_positions(target)

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

        if parts[2] == 'to':
            source_workstation = parts[1][2:]
            target_workstation = parts[3][2:]
        else:
            source_workstation = parts[1][2:]
            target_workstation = parts[2][2:]

        if source_workstation == robot_code and target_workstation != robot_code:
            workstation = target_workstation
        elif target_workstation == robot_code and source_workstation != robot_code:
            workstation = source_workstation
        elif source_workstation == robot_code and target_workstation == robot_code:
            workstation = target_workstation
        else:
            workstation = source_workstation if parts[2] == 'to' else target_workstation

        operation = operation_string

        return workstation, operation

    def get_specific_operation(self, workstation_operations, workstation, operation):
        '''根据工作站和操作获取具体的操作字符串'''
        if not workstation_operations:
            logger.error("机械臂操作配置文件(workstation_operations.json)未加载或为空")
        elif 'param' not in workstation_operations:
            logger.error("机械臂操作配置文件格式错误: 缺少'param'字段")

        for param in workstation_operations['param']:
            if param['workstation'] == workstation:
                if operation in param['operations']:
                    return param['operations'][operation]
                else:
                    available_operations = list(param['operations'].keys())
                    logger.error(f"工作站 '{workstation}' 未找到操作 '{operation}', 可用操作: {available_operations}")

        available_workstations = [p.get('workstation', 'unknown') for p in workstation_operations.get('param', [])]
        logger.error(f"未找到工作站 '{workstation}', 可用工作站: {available_workstations}")

        return "{workstation_operations}-{operation} not found"

    def process_operation(self, operations):
        robot_code = self.config_param["robotCode"]
        operation_string = self.build_operation_string(operations, robot_code)
        if operation_string:
            workstation, operation = self.extract_workstation_and_operation(operation_string, robot_code)
            specific_operation = self.get_specific_operation(self.workstation_operations, workstation, operation)

        return specific_operation

    def generate_and_send_status(self, code=200):
        self.statusFeedbackCode = code
        logger.info(f"invoke generate_and_send_status statusFeedbackCode= {self.statusFeedbackCode}")

        msg = "Error" if code == 500 else "Success"
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

    #     self.obstacle_avoidance_feedback_status(code)

    # def obstacle_avoidance_feedback_status(self, code=200):
    #     msg = "Error" if code == 500 else "Success"

    #     obstacle_avoidance_feedback_msg = {
    #         "id": 666,
    #         "stamp": 172434343443,
    #         "param": {
    #             "chainName": "methodExecute",
    #             "ifAsynchronous": False,
    #             "sn": "L04A79D0ED8252",
    #             "method": "receiveData",
    #             "body": {
    #                 "id": self.DMS_msg.get("id"),
    #                 "stamp": str(int(round(time.time() * 1000))),
    #                 "code": code,
    #                 "message": msg,
    #                 "isRobot": True,
    #                 "result": None,
    #                 "retry": False,
    #                 "data": {},
    #                 "vars": None
    #             }
    #         }
    #     }

    #     try:
    #         response = requests.post(
    #             self.obstacle_avoidance_feedback_url,
    #             json=obstacle_avoidance_feedback_msg,
    #             headers=self.headers,
    #             timeout=(60, 15),
    #         )
    #         logger.info(f"与中央工作台上的机器人避障信息发送成功: {obstacle_avoidance_feedback_msg}")
    #     except Exception as e:
    #         logger.info(f"与中央工作台上的机器人避障信息发送失败: {e}")


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
