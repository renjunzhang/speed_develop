#!/usr/bin/env python3
import time
import roslaunch
import rospy
import json
from std_msgs.msg import String
from func_timeout import func_set_timeout
import requests
import rosnode
from aichem_msg_srv.srv import *
from communication_rs485.msg import platformInfo
import threading
from ui_flask_server import UiFlaskSever
import subprocess
import os
import signal


class UiServer:
    def __init__(self):
        self.navi_pub = rospy.Publisher(
            '/obsNavigation_in', String, queue_size=10)
        self.arm_pub = rospy.Publisher(
            '/obsOperation_in', String, queue_size=10)
        self.grip_pub = rospy.Publisher(
            '/grip_operation_in', String, queue_size=10)
        self.error_messages = ""
        self.start_flag = False
        self.start_stop_flag = -1  # -1不执行，1开，0关
        self.start_flag_mutex = threading.Lock()
        self.start_stop_mutex = threading.Lock()
        self.status = {"radar": -1, "platform": -1, "arm": -1, "camera": -1, "electricityQuantity": -1,
                       "error_messages": self.error_messages}

        self.oper_funs = {"Start": self.Start,
                          "Stop": self.Stop, "ArmReset": self.ArmReset, "GripReset": self.GripReset}
        self.robot_flask_sever = UiFlaskSever(
            self.HttpSeverCallback, self.PubStatus, 3040, False, '0.0.0.0')

        self.launch_list = [
            ["roslaunch communication_rs485 platform_init.launch"],
            ["roslaunch hf_navigation  clear_amcl_params.launch"],
            ["roslaunch hf_navigation hf_amcl.launch"],
            ["roslaunch hf_navigation hf_navigation.launch"],
            ["roslaunch task_communication task_communication.launch"],
            ["roslaunch ur_robot_driver ur5e_bringup.launch"],
            ["roslaunch robotiq_ft_sensor ft_sensor.launch"],
            ["roslaunch ur5e_tfsensor_pgi140_moveit_config moveit_planning_execution.launch"],
            ["roslaunch locator cv_locator.launch"],
            ["roslaunch chemicalrobot_arm_new chemicalrobot_arm_new.launch"],
            ["roslaunch web_video_server web_video_server.launch"],
            ["roslaunch robot_message_bridge robot_message_bridge.launch"]

        ]

    def RunServer(self):
        self.robot_flask_sever.RunSever()

    def Stop(self, detail):
        with self.start_stop_mutex:
            self.start_stop_flag = 0
        while self.start_flag:
            time.sleep(0.01)
        time.sleep(8)
        return {"message": "done", "detail": ""}

    def Start(self, detail):
        with self.start_stop_mutex:
            self.start_stop_flag = 1
        while not self.start_flag:
            time.sleep(0.01)
        time.sleep(8)
        return {"message": "done", "detail": ""}

    """
    开进程启动launch有问题
    def Start(self,detail):
        self.process_nodes=[subprocess.Popen(launch,shell=True,preexec_fn=os.setsid) for launch in self.launch_list]
        with self.start_flag_mutex:
            self.start_flag=True
    def Stop(self,detail):
        for process_node in self.process_nodes:
            process_node.terminate()
            process_node.wait()
            os.killpg(process_node.pid,signal.SIGTERM)
        with self.start_flag_mutex:
            self.start_flag=False
        print("关闭机器人")
    """

    def HttpSeverCallback(self, oper_message):
        """
        :param oper_message: {"operation":"ArmReset","detail":""}
        :return:
        """

        operation = oper_message["operation"]
        detail = oper_message["detail"]
        if operation in self.oper_funs:
            response_opera_message = self.oper_funs[operation](detail)
            return response_opera_message
        else:
            return {"message": "error", "detail": "没有操作指令"}

    def GetStatus(self):
        node_list = {
            '/aruco_to_point_and_talk': "camera",
            '/sick_front/sick':   "radar",
            '/sick_rear/sick':  "radar",
            '/hf_platform/rs485_control': "platform",
            '/platform_communication_node': 'platform',
            '/chemicalrobot_arm_new': 'arm',
            # '/ur5e', #机器臂的控制程
            # '/rq_sensor': '力矩传感器'
            # '/pgitest' #夹爪
        }

        for key in node_list:
            if rosnode.rosnode_ping(node_name=key, max_count=1):
                self.status[node_list[key]] = 1
            else:
                self.status[node_list[key]] = 0

        try:
            self.NaviServeCallback()
        except:
            self.error_messages += "底盘异常"
            self.status["platform"] = 0
        try:
            self.OperServeCallback()
        except:
            self.error_messages += "机械臂异常"
            self.status["arm"] = 0

        try:
            self.status["electricityQuantity"] = self.GetElectricityQuantity()
        except:
            self.error_messages += "底盘电池获取异常"

    @func_set_timeout(1)
    def GetElectricityQuantity(self):
        platform_info_datas = rospy.wait_for_message(
            '/hf_platform/platform_info', platformInfo)
        return platform_info_datas.batteryPower

    @func_set_timeout(1)
    def NaviServeCallback(self):
        client = rospy.ServiceProxy(
            "/inquiryNavigation_out", DmsService)
        client.wait_for_service()
        req = DmsServiceRequest()
        req.data = ""  # self.DMS_msg
        resp = client.call(req.data)
        raw_info = resp.data
        navi_serve_feedback_msg = json.loads(raw_info)

        if "error" == navi_serve_feedback_msg["state"]:
            self.error_messages += "底盘异常"
            self.status["platform"] = 0

    @func_set_timeout(1)
    def OperServeCallback(self):
        client = rospy.ServiceProxy("/robot", DmsService)
        client.wait_for_service()
        req = DmsServiceRequest()
        req.data = ""  # self.DMS_msg
        resp = client.call(req.data)
        raw_info = resp.data
        oper_serve_feedback_msg = json.loads(raw_info)

        if "error" == oper_serve_feedback_msg["state"]:
            self.error_messages += "机械臂异常"
            self.status["arm"] = 0

    def ArmReset(self, detail):
        oper_messages = {"message": "", "detail": ""}
        with self.start_flag_mutex:
            start_flag = self.start_flag
        if start_flag:
            print("机械臂复位")
            oper_reset_msg = {
                "id": '', "destination": "charge_station", "operations": [
                    {
                        "operation": "reset"
                    }
                ]}
            self.arm_pub.publish(String(json.dumps(oper_reset_msg)))
            try:
                arm_topic_feedback_msg = self.RosWaitForMessage(
                    "/obsOperation_out")
            except:
                print("机械臂复位超时")
                arm_topic_feedback_msg = {'state': 'error'}

            if arm_topic_feedback_msg['state'] != 'done':
                oper_messages["message"] = "error"
                oper_messages["detail"] = "机械臂复位失败，请用机械臂示教版自由驱动到合适位置"
            else:
                oper_messages["message"] = "done"
                oper_messages["detail"] = ""
        else:
            oper_messages["message"] = "error"
            oper_messages["detail"] = "请先启动机器人再进行操作"
        return oper_messages

    def GripReset(self, detail):
        """
        你接收话题名称grip_operation_out
        发送话题名称grip_operation_in
        接收的指令{"operations": 1000}
        返回的值令{'state':'done'} done或error
        """
        oper_messages = {"message": "", "detail": ""}
        with self.start_flag_mutex:
            start_flag = self.start_flag
        if start_flag:
            grip_reset_msg = {"operations": 0}
            self.grip_pub.publish(String(json.dumps(grip_reset_msg)))

            try:
                grip_topic_feedback_msg = self.RosWaitForMessage(
                    "/grip_operation_out")
            except:
                grip_topic_feedback_msg = {'state': 'error'}

            if grip_topic_feedback_msg['state'] != 'done':
                oper_messages["message"] = "error"
                oper_messages["detail"] = "复位失败"
            else:
                oper_messages["message"] = "done"
                oper_messages["detail"] = ""

        else:
            oper_messages["message"] = "error"
            oper_messages["detail"] = "请先启动机器人再进行操作"

        return oper_messages

    @func_set_timeout(20)
    def RosWaitForMessage(self, subscribe_name):
        topic_feedback_msg = json.loads(
            rospy.wait_for_message(subscribe_name, String).data)
        return topic_feedback_msg

    def MoveChargeStation(self):
        pass

    def AutoCharge(self):
        pass

    def AutoUnCharge(self):
        pass

    def PubStatus(self):

        self.error_messages = ""
        with self.start_flag_mutex:
            start_flag = self.start_flag
        if start_flag:
            self.status = {"radar": 0, "platform": 0, "arm": 0, "camera": 0, "electricityQuantity": 0,
                           "error_messages": ""}
            self.GetStatus()
            self.status["error_messages"] = self.error_messages

        else:
            self.status = {"radar": -1, "platform": -1, "arm": -1, "camera": -1, "electricityQuantity": -1,
                           "error_messages": ""}
        return self.status

    def InitLaunchConfig(self):
        launch_list = [
            ["communication_rs485", "platform_init.launch"],
            ["hf_navigation", "hf_amcl.launch"],
            ["hf_navigation", "hf_navigation.launch"],
            ["task_communication", "task_communication.launch"],
            ["ur_robot_driver", "ur5e_bringup.launch"],
            ["robotiq_ft_sensor", "ft_sensor.launch"],
            ["ur5e_tfsensor_pgi140_moveit_config",
                "moveit_planning_execution.launch"],
            ["locator", "cv_locator.launch"],
            ["chemicalrobot_arm_new", "chemicalrobot_arm_new.launch"],
            ["web_video_server", "web_video_server.launch"],
            ["robot_message_bridge", "robot_message_bridge.launch"],
        ]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)  # 等待参数服务器响应
        roslaunch.configure_logging(uuid)

        launch_files = []
        for launch in launch_list:
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch)
            launch_files.append(roslaunch_file[0])
        self.launch_start = roslaunch.parent.ROSLaunchParent(
            uuid, launch_files)

        # roslaunch.core.Node("hf_navigation", "clear_amcl_params.py")
        # self.launch_set_amcl_pose = roslaunch.scriptapi.ROSLaunch()

    def StartStop(self):
        while True:
            with self.start_stop_mutex:
                if -1 == self.start_stop_flag:
                    pass
                elif 1 == self.start_stop_flag:
                    self.InitLaunchConfig()
                    # self.launch_set_amcl_pose.start()
                    # time.sleep(5)
                    print("启动其他")
                    self.launch_start.start()
                    with self.start_flag_mutex:
                        self.start_flag = True
                    print("打开机器人")

                    self.start_stop_flag = -1
                elif 0 == self.start_stop_flag:
                    with self.start_flag_mutex:
                        self.start_flag = False
                    # self.launch_set_amcl_pose.stop()
                    self.launch_start.shutdown()
                    print("关闭机器人")
                    self.start_stop_flag = -1
            time.sleep(0.1)

    def RunNode(self):
        rospy.init_node('ui_server_node', anonymous=True, disable_signals=True)
        rospy.spin()
        # while self.start_flag:
        #     time.sleep(0.01)
        # print("节点关闭")


if __name__ == '__main__':
    ui_server = UiServer()
    threading.Thread(target=ui_server.RunNode).start()
    threading.Thread(target=ui_server.RunServer).start()
    ui_server.StartStop()
