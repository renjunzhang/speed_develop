#!/usr/bin/env python3
import rospy
import requests
import json
import math
import time
import sys
# Old ROS message imports - no longer needed
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from communication_rs485.msg import platformInfo
# from tf.transformations import euler_from_quaternion
import moveit_commander
# with open("/home/geist/.chemrob/rob_heartmsg.json",'w',encoding='utf-8') as f:
#     json.dump(self.robot_heartbeat_msg, f,ensure_ascii=False)
# new---------
from rtde_receive import RTDEReceiveInterface
import rospkg
from agv_tasklist_manager import AgvTaskListManager
# new---------
# 仿射变换矩阵和平移向量
# 变换公式: target = A * original + b
# target_x = a11 * pos_x_m + a21 * pos_y_m + b1
# target_y = a12 * pos_x_m + a22 * pos_y_m + b2
# 通过7个标定点(A,B,C,D,E,F,H)最小二乘法计算，RMSE=0.189m
# 坐标系关系：target(x2,y2) ≈ (y1,-x1)，即旋转90度
# 注意：a21 是 pos_y 对 target_x 的影响，a12 是 pos_x 对 target_y 的影响
# 更新时间：2025-11-03，使用 find_best_transform.py 重新计算
AFFINE_TRANSFORM = {
    "a11": -0.00000000,   # pos_x 对 target_x 的系数
    "a21": 1.07836757,    # pos_y 对 target_x 的系数
    "b1": -352.80454030,  # target_x 的平移
    "a12": -1.01045639,   # pos_x 对 target_y 的系数
    "a22": -0.01007877,   # pos_y 对 target_y 的系数
    "b2": 329.52622324    # target_y 的平移
}


# 旧的简单偏移量（已废弃）
# SELFDEFINED_OFFSET={
#     "x":1.13562,
#     "y":-14.79+3.88491
# }
class RobotCommNode:
    def __init__(self, server_url,heartbeat_url):
        # Initialize ROS node
        rospy.init_node('robot_comm_node', anonymous=True)
        self.server_url = server_url
        self.heartbeat_url = heartbeat_url
        self.heartbeat_msg = {}
        self.count = 0

        # new---------
        self.arm_rtde_receive = RTDEReceiveInterface("192.168.56.100")  # 替换为你机械臂的 IP 地址
        # new---------

        # Load configuration files
        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/param.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.config_param = json.load(f)

        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/status.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.agv_status = json.load(f)

        param_file_path = rospkg.RosPack().get_path("robot_message_bridge") + "/config/station_mapping.json"
        with open(param_file_path, encoding="utf-8") as f:
            self.station_mapping = json.load(f)

        # Initialize AGV task list manager
        self.hik_system = rospy.get_param('~hiksystem', 'rcs_2000')
        self.agvlist = AgvTaskListManager(self.config_param, self.station_mapping, self.agv_status)
        self.headers = {"content-type": "application/json"}

        # Initialize data structure
        self.robot_data = {
            "stamp": "",
            "name": "11119",
            "floor": 1,
            "platform": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "battery": 0.0
            },
            "arm": {
                "dual": False,
                "positions": [],
                "joints": [],
                "effort": []
            }
        }

        # Remove old ROS subscribers - now using HTTP API
        # self.pose_sub = rospy.Subscriber('/amcl_pose_tf', PoseWithCovarianceStamped, self.pose_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

    # Old pose_callback - replaced by get_agv_status_from_api
    # def pose_callback(self, msg):
    #     """Callback for chassis pose"""
    #     self.robot_data["platform"]["x"] = - msg.pose.pose.position.y + SELFDEFINED_OFFSET["x"]
    #     self.robot_data["platform"]["y"] = msg.pose.pose.position.x + SELFDEFINED_OFFSET["y"]
    #     # Convert quaternion to yaw
    #     orientation = msg.pose.pose.orientation
    #     quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    #     roll, pitch, yaw = euler_from_quaternion(quaternion)
    #     self.robot_data["platform"]["yaw"] = yaw + math.pi/2

    def get_agv_status_from_api(self):
        """Get AGV status from Hikrobot API (battery, posX, posY, podDir)"""
        try:
            jreq = self.agvlist.StatusQuest(self.hik_system)
            resp = requests.post(
                "http://192.168.200.157:8182/rcms-dps/rest/queryAgvStatus",
                data=jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(60, 15),
            )

            if resp and resp.status_code == 200:
                raw_info = json.loads(resp.text)
                # Extract specified AGV information
                agv_info = self.agvlist.ExtractAgvInfo(raw_info)

                if agv_info:
                    return agv_info
                else:
                    rospy.logwarn("Failed to extract AGV info")
                    return None
            else:
                rospy.logwarn(f"Failed to fetch AGV status: {resp.status_code}")
                return None

        except Exception as e:
            rospy.logwarn(f"Failed to get AGV status from API: {e}")
            return None

    def get_platform_info(self):
        """Get platform information (battery, position, yaw) from Hikrobot API"""
        try:
            agv_info = self.get_agv_status_from_api()

            if agv_info:
                # Update battery
                self.robot_data["platform"]["battery"] = float(agv_info.get("battery", 0.0))

                # Update position (x, y) - use affine transformation
                pos_x = float(agv_info.get("posX", 0.0))  # 原始坐标，单位：毫米
                pos_y = float(agv_info.get("posY", 0.0))  # 原始坐标，单位：毫米

                # 转换为米
                pos_x_m = pos_x / 1000.0
                pos_y_m = pos_y / 1000.0

                # 应用仿射变换
                # 正确的公式：
                #   target_x = a11 * pos_x_m + a21 * pos_y_m + b1
                #   target_y = a12 * pos_x_m + a22 * pos_y_m + b2
                self.robot_data["platform"]["x"] = (
                    AFFINE_TRANSFORM["a11"] * pos_x_m +
                    AFFINE_TRANSFORM["a21"] * pos_y_m +
                    AFFINE_TRANSFORM["b1"]
                )
                self.robot_data["platform"]["y"] = (
                    AFFINE_TRANSFORM["a12"] * pos_x_m +
                    AFFINE_TRANSFORM["a22"] * pos_y_m +
                    AFFINE_TRANSFORM["b2"]
                )

                # Update yaw from podDir (robotDir or podDir)
                # podDir is in degrees, convert to radians
                # 注意：由于位置坐标进行了90度旋转（仿射变换，旋转角度91.60°），
                # 姿态角度也需要相应调整
                # 坐标系关系：target(x2,y2) = (y1,-x1)，相当于逆时针旋转90度
                pod_dir = float(agv_info.get("podDir", 0.0))
                self.robot_data["platform"]["yaw"] = math.radians(pod_dir) + math.pi/2

                # 详细日志：显示原始坐标和转换后的坐标
                rospy.logdebug(f"Platform info updated:")
                rospy.logdebug(f"  原始坐标: pos_x={pos_x}mm, pos_y={pos_y}mm, podDir={pod_dir}°")
                rospy.logdebug(f"  转换后: x={self.robot_data['platform']['x']:.3f}m, "
                              f"y={self.robot_data['platform']['y']:.3f}m, "
                              f"yaw={self.robot_data['platform']['yaw']:.3f}rad ({math.degrees(self.robot_data['platform']['yaw']):.1f}°)")
                rospy.logdebug(f"  电量: {self.robot_data['platform']['battery']:.1f}%")
            else:
                rospy.logwarn("Failed to get platform info from API")

        except Exception as e:
            rospy.logwarn(f"Failed to get platform info: {e}")

    def get_arm_states(self):
        """Get UR5e arm states using MoveIt"""
        try:
            # new----------------------------------------
            # 获取末端 TCP 姿态（位置 + 轴角姿态）
            tcp_pose = self.arm_rtde_receive.getActualTCPPose()  # [x, y, z, rx, ry, rz]
            self.robot_data["arm"]["positions"] = tcp_pose

            # 获取当前关节角（单位：弧度）
            joint_positions = self.arm_rtde_receive.getActualQ()  # [q1, q2, q3, q4, q5, q6]
            self.robot_data["arm"]["joints"] = joint_positions
            # print("末端位姿 (TCP pose):", tcp_pose)
            # print("关节角 (Joint positions):", joint_positions)
            # new----------------------------------------
        except Exception as e:
            rospy.logwarn(f"Failed to get arm states: {e}")

    def send_data(self):
        """
        Send data to server via HTTP POST

        两个接口的区别:
        1. server_url (10010端口): 发送实时机器人数据 (robot_data)
           - 包含底盘状态 (x, y, yaw, battery)
           - 包含机械臂状态 (positions, joints)
           - 每次循环都发送 (10Hz频率)

        2. heartbeat_url (10012端口): 发送心跳消息 (heartbeat_msg)
           - 包含机器人基本信息和健康状态
           - 每10次循环发送一次 (1Hz频率)
        """
        self.count += 1
        self.count %= 10
        try:
            with open("/home/geist/.chemrob/rob_heartmsg.json",'r',encoding='utf-8') as f:
                self.heartbeat_msg = json.load(f)
        except Exception as e:
            rospy.logwarn(f"Json load failed: {e}")


        # ========== 发送实时机器人数据到 10010 端口 ==========
        try:
            headers = {'Content-Type': 'application/json'}
            self.robot_data["stamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
            # self.robot_data["name"] = self.heartbeat_msg["machineCode"]

            rospy.loginfo("=" * 80)
            rospy.loginfo(f"[机器人数据上报] URL: {self.server_url}")
            rospy.loginfo(f"[机器人数据上报] 数据内容:")
            rospy.loginfo(f"  - 时间戳: {self.robot_data['stamp']}")
            rospy.loginfo(f"  - 机器人名称: {self.robot_data['name']}")
            rospy.loginfo(f"  - 楼层: {self.robot_data['floor']}")
            rospy.loginfo(f"  - 底盘状态:")
            rospy.loginfo(f"      x={self.robot_data['platform']['x']:.3f}, "
                         f"y={self.robot_data['platform']['y']:.3f}, "
                         f"yaw={self.robot_data['platform']['yaw']:.3f}, "
                         f"battery={self.robot_data['platform']['battery']:.1f}%")
            rospy.loginfo(f"  - 机械臂状态:")
            rospy.loginfo(f"      positions: {self.robot_data['arm']['positions']}")
            rospy.loginfo(f"      joints: {self.robot_data['arm']['joints']}")
            rospy.loginfo(f"[机器人数据上报] 完整JSON:\n{json.dumps(self.robot_data, indent=2, ensure_ascii=False)}")

            response = requests.post(self.server_url, json=self.robot_data, headers=headers, timeout=5)
            if response.status_code == 200:
                rospy.loginfo(f"[机器人数据上报] ✓ 发送成功 (HTTP {response.status_code})")
                try:
                    rospy.loginfo(f"[机器人数据上报] 服务器响应: {response.text}")
                except:
                    pass
            else:
                rospy.logwarn(f"[机器人数据上报] ✗ 发送失败 (HTTP {response.status_code})")
                rospy.logwarn(f"[机器人数据上报] 响应内容: {response.text}")
        except requests.RequestException as e:
            rospy.logwarn(f"[机器人数据上报] ✗ HTTP请求异常: {e}")

        # ========== 每10次发送一次心跳消息到 10012 端口 ==========
        if self.count == 0:
            try:
                headers = {'Content-Type': 'application/json'}

                # 同步电量：将 API 获取的电量更新到心跳消息中
                # 确保 robot_data 和 heartbeat_msg 中的电量一致
                if 'electricityQuantity' in self.heartbeat_msg:
                    self.heartbeat_msg['electricityQuantity'] = self.robot_data["platform"]["battery"]

                rospy.loginfo("=" * 80)
                rospy.loginfo(f"[心跳消息上报] URL: {self.heartbeat_url}")
                rospy.loginfo(f"[心跳消息上报] 数据内容:")
                rospy.loginfo(f"  - 机器编号: {self.heartbeat_msg.get('machineCode', 'N/A')}")
                rospy.loginfo(f"  - 识别码: {self.heartbeat_msg.get('identifyingCode', 'N/A')}")
                rospy.loginfo(f"  - 状态: {self.heartbeat_msg.get('status', 'N/A')}")
                rospy.loginfo(f"  - 电量: {self.heartbeat_msg.get('electricityQuantity', 'N/A')}% (已同步自AGV API)")
                rospy.loginfo(f"  - IP: {self.heartbeat_msg.get('ip', 'N/A')}")
                rospy.loginfo(f"  - Port: {self.heartbeat_msg.get('port', 'N/A')}")
                rospy.loginfo(f"[心跳消息上报] 完整JSON:\n{json.dumps(self.heartbeat_msg, indent=2, ensure_ascii=False)}")

                response = requests.post(self.heartbeat_url, json=self.heartbeat_msg, headers=headers, timeout=5)
                if response.status_code == 200:
                    rospy.loginfo(f"[心跳消息上报] ✓ 发送成功 (HTTP {response.status_code})")
                    try:
                        rospy.loginfo(f"[心跳消息上报] 服务器响应: {response.text}")
                    except:
                        pass
                else:
                    rospy.logwarn(f"[心跳消息上报] ✗ 发送失败 (HTTP {response.status_code})")
                    rospy.logwarn(f"[心跳消息上报] 响应内容: {response.text}")
            except requests.RequestException as e:
                rospy.logwarn(f"[心跳消息上报] ✗ HTTP请求异常: {e}")

    def run(self):
        """Main loop to collect and send data"""
        while not rospy.is_shutdown():
            self.get_platform_info()
            self.get_arm_states()
            self.send_data()
            self.rate.sleep()

        # Cleanup
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    server_url = rospy.get_param('~server_url', 'http://192.168.202.31:10010/api/robot_data')
    heartbeat_url = rospy.get_param('~heartbeat_url', 'http://192.168.202.31:10012/api/robot_data')  # Configurable server URL
    try:
        node = RobotCommNode(server_url,heartbeat_url)
        node.run()
    except rospy.ROSInterruptException:
        pass