#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
站点标点工具
功能: 读取机器人当前坐标和姿态，保存到workstation_plot.json

使用方法:
1. 运行本脚本: python3 station_marker.py
2. 输入1开始标点，输入站点名称，回车确认
3. 标点信息自动保存到workstation_plot.json
'''

import json
import sys
import rospkg
from agv_tasklist_manager import AgvTaskListManager
import requests


class StationMarker:
    def __init__(self):
        # 加载配置文件
        package_path = rospkg.RosPack().get_path("robot_message_bridge")

        # 加载param.json
        param_file_path = package_path + "/config/param.json"
        try:
            with open(param_file_path, encoding="utf-8") as f:
                self.config_param = json.load(f)
            print(f"[INFO] 配置文件加载成功: {param_file_path}")
        except FileNotFoundError:
            print(f"[ERROR] 配置文件不存在: {param_file_path}")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"[ERROR] 配置文件JSON格式错误: {param_file_path}, 错误: {e}")
            sys.exit(1)

        # 加载station_mapping.json
        station_mapping_file_path = package_path + "/config/station_mapping.json"
        try:
            with open(station_mapping_file_path, encoding="utf-8") as f:
                self.station_mapping = json.load(f)
        except:
            self.station_mapping = {}

        # 加载status.json
        status_file_path = package_path + "/config/status.json"
        try:
            with open(status_file_path, encoding="utf-8") as f:
                self.agv_status = json.load(f)
        except:
            self.agv_status = {}

        # 加载workstation_plot.json
        self.workstation_plot_file = package_path + "/config/workstation_plot.json"
        try:
            with open(self.workstation_plot_file, encoding="utf-8") as f:
                self.workstation_plot = json.load(f)
            print(f"[INFO] 站点坐标配置加载成功: {self.workstation_plot_file}")
        except FileNotFoundError:
            print(f"[WARN] 站点坐标配置文件不存在，将创建新文件")
            self.workstation_plot = {
                "comment": "站点坐标配置文件 - 用于验证机器人是否到达指定站点",
                "tolerance": 500,
                "comment_tolerance": "坐标容差范围（毫米），默认500mm",
                "stations": {}
            }
        except json.JSONDecodeError as e:
            print(f"[ERROR] 站点坐标配置JSON格式错误: {e}")
            sys.exit(1)

        # 使用AgvTaskListManager（与dms_to_robot.py保持一致）
        self.agvlist = AgvTaskListManager(self.config_param, self.station_mapping, self.agv_status)
        self.AGV_url2 = self.config_param["AGV_url2"]
        self.headers = {"content-type": "application/json"}
        self.hik_system = "rcs_2000"  # 默认值，可根据需要修改

    def get_agv_info(self):
        """
        获取AGV当前状态信息
        使用与dms_to_robot.py相同的方式
        """
        # 使用AgvTaskListManager构建请求（与dms_to_robot.py一致）
        jreq = self.agvlist.StatusQuest(self.hik_system)

        try:
            resp = requests.post(
                self.AGV_url2 + "queryAgvStatus",
                data=jreq.encode("utf-8"),
                headers=self.headers,
                timeout=(10, 10),
            )

            if resp and resp.status_code == 200:
                raw_info = json.loads(resp.text)
                # 使用AgvTaskListManager提取AGV信息（与dms_to_robot.py一致）
                agv_info = self.agvlist.ExtractAgvInfo(raw_info)
                return agv_info
            else:
                print(f"[ERROR] HTTP请求失败: {resp.status_code}")
                return None

        except Exception as e:
            print(f"[ERROR] 获取AGV信息异常: {e}")
            return None

    def list_existing_stations(self):
        """列出已有站点名称"""
        stations = self.workstation_plot.get("stations", {})
        if stations:
            print("\n" + "=" * 50)
            print("已有站点列表:")
            print("-" * 50)
            for idx, name in enumerate(sorted(stations.keys()), 1):
                print(f"  {idx}. {name}")
            print("=" * 50)
        else:
            print("\n[INFO] 当前没有已保存的站点")

    def save_station(self, station_name, posX, posY, robotDir):
        """保存站点坐标到配置文件"""
        self.workstation_plot["stations"][station_name] = {
            "posX": str(posX),
            "posY": str(posY),
            "robotDir": str(robotDir)
        }

        try:
            with open(self.workstation_plot_file, 'w', encoding='utf-8') as f:
                json.dump(self.workstation_plot, f, ensure_ascii=False, indent=4)
            return True
        except Exception as e:
            print(f"[ERROR] 保存配置文件失败: {e}")
            return False

    def normalize_direction(self, direction):
        """
        将角度归一化为90度的倍数

        参数:
            direction: 原始角度（-180~360度，兼容API返回范围）
        返回:
            归一化后的角度（0, 90, 180, -90）
            注意: -180统一为180，保证同一方向值一致
        """
        try:
            angle = float(direction)
            # 先归一化到 -180 ~ 180 范围（处理360等超范围值）
            while angle > 180:
                angle -= 360
            while angle < -180:
                angle += 360

            # 四舍五入到最近的90度倍数
            normalized = round(angle / 90) * 90

            # 确保在 -180 ~ 180 范围内
            if normalized > 180:
                normalized -= 360
            if normalized < -180:
                normalized += 360

            # 统一 -180 为 180（同一方向保持一致）
            if normalized == -180:
                normalized = 180

            return int(normalized)
        except (ValueError, TypeError):
            return 0

    def mark_station(self):
        """标记当前位置为站点（只在此时获取坐标）"""
        print("\n[INFO] 正在获取机器人坐标...")

        # 只在标点时获取AGV信息
        agv_info = self.get_agv_info()

        if agv_info is None:
            print("[ERROR] 无法获取机器人坐标信息，请检查:")
            print("  1. 机器人是否已连接")
            print("  2. RCS系统是否正常运行")
            print("  3. param.json中AGV_url2配置是否正确")
            return False

        # 获取坐标和姿态
        posX = agv_info.get("posX", "0")
        posY = agv_info.get("posY", "0")
        robotDir_raw = agv_info.get("robotDir", "0")

        # 将姿态归一化为90度倍数
        robotDir_normalized = self.normalize_direction(robotDir_raw)

        print(f"\n当前机器人位置:")
        print(f"  posX: {posX}")
        print(f"  posY: {posY}")
        print(f"  robotDir: {robotDir_raw} -> 归一化: {robotDir_normalized}")

        # 输入站点名称
        station_name = input("\n请输入站点名称: ").strip()

        if not station_name:
            print("[WARN] 站点名称不能为空，取消标点")
            return False

        # 检查是否已存在
        if station_name in self.workstation_plot.get("stations", {}):
            confirm = input(f"[WARN] 站点 '{station_name}' 已存在，是否覆盖? (y/n): ").strip().lower()
            if confirm != 'y':
                print("[INFO] 取消标点")
                return False

        # 保存站点（使用归一化后的姿态）
        if self.save_station(station_name, posX, posY, robotDir_normalized):
            print("\n" + "=" * 50)
            print("[SUCCESS] 标点成功!")
            print("-" * 50)
            print(f"  站点名称: {station_name}")
            print(f"  posX: {posX}")
            print(f"  posY: {posY}")
            print(f"  robotDir: {robotDir_normalized} (原始值: {robotDir_raw})")
            print("=" * 50)
            return True

        return False

    def run(self):
        """主循环"""
        print("\n" + "=" * 50)
        print("       站点标点工具")
        print("=" * 50)
        print("命令: 输入1开始标点, 输入q退出")
        print("=" * 50)

        # 显示已有站点
        self.list_existing_stations()

        while True:
            print()
            user_input = input("请输入命令 (1=标点, q=退出): ").strip().lower()

            if user_input == '1':
                self.mark_station()
                # 标点后刷新显示已有站点
                self.list_existing_stations()
            elif user_input == 'q':
                print("\n[INFO] 退出标点工具")
                break
            else:
                print("[WARN] 无效命令，请输入1标点或q退出")


def main():
    try:
        marker = StationMarker()
        marker.run()
    except KeyboardInterrupt:
        print("\n\n[INFO] 用户中断，退出")
    except Exception as e:
        print(f"\n[ERROR] 程序异常: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
