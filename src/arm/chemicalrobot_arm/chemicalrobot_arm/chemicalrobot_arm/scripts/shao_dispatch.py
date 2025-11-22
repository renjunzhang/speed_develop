#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
from std_msgs.msg import String
import time

global pub_operation
global pub_navigation
pub_operation = rospy.Publisher('/obsOperation_in', String, queue_size=10)
time.sleep(2)
pub_navigation = rospy.Publisher('/obsNavigation_in', String, queue_size=10)


def operation_cmd(curr_station_name, operation_name):
    time.sleep(2)
    print("2222222222222222")
    global pub_operation
    obsOperation_in = {"curr_station": curr_station_name,
                       "operation": operation_name, "bottle": 0, "rack": 0}
    operation_in = String(json.dumps(
        obsOperation_in, sort_keys=True, indent=4, separators=(',', ': ')))
    pub_operation.publish(operation_in)
    print("2222222222222222")
    sub_operation = None
    while sub_operation is None:
        try:
            sub_operation = rospy.wait_for_message("/obsOperation_out", String)
        except:
            pass
    sub_operation = json.loads(sub_operation.data)
    if "YES" == sub_operation["IsDone"]:
        print(operation_name+"IsDone")
        return True
    else:
        print(operation_name)
        return False


def navigation_cmd(action_name, dest_station_name):
    global pub_navigation
    time.sleep(2)
    obsNavigation_in = {'action': action_name,
                        'dest_station': dest_station_name}
    navigation_in = String(json.dumps(
        obsNavigation_in))
    pub_navigation.publish(navigation_in)
    sub_navigation = None
    while sub_navigation is None:
        try:
            sub_navigation = rospy.wait_for_message(
                '/obsNavigation_out', String)
        except:
            pass
    sub_navigation = json.loads(sub_navigation.data)
    if 'YES' == sub_navigation['IsDone']:
        print("chenggong")
        return True

    else:
        print(dest_station_name)
        return False


def dispatch_order():
    # 1去站点1
    # if not navigation_cmd('move', '拍0站'):
    #     print("error")
    #     return
    # # # 工作站1流程
    # if not operation_cmd('测试站', '机械臂复位'):
    #     print("error")
    #     return
    # if not operation_cmd('测试站', 'relocation'):
    #     print("error")
    #     return
    # if not operation_cmd("测试站", "step1_left"):
    #     print("error")
    #     return
    # if not operation_cmd("测试站", "step2_rack"):
    #     print("error")
    #     return
    # if not operation_cmd("测试站", "step3_back"):
    #     print("error")
    #     return
    # # time.sleep(20)

    # if not operation_cmd("测试站", "step1_right"):
    #     print("error")
    #     return

    # if not operation_cmd("测试站", "step2_rack"):
    #     print("error")
    #     return

    # if not operation_cmd("测试站", "机械臂复位"):
    #     print("error")
    #     return

    # 前往2站
    if not navigation_cmd("move", "拍1站"):
        print("error")
        return

    # #2站操作
    # if not operation_cmd("液体站", "机械臂复位"):
    #     print("error")
    #     return
    # if not operation_cmd("液体站", "relocation"):
    #     print("error")
    #     return
    # if not operation_cmd("液体站", "拉伸"):
    #     print("error")
    #     return
    # time.sleep(260)
    # if not operation_cmd("液体站", "放置"):
    #     print("error")
    #     return
    # if not operation_cmd("液体站", "机械臂复位"):
    #     print("error")
    #     return

    # 前往1站
    if not navigation_cmd("move", "拍0站"):
        print("error")
        return

    #1站继续工作
    if not operation_cmd("测试站", "机械臂复位"):
        print("error")
        return
    if not operation_cmd("测试站", "relocation"):
        print("error")
        return

    if not operation_cmd("测试站", "step4_last"):
        print("error")
        return

    if not operation_cmd("测试站", "机械臂复位"):
        print("error")
        return

    # 前往3站
    if not navigation_cmd("move", "拍2站"):
        print("error")
        return
    if not navigation_cmd("move", "拍3站"):
        print("error")
        return

    # 3站工作
    if not operation_cmd("液体1站", "机械臂复位"):
        print("error")
        return
    if not operation_cmd("液体1站", "relocation"):
        print("error")
        return
    if not operation_cmd("液体1站", "put"):
        print("error")
        return


if __name__ == '__main__':
    rospy.init_node('shao_dispatch', anonymous=True)
    time.sleep(2)
    dispatch_order()
