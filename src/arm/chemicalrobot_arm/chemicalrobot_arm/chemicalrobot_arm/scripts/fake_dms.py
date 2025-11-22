#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import time

from std_msgs.msg import String

done_flag = False


stationsList = [
    'starting_station',
    'infrared_spectrum',
    'libs',
    'xrd',
    'raman_spectra',
    'muffle_station',
    'imbibition_workstation',
    'liquid_dispensing',
    'solid_dispensing',
    'magnetic_stirring',
    'capping_station',
    'dryer_workstation',
    'new_centrifugation',
    'photocatalysis_workstation',
    'electrocatalysis_workstation',
    'uv_vis',
    'flourescence',
    'gc',
    'ir',
    'ultrasonic_cleaner']

actionList = ['reset', 'relocation', 'put', 'take', 'put_xrd', 'take_xrd',
              'put_libs', 'take_libs', 'move', 'button',
              "opendoor", "closedoor",
              "get_clamp", "get_carbon_paper",
              "move_to_safe_from_carbon_paper",
              "move_carbon_paper_to_dripping_station",
              "move_carbon_paper_to_drying_platform",
              "move_to_safe_from_drying_platform_with_no_matter",
              "reset_clamp",

              "get_clamp_again",
              "get_carbon_paper_from_drying_platform",
              "move_to_safe_from_drying_platform_with_matter",
              "move_carbon_paper_to_pool",
              "move_to_safe_from_pool",
              "reset_clamp",

              "get_clamp_again",
              "get_carbon_paper_from_pool",
              "move_to_safe_from_pool",
              "move_carbon_paper_to_recycle",
              "move_to_safe_from_recycle",
              "reset_clamp",
              "put_ir",
              "take_ir"]

phaseList = ['start', 'reset']


def command_pub():
    global done_flag
    rate = rospy.Rate(1)
    pub = rospy.Publisher('obsOperation_in', String, queue_size=1)

    num = len(actionList)
    tips = ""
    button_idx = "-1"
    phase = "-1"

    for i in range(num):
        tips = tips + str(i)+":"+actionList[i]+"\n"

    while(not rospy.is_shutdown()):

        for index, item in enumerate(stationsList):
            print('%d:%s' % (index, item))

        char = input("\n输入工作站序号: ")
        station_num = int(char) if char else 0
        destination = stationsList[station_num]

        char = input("\n输入id: ")
        id = int(char) if char else 0

        char = input("\n输入exp id: \n")
        exp_id = int(char) if char else 0

        stamp = (int)(time.time()*1000000)

        ###########################################################
        char = input("\n输入动作类型\n"+tips)
        action_num = int(char) if char else 0
        action_type = actionList[action_num]
        obsOperation_in = ""
        if(action_type == 'reset' or action_type == 'relocation'):
            obsOperation_in = {"stamp": str(
                stamp), "destination": destination, "operation": action_type}
        else:
            if(action_type == 'button'):
                button_idx = (int)(input("\n输入button_idx: "))

            char = input("\n输入station_rack_idx: ")
            station_rack_idx = int(char) if char else 0

            char = input("\n输入station_slot_idx: ")
            station_slot_idx = int(char) if char else 0

            char = input("\n输入robot_rack_idx: ")
            robot_rack_idx = int(char) if char else 0

            char = input("\n输入robot_slot_idx: ")
            robot_slot_idx = int(char) if char else 0

            ###
            obsOperation_in = {
                "id": str(id), "exper_no": str(exp_id), "stamp": str(stamp), "destination": destination,
                "operations": [
                    {
                        "operation": action_type,
                        "rack_station": str(station_rack_idx),
                        "slot_station": str(station_slot_idx),
                        "button_index": str(button_idx),
                        "rack_robot": str(robot_rack_idx),
                        "slot_robot": str(robot_slot_idx),
                        "phase": phase
                    }
                ]}

        operation_in = String(json.dumps(
            obsOperation_in, sort_keys=True, indent=4, separators=(',', ': ')))
        print('=============pub msg===============')
        print(operation_in.data)
        print('============================')
        done_flag = False
        pub.publish(operation_in)
        while(not done_flag and not rospy.is_shutdown()):
            rate.sleep()


def feedback_callback(msg):
    global done_flag
    print('=============rev msg===============')
    print(msg.data)
    print('============================')
    done_flag = True


if __name__ == '__main__':
    rospy.init_node('fake_command', anonymous=True)
    try:
        rospy.Subscriber('obsOperation_out', String, feedback_callback)
        command_pub()
    except rospy.ROSInterruptException:
        pass
