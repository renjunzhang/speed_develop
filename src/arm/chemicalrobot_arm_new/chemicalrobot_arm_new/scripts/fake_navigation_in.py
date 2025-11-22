#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import rospy
import json
from std_msgs.msg import String

done_flag = False


def command_pub():
    global done_flag
    rate = rospy.Rate(1)
    pub = rospy.Publisher("/obsNavigation_in", String, queue_size=10)

    # rotation_recovery
    while(not rospy.is_shutdown()):
        nav_cmd = input()
        if "0" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "uv_vis", "action": "charge"}
        elif "1" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "uv_vis", "action": "uncharge"}
        elif "2" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "starting_station", "action": "move"}
        elif "3" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "storage_workstation", "action": "move"}
        elif "4" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "new_centrifugation", "action": "move"}
        elif "5" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "magnetic_stirring", "action": "move"}
        elif "6" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "magnetic_stirring_2", "action": "move"}
        elif "7" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "photocatalysis_workstation", "action": "move"}
        elif "8" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "gc", "action": "move"}
        elif "9" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "dryer_workstation", "action": "move"}
        elif "10" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "imbibition_workstation", "action": "move"}
        elif "11" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "photocatalysis_workstation_2", "action": "move"}
        elif "12" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "liquid_dispensing", "action": "move"}
        elif "13" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "solid_dispensing", "action": "move"}
        elif "14" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "map_center", "action": "move"}
        elif "15" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "capping_station", "action": "move"}
        elif "16" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "manual_workstation", "action": "move"}
        elif "17" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "charge_station", "action": "move"}
        elif "18" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "magnetic_stirring_1", "action": "move"}
        elif "19" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "solid_dispensing_1", "action": "move"}
        elif "20" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "ultrasonic_station", "action": "move"}
        # obsNavigation_in = {
        #         "action": "move", "dest_station": "XRD工作站"}
        elif "21" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "shake_station", "action": "move"}
        elif "22" == nav_cmd:
            obsNavigation_in = {
                "id": "0001", "exper_no": "1", "stamp": "1212",
                "destination": "dryer_workstation_1", "action": "move"}
        navigation_in = String(json.dumps(
            obsNavigation_in, sort_keys=True, indent=4, separators=(',', ': ')))
        print('=============pub msg===============')
        print(navigation_in.data)
        print(json.dumps(obsNavigation_in, sort_keys=True,
                         indent=4, separators=(',', ': ')))
        time.sleep(1)
        print('============================')
        done_flag = False
        pub.publish(navigation_in)


def feedback_callback(msg):
    global done_flag
    print('=============rev msg===============')
    print(msg.data)
    print('============================')
    done_flag = True


if __name__ == '__main__':
    rospy.init_node('fake_navigation_in', anonymous=True)
    try:
        rospy.Subscriber('/obsNavigation_out', String, feedback_callback)
        command_pub()
    except rospy.ROSInterruptException:
        pass
    # rospy.spin()
