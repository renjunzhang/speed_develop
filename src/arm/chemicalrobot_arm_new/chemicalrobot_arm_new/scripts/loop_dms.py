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
    'high_flux_xrd_workstation',
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
    'high_flux_electrocatalysis_workstation',
    'uv_vis',
    'flourescence',
    'gc_workstation',
    'storage_workstation',
    'map_center',
    'charge_station']

actionList = ['reset', 'relocation', 'put', 'take', 'put_xrd', 'take_xrd',
              'put_libs', 'take_libs', 'move', 'button',
              "open_door", "close_door",
              
              "take_sample_carrier_above_platform",
              "take_dripping_head",
              "extraction",
              "dripping",
              "take_sample_carrier_from_platform",
              "move_sample_carrier_into_pool",
              "release_sample_carrier",
              "move_clamp_above_pool",
              "recycle"

              "setgripper",
              "getgripper"]

phaseList = ['start', 'reset']


def command_pub(pub,destination,action_type,station_rack_idx=0,station_slot_idx = 0,robot_rack_idx = 0,robot_slot_idx = 0):
    global done_flag
    rate = rospy.Rate(1)
    id = 0
    exp_id = 0
    stamp = (int)(time.time()*1000000)
    button_idx = -1
    phase = "-1"

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
        pub = rospy.Publisher('obsOperation_in', String, queue_size=1)
        rospy.Rate(1).sleep()

        station_slot_id = 0
        robot_slot_id = 0
        while(not rospy.is_shutdown()):
            #command_pub(pub,'storage_workstation','take')
            #command_pub(pub,'high_flux_xrd_workstation','open_door')
            #command_pub(pub,'high_flux_xrd_workstation','close_door')
            #command_pub(pub,'storage_workstation','put')

            command_pub(pub,'high_flux_electrocatalysis_workstation','take_sample_carrier_above_platform',0,station_slot_id,0,robot_slot_id)
            command_pub(pub,'high_flux_electrocatalysis_workstation','take_sample_carrier_from_platform',0,station_slot_id,0,robot_slot_id)
            command_pub(pub,'high_flux_electrocatalysis_workstation','move_sample_carrier_into_pool',0,station_slot_id,0,robot_slot_id)
            command_pub(pub,'high_flux_electrocatalysis_workstation','release_sample_carrier',0,station_slot_id,0,robot_slot_id)
            command_pub(pub,'high_flux_electrocatalysis_workstation','move_clamp_above_pool',0,station_slot_id,0,robot_slot_id)
            command_pub(pub,'high_flux_electrocatalysis_workstation','recycle',0,station_slot_id,0,robot_slot_id)

            station_slot_id = (station_slot_id + 1) % 5
            if(station_slot_id == 3):
                station_slot_id += 1

    except rospy.ROSInterruptException:
        pass
