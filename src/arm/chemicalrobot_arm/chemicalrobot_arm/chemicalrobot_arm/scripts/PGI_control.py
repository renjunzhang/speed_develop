#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import serial
import numpy as np

def CalCRC(data):
    wcrc = 0xFFFF
    for each in data:
        temp = each & 0X00FF
        wcrc ^= temp
        for i in range(8):
            if wcrc & 0X0001:
                wcrc = wcrc >> 1
                wcrc ^= 0XA001
            else:
                wcrc >>= 1
    CRC_L = wcrc&0xFF
    CRC_H = wcrc >> 8
    data.append(CRC_L)
    data.append(CRC_H)


if __name__ == '__main__':
    print('该程序已弃用，请运行')
    print('rosrun pgi140 pgitest')
    # rospy.init_node('PGI_control')
    # ser = serial.Serial("/dev/PGI", baudrate=115200,
    #                     bytesize=8, parity='N', stopbits=1)
    # if(not ser.isOpen()):
    #     ser.open()
    # while(not rospy.is_shutdown()):
    #     pos = np.uint16(input("position:\n"))
    #     data = [0x01,0x06,0x01,0x03,np.uint8(pos>>8),np.uint8(pos&0xFF)]
    #     CalCRC(data)
    #     ser.write(data)
