#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import serial
import numpy as np

# 二代平台夹爪别名PGI，比特率9600

if __name__ == '__main__':
    rospy.init_node('PGI_init')
    ser = serial.Serial("/dev/PGI", baudrate=115200,
                        bytesize=8, parity='N', stopbits=1)
    if(not ser.isOpen()):
        ser.open()
    data = [0x01, 0x06, 0x01, 0x00, 0x00, 0xA5, 0x48, 0x4D]
    ser.write(data)
    # ser.flushInput()
    # data1 = [0x01, 0x06, 0x05, 0x04, 0x00, 0x01, 0x09, 0x07]
    # data2 = [0x01, 0x06, 0x03, 0x00, 0x00, 0x01, 0x48, 0x4E]
    # ser.write(data1)
    # rospy.sleep(1)
    # ser.write(data2)
