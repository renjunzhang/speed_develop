#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 记录传感器的力力矩程序

import rospy
from geometry_msgs.msg import WrenchStamped


if __name__ == '__main__':
    rospy.init_node('saveft', anonymous=False)
    f = open("./ftdata.csv", 'a')
    while(not rospy.is_shutdown()):
        char = input("\n输入接触状态(0,1为正负x轴旋转,2,3为正负y轴旋转)Q退出):  ")
        if(char == 'Q'):
            break
        ftwrench = rospy.wait_for_message('robotiq_ft_wrench', WrenchStamped)
        s = str(char) + "," + str(ftwrench.wrench.force.x) + "," + str(ftwrench.wrench.force.y) + "," + str(ftwrench.wrench.force.z) + \
            "," + str(ftwrench.wrench.torque.x) + "," + str(ftwrench.wrench.torque.y) + \
            "," + str(ftwrench.wrench.torque.z) + "\n"
        f.write(s)
    f.close()
