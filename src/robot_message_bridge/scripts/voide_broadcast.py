#!/usr/bin/env python3
import time
import pyttsx3
import re
import sys
import rospy
import json
from std_msgs.msg import String


def demo():
    engine = pyttsx3.init()
    # 获取语音包
    voices = engine.getProperty('voices')
    for voice in voices:
        print('id = {}\tname = {} \n'.format(voice.id, voice.name))
    # 设置使用的语音包
    engine.setProperty('voice', 'zh')  # 开启支持中文
    # engine.setProperty('voice', voices[0].id)

    # 改变语速  范围为0-200   默认值为200
    rate = engine.getProperty('rate')  # 获取当前语速
    engine.setProperty('rate', rate+40)

    # 设置音量  范围为0.0-1.0  默认值为1.0
    engine.setProperty('volume', 0.7)

    # 预设要朗读的文本数据
    line = "你好，张皖军"  # 要播报的内容
    engine.say(line)

    # 朗读
    engine.runAndWait()


class VoideBroadcast:
    def __init__(self):
        self.engine = pyttsx3.init()
        self.SetProperty()

    def IsAllChinese(self, strs):
        for _char in strs:
            if not '\u4e00' <= _char <= '\u9fa5':
                return False
        return True

    def IsContainsEnglish(self, str):
        my_re = re.compile(r'[A-Za-z]', re.S)
        res = re.findall(my_re, str)
        if len(res):
            return True
        else:
            return False

    def SetProperty(self, voice='zh', rate=200, volume=0.7):

        self.engine.setProperty('voice', voice)
        self.engine.setProperty('rate', rate)
        self.engine.setProperty('volume', 0.7)

    def Say(self, str):

        if self.IsContainsEnglish(str):
            self.engine.setProperty('voice', 'english')
        else:
            self.engine.setProperty('voice', 'zh')

        self.engine.say(str)
        self.engine.runAndWait()

    def voide_broadcast_callback(self, msg):
        print(msg.data.encode('utf-8').decode('utf-8'))
        self.Say(msg.data)


if __name__ == '__main__':
    rospy.init_node('voide_broadcast_node', anonymous=True)
    voide_broadcast = VoideBroadcast()

    rospy.Subscriber('/voide_broadcast_msg', String,
                     voide_broadcast.voide_broadcast_callback, queue_size=5)
    rospy.spin()
