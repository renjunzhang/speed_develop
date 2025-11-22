#!/usr/bin/env python3

import typing
import roslaunch
# from PyQt5.QtWidgets import *
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from ui_main import Ui_Form
import sys
import json
import requests
import flask
from PyQt5 import QtWidgets
from PyQt5 import QtCore
import threading
from copy import deepcopy
from queue import Queue
import time


class ServerThread(QThread):
    my_signal = pyqtSignal(str)     # 1

    def __init__(self):
        super(ServerThread, self).__init__()
        self.ui_url = "http://0.0.0.0:3040"

    def run(self):
        while True:
            query_messages = {"query": "msg"}
            try:
                response = requests.post(
                    self.ui_url, json.dumps(query_messages).encode("utf-8"), headers={"content-type": "application/json"})
                if 200 == response.status_code:
                    self.my_signal.emit(deepcopy(response.text))
            except Exception as e:
                print("query request error")
            time.sleep(5)

    def GetStatus(self):
        status_json = flask.request.get_data().decode('utf-8')
        self.my_signal.emit(deepcopy(status_json))

        # for state_key,state_value in status.items():
        #     self.SetStatus(state_key,state_value)
        response = json.dumps(
            {"msg": "received successfully", "code": 200}).encode('utf-8')

        return response, 200, [("Content-type", "application/json")]


class MyMainWindow(QtWidgets.QWidget, Ui_Form, QThread):
    def __init__(self):
        super(MyMainWindow, self).__init__()
        self.setupUi(self)

        # Form.setFixedSize(Form.width(), Form.height())
        # MyMainWindow.setFixedSize(686, 555)

        self.progressBar.setMinimum(-1)  # 设置进度条的最小值
        self.progressBar.setMaximum(100)  # 设置进度条的最大值
        self.progressBar.setValue(80)
        self.ui_url = "http://0.0.0.0:3040"

        self.color_dict = {-1: "background-color: rgb(85, 85, 127);",
                           0: "background-color: rgb(255, 0, 0);",
                           1: "background-color: rgb(0, 255, 0);"}

        self.label_dict = {"arm": self.label_arm, "camera": self.label_camera,
                           "platform": self.label_platform, "radar": self.label_radar}

        self.server_thread = ServerThread()
        self.server_thread.my_signal.connect(self.ShowStatus)
        self.server_thread.start()

    def StartStop(self, status):
        if status:
            self.pushButton.setCheckable(not status)
            oper_messages = {"operation": "Stop", "detail": ""}
            response = requests.post(
                self.ui_url, json.dumps(oper_messages).encode("utf-8"), headers={"content-type": "application/json"})
            print(response.text)
            self.pushButton.setText("开始")
        else:
            oper_messages = {"operation": "Start", "detail": ""}
            response = requests.post(
                self.ui_url, json.dumps(oper_messages).encode("utf-8"), headers={"content-type": "application/json"})
            print(response.text)
            self.pushButton.setCheckable(not status)
            self.pushButton.setText("停止")

    def ArmRest(self):
        oper_messages = {"operation": "ArmReset", "detail": ""}
        response = requests.post(
            self.ui_url, json.dumps(oper_messages).encode("utf-8"), headers={"content-type": "application/json"})
        print(response.text)

    def Grip(self):
        oper_messages = {"operation": "GripReset", "detail": ""}
        response = requests.post(
            self.ui_url, json.dumps(oper_messages).encode("utf-8"), headers={"content-type": "application/json"})
        print(response.text)

    def SetStatus(self, key, value):
        if "error_messages" == key:
            if "" != value:
                self.label_error_messages.setText(value)
        elif"electricityQuantity" == key:
            # self.label_error_messages.setText("电量"+str(value))
            self.progressBar.setValue(int(value))
        else:
            # promos = [globals()[name] for name in globals()
            #           if name.endswith("label_"+key)]
            # if 0==len(promos):
            #     print("没找到标签")
            # else:
            #     promos[0].setStyleSheet(self.color_dict[value])

            if key in self.label_dict:
                self.label_dict[key].setStyleSheet(self.color_dict[value])

    def ShowStatus(self, status_json):

        status = json.loads(status_json)  # json字符串转为字典类型
        # print(status)
        for state_key, state_value in status.items():
            self.SetStatus(state_key, state_value)

    # 关窗口事件
    def closeEvent(self, event):

        oper_messages = {"operation": "Stop", "detail": ""}
        try:
            response = requests.post(
                self.ui_url, json.dumps(oper_messages).encode("utf-8"), headers={"content-type": "application/json"})
            print(response.text)
        except Exception as e:
            print(e)

        if self.server_thread.isRunning():
            self.server_thread.quit()


if __name__ == '__main__':
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QtWidgets.QApplication(sys.argv)
    myshow = MyMainWindow()
    myshow.show()
    sys.exit(app.exec_())
