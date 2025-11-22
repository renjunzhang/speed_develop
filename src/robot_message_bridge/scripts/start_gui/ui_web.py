#! /usr/bin/env python3

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWebEngineWidgets import *
import requests
import json


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.ui_url = "http://0.0.0.0:3040"
        self.setWindowTitle('小来')  # 窗口标题
        self.setGeometry(5, 30, 1000, 730)  # 窗口的大小和位置设置
        self.browser = QWebEngineView()
        # 加载外部的web界面
        self.browser.load(QUrl('http://0.0.0.0:8081'))
        self.setCentralWidget(self.browser)

    def closeEvent(self, event):

        oper_messages = {"operation": "Stop", "detail": ""}
        try:
            response = requests.post(
                self.ui_url, json.dumps(oper_messages).encode("utf-8"), headers={"content-type": "application/json"})
            print(response.text)
        except Exception as e:
            print(e)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    app.exit(app.exec_())
