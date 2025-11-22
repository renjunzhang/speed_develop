###############################################################
##工作站通信代码，用于任务管理模块运行使用##
###############################################################

import socket
import time
import datetime


class WrappedSocket:
    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def send(self, value):
        retry_count = 0
        while retry_count <= 2:
            try:
                print("before send..." + value.decode('utf-8'), datetime.datetime.now())
                self._socket.send(value)
                print("send succeed..." + value.decode('utf-8'), datetime.datetime.now())
                break
            except Exception as expt:
                print(expt)
                retry_count = +1
                time.sleep(5)

    def connect(self, host):
        retry_count = 0
        while retry_count <= 2:
            try:
                print("before connect..." + str(host[0]) + " port: " + str(host[1]), datetime.datetime.now())
                self._socket.connect(host)
                print("end connect..." + str(host[0]) + " port: " + str(host[1]), datetime.datetime.now())
                break
            except Exception as expt:
                print(expt)
                retry_count = +1
                time.sleep(5)

    def settimeout(self, seconds=None):
        self._socket.settimeout(seconds)

    def close(self):
        self._socket.close()

    def recv(self, value):
        return self._socket.recv(value)


def get_socket():
    return WrappedSocket()
