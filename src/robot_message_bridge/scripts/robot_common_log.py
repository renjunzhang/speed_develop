#!/usr/bin/python
# -*- coding:utf-8 -*-

import logging
import time
import os
import rospkg
from logging.handlers import TimedRotatingFileHandler
import re


class Log(object):
    """
    封装后的logging
    """

    def __init__(self, logger=None, file_name=None):
        """
        指定保存日志的文件路径，日志级别，以及调用文件
        将日志存入到指定的文件中
        """

        # 创建一个logger
        self.logger = logging.getLogger(logger)
        self.logger.setLevel(logging.INFO)
        # 创建一个handler，用于写入日志文件
        self.log_time = time.strftime("%Y_%m_%d")
        if not os.path.exists("./Logger_file/"):
            os.makedirs("./Logger_file/")
        # 设置Log文件格式
        rospack = rospkg.RosPack()
        if not file_name:
            log_file_name = (
                rospack.get_path("robot_message_bridge") + "/Logger_file/Robot_manager"
            )
        else:
            log_file_name = (
                rospack.get_path("robot_message_bridge") + "/Logger_file/" + file_name
            )

        self.log_path = log_file_name
        
        fh = TimedRotatingFileHandler(
            filename=self.log_path, when="MIDNIGHT", interval=1, backupCount=3
        )
        # filename="mylog" suffix设置，会生成文件名为mylog.2020-02-25.log
        fh.suffix = "%Y-%m-%d.log"
        # extMatch是编译好正则表达式，用于匹配日志文件名后缀
        # 需要注意的是suffix和extMatch一定要匹配的上，如果不匹配，过期日志不会被删除。
        fh.extMatch = re.compile(r"^\d{4}-\d{2}-\d{2}.log$")

        fh.setLevel(logging.INFO)

        # 再创建一个handler，用于输出到控制台
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)

        # 定义handler的输出格式
        formatter = logging.Formatter(
            "[%(asctime)s] %(filename)s [%(levelname)s]%(message)s"
        )
        # formatter = logging.Formatter(
        # '[%(asctime)s] %(filename)s->%(funcName)s line:%(lineno)d [%(levelname)s]%(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)

        # 给logger添加handler
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)

        # 关闭打开的文件
        fh.close()
        ch.close()

    def getlog(self):
        return self.logger
