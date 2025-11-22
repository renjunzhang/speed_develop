import time

import flask
import json
import logging

from flask_executor import Executor


class RobotFlaskSever:
    def __init__(self, port=3040, debug=False, host='0.0.0.0'):
        self.robot_api = flask.Flask(__name__)
        self.executor = Executor(self.robot_api)
        self.robot_port = port
        self.robot_debug = debug
        self.robot_host = host


    def RunSever(self):

        self.robot_api.route('/', methods=['post'])(self.UiOperaPost)
        self.robot_api.run(port=self.robot_port,
                           debug=self.robot_debug, host=self.robot_host)

    def OperaCallbackWarp(self):
        print(1234)
        time.sleep(10)
        print(5678)


    def UiOperaPost(self):
        try:
            datas = flask.request.get_data().decode('utf-8')
            self.opera_datas = json.loads(datas)  # json字符串转为字典类型

            logging.info("received",self.opera_datas)

            response = json.dumps(
                {"msg": "received successfully", "code": 200}).encode('utf-8')
            self.OperaCallbackWarp()

            return response,200,[("Content-type", "application/json")]

        except Exception as e:
            print(e)
            response = json.dumps(
                {"msg": "some error happen: " + str(e), "code": 500}).encode('utf-8')
            return response,200,[("Content-type", "application/json")]


if __name__ == '__main__':

    robot_flask_sever = RobotFlaskSever()
    robot_flask_sever.RunSever()
