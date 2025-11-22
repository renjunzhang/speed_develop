import flask
import json
import logging
import threading
from flask_executor import Executor


class UiFlaskSever:
    def __init__(self,  http_sever_callback, pub_status, port=3040, debug=False, host='192.168.1.146'):
        self.robot_api = flask.Flask(__name__)
        self.executor = Executor(self.robot_api)
        self.robot_port = port
        self.robot_debug = debug
        self.robot_host = host
        self.http_sever_callback = http_sever_callback
        self.pub_status = pub_status

        self.query_mutex = threading.Lock()

    def RunSever(self):

        self.robot_api.route('/', methods=['post'])(self.UiOperaPost)
        self.robot_api.run(port=self.robot_port,
                           debug=self.robot_debug, host=self.robot_host)

    def UiOperaPost(self):
        try:
            datas = flask.request.get_data().decode('utf-8')
            opera_datas = json.loads(datas)  # json字符串转为字典类型
            status_code = 200

            print("received", opera_datas)

            if 'operation' in opera_datas and 'detail' in opera_datas:
                # self.executor.submit(self.OperaCallbackWarp)异步执行
                response_opera_message = self.http_sever_callback(
                    opera_datas)
            elif "query" in opera_datas:
                with self.query_mutex:
                    response_opera_message = self.pub_status()
            else:
                response_opera_message = {
                    "message": "error", "detail": "没有操作指令"}
                status_code = 500

            return json.dumps(response_opera_message).encode('utf-8'), status_code, [("Content-type", "application/json")]

        except Exception as e:
            response = json.dumps(
                {"message": "error", "detail": "异常错误"}).encode('utf-8')
            return response, 500, [("Content-type", "application/json")]


if __name__ == '__main__':
    data = {}

    def dms_callback():
        pass
    robot_flask_sever = UiFlaskSever(
        dms_callback, port=3040, debug=False, host='192.168.1.146')
