import flask
import json
from flask_executor import Executor
from robot_common_log import Log

logger = Log(__name__, "flask").getlog()

class HikFlaskServer:
    def __init__(self, port=3030, debug=False, host="127.0.0.1"):
        self.robot_api = flask.Flask(__name__)
        self.executor = Executor(self.robot_api)
        self.robot_port = port
        self.robot_debug = debug
        self.robot_host = host

        self.agv_status = ""

    def RunServer(self):
        # http://IP:PORT/xxx/agv/agvCallbackService/agvCallback
        self.robot_api.route("/xxx/agv/agvCallbackService/agvCallback", methods=["post"])(self.AgvCallback)

        # http://IP:PORT/service/rest/agvCallbackService/warnCallback
        self.robot_api.route("/service/rest/agvCallbackService/warnCallback", methods=["post"])(self.WarnCallback)

        self.robot_api.run(port=self.robot_port, debug=self.robot_debug, host=self.robot_host)

    def AgvCallback(self):
        try:
            datas = flask.request.get_data().decode("utf-8")
            datas = json.loads(datas)  # 将请求数据解析为字典
            logger.info("收到AGV回调请求: %r", datas)

            # 判断任务执行的method
            if "method" in datas and datas["method"] == "start":
                logger.info("任务开始")
                self.agv_status = "start"
            elif "method" in datas and datas["method"] == "outbin":
                logger.info("走出储位")
                self.agv_status = "outbin"
            elif "method" in datas and datas["method"] == "end":
                logger.info("任务结束")
                self.agv_status = "end"

            # 返回成功响应
            response = json.dumps({"code": "0", "message": "成功", "reqCode": datas.get("reqCode", "")})
            return response, 200, [("Content-type", "application/json")]

        except Exception as e:
            logger.error("AGV回调请求异常: %s", str(e))
            response = json.dumps({"code": "500", "message": "请求异常", "reqCode": ""})
            return response, 500, [("Content-type", "application/json")]

    def WarnCallback(self):
        try:
            datas = flask.request.get_data().decode("utf-8")
            datas = json.loads(datas)  # 将请求数据解析为字典
            logger.info("收到告警回调请求: %r", datas)

            # 打印告警请求的data
            if "data" in datas:
                for warn in datas["data"]:
                    logger.warning("告警内容: %s, 任务代码: %s", warn["warnContent"], warn["taskCode"])

            # 返回成功响应
            response = json.dumps({"code": "0", "message": "成功", "reqCode": datas.get("reqCode", "")})
            return response, 200, [("Content-type", "application/json")]

        except Exception as e:
            logger.error("告警回调请求异常: %s", str(e))
            response = json.dumps({"code": "500", "message": "请求异常", "reqCode": ""})
            return response, 500, [("Content-type", "application/json")]
        
if __name__ == "__main__":
    server = HikFlaskServer(port=8801, debug=False, host="192.168.10.50")
    server.RunServer()