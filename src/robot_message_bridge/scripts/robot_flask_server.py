import json
from flask import Flask, jsonify, request
from flask_executor import Executor
from robot_common_log import Log
from functools import partial

logger = Log(__name__, "flask").getlog()


class RobotFlaskServer:

    def __init__(
        self,
        dms_callback,
        nav_lock_callback,
        status_alarm_msg,
        port=3030,
        debug=False,
        host="0.0.0.0",
    ):
        self.robot_api = Flask(__name__)
        self.executor = Executor(self.robot_api)
        self.robot_port = port
        self.robot_debug = debug
        self.robot_host = host
        self.dms_callback = dms_callback
        self.nav_lock_callback = nav_lock_callback
        self.status_alarm_msg = status_alarm_msg
        # self.baselock_msg = baselock_msg

    def RunServer(self):

        self.robot_api.route('/', methods=['post'])(self.DmsPost)
        self.robot_api.route('/health', methods=['post'])(self.StatusAlarm)
        self.robot_api.route("/baselock", methods=["get", "post"])(self.BaseLock)
        
        self.robot_api.run(host=self.robot_host, port=self.robot_port, debug=self.robot_debug)
    
    def BaseLock(self):
        lock = {}
        # lock["agv_id"] = self.baselock_msg()["agv_id"]
        lock["status"] = "BUSY"
        return json.dumps(lock), 200, [("Content-type", "application/json")]

    def StatusAlarm(self):
        tmp = self.status_alarm_msg()
        electricity = tmp["battery"]
        status = tmp["status"]

        msg = {}
        msg["electricity"] = electricity
        msg["status"] = status
        # msg['exec_state'] = tmp['exec_state']
        # msg['workstation'] = tmp['workstation']
        if "ERROR" == status and electricity < 19 or electricity < 15:
            msg["code"] = 500
            msg["alarm"] = True
        else:
            msg["code"] = 200
            msg["alarm"] = False

        return jsonify(msg), msg["code"]

    def DmsPost(self):
        try:
            logger.info("收到请求")
            datas = request.get_data().decode('utf-8')
            datas = json.loads(datas)  # json字符串转为字典类型
            logger.info("收到指令%r", datas)

            if "param" in datas:
                response = json.dumps({"msg": "received successfully", "code": 200}).encode('utf-8')
                call_back = partial(self.dms_callback, datas)
                self.executor.submit(call_back)
                return response, 200, [("Content-type", "application/json")]
            elif "nav" in datas:
                self.nav_lock_callback(datas)
                response = json.dumps({"msg": "received successfully", "code": 200}).encode('utf-8')
                return response, 200, [("Content-type", "application/json")]

            else:
                response = json.dumps({
                    "msg": "received successfully,but operate code error",
                    "code": 200
                }).encode('utf-8')
                print("not normal")
                return response, 500, [("Content-type", "application/json")]

        except Exception as e:
            logger.info("请求异常")
            response = json.dumps({"msg": "some error happen: " + str(e), "code": 500}).encode('utf-8')
            return response, 500, [("Content-type", "application/json")]
