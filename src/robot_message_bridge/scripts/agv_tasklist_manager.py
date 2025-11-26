'''
Author: LightInDust bariko@mail.ustc.edu.cn
Date: 2024-08-21 20:06:12
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2024-08-23 20:06:24
FilePath: \chem_newbase\src\robot_message_bridge\scripts\agv_tasklist_manager.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import json
import datetime

class AgvTaskListManager:
    def __init__(self, config_para, station_mapping,agv_status):
        #self.config_para = config_para
        self.station_mapping = station_mapping
        self.AGV_name = config_para["AGV_name"]
        self.AGV_url = config_para["AGV_url"]
        self.MAP_name = config_para["map"]
        self.Task_typ = config_para["taskTyp"]
        self.AGV_status = agv_status

    # 请求返回当前地图所有AGV信息
    def StatusQuest(self, hik_system):
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S") # YYYY年，MM月，DD日，HH时，MM分，SS秒
        if hik_system == "rcs_lite":
            requests = {
                "reqCode":f"agvlist123_{current_time}", # RCS-2000每个请求reqCode唯一
                #"mapCode": str(self.MAP_name) # RCS-2000
                "mapShortName": str(self.MAP_name) # RCS-lite
            }
        else:
            requests = {
                "reqCode":f"agvlist123_{current_time}", # RCS-2000每个请求reqCode唯一
                "reqTime":"",
                "clientCode":"",
                "tokenCode":"",
                "mapCode":str(self.MAP_name) # RCS-2000
                #"mapShortName": str(self.MAP_name) # RCS-lite
            }
        jrequests = json.dumps(requests)
        return jrequests
    
    # 获取指定AGV信息
    def ExtractAgvInfo(self, response_data):
        if response_data.get("code") == "0":
            for agv in response_data.get("data",[]):
                if agv["robotCode"] == self.AGV_name:
                    # 替换 status 字段
                    agv["status"] = self.AGV_status.get(agv["status"], agv["status"])  # AGV状态映射
                    return agv
        else:
            return None

    def ListQuest(self,task_codes):
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        requests = {
            "reqCode":f"tasklist123_{current_time}", #RCS-2000
            "taskCodes": str(task_codes),   #与agvCode传其一
            #"agvCode": str(self.AGV_name),
        }
        jrequests = json.dumps(requests)
        return jrequests
    
    #生成任务单
    def TaskList(self, navi_command, navi_destination):
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        if str(navi_destination) in list(self.station_mapping.keys()):
            mapping_destination = self.station_mapping[navi_destination]
        else:
            mapping_destination = "E0"
        requests = {
            "reqCode":f"pubtask1_{current_time}", # RCS-2000
            "taskTyp": str(self.Task_typ),   # 任务类型,与RCS-lite设置的任务类型一样（移动）
            #"wbCode": mapping_destination,     # 工作位
            # "positionCodePath": [{
            #                     #"positionCode": str(mapping_destination),
            #                     "positionCode": "S1",
            #                     "type": "00"
            #                     },],
            "agvCode": str(self.AGV_name),
            "robotCode": str(self.AGV_name),
            "userCallCodePath":[str(mapping_destination)]
            # "taskCode":f"pubtask_{current_time}"  # 选填
        }
        jrequests = json.dumps(requests)
        print(jrequests)
        return jrequests
    
    def ContinueTask(self):
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        requests = {
            "reqCode":f"continuetask_{current_time}", #RCS-2000
            "agvCode": str(self.AGV_name),    #与RCS-lite选择的触发方式相同
        }
        jrequests = json.dumps(requests)
        return jrequests
    
    def CheckTaskFeedback(self, resp):
        code = resp.get("code")
        if code == "0":
            return "OK"
        else:
            return "NG"
        
    #释放底盘
    def FreeRobot(self):
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

        requests = {
            "reqCode":f"pubtask2_{current_time}", # RCS-2000
            "robotCode": str(self.AGV_name)
        }
        jrequests = json.dumps(requests)
        print(jrequests)
        return jrequests
    
    #主动调用充电接口
    def GoCharge(self):
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        requests = {
            "mapCode":str(self.MAP_name), # RCS-2000
            "mapShortName":"",
            "needReqCode":"",
            "operator": 1,
            "reqCode":f"gocharge_{current_time}", # RCS-2000
            "reqTime":"",
            "robotCode": str(self.AGV_name),
            "uname":""
        }
        jrequests = json.dumps(requests)
        print(jrequests)
        return jrequests
    
    #主动取消充电接口
    def UnCharge(self):
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        requests = {
            "mapCode":str(self.MAP_name), # RCS-2000
            "mapShortName":"",
            "needReqCode":"",
            "operator": 0,
            "reqCode":f"uncharge_{current_time}", # RCS-2000
            "reqTime":"",
            "robotCode": str(self.AGV_name),
            "uname":""
        }
        jrequests = json.dumps(requests)
        print(jrequests)
        return jrequests