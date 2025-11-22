# RCS 接口调用系统说明文档

## 一、系统概述

本系统是一个基于 ROS 的机器人调度管理系统，主要实现了**化学机器人**与**海康 RCS 调度系统**之间的通信与协作。系统由两个核心文件组成：

1. **`agv_tasklist_manager.py`** - AGV 任务列表管理器（底盘操作封装）
2. **`dms_to_robot.py`** - DMS 到机器人的消息管理器（主逻辑流程）

---

## 二、文件详细说明

### 2.1 `agv_tasklist_manager.py` - AGV 任务列表管理器

#### 2.1.1 核心功能
该文件封装了与**海康 RCS 调度系统**交互的所有底盘相关操作，是一个工具类。

#### 2.1.2 主要方法

| 方法名 | 功能描述 | 返回值 |
|--------|---------|--------|
| `StatusQuest()` | 构建查询当前地图所有 AGV 信息的请求 JSON | JSON 字符串 |
| `ExtractAgvInfo()` | 从 RCS 返回的响应中提取指定 AGV 的信息 | AGV 信息字典或 None |
| `TaskList()` | 生成移动任务单（指定目标工作站） | JSON 字符串 |
| `ChargeTask()` | 生成充电任务单 | JSON 字符串 |
| `FreeRobotTask()` | 生成释放机器人的请求 | JSON 字符串 |
| `ListQuest()` | 构建查询任务状态的请求 | JSON 字符串 |
| `CheckTaskFeedback()` | 检查任务反馈是否成功 | "OK" 或 "NG" |

#### 2.1.3 配置依赖
- `param.json` - 基础配置（AGV 名称、URL、地图名、任务类型）
- `station_mapping.json` - 工作站映射表（逻辑名 → RCS 物理点位）
- `status.json` - AGV 状态码映射表

#### 2.1.4 关键设计
- **RCS 版本兼容**：同时支持 RCS-2000 和 RCS-lite（通过 `hik_system` 参数区分）
- **请求唯一性**：每个请求都带有时间戳的唯一 `reqCode`
- **状态映射**：将 RCS 返回的状态码转换为系统内部状态

---

### 2.2 `dms_to_robot.py` - DMS 消息管理器

#### 2.2.1 核心功能
这是系统的**主控制逻辑**文件，负责：
- 接收 DMS（数据管理系统）下发的任务
- 协调底盘移动和机械臂操作
- 管理机器人状态并上报心跳
- 处理异常和中断

#### 2.2.2 核心类 `RobotMsgManager`

##### 初始化 (`__init__`)
1. 加载配置文件：
   - `param.json` - 基本配置
   - `workstataion_operations.json` - 工作站操作定义
   - `station_mapping.json` - 站点映射
   - `status.json` - 状态映射

2. 初始化 Flask 服务器（接收 DMS 指令）
3. 初始化 AGV 任务管理器
4. 设置心跳消息结构

##### 主要方法分类

**1. DMS 回调处理**
```python
dms_callback(operate_datas)  # 核心入口：处理 DMS 下发的操作指令
```
- 等待机器人空闲
- 遍历 `param` 中的每个操作
- 调用 `determine_destination()` 判断目标工作站
- 调用 `process_operation()` 生成操作字符串
- 根据当前位置决定是否移动底盘
- 执行机械臂操作
- 上报状态

**2. 底盘相关操作**
```python
PubAgvTask(navi_command, navi_destination)  # 发布移动任务
TaskPeriod(feedback_msg)                     # 轮询任务状态直到完成
PubFreeRobot()                               # 释放机器人（从任务队列中移除）
PubChargeTask()                              # 发布充电任务
navi_serve_callback()                        # 查询 AGV 当前状态
```

**3. 机械臂相关操作**
```python
OperAction(oper_msg, generate_msg_flag)  # 发布机械臂操作指令并等待反馈
oper_serve_callback()                     # 查询机械臂状态
oper_msg_gen(cur_dest, cur_oper)         # 生成机械臂操作消息
```

**4. 状态管理**
```python
SendHreartbeatMsg()          # 发送心跳到 DMS
update_status()              # 定时更新机器人状态（10Hz）
GetStatusAlarmMsg()          # 获取状态告警消息
generate_and_send_status()   # 生成并发送状态反馈到 DMS
```

**5. 操作字符串解析**
```python
determine_destination(operations)                        # 判断目标工作站
build_operation_string(origin_datas, robot_code)        # 构建操作字符串
extract_workstation_and_operation(operation_string)     # 解析操作字符串
process_operation(operations)                           # 处理操作并返回具体指令
```

#### 2.2.3 状态机设计

机器人状态流转：
```
IDLE → BUSY → IDLE
  ↓      ↑
CHARGING ERROR
```

- **IDLE**：空闲，可接受新任务
- **BUSY**：执行任务中（移动或操作）
- **CHARGING**：充电中
- **ERROR**：故障状态

#### 2.2.4 空闲自动释放机制
代码中实现了"空闲看门狗"：
- 当机器人连续空闲 **600 秒**（10 分钟）
- 且距离上次释放已过 **1200 秒**（冷却期）
- 自动调用 `PubFreeRobot()` 释放机器人

#### 2.2.5 多线程架构
系统启动三个线程：
1. **心跳线程**：`heartbeat_monitoring()` - 1Hz 发送心跳
2. **状态更新线程**：`update_status()` - 10Hz 更新状态
3. **Flask 服务器线程**：`DmsFlaskHttpSever()` - 接收 DMS 指令

---

## 三、工作流程详解

### 3.1 任务执行标准流程

```
1. DMS 下发任务 → Flask 服务器接收
   ↓
2. dms_callback() 接收并解析任务
   ↓
3. 等待机器人状态为 IDLE
   ↓
4. 遍历 param 中的每个操作
   ↓
5. determine_destination() 判断目标站点
   ↓
6. process_operation() 生成操作指令
   ↓
7. 判断是否需要移动：
   
   【情况A：当前站点 = 目标站点】
   - 直接执行机械臂操作
   
   【情况B：当前站点 ≠ 目标站点】
   - 7.1 机械臂复位
   - 7.2 调用 PubAgvTask() 发布移动任务
   - 7.3 调用 TaskPeriod() 轮询任务状态
   - 7.4 移动完成后，机械臂复位
   - 7.5 机械臂重定位（relocation）
   - 7.6 执行目标操作
   ↓
8. 所有操作完成后最终复位
   ↓
9. 上报 "done" 状态到 DMS
```

### 3.2 异常处理流程

- **中断检测**：每次操作后检查 `state == "interrupt"`
- **错误上报**：失败时调用 `generate_and_send_status(500)`
- **互斥锁保护**：`callback_fun_mutex` 防止并发执行

---

## 四、关键配置文件说明

### 4.1 `param.json`
```json
{
  "robotCode": "robot_01",          // 机器人代码
  "AGV_name": "AGV_01",             // AGV 名称
  "AGV_url": "http://...",          // RCS 接口地址
  "dms_url": "http://...",          // DMS 心跳地址
  "response_url": "http://...",     // DMS 状态反馈地址
  "map": "map_01",                  // 地图名称
  "taskTyp": "move",                // 任务类型
  "identifyingCode": "xxx"          // 识别码
}
```

### 4.2 `station_mapping.json`
```json
{
  "centrifuge_station": "S1",       // 离心机站点 → RCS点位S1
  "balance_station": "S2",          // 天平站点 → RCS点位S2
  "charge_station": "C1"            // 充电站 → RCS点位C1
}
```

### 4.3 `workstataion_operations.json`
```json
{
  "param": [
    {
      "workstation": "centrifuge_station",
      "operations": {
        "move-w_centrifuge_station-to-r_robot_01-oper-tube_1-and-rack_2": "pick_tube_1_to_rack_2"
      }
    }
  ]
}
```

### 4.4 `status.json`
```json
{
  "1": "Idle_task",              // 空闲
  "2": "Executing_task",         // 执行中
  "3": "Charging_status",        // 充电中
  "9": "Task_completed",         // 任务完成
  "10": "Abnormal_task"          // 异常
}
```

---

## 五、后续修改指南

### 5.1 添加新的工作站

**步骤：**
1. 在 `station_mapping.json` 中添加映射：
   ```json
   "new_station": "S10"
   ```

2. 在 `workstataion_operations.json` 中添加操作定义：
   ```json
   {
     "workstation": "new_station",
     "operations": {
       "move-w_new_station-to-r_robot_01-oper-xxx": "specific_operation"
     }
   }
   ```

3. 确保 RCS 系统中已配置点位 `S10`

### 5.2 修改操作逻辑

**场景：添加新的操作类型**

1. 在 `determine_destination()` 中添加判断逻辑
2. 在 `process_operation()` 中添加操作字符串构建规则
3. 在机械臂控制节点中实现具体操作

### 5.3 调整状态机行为

**场景：修改空闲释放时间**

在 `dms_to_robot.py` 的 `__init__` 中修改：
```python
self.auto_free_cooldown = 1200     # 冷却时间（秒）
```

在 `update_status()` 中修改：
```python
if idle_duration >= 600 and ...    # 修改为目标秒数
```

### 5.4 添加新的 RCS 接口

**步骤：**
1. 在 `agv_tasklist_manager.py` 中添加新方法：
   ```python
   def NewTask(self):
       current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
       requests = {
           "reqCode": f"newtask_{current_time}",
           "agvCode": str(self.AGV_name),
           # 添加其他必需字段
       }
       return json.dumps(requests)
   ```

2. 在 `dms_to_robot.py` 中调用：
   ```python
   def PubNewTask(self):
       jreq = self.agvlist.NewTask()
       resp = requests.post(
           self.AGV_url + "newTaskEndpoint",
           jreq.encode("utf-8"),
           headers=self.headers,
           timeout=REQUESTTIMEOUT
       )
       # 处理返回值
   ```

### 5.5 修改心跳频率

在 `dms_to_robot.py` 中：
- `heartbeat_monitoring()` 中的 `rospy.Rate(1)` 改为目标频率
- `update_status()` 中的 `rospy.Rate(10)` 改为目标频率

### 5.6 调试建议

1. **查看日志**：
   ```bash
   tail -f ~/speed_develop/chemrob_hikbase/Logger_file/*.log
   ```

2. **ROS 话题监控**：
   ```bash
   rostopic echo /obsOperation_out    # 机械臂操作反馈
   rostopic echo /obsOperation_in     # 机械臂操作指令
   ```

3. **手动测试 RCS 接口**：
   ```python
   import requests
   # 测试查询 AGV 状态
   resp = requests.post(
       "http://192.168.200.157:8182/rcms-dps/rest/queryAgvStatus",
       data=jreq,
       headers={"content-type": "application/json"}
   )
   ```

---

## 六、常见问题与解决方案

### 6.1 机器人长时间 BUSY 无法接受任务
**原因**：可能卡在某个操作或互斥锁未释放
**解决**：
- 检查 `callback_fun_mutex` 是否正常释放
- 查看日志中最后执行的操作
- 手动重启节点：`rosnode kill /robot_msg_manager_node`

### 6.2 底盘移动任务一直 Pending
**原因**：RCS 系统未接单或路径规划失败
**解决**：
- 检查 RCS Web 界面中的任务状态
- 调用 `queryTaskStatus` 查看详细错误
- 检查目标点位是否在地图中存在

### 6.3 机械臂操作失败
**原因**：操作字符串不匹配或机械臂节点异常
**解决**：
- 检查 `workstataion_operations.json` 中是否定义了该操作
- 确认机械臂节点 `/chemicalrobot_arm_new` 正常运行
- 查看机械臂日志

### 6.4 心跳断开
**原因**：网络问题或 DMS 服务不可达
**解决**：
- 检查 `param.json` 中的 `dms_url` 是否正确
- 测试网络连通性：`ping <dms_ip>`
- 查看异常日志

---

## 七、系统架构图

```
┌─────────────────────────────────────────────────────────┐
│                        DMS 系统                          │
│            (任务下发 & 状态接收 & 心跳监听)                │
└────────────┬─────────────────────────────┬──────────────┘
             │ HTTP POST                   │ HTTP POST
             │ (任务指令)                   │ (心跳/状态)
             ↓                             ↑
┌────────────────────────────────────────────────────────┐
│              dms_to_robot.py (主控)                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Flask Server (接收DMS)  ROS Node (发布/订阅)    │  │
│  └──────────────────────────────────────────────────┘  │
│         ↓                                    ↑          │
│  ┌──────────────┐                  ┌──────────────┐    │
│  │ dms_callback │                  │ update_status│    │
│  └──────────────┘                  └──────────────┘    │
│         ↓                                               │
│  ┌──────────────────────────────────────────────────┐  │
│  │      agv_tasklist_manager.py (底盘封装)          │  │
│  └──────────────────────────────────────────────────┘  │
└──────┬─────────────────────────┬────────────────────────┘
       │ HTTP POST               │ ROS Topic/Service
       │ (RCS接口)               │ (/obsOperation_in/out)
       ↓                         ↓
┌─────────────────┐       ┌──────────────────┐
│  海康 RCS 系统   │       │  机械臂控制节点   │
│  (底盘调度)      │       │ (/chemicalrobot  │
│                 │       │  _arm_new)       │
└─────────────────┘       └──────────────────┘
       ↓                         ↓
┌─────────────────┐       ┌──────────────────┐
│    AGV 底盘      │       │     UR5e 机械臂   │
└─────────────────┘       └──────────────────┘
```

---

## 八、版本信息

- **作者**：LightInDust (bariko@mail.ustc.edu.cn)
- **创建日期**：2024-08-21
- **最后修改**：2024-08-23
- **ROS 版本**：ROS Noetic
- **Python 版本**：Python 3
- **海康 RCS 版本**：支持 RCS-2000 和 RCS-lite

---

## 九、许可与联系

如有问题请联系开发团队或查看项目 README.md 文件。

**文档生成日期**：2025年11月22日
