#ifndef MOTION_PLANNING_H
#define MOTION_PLANNING_H
#include "chemicalrobot_arm/DmsService.h"
#include <actionlib_msgs/GoalID.h>
#include <algorithm>
#include <chemicalrobot_arm/dashboard.h>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/GetMotionSequence.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/MotionSequenceItem.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <pgi140/pgi_control.h>
#include <regex>
#include <robotiq_ft_sensor/sensor_accessor.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <time.h>
using namespace std;
using namespace Eigen;

#define M_PI 3.14159265358979323846

// 近距离移动速度,用于LIN规划
#define CLOSE_SPEED 0.1

// 近距离加速度,用于LIN规划
#define CLOSE_ACC 0.05

// 正常移动速度,用于PTP规划
#define NORMAL_SPEED 0.15

// 正常加速度,用于PTP规划
#define NORMAL_ACC 0.1

// 单个试管架西林瓶数量
#define HOLE_MAX 10

/**
 * @brief 异常
 *
 */
class myException {
public:
  myException(vector<string> msgs = {"YES", ""},
              vector<double> relocation_err = {},
              vector<int> exception_bottle = {}, int rack_absnum = 0);
  std_msgs::String what();
  string getException();
  Json::Value root1, root2;
  std_msgs::String _pubmsgs;
};

enum Movetowards { UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD };
enum Grip { GRIP, RELEASE };

enum CirType { CENTER, INTERIM };

struct MSG {
  string id;
  string exper_no;
  string state;
  string detail;

  MSG() {
    id = "-1";
    exper_no = "-1";
    state = "idle";
    detail = "";
  }
};

struct COMMAND {
  string operation;
  string station_rack_idx;
  string station_slot_idx;
  string station_btn_idx;
  string robot_rack_idx;
  string robot_slot_idx;
  string phase;

  COMMAND(string _operation, string _station_rack, string _station_slot,
          string _station_btn, string _robot_rack, string _robot_slot,
          string _phase) {
    operation = _operation;
    station_rack_idx = _station_rack;
    station_slot_idx = _station_slot;
    station_btn_idx = _station_btn;
    robot_rack_idx = _robot_rack;
    robot_slot_idx = _robot_slot;
    phase = _phase;
  }
};

/**
 * @brief 机器人化学家机械臂部分主要模块
 *
 */
class ur5e_action {
public:
  /**
   * @brief moveit movegroup 指针
   *
   */
  moveit::planning_interface::MoveGroupInterfacePtr _mgptr;
  /**
   * @brief UR5e的dashboard客户端,用于远程启动UR5e,加载远程控制程序
   *
   */
  shared_ptr<dashboardsrv_client> _dbptr;
  /**
   * @brief 存放站点位姿信息文件的路径
   *
   */
  string _file_root;
  /**
   * @brief 机器人当前站点
   *
   */
  string _curr_station;
  /**
   * @brief 离心机标签位姿
   *
   */
  geometry_msgs::Pose _CTF_pose;

  std::shared_ptr<MSG> _msgptr;

private:
  map<string, string> station_list{
      {"map_center", "map_center"},
      {"charge_station", "charge_station"},
      {"starting_station", "起始样品架"},
      {"libs", "LIBS工作站"},
      {"xrd", "XRD工作站"},
      {"ir", "红外光谱工作站"},
      {"imbibition_workstation", "吸液工作站"},
      {"liquid_dispensing", "液体进样站"},
      {"solid_dispensing", "固体进样站"},
      {"magnetic_stirring", "磁力搅拌站"},
      {"capping_station", "封装工作站"},
      {"dryer_workstation", "烘干工作站"},
      {"new_centrifugation", "新离心工作站"},
      {"photocatalysis_workstation", "光催化工作站"},
      {"electrocatalysis_workstation", "电催化工作站"},
      {"uv_vis", "紫外光谱工作站"},
      {"flourescence", "荧光光谱工作站"},
      {"raman_spectra", "拉曼光谱工作站"},
      {"muffle_station", "马弗炉工作站"},
      {"gc", "气相色谱工作站",},
      {"ultrasonic_cleaner", "超声工作站",}
      
      };

  /**
   * @brief pgi140夹爪
   *
   */
  shared_ptr<pgi> _pgi_ptr;
  robot_model::RobotModelPtr _rmodel;
  robot_state::RobotStatePtr _kinematic_state;
  const moveit::core::JointModelGroup *_jmg;
  /**
   * @brief 用于指示 是否打开串口(包括夹爪,力传感器,气动夹爪),一般用于仿真
   *
   */
  bool _no_serial;
  /**
   * @brief 用于指示 是否检查抓起\掉落
   *
   */
  bool _checkgrasp;
  /**
   * @brief 夹持标志位，指示是否夹持有物体
   *
   */
  bool _grasp_flag = false;
  /**
   * @brief 保护性停止标志位
   *
   */
  int _safety_flag;
  /**
   * @brief 操作执行完成标志位
   *
   */
  int _exec_flag;
  /**
   * @brief 当前站点marker id
   *
   */
  int _marker_id;
  /**
   * @brief 末端link的名字，默认tool0
   *
   */
  string _end_link;
  /**
   * @brief 机器人当前位置
   *
   */
  string _curr_pose = "Jdefault_0";

  /**
   * @brief 用于存放工作站关节角
   *
   */
  map<string, vector<double>> _Inst_joints;
  /**
   * @brief 用于存放机器人平台的关节角
   *
   */
  map<string, vector<double>> _Robot_joints;
  /**
   * @brief 用于存放相对标签的位姿
   *
   */
  map<string, geometry_msgs::Pose> _Inst_relPoses;
  /**
   * @brief 用于存放相对基座的位姿
   *
   */
  map<string, geometry_msgs::Pose> _Inst_absPoses;
  /**
   * @brief
   * _sub1_用于接受操作指令，_sub2_用于监测机械臂safety_mode,_sub3_用于监测轨迹执行状态
   *
   */
  ros::Subscriber _sub1_, _sub2_, _sub3_;
  /**
   * @brief pub1_用于发布操作反馈,pub2_用于发布相机指令,_pub3_用于取消轨迹执行
   *
   */
  ros::Publisher _pub1_, _pub2_, _pub3_;
  /**
   * @brief
   * 轨迹规划算法(pilz_sequence_plan),力矩传感器(robotiq_ft_sensor)的service客户端
   *
   */
  ros::ServiceClient _client1_, _client2_, _client3_;

  ros::ServiceServer _service;

  /**
   * @brief 机械臂关节名称
   *
   */
  vector<string> _joints_names{"shoulder_pan_joint", "shoulder_lift_joint",
                               "elbow_joint",        "wrist_1_joint",
                               "wrist_2_joint",      "wrist_3_joint"};

  // *用于异常恢复的数据：
  /**
   * @brief 异常发生前存储的规划轨迹信息
   *
   */
  vector<string> _Vname;
  /**
   * @brief 异常发生前存储的规划轨迹信息
   *
   */
  vector<double> _Vradius;
  /**
   * @brief 异常发生前存储的规划轨迹信息
   *
   */
  vector<double> _Vspeed;
  /**
   * @brief 异常发生前存储的规划轨迹信息
   *
   */
  vector<double> _Vacc;
  /**
   * @brief 异常发生前存储的规划轨迹信息
   *
   */
  vector<string> _Vplanner;
  /**
   * @brief 异常前存储的路径点(位姿)
   *
   */
  vector<geometry_msgs::Pose> _waypoints;

public:
  ur5e_action(ros::NodeHandle nh, shared_ptr<dashboardsrv_client> dbptr,
              moveit::planning_interface::MoveGroupInterfacePtr mgptr);
  ~ur5e_action();

  /**
   * @brief
   * 载入特定站点的结构化信息（关节角和位姿）(注意：该函数只在机械臂复位后才会调用，所以需要在站点操作前发送复位指令)
   * @param station_name 需要载入的站点名称
   */
  void LoadStationInfo(string station_name);

  /**
   * @brief 载入机器人平台上的关节角
   */
  void LoadRobotJointInfo();

  /**
   * @brief 力反馈控制
   * @param name 目标位姿
   * @param speed 移动速度系数，建议0.01，以便有足够时间反应
   * @param thre 力矩阈值，超过此阈值会停止移动
   */
  void ForceFeedback(string name, double speed, double thre);

  /**
   * @brief
   * 初始化UR5e和夹爪,包括启动电源,释放制动器,加载远程控制程序(该函数要求UR5e处于远程控制模式)
   *
   * @param filename 用于指定加载哪个程序,目前只需要加载"remote_control.urp\n"
   */
  void Init(const string filename);

  /**
   \brief 将机械臂移动到默认位姿
   \param name 指定需要移动的关节角
   */
  bool GotoDefaultPose(string name);

  /**
   * @brief 站点信息记录主函数,包含Save模式和Go模式(TODO)
   *
   * @param station 站点名称
   * @param isJoint 选择记录关节角还是位姿
   */
  void SaveStationInfo(std::string station, bool isJoint);

  /**
   * @brief 用于切换模式(TODO)
   *
   * @param mode 0为Save模式(默认),1为Go模式.
   */
  void SwitchMode(int mode);

  /**
   * @brief
   * 站点信息记录中的Save模式,此模式下输入名称会调用SaveStationJoint()或SaveStationPose()来保存信息至对应json文件.(TODO)
   *
   */
  void SaveMode();

  /**
   * @brief
   * 站点信息记录中的Go模式,此模式下输入关节角或位姿名称就可以移动到对应位置.(TODO)
   *
   */
  void GoMode();

  /**
   * @brief 记录当前位姿到对应json文件中(TODO)
   *
   * @param file_path 需要写入的json文件路径
   * @param name 位姿名称
   * @param X 位姿平移
   * @param Q 位姿旋转四元数
   */
  void SaveStationPose(string file_path, string name, Vector3d X,
                       Quaterniond Q);

  /**
   * @brief 记录当前关节角到对应json文件中(TODO)
   *
   * @param file_path 需要写入的json文件路径
   * @param name 关节角名称
   * @param joints 关节角数值
   */
  void SaveStationJoint(string file_path, string name, vector<double> joints);

  /**
   * @brief 解析并执行任务调度模块发送的指令(TODO)
   *
   * @param station 工作站名称
   * @param operation 操作名称
   * @param rack 机器人平台上的试管架编号
   * @param bottle 试管架上的试管编号
   * @param rack_oper 起始样品架,回收样品架的试管架编号
   * @param light_num 光催化位置编号
   * @param paper 电催化碳纸编号
   * @param dryer_num 烘干机位置编号
   * @param libs_num libs样品编号
   */
  void CommandExecute(std::string station, std::string operation, int rack,
                      int bottle, int rack_oper, int light_num, int paper,
                      int dryer_num, int libs_num);

  /**
   \brief
   机械臂末端在工作空间中沿直线轨迹移动，会在起点和目标之间按照一条直线运动
   \param towards 移动方向
   \param dis 移动距离(m)
   \param speed_factor 移动速度缩放系数(0,1]
   */
  void MoveLine_pilz(Movetowards towards, double dis, double speed_factor);

  /**
   * @brief 机械臂末端在工作空间中沿圆弧轨迹运动，需要指定圆心或中间点(TODO)
   *
   * @param type
   * 指定圆弧的类型。CENTER：指定圆心，则会在起点和目标之间按照较短的弧运动；INTERIM：指定中间点，则圆弧轨迹会经过指定的中间点
   * @param auxiliary_point 圆弧的圆心名称或中间点名称
   * @param goal_point 目标点名称
   */
  void MoveCircle_pilz(CirType type, std::string auxiliary_point,
                       std::string goal_point);

  /**
   * @brief 根据设定的操作进行移动
   *
   * @param V_name 需要经过的路点名称
   * @param V_radius
   * 指定每个路点的融合半径系数(0,1)(最后一个路径点必须为0)。如下图所示,真实融合半径=融合半径系数*min(||Pm-1Pm||,||PmPm+1||)
   *  @image html blend.png "平滑过渡段示意图"
   * @param V_speed
   * 指定每个路点的速度缩放系数(0,1]。实际运动速度=速度缩放系数*最大运动速度
   * @param V_acc
   * 指定每个路点的加速度缩放系数(0,1]。实际运动加速度=加速度缩放系数*最大运动加速度
   * @param V_planner
   * 指定每个路点的规划器(PTP,LIN)。PTP为点到点规划，LIN为直线规划
   * @param validate 是否只是验证该轨迹可行（该参数已被抛弃）
   * @param record 是否实时记录末端到达了哪个路径点
   */
  void SequenceMove_pilz(vector<string> &V_name, vector<double> &V_radius,
                         vector<double> &V_speed, vector<double> &V_acc,
                         vector<string> &V_planner, bool validate, bool record);

  /**
   * @brief  根据设定的操作进行移动(TODO)
   *
   * @param V_name 需要经过的路点名称
   * @param V_radius
   * 指定每个路点的融合半径系数(0,1)(最后一个路径点必须为0)。如下图所示,真实融合半径=融合半径系数*min(||Pm-1Pm||,||PmPm+1||)
   *  @image html blend.png "平滑过渡段示意图"
   * @param V_speed
   * 指定每个路点的速度缩放系数(0,1]。实际运动速度=速度缩放系数*最大运动速度
   * @param V_acc
   * 指定每个路点的加速度缩放系数(0,1]。实际运动加速度=加速度缩放系数*最大运动加速度
   * @param V_planner
   * 指定每个路点的规划器(PTP,LIN,CRIC)。PTP为点到点轨迹规划，LIN为直线轨迹规划，CIRC为圆弧轨迹规划
   * @param validate 是否只是验证该轨迹可行（该参数已被抛弃）
   * @param record 是否实时记录末端到达了哪个路径点
   * @param speedprofile
   * 选择速度轮廓曲线(trap,doubleS)。trap为梯形速度规划，doubleS为双S形速度规划
   */
  void SequenceMove_pilz(vector<string> &V_name, vector<double> &V_radius,
                         vector<double> &V_speed, vector<double> &V_acc,
                         vector<string> &V_planner, bool validate, bool record,
                         std::string speedprofile);

  /**
   \brief 验证给定的单个操作是否可行(该操作在必要时会插入路径点)
   \param V_name 需要经过的路点名称
   \param V_radius 每个路点的融合半径系数(0,1)(最后一个路径点必须为0)
   \param V_speed 每个路点的速度系数(0,1]
   \param V_acc 每个路点的加速度系数(0,1]
   \param V_planner 每个路点的规划器(PTP,LIN)
   \param validate 是否仅验证该操作是否可行
   */
  int32_t ValidateOneReq(vector<string> &V_name, vector<double> &V_radius,
                         vector<double> &V_speed, vector<double> &V_acc,
                         vector<string> &V_planner);

  /**
   \brief 验证给定的多个操作是否可行
   \param V_name 多个需要经过的路点名称
   \param V_radius 多个路点的融合半径系数(0,1)(最后一个路径点必须为0)
   \param V_speed 多个路点的速度系数(0,1]
   \param V_acc 多个路点的加速度系数(0,1]
   \param V_planner 多个路点的规划器(PTP,LIN)
   */
  void SequenceValidate_pilz(vector<vector<string> *> V_name,
                             vector<vector<double> *> V_radius,
                             vector<vector<double> *> V_speed,
                             vector<vector<double> *> V_acc,
                             vector<vector<string> *> V_planner);

  /**
   * @brief 根据给定的操作，构造轨迹规划的request
   *
   * @param V_name 需要经过的路点名称
   * @param V_radius 每个路点的融合半径系数(0,1)(最后一个路径点必须为0)
   * @param V_speed 每个路点的速度系数(0,1]
   * @param V_acc 每个路点的加速度系数(0,1]
   * @param V_planner 每个路点的规划器(PTP,LIN)
   * @param validate 是否仅验证该操作是否可行
   * @return moveit_msgs::MotionSequenceRequest 构造得到的轨迹规划request
   */
  moveit_msgs::MotionSequenceRequest
  ConstructSeqReq(vector<string> &V_name, vector<double> &V_radius,
                  vector<double> &V_speed, vector<double> &V_acc,
                  vector<string> &V_planner, bool validate);

  /**
   * @brief
   * 异常情况下复位机械臂。该函数会倒序执行存储的路径点和夹爪操作，直到末端没有夹持物体且到达安全位置。若在恢复过程中出现异常，会停止运动并杀死机械臂节点。此时需要人工介入。
   *
   */
  void ResetArm();

  /**
   * @brief 控制夹爪闭合和张开,并检查是否夹持有物体
   *
   * @param position 夹爪闭合位置(0:完全打开,1000:完全闭合)
   * @param check 是否检查夹持到物体
   * @param block 是否等待夹爪完全到位
   * @param force 设定抓取力(0,100]
   * @param speed 设定开合速度(0,100]
   */
  void Gripper_op(uint16_t position, bool check = false, bool block = false,
                  uint16_t force = 20, uint16_t speed = 50);

  /**
   \brief 操作状态反馈,向上位机发布/obsOperation_out话题
   \param msgs 包含IsDone和Exception
   \param relocation_err 底盘定位误差{位置x,y,z,姿态x,y,z,w}
   \param exception_bottle
   试管架中没有试管的位置,若Exception为"bottle_not_all",则该参数非空(暂未使用)
   \param rack_absnum 启动区扫码完成后,发送试管架绝对编号(暂未使用)
   */
  // void JsonPub(vector<string> msgs = {"YES", ""}, vector<double>
  // relocation_err = {}, vector<int> exception_bottle = {}, int rack_absnum =
  // 0);

  /**
   \brief 设定笛卡尔轨迹的末端速度,已弃用
   \param plan 需要设定速度的plan
   \param end_effector 指定末端link
   \param speed 设定的末端速度(m/s)
   */
  void setAvgCartesianSpeed(
      moveit::planning_interface::MoveGroupInterface::Plan &plan,
      const string end_effector, const double speed);

  /**
   \brief 根据当前站点的标签位姿计算相对基座位姿(用于重定位之后)
   \param station_name 当前站点名称
   \return 标签位姿
   */
  Isometry3d CalculatePoses(string station_name);

  /**
   \brief 根据给定的路径点序列插入路径点
   \param V_name 待插入的路径点序列
   \return 是否插入了路径点
   */
  bool InsertPoses(vector<string> &V_name);

  /**
   \brief 订阅/obsOperation_in的回调函数,接收上位机指令进行对应操作
   */
  void obs_Callback(const std_msgs::StringConstPtr &obsOperation_in);

  /**
   \brief 订阅/ur_hardware_interface/safety_mode的回调函数,解除保护性停止
   */
  void safety_Callback(const ur_dashboard_msgs::SafetyModeConstPtr &safetymode);

  /**
   \brief 订阅/execute_trajectory/result的回调函数,实时获取轨迹执行结果
   */
  void exec_Callback(
      const moveit_msgs::ExecuteTrajectoryActionResultConstPtr &execresult);

  /**
   \brief 记录上一次轨迹规划的信息
   \param i 下一个目标路径点的索引
   \param V_name 当前执行轨迹的路点名称
   \param V_radius 当前执行轨迹的混合半径
   \param V_speed 当前执行轨迹的速度
   \param V_acc 当前执行轨迹的加速度
   \param V_planner 当前执行轨迹的规划器
   */
  void RecordSequence(size_t &i, vector<string> V_name, vector<double> V_radius,
                      vector<double> V_speed, vector<double> V_acc,
                      vector<string> V_planner);

  /**
   \brief 记录当前位置到下一位置的轨迹规划信息
   \param speed 当前执行轨迹的速度
   \param acc 当前执行轨迹的加速度
   \param planner 当前执行轨迹的规划器
   */
  void RecordCurrSequence(double speed, double acc, string planner);

  /**
   \brief 根据夹持状态和当前末端位置清空已记录的路径点
   */
  bool ClearSequence();

  /**
   \brief 根据指定名称,计算下一个目标位置和约束
   \param name 指定路点名,"J"开头则为关节角,"P"开头则为位置
   \param pose_goal 需要构造的目标约束
   \param next_pose 需要构造的目标位置
   */
  void GetNextpose(string name, moveit_msgs::Constraints &pose_goal,
                   geometry_msgs::PoseStamped &next_pose);

  /**
   \brief 根据当前位置,目标位置和下一位置计算融合半径
   \param curr_pose 当前位置
   \param next_pose 目标位置
   \param n_next_pose 目标位置的下一位置
   */
  double GetBlendradius(geometry_msgs::Pose curr_pose,
                        geometry_msgs::Pose next_pose,
                        geometry_msgs::Pose n_next_pose);
  /**
   * @brief 从_Init_joints和_Robot_joints中获取关节角
   * @param 关节角的名字
   */
  vector<double> GetJoints(string name);

  bool query_Callback(chemicalrobot_arm::DmsService::Request &req,
                      chemicalrobot_arm::DmsService::Response &res);

  string GetStamp();

  void JsonPubDone();

  void JsonPub();

  void JsonPubErr(string detail);

  bool RobotAction(COMMAND cmd);

  void GoDefault(string station_name = "");

  void Relocation();

  void RelocationElectrocatalysis();

  void RelocationLibs();
};

/**
 * @brief 将平移向量和四元数转为变换矩阵
 *
 * @param X 平移向量
 * @param Q 四元数
 * @return Isometry3d 对应的变换矩阵
 */
Isometry3d toMatrix(Vector3d &X, Quaterniond &Q);

/**
 * @brief 将平移向量和YPR欧拉角转为变换矩阵
 *
 * @param X 平移向量
 * @param YPR YPR欧拉角
 * @return Isometry3d 对应的变换矩阵
 */
Isometry3d toMatrix(Vector3d &X, Vector3d &YPR);

/**
 * @brief 将geometry_msgs::Pose转为变换矩阵
 *
 * @param pose 待转换的pose
 * @return Isometry3d 对应的变换矩阵
 */
Isometry3d toMatrix(geometry_msgs::Pose &pose);

/**
 * @brief 将变换矩阵转为geometry_msgs::Pose
 *
 * @param T
 * @return geometry_msgs::Pose 对应的geometry_msgs::Pose
 */
geometry_msgs::Pose toPose(Isometry3d &T);

/**
 * @brief 将平移向量和YPR欧拉角转为geometry_msgs::Pose
 *
 * @param X 平移向量
 * @param YPR YPR欧拉角
 * @return geometry_msgs::Pose 对应的geometry_msgs::Pose
 */
geometry_msgs::Pose toPose(Vector3d &X, Vector3d &YPR);

/**
 * @brief 将平移向量和四元数转为geometry_msgs::Pose
 *
 * @param X 平移向量
 * @param Q 四元数
 * @return geometry_msgs::Pose 对应的geometry_msgs::Pose
 */
geometry_msgs::Pose toPose(Vector3d &X, Quaterniond &Q);

#endif