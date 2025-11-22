#ifndef CHEMICAL_PLATFORM_SYSTEM_H
#define CHEMICAL_PLATFORM_SYSTEM_H
#include <std_msgs/String.h>
#include <jsoncpp/json/json.h>
#include <QWidget>

#include "chemicalrobot_arm_new/DmsService.h"
#include "devices/device.h"
#include "devices/Robot/ur5e_robot_control.h"
#include "devices/Gripper/pgi_control.h"
#include "devices/Dripper/dripper_control.h"
#include "devices/LinearModule/screwslide_control.h"

#include <tf2_ros/transform_listener.h>

class mywidget : public QWidget
{
  Q_OBJECT
public:
  mywidget(QWidget *parent = 0);
  ~mywidget();
  int button_num;

signals:
  void open_message_box(QString tips);

private slots:
  void on_message_box_open(QString tips);
};

struct MSG {
  std::string id;
  std::string exper_no;
  std::string state;
  std::string detail;

  MSG() {
    id = "-1";
    exper_no = "-1";
    state = "idle";
    detail = "";
  }
};

/**
 * @brief 机器人化学家机械臂部分主要模块
 *
 */
class ChemicalPlatform {
public:
  std::shared_ptr<MSG> _msgptr;
  std::shared_ptr<chemical::device> _pgiptr, _dripper_ptr, _screwslide_ptr, _robot_ptr;

private:

  /**
   * @brief
   * _sub1_用于接受dms操作指令,_sub2_用于接受夹爪操作指令
   *
   */
  ros::Subscriber _sub1_,_sub2_;
  /**
   * @brief pub1_用于发布操作反馈,_pub3_用于取消轨迹执行,_pub4_用于发布关节值
   * ,_pub5_用于发布数字孪生目标类型,_pub6_用于发布夹爪操作
   *
   */
  ros::Publisher _pub1_, _pub3_,_pub4_,_pub5_,_pub6_;

  ros::ServiceServer _service;

  QString current_statison_name,last_station_name,current_operation_name;
  bool relocationed;
  ros::Timer _timer;
  mywidget* _widget;

public:
  ChemicalPlatform(ros::NodeHandle& nh);
  ~ChemicalPlatform();

  /**
   * @brief
   * 初始化UR5e和夹爪,包括启动电源,释放制动器,加载远程控制程序(该函数要求UR5e处于远程控制模式)
   *
   */
  void Init();

  /**
   \brief 订阅/obsOperation_in的回调函数,接收上位机指令进行对应操作
   */
  void obs_Callback(const std_msgs::StringConstPtr &obsOperation_in);

  /**
   \brief 订阅/grip_operation_in的回调函数,接收上位机指令进行对应操作
   */
  void pgi_Callback(const std_msgs::StringConstPtr &obsOperation_in);

  /*
   \brief 执行脚本
   \param scripts 脚本列表
   \return 错误码
  
  DeviceErrorCode execute_script(std::string script); 
  */

  /**
   \brief 执行脚本
   \param scripts 脚本列表
   \return 错误码
   */
  DeviceErrorCode execute_scripts(StringList scripts);

  bool query_Callback(chemicalrobot_arm_new::DmsService::Request &req,
                      chemicalrobot_arm_new::DmsService::Response &res);

  std::string GetStamp();

  void JsonPubDone();

  void JsonPub();

  void JsonPubErr(std::string detail, bool need_reboot = false, bool not_pub = false);

  void JointsPub(const ros::TimerEvent& event);
};


#endif