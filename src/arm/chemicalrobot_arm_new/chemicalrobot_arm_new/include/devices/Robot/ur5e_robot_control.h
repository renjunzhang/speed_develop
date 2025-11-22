#ifndef UR5E_ROBOT_CONTROL_H
#define UR5E_ROBOT_CONTROL_H

#include <string>

#include <Eigen/Geometry>

#include <QtCore/QDebug>
#include <QtCore/QDir>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QCoreApplication>
#include <QString>

#include <ros/ros.h>

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

#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/GetRobotMode.h>
#include <ur_dashboard_msgs/RawRequest.h>
#include <ur_dashboard_msgs/GetProgramState.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

#include <robotiq_ft_sensor/sensor_accessor.h>

#include "devices/device.h"

using namespace Eigen;

// 用于ur5e手操板通信
class ur5e_robot_dashboard
{
public:
    ur5e_robot_dashboard(ros::NodeHandle& nh);

    /**
     * @brief 载入并启动程序
     *
     * @param filename 程序名称
     */
    void load_program(const std::string filename);

    /**
     * @brief 运行加载的程序
     *
     */
    bool play();

    /**
     * @brief 停止程序
     *
     */
    void stop();

    /**
     * @brief 打开UR5e电源并释放制动
     *
     */
    void robot_init();

    /**
     * @brief 设置数字输出引脚的电平
     *
     * @param pin 引脚的位置
     * @param state 设置的电平
     * @return true success ,false failed
     */
    bool setDO(int8_t pin, bool state);

    /**
     * @brief 解除保护性停止
     *
     */
    void unlockPS();

private:
    ros::NodeHandle nh_;
    ros::ServiceClient srv_client_;
    ros::ServiceClient load_client_;
    ros::ServiceClient DO_client_;
    ros::ServiceClient program_client_;
    ros::ServiceClient safety_client_;
    ros::ServiceClient close_popup_client_;
    ros::ServiceClient close_safety_popup_client_;
};

//机器人标定数据结构体
struct robot_record_data {
  std::map<QString, double[6]> _joints;
  struct marker {
    int id;
    float size[2];
  };
  std::map<QString, double[7]> _poses;
  std::map<QString, QStringList> _cmds; 
  std::map<QString, std::pair<QString, QString>> _cmd_to_pointname; //map cmd to point mid name
  marker _marker;
  QString station_name;

  void clear() {
    _marker.id = 0;
    _marker.size[0] = 0;
    _marker.size[1] = 0;
    _joints.clear();
    _poses.clear();
    _cmds.clear();
    _cmd_to_pointname.clear();
    station_name = "";
  }
};

class ur5e_kinematics{
public:
    ur5e_kinematics(ros::NodeHandle& nh);
    void forward(const double *q, double *T);
    void forward_all(const double *q, double *T1, double *T2, double *T3,
                     double *T4, double *T5, double *T6);
    void forward(const double *q, Eigen::Isometry3d& mat);
    int inverse(const double *T, double *q_sols, double q6_des = 0.0);
    int inverse(Eigen::Isometry3d mat, double *q_sols, double q6_des = 0.0);
    double cal_joint_distance(double *J1, double *J2);
    int get_nearest_solution(double *q_sols,int solution_num,double *seed_joints);
private:
    const double ZERO_THRESH = 0.00000001;
    const double PI = M_PI;
    double d1 = 0.1625;
    double a2 = -0.42500;
    double a3 = -0.39225;
    double d4 = 0.1333;
    double d5 = 0.0997;
    double d6 = 0.0996;

    int SIGN(double x){
      return (x > 0) - (x < 0);
    }
};

// 基于movit的路径规划和执行
class ur5e_robot : public chemical::device{
private:
    ros::NodeHandle* nh = nullptr;
    moveit::core::RobotModelPtr rmodel;
    bool have_ft_sensor;
    double relocation_tolerance;
    moveit::planning_interface::MoveGroupInterfacePtr _mgptr;
    std::shared_ptr<ur5e_robot_dashboard> _dbptr;
    //_sub_safetymode用于监测机械臂safety_mode, _sub_trajectory_status用于监测轨迹执行状态
    ros::Subscriber _sub_safetymode, _sub_trajectory_result;
    //_pub_trajectory_cancle用于取消轨迹执行,_pub_camera_op_用于发布相机指令,_pub_send_scripts_用于发送脚本指令到UR机械臂
    ros::Publisher _pub_trajectory_cancle, _pub_camera_op_,_pub_send_scripts_;
    //轨迹规划算法(pilz_sequence_plan)、矩传感器(robotiq_ft_sensor)、movit内置路径规划算法（plan_kinematic_path）的客户端
    ros::ServiceClient _client_pilz, _client_ft_sensor, _client_kinematic;

    int32_t _safety_flag,_exec_flag;

    QString _rootpath, _record_data_path;
    QDir _rootdir;
    QFileInfoList _datafilelist;
    std::map<QString,robot_record_data> _robot_record_data;
    robot_record_data _robot_joint_station;

    QString _current_station_name,_current_operation;
    int _rack_robot,_slot_robot,_rack_station,_slot_station;
    Eigen::Isometry3d current_station_base2marker;

    std::string _end_link;
    ur5e_kinematics *_kinematics = nullptr;

public:
    ur5e_robot(ros::NodeHandle& nh);
    ~ur5e_robot();

    virtual DeviceErrorCode init(void*) override;
    virtual DeviceErrorCode connect() override;
    virtual DeviceErrorCode execute_script(std::string script) override;
    virtual DeviceErrorCode execute_scripts(std::vector<std::string> scripts) override;
    virtual DeviceErrorCode back_to_checkpoint() override;
    virtual DeviceErrorCode getdata(std::string dataname,std::string& data) override;


    bool set_teach_mode();
    bool quit_teach_mode();
private:
    void safety_Callback(const ur_dashboard_msgs::SafetyModeConstPtr &safetymode);
    void exec_Callback(const moveit_msgs::ExecuteTrajectoryActionResultConstPtr &execresult);
    bool GetPoseConstraints(QString name, QString fuc, bool isrobotjoint,
        moveit_msgs::Constraints &constraints,geometry_msgs::PoseStamped &next_pose);
    bool get_dynamic_point_name(
        QString &pointname, QString station_name, QString operation,
        std::map<QString, std::pair<QString, QString>> operation_to_pointname,
        int slot_num_per_rack = 10, int rack_station = 0,
        int slot_station = 0, int rack_robot = 0, int slot_robot = 0);
    moveit_msgs::MotionSequenceRequest 
        ConstructSeqReq(QStringList path_scripts, bool force_blend_radius_0 = false);
    double GetBlendradius(geometry_msgs::Pose curr_pose,
        geometry_msgs::Pose next_pose,geometry_msgs::Pose n_next_pose);
    bool asyncExecutePose(geometry_msgs::Pose pose);
    /**
       \brief 根据当前站点的标签位姿计算相对基座位姿(用于重定位之后)
      \return 标签位姿
      */
    DeviceErrorCode CalculatePoses(Isometry3d& marker2base,bool remove_xy = true);
    void MoveLine_pilz(QString towards, double dis, double speed_factor);
};

inline bool RobotJsonDataPaser(QString filename, robot_record_data &data);
inline bool RobotJsonDataSave(QString filename, robot_record_data data);
#endif