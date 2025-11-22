#include "devices/Robot/ur5e_robot_control.h"
#include <opencv2/opencv.hpp>
#include "chemical_util.h"
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
/************************dashboard start**************************/

ur5e_robot_dashboard::ur5e_robot_dashboard(ros::NodeHandle& nh)
{
  nh_ = nh;
  srv_client_ = nh_.serviceClient<ur_dashboard_msgs::RawRequest>("/ur_hardware_interface/dashboard/raw_request");
  load_client_ = nh_.serviceClient<ur_dashboard_msgs::Load>("/ur_hardware_interface/dashboard/load_program");
  DO_client_ = nh_.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
  program_client_ = nh_.serviceClient<ur_dashboard_msgs::GetProgramState>("/ur_hardware_interface/dashboard/program_state");
  safety_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/unlock_protective_stop");
  close_popup_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/close_popup");
  close_safety_popup_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/close_safety_popup");
}

bool ur5e_robot_dashboard::play()
{
  ur_dashboard_msgs::RawRequest play;
  play.request.query = "play\n";
  while (true)
  {
    srv_client_.call(play);
    if (ros::topic::waitForMessage<std_msgs::Bool>("/ur_hardware_interface/robot_program_running")->data)
      break;
    else
      ROS_WARN_STREAM("Launch program failed, please operate dashboard controller manually");
    ros::Duration(0.5).sleep();
  }
  return true;
}

void ur5e_robot_dashboard::stop()
{
  ur_dashboard_msgs::RawRequest stop;
  stop.request.query = "stop\n";
  for (size_t i = 0; i < 5; ++i)
  {
    if (ros::topic::waitForMessage<std_msgs::Bool>("/ur_hardware_interface/robot_program_running")->data)
    {
      srv_client_.call(stop);
      ros::Duration(0.5).sleep();
    }
    else
      return;
  }
  ROS_WARN_STREAM("Stop program failed, please operate dashboard controller manually");
}

void ur5e_robot_dashboard::unlockPS()
{
  std_srvs::Trigger unlock;
  while (true)
  {
    safety_client_.call(unlock);
    if (ros::topic::waitForMessage<ur_dashboard_msgs::SafetyMode>("/ur_hardware_interface/safety_mode")->mode ==
        ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP)
      ROS_WARN_STREAM("Unlock protective stop failed,robot is in emergency stop status,waiting for recovering...");
    else
      break;
    ros::Duration(1).sleep();
  }
}

void ur5e_robot_dashboard::load_program(const std::string filename)
{
  ROS_INFO_STREAM("Loading program " << filename.substr(0, filename.size() - 1) << " ...");
  ur_dashboard_msgs::Load load;
  load.request.filename = std::string(filename);
  load_client_.call(load);
  if (load.response.success)
    ROS_INFO_STREAM("Program load succeed");
  play();
}

bool ur5e_robot_dashboard::setDO(int8_t pin, bool state)
{
  ur_msgs::SetIO srv;
  srv.request.fun = srv.request.FUN_SET_DIGITAL_OUT;
  srv.request.pin = pin;
  srv.request.state = state ? srv.request.STATE_ON : srv.request.STATE_OFF;
  DO_client_.call(srv);
  if (!srv.response.success)
  {
    ROS_ERROR_STREAM(std::string("ur5e_robot_dashboard set DO-") + std::to_string(pin) + ":" + std::to_string(state) + " failed");
    return false;
  }
  return true;
}

void ur5e_robot_dashboard::robot_init()
{
  ur_dashboard_msgs::RawRequest init;
  ur_dashboard_msgs::RobotModeConstPtr robotmode = 
    ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>("/ur_hardware_interface/robot_mode");
  ur_dashboard_msgs::SafetyModeConstPtr robotsafety = 
    ros::topic::waitForMessage<ur_dashboard_msgs::SafetyMode>("/ur_hardware_interface/safety_mode");
  if (robotmode->mode == robotmode->RUNNING &&
      robotsafety->mode == robotsafety->NORMAL) {
    ROS_INFO_STREAM("Robot is RUNNING");
    return;
  }
  ROS_INFO_STREAM("Robot is Initializing");

  std_srvs::Trigger trigger;
  close_safety_popup_client_.call(trigger);
  close_popup_client_.call(trigger);

  init.request.query = "is in remote control\n";
  while(true){
    srv_client_.call(init);
    if(init.response.answer == "true")
      break;
    else
      ROS_WARN_STREAM("Robot send \"is in remote control\",response is :" << init.response.answer);
    ros::Duration(3.0).sleep();
  }
  ROS_INFO_STREAM("Robot is in remote control now");

  int time_count = 0;
  bool restart = true;
  while (true)
  {
    robotsafety = ros::topic::waitForMessage<ur_dashboard_msgs::SafetyMode>
                  ("/ur_hardware_interface/safety_mode");
    if (robotsafety->mode == robotsafety->NORMAL)
      break;

    ROS_INFO_STREAM("Robot safety status:" << int(robotsafety->mode) << ",recovering...");
    ros::Duration(3.0).sleep();
    time_count += 3;
    if (time_count > 6)
    {
      time_count = 0;
      close_safety_popup_client_.call(trigger);
      close_popup_client_.call(trigger);
      if(robotsafety->mode == robotsafety->FAULT){
        init.request.query = "restart safety\n";
        if(restart)
          srv_client_.call(init);
        
        if(init.response.answer == "Restarting safety")
          restart = false;
        else
          ROS_WARN_STREAM("Robot send \"restart safety\",response is :" << init.response.answer);
      }
      else if(robotsafety->mode == robotsafety->PROTECTIVE_STOP){
        unlockPS();
      }
    }
  }
  ROS_INFO_STREAM("Robot safety status:" << int(robotsafety->mode));

  robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>("/ur_hardware_interface/robot_mode");
  ROS_INFO_STREAM("Robot mode:" << int(robotmode->mode));
  if (robotmode->mode == robotmode->POWER_OFF)
  {
    ROS_INFO_STREAM("Robot power on...");
    init.request.query = "power on\n";
    bool send_again = true;
    while (robotmode->mode != robotmode->IDLE)
    {
      if(send_again)
        srv_client_.call(init);

      if(init.response.answer == "Powering on")
        send_again = false;
      else if(init.response.answer.find("switch robot to Remote Control mode") != std::string::npos)
        ROS_ERROR_STREAM("Send power on failed," << init.response.answer);

      robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>(
          "/ur_hardware_interface/robot_mode");
      ROS_INFO_STREAM("Robot mode:" << int(robotmode->mode));
      ros::Duration(1.0).sleep();
    }
  }
  if(robotmode->mode == robotmode->IDLE){
    ROS_INFO_STREAM("Robot is IDLE,releasing brake");
    init.request.query = "brake release\n";
    init.response.answer = "";
    bool send_again = true;
    while (robotmode->mode != robotmode->RUNNING) {
      if(send_again)
        srv_client_.call(init);

      if(init.response.answer == "Brake releasing")
        send_again = false;
      else if(init.response.answer.find("switch robot to Remote Control mode") != std::string::npos)
        ROS_ERROR_STREAM("Send brake release failed," << init.response.answer);

      ROS_INFO_STREAM("Robot mode:" << int(robotmode->mode));
      robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>(
          "/ur_hardware_interface/robot_mode");
      ros::Duration(1.0).sleep();
    }
  }
  ROS_INFO_STREAM("Robot is RUNNING");
}

/************************dashboard end**************************/

/************************ur5e_robot start**************************/
std::vector<std::string> _joints_names{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

ur5e_robot::ur5e_robot(ros::NodeHandle& nh)
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  rmodel = robot_model_loader.getModel();
  _status = DS_UNCONNECTED;
  _attribute.clear();
  _kinematics = new ur5e_kinematics(nh);
  nh.param<bool>("/have_ft_sensor", have_ft_sensor, false);
  nh.param<double>("/relocation_tolerance", relocation_tolerance, 0.15);
}

ur5e_robot::~ur5e_robot()
{
  if (_kinematics != nullptr)
    delete _kinematics;
}

DeviceErrorCode ur5e_robot::init(void *p)
{
  nh = (ros::NodeHandle *)p;

  _record_data_path = QString::fromLocal8Bit((*(std::string *)getAttribute("record_data_path")).c_str());
  _end_link = *(std::string *)getAttribute("end_link");
  _mgptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");
  _mgptr->setEndEffectorLink(_end_link);

  _sub_safetymode = nh->subscribe<ur_dashboard_msgs::SafetyMode>(
      "/ur_hardware_interface/safety_mode", 1, &ur5e_robot::safety_Callback,
      this);

  _sub_trajectory_result = nh->subscribe<moveit_msgs::ExecuteTrajectoryActionResult>(
      "/execute_trajectory/result", 1, &ur5e_robot::exec_Callback, this);

  _pub_trajectory_cancle =
      nh->advertise<actionlib_msgs::GoalID>("/execute_trajectory/cancel", 1);

  _pub_camera_op_ = 
      nh->advertise<std_msgs::Int32>("/camera_operation", 1);

  _pub_send_scripts_ =
      nh->advertise<std_msgs::String>("/ur_hardware_interface/script_command", 1);

  if (have_ft_sensor)
  {
    _client_ft_sensor = nh->serviceClient<robotiq_ft_sensor::sensor_accessor>(
        "/robotiq_ft_sensor_acc");
    _client_ft_sensor.waitForExistence(ros::Duration(5));
  }

  _robot_record_data.clear();
  _robot_joint_station.clear();
  auto rootdir = QDir(_record_data_path);
  if (!rootdir.exists())
    rootdir.mkdir(_record_data_path);
  auto filelist = rootdir.entryInfoList(QDir::Files);
  if (!filelist.isEmpty())
  {
    foreach (QFileInfo file, filelist)
    {
      if (file.suffix() == "json")
      {
        _datafilelist.push_back(file);
        robot_record_data data;
        RobotJsonDataPaser(file.filePath(), data);
        _robot_record_data[data.station_name] = data;

        if (data.station_name == "机械臂关节")
          _robot_joint_station = data;
      }
    }
  }

  if (_robot_joint_station.station_name == "")
    ROS_WARN_STREAM("no robot joint station file founded");

  // save attribute
  setAttribute("_robot_record_data", (void *)&_robot_record_data);
  setAttribute("_current_station_name", (void *)&_current_station_name);
  setAttribute("_mgptr", (void *)&_mgptr);

  auto res = connect();
  if (res != DEVICE_SUCCESS)
    _status = DS_UNCONNECTED;
  else
    _status = DS_IDEL;
  return res;
}

DeviceErrorCode ur5e_robot::connect()
{
  _dbptr = std::make_shared<ur5e_robot_dashboard>(*nh);

  _client_pilz =
      nh->serviceClient<moveit_msgs::GetMotionSequence>("/plan_sequence_path");
  if (!_client_pilz.waitForExistence(ros::Duration(5)))
  {
    ROS_ERROR_STREAM("waiting for service \"plan_sequence_path\" overtimed,check if moveit-node existing");
    return DEVICE_CONNECT_FAILED;
  }

  _client_kinematic =
      nh->serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");
  if (!_client_kinematic.waitForExistence())
  {
    ROS_ERROR_STREAM("waiting for service \"plan_sequence_path\" overtimed,check if moveit-node existing");
    return DEVICE_CONNECT_FAILED;
  }

  _dbptr->robot_init();
  _dbptr->load_program("remote_control.urp\n");

  return DEVICE_SUCCESS;
}

DeviceErrorCode ur5e_robot::execute_script(std::string script)
{
  if (_status == DS_RUNNING)
    return DEVICE_ISRUNNIN;
  else if (_status == DS_UNCONNECTED)
    return DEVICE_CONNECT_FAILED;
  else if (_status == DS_FAULT)
    return DEVICE_IS_FAULT;

  _status = DS_RUNNING;
  ROS_INFO_STREAM("scripts size: 1");
  ROS_INFO_STREAM("script    : " + script);

  _current_station_name = QString::fromLocal8Bit((*(std::string *)getAttribute("_current_station_name")).c_str());
  _current_operation = QString::fromLocal8Bit((*(std::string *)getAttribute("_current_operation")).c_str());

  auto list = QString::fromLocal8Bit(script.c_str()).split(' ');
  auto stationjoints = _robot_record_data[_current_station_name]._joints;
  auto res = DEVICE_SUCCESS;

  if (list[0] == "J" || list[0] == "P")
  {
    assert(list.size() == 6);
    auto fuc = list[0];
    auto pointname = list[1];
    auto r = list[2];
    auto speed = list[3];
    auto acc = list[4];
    auto planid = list[5];
    auto pointname_old = pointname;
    // dynamic point
    bool isrobotjoint = get_dynamic_point_name(
        pointname, _current_station_name, _current_operation,
        _robot_record_data[_current_station_name]._cmd_to_pointname, 10,
        _rack_station, _slot_station, _rack_robot, _slot_robot);
    if (pointname_old != pointname)
    {
      // is dynamic point
      ROS_INFO_STREAM("Dynamic point name: " << pointname.toLocal8Bit().toStdString());
    }

    if (fuc == "J")
    {
      std::map<QString, double[6]>::iterator iter;
      if (isrobotjoint)
      {
        if (_robot_joint_station.station_name == "")
        {
          _status = DS_IDEL;
          return DEVICE_INVALID_ATTRIBUTION;
        }

        auto robotjoints = _robot_joint_station._joints;
        iter = robotjoints.find(pointname);
        if (iter == robotjoints.end())
        {
          ROS_ERROR_STREAM(
              "robot joint point : " + pointname.toLocal8Bit().toStdString() +
              " not found");
          _status = DS_IDEL;
          return DEVICE_INVALID_ATTRIBUTION;
        }
      }
      else
      {
        iter = stationjoints.find(pointname);
        if (iter == stationjoints.end())
        {
          ROS_ERROR_STREAM(
              "station joint point : " + pointname.toLocal8Bit().toStdString() +
              " not found");
          _status = DS_IDEL;
          return DEVICE_INVALID_ATTRIBUTION;
        }
      }
      double *jvalue = iter->second;
      std::vector<double> jointvalues(jvalue, jvalue + 6);
      _mgptr->setPlannerId(planid.toLocal8Bit().toStdString());
      _mgptr->setJointValueTarget(jointvalues);
      _mgptr->setMaxVelocityScalingFactor(speed.toDouble());
      _mgptr->setMaxAccelerationScalingFactor(acc.toDouble());

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      int replan_num = 10;
      int replan_count = 0;
      while (_mgptr->plan(plan) !=
                 moveit::planning_interface::MoveItErrorCode::SUCCESS &&
             replan_count < replan_num)
      {
        ++replan_count;
      }
      if (replan_count == 10)
      {
        ROS_ERROR_STREAM("MoveJ plan failed");
        _status = DS_IDEL;
        return DEVICE_EXECUTE_FAILED;
      }

      _exec_flag = 0;
      _safety_flag = 1;
      auto trajectory = plan.trajectory_;
      for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
      {
        if (trajectory.joint_trajectory.points[i].positions[2] < M_PI / 180)
        {
          ROS_ERROR_STREAM("Invalid trajectory, " << trajectory.joint_trajectory.joint_names[2] << " joint will be less than 1 degree");
          _status = DS_IDEL;
          return DEVICE_EXECUTE_FAILED;
        }
      }
      _mgptr->asyncExecute(trajectory);
      while (_exec_flag == 0)
      {
        // safty stop
        if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
          break;

        // qt loop wait 100 ms
        ros::Duration(0.1).sleep();
      }

      if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
      {
        res = DEVICE_EXECUTE_FAILED;
        ROS_ERROR_STREAM("MoveJ execute failed by safety stop");
        _status = DS_FAULT;
        _dbptr->robot_init();
        _dbptr->unlockPS();
        _dbptr->stop();

        std::string user_input;
        std::cout << "Press Y/y into teach mode\n";
        std::cin >> user_input;
        if(user_input == "Y" || user_input == "y"){
          set_teach_mode();
          std::cout << "Press any key to cancel teach mode\n";
          std::cin >> user_input;
          quit_teach_mode();
        }

        if (_dbptr->play())
          _status = DS_IDEL;
        return res;
      }

      ROS_INFO_STREAM("_exec_flag : " + std::to_string(_exec_flag));
      if (_exec_flag != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR_STREAM("MoveJ execute failed");
        _status = DS_IDEL;
        return DEVICE_EXECUTE_FAILED;
      }
    }
    else //"P"
    {
      moveit_msgs::Constraints constraints;
      geometry_msgs::PoseStamped pose_marker2tool;
      if (!GetPoseConstraints(pointname, "P", true, constraints,
                              pose_marker2tool))
      {
        _status = DS_IDEL;
        return DEVICE_INVALID_ATTRIBUTION;
      }

      planning_interface::MotionPlanRequest req;
      _mgptr->constructMotionPlanRequest(req);
      req.group_name = "manipulator";
      req.max_velocity_scaling_factor = speed.toDouble();
      req.max_acceleration_scaling_factor = acc.toDouble();
      req.planner_id = planid.toLocal8Bit().toStdString();
      req.goal_constraints.resize(1);
      req.goal_constraints[0] = constraints;
      robot_state::robotStateToRobotStateMsg(
          *(_mgptr->getCurrentState()),
          req.start_state); // set current robot state to start state

      moveit_msgs::GetMotionPlan srvreq;
      srvreq.request.motion_plan_request = req;
      srvreq.response.motion_plan_response.error_code.val = 0;

      _client_kinematic.call(srvreq);
      if (srvreq.response.motion_plan_response.error_code.val != 1)
      {
        ROS_ERROR_STREAM("MoveP plan failed");
        _status = DS_IDEL;
        return DEVICE_EXECUTE_FAILED;
      }

      // auto err =
      // _mgptr->execute(srvreq.response.motion_plan_response.trajectory);
      _exec_flag = 0;
      _safety_flag = 1;
      _mgptr->setPlannerId(planid.toLocal8Bit().toStdString());
      auto trajectory = srvreq.response.motion_plan_response.trajectory;
      for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
      {
        if (trajectory.joint_trajectory.points[i].positions[2] < M_PI / 180)
        {
          ROS_ERROR_STREAM("Invalid trajectory, " << trajectory.joint_trajectory.joint_names[2] << " joint will be less than 1 degree");
          _status = DS_IDEL;
          return DEVICE_EXECUTE_FAILED;
        }
      }
      _mgptr->asyncExecute(trajectory);
      while (_exec_flag == 0)
      {
        // safty stop
        if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
          break;

        // qt loop wait 100 ms
        ros::Duration(0.1).sleep();
      }

      if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
      {
        res = DEVICE_EXECUTE_FAILED;
        ROS_ERROR_STREAM("MoveP execute failed by safety stop");
        _status = DS_FAULT;
        _dbptr->robot_init();
        _dbptr->unlockPS();
        _dbptr->stop();

        std::string user_input;
        std::cout << "Press Y/y into teach mode\n";
        std::cin >> user_input;
        if(user_input == "Y" || user_input == "y"){
          set_teach_mode();
          std::cout << "Press any key to cancel teach mode\n";
          std::cin >> user_input;
          quit_teach_mode();
        }

        if (_dbptr->play())
          _status = DS_IDEL;
        return res;
      }

      ROS_INFO_STREAM("_exec_flag : " + std::to_string(_exec_flag));
      if (_exec_flag != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR_STREAM("MoveP execute failed");
        _status = DS_IDEL;
        return DEVICE_EXECUTE_FAILED;
      }
    }
  }
  else if (list[0] == "C")
  {
    assert(list.size() == 7);
    auto fuc = list[0];
    auto pointname1 = list[1];
    auto pointname2 = list[2];
    auto r = list[3];
    auto speed = list[4];
    auto acc = list[5];
    auto planid = list[6];

    // dynamic point
    bool isrobotjoint = get_dynamic_point_name(
        pointname1, _current_station_name, _current_operation,
        _robot_record_data[_current_station_name]._cmd_to_pointname, 10,
        _rack_station, _slot_station, _rack_robot, _slot_robot);

    moveit_msgs::Constraints pose_goal;
    geometry_msgs::PoseStamped target_pose;
    if (!GetPoseConstraints(pointname1, "P", isrobotjoint, pose_goal,
                            target_pose))
    {
      _status = DS_IDEL;
      return DEVICE_INVALID_ATTRIBUTION;
    }

    moveit_msgs::PositionConstraint posConstraint;
    posConstraint.link_name = _mgptr->getEndEffectorLink();
    posConstraint.constraint_region.primitive_poses.emplace_back(
        target_pose.pose);

    planning_interface::MotionPlanRequest req;
    _mgptr->constructMotionPlanRequest(req);
    //_mgptr->setPlanningPipelineId("ompl");
    req.group_name = "manipulator";
    req.max_velocity_scaling_factor = speed.toDouble();
    req.max_acceleration_scaling_factor = acc.toDouble();
    req.planner_id = "CIRC";
    req.path_constraints.name = "interim";
    req.path_constraints.position_constraints.resize(1);
    req.path_constraints.position_constraints[0] = posConstraint;

    isrobotjoint = get_dynamic_point_name(
        pointname2, _current_station_name, _current_operation,
        _robot_record_data[_current_station_name]._cmd_to_pointname, 10,
        _rack_station, _slot_station, _rack_robot, _slot_robot);

    if (!GetPoseConstraints(pointname2, "P", isrobotjoint, pose_goal,
                            target_pose))
    {
      _status = DS_IDEL;
      return DEVICE_INVALID_ATTRIBUTION;
    }

    req.goal_constraints.resize(1);
    req.goal_constraints[0] = pose_goal;

    moveit_msgs::GetMotionPlan srvreq;
    srvreq.request.motion_plan_request = req;
    srvreq.response.motion_plan_response.error_code.val = 0;

    int replan_num = 10;
    int replan_count = 0;
    while (srvreq.response.motion_plan_response.error_code.val != 1 &&
           replan_count < replan_num)
    {
      _client_kinematic.call(srvreq);
      ++replan_count;
    }
    if (replan_count == replan_num)
    {
      ROS_ERROR_STREAM("MoveC plan failed");
      _status = DS_IDEL;
      return DEVICE_EXECUTE_FAILED;
    }
    
    _exec_flag = 0;
    _safety_flag = 1;
    auto trajectory = srvreq.response.motion_plan_response.trajectory;
    for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
    {
      if (trajectory.joint_trajectory.points[i].positions[2] < M_PI / 180)
      {
        ROS_ERROR_STREAM("Invalid trajectory, " << trajectory.joint_trajectory.joint_names[2] << " joint will be less than 1 degree");
        _status = DS_IDEL;
        return DEVICE_EXECUTE_FAILED;
      }
    }
    _mgptr->asyncExecute(trajectory);
    while (_exec_flag == 0)
    {
      // safty stop
      if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
        break;

      // qt loop wait 100 ms
      ros::Duration(0.1).sleep();
    }

    if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
    {
      res = DEVICE_EXECUTE_FAILED;
      ROS_ERROR_STREAM("MoveC execute failed by safety stop");
      _status = DS_FAULT;
      _dbptr->robot_init();
      _dbptr->unlockPS();
      _dbptr->stop();

        std::string user_input;
        std::cout << "Press Y/y into teach mode\n";
        std::cin >> user_input;
        if(user_input == "Y" || user_input == "y"){
          set_teach_mode();
          std::cout << "Press any key to cancel teach mode\n";
          std::cin >> user_input;
          quit_teach_mode();
        }

      if (_dbptr->play())
        _status = DS_IDEL;
      return res;
    }

    ROS_INFO_STREAM("_exec_flag : " + std::to_string(_exec_flag));
    if (_exec_flag != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("MoveC execute failed");
      _status = DS_IDEL;
      return DEVICE_EXECUTE_FAILED;
    }
  }
  else if (list[0] == "DO")
  {
    assert(list.size() == 3);
    auto pin = list[1].toLocal8Bit().toStdString();
    auto state = list[2].toLocal8Bit().toStdString();
    if (!_dbptr->setDO(std::stoi(pin), state == "on" ? true : false))
      return DEVICE_EXECUTE_FAILED;
  }
  else if(list[0] == "RE"){
    static Eigen::Isometry3d transformation;
    res = CalculatePoses(transformation);
    if(res != DEVICE_SUCCESS){
      _status = DS_IDEL;
      return res;
    }
    setAttribute("_transformation", &transformation);
  }
  else
  {
    ROS_ERROR_STREAM("Invalid script with start string :" + list[0].toLocal8Bit().toStdString());
    _status = DS_IDEL;
    return DEVICE_INVALID_ATTRIBUTION;
  }
  _status = DS_IDEL;
  return DEVICE_SUCCESS;
}

DeviceErrorCode ur5e_robot::execute_scripts(std::vector<std::string> scripts_in)
{
  if (_status == DS_RUNNING)
    return DEVICE_ISRUNNIN;
  else if (_status == DS_UNCONNECTED)
    return DEVICE_CONNECT_FAILED;
  else if (_status == DS_FAULT)
    return DEVICE_IS_FAULT;

  // update target ids
  _rack_station = std::stoi(*(std::string *)getAttribute("_rack_station"));
  _slot_station = std::stoi(*(std::string *)getAttribute("_slot_station"));
  _rack_robot = std::stoi(*(std::string *)getAttribute("_rack_robot"));
  _slot_robot = std::stoi(*(std::string *)getAttribute("_slot_robot"));

  auto res = DEVICE_SUCCESS;
  _current_station_name = QString::fromLocal8Bit((*(std::string *)getAttribute("_current_station_name")).c_str());
  _current_operation = QString::fromLocal8Bit((*(std::string *)getAttribute("_current_operation")).c_str());
  auto robot_default_joints = _robot_joint_station._joints["robot_default"];
  auto robot_current_joints = _mgptr->getCurrentJointValues();
  if(robot_default_joints && _current_operation == "reset"){
    double dis=0;
    for(int i=0;i<robot_current_joints.size();i++)
      dis += std::pow(robot_current_joints[i] - robot_default_joints[i],2);
    dis = std::sqrt(dis);
    if(dis < 1 / M_PI){
      ROS_INFO_STREAM("Robot is already in default joint,skip reset");
      return DEVICE_SUCCESS;
    }
  }

  DeviceErrorCode code = DEVICE_SUCCESS;
  _current_scripts.swap(scripts_in);
  _check_point = _current_scripts.begin();

  if (_current_scripts.empty())
    return DEVICE_INVALID_ATTRIBUTION;

  auto scripts = QStringList();
  foreach (auto script, _current_scripts)
    scripts.push_back(QString::fromLocal8Bit(script.c_str()));

  std::vector<cv::Range> seq_ranges; // all continuously scripts' range to be executed in scripts_list (J or P)
  auto range = cv::Range(-1, -1);
  for (int i = 0; i < scripts.size(); i++)
  {
    auto list = scripts[i].split(" ");
    assert(list[0] == "J" || list[0] == "P" || list[0] == "C");

    if ((list[0] == "J" || list[0] == "P"))
    {
      if (range.start == -1)
        range.start = i;
      else if (i == scripts.size() - 1)
      {
        range.end = i + 1;
        if (range.size() > 1)
          seq_ranges.push_back(range);
      }
    }
    else if (((list[0] != "J" && list[0] != "P") ||
              i == scripts.size() - 1) &&
             range.start != -1)
    {
      range.end = i;
      if (range.size() > 1)
        seq_ranges.push_back(range);
      range = cv::Range(-1, -1);
    }
  }

  // execute
  _status = DS_RUNNING;
  int seq_id = seq_ranges.empty() ? -1 : 0;
  for (int i = 0; i < scripts.size(); i++)
  {
    // check seq start point
    if (seq_id >= 0 && seq_id < seq_ranges.size() && i == seq_ranges[seq_id].start) // seq of P or J
    {
      _curr_point = _current_scripts.begin() + i;
      // get sequence path
      QStringList seq;
      int j = i;
      for (; j < seq_ranges[seq_id].end; j++)
        seq.push_back(scripts[j]);

      // construct request for sequence path
      int trajectory_invalid_count = 0;
      bool force_r_0 = false;
    ss:
      auto req = ConstructSeqReq(seq, force_r_0);
      if (req.items.empty())
      {
        ROS_ERROR_STREAM("path sequence is empty,construction failed");
        res = DEVICE_INVALID_ATTRIBUTION;
        break;
        // return DEVICE_INVALID_ATTRIBUTION;
      }

      // call plan service
      moveit_msgs::GetMotionSequence srvseq;
      srvseq.response.response.error_code.val = 0;
      srvseq.request.request = req;
      int replan_num = 10;
      int replan_count = 0;
      while (srvseq.response.response.error_code.val != 1 &&
             replan_count < replan_num)
      {
        _client_pilz.call(srvseq);
        if (srvseq.response.response.error_code.val != 1)
        {
          // 将混合半径设为0，再次尝试规划
          ROS_WARN_STREAM("BLENDING FAILED! Try setting blend_radius to 0 and "
                          "plan again...");
          srvseq.request.request = ConstructSeqReq(seq, true);
        }
        ++replan_count;
      }
      if (replan_count == replan_num)
      {
        ROS_ERROR_STREAM("plan sequence_goal failed.ERROR CODE is"
                         << srvseq.response.response.error_code.val);
        // throw(std::runtime_error("plan_failed"));
        res = DEVICE_EXECUTE_FAILED;
        break;
        // return DEVICE_EXECUTE_FAILED;
      }

      // excute
      bool breaked = false;
      auto trajectory = srvseq.response.response.planned_trajectories[0];
      for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
      {
        // if (trajectory.joint_trajectory.points[i].positions[2] < M_PI / 180)
        // {
        //   ROS_WARN_STREAM("Invalid trajectory, " << trajectory.joint_trajectory.joint_names[2] << " joint will be less than 1 degree");
        //   breaked = true;
        //   break;
        // }
      }

      if (breaked)
      {
        trajectory_invalid_count++;
        force_r_0 = true;
        if (trajectory_invalid_count > 1)
          throw(std::runtime_error("Invalid trajectory," + trajectory.joint_trajectory.joint_names[2] + " joint will be less than 1 degree"));
        else
          goto ss;
      }

      _exec_flag = 0;
      _safety_flag = 1;
      _mgptr->asyncExecute(trajectory);
      while (_exec_flag == 0)
      {
        // safty stop
        if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
          break;

        // qt loop wait 100 ms
        ros::Duration(0.1).sleep();
      }

      if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
      {
        res = DEVICE_EXECUTE_FAILED;
        ROS_ERROR_STREAM("Sequence move execute failed by safety stop");
        _status = DS_FAULT;
        _dbptr->robot_init();
        _dbptr->unlockPS();
        _dbptr->stop();

        std::string user_input;
        std::cout << "Press Y/y into teach mode\n";
        std::cin >> user_input;
        if(user_input == "Y" || user_input == "y"){
          set_teach_mode();
          std::cout << "Press any key to cancel teach mode\n";
          std::cin >> user_input;
          quit_teach_mode();
        }

        if (_dbptr->play())
          _status = DS_IDEL;

        //back_to_checkpoint();
        break;
      }

      if (_exec_flag == 1)
        res = DEVICE_SUCCESS;
      else
      {
        res = DEVICE_EXECUTE_FAILED;
        ROS_ERROR_STREAM("Sequence move execute failed,_exec_flag : " + std::to_string(_exec_flag));
        //back_to_checkpoint();
        break;
      }

      // update seq id and check point
      seq_id++;
      i = --j; // for next i++
      _check_point = _current_scripts.begin() + j;
    }
    else // C or DO
    {
      _curr_point = _current_scripts.begin() + i;
      res = execute_script(*_curr_point);
      if (res != DEVICE_SUCCESS)
      {
        ROS_ERROR_STREAM("Script Sequence Excute failed at : [" +
                         std::to_string(i) + "] " +
                         *_curr_point);
        break;
      }
      _check_point = _curr_point;
    }
  }
  if (_status == DS_RUNNING) // DS_FAULT or DS_UNCONNECTED 保留状态不处理
    _status = DS_IDEL;
  return res;
}

DeviceErrorCode ur5e_robot::back_to_checkpoint()
{
  ROS_INFO_STREAM("Go back to ur5e robot check point ...");
  auto backward_script = std::vector<std::string>(_current_scripts.begin(), _check_point + 2);
  std::reverse(backward_script.begin(), backward_script.end());

  std::vector<std::string> scripts;
  if(backward_script.size() > 1){
    for (int i = 1; i < backward_script.size(); i++)
    {
      auto list_0 = splitstr(backward_script[i - 1], ' ');
      auto list = splitstr(backward_script[i], ' ');
      if (list[0] == "C")
      {
        // //check C script, cause point1 and point2 need swapped to inverse path
        // std::iter_swap(list.begin() + 1, list.begin() + 2);//swap point1 and point2
        // std::string script;
        // for (int j = 0; j < list.size(); j++)
        //   script += list[j] + " ";
        // script.pop_back();//pop back the last " "
        // backward_script[i] = script;//replace the "C" script

        ROS_ERROR_STREAM("Can not go back to check point for C script");
        return DEVICE_EXECUTE_FAILED;
      }
      else if (list[0] == "P" || list[0] == "J")
      {
        auto script_temp = list[0] + " " +
                          list[1] + " " +
                          list[2] + " " +
                          list[3] + " " +
                          list[4] + " " +
                          list_0[5]; // planid should be
        scripts.push_back(script_temp);
      }
    }
  }
  else if(backward_script.size() == 1)
  {
    scripts.push_back(backward_script[0]);
  }  

  // execute inverse path
  DeviceErrorCode code;
  if(scripts.size() > 1)
    code = execute_scripts(scripts);
  else
    code = execute_script(scripts[0]);

  if (code == DEVICE_SUCCESS)
    ROS_INFO_STREAM("Go back to ur5e robot check point ok");
  else
    ROS_INFO_STREAM("Go back to ur5e robot check point failed");
  return code;
}

DeviceErrorCode ur5e_robot::getdata(std::string dataname, std::string &data)
{
  data.clear();
  try
  {
    if (dataname == "_safety_flag")
    {
      data = std::to_string(_safety_flag);
    }
    else if (dataname == "_exec_flag")
    {
      data = std::to_string(_exec_flag);
    }
    else if (dataname == "_current_station_name")
    {
      data = _current_station_name.toLocal8Bit().toStdString();
    }
    else if (dataname == "_current_operation")
    {
      data = _current_operation.toLocal8Bit().toStdString();
    }
    else
      return DEVICE_INVALID_ATTRIBUTION;
  }
  catch (std::string e)
  {
    ROS_ERROR_STREAM("Robot " + dataname + " failed,error message:" + e);
    return DEVICE_CONNECT_FAILED;
  }
  return DEVICE_SUCCESS;
}

bool ur5e_robot::set_teach_mode()
{
  //设置自由驱动模式前，需要暂时关闭remote control的重连机制
  _dbptr->stop();

  std_msgs::String msg;
  msg.data =
"def free():\n\
  freedrive_mode()\n\
  while (True) :\n\
    sync()\n\
  end\n\
end\n";
  _pub_send_scripts_.publish(msg);
  return true;
}

bool ur5e_robot::quit_teach_mode()
{
  //先停止自由驱动程序: free()
  _dbptr->stop();
  //再执行退出自由驱动程序: unfree()
  std_msgs::String msg;
   msg.data = 
"def unfree():\n\
  end_freedrive_mode()\n\
end\n";
  _pub_send_scripts_.publish(msg);
  
  //等待unfree程序执行完毕
  ros::Duration(1).sleep();
  //取消自由驱动模式后，需要重启remote control的重连机制
  _dbptr->play();
  return true;
}

void ur5e_robot::safety_Callback(
    const ur_dashboard_msgs::SafetyModeConstPtr &safetymode)
{
  this->_safety_flag = safetymode->mode;
}

void ur5e_robot::exec_Callback(
    const moveit_msgs::ExecuteTrajectoryActionResultConstPtr &execresult)
{
  this->_exec_flag = execresult->result.error_code.val;
}

bool ur5e_robot::GetPoseConstraints(QString name, QString fuc, bool isrobotjoint,
                                    moveit_msgs::Constraints &constraints, geometry_msgs::PoseStamped &next_pose)
{
  assert(fuc == "P" || fuc == "J");

  if (fuc == "J")
  {
    if (isrobotjoint) // robot_joint
    {
      if (_robot_joint_station.station_name == "")
      {
        ROS_ERROR_STREAM("file: \"机械臂关节.json\" not found");
        return false;
      }

      auto robotjoints = _robot_joint_station._joints;
      auto iter = robotjoints.find(name);
      if (iter == robotjoints.end())
      {
        ROS_ERROR_STREAM("robot joint point : " +
                         name.toLocal8Bit().toStdString() + " not found");
        return false;
      }
    }
    else // station_joints
    {
      if (_robot_record_data[_current_station_name]._joints.find(name) ==
          _robot_record_data[_current_station_name]._joints.end())
      {
        ROS_ERROR_STREAM("no joint found,jointname:" +
                         name.toLocal8Bit().toStdString());
        return false;
      }
    }
  }
  else // station_pose
  {
    if (_robot_record_data[_current_station_name]._poses.find(name) ==
            _robot_record_data[_current_station_name]._poses.end() ||
        _robot_record_data[_current_station_name]._joints.find(name + "_inverse") ==
            _robot_record_data[_current_station_name]._joints.end())
    {
      ROS_ERROR_STREAM("no pose or pose_inverse_joint found,posename:" +
                       name.toLocal8Bit().toStdString());
      return false;
    }
  }

  // Robot State for calculating Constraints
  robot_state::RobotState rstate(rmodel);
  rstate.setToDefaultValues();
  auto endlink = _mgptr->getEndEffectorLink();
  if (fuc == "J")
  {
    double *jt = nullptr;
    if (isrobotjoint)
      jt = _robot_joint_station._joints[name];
    else // station_joint
      jt = _robot_record_data[_current_station_name]._joints[name];

    std::vector<double> joints(jt, jt + 6);
    rstate.setVariablePositions(_joints_names, joints);
    Eigen::Isometry3d mat = rstate.getGlobalLinkTransform(endlink);
    next_pose.pose.position.x = mat.translation()[0];
    next_pose.pose.position.y = mat.translation()[1];
    next_pose.pose.position.z = mat.translation()[2];
  }
  else //"P"
  {
    // get pose to marker
    auto pos = _robot_record_data[_current_station_name]._poses[name];
    next_pose.pose.position.x = pos[0];
    next_pose.pose.position.y = pos[1];
    next_pose.pose.position.z = pos[2];
    next_pose.pose.orientation.x = pos[3];
    next_pose.pose.orientation.y = pos[4];
    next_pose.pose.orientation.z = pos[5];
    next_pose.pose.orientation.w = pos[6];

    // get pose to base
    Eigen::Isometry3d transformation = *(Eigen::Isometry3d *)getAttribute("_transformation");
    auto base_pose = transformation * toMatrix(next_pose.pose);
    next_pose.pose = toPose(base_pose);

    // inverse joints
    auto joint_inverse = _robot_record_data[_current_station_name]._joints[name + "_inverse"];
    
    //use ur kinematics solution first, get rough but currect joints
    double q_sols[8*6];//8 solutions
    int num_sols = _kinematics->inverse(base_pose, q_sols);
    auto id = _kinematics->get_nearest_solution(q_sols, num_sols, joint_inverse);
    std::vector<double> solution_joints(&q_sols[id * 6], &q_sols[id * 6] + 6);
    rstate.setVariablePositions(_joints_names, {solution_joints[0], solution_joints[1], solution_joints[2],
                                                solution_joints[3], solution_joints[4], solution_joints[5]});

    //use ik trace solution, get accurate solution
    double iktrace_joint[6] = {0};
    rstate.setFromIK(rmodel->getJointModelGroup("manipulator"), next_pose.pose, _mgptr->getEndEffectorLink());
    rstate.copyJointGroupPositions(rmodel->getJointModelGroup("manipulator"), iktrace_joint);
  }
  constraints = kinematic_constraints::constructGoalConstraints(
      rstate, rmodel->getJointModelGroup("manipulator"));
  return true;
}

bool ur5e_robot::asyncExecutePose(geometry_msgs::Pose pose){
  robot_state::RobotState rstate(rmodel);
  double q_sols[8 * 6]; // 8 solutions
  int num_sols = _kinematics->inverse(toMatrix(pose), q_sols);
  auto id = _kinematics->get_nearest_solution(q_sols, num_sols, &(_mgptr->getCurrentJointValues()[0]));
  std::vector<double> solution_joints(&q_sols[id * 6], &q_sols[id * 6] + 6);
  rstate.setVariablePositions(_joints_names, {solution_joints[0], solution_joints[1], solution_joints[2],
                                              solution_joints[3], solution_joints[4], solution_joints[5]});
  // double iktrace_joint[6] = {0};
  rstate.setFromIK(rmodel->getJointModelGroup("manipulator"), pose, "tool0");
  auto constraints = kinematic_constraints::constructGoalConstraints(
      rstate, rmodel->getJointModelGroup("manipulator"));

  moveit_msgs::MotionPlanRequest req;
  _mgptr->constructMotionPlanRequest(req);
  req.group_name = "manipulator";
  req.goal_constraints.resize(1);
  req.goal_constraints[0] = constraints;
  req.max_velocity_scaling_factor = 0.15;
  req.max_acceleration_scaling_factor = 0.1;
  req.planner_id = "PTP";

  robot_state::robotStateToRobotStateMsg(
      *(_mgptr->getCurrentState()),
      req.start_state); // set current robot state to start state

  moveit_msgs::GetMotionPlan srvreq;
  srvreq.request.motion_plan_request = req;
  srvreq.response.motion_plan_response.error_code.val = 0;

  _client_kinematic.call(srvreq);
  if (srvreq.response.motion_plan_response.error_code.val != 1)
  {
    ROS_ERROR_STREAM("asyncExecutePose plan failed");
    return false;
  }

  _exec_flag = 0;
  _safety_flag = 1;
  auto trajectory = srvreq.response.motion_plan_response.trajectory;
  // for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
  // {
  //   if (trajectory.joint_trajectory.points[i].positions[2] < M_PI / 180)
  //   {
  //     ROS_ERROR_STREAM("Invalid trajectory, " << trajectory.joint_trajectory.joint_names[2] << " joint will be less than 1 degree");
  //     _status = DS_IDEL;
  //     return DEVICE_EXECUTE_FAILED;
  //   }
  // }
  _mgptr->asyncExecute(trajectory);
  while (_exec_flag == 0)
  {
    // safty stop
    if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
      break;

    // qt loop wait 100 ms
    ros::Duration(0.1).sleep();
  }

  if (_safety_flag == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP || _safety_flag == ur_dashboard_msgs::SafetyMode::FAULT)
  {
    ROS_ERROR_STREAM("asyncExecutePose execute failed by safety stop");
    _status = DS_FAULT;
    _dbptr->robot_init();
    _dbptr->unlockPS();
    _dbptr->stop();

    std::string user_input;
    std::cout << "Press Y/y into teach mode\n";
    std::cin >> user_input;
    if (user_input == "Y" || user_input == "y"){
      set_teach_mode();
      std::cout << "Press any key to cancel teach mode\n";
      std::cin >> user_input;
      quit_teach_mode();
    }

    if (_dbptr->play())
      _status = DS_IDEL;
    return false;
  }

  ROS_INFO_STREAM("_exec_flag : " + std::to_string(_exec_flag));
  if (_exec_flag != moveit::core::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR_STREAM("asyncExecutePose execute failed");
    _status = DS_IDEL;
    return false;
  }
  return true;
}

Isometry3d emptyIsometry3d() {
  double pose[7] = {0};
  // pose[5] = 1;//rz = 1
  pose[6] = 1; // rw = 1
  return toMatrix(pose);
}

//#define testmode
DeviceErrorCode ur5e_robot::CalculatePoses(Isometry3d& marker2base,bool remove_xy) {
  marker2base = emptyIsometry3d();
#ifdef testmode
  return DEVICE_SUCCESS;
#endif
  geometry_msgs::PoseStamped start_pose = _mgptr->getCurrentPose();

  if (_current_station_name == "") {
    ROS_ERROR_STREAM("CalculatePoses failed,no statoion selected");
    return DEVICE_LACK_ATTRIBUTION;
  }
  std_msgs::Int32 op_code;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  op_code.data = _robot_record_data[_current_station_name]._marker.id;

  // 等待第一次重定位的位姿
  if (_pub_camera_op_.getNumSubscribers() == 0)
  {
    ROS_ERROR_STREAM("relocation_error, no camera pose calculator exist");
    return DEVICE_CV_LOCATOR_OFFLINE;
  }
  _pub_camera_op_.publish(op_code);
  geometry_msgs::PoseStampedConstPtr camera_pose =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          "/point_coordinate", ros::Duration(3));
  if (!camera_pose)
  {
    ROS_ERROR_STREAM("relocation_error, overtimed");
    return DEVICE_RELOCATION_FAILED;
  }

  if (camera_pose->pose.orientation.w == 0) {
    std::vector<double> towards{-0.05, 0.1};
    for (double each : towards) {
      MoveLine_pilz("y+", each, 0.15);
      _pub_camera_op_.publish(op_code);
      camera_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          "/point_coordinate", ros::Duration(3));
      if (!camera_pose)
      {
        ROS_ERROR_STREAM("relocation_error, overtimed");
        return DEVICE_RELOCATION_FAILED;
      }
    }
  }
  if (camera_pose->pose.orientation.w == 0)
  {
    ROS_ERROR_STREAM("relocation_error, no pose found");
    return DEVICE_RELOCATION_FAILED;
  }

  geometry_msgs::PoseStamped marker_pose =
      tfBuffer.transform(*camera_pose, "base_link", ros::Duration(1));

  // 如果有标定重定位的位姿，则移动到该位姿
  QString targetpose_name;
  foreach (auto iter, _robot_record_data[_current_station_name]._poses) {
    if (iter.first.contains("重定位")) {
      targetpose_name = iter.first;
      break;
    }
  }
  if (!targetpose_name.isEmpty()) {
    // 重定位次数
    size_t re_num = 2;
    Isometry3d mTre_ = toMatrix(
        _robot_record_data[_current_station_name]._poses[targetpose_name]);
    
    //根据重定位的inverse角度，求机械臂原始重定位位姿
    auto inverse_joints = _robot_record_data[_current_station_name]._joints[targetpose_name + "_inverse"];
    Eigen::Isometry3d pose_relocation;
    _kinematics->forward(inverse_joints,pose_relocation);
    start_pose.pose = toPose(pose_relocation);
    std::cout << start_pose << std::endl;

    for (size_t i = 0; i < re_num; ++i) {
      Isometry3d bTm_ = toMatrix(marker_pose.pose);
      // if (remove_xy)
      //   bTm_ = remove_xy_rotaion(bTm_, marker_pose.pose.orientation);
      Isometry3d bTre_ = bTm_ * mTre_;
      auto target_pose = toPose(bTre_);
      if(!asyncExecutePose(target_pose))
        return DEVICE_EXECUTE_FAILED;

      // 等待第i+1次重定位的位姿
      _pub_camera_op_.publish(op_code);
      camera_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          "/point_coordinate", ros::Duration(3));
      if (!camera_pose)
      {
        ROS_ERROR_STREAM("relocation_error, overtimed");
        return DEVICE_RELOCATION_FAILED;
      }

      if (camera_pose->pose.orientation.w == 0)
      {
        ROS_ERROR_STREAM("relocation_error, no pose found");
        return DEVICE_RELOCATION_FAILED;
      }

      marker_pose =
          tfBuffer.transform(*camera_pose, "base_link", ros::Duration(1));
    }

    ROS_INFO_STREAM(_current_station_name.toLocal8Bit().toStdString()
                    << ":" << op_code.data << std::endl
                    << *camera_pose);
  }

  // 将marker_pose转为齐次变换矩阵bTm(Base_link到Marker_link的变换)
  Isometry3d bTm = toMatrix(marker_pose.pose);
  if (remove_xy)
    bTm = remove_xy_rotaion(bTm, marker_pose.pose.orientation);
  marker_pose.pose = toPose(bTm);
  ROS_INFO_STREAM(_current_station_name.toLocal8Bit().toStdString()
                  << ":" << op_code.data << std::endl
                  << marker_pose);

  geometry_msgs::PoseStamped end_pose = _mgptr->getCurrentPose();
  auto dis = std::sqrt(std::pow(end_pose.pose.position.x - start_pose.pose.position.x, 2) +
                       std::pow(end_pose.pose.position.y - start_pose.pose.position.y, 2) +
                       std::pow(end_pose.pose.position.z - start_pose.pose.position.z, 2));
  std::cout << "dis = " << std::to_string(dis) << std::endl;
  if(dis > relocation_tolerance){
    ROS_ERROR_STREAM("Relocation end_pose faraway from start_pose,dis = " << std::to_string(dis));
    return DEVICE_RELOCATION_FAILED;
  }
  marker2base = bTm;
  return DEVICE_SUCCESS;
}

void ur5e_robot::MoveLine_pilz(QString towards, double dis,
                                     double speed_factor) {
  moveit::planning_interface::MoveGroupInterface::Plan Lineplan;
  geometry_msgs::PoseStamped target_pose = _mgptr->getCurrentPose();
  if (towards == "z+")
    target_pose.pose.position.z += dis;
  else if (towards == "z-")
    target_pose.pose.position.z -= dis;
  else if (towards == "x-")
    target_pose.pose.position.x -= dis;
  else if (towards == "x+")
    target_pose.pose.position.x += dis;
  else if (towards == "y+")
    target_pose.pose.position.y += dis;
  else if (towards == "y-")
    target_pose.pose.position.y -= dis;

  _mgptr->setPlannerId("PTP");
  _mgptr->setPoseTarget(target_pose);
  _mgptr->setMaxVelocityScalingFactor(speed_factor);
  if (_mgptr->plan(Lineplan) !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR_STREAM("plan line trajectory FAILED");
    return;
  }
  _mgptr->execute(Lineplan);
  _mgptr->setMaxVelocityScalingFactor(0.15);
  _mgptr->setPlannerId("PTP");
}

bool ur5e_robot::get_dynamic_point_name(
    QString &pointname, QString station_name, QString operation,
    std::map<QString, std::pair<QString, QString>> operation_to_pointname,
    int slot_num_per_rack,
    int rack_station, int slot_station, int rack_robot, int slot_robot)
{
  // return true if pointname belong to robot joints
  bool res = false;
  if (pointname.contains("robot_operation"))
  {
    pointname = pointname.replace(
        "robot_operation",
        "robot_" + operation_to_pointname[operation].first);
  }
  else if (pointname.contains("station_operation"))
  {
    pointname = pointname.replace(
        "station_operation",
        station_name + "_" + operation_to_pointname[operation].second);
  }

  if (operation == "move") // operation move need station_rack and station_slot,but used as robot_rack and robot_slot
  {
    if (pointname.contains("_i_")) // dynamic point
    {
      res = true;
      if (pointname.contains(station_name + "_")) // station_operation in move
      {
        // remove station name if contains
        pointname = pointname.remove(station_name + "_");
        pointname = pointname.replace("_i_", "_" + QString("%1").arg(slot_num_per_rack * rack_station + slot_station) + "_");
      }
      else // robot_operation in move
      {
        pointname = pointname.replace("_i_", "_" + QString("%1").arg(slot_num_per_rack * rack_robot + slot_robot) + "_");
      }
    }
    else
    {
      // nothing to do for static point
    }
  }
  else if (pointname.contains("robot_")) // robot joints
  {
    res = true;
    if (pointname.contains("_i_")) // dynamic point
    {
      if (pointname.contains("rack"))
      {
        pointname = pointname.replace(
            "_i_", "_" + QString("%1").arg(rack_robot) + "_");
      }
      else if (pointname.contains("hole"))
      {
        // 中转洞
        if (pointname.contains("interim_hole_"))
        {
          // nothing todo
        }
        // 试管架洞
        else
        {
          pointname = pointname.replace(
              "_i_", "_" + QString("%1").arg(slot_num_per_rack * rack_robot + slot_robot) + "_");
        }
      }
    }
    else
    {
      // nothing to do for static point
    }
  }
  else if (pointname.contains(station_name + "_")) // station joints
  {
    if (pointname.contains("_i_")) // dynamic point
    {
      if (pointname.contains("rack"))
      {
        pointname = pointname.replace(
            "_i_", "_" + QString("%1").arg(rack_station) + "_");
      }
      else
      {
        pointname = pointname.replace(
            "_i_", "_" + QString("%1").arg(slot_num_per_rack * rack_station + slot_station) + "_");
      }
    }
    else
    {
      // nothing to do for static point
    }
  }
  else
  {
    // pointname stay unchaged
  }
  // ROS_INFO_STREAM("Point name: " << pointname.toLocal8Bit().toStdString());
  return res;
}

moveit_msgs::MotionSequenceRequest ur5e_robot::ConstructSeqReq(QStringList path_scripts, bool force_blend_radius_0)
{
  moveit_msgs::MotionSequenceRequest sequenceRequest;
  auto curr_pose = _mgptr->getCurrentPose();
  int i = 0;
  ROS_INFO_STREAM("scripts size:" + std::to_string(path_scripts.size()));
  for (; i < path_scripts.size() - 1; i++)
  {
    auto list_curr = path_scripts[i].split(" ");
    auto list_next = path_scripts[i + 1].split(" ");
    assert((list_curr.size() == 6 && list_next.size() == 6));
    assert((list_curr[0] == "J" || list_curr[0] == "P") &&
           (list_next[0] == "J" || list_next[0] == "P"));
    ROS_INFO_STREAM("script " << i << ": [ " << path_scripts[i].toLocal8Bit().toStdString() << " ]");

    auto fuc = list_curr[0];
    auto pointname_curr = list_curr[1];
    auto pointname_next = list_next[1];
    auto r = list_curr[2].toDouble();
    auto speed = list_curr[3].toDouble();
    auto acc = list_curr[4].toDouble();
    auto planid = list_curr[5];
    auto pointname_curr_old = pointname_curr;

    // dynamic point
    bool isrobotjoint = get_dynamic_point_name(
        pointname_curr, _current_station_name, _current_operation,
        _robot_record_data[_current_station_name]._cmd_to_pointname, 10,
        _rack_station, _slot_station, _rack_robot, _slot_robot);
    if (pointname_curr_old != pointname_curr)
    {
      // is dynamic point
      ROS_INFO_STREAM("Dynamic point name: " << pointname_curr.toLocal8Bit().toStdString());
    }
    geometry_msgs::PoseStamped next_pose;
    moveit_msgs::Constraints constraints;
    if (!GetPoseConstraints(pointname_curr, fuc, isrobotjoint, constraints,
                            next_pose))
    {
      sequenceRequest.items.clear();
      return sequenceRequest;
    }

    planning_interface::MotionPlanRequest req;
    req.group_name = "manipulator";
    req.goal_constraints.resize(1);
    req.goal_constraints[0] = constraints;
    req.max_velocity_scaling_factor = speed;
    req.max_acceleration_scaling_factor = acc;
    req.planner_id = planid.toLocal8Bit().toStdString();

    // dynamic point
    isrobotjoint = get_dynamic_point_name(
        pointname_next, _current_station_name, _current_operation,
        _robot_record_data[_current_station_name]._cmd_to_pointname, 10,
        _rack_station, _slot_station, _rack_robot, _slot_robot);
    fuc = list_next[0];
    geometry_msgs::PoseStamped n_next_pose;
    if (!GetPoseConstraints(pointname_next, fuc, isrobotjoint, constraints,
                            n_next_pose))
    {
      sequenceRequest.items.clear();
      return sequenceRequest;
    }

    moveit_msgs::MotionSequenceItem sequenceItem;
    sequenceItem.req = req;
    sequenceItem.blend_radius =
        force_blend_radius_0 ? 0
                             : r * GetBlendradius(curr_pose.pose, next_pose.pose,
                                                  n_next_pose.pose);
    sequenceRequest.items.emplace_back(sequenceItem);

    curr_pose = next_pose; // update current pose
  }
  // last pointq
  ROS_INFO_STREAM("script " << i << ": [ " << path_scripts[i].toLocal8Bit().toStdString() << " ]");
  auto list_last = path_scripts.back().split(" ");
  assert(list_last.size() == 6);
  auto fuc = list_last[0];
  auto pointname_last = list_last[1];
  auto speed = list_last[3].toDouble();
  auto acc = list_last[4].toDouble();
  auto planid = list_last[5];
  auto pointname_last_old = pointname_last;
  geometry_msgs::PoseStamped last_pose;
  moveit_msgs::Constraints constraints;
  bool isrobotjoint = get_dynamic_point_name(
      pointname_last, _current_station_name, _current_operation,
      _robot_record_data[_current_station_name]._cmd_to_pointname, 10,
      _rack_station, _slot_station, _rack_robot, _slot_robot);
  if (pointname_last_old != pointname_last)
  {
    // is dynamic point
    ROS_INFO_STREAM("Dynamic point name: " << pointname_last.toLocal8Bit().toStdString());
  }
  if (!GetPoseConstraints(pointname_last, fuc, isrobotjoint, constraints,
                          last_pose))
  {
    sequenceRequest.items.clear();
    return sequenceRequest;
  }
  moveit_msgs::MotionPlanRequest req;
  req.group_name = "manipulator";
  req.goal_constraints.resize(1);
  req.goal_constraints[0] = constraints;
  req.max_velocity_scaling_factor = list_last[3].toDouble();
  req.max_velocity_scaling_factor = speed;
  req.max_acceleration_scaling_factor = acc;
  req.planner_id = planid.toLocal8Bit().toStdString();

  moveit_msgs::MotionSequenceItem item_last;
  item_last.blend_radius = 0.0;
  item_last.req = req;
  sequenceRequest.items.emplace_back(item_last);

  return sequenceRequest;
}

double ur5e_robot::GetBlendradius(geometry_msgs::Pose curr_pose,
                                  geometry_msgs::Pose next_pose, geometry_msgs::Pose n_next_pose)
{
  // 若curr_pose和n_next_pose重合,将融合半径置0
  if (std::pow(n_next_pose.position.x - curr_pose.position.x, 2) +
          std::pow(n_next_pose.position.y - curr_pose.position.y, 2) +
          std::pow(n_next_pose.position.z - curr_pose.position.z, 2) <
      0.0025)
    return 0.0;
  double radius1 =
      std::sqrt(std::pow(next_pose.position.x - curr_pose.position.x, 2) +
                std::pow(next_pose.position.y - curr_pose.position.y, 2) +
                std::pow(next_pose.position.z - curr_pose.position.z, 2));
  double radius2 =
      std::sqrt(std::pow(n_next_pose.position.x - next_pose.position.x, 2) +
                std::pow(n_next_pose.position.y - next_pose.position.y, 2) +
                std::pow(n_next_pose.position.z - next_pose.position.z, 2));
  double radius = 0.5 * std::min(radius1, radius2);
  if (radius < 0.02)
    return 0.0;
  return radius;
}

bool RobotJsonDataPaser(QString filename, robot_record_data &data)
{
  data.clear();
  // init json file data
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly))
  {
    qCritical() << "open file error,file name: " << file.fileName();
    return false;
  }
  auto rawdata = file.readAll();
  file.close();

  QJsonParseError jerr;
  QJsonDocument doc = QJsonDocument::fromJson(rawdata, &jerr);
  if (jerr.error != QJsonParseError::NoError)
  {
    qCritical() << "json paser error," << jerr.errorString()
                << ", file name:" << file.fileName();
    return false;
  }
  QJsonObject root = doc.object();

  if (root.contains("joints"))
  {
    QJsonArray joints = root["joints"].toArray();
    for (int i = 0; i < joints.count(); i++)
    {
      auto joint = joints[i].toObject();
      auto jointname = joint["name"].toString();
      auto value = joint["value"].toArray();
      assert(value.size() == 6);
      for (int j = 0; j < 6; j++)
        data._joints[jointname][j] = value[j].toDouble();
    }
  }
  if (root.contains("poses"))
  {
    QJsonArray poses = root["poses"].toArray();
    for (int i = 0; i < poses.size(); i++)
    {
      auto pose = poses[i].toObject();
      auto posename = pose["name"].toString();
      auto value = pose["value"].toArray();
      assert(value.size() == 7);
      for (int j = 0; j < 7; j++)
        data._poses[posename][j] = value[j].toDouble();
    }
  }
  if (root.contains("marker"))
  {
    auto marker = root["marker"].toObject();
    auto size = marker["size"].toArray();
    assert(size.size() == 2);
    data._marker.id = marker["id"].toInt();
    data._marker.size[0] = size[0].toDouble();
    data._marker.size[1] = size[1].toDouble();
  }
  if (root.contains("cmd"))
  {
    auto cmd = root["cmd"].toArray();
    for (int i = 0; i < cmd.size(); i++)
    {
      auto cmdi = cmd[i].toObject();
      auto cmdname = cmdi["name"].toString();
      auto scripts = cmdi["scripts"].toArray();

      data._cmds[cmdname] = QStringList();
      for (int j = 0; j < scripts.size(); j++)
        data._cmds[cmdname].push_back(scripts[j].toString());
    }
  }
  if (root.contains("cmd_to_pointname"))
  {
    auto cmd_to_pointname = root["cmd_to_pointname"].toArray();
    for (int i = 0; i < cmd_to_pointname.size(); i++)
    {
      auto cmd_to_pointnamei = cmd_to_pointname[i].toObject();
      auto cmd = cmd_to_pointnamei["cmd"].toString();
      auto robot_pointname = cmd_to_pointnamei["robot_pointname"].toString();
      auto station_pointname = cmd_to_pointnamei["station_pointname"].toString();
      data._cmd_to_pointname[cmd].first = robot_pointname;
      data._cmd_to_pointname[cmd].second = station_pointname;
    }
  }
  QFileInfo info(filename);
  data.station_name = info.baseName();
  return true;
}

bool RobotJsonDataSave(QString filename, robot_record_data data)
{

  QJsonDocument doc;
  QJsonObject root, marker;
  QJsonArray cmds, joints, poses, markersize, cmd_to_pointnames;

  // marker
  markersize.append(data._marker.size[0]);
  markersize.append(data._marker.size[1]);
  marker["id"] = data._marker.id;
  marker["size"] = markersize;
  root["marker"] = marker;

  // joints
  foreach (auto jt, data._joints)
  {
    QJsonObject joint;
    QJsonArray value;

    int count = sizeof(jt.second) / sizeof(jt.second[0]);
    for (int j = 0; j < count; j++)
      value.append(jt.second[j]);

    joint["name"] = jt.first;
    joint["value"] = value;
    joints.push_back(joint);
  }
  root["joints"] = joints;

  // poses
  foreach (auto pos, data._poses)
  {
    QJsonObject pose;
    QJsonArray value;

    int count = sizeof(pos.second) / sizeof(pos.second[0]);
    for (int j = 0; j < count; j++)
      value.append(pos.second[j]);

    pose["name"] = pos.first;
    pose["value"] = value;
    poses.push_back(pose);
  }
  root["poses"] = poses;

  // cmds
  foreach (auto _cmd, data._cmds)
  {
    QJsonObject cmd;
    QJsonArray scripts;

    for (int j = 0; j < _cmd.second.size(); j++)
      scripts.append(_cmd.second[j]);

    cmd["name"] = _cmd.first;
    cmd["scripts"] = scripts;
    cmds.append(cmd);
  }
  root["cmd"] = cmds;

  // cmd_to_pointname
  auto iter = data._cmd_to_pointname.begin();
  for (int i = 0; i < data._cmd_to_pointname.size(); i++)
  {
    QJsonObject cmd_to_pointnamei;
    cmd_to_pointnamei["cmd"] = iter->first;
    cmd_to_pointnamei["robot_pointname"] = iter->second.first;
    cmd_to_pointnamei["station_pointname"] = iter->second.second;
    iter++;
    cmd_to_pointnames.append(cmd_to_pointnamei);
  }
  root["cmd_to_pointname"] = cmd_to_pointnames;

  doc.setObject(root);
  // save file
  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly))
  {
    qCritical() << "open file error,file name: " << file.fileName();
    return false;
  }
  file.write(doc.toJson());
  file.close();
  return true;
}

/************************ur5e_robot end**************************/

/************************ur5e_kinematics start**************************/

ur5e_kinematics::ur5e_kinematics(ros::NodeHandle& nh){
  nh.param<double>("/ur_hardware_interface/kinematics/shoulder/z", d1, 0.1625);
  nh.param<double>("/ur_hardware_interface/kinematics/forearm/x", a2, -0.425);
  nh.param<double>("/ur_hardware_interface/kinematics/wrist_1/x", a3, -0.3922);
  nh.param<double>("/ur_hardware_interface/kinematics/wrist_1/z", d4, 0.1333);
  nh.param<double>("/ur_hardware_interface/kinematics/wrist_2/y", d5, -0.0997);
  nh.param<double>("/ur_hardware_interface/kinematics/wrist_3/y", d6, 0.0996);
  d5*=-1;//d5的值，算法需要的值与ur机械臂的配置文件符号相反
}

  void ur5e_kinematics::forward(const double *q, double *T)
  {
    double s1 = sin(*q), c1 = cos(*q);
    q++;
    double q234 = *q, s2 = sin(*q), c2 = cos(*q);
    q++;
    double s3 = sin(*q), c3 = cos(*q);
    q234 += *q;
    q++;
    q234 += *q;
    q++;
    double s5 = sin(*q), c5 = cos(*q);
    q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s234 = sin(q234), c234 = cos(q234);

    *T = ((c1 * c234 - s1 * s234) * s5) / 2.0 - c5 * s1 + ((c1 * c234 + s1 * s234) * s5) / 2.0;
    T++;
    *T = (c6 * (s1 * s5 + ((c1 * c234 - s1 * s234) * c5) / 2.0 + ((c1 * c234 + s1 * s234) * c5) / 2.0) -
          (s6 * ((s1 * c234 + c1 * s234) - (s1 * c234 - c1 * s234))) / 2.0);
    T++;
    *T = (-(c6 * ((s1 * c234 + c1 * s234) - (s1 * c234 - c1 * s234))) / 2.0 -
          s6 * (s1 * s5 + ((c1 * c234 - s1 * s234) * c5) / 2.0 + ((c1 * c234 + s1 * s234) * c5) / 2.0));
    T++;
    *T = ((d5 * (s1 * c234 - c1 * s234)) / 2.0 - (d5 * (s1 * c234 + c1 * s234)) / 2.0 -
          d4 * s1 + (d6 * (c1 * c234 - s1 * s234) * s5) / 2.0 + (d6 * (c1 * c234 + s1 * s234) * s5) / 2.0 -
          a2 * c1 * c2 - d6 * c5 * s1 - a3 * c1 * c2 * c3 + a3 * c1 * s2 * s3);
    T++;
    *T = c1 * c5 + ((s1 * c234 + c1 * s234) * s5) / 2.0 + ((s1 * c234 - c1 * s234) * s5) / 2.0;
    T++;
    *T = (c6 * (((s1 * c234 + c1 * s234) * c5) / 2.0 - c1 * s5 + ((s1 * c234 - c1 * s234) * c5) / 2.0) +
          s6 * ((c1 * c234 - s1 * s234) / 2.0 - (c1 * c234 + s1 * s234) / 2.0));
    T++;
    *T = (c6 * ((c1 * c234 - s1 * s234) / 2.0 - (c1 * c234 + s1 * s234) / 2.0) -
          s6 * (((s1 * c234 + c1 * s234) * c5) / 2.0 - c1 * s5 + ((s1 * c234 - c1 * s234) * c5) / 2.0));
    T++;
    *T = ((d5 * (c1 * c234 - s1 * s234)) / 2.0 - (d5 * (c1 * c234 + s1 * s234)) / 2.0 + d4 * c1 +
          (d6 * (s1 * c234 + c1 * s234) * s5) / 2.0 + (d6 * (s1 * c234 - c1 * s234) * s5) / 2.0 + d6 * c1 * c5 -
          a2 * c2 * s1 - a3 * c2 * c3 * s1 + a3 * s1 * s2 * s3);
    T++;
    *T = ((c234 * c5 - s234 * s5) / 2.0 - (c234 * c5 + s234 * s5) / 2.0);
    T++;
    *T = ((s234 * c6 - c234 * s6) / 2.0 - (s234 * c6 + c234 * s6) / 2.0 - s234 * c5 * c6);
    T++;
    *T = (s234 * c5 * s6 - (c234 * c6 + s234 * s6) / 2.0 - (c234 * c6 - s234 * s6) / 2.0);
    T++;
    *T = (d1 + (d6 * (c234 * c5 - s234 * s5)) / 2.0 + a3 * (s2 * c3 + c2 * s3) + a2 * s2 -
          (d6 * (c234 * c5 + s234 * s5)) / 2.0 - d5 * c234);
    T++;
    *T = 0.0;
    T++;
    *T = 0.0;
    T++;
    *T = 0.0;
    T++;
    *T = 1.0;
  }

  void ur5e_kinematics::forward(const double *q, Eigen::Isometry3d& mat){
    mat = Eigen::Isometry3d::Identity();
    int rows = mat.matrix().rows();
    int cols = mat.matrix().cols();

    double matrix0[16] = {0};
    forward(q,matrix0);

    for (int i = 0; i < rows; i++)
      for (int j = 0; j < cols; j++)
      mat.matrix().data()[j * rows + i] = matrix0[i * cols + j];
  }

  void ur5e_kinematics::forward_all(const double *q, double *T1, double *T2, double *T3,
                   double *T4, double *T5, double *T6)
  {
    double s1 = sin(*q), c1 = cos(*q);
    q++; // q1
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q);
    q++; // q2
    double s3 = sin(*q), c3 = cos(*q);
    q23 += *q;
    q234 += *q;
    q++; // q3
    q234 += *q;
    q++; // q4
    double s5 = sin(*q), c5 = cos(*q);
    q++;                               // q5
    double s6 = sin(*q), c6 = cos(*q); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);

    if (T1 != NULL)
    {
      *T1 = c1;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = s1;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = s1;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = -c1;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = 1;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = d1;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = 0;
      T1++;
      *T1 = 1;
      T1++;
    }

    if (T2 != NULL)
    {
      *T2 = c1 * c2;
      T2++;
      *T2 = -c1 * s2;
      T2++;
      *T2 = s1;
      T2++;
      *T2 = a2 * c1 * c2;
      T2++;
      *T2 = c2 * s1;
      T2++;
      *T2 = -s1 * s2;
      T2++;
      *T2 = -c1;
      T2++;
      *T2 = a2 * c2 * s1;
      T2++;
      *T2 = s2;
      T2++;
      *T2 = c2;
      T2++;
      *T2 = 0;
      T2++;
      *T2 = d1 + a2 * s2;
      T2++;
      *T2 = 0;
      T2++;
      *T2 = 0;
      T2++;
      *T2 = 0;
      T2++;
      *T2 = 1;
      T2++;
    }

    if (T3 != NULL)
    {
      *T3 = c23 * c1;
      T3++;
      *T3 = -s23 * c1;
      T3++;
      *T3 = s1;
      T3++;
      *T3 = c1 * (a3 * c23 + a2 * c2);
      T3++;
      *T3 = c23 * s1;
      T3++;
      *T3 = -s23 * s1;
      T3++;
      *T3 = -c1;
      T3++;
      *T3 = s1 * (a3 * c23 + a2 * c2);
      T3++;
      *T3 = s23;
      T3++;
      *T3 = c23;
      T3++;
      *T3 = 0;
      T3++;
      *T3 = d1 + a3 * s23 + a2 * s2;
      T3++;
      *T3 = 0;
      T3++;
      *T3 = 0;
      T3++;
      *T3 = 0;
      T3++;
      *T3 = 1;
      T3++;
    }

    if (T4 != NULL)
    {
      *T4 = c234 * c1;
      T4++;
      *T4 = s1;
      T4++;
      *T4 = s234 * c1;
      T4++;
      *T4 = c1 * (a3 * c23 + a2 * c2) + d4 * s1;
      T4++;
      *T4 = c234 * s1;
      T4++;
      *T4 = -c1;
      T4++;
      *T4 = s234 * s1;
      T4++;
      *T4 = s1 * (a3 * c23 + a2 * c2) - d4 * c1;
      T4++;
      *T4 = s234;
      T4++;
      *T4 = 0;
      T4++;
      *T4 = -c234;
      T4++;
      *T4 = d1 + a3 * s23 + a2 * s2;
      T4++;
      *T4 = 0;
      T4++;
      *T4 = 0;
      T4++;
      *T4 = 0;
      T4++;
      *T4 = 1;
      T4++;
    }

    if (T5 != NULL)
    {
      *T5 = s1 * s5 + c234 * c1 * c5;
      T5++;
      *T5 = -s234 * c1;
      T5++;
      *T5 = c5 * s1 - c234 * c1 * s5;
      T5++;
      *T5 = c1 * (a3 * c23 + a2 * c2) + d4 * s1 + d5 * s234 * c1;
      T5++;
      *T5 = c234 * c5 * s1 - c1 * s5;
      T5++;
      *T5 = -s234 * s1;
      T5++;
      *T5 = -c1 * c5 - c234 * s1 * s5;
      T5++;
      *T5 = s1 * (a3 * c23 + a2 * c2) - d4 * c1 + d5 * s234 * s1;
      T5++;
      *T5 = s234 * c5;
      T5++;
      *T5 = c234;
      T5++;
      *T5 = -s234 * s5;
      T5++;
      *T5 = d1 + a3 * s23 + a2 * s2 - d5 * c234;
      T5++;
      *T5 = 0;
      T5++;
      *T5 = 0;
      T5++;
      *T5 = 0;
      T5++;
      *T5 = 1;
      T5++;
    }

    if (T6 != NULL)
    {
      *T6 = c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6;
      T6++;
      *T6 = -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6;
      T6++;
      *T6 = c5 * s1 - c234 * c1 * s5;
      T6++;
      *T6 = d6 * (c5 * s1 - c234 * c1 * s5) + c1 * (a3 * c23 + a2 * c2) + d4 * s1 + d5 * s234 * c1;
      T6++;
      *T6 = -c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6;
      T6++;
      *T6 = s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1;
      T6++;
      *T6 = -c1 * c5 - c234 * s1 * s5;
      T6++;
      *T6 = s1 * (a3 * c23 + a2 * c2) - d4 * c1 - d6 * (c1 * c5 + c234 * s1 * s5) + d5 * s234 * s1;
      T6++;
      *T6 = c234 * s6 + s234 * c5 * c6;
      T6++;
      *T6 = c234 * c6 - s234 * c5 * s6;
      T6++;
      *T6 = -s234 * s5;
      T6++;
      *T6 = d1 + a3 * s23 + a2 * s2 - d5 * c234 - d6 * s234 * s5;
      T6++;
      *T6 = 0;
      T6++;
      *T6 = 0;
      T6++;
      *T6 = 0;
      T6++;
      *T6 = 1;
      T6++;
    }
  }

  int ur5e_kinematics::inverse(const double *T, double *q_sols, double q6_des)
  {
    int num_sols = 0;
    double T02 = -*T;
    T++;
    double T00 = *T;
    T++;
    double T01 = *T;
    T++;
    double T03 = -*T;
    T++;
    double T12 = -*T;
    T++;
    double T10 = *T;
    T++;
    double T11 = *T;
    T++;
    double T13 = -*T;
    T++;
    double T22 = *T;
    T++;
    double T20 = -*T;
    T++;
    double T21 = -*T;
    T++;
    double T23 = *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6 * T12 - T13;
      double B = d6 * T02 - T03;
      double R = A * A + B * B;
      if (fabs(A) < ZERO_THRESH)
      {
        double div;
        if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4) * SIGN(B);
        else
          div = -d4 / B;
        double arcsin = asin(div);
        if (fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if (arcsin < 0.0)
          q1[0] = arcsin + 2.0 * PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if (fabs(B) < ZERO_THRESH)
      {
        double div;
        if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
          div = SIGN(d4) * SIGN(A);
        else
          div = d4 / A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0 * PI - arccos;
      }
      else if (d4 * d4 > R)
      {
        return num_sols;
      }
      else
      {
        double arccos = acos(d4 / sqrt(R));
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if (fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if (fabs(neg) < ZERO_THRESH)
          neg = 0.0;
        if (pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0 * PI + pos;
        if (neg >= 0.0)
          q1[1] = neg;
        else
          q1[1] = 2.0 * PI + neg;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for (int i = 0; i < 2; i++)
      {
        double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - d4);
        double div;
        if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0 * PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 2; j++)
        {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if (fabs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else
          {
            q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1),
                       SIGN(s5) * (T00 * s1 - T10 * c1));
            if (fabs(q6) < ZERO_THRESH)
              q6 = 0.0;
            if (q6 < 0.0)
              q6 += 2.0 * PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
          double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
          double p13x = d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - d6 * (T02 * c1 + T12 * s1) +
                        T03 * c1 + T13 * s1;
          double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);

          double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
          if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if (fabs(c3) > 1.0)
          {
            // TODO NO SOLUTION
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0 * PI - arccos;
          double denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
          double s3 = sin(arccos);
          double A = (a2 + a3 * c3), B = a3 * s3;
          q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
          q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
          double c23_0 = cos(q2[0] + q3[0]);
          double s23_0 = sin(q2[0] + q3[0]);
          double c23_1 = cos(q2[1] + q3[1]);
          double s23_1 = sin(q2[1] + q3[1]);
          q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
          q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for (int k = 0; k < 2; k++)
          {
            if (fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if (q2[k] < 0.0)
              q2[k] += 2.0 * PI;
            if (fabs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if (q4[k] < 0.0)
              q4[k] += 2.0 * PI;
            q_sols[num_sols * 6 + 0] = q1[i];
            q_sols[num_sols * 6 + 1] = q2[k];
            q_sols[num_sols * 6 + 2] = q3[k];
            q_sols[num_sols * 6 + 3] = q4[k];
            q_sols[num_sols * 6 + 4] = q5[i][j];
            q_sols[num_sols * 6 + 5] = q6;
            num_sols++;
          }
        }
      }
    }
    return num_sols;
  }

  int ur5e_kinematics::inverse(Eigen::Isometry3d mat, double *q_sols, double q6_des){
    // Transform from 6th link to end effector
    //  0, -1,  0,  0
    //  0,  0, -1,  0
    //  1,  0,  0,  0
    //  0,  0,  0,  1
    Eigen::Isometry3d T6e = Eigen::Isometry3d::Identity();
    T6e(0,0)=0;T6e(0,1)=-1;T6e(0,2)=0;T6e(0,3)=0;
    T6e(1,0)=0;T6e(1,1)=0;T6e(1,2)=-1;T6e(1,3)=0;
    T6e(2,0)=1;T6e(2,1)=0;T6e(2,2)=0;T6e(2,3)=0;
    T6e(3,0)=0;T6e(3,1)=0;T6e(3,2)=0;T6e(3,3)=1;
    mat = mat * T6e;

    double base_pose_transposed[16] = {0};
    int rows = mat.matrix().rows();
    int cols = mat.matrix().cols();

    for (int i = 0; i < rows; i++)
      for (int j = 0; j < cols; j++)
        base_pose_transposed[i * cols + j] = mat.matrix().data()[j * rows + i];

    return inverse(base_pose_transposed, q_sols, q6_des);
  }
  
  int ur5e_kinematics::get_nearest_solution(double *q_sols,int solution_num,double *seed_joints){
    std::vector<double> distances;
    for (int i = 0; i < solution_num; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        //constraints1 : close to seed joint
        if (q_sols[i * 6 + j] - seed_joints[j] > M_PI)
          q_sols[i * 6 + j] -= 2 * M_PI;
        else if (q_sols[i * 6] - seed_joints[j] < -M_PI)
          q_sols[i * 6 + j] += 2 * M_PI;

        //constraints2 : [-2pi,+2pi]
        if (q_sols[i * 6 + j] >= 2 * M_PI)
          q_sols[i * 6 + j] -= 2 * M_PI;
        else if (q_sols[i * 6] <= -2 * M_PI)
          q_sols[i * 6 + j] += 2 * M_PI;
      }
      distances.push_back(cal_joint_distance(&q_sols[i * 6], seed_joints));
    }

    auto min_id = std::min_element(distances.begin(),distances.end()) - distances.begin();
    std::vector<double> solution_joints(&q_sols[min_id * 6], &q_sols[min_id * 6] + 6);
    // printf("Chose the closest solution : %ld, distance = %f,joints:\n[%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f]\n",
    //   min_id, *(distances.begin() + min_id),
    //   180 * solution_joints[0] / M_PI, 180 * solution_joints[1] / M_PI, 180 * solution_joints[2] / M_PI,
    //   180 * solution_joints[3] / M_PI, 180 * solution_joints[4] / M_PI, 180 * solution_joints[5] / M_PI);
    return min_id;
  }

  double ur5e_kinematics::cal_joint_distance(double *J1,double *J2){
    double res=0.0;
    for(int i=0;i<6;i++){
      res += abs(J1[i] - J2[i]);
    }
    return res;
  }


/************************ur5e_kinematics end**************************/
