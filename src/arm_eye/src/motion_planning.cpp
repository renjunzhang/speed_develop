#include <chemicalrobot_arm/motion_planning.h>
#include <manif/SE3.h>
ur5e_action::ur5e_action(
    ros::NodeHandle nh, shared_ptr<dashboardsrv_client> dbptr,
    moveit::planning_interface::MoveGroupInterfacePtr mgptr) {
  string gripper;
  nh.param<string>("/chemicalrobot_arm/file_root", _file_root,
                   getenv("HOME") +
                       string("/chemist_robot/src/chemicalrobot_arm/data/"));
  nh.param<bool>("/chemicalrobot_arm/no_serial", _no_serial, true);
  nh.param<bool>("/chemicalrobot_arm/checkgrasp", _checkgrasp, true);
  nh.param<string>("/chemicalrobot_arm/end_link", _end_link, "tool0");
  nh.param<string>("/chemicalrobot_arm/gripper", gripper, "/dev/PGI");
  this->_mgptr = mgptr;
  this->_dbptr = dbptr;
  if (_no_serial)
    ;
  else {
    this->_pgi_ptr = make_shared<pgi>(gripper, uint32_t(115200));
    _pgi_ptr->initSingle();
    _pgi_ptr->setSpeed(50);
    _pgi_ptr->setForce(20);
    _client2_ = nh.serviceClient<robotiq_ft_sensor::sensor_accessor>(
        "/robotiq_ft_sensor_acc");
    _client2_.waitForExistence();
  }
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  _rmodel = robot_model_loader.getModel();
  _kinematic_state = make_shared<robot_state::RobotState>(_rmodel);
  _jmg = _rmodel->getJointModelGroup("manipulator");
  _sub1_ = nh.subscribe<std_msgs::String>("/obsOperation_in", 1,
                                          &ur5e_action::obs_Callback, this);
  _sub2_ = nh.subscribe<ur_dashboard_msgs::SafetyMode>(
      "/ur_hardware_interface/safety_mode", 1, &ur5e_action::safety_Callback,
      this);
  _sub3_ = nh.subscribe<moveit_msgs::ExecuteTrajectoryActionResult>(
      "/execute_trajectory/result", 1, &ur5e_action::exec_Callback, this);
  _sub4_ = nh.subscribe<std_msgs::String>("grip_operation_in", 1,
                                          &ur5e_action::pgi_Callback, this);
  _pub1_ = nh.advertise<std_msgs::String>("/obsOperation_out", 1);
  _pub2_ = nh.advertise<std_msgs::Int32>("/camera_operation", 1);
  _pub3_ =
      nh.advertise<actionlib_msgs::GoalID>("/execute_trajectory/cancel", 1);
  _pub4_ = nh.advertise<std_msgs::String>("grip_operation_out", 1);
  _client1_ =
      nh.serviceClient<moveit_msgs::GetMotionSequence>("/plan_sequence_path");
  _client1_.waitForExistence();
  _client3_ =
      nh.serviceClient<moveit_msgs::GetMotionPlan>("/plan_kinematic_path");
  _client3_.waitForExistence();

  _service = nh.advertiseService("robot", &ur5e_action::query_Callback, this);

  _mgptr->setEndEffectorLink(_end_link);
  _mgptr->setPlannerId("PTP");
  _mgptr->setMaxVelocityScalingFactor(NORMAL_SPEED);
  _mgptr->setMaxAccelerationScalingFactor(NORMAL_ACC);
  LoadRobotJointInfo();
  // this->Init("remote_control.urp\n");

  _msgptr = std::make_shared<MSG>();
  _msgptr->state = "error";
}

string ur5e_action::GetStamp() {
  long long stamp = ros::Time::now().toNSec() / 1000000;
  return to_string(stamp);
}

bool ur5e_action::query_Callback(arm_eye::DmsService::Request &req,
                                 arm_eye::DmsService::Response &res) {
  std::string msg = (std::string)req.data;

  Json::Value root1;
  root1["id"] = _msgptr->id;
  root1["exper_no"] = _msgptr->exper_no;
  root1["stamp"] = GetStamp();
  root1["state"] = _msgptr->state;

  if ((_msgptr->state == "running") || (_msgptr->state == "error")) {
    root1["detail"] = _msgptr->detail;
  }

  res.data = root1.toStyledString();

  // ROS_INFO_STREAM("--------------query msg reply info---------------\n"
  //                 << root1.toStyledString());

  return true;
}

void ur5e_action::ForceFeedback(string name, double speed, double thre) {
  ros::Duration(1.0).sleep();
  int thre_num = 0;
  geometry_msgs::WrenchStampedConstPtr forceptr;
  // !robotiq_ft_sensor
  robotiq_ft_sensor::sensor_accessor seq;
  seq.request.command_id = seq.request.COMMAND_SET_ZERO;
  _client2_.call(seq);
  ros::Duration(1.0).sleep();
  vector<double> V_force{0, 0, 0};
  geometry_msgs::PoseStamped target_pose;
  target_pose.pose = this->_Inst_absPoses.at(name);
  this->_mgptr->setPlannerId("LIN");
  moveit::planning_interface::MoveGroupInterface::Plan Lineplan;
  _mgptr->setPoseTarget(target_pose);
  _mgptr->setMaxVelocityScalingFactor(speed);
  if (_mgptr->plan(Lineplan) !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR_STREAM("plan line trajectory FAILED");
    throw(myException(vector<string>{"NO", "plan_failed"}));
  }
  this->_mgptr->asyncExecute(Lineplan);
  while (thre_num < 5) {
    forceptr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
        "/robotiq_ft_wrench");
    double force = abs(forceptr->wrench.force.x) +
                   abs(forceptr->wrench.force.y) +
                   abs(forceptr->wrench.force.z);
    if (force > thre) {
      ++thre_num;
      ROS_INFO_STREAM("force_feedback: " << thre_num << "  " << force);
    } else
      thre_num = 0;
    // force > thre ? ++thre_num : thre_num = 0;
  }
  _pub3_.publish(actionlib_msgs::GoalID());
  _mgptr->setMaxVelocityScalingFactor(NORMAL_SPEED);
  _mgptr->setPlannerId("PTP");
}

bool ur5e_action::GotoDefaultPose(string name) {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  _mgptr->setJointValueTarget(GetJoints(name));
  if (_mgptr->plan(plan) !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR_STREAM("Default pose can not be reached!");
    throw(myException(vector<string>{"NO", "plan_failed"}));
    return false;
  }
  _mgptr->execute(plan);
  return true;
}

void ur5e_action::RecordSequence(size_t &i, vector<string> V_name,
                                 vector<double> V_radius,
                                 vector<double> V_speed, vector<double> V_acc,
                                 vector<string> V_planner) {
  // 操作臂实时位姿
  Isometry3d curr_pose =
      this->_mgptr->getCurrentState()->getGlobalLinkTransform(_end_link);
  // 下一个目标路径点
  geometry_msgs::Pose target_pose = this->_waypoints[i];
  // 计算实时位置和目标位置的笛卡尔距离
  double radius = pow(target_pose.position.x - curr_pose.translation()[0], 2) +
                  pow(target_pose.position.y - curr_pose.translation()[1], 2) +
                  pow(target_pose.position.z - curr_pose.translation()[2], 2);
  if (radius < 0.0025) {
    this->_curr_pose = V_name[i];
    ClearSequence();
    // 最后一个Vname和Vradius是当前位置和0,不需要记录
    if (i < V_name.size() - 1 && this->_curr_pose != this->_Vname.back()) {
      this->_Vname.emplace_back(V_name[i]);
      this->_Vradius.emplace_back(V_radius[i]);
      this->_Vspeed.emplace_back(V_speed[i + 1]);
      this->_Vacc.emplace_back(V_acc[i + 1]);
      this->_Vplanner.emplace_back(V_planner[i + 1]);
    }
    ++i;
  }
}

void ur5e_action::RecordCurrSequence(double speed, double acc, string planner) {
  ClearSequence();
  this->_Vname.emplace_back(this->_curr_pose);
  this->_Vradius.emplace_back(0.0);
  this->_Vspeed.emplace_back(speed);
  this->_Vacc.emplace_back(acc);
  this->_Vplanner.emplace_back(planner);
}

bool ur5e_action::ClearSequence() {
  vector<regex> defaultPose{regex("^Jrack[0-3]{1}_0$"),
                            regex("^JH?hole[0-9]*_0$"), regex("^Jdefault_0$"),
                            regex("^J(LIBS)?[\u4e00-\u9fa5]+[站,架]{1}_?0?$"),
                            regex("^P[\u4e00-\u9fa5]+_camera")};
  bool flag = false;
  // 判断_curr_pose与默认位置是否匹配
  for_each(defaultPose.begin(), defaultPose.end(), [this, &flag](regex a) {
    if (regex_match(this->_curr_pose, a)) {
      flag = true;
      return;
    }
  });
  // 如果此时没有夹持物体,且末端处在默认位置，则可以清空记录的路径点
  if (!this->_grasp_flag && flag) {
    this->_Vname.clear();
    this->_Vradius.clear();
    this->_Vspeed.clear();
    this->_Vacc.clear();
    this->_Vplanner.clear();
    return true;
  }
  return false;
}

void ur5e_action::SequenceValidate_pilz(vector<vector<string> *> V_name,
                                        vector<vector<double> *> V_radius,
                                        vector<vector<double> *> V_speed,
                                        vector<vector<double> *> V_acc,
                                        vector<vector<string> *> V_planner) {
  _kinematic_state->setVariablePositions(_joints_names,
                                         _mgptr->getCurrentJointValues());
  int32_t plan_flag = 0;
  // 只有在验证时的第一个Vname需要计算是否插入路径点
  if (InsertPoses(*V_name[0])) {
    V_radius[0]->insert(V_radius[0]->begin(), 0.0);
    V_speed[0]->insert(V_speed[0]->begin(), NORMAL_SPEED);
    V_acc[0]->insert(V_acc[0]->begin(), NORMAL_ACC);
    V_planner[0]->insert(V_planner[0]->begin(), "PTP");
  }
  for (size_t i = 0; i < V_name.size(); ++i) {
    // BUG：在验证时不能更新this->_curr_pose，所以在验证第2,3...条轨迹时会因为没有更新导致错误，先打一个补丁
    int replan_num = 10;
    int replan_count = 0;
    while (plan_flag != 1 && replan_count < replan_num) {
      plan_flag = ValidateOneReq(*V_name[i], *V_radius[i], *V_speed[i],
                                 *V_acc[i], *V_planner[i]);
      if (plan_flag != 1)
      // if (plan_flag == 99999)
      {
        // 将混合半径设为0，再次尝试规划
        ROS_WARN_STREAM(
            "BLENDING FAILED! Try setting blend_radius to 0 and plan again...");
        vector<double>((*V_radius[i]).size(), 0).swap(*V_radius[i]);
      }
      ++replan_count;
    }
    if (plan_flag != 1 && replan_count == replan_num) {
      ROS_ERROR_STREAM("validate sequence_goal failed.ERROR CODE is"
                       << plan_flag);
      throw(myException(vector<string>{"NO", "plan_failed"}));
    }
  }
}

int32_t ur5e_action::ValidateOneReq(vector<string> &V_name,
                                    vector<double> &V_radius,
                                    vector<double> &V_speed,
                                    vector<double> &V_acc,
                                    vector<string> &V_planner) {
  moveit_msgs::GetMotionSequence srvseq;
  srvseq.request.request =
      ConstructSeqReq(V_name, V_radius, V_speed, V_acc, V_planner, true);
  this->_client1_.call(srvseq);
  if (srvseq.response.response.error_code.val == 1) {
    // 获取轨迹的最后关节角作为机器人状态
    _kinematic_state->setVariablePositions(
        this->_joints_names, srvseq.response.response.planned_trajectories[0]
                                 .joint_trajectory.points.back()
                                 .positions);
  }
  return srvseq.response.response.error_code.val;
}

void ur5e_action::SequenceMove_pilz(vector<string> &V_name,
                                    vector<double> &V_radius,
                                    vector<double> &V_speed,
                                    vector<double> &V_acc,
                                    vector<string> &V_planner,
                                    bool /*validate*/, bool record) {
  moveit_msgs::GetMotionSequence srvseq;
  srvseq.request.request =
      ConstructSeqReq(V_name, V_radius, V_speed, V_acc, V_planner, false);
  srvseq.response.response.error_code.val = 0;
  int replan_num = 10;
  int replan_count = 0;
  while (srvseq.response.response.error_code.val != 1 &&
         replan_count < replan_num) {
    this->_client1_.call(srvseq);
    if (srvseq.response.response.error_code.val != 1)
    // if (srvseq.response.response.error_code.val == 99999)
    {
      // 将混合半径设为0，再次尝试规划
      ROS_WARN_STREAM(
          "BLENDING FAILED! Try setting blend_radius to 0 and plan again...");
      vector<double>(V_radius.size(), 0).swap(V_radius);
      srvseq.request.request =
          ConstructSeqReq(V_name, V_radius, V_speed, V_acc, V_planner, false);
    }
    ++replan_count;
  }
  if (srvseq.response.response.error_code.val != 1) {
    ROS_ERROR_STREAM("plan sequence_goal failed.ERROR CODE is"
                     << srvseq.response.response.error_code.val);
    throw(myException(vector<string>{"NO", "plan_failed"}));
  }
  if (record) {
    RecordCurrSequence(V_speed[0], V_acc[0], V_planner[0]);
  }
  size_t pos_flag = 0;
  this->_safety_flag = 1;
  this->_exec_flag = 0;
  int grasp_flag = 0;
  this->_mgptr->asyncExecute(srvseq.response.response.planned_trajectories[0]);
  while (true) {
    if (record && pos_flag < V_name.size())
      RecordSequence(pos_flag, V_name, V_radius, V_speed, V_acc, V_planner);
    if (_no_serial)
      ;
    else {
      // 保护性停止
      if (this->_safety_flag == 3) {
        this->_dbptr->unlockPS();
        this->_dbptr->stop();
        this->_dbptr->play();
        throw(myException(vector<string>{"NO", "exec_failed"}));
      }
      // 物体掉落
      // if (this->_checkgrasp && this->_grasp_flag && _pgi_ptr->isDropCur())
      // {
      //   this->_mgptr->stop();
      //   this->_grasp_flag = false;
      //   std::cout << "检测到物体掉落" << std::endl;
      //   throw(myException(vector<string>{"NO", "grasp_drop"}));
      // }
    }
    // 完成轨迹
    if (this->_exec_flag == 1) {
      break;
    }
  }
  this->_waypoints.clear();
}

Isometry3d ur5e_action::CalculatePoses(string station_name) {
  std_msgs::Int32 op_code;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  op_code.data = _marker_id;
  // 等待第一次重定位的位姿
  while (_pub2_.getNumSubscribers() == 0)
    ;
  _pub2_.publish(op_code);
  geometry_msgs::PoseStampedConstPtr camera_pose =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          "/point_coordinate");
  if (station_name != string("离心孔") &&
      camera_pose->pose.orientation.w == 0) {
    vector<double> towards{-0.05, 0.1};
    for (double each : towards) {
      MoveLine_pilz(FORWARD, each, CLOSE_SPEED);
      while (_pub2_.getNumSubscribers() == 0)
        ;
      _pub2_.publish(op_code);
      camera_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          "/point_coordinate");
    }
  }
  if (camera_pose->pose.orientation.w == 0) {
    throw(myException(vector<string>{"NO", "relocation_error"}));
  }
  geometry_msgs::PoseStamped marker_pose =
      tfBuffer.transform(*camera_pose, "base_link", ros::Duration(1));

  // 如果有标定重定位的位姿，则移动到该位姿
  if (_Inst_relPoses.find(station_name + string("重定位")) !=
      _Inst_relPoses.end()) {
    _mgptr->setMaxVelocityScalingFactor(0.1);
    _mgptr->setMaxAccelerationScalingFactor(0.1);
    // 重定位次数
    size_t re_num = 2;
    Isometry3d mTre_ =
        toMatrix(_Inst_relPoses.at(station_name + string("重定位")));
    for (size_t i = 0; i < re_num; ++i) {
      Isometry3d bTm_ = toMatrix(marker_pose.pose);
      Isometry3d bTre_ = bTm_ * mTre_;
      marker_pose.pose = toPose(bTre_);

      this->_mgptr->setPoseTarget(marker_pose);
      this->_mgptr->move();
      ros::Duration(0.5).sleep();
      while (_pub2_.getNumSubscribers() == 0)
        ;
      _pub2_.publish(op_code);
      // 等待第i+1次重定位的位姿
      camera_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
          "/point_coordinate");
      if (camera_pose->pose.orientation.w == 0) {
        throw(myException(vector<string>{"NO", "relocation_error"}));
      }
      marker_pose =
          tfBuffer.transform(*camera_pose, "base_link", ros::Duration(1));
    }

    // if (station_name == "离心工作站")
    //   this->_CTF_pose = marker_pose.pose;

    // TODO:底盘定位误差修正
    // // 计算底盘定位误差（实际末端位置和理论末端位置的误差）
    // geometry_msgs::PoseStamped curr_pose = this->_mgptr->getCurrentPose();
    // vector<double> err;
    // err.emplace_back(marker_pose.pose.position.x-curr_pose.pose.position.x);
    // err.emplace_back(marker_pose.pose.position.y-curr_pose.pose.position.y);
    // err.emplace_back(marker_pose.pose.position.z-curr_pose.pose.position.z);
    // err.emplace_back(marker_pose.pose.orientation.x-curr_pose.pose.orientation.x);
    // err.emplace_back(marker_pose.pose.orientation.y-curr_pose.pose.orientation.y);
    // err.emplace_back(marker_pose.pose.orientation.z-curr_pose.pose.orientation.z);
    // err.emplace_back(marker_pose.pose.orientation.w-curr_pose.pose.orientation.w);
    // // 底盘定位大于15cm，重定位错误
    // if(fabs(err[0])>0.15 || fabs(err[1])>0.15){
    //     // TODO:直接向底盘发送/cmd_vel or 向cbb发送pose or 向st发送误差
    //     throw(myException(vector<string>{"NO","relocation_error"},err));
    //     // JsonPub(vector<string>{"NO","relocation_error"},err);
    // }
    // this->_curr_pose = "P"+station_name+string("重定位");
    ROS_INFO_STREAM(station_name << ":" << _marker_id << std::endl
                                 << *camera_pose);
    _mgptr->setMaxVelocityScalingFactor(NORMAL_SPEED);
    _mgptr->setMaxAccelerationScalingFactor(NORMAL_ACC);
  }
  // 将marker_pose转为齐次变换矩阵bTm(Base_link到Marker_link的变换)
  Isometry3d bTm = toMatrix(marker_pose.pose);
  if (station_name == "离心孔") {
    Isometry3d mTm1 = bTm.inverse() * toMatrix(this->_CTF_pose);
    geometry_msgs::Pose mpose = toPose(mTm1);
    double dyaw = tf2::getYaw(mpose.orientation) + M_PI;
    int seq = floor(dyaw / (M_PI / 6));
    cout << "当前yaw角偏差:" << dec << (dyaw / M_PI * 180)
         << ",当前离心孔序号: " << seq << endl;
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 2; ++j) {
        Isometry3d mTt =
            toMatrix(_Inst_relPoses.at("离心孔" + to_string(i) + "_" +
                                       to_string(seq) + "_" + to_string(j)));
        Isometry3d bTt = bTm * mTt;
        _Inst_joints["离心孔" + to_string(i) + "_" + to_string(j) +
                     "_inverse"] =
            _Inst_joints.at("离心孔" + to_string(i) + "_" + to_string(seq) +
                            "_" + to_string(j) + "_inverse");
        _Inst_absPoses["离心孔" + to_string(i) + "_" + to_string(j)] =
            toPose(bTt);
      }
    }
  }
  // 除了离心孔定位，其他都仅保留关于Z轴旋转的角度
  else {
    Vector3d bXm = bTm.translation();
    Vector3d bEAm;
    bEAm = bTm.rotation().eulerAngles(2, 1, 0);
    bEAm[0] = tf2::getYaw(marker_pose.pose.orientation);
    bEAm[1] = 0;
    bEAm[2] = 0;
    bTm = toMatrix(bXm, bEAm);
    // 计算绝对位姿
    // 将json中的位姿转为齐次变换矩阵mTt(Marker_link到Tool_link)
    auto match_name = regex(".*" + station_name + ".*");
    for (auto each : _Inst_relPoses) {
      if (regex_match(each.first, match_name)) {
        Isometry3d mTt = toMatrix(each.second);
        // Base_link到Tool_link
        Isometry3d bTt = bTm * mTt;
        _Inst_absPoses[each.first] = toPose(bTt);
      }
      // if (_Inst_absPoses.find(each.first) != _Inst_absPoses.end())
      //   continue;
    }
  }
  marker_pose.pose = toPose(bTm);
  ROS_INFO_STREAM(station_name
                << ":" << op_code.data << std::endl
                << marker_pose);
  return bTm;
}

void ur5e_action::obs_Callback(
    const std_msgs::StringConstPtr &obsOperation_in) {

  Json::Reader jsonreader;
  Json::Value root;
  jsonreader.parse(obsOperation_in->data, root);
  cout << "----------cmd recv-------------\n" << root.toStyledString() << endl;

  if (!root["id"].isNull()) {
    _msgptr->id = root["id"].asString();
  }
  if (!root["exper_no"].isNull()) {
    _msgptr->exper_no = root["exper_no"].asString();
  }
  _msgptr->state = "running";
  _msgptr->detail = "";

  try {
    if (!root["operation"].isNull()) {

      string cmd_station = root["destination"].asString();
      string operation = root["operation"].asString();
      string station = station_list[cmd_station];
      if (station.empty()) {
        ROS_ERROR_STREAM("station not found");
        JsonPubErr("station not found");
        return;
      }
      this->_curr_station = station;

      // 复位
      if (operation == "reset") {
        if ((this->_curr_station == "map_center") ||
            (this->_curr_station == "charge_station")) {
          cout << "reset:" << this->_curr_station << endl;
          GoDefault();
        } else {
          cout << "reset:" << this->_curr_station << endl;
          GoDefault(this->_curr_station);
        }
        JsonPubDone();
      }
      // 重定位
      else if (operation == "relocation") {
        if ((this->_curr_station == "map_center") ||
            (this->_curr_station == "charge_station")) {
          cout << "relocation not necessary" << endl;
          JsonPubErr("not necessary");
          return;
        }

        if (this->_curr_station == "电催化工作站") {
          Relocation();
          RelocationElectrocatalysis();
        } else if (this->_curr_station == "LIBS工作站") {
          Relocation();
          RelocationLibs();
        } else {
          cout << "Relocation:" << this->_curr_station << endl;
          Relocation();
        }
        JsonPubDone();
      } else {
        ROS_ERROR_STREAM("unknown cmd");
        JsonPubErr("unknown cmd");
      }
      return;
    }

    string station = station_list[root["destination"].asString()];
    if (station.empty()) {
      ROS_ERROR_STREAM("station not found");
      JsonPubErr("station not found");
      return;
    }

    Json::Value obj = root["operations"];

    for (int i = 0; i < 1; i++) {
      string operation = obj[i]["operation"].asString();
      string station_rack_idx = obj[i]["rack_station"].asString();
      string station_slot_idx = obj[i]["slot_station"].asString();

      string station_btn_idx = obj[i]["button_index"].asString();
      string robot_rack_idx = obj[i]["rack_robot"].asString();
      string robot_slot_idx = obj[i]["slot_robot"].asString();
      string phase = obj[i]["phase"].asString();
      COMMAND cmd(operation, station_rack_idx, station_slot_idx,
                  station_btn_idx, robot_rack_idx, robot_slot_idx, phase);

      // 当前操作
      _msgptr->detail = operation;

      RobotAction(cmd);

      sleep(1);
    }
    // 执行结束
    JsonPubDone();
  }

  catch (myException &e) {

    if (e.getException() != string("")) {
      ROS_ERROR_STREAM("--------------feedback jsoninfo---------------"
                       << endl
                       << e.getException());
      JsonPubErr(e.getException());
    } else {
      ROS_INFO_STREAM("unknown error");
      JsonPubErr("unknown error");
    }
  }
}

bool ur5e_action::RobotAction(COMMAND cmd) {
  cout << "\n######RobotAction########" << endl;
  string operation = cmd.operation;
  string station_rack_idx = cmd.station_rack_idx;
  string station_slot_idx = cmd.station_slot_idx;
  string station_btn_idx = cmd.station_btn_idx;
  string robot_rack_idx = cmd.robot_rack_idx;
  string robot_slot_idx = cmd.robot_slot_idx;
  string phase = cmd.phase;

  cout << "operation =" << operation << endl;
  cout << "station_rack_idx =" << station_rack_idx << endl;
  cout << "station_slot_idx =" << station_slot_idx << endl;
  cout << "station_btn_idx =" << station_btn_idx << endl;
  cout << "robot_rack_idx =" << robot_rack_idx << endl;
  cout << "robot_slot_idx =" << robot_slot_idx << endl;
  // cout << "phase =" << phase << endl;
  std::string qxsp = "⽓相⾊谱⼯作站";
  cout << "_curr_station: " << _curr_station << std::endl;
  // take
  if (_curr_station == "起始样品架") {
    cout << "#起始样品架#" << endl;

    // station
    string Pname =
        "P" + _curr_station + station_rack_idx; // xxx0-> xxx0_0   xxx0_1

    // robot
    string Jname1 = "Jrack" + robot_rack_idx; // rack0 -> rack0_0  rack0_1
    string Jname2 =
        "Jhole" + robot_rack_idx + "0"; // rack00-> rack00_0 rack00_1

    if (operation == "take") {
      cout << "action = take" << endl;
      vector<string> V_name1{Pname + "_0", Pname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname + "_0", "J" + _curr_station, Jname1 + "_0",
                             Jname1 + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname1 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      // vector<string> V_name3{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      // vector<double> V_radius3{0.2, 0.2, 0.0};
      // vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      // vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      // vector<string> V_planner3{"LIN", "PTP", "LIN"};

      // vector<string> V_name4{Jname2 + "_0"};
      // vector<double> V_radius4{0.0};
      // vector<double> V_speed4{NORMAL_SPEED};
      // vector<double> V_acc4{NORMAL_ACC};
      // vector<string> V_planner4{"LIN"};

      // SequenceValidate_pilz(
      //     vector<vector<string> *>{&V_name1, &V_name2, &V_name3, &V_name4},
      //     vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3,
      //                              &V_radius4},
      //     vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3, &V_speed4},
      //     vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3, &V_acc4},
      //     vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3,
      //                              &V_planner4});

      SequenceValidate_pilz(
        vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
        vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
        vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
        vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
        vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往起始站点
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(470, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往对应试管
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
      // // 抓取试管(这一步是为了让试管架在机器人平台上的位置准确)
      // Gripper_op(850, true, true);
      // Gripper_op(0, false, true);
      // SequenceMove_pilz(V_name4, V_radius4, V_speed4, V_acc4, V_planner4, false,
      //                   true);

    } else if (operation == "put") {
      cout << "put" << endl;
      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname1 + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.0, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname + "_0", "J" + _curr_station};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往回收样品站
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; // from
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     // to

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, false);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "红外光谱工作站") {

    string Jname = "Jplate";
    string Pname = "P" + _curr_station;
    if (operation == "put_ir") {
      if (1) // open
      {
        vector<string> V_name1{Pname + "_0", Pname + "_1"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        std::string auxiliary(Pname + "_4");
        std::string goal(Pname + "_7");

        vector<string> V_name3{Pname + "_8", Pname + "_9", "J红外光谱工作站"};
        vector<double> V_radius3{0.2, 0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP", "PTP"};

        Gripper_op(0, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取把手
        Gripper_op(900, false, true);
        // 开盖（圆弧）
        // SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
        // false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        // 松开把手
        Gripper_op(800, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
      if (1) // put
      {
        vector<string> V_name1{Pname + "_s", Jname + "_2", Jname + "_3",
                               Jname + "_4"};
        vector<double> V_radius1{0.2, 0.2, 0.0, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED,
                                CLOSE_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC, CLOSE_ACC};
        vector<string> V_planner1{"PTP", "PTP", "LIN", "LIN"};

        vector<string> V_name2{Jname + "_3", Jname + "_2", Pname + "_s",
                               Pname + "_10", Pname + "_11"};
        vector<double> V_radius2{0.0, 0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed2{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc2{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                              CLOSE_ACC};
        vector<string> V_planner2{"LIN", "LIN", "PTP", "PTP", "LIN"};

        vector<string> V_name3{Pname + "_10", Pname + "_9", "J红外光谱工作站"};
        vector<double> V_radius3{0.2, 0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(0, false, true);
        // 前往机器人平台上的暂存架
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取托盘
        Gripper_op(900, false, true);
        // 前往红外
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下托盘
        Gripper_op(0, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
      if (1) // close
      {
        vector<string> V_name1{Pname + "_8_1", Pname + "_8", Pname + "_7"};
        vector<double> V_radius1{0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner1{"PTP", "PTP", "LIN"};

        std::string auxiliary(Pname + "_4");
        std::string goal(Pname + "_1");

        vector<string> V_name3{Pname + "_0", "J红外光谱工作站"};
        vector<double> V_radius3{0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP"};

        Gripper_op(800, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取把手
        Gripper_op(900, false, true);
        // 关盖（圆弧）
        // SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
        // false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        // 松开把手
        Gripper_op(0, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
    } else if (operation == "take_ir") {
      if (1) // open
      {
        vector<string> V_name1{Pname + "_0", Pname + "_1"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        std::string auxiliary(Pname + "_4");
        std::string goal(Pname + "_7");

        vector<string> V_name3{Pname + "_8", Pname + "_9", Pname + "_8_1",
                               "J红外光谱工作站"};
        vector<double> V_radius3{0.2, 0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP", "PTP", "PTP"};

        Gripper_op(0, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取把手
        Gripper_op(900, false, true);
        // 开盖（圆弧）
        // SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
        // false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        // 松开把手
        Gripper_op(800, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
      if (1) // take
      {
        vector<string> V_name1{Pname + "_9", Pname + "_10", Pname + "_11"};
        vector<double> V_radius1{0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner1{"PTP", "PTP", "LIN"};

        vector<string> V_name2{Pname + "_10", Pname + "_s", Jname + "_2",
                               Jname + "_3", Jname + "_4"};
        vector<double> V_radius2{0.2, 0.2, 0.2, 0.0, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                CLOSE_SPEED, CLOSE_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC,
                              CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN", "LIN"};

        vector<string> V_name3{Jname + "_2", Pname + "_s", "J红外光谱工作站"};
        vector<double> V_radius3{0.2, 0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(0, false, true);
        // 前往机器人平台上的暂存架
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取托盘
        Gripper_op(900, true, true);
        // 前往红外
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下托盘
        Gripper_op(0, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
      if (1) // close
      {
        vector<string> V_name1{Pname + "_8_1", Pname + "_8", Pname + "_7"};
        vector<double> V_radius1{0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner1{"PTP", "PTP", "LIN"};

        std::string auxiliary(Pname + "_4");
        std::string goal(Pname + "_1");

        vector<string> V_name3{Pname + "_0", "J红外光谱工作站"};
        vector<double> V_radius3{0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP"};

        Gripper_op(800, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取把手
        Gripper_op(900, false, true);
        // 关盖（圆弧）
        // SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
        // false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        // 松开把手
        Gripper_op(0, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
    }
  } else if (_curr_station == "XRD工作站") {
    string Pname1 = "P" + _curr_station;
    string Pname2 = Pname1 + to_string(0);
    string Jname = "Jplate";
    if (operation == "opendoor") {
    } else if (operation == "closedoor") {
    } else if (operation == "put_xrd") {
      // 打开门
      if (1) {
        // vector<string> V_name1{
        //     "J" + _curr_station, Pname1 + "_0_", Pname1 + "_3", Pname1 +
        //     "_4", Pname1 + "_0_", Pname1 + "_0", Pname1 + "_1"};
        // vector<double> V_radius1{0.2, 0.2, 0.2, 0.0, 0.2, 0.2, 0.0};
        // vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
        //                         NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
        //                         NORMAL_SPEED};
        // vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
        //                       NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        // vector<string> V_planner1{"PTP", "PTP", "PTP", "LIN",
        //                           "PTP", "PTP", "PTP"};

        vector<string> V_name1{"J" + _curr_station, Pname1 + "_0_1",
                               Pname1 + "_0_", Pname1 + "_3", Pname1 + "_4"};
        vector<double> V_radius1{0.2, 0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                              NORMAL_ACC};
        vector<string> V_planner1{"PTP", "PTP", "PTP", "PTP", "LIN"};

        vector<string> V_name11{Pname1 + "_0_", Pname1 + "_0", Pname1 + "_1"};
        vector<double> V_radius11{0.2, 0.2, 0.0};
        vector<double> V_speed11{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc11{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner11{"PTP", "PTP", "PTP"};

        std::string auxiliary(Pname1 + "_circ_" + to_string(10));
        std::string goal(Pname1 + "_circ_19");

        vector<string> V_name2{Pname1 + "_2", Pname1 + "_0"};
        vector<double> V_radius2{0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner2{"LIN", "PTP"};

        // TODO:无论是记录路径点还是验证轨迹是否可行,都没有考虑过CIRC规划
        // TODO:还要考虑之后做成外部来确定轨迹规划顺序,方法等
        // TODO:validate,resetarm,move重构成纯vector,还要添加额外信息如center,interim

        Gripper_op(0, false, true);
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        SequenceMove_pilz(V_name11, V_radius11, V_speed11, V_acc11, V_planner11,
                          false, true);
        Gripper_op(1000, false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        Gripper_op(600, false, true);
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
      }

      // 放入xrd
      if (1) {
        vector<string> V_name1{Pname1 + "_5_s", Pname1 + "_5", Jname + "_0_s",
                               Jname + "_1_s"};
        vector<double> V_radius1{0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                CLOSE_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner1{"PTP", "PTP", "PTP", "LIN"};

        vector<string> V_name2{Jname + "_0_s", Pname1 + "_5", Pname1 + "_6",
                               Pname1 + "_7"};
        vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed2{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                CLOSE_SPEED};
        vector<double> V_acc2{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

        vector<string> V_name3{Pname1 + "_6", Pname1 + "_5", Pname1 + "_5_s",
                               Pname1 + "_0"};
        vector<double> V_radius3{0.2, 0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed3{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                NORMAL_SPEED};
        vector<double> V_acc3{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP", "PTP", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(0, false, false);
        // 前往XRD样品
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取XRD样品
        Gripper_op(900, true, true);
        // 前往XRD
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下XRD样品
        Gripper_op(800, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }

      // 关上门
      if (1) {
        vector<string> V_name1{Pname1 + "_0", Pname1 + "_2",
                               Pname1 + "_circ_19"};
        vector<double> V_radius1{0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "PTP", "LIN"};

        std::string auxiliary(Pname1 + "_circ_" + to_string(10));
        std::string goal(Pname1 + "_1");

        vector<string> V_name2{Pname1 + "_x0", Pname1 + "_0", "Jdefault_0"};
        vector<double> V_radius2{0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner2{"PTP", "PTP", "PTP"};

        // TODO:无论是记录路径点还是验证轨迹是否可行,都没有考虑过CIRC规划
        // TODO:还要考虑之后做成外部来确定轨迹规划顺序,方法等
        // TODO:validate,resetarm,move重构成纯vector,还要添加额外信息如center,interim

        Gripper_op(600, false, true);
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        Gripper_op(1000, false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        Gripper_op(0, false, true);
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        this->_curr_pose = "Jdefault_0";
      }
    } else if (operation == "take_xrd") {
      // 打开门
      if (1) {
        vector<string> V_name1{"J" + _curr_station, Pname1 + "_0_1",
                               Pname1 + "_0_",      Pname1 + "_3",
                               Pname1 + "_4",       Pname1 + "_0_",
                               Pname1 + "_0",       Pname1 + "_1"};
        vector<double> V_radius1{0.2, 0.2, 0.2, 0.2, 0.0, 0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                              NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "PTP", "PTP", "PTP",
                                  "LIN", "PTP", "PTP", "PTP"};

        std::string auxiliary(Pname1 + "_circ_" + to_string(10));
        std::string goal(Pname1 + "_circ_19");

        vector<string> V_name2{Pname1 + "_2", Pname1 + "_0"};
        vector<double> V_radius2{0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner2{"LIN", "PTP"};

        // TODO:无论是记录路径点还是验证轨迹是否可行,都没有考虑过CIRC规划
        // TODO:还要考虑之后做成外部来确定轨迹规划顺序,方法等
        // TODO:validate,resetarm,move重构成纯vector,还要添加额外信息如center,interim

        Gripper_op(0, false, true);
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        Gripper_op(1000, false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        Gripper_op(600, false, true);
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
      }

      // 取出xrd
      if (1) {
        vector<string> V_name1{Pname1 + "_5_s", Pname1 + "_5", Pname1 + "_6",
                               Pname1 + "_7"};
        vector<double> V_radius1{0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                CLOSE_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner1{"PTP", "PTP", "PTP", "LIN"};

        vector<string> V_name2{Pname1 + "_6", Pname1 + "_5", Jname + "_0_s",
                               Jname + "_1_s"};
        vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed2{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                CLOSE_SPEED};
        vector<double> V_acc2{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

        vector<string> V_name3{Jname + "_0_s", Pname1 + "_5", Pname1 + "_5_s",
                               Pname1 + "_0"};
        vector<double> V_radius3{0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed3{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                NORMAL_SPEED};
        vector<double> V_acc3{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP", "PTP", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(800, false, false);
        // 前往XRD样品
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取XRD样品
        Gripper_op(900, true, true);
        // 前往XRD
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下XRD样品
        Gripper_op(0, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }

      // 关上门
      if (1) {
        vector<string> V_name1{Pname1 + "_0", Pname1 + "_2",
                               Pname1 + "_circ_19"};
        vector<double> V_radius1{0.2, 0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "PTP", "LIN"};

        std::string auxiliary(Pname1 + "_circ_" + to_string(10));
        std::string goal(Pname1 + "_1");

        vector<string> V_name2{Pname1 + "_x0", Pname1 + "_0", "Jdefault_0"};
        vector<double> V_radius2{0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner2{"PTP", "PTP", "PTP"};

        // TODO:无论是记录路径点还是验证轨迹是否可行,都没有考虑过CIRC规划
        // TODO:还要考虑之后做成外部来确定轨迹规划顺序,方法等
        // TODO:validate,resetarm,move重构成纯vector,还要添加额外信息如center,interim

        Gripper_op(600, false, true);
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        Gripper_op(1000, false, true);
        MoveCircle_pilz(INTERIM, auxiliary, goal);
        Gripper_op(0, false, true);
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        this->_curr_pose = "Jdefault_0";
      }
    }
  } else if (_curr_station == "LIBS工作站") {
    string Pname1 = "P" + _curr_station;
    string Pname2 = "PLIBS样品架0";
    // string Jname = "Jrack" + to_string(rack_relnum);
    if (operation == "put_libs") {
      vector<string> V_name1{Pname2 + "_0", Pname2 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname2 + "_0", "JLIBS样品架", Pname1 + "_0",
                             Pname1 + "_1", Pname1 + "_2"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname1 + "_1", Pname1 + "_0"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, false);
      // 前往LIBS样品架
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取LIBS样品
      Gripper_op(270, true, true);
      // 前往LIBS工作站
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下LIBS样品
      Gripper_op(0, false, true);
      // 前往LIBS关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take_libs") {
      vector<string> V_name1{Pname1 + "_0", Pname1 + "_1", Pname1 + "_2"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Pname1 + "_1", Pname1 + "_0", "JLIBS样品架",
                             Pname2 + "_0", Pname2 + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname2 + "_0", "JLIBS样品架"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, false);
      // 前往LIBS工作站
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取LIBS样品
      Gripper_op(270, true, true);
      // 前往LIBS样品架
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下LIBS样品
      Gripper_op(0, false, true);
      // 前往LIBS关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "吸液工作站") {
    string Jname = "Jrack" + robot_rack_idx;
    string Pname = "P" + _curr_station;
    if (operation == "put") {
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1", Pname + "_2"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname + "_1", Pname + "_camera"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往吸液
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往吸液观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      vector<string> V_name1{Pname + "_0", Pname + "_1", Pname + "_2"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Pname + "_1", "J" + _curr_station, Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往吸液
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; // from
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     // to

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "液体进样站贰") {
    cout << "液体进样站贰" << endl;

    string Pname = "P" + _curr_station; // xxx => xxx_0  xxx_1  xx_2
    string Jname = "Jhole" + robot_rack_idx +
                   robot_slot_idx; // hole00 => hole00_0  hole00_1

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "action = put " << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1", Pname + "_2"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN", "LIN"};

      vector<string> V_name3{Pname + "_1", Pname + "_0", Pname + "_camera"};
      vector<double> V_radius3{0.0, 0.2, 0.0};
      vector<double> V_speed3{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(400, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往液体进样站
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(400, false, true);
      // 前往液体进样站观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      cout << "action = take " << endl;
      vector<string> V_name1{Pname + "_0", Pname + "_1", Pname + "_2"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "LIN", "LIN"};

      vector<string> V_name2{Pname + "_1", Pname + "_0", "J" + _curr_station,
                             Jname + "_0", Jname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0, 0.0};
      vector<double> V_speed2{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(400, false, true);
      // 前往液体进样站
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(400, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; // from
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     // to

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "烘干工作站") {
    cout << "烘⼲⼯作站" << endl;
    string Jname = "Jrack" + robot_rack_idx;
    string Pname = "P" + _curr_station + station_rack_idx;
    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_1",
                             Pname + "_0"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{
          Pname + "_1",
          "J" + _curr_station,
      };
      vector<double> V_radius3{
          0.0,
          0.2,
      };
      vector<double> V_speed3{
          NORMAL_SPEED,
          NORMAL_SPEED,
      };
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往烘干工作站
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往烘干观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      cout << "take" << endl;
      vector<string> V_name1{"J" + _curr_station, Pname + "_1", Pname + "_0"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Pname + "_1", "J" + _curr_station, Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius2{0.0, 0.0, 0.2, 0.0};
      vector<double> V_speed2{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往烘干工作站
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; // from
      string Jname1_h = "Jhole" + station_rack_idx + "_h";
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx; // to
      string Jname2_h = "Jhole" + robot_rack_idx + "_h";

      vector<string> V_name1{Jname1_h, Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Jname1 + "_0", Jname1_h, Jname2_h, Jname2 + "_0",
                             Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0", Jname2_h};
      vector<double> V_radius3{0.0, 0.2};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "光催化工作站") {
    cout << "#光催化工作站#" << endl;
    string Jname = "Jhole" + robot_rack_idx + robot_slot_idx;
    string Pname = "P" + _curr_station + station_slot_idx;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname + "_0", "J" + _curr_station};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(600, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往光催化
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往光催化观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      cout << "take" << endl;
      vector<string> V_name1{Pname + "_0", Pname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname + "_0", "J" + _curr_station, Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.0, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(600, false, true);
      // 前往光催化
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; // from
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     // to

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "磁力搅拌站") {
    cout << "磁⼒搅拌站" << endl;
    string Jname = "Jrack" + robot_rack_idx;
    string Pname = "P" + _curr_station;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "action = put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1", Pname + "_2"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname + "_1", "J" + _curr_station};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往磁力搅拌站
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往磁力搅拌站观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
      // JsonPub();
    } else if (operation == "take") {
      cout << "action = take" << endl;
      vector<string> V_name1{Pname + "_0", Pname + "_1", Pname + "_2"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Pname + "_1", Pname + "_0", "J" + _curr_station,
                             Jname + "_0", Jname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往磁力搅拌
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; // from
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     // to

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "封装工作站") {
    cout << "封装⼯作站" << endl;
    string Pname = "P" + _curr_station;
    string Jname = "Jhole" + robot_rack_idx + robot_slot_idx;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_1",
                             Pname + "_2", Pname + "_3"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN", "LIN"};

      vector<string> V_name3{Pname + "_2", Pname + "_camera"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      //
      Gripper_op(600, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管
      Gripper_op(850, true, true);

      // 前往封装工作站
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);

      // 前往封装关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      cout << "take" << endl;
      vector<string> V_name1{Pname + "_1", Pname + "_2", Pname + "_3"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Pname + "_2", Pname + "_1", "J" + _curr_station,
                             Jname + "_0", Jname + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(600, false, true);
      // 前往封装
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);

      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往复位关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed2, V_acc2, V_planner2, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "固体进样站") {
    cout << "固体进样站" << endl;
    string Jname = "Jhole" + robot_rack_idx + robot_slot_idx;
    string Pname = "P" + _curr_station;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "action = put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "Jhole_0", "Jhole_1"};
      vector<double> V_radius2{0.2, 0.0, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{"Jhole_0", "JHhole_0", "JHhole_1"};
      vector<double> V_radius3{0.0, 0.0, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner3{"LIN", "PTP", "LIN"};

      vector<string> V_name4{"JHhole_0", "J固体进样站_", Pname + "_0",
                             Pname + "_1", Pname + "_2"};
      vector<double> V_radius4{0.2, 0.0, 0.2, 0.2, 0.0};
      vector<double> V_speed4{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED, CLOSE_SPEED};
      vector<double> V_acc4{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC,
                            CLOSE_ACC};
      vector<string> V_planner4{"LIN", "PTP", "PTP", "LIN", "LIN"};

      vector<string> V_name5{Pname + "_0", "J固体进样站_", "Jrack0_0"};
      vector<double> V_radius5{0.2, 0.0, 0.0};
      vector<double> V_speed5{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc5{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner5{"LIN", "PTP", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3, &V_name4,
                                   &V_name5},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3,
                                   &V_radius4, &V_radius5},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3, &V_speed4,
                                   &V_speed5},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3, &V_acc4, &V_acc5},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3,
                                   &V_planner4, &V_planner5});

      Gripper_op(600, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往试管中转
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往试管中转（横抓）
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
      // 夹取试管
      Gripper_op(1000, true, true);
      // 前往固体进样站
      SequenceMove_pilz(V_name4, V_radius4, V_speed4, V_acc4, V_planner4, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name5, V_radius5, V_speed5, V_acc5, V_planner5, false,
                        true);
    } else if (operation == "take") {
      cout << "action = take" << endl;
      vector<string> V_name1{"J固体进样站_", Pname + "_0", Pname + "_1",
                             Pname + "_2"};
      vector<double> V_radius1{0.0, 0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN", "LIN"};

      vector<string> V_name2{Pname + "_1", Pname + "_0", "J固体进样站_",
                             "JHhole_0", "JHhole_1"};
      vector<double> V_radius2{0.2, 0.2, 0.0, 0.0, 0.0};
      vector<double> V_speed2{CLOSE_SPEED, CLOSE_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{CLOSE_ACC, CLOSE_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner2{"LIN", "LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{"JHhole_0", "Jhole_0", "Jhole_1"};
      vector<double> V_radius3{0.0, 0.0, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner3{"PTP", "PTP", "LIN"};

      vector<string> V_name4{"Jhole_0", Jname + "_0", Jname + "_1"};
      vector<double> V_radius4{0.2, 0.0, 0.0};
      vector<double> V_speed4{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc4{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner4{"LIN", "PTP", "LIN"};

      vector<string> V_name5{Jname + "_0"};
      vector<double> V_radius5{0.0};
      vector<double> V_speed5{NORMAL_SPEED};
      vector<double> V_acc5{NORMAL_ACC};
      vector<string> V_planner5{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3, &V_name4,
                                   &V_name5},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3,
                                   &V_radius4, &V_radius5},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3, &V_speed4,
                                   &V_speed5},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3, &V_acc4, &V_acc5},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3,
                                   &V_planner4, &V_planner5});

      Gripper_op(600, false, true);
      // 前往固体进样
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(1000, true, true);
      // 前往试管中转(横向)
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往试管中转
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往试管
      SequenceMove_pilz(V_name4, V_radius4, V_speed4, V_acc4, V_planner4, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name5, V_radius5, V_speed5, V_acc5, V_planner5, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "紫外光谱工作站") {
    cout << "紫外光谱⼯作站" << endl;
    string Jname = "Jrack" + robot_rack_idx;
    string Pname = "P" + _curr_station;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J紫外光谱工作站", Pname + "_0",
                             Pname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname + "_0", Pname + "_camera"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往紫外光谱
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往紫外光谱观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      cout << "take" << endl;
      vector<string> V_name1{Pname + "_0", Pname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname + "_0", "J紫外光谱工作站", Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往紫外光谱
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, false);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "荧光光谱工作站") {
    cout << "荧光光谱⼯作站" << endl;
    string Jname = "Jrack" + robot_rack_idx;
    string Pname = "P" + _curr_station;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname + "_0", Pname + "_camera"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往荧光光谱
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往荧光光谱观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      cout << "take" << endl;
      vector<string> V_name1{Pname + "_0", Pname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname + "_0", "J" + _curr_station, Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往荧光光谱
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "拉曼光谱工作站") {
    string Jname = "Jhole" + robot_rack_idx + robot_slot_idx;
    string Pname = "P" + _curr_station;
    if (operation == "put") {
      // open
      if (1) {
        vector<string> V_name1{Pname + "_0", Pname + "_1"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        vector<string> V_name2{Pname + "_0", Pname + "_2", Pname + "_3"};
        vector<double> V_radius2{0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "LIN"};

        vector<string> V_name3{Pname + "_2", "J" + _curr_station};
        vector<double> V_radius3{0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(0, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 抓取盖子
        Gripper_op(900, true, true);
        // 移动盖子
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下盖子
        Gripper_op(600, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
      // put
      if (1) {
        vector<string> V_name1{Jname + "_0", Jname + "_1"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_4",
                               Pname + "_5"};
        vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                CLOSE_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

        // vector<string> V_name3{Pname + "_4", "J" + _curr_station};
        // vector<double> V_radius3{0.2, 0.0};
        // vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        // vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        // vector<string> V_planner3{"LIN", "PTP"};
        vector<string> V_name3{Pname + "_4"};
        vector<double> V_radius3{0.2};
        vector<double> V_speed3{NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC};
        vector<string> V_planner3{"LIN"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(0, false, true);
        // 前往瓶子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 抓取瓶子
        Gripper_op(900, true, true);
        // 移动瓶子至工作站
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下瓶子
        Gripper_op(600, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }

      // close
      if (1) {
        vector<string> V_name1{Pname + "_2", Pname + "_3"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        vector<string> V_name2{Pname + "_2", Pname + "_0", Pname + "_1"};
        vector<double> V_radius2{0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "LIN"};

        vector<string> V_name3{Pname + "_0", "J" + _curr_station};
        vector<double> V_radius3{0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(600, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 抓取盖子
        Gripper_op(900, true, true);
        // 移动盖子
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下盖子
        Gripper_op(600, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
    } else if (operation == "take") {
      // open
      if (1) {
        vector<string> V_name1{Pname + "_0", Pname + "_1"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        vector<string> V_name2{Pname + "_0", Pname + "_2", Pname + "_3"};
        vector<double> V_radius2{0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "LIN"};

        // vector<string> V_name3{Pname + "_2", "J" + _curr_station};
        // vector<double> V_radius3{0.2, 0.0};
        // vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        // vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        // vector<string> V_planner3{"LIN", "PTP"};
        vector<string> V_name3{Pname + "_2"};
        vector<double> V_radius3{0.2};
        vector<double> V_speed3{NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC};
        vector<string> V_planner3{"LIN"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(0, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 抓取盖子
        Gripper_op(900, true, true);
        // 移动盖子
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下盖子
        Gripper_op(600, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }

      // take
      if (1) {
        vector<string> V_name1{Pname + "_4", Pname + "_5"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        vector<string> V_name2{Pname + "_4", "J" + _curr_station, Jname + "_0",
                               Jname + "_1"};
        vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                                CLOSE_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

        vector<string> V_name3{Jname + "_0", "J" + _curr_station};
        vector<double> V_radius3{0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(600, false, true);
        // 前往拉曼
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 夹取试管
        Gripper_op(900, true, true);
        // 前往机器人平台
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下试管
        Gripper_op(600, false, true);
        // 前往平台关节
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }

      // close
      if (1) {
        vector<string> V_name1{Pname + "_2", Pname + "_3"};
        vector<double> V_radius1{0.2, 0.0};
        vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner1{"PTP", "LIN"};

        vector<string> V_name2{Pname + "_2", Pname + "_0", Pname + "_1"};
        vector<double> V_radius2{0.2, 0.2, 0.0};
        vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
        vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
        vector<string> V_planner2{"LIN", "PTP", "LIN"};

        vector<string> V_name3{Pname + "_0", "J" + _curr_station};
        vector<double> V_radius3{0.2, 0.0};
        vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner3{"LIN", "PTP"};

        SequenceValidate_pilz(
            vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
            vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
            vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
            vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
            vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

        Gripper_op(600, false, true);
        // 前往盖子
        SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                          false, true);
        // 抓取盖子
        Gripper_op(900, true, true);
        // 移动盖子
        SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2,
                          false, true);
        // 放下盖子
        Gripper_op(600, false, true);
        // 复位
        SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3,
                          false, true);
      }
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "气相色谱工作站") {
    cout << "⽓相⾊谱⼯作站" << endl;
    string Jname = "Jhole" + robot_rack_idx + robot_slot_idx;
    string Pname = "P" + _curr_station + station_slot_idx;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "put" << endl;
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.0, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname + "_0", "J" + _curr_station};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(600, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往气相色谱
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往气相色谱观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      cout << "take" << endl;
      vector<string> V_name1{Pname + "_0", Pname + "_1"};
      vector<double> V_radius1{0.0, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname + "_0", "J" + _curr_station, Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius2{0.2, 0.0, 0.0, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(600, false, true);
      // 前往气相色谱
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "马弗炉工作站") {
    cout << "马弗炉工作站" << endl;
    string Jname = "Jhole" + robot_rack_idx + robot_slot_idx;
    string Pname = "P" + _curr_station;

    cout << "operation=" << operation << endl;
    cout << "Pname=" << Pname << endl;
    cout << "Jname=" << Jname << endl;

    if (operation == "put") {
      cout << "action = put" << endl;
      vector<string> V_name1{"J马弗炉工作站", Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "Jhole_safe", "Jhole_0", "Jhole_1"};
      vector<double> V_radius2{0.2, 0.2, 0.0, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{"Jhole_0", "JHhole_safe", "JHhole_safe1",
                             "JHhole_0", "JHhole_1"};
      vector<double> V_radius3{0.0, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner3{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name4{"JHhole_0", Pname + "_0", Pname + "_1",
                             Pname + "_2"};
      vector<double> V_radius4{0.0, 0.2, 0.0, 0.0};
      vector<double> V_speed4{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc4{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner4{"LIN", "PTP", "LIN", "LIN"};

      vector<string> V_name5{Pname + "_1", Pname + "_0"};
      vector<double> V_radius5{0.0, 0.0};
      vector<double> V_speed5{CLOSE_SPEED, NORMAL_SPEED};
      vector<double> V_acc5{CLOSE_ACC, NORMAL_ACC};
      vector<string> V_planner5{"LIN", "LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3, &V_name4,
                                   &V_name5},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3,
                                   &V_radius4, &V_radius5},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3, &V_speed4,
                                   &V_speed5},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3, &V_acc4, &V_acc5},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3,
                                   &V_planner4, &V_planner5});

      Gripper_op(600, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往试管中转
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往试管中转（横抓）
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
      // 夹取试管
      Gripper_op(950, true, true);

      // 前往马弗炉工作站
      SequenceMove_pilz(V_name4, V_radius4, V_speed4, V_acc4, V_planner4, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 炉外等待
      SequenceMove_pilz(V_name5, V_radius5, V_speed5, V_acc5, V_planner5, false,
                        true);
    } else if (operation == "take") {
      cout << "action = take" << endl;
      vector<string> V_name1{Pname + "_1", Pname + "_2"};
      vector<double> V_radius1{0.0, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"LIN", "LIN"};

      vector<string> V_name2{Pname + "_1", Pname + "_0", "JHhole_0",
                             "JHhole_1"};
      vector<double> V_radius2{0.0, 0.0, 0.2, 0.0};
      vector<double> V_speed2{CLOSE_SPEED, CLOSE_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{CLOSE_ACC, CLOSE_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "LIN", "PTP", "LIN"};

      vector<string> V_name3{"JHhole_0", "JHhole_safe1", "JHhole_safe",
                             "Jhole_0", "Jhole_1"};
      vector<double> V_radius3{0.0, 0.2, 0.2, 0.0, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner3{"LIN", "PTP", "PTP", "PTP", "LIN"};

      vector<string> V_name4{"Jhole_0", "Jhole_safe", Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius4{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed4{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc4{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner4{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name5{Jname + "_0"};
      vector<double> V_radius5{0.0};
      vector<double> V_speed5{NORMAL_SPEED};
      vector<double> V_acc5{NORMAL_ACC};
      vector<string> V_planner5{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3, &V_name4,
                                   &V_name5},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3,
                                   &V_radius4, &V_radius5},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3, &V_speed4,
                                   &V_speed5},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3, &V_acc4, &V_acc5},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3,
                                   &V_planner4, &V_planner5});

      Gripper_op(600, false, true);
      // 前往固体进样
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(950, true, true);
      // 前往试管中转(横向)
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往试管中转
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往试管
      SequenceMove_pilz(V_name4, V_radius4, V_speed4, V_acc4, V_planner4, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name5, V_radius5, V_speed5, V_acc5, V_planner5, false,
                        true);
    } else if (operation == "button") {
      vector<string> V_name1{Pname + "_btn0", Pname + "_btn1"};
      vector<double> V_radius1{0.2, 0.2};
      vector<double> V_speed1{NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP"};

      vector<string> V_name2{Pname + "_btn0", Pname + "_0"};
      vector<double> V_radius2{0.2, 0.2};
      vector<double> V_speed2{NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"PTP", "PTP"};

      SequenceValidate_pilz(vector<vector<string> *>{&V_name1, &V_name2},
                            vector<vector<double> *>{&V_radius1, &V_radius2},
                            vector<vector<double> *>{&V_speed1, &V_speed2},
                            vector<vector<double> *>{&V_acc1, &V_acc2},
                            vector<vector<string> *>{&V_planner1, &V_planner2});

      // 移动到按钮位置
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      sleep(1);

      // 回到等待位
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "新离心工作站") {
    string Jname = "Jhole" + robot_rack_idx + robot_slot_idx;
    string Pname = "P" + _curr_station;
    if (operation == "put") {
      // put
      vector<string> V_name1{"J" + _curr_station, Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname + "_0",
                             Pname + "_1", Pname + "_2",        Pname + "_3",
                             Pname + "_4", Pname + "_5"};
      vector<double> V_radius2{0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN",
                                "LIN", "LIN", "LIN", "LIN"};

      vector<string> V_name3{Pname + "_4", Pname + "_3", Pname + "_2",
                             Pname + "_1", Pname + "_0", "J" + _curr_station};
      vector<double> V_radius3{0.0, 0.0, 0.0, 0.0, 0.2, 0.2};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner3{"LIN", "LIN", "LIN", "LIN", "PTP", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往瓶子
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取瓶子
      Gripper_op(850, true, true);
      // 移动瓶子至工作站
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下瓶子
      Gripper_op(500, false, true);
      // 复位
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    } else if (operation == "take") {
      // take
      vector<string> V_name1{Pname + "_0", Pname + "_1", Pname + "_2",
                             Pname + "_3", Pname + "_4", Pname + "_5"};
      vector<double> V_radius1{0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "LIN", "LIN", "LIN", "LIN", "LIN"};

      vector<string> V_name2{Pname + "_4", Pname + "_3", Pname + "_2",
                             Pname + "_1", Pname + "_0", "J" + _curr_station};
      vector<double> V_radius2{0.0, 0.0, 0.0, 0.0, 0.2, 0.2};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "LIN", "LIN", "LIN", "PTP", "PTP"};

      vector<string> V_name3{Jname + "_0", Jname + "_1"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc3{NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner3{"PTP", "LIN"};

      vector<string> V_name4{Jname + "_0"};
      vector<double> V_radius4{0.0};
      vector<double> V_speed4{CLOSE_SPEED};
      vector<double> V_acc4{CLOSE_ACC};
      vector<string> V_planner4{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3, &V_name4},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3,
                                   &V_radius4},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3, &V_speed4},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3, &V_acc4},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3,
                                   &V_planner4});

      Gripper_op(500, false, true);
      // 前往
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管
      Gripper_op(850, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);

      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
      // 放下试管
      Gripper_op(600, false, true);

      SequenceMove_pilz(V_name4, V_radius4, V_speed4, V_acc4, V_planner4, false,
                        true);
    } else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else if (_curr_station == "电催化工作站") {
    string Jname = "Jrack" + robot_rack_idx;
    string Pname1 = "P电催化工作站"; // 左边
    string Pname2 = "P滴液制样";     // 右边
    string plat = "_PLAT" + station_slot_idx;
    string dry = "_DRY" + station_slot_idx;
    string pool = "_POOL" + station_slot_idx;

    // 放试管架到催化台
    if (operation == "put") {
      vector<string> V_name1{Jname + "_0", Jname + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Jname + "_0", "J" + _curr_station, Pname1 + "_0",
                             Pname1 + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Pname1 + "_0", Pname1 + "_camera"};
      vector<double> V_radius3{0.2, 0.0};
      vector<double> V_speed3{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner3{"LIN", "PTP"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往电催化
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往电催化观察
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
    // 从催化台取试管架
    else if (operation == "take") {
      vector<string> V_name1{"J" + _curr_station, Pname1 + "_0", Pname1 + "_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      vector<string> V_name2{Pname1 + "_0", "J" + _curr_station, Jname + "_0",
                             Jname + "_1"};
      vector<double> V_radius2{0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner2{"LIN", "PTP", "PTP", "LIN"};

      vector<string> V_name3{Jname + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      Gripper_op(0, false, true);
      // 前往电化学
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 夹取试管架
      Gripper_op(470, true, true);
      // 前往机器人平台
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 放下试管架
      Gripper_op(0, false, true);
      // 前往平台关节
      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
    // 取气爪---增加俩点，可能影响后续
    else if (operation == "get_clamp") {
      vector<string> V_name1{Pname2 + "_CLAW0", Pname2 + "_CLAW1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname2 + "_CLAW2", Pname2 + "_CLAW3",
                             Pname2 + "_CLAW4", Pname2 + "_CLAW5",
                             Pname2 + "_CLAW6"};
      vector<double> V_radius2{0.0, 0.0, 0.2, 0.2, 0.2};
      vector<double> V_speed2{CLOSE_SPEED, CLOSE_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{CLOSE_ACC, CLOSE_ACC, NORMAL_ACC, NORMAL_ACC,
                            NORMAL_ACC};
      vector<string> V_planner2{"LIN", "LIN", "PTP", "PTP", "PTP"};

      SequenceValidate_pilz(vector<vector<string> *>{&V_name1, &V_name2},
                            vector<vector<double> *>{&V_radius1, &V_radius2},
                            vector<vector<double> *>{&V_speed1, &V_speed2},
                            vector<vector<double> *>{&V_acc1, &V_acc2},
                            vector<vector<string> *>{&V_planner1, &V_planner2});

      Gripper_op(400, false, true);
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);

      Gripper_op(900, false, true);
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
    }
    // 取气爪---增加俩点，可能影响后续
    else if (operation == "get_clamp_again") {
      vector<string> V_name1{Pname2 + "_CLAW0", Pname2 + "_CLAW1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      vector<string> V_name2{Pname2 + "_CLAW2", Pname2 + "_CLAW3",
                             Pname2 + "_CLAW4"};
      vector<double> V_radius2{0.0, 0.0, 0.2};
      vector<double> V_speed2{CLOSE_SPEED, CLOSE_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{CLOSE_ACC, CLOSE_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "LIN", "PTP"};

      Gripper_op(400, false, true);
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);

      Gripper_op(900, false, true);
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
    }
    // 1从原始位置获取碳纸
    else if (operation == "get_carbon_paper") {

      vector<string> V_name1{Pname2 + "_PLAT0_0", Pname2 + plat + "_0",
                             Pname2 + plat + "_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 2获取碳纸后抬起来
    else if (operation == "move_to_safe_from_carbon_paper") {

      vector<string> V_name1{Pname2 + plat + "_0"};
      vector<double> V_radius1{0.0};
      vector<double> V_speed1{CLOSE_SPEED};
      vector<double> V_acc1{CLOSE_ACC};
      vector<string> V_planner1{"LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 3将碳纸移动到滴液平台
    else if (operation == "move_carbon_paper_to_dripping_station") {

      vector<string> V_name1{Pname2 + "_PLAT_0", Pname2 + "_PLAT_1",
                             Pname1 + "_T0", Pname1 + "_T1"};
      vector<double> V_radius1{0.0, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed1{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner1{"LIN", "PTP", "PTP", "PTP", "LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 4从滴液平台移动碳纸到晾干台
    else if (operation == "move_carbon_paper_to_drying_platform") {
      vector<string> V_name1{Pname1 + "_T0", Pname2 + "_T0", Pname2 + "_T1",
                             Pname2 + dry + "_0", Pname2 + dry + "_1"};
      vector<double> V_radius1{0.0, 0.2, 0.2, 0.2, 0.0};
      vector<double> V_speed1{CLOSE_SPEED, NORMAL_SPEED, NORMAL_SPEED,
                              NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{CLOSE_ACC, NORMAL_ACC, NORMAL_ACC, NORMAL_ACC,
                            CLOSE_ACC};
      vector<string> V_planner1{"LIN", "PTP", "PTP", "PTP", "LIN"};

      // 从滴液平台抬起,放到晾干平台
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 5不带物料直接回到安全位
    else if (operation == "move_to_safe_from_drying_platform_with_no_matter") {
      vector<string> V_name1{Pname2 + dry + "_2", Pname2 + "_DRY_SAFE"};
      vector<double> V_radius1{0.0, 0.2};
      vector<double> V_speed1{CLOSE_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{CLOSE_ACC, NORMAL_ACC};
      vector<string> V_planner1{"LIN", "PTP"};
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }

    // 复位气爪---执行本操作之前一定要先回到安全位，代表第一阶段结束
    else if (operation == "reset_clamp") {
      vector<string> V_name1{Pname2 + "_CLAW4", Pname2 + "_CLAW3",
                             Pname2 + "_CLAW2", Pname2 + "_CLAW1"};
      vector<double> V_radius1{0.2, 0.2, 0.0, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED,
                              CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN", "LIN"};

      vector<string> V_name2{Pname2 + "_CLAW0"};
      vector<double> V_radius2{0.0};
      vector<double> V_speed2{CLOSE_SPEED};
      vector<double> V_acc2{CLOSE_ACC};
      vector<string> V_planner2{"LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      Gripper_op(0, false, true);
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
    }

    //****************************************************
    // 第二阶段-1（先执行旧动作取气爪）去取晾干台的碳纸
    else if (operation == "get_carbon_paper_from_drying_platform") {

      vector<string> V_name1{Pname2 + "_DRY_SAFE", Pname2 + dry + "_2",
                             Pname2 + dry + "_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 带着物料直接回到安全位
    else if (operation == "move_to_safe_from_drying_platform_with_matter") {
      vector<string> V_name1{Pname2 + dry + "_0"};
      vector<double> V_radius1{0.0};
      vector<double> V_speed1{CLOSE_SPEED};
      vector<double> V_acc1{CLOSE_ACC};
      vector<string> V_planner1{"LIN"};
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }

    else if (operation == "move_carbon_paper_to_pool") {
      //_POOL_0为了走线不出错
      vector<string> V_name1{Pname2 + "_POOL_0", Pname2 + pool + "_0",
                             Pname2 + pool + "_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 第二阶段-3从催化池抬起到安全位
    // 再反复执行12-本阶段执行完毕后，应该处于安全位
    else if (operation == "move_to_safe_from_pool") {
      vector<string> V_name1{Pname2 + pool + "_0"};
      vector<double> V_radius1{0.0};
      vector<double> V_speed1{CLOSE_SPEED};
      vector<double> V_acc1{CLOSE_ACC};
      vector<string> V_planner1{"LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }

    //****************************************
    // 第三阶段-1从催化池取出---被顶死导致取料位置发生了改变
    else if (operation == "get_carbon_paper_from_pool") {

      vector<string> V_name1{Pname2 + pool + "_0_B", Pname2 + pool + "_1_B"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "LIN"};

      // 从某安全位，移动到对应的安全位，然后下去
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 第三阶段-2扔掉-记得先执行move_to_safe_from_pool
    else if (operation == "move_carbon_paper_to_recycle") {

      vector<string> V_name1{Pname2 + "_WASTE_SAFE", Pname2 + "_WASTE_0",
                             Pname2 + "_WASTE_1"};
      vector<double> V_radius1{0.2, 0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, CLOSE_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, CLOSE_ACC};
      vector<string> V_planner1{"PTP", "PTP", "LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }
    // 第三阶段-3抬起--高一点
    else if (operation == "move_to_safe_from_recycle") {
      vector<string> V_name1{Pname2 + "_WASTE_0"};
      vector<double> V_radius1{0.0};
      vector<double> V_speed1{CLOSE_SPEED};
      vector<double> V_acc1{CLOSE_ACC};
      vector<string> V_planner1{"LIN"};

      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
    }

    else if (operation == "move") {
      string Jname1 = "Jhole" + station_rack_idx + station_slot_idx; //
      string Jname2 = "Jhole" + robot_rack_idx + robot_slot_idx;     //

      vector<string> V_name1{Jname1 + "_0", Jname1 + "_1"};
      vector<double> V_radius1{0.2, 0.0};
      vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};

      vector<string> V_planner1{"PTP", "LIN"};
      vector<string> V_name2{Jname1 + "_0", Jname2 + "_0", Jname2 + "_1"};
      vector<double> V_radius2{0.0, 0.2, 0.0};
      vector<double> V_speed2{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
      vector<double> V_acc2{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
      vector<string> V_planner2{"LIN", "PTP", "LIN"};

      vector<string> V_name3{Jname2 + "_0"};
      vector<double> V_radius3{0.0};
      vector<double> V_speed3{NORMAL_SPEED};
      vector<double> V_acc3{NORMAL_ACC};
      vector<string> V_planner3{"LIN"};

      SequenceValidate_pilz(
          vector<vector<string> *>{&V_name1, &V_name2, &V_name3},
          vector<vector<double> *>{&V_radius1, &V_radius2, &V_radius3},
          vector<vector<double> *>{&V_speed1, &V_speed2, &V_speed3},
          vector<vector<double> *>{&V_acc1, &V_acc2, &V_acc3},
          vector<vector<string> *>{&V_planner1, &V_planner2, &V_planner3});

      // 松开夹爪
      Gripper_op(400, false, true);
      // 前往起始rack
      SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                        true);
      // 抓取试管架
      Gripper_op(850, true, true);
      // 放到目标rack
      SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                        true);
      // 松开夹爪
      Gripper_op(400, false, true);

      SequenceMove_pilz(V_name3, V_radius3, V_speed3, V_acc3, V_planner3, false,
                        true);
    }
  } else {
    cout << "err station" << endl;
  }

  return true;
}

void ur5e_action::GetNextpose(string name, moveit_msgs::Constraints &pose_goal,
                              geometry_msgs::PoseStamped &next_pose) {
  robot_state::RobotState rstate(_rmodel);
  rstate.setToDefaultValues();
  string name1 = name.substr(1);
  if (name.substr(0, 1) == string("J")) {
    rstate.setVariablePositions(_joints_names, GetJoints(name1));
    pose_goal = kinematic_constraints::constructGoalConstraints(
        rstate, _rmodel->getJointModelGroup("manipulator"));
    Eigen::Isometry3d mat = rstate.getGlobalLinkTransform(_end_link);
    next_pose.pose.position.x = mat.translation()[0];
    next_pose.pose.position.y = mat.translation()[1];
    next_pose.pose.position.z = mat.translation()[2];
  } else {
    if (_Inst_absPoses.find(name1) == _Inst_absPoses.end()) {
      ROS_ERROR_STREAM("no_pose_value: " + name1);
      throw(myException(vector<string>{"NO", "internal_error"}));
    }
    next_pose.pose = _Inst_absPoses.at(name1);
    rstate.setVariablePositions(_joints_names, GetJoints(name1 + "_inverse"));
    rstate.setFromIK(_rmodel->getJointModelGroup("manipulator"), next_pose.pose,
                     _end_link);
    pose_goal = kinematic_constraints::constructGoalConstraints(
        rstate, _rmodel->getJointModelGroup("manipulator"));
  }
  // 将next_pose emplace_back到cur_waypoints。得到所有路径点（位姿表示）
  _waypoints.emplace_back(next_pose.pose);
}

double ur5e_action::GetBlendradius(geometry_msgs::Pose curr_pose,
                                   geometry_msgs::Pose next_pose,
                                   geometry_msgs::Pose n_next_pose) {
  // 若curr_pose和n_next_pose重合,将融合半径置0
  if (pow(n_next_pose.position.x - curr_pose.position.x, 2) +
          pow(n_next_pose.position.y - curr_pose.position.y, 2) +
          pow(n_next_pose.position.z - curr_pose.position.z, 2) <
      0.0025)
    return 0.0;
  double radius1 = sqrt(pow(next_pose.position.x - curr_pose.position.x, 2) +
                        pow(next_pose.position.y - curr_pose.position.y, 2) +
                        pow(next_pose.position.z - curr_pose.position.z, 2));
  double radius2 = sqrt(pow(n_next_pose.position.x - next_pose.position.x, 2) +
                        pow(n_next_pose.position.y - next_pose.position.y, 2) +
                        pow(n_next_pose.position.z - next_pose.position.z, 2));
  double radius = min(radius1, radius2);
  if (radius < 0.02)
    return 0.0;
  return radius;
}

ur5e_action::~ur5e_action() {
  if (_kinematic_state) {
    _kinematic_state.reset();
  }
  if (_dbptr) {
    _dbptr.reset();
  }
  if (_mgptr) {
    _mgptr.reset();
  }
  ROS_INFO_STREAM("Bye bye!");
}

void ur5e_action::MoveLine_pilz(Movetowards towards, double dis,
                                double speed_factor) {
  this->_mgptr->setPlannerId("LIN");
  moveit::planning_interface::MoveGroupInterface::Plan Lineplan;
  geometry_msgs::PoseStamped target_pose = _mgptr->getCurrentPose();
  switch (towards) {
  case UP:
    target_pose.pose.position.z += dis;
    break;
  case DOWN:
    target_pose.pose.position.z -= dis;
    break;
  case LEFT:
    target_pose.pose.position.x -= dis;
    break;
  case RIGHT:
    target_pose.pose.position.x += dis;
    break;
  case FORWARD:
    target_pose.pose.position.y += dis;
    break;
  case BACKWARD:
    target_pose.pose.position.y -= dis;
    break;
  }
  this->_mgptr->setPoseTarget(target_pose);
  this->_mgptr->setMaxVelocityScalingFactor(speed_factor);

  int replan_num = 10;
  int replan_count = 0;
  while (this->_mgptr->plan(Lineplan) !=
             moveit::planning_interface::MoveItErrorCode::SUCCESS &&
         replan_count < replan_num) {
    ++replan_count;
  }
  if (replan_count == 10) {
    cerr << "\033[31mERROR:plan line trajectory FAILED\n\033[0m";
    this->_mgptr->setMaxVelocityScalingFactor(NORMAL_SPEED);
    this->_mgptr->setPlannerId("PTP");
    throw(myException(vector<string>{"NO", "plan_failed"}));
  }

  this->_mgptr->execute(Lineplan);
  this->_mgptr->setMaxVelocityScalingFactor(NORMAL_SPEED);
  this->_mgptr->setPlannerId("PTP");
}

void ur5e_action::MoveCircle_pilz(CirType type, std::string auxiliary_point,
                                  std::string goal_point) {
  moveit_msgs::GetMotionPlan srvreq;
  moveit_msgs::Constraints pose_goal;
  planning_interface::MotionPlanRequest req;
  geometry_msgs::PoseStamped next_pose;
  moveit_msgs::PositionConstraint posConstraint;

  _mgptr->constructMotionPlanRequest(req);
  req.group_name = "manipulator";
  req.goal_constraints.resize(1);
  req.path_constraints.position_constraints.resize(1);
  req.max_velocity_scaling_factor = CLOSE_SPEED;
  req.max_acceleration_scaling_factor = CLOSE_ACC;
  req.planner_id = "CIRC";
  // _kinematic_state->setVariablePositions(this->_joints_names,
  // this->_mgptr->getCurrentState());
  // robot_state::robotStateToRobotStateMsg(*(this->_mgptr->getCurrentState()),
  // req.start_state);
  if (type == CENTER)
    req.path_constraints.name = "center";
  else if (type == INTERIM)
    req.path_constraints.name = "interim";
  GetNextpose(auxiliary_point, pose_goal, next_pose);
  posConstraint.link_name = _end_link;
  posConstraint.constraint_region.primitive_poses.emplace_back(next_pose.pose);
  req.path_constraints.position_constraints[0] = posConstraint;

  GetNextpose(goal_point, pose_goal, next_pose);
  req.goal_constraints[0] = pose_goal;
  srvreq.request.motion_plan_request = req;
  srvreq.response.motion_plan_response.error_code.val = 0;

  int replan_num = 10;
  int replan_count = 0;
  while (srvreq.response.motion_plan_response.error_code.val != 1 &&
         replan_count < replan_num) {
    this->_client3_.call(srvreq);
    ++replan_count;
  }
  if (replan_count == 10)
    throw(myException(vector<string>{"NO", "MoveCircle plan failed"}));

  this->_mgptr->execute(srvreq.response.motion_plan_response.trajectory);
}

void ur5e_action::Init(const string filename) {

  while (_dbptr->robot_init() != 0) {
    JsonPubErr("robot init failed,please check the robot darhboard");
  }

  while (_dbptr->load_program(filename) != 0) {
    JsonPubErr("robot load filename failed,please check the robot darhboard");
  }

  _msgptr->state = "idle";
  // _dbptr->DO_init();
}

void ur5e_action::Gripper_op(uint16_t position, bool check, bool block,
                             uint16_t force, uint16_t speed) {
  if (_no_serial)
    ;
  else {
    this->_Vname.emplace_back(to_string(min(_pgi_ptr->getPos(), position)));
    this->_Vplanner.emplace_back("");
    this->_Vspeed.emplace_back(0);
    this->_Vacc.emplace_back(0);
    this->_Vradius.emplace_back(0);
    this->_grasp_flag = false;
    _pgi_ptr->setall(position, block, force, speed);
    if (this->_checkgrasp && check) {
      if (_pgi_ptr->isGripCur()) {
        this->_grasp_flag = true;
      } else {
        ROS_ERROR_STREAM("没有夹到物体");
        throw(myException(vector<string>{"NO", "grasp_empty"}));
      }
    }
  }
}

void ur5e_action::JsonPubDone() {
  _msgptr->state = "done";
  JsonPub();
  _msgptr->state = "idle";
}

void ur5e_action::JsonPubErr(string detail) {
  _msgptr->state = "error";
  _msgptr->detail = detail;
  JsonPub();
}

void ur5e_action::JsonPub() {
  Json::Value root;
  root["id"] = _msgptr->id;
  root["exper_no"] = _msgptr->exper_no;
  root["stamp"] = GetStamp();
  root["state"] = _msgptr->state;

  if ((_msgptr->state == "running") || (_msgptr->state == "error")) {
    root["detail"] = _msgptr->detail;
  }

  std_msgs::String pubmsgs;
  pubmsgs.data = root.toStyledString();
  ROS_INFO_STREAM("--------------auto reply info---------------\n"
                  << pubmsgs.data);
  _pub1_.publish(pubmsgs);
}

bool ur5e_action::InsertPoses(vector<string> &V_name) {
  if (_curr_station == "地图原点")
    return false;
  vector<string> noInsertstation{};
  vector<regex> defaultPose1{regex("^Jrack.*"), regex("^Jhole.*"),
                             regex("^Jdefault_0$")};
  bool flag1 = false, flag2 = false, flag_t1 = false, flag_t2 = false;
  // 判断操作臂当前是否在机器人侧
  for_each(defaultPose1.begin(), defaultPose1.end(), [this, &flag_t1](regex a) {
    if (regex_match(this->_curr_pose, a)) {
      flag_t1 = true;
      return;
    }
  });
  // 判断下一路径点是否在机器人侧
  for_each(defaultPose1.begin(), defaultPose1.end(),
           [&V_name, &flag_t2](regex a) {
             if (regex_match(V_name[0], a)) {
               flag_t2 = true;
               return;
             }
           });
  // 操作臂在机器人侧，需要移动到仪器侧
  flag1 = flag_t1 && V_name[0].substr(0, 1) == "P";
  // 操作臂在机器人侧，需要移动到重定位处
  flag2 = flag_t1 && V_name[0].substr(0, 5) == "Jrelo";
  // 操作臂在机器人侧，需要移动到重定位处，但不需要插入过渡点的工作站
  // flag2 = flag2 && find(noInsertstation.begin(), noInsertstation.end(),
  // _curr_station) == noInsertstation.end();
  // 操作臂在仪器侧，需要移动到机器人侧
  bool flag3 = this->_curr_pose.substr(0, 1) == "P" && flag_t2;
  // 操作臂在重定位处，需要移动到机器人侧
  bool flag4 = this->_curr_pose.substr(0, 5) == "Jrelo" && flag_t2;
  // 操作臂在重定位处，需要移动到机器人侧，但不需要插入过渡点的工作站
  // flag4 = flag4 && find(noInsertstation.begin(), noInsertstation.end(),
  // _curr_station) == noInsertstation.end();
  bool flag = flag1 || flag2 || flag3 || flag4;
  if (flag) {
    std::cout << "Insert: J" + this->_curr_station << std::endl;
    V_name.insert(V_name.begin(), "J" + this->_curr_station);
    return true;
  }
  return false;
}

void ur5e_action::ResetArm() {
  ros::Duration(3.0).sleep();
  vector<regex> defaultPose{regex("^Jrack[0-3]{1}_0$"),
                            regex("^JH?hole[0-9]*_0$"), regex("^Jdefault_0$"),
                            regex("^J(LIBS)?[\u4e00-\u9fa5]+[站,架]{1}_?0?$"),
                            regex("^P[\u4e00-\u9fa5]+_camera")};
  regex base("^[0-9]{0,4}$");
  vector<string> Vname(this->_Vname);
  vector<string> Vplanner(this->_Vplanner);
  vector<double> Vradius(this->_Vradius.size(), 0);
  vector<double> Vspeed(this->_Vspeed.size(), 0.3);
  vector<double> Vacc(this->_Vacc.size(), 0.2);
  // 清除记录的轨迹规划信息
  this->_Vname.clear();
  this->_Vradius.clear();
  this->_Vspeed.clear();
  this->_Vacc.clear();
  this->_Vplanner.clear();
  reverse(Vname.begin(), Vname.end());
  reverse(Vplanner.begin(), Vplanner.end());
  while (Vname.size() > 0) {
    size_t i = 0;
    // 不是夹爪操作，递增
    for (; i < Vname.size(); ++i) {
      if (regex_match(Vname[i], base))
        break;
    }
    // 说明夹爪操作前有路径点
    if (i > 0) {
      vector<string> Vname_(Vname.begin(), Vname.begin() + i);
      vector<double> Vradius_(Vradius.begin(), Vradius.begin() + i);
      vector<double> Vspeed_(Vspeed.begin(), Vspeed.begin() + i);
      vector<double> Vacc_(Vacc.begin(), Vacc.begin() + i);
      vector<string> Vplanner_(Vplanner.begin(), Vplanner.begin() + i);
      this->SequenceMove_pilz(Vname_, Vradius_, Vspeed_, Vacc_, Vplanner_,
                              false, false);
      // 更新机械臂当前位姿。异常处理不需要实时更新。
      this->_curr_pose = Vname_.back();
      // 左闭右开
      Vname.erase(Vname.begin(), Vname.begin() + i);
      Vplanner.erase(Vplanner.begin(), Vplanner.begin() + i);
      Vradius.erase(Vradius.begin(), Vradius.begin() + i);
      Vspeed.erase(Vspeed.begin(), Vspeed.begin() + i);
      Vacc.erase(Vacc.begin(), Vacc.begin() + i);
    } else {
      _pgi_ptr->setall(uint16_t(atoi(Vname[0].c_str())), true);
      _grasp_flag = (_pgi_ptr->isGripCur() ? true : false);
      Vname.erase(Vname.begin());
      Vplanner.erase(Vplanner.begin());
      Vradius.erase(Vradius.begin());
      Vspeed.erase(Vspeed.begin());
      Vacc.erase(Vacc.begin());
    }
    // 如果当前没有夹物体，则移动到最近的默认位置即可，后续路径点直接删除
    if (!this->_grasp_flag) {
      bool flag = false;
      size_t j = 0;
      // 找到Vname中最近的默认位置
      for (; j < Vname.size(); ++j) {
        for_each(defaultPose.begin(), defaultPose.end(),
                 [this, &flag, &Vname, &j](regex a) {
                   if (regex_match(Vname[j], a)) {
                     flag = true;
                     return;
                   }
                 });
        if (flag)
          break;
      }
      if (flag && j < Vname.size()) {
        Vname.erase(Vname.begin() + j + 1, Vname.end());
        Vradius.erase(Vradius.begin() + j + 1, Vradius.end());
        Vspeed.erase(Vspeed.begin() + j + 1, Vspeed.end());
        Vacc.erase(Vacc.begin() + j + 1, Vacc.end());
        Vplanner.erase(Vplanner.begin() + j + 1, Vplanner.end());
      }
    }
  }
}

moveit_msgs::MotionSequenceRequest
ur5e_action::ConstructSeqReq(vector<string> &V_name, vector<double> &V_radius,
                             vector<double> &V_speed, vector<double> &V_acc,
                             vector<string> &V_planner, bool validate) {
  moveit_msgs::Constraints pose_goal;
  planning_interface::MotionPlanRequest freq, req;
  moveit_msgs::MotionSequenceItem sequenceItem;
  moveit_msgs::MotionSequenceRequest sequenceRequest;
  geometry_msgs::PoseStamped curr_pose, next_pose, n_next_pose;

  _mgptr->constructMotionPlanRequest(freq);
  _mgptr->constructMotionPlanRequest(req);
  freq.group_name = "manipulator";
  req.group_name = "manipulator";
  freq.goal_constraints.resize(1);
  req.goal_constraints.resize(1);
  // 如果是验证，需要将上一条路径的最终路径点作为此次的起始点
  if (validate) {
    Isometry3d mat = _kinematic_state->getGlobalLinkTransform(_end_link);
    curr_pose.pose.position.x = mat.translation()[0];
    curr_pose.pose.position.y = mat.translation()[1];
    curr_pose.pose.position.z = mat.translation()[2];
    robot_state::robotStateToRobotStateMsg(*_kinematic_state, freq.start_state);
  } else
    curr_pose = this->_mgptr->getCurrentPose();
  // 构造第1个MotionPlanRequest
  GetNextpose(V_name[0], pose_goal, next_pose);
  freq.goal_constraints[0] = pose_goal;
  freq.max_velocity_scaling_factor = V_speed[0];
  freq.max_acceleration_scaling_factor = V_acc[0];
  freq.planner_id = V_planner[0];
  // 若多个目标路点,则计算融合半径;若只有一个目标路点,融合半径为0
  if (V_name.size() > 1) {
    GetNextpose(V_name[1], pose_goal, n_next_pose);
    sequenceItem.blend_radius =
        GetBlendradius(curr_pose.pose, next_pose.pose, n_next_pose.pose) *
        V_radius[0];
  } else
    sequenceItem.blend_radius = 0;
  sequenceItem.req = freq;
  sequenceRequest.items.emplace_back(sequenceItem);

  if (V_name.size() > 1) {
    // 构造第2~n-1个MotionPlanRequest
    int i;
    for (i = 1; i < V_name.size() - 1; ++i) {
      curr_pose = next_pose;
      next_pose = n_next_pose;
      req.goal_constraints[0] = pose_goal;
      req.max_velocity_scaling_factor = V_speed[i];
      req.max_acceleration_scaling_factor = V_acc[i];
      req.planner_id = V_planner[i];
      GetNextpose(V_name[i + 1], pose_goal, n_next_pose);
      sequenceItem.blend_radius =
          GetBlendradius(curr_pose.pose, next_pose.pose, n_next_pose.pose) *
          V_radius[i];
      sequenceItem.req = req;
      sequenceRequest.items.emplace_back(sequenceItem);
    }
    // 构造第n个MotionPlanRequest
    req.goal_constraints[0] = pose_goal;
    req.max_velocity_scaling_factor = V_speed[i];
    req.max_acceleration_scaling_factor = V_acc[i];
    req.planner_id = V_planner[i];
    sequenceItem.blend_radius = 0.0;
    sequenceItem.req = req;
    sequenceRequest.items.emplace_back(sequenceItem);
  }
  return sequenceRequest;
}

void ur5e_action::GoDefault(string station_name) {
  cout << "GoDefault:" << station_name << endl;

  if (station_name != "") {
    cout << "LoadStationInfo:" << station_name << endl;
    this->_Inst_relPoses.clear();
    this->_Inst_absPoses.clear();
    LoadStationInfo(station_name);
  }
  cout << "go Jdefault_0:" << endl;
  vector<string> V_name1{"Jdefault_0"};
  vector<double> V_radius1{0.0};
  vector<double> V_speed1{NORMAL_SPEED};
  vector<double> V_acc1{NORMAL_ACC};
  vector<string> V_planner1{"PTP"};
  SequenceValidate_pilz(
      vector<vector<string> *>{&V_name1}, vector<vector<double> *>{&V_radius1},
      vector<vector<double> *>{&V_speed1}, vector<vector<double> *>{&V_acc1},
      vector<vector<string> *>{&V_planner1});
  SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                    true);
}

void ur5e_action::Relocation() {
  vector<string> V_name1{"Jrelocation_joints_" + _curr_station};
  vector<double> V_radius1{0.0};
  vector<double> V_speed1{NORMAL_SPEED};
  vector<double> V_acc1{NORMAL_ACC};
  vector<string> V_planner1{"PTP"};
  SequenceValidate_pilz(
      vector<vector<string> *>{&V_name1}, vector<vector<double> *>{&V_radius1},
      vector<vector<double> *>{&V_speed1}, vector<vector<double> *>{&V_acc1},
      vector<vector<string> *>{&V_planner1});
  SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                    true);
  CalculatePoses(_curr_station);

  // 重定位后，回到默认点
  V_name1 = {"J" + _curr_station};
  SequenceValidate_pilz(
      vector<vector<string> *>{&V_name1}, vector<vector<double> *>{&V_radius1},
      vector<vector<double> *>{&V_speed1}, vector<vector<double> *>{&V_acc1},
      vector<vector<string> *>{&V_planner1});
  SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                    true);
}

void ur5e_action::RelocationElectrocatalysis() {
  LoadStationInfo("滴液制样");
  vector<string> V_name1{"J" + _curr_station, "Jrelocation_joints_滴液制样"};
  vector<double> V_radius1{0.2, 0.0};
  vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED};
  vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC};
  vector<string> V_planner1{"PTP", "PTP"};
  SequenceValidate_pilz(
      vector<vector<string> *>{&V_name1}, vector<vector<double> *>{&V_radius1},
      vector<vector<double> *>{&V_speed1}, vector<vector<double> *>{&V_acc1},
      vector<vector<string> *>{&V_planner1});

  Gripper_op(0, false, true);
  // 前往电化学定位
  SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                    true);
  // 进行电化学定位
  CalculatePoses("滴液制样");
}

void ur5e_action::RelocationLibs() {
  LoadStationInfo("LIBS样品架");
  vector<string> V_name1{"J" + _curr_station, "JLIBS样品架",
                         "Jrelocation_joints_LIBS样品架"};
  vector<double> V_radius1{0.2, 0.2, 0.0};
  vector<double> V_speed1{NORMAL_SPEED, NORMAL_SPEED, NORMAL_SPEED};
  vector<double> V_acc1{NORMAL_ACC, NORMAL_ACC, NORMAL_ACC};
  vector<string> V_planner1{"PTP", "PTP", "PTP"};

  vector<string> V_name2{"JLIBS样品架"};
  vector<double> V_radius2{0.0};
  vector<double> V_speed2{NORMAL_SPEED};
  vector<double> V_acc2{NORMAL_ACC};
  vector<string> V_planner2{"PTP"};

  SequenceValidate_pilz(vector<vector<string> *>{&V_name1, &V_name2},
                        vector<vector<double> *>{&V_radius1, &V_radius2},
                        vector<vector<double> *>{&V_speed1, &V_speed2},
                        vector<vector<double> *>{&V_acc1, &V_acc2},
                        vector<vector<string> *>{&V_planner1, &V_planner2});

  Gripper_op(0, false, false);
  // 前往LIBS样品定位
  SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1, false,
                    true);
  // 进行LIBS样品定位
  CalculatePoses("LIBS样品架");
  SequenceMove_pilz(V_name2, V_radius2, V_speed2, V_acc2, V_planner2, false,
                    true);
}