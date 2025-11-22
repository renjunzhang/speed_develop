#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <time.h>
#include <QtWidgets/QMessageBox>

#include "chemical_platform_system.h"
#include "chemical_util.h"
#include "devices/Robot/ur5e_robot_control.h"
#include "geometry_msgs/WrenchStamped.h"

std::map<DeviceErrorCode, std::string> DeviceErrorCode_Str = {
    {DeviceErrorCode::DEVICE_SUCCESS, "DEVICE_SUCCESS"},
    {DeviceErrorCode::DEVICE_LACK_ATTRIBUTION, "DEVICE_LACK_ATTRIBUTION"},
    {DeviceErrorCode::DEVICE_INVALID_ATTRIBUTION, "DEVICE_INVALID_ATTRIBUTION"},
    {DeviceErrorCode::DEVICE_CONNECT_FAILED, "DEVICE_CONNECT_FAILED"},
    {DeviceErrorCode::DEVICE_INVALID_SCRIPT, "DEVICE_INVALID_SCRIPT"},
    {DeviceErrorCode::DEVICE_ISRUNNIN, "DEVICE_ISRUNNIN"},
    {DeviceErrorCode::DEVICE_PGI_UNGRIPED, "DEVICE_PGI_UNGRIPED"},
    {DeviceErrorCode::DEVICE_IS_FAULT, "DEVICE_IS_FAULT"},
    {DeviceErrorCode::DEVICE_COMMUNICATION_ERROR, "DEVICE_COMMUNICATION_ERROR"},
    {DeviceErrorCode::DEVICE_EXECUTE_FAILED, "DEVICE_EXECUTE_FAILED"},
    {DeviceErrorCode::DEVICE_RELOCATION_FAILED,"relocation_error,DEVICE_RELOCATION_FAILED"},
    {DeviceErrorCode::DEVICE_CV_LOCATOR_OFFLINE,"relocation_error,DEVICE_CV_LOCATOR_OFFLINE"}
    };

std::map<DeviceStatus, std::string> DeviceStatus_Str = {
    {DeviceStatus::DS_IDEL, "DS_IDEL"},
    {DeviceStatus::DS_RUNNING, "DS_RUNNING"},
    {DeviceStatus::DS_UNCONNECTED, "DS_UNCONNECTED"},
    {DeviceStatus::DS_FAULT, "DS_FAULT"}};

std::map<std::string, std::string> station_list;
// {
//     {"libs", "LIBS工作站"},
//     {"high_flux_xrd_workstation", "XRD工作站"},
//     {"infrared_spectrum", "红外光谱工作站"},
//     {"high_flux_electrocatalysis_workstation", "电催化工作站"},
//     {"uv_vis", "紫外光谱工作站"},
//     {"flourescence", "荧光光谱工作站"},
//     {"raman_spectra", "拉曼光谱工作站"},
//     {"storage_workstation", "机器人物料站"}};

std::map<std::string, std::string>
get_station_name_map(QString station_config_path, QString filename) {
  std::map<std::string, std::string> list;
  list.clear();

  auto dir = QDir(station_config_path);
  if (!dir.exists())
    dir.mkdir(station_config_path);

  QFile stations(station_config_path + filename);
  if (!stations.open(QIODevice::ReadOnly)) {
    ROS_WARN_STREAM("Load station name file failed, file: ["
                    << stations.fileName().toLocal8Bit().toStdString()
                    << "] may not exist");
    return list;
  }
  auto rawdata = stations.readAll();
  stations.close();
  QJsonParseError jerr;
  QJsonDocument doc = QJsonDocument::fromJson(rawdata, &jerr);
  if (jerr.error != QJsonParseError::NoError) {
    ROS_WARN_STREAM("Paser json file failed, file: ["
                    << stations.fileName().toLocal8Bit().toStdString()
                    << "] format incorrect");
    return list;
  }
  QJsonObject root = doc.object();
  auto key_itr = root.begin();
  while (key_itr != root.end()) {
    auto key = key_itr.key().toLocal8Bit().toStdString();
    auto value = key_itr.value().toString().toLocal8Bit().toStdString();

    ROS_INFO_STREAM(key << "####" << value);
    list[key] = value;
    key_itr++;
  }
  return list;
}

ChemicalPlatform::ChemicalPlatform(ros::NodeHandle &nh) {
  std::string gripper, gripper_baudrate, screwslider, screwslider_baudrate,
      record_data_path, end_link;
  bool have_ft_sensor, need_pgi, need_dripper, need_screwslide;
  nh.param<std::string>("/chemicalrobot_arm_new/gripper", gripper, "/dev/PGI");
  nh.param<std::string>("/chemicalrobot_arm_new/gripper_baudrate",
                        gripper_baudrate, "115200");
  nh.param<std::string>("/chemicalrobot_arm_new/screwslider", screwslider,
                        "/dev/track");
  nh.param<std::string>("/chemicalrobot_arm_new/screwslider_baudrate",
                        screwslider_baudrate, "115200");
  nh.param<bool>("/chemicalrobot_arm_new/have_ft_sensor", have_ft_sensor,
                 false);
  nh.param<std::string>(
      "/chemicalrobot_arm_new/record_data_path", record_data_path,
      ros::package::getPath("chemicalrobot_arm_new") + "/Data");
  nh.param<std::string>("/chemicalrobot_arm_new/end_link", end_link, "tool0");
  nh.param<bool>("chemicalrobot_arm_new/have_pgi", need_pgi, false);
  nh.param<bool>("chemicalrobot_arm_new/have_dripper", need_dripper, false);
  nh.param<bool>("chemicalrobot_arm_new/have_screwslide", need_screwslide, false);

  auto station_config_path = QString::fromLocal8Bit(
      (ros::package::getPath("chemicalrobot_arm_new") + "/Config/").c_str());
  auto station_filename = QString("station_name_map.json");
  station_list = get_station_name_map(station_config_path, station_filename);

  // init message bridge
  _msgptr = std::make_shared<MSG>();
  _msgptr->state = "error";

  // init pgi if exist
  DeviceErrorCode res;
  if (need_pgi) {
    ROS_INFO_STREAM("PGI init...");
    _pgiptr = std::make_shared<pgi>(nh);
    _pgiptr->setAttribute("device_name", (void *)&gripper);
    _pgiptr->setAttribute("baudrate", (void *)&gripper_baudrate);
    res = _pgiptr->init(nullptr);
    if (res != DEVICE_SUCCESS)
      ROS_WARN_STREAM("pgi init failed with error message:" +
                      DeviceErrorCode_Str[res]);
  }

  // init dripper if exist
  if (need_dripper) {
    _dripper_ptr = std::make_shared<DripperControl>();
    _dripper_ptr->setAttribute("device_name", (void *)&gripper);
    res = _dripper_ptr->init(nullptr);
    if (res != DEVICE_SUCCESS)
      ROS_WARN_STREAM("dripper init failed with error message:" +
                      DeviceErrorCode_Str[res]);
  }

  // init screw slide if exist
  if (need_screwslide) {
    _screwslide_ptr = std::make_shared<ScrewSlideControl>();
    _screwslide_ptr->setAttribute("device_name", (void *)&screwslider);
    _screwslide_ptr->setAttribute("baudrate", (void *)&screwslider_baudrate);
    res = _screwslide_ptr->init(nullptr);
    if (res != DEVICE_SUCCESS)
      ROS_WARN_STREAM("screw slide init failed with error message:" +
                      DeviceErrorCode_Str[res]);
  }

  // init robot if exist
  _robot_ptr = std::make_shared<ur5e_robot>(nh);
  _robot_ptr->setAttribute("record_data_path", (void *)&record_data_path);
  _robot_ptr->setAttribute("end_link", (void *)&end_link);
  res = _robot_ptr->init(&nh);
  if (res != DEVICE_SUCCESS)
    ROS_WARN_STREAM("ur5e robot init failed with error message:" +
                    DeviceErrorCode_Str[res]);
  relocationed = false;
  last_station_name = "";

  _sub1_ = nh.subscribe<std_msgs::String>(
      "/obsOperation_in", 1, &ChemicalPlatform::obs_Callback, this);
  _sub2_ = nh.subscribe<std_msgs::String>(
      "/grip_operation_in", 1, &ChemicalPlatform::pgi_Callback, this);
  _pub1_ = nh.advertise<std_msgs::String>("/obsOperation_out", 1);
  _pub3_ =
      nh.advertise<actionlib_msgs::GoalID>("/execute_trajectory/cancel", 1);
  _pub4_ = nh.advertise<std_msgs::String>("/robot_joints",1);
  _pub5_ = nh.advertise<std_msgs::String>("/gripped_target",1);
  _pub6_ = nh.advertise<std_msgs::String>("/grip_operation_out",1);
  _service =
      nh.advertiseService("robot", &ChemicalPlatform::query_Callback, this);

  _timer = nh.createTimer(ros::Duration(0.05), &ChemicalPlatform::JointsPub, this); // 定时器

  _widget = new mywidget();
}

ChemicalPlatform::~ChemicalPlatform()
{
  delete _widget;
  ROS_INFO_STREAM("Bye bye!");
}

std::string ChemicalPlatform::GetStamp() {
  long long stamp = ros::Time::now().toNSec() / 1000000;
  return std::to_string(stamp);
}

bool ChemicalPlatform::query_Callback(
    chemicalrobot_arm_new::DmsService::Request &req,
    chemicalrobot_arm_new::DmsService::Response &res) {
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

void ChemicalPlatform::pgi_Callback(const std_msgs::StringConstPtr &pgiOperation_in){
  Json::Reader jsonreader;
  Json::Value root;
  jsonreader.parse(pgiOperation_in->data, root);
  ROS_INFO_STREAM("----------cmd recv-------------\n" + root.toStyledString());

  Json::Value pub_data;
  std_msgs::String pubmsg;
  pub_data["state"] = "error";
  if(root["operations"].empty())
  {
    ROS_ERROR_STREAM("Set pgi failed, root[\"operations\"] is empty");
  }
  else{
    try{
      auto dis = root["operations"].asInt();
      dis = dis > 1000 ? 1000 : dis;
      dis = dis < 0 ? 0 : dis;
      std::string cmd = "G " + std::to_string(dis) + " false true 20 50";
      auto res = _pgiptr->execute_script(cmd);
      if(res == DEVICE_SUCCESS)
        pub_data["state"] = "done";
      else
        ROS_ERROR_STREAM("Set pgi failed with error message: " << DeviceErrorCode_Str[res]);
    }
    catch(std::invalid_argument e){
      ROS_ERROR_STREAM("Set pgi failed, invalid_argument: " << e.what());
    }
  }    
  pubmsg.data = pub_data.toStyledString();
  _pub6_.publish(pubmsg);
}

void ChemicalPlatform::obs_Callback(
    const std_msgs::StringConstPtr &obsOperation_in) {
  Json::Reader jsonreader;
  Json::Value root;
  jsonreader.parse(obsOperation_in->data, root);
  ROS_INFO_STREAM("----------cmd recv-------------\n" + root.toStyledString());

  // if(_msgptr->state != "idle"){
  //   return;
  // }

  if (!root["id"].isNull()) {
    _msgptr->id = root["id"].asString();
  }
  if (!root["exper_no"].isNull()) {
    _msgptr->exper_no = root["exper_no"].asString();
  }
  _msgptr->state = "running";
  _msgptr->detail = "";

  std::string station = station_list[root["destination"].asString()];
  if (station.empty())
  {
    ROS_ERROR_STREAM("empty station received:");
    JsonPubErr("empty station received");
    return;
  }

  auto _robot_record_data =
      *(std::map<QString, robot_record_data> *)_robot_ptr->getAttribute(
          "_robot_record_data");
  if (_robot_record_data[QString::fromLocal8Bit(station.c_str())]
          .station_name.isEmpty())
  {
    ROS_ERROR_STREAM("station file:" + station + ".json not found");
    JsonPubErr("station file:" + station + ".json not found");
  }

  Json::Value obj = root["operations"];

  for (int i = 0; i < 1; i++)
  {
    std::string operation =
        obj[i]["operation"].empty() ? "" : obj[i]["operation"].asString();
    std::string station_rack_idx = obj[i]["rack_station"].empty()
                                       ? "0"
                                       : obj[i]["rack_station"].asString();
    std::string station_slot_idx = obj[i]["slot_station"].empty()
                                       ? "0"
                                       : obj[i]["slot_station"].asString();
    std::string station_btn_idx = obj[i]["button_index"].empty()
                                      ? "0"
                                      : obj[i]["button_index"].asString();
    std::string robot_rack_idx =
        obj[i]["rack_robot"].empty() ? "0" : obj[i]["rack_robot"].asString();
    std::string robot_slot_idx =
        obj[i]["slot_robot"].empty() ? "0" : obj[i]["slot_robot"].asString();
    std::string phase =
        obj[i]["phase"].empty() ? "" : obj[i]["phase"].asString();

    // 当前操作
    _msgptr->detail = operation;

    // set dms message to robot controller
    _robot_ptr->setAttribute("_current_station_name", &station);
    _robot_ptr->setAttribute("_current_operation", &operation);
    _robot_ptr->setAttribute("_rack_station", &station_rack_idx);
    _robot_ptr->setAttribute("_slot_station", &station_slot_idx);
    _robot_ptr->setAttribute("_rack_robot", &robot_rack_idx);
    _robot_ptr->setAttribute("_slot_robot", &robot_slot_idx);
    auto cmds =
        _robot_record_data[QString::fromLocal8Bit(station.c_str())]._cmds;
    auto scripts_qt = cmds[QString::fromLocal8Bit(operation.c_str())];
    if (scripts_qt.isEmpty())
    {
      JsonPubErr("Script of operation:" + operation + " is empty");
      return;
    }

    StringList scripts;
    foreach (auto script, scripts_qt)
      scripts.push_back(script.toLocal8Bit().toStdString());

    // execute origional scripts
    current_statison_name = station.c_str();
    current_operation_name = operation.c_str();
    auto res = execute_scripts(scripts);
    last_station_name = current_statison_name;

    if(current_operation_name == "relocation" && res == DEVICE_SUCCESS)
      relocationed = true;
    
    if (res != DEVICE_SUCCESS)
    {
      if (res == DEVICE_RELOCATION_FAILED || res == DEVICE_CV_LOCATOR_OFFLINE)
      {
        // relocation error,reset first
        auto reset_script_qt = cmds["reset"];
        if (reset_script_qt.isEmpty())
          ROS_ERROR_STREAM("Script of operation \"reset\" is empty");
        else
        {
          StringList reset_scripts;
          foreach (auto script, reset_script_qt)
            reset_scripts.push_back(script.toLocal8Bit().toStdString());
          execute_scripts(reset_scripts);
        }
        JsonPubErr("chemical platform system execute_scripts failed with error message : " +
                   DeviceErrorCode_Str[res]);
        return;
      }

      ROS_ERROR_STREAM("chemical platform system execute_scripts failed with error message : " <<
                       DeviceErrorCode_Str[res]);
      std::string tips = "请先手动恢复机械臂异常，然后再关闭弹窗,当前操作：\n";
      tips += operation + "," + 
              (obj[i]["rack_station"].empty() ? "" : "rack_station :" + obj[i]["rack_station"].asString()) + "," +
              (obj[i]["slot_station"].empty() ? "" : "slot_station :" + obj[i]["slot_station"].asString()) + "," +
              (obj[i]["rack_robot"].empty() ? "" : "rack_robot :" + obj[i]["rack_robot"].asString()) + "," +
              (obj[i]["slot_robot"].empty() ? "" : "slot_robot :" + obj[i]["slot_robot"].asString());

      //此处使用信号触发弹窗，因为ros可能工作在多线程中，obs_callback可能执行在不同线程中
      //qt的messagebox必须在主线程中调用，否则会报错多线程重绘错误
      //因此在主线程中构造mywidget对象：_widget，通过信号在子线程中发出弹窗信号，主线程中绘制弹窗
      _widget->button_num = -1;//初始化返回按钮值
      emit _widget->open_message_box(QString::fromLocal8Bit(tips.c_str()));
      while(_widget->button_num == -1){
        //循环等待弹窗关闭时的按钮选择结果
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (_widget->button_num == QMessageBox::Yes)
      {
        //用户选择上报错误
        JsonPubErr("chemical platform system execute_scripts failed with error message : " +
                   DeviceErrorCode_Str[res]);
        return;
      }
      //用户选择跳过错误
      ROS_ERROR_STREAM("User canceled error publishment,mission continue");
    }

    ros::Duration(1).sleep();
  }
  // 执行结束
  JsonPubDone();
}

mywidget::mywidget(QWidget *parent) :
    QWidget(parent)
{
  connect(this, SIGNAL(open_message_box(QString)), this,
                   SLOT(on_message_box_open(QString)));
}

mywidget::~mywidget(){
  
}

void mywidget::on_message_box_open(QString tips){
  button_num = QMessageBox::question(this,
                                     QString::fromLocal8Bit("机械臂异常！"),
                                     tips + "\n点击Yes:上报异常,No:取消上报",
                                     QMessageBox::Yes, QMessageBox::No);
}

void ChemicalPlatform::Init() { _msgptr->state = "idle"; }

DeviceErrorCode ChemicalPlatform::execute_scripts(StringList scripts) {
  auto res = DEVICE_SUCCESS;
  if (scripts.empty())
    return DEVICE_INVALID_SCRIPT;

  bool haveP = false;
  bool haveC = false;
  bool haveG = false;
  bool haveDP = false;
  bool pgi_online =
      (_pgiptr.get() != nullptr && _pgiptr->_status != DS_UNCONNECTED);
  bool dripper_online = (_dripper_ptr.get() != nullptr &&
                         _dripper_ptr->_status != DS_UNCONNECTED);
  auto _robot_record_data =
      *(std::map<QString, robot_record_data> *)_robot_ptr->getAttribute(
          "_robot_record_data");

  // extract sequence paths
  std::vector<cv::Range> seq_ranges;
  auto range = cv::Range(-1, -1);
  for (int i = 0; i < scripts.size(); i++) {
    auto list = splitstr(scripts[i], ' ');
    if ((list[0] == "J" || list[0] == "P")) {
      if (range.start == -1)
        range.start = i;
      else if (i == scripts.size() - 1) {
        range.end = i + 1;
        if (range.size() > 1)
          seq_ranges.push_back(range);
      }

      if (list[0] == "P")
        haveP = true;
    } else if (((list[0] != "J" && list[0] != "P") ||
                i == scripts.size() - 1) &&
               range.start != -1) {
      range.end = i;
      if (range.size() > 1)
        seq_ranges.push_back(range);
      range = cv::Range(-1, -1);
    }

    if (list[0] == "G")
      haveG = true;
    if (list[0] == "DP")
      haveDP = true;
    if (list[0] == "C")
      haveC = true;
  }

  // check pgi status before pgi move
  if (haveG && !pgi_online && _pgiptr.get() != nullptr &&
      _dripper_ptr.get() != nullptr) {
    ROS_INFO_STREAM(
        "Scripts contains PGI operation,which is offline,get gripper...");
    if (_robot_record_data.count("换手工作站") == 0) {
      ROS_ERROR_STREAM("file:\"换手工作站.json\" not exist");
      return DEVICE_LACK_ATTRIBUTION;
    }

    if (dripper_online) {
      ROS_INFO_STREAM("Dripper is online, set dripper first");
      auto insert_scripts =
          _robot_record_data["换手工作站"]._cmds["setdripper"];
      if (insert_scripts.isEmpty()) {
        ROS_ERROR_STREAM("cmd : \"setdripper\" not exist");
        return DEVICE_LACK_ATTRIBUTION;
      }
      StringList newscripts;
      for (int i = 0; i < insert_scripts.size(); i++)
        newscripts.push_back(insert_scripts[i].toLocal8Bit().toStdString());

      auto res = execute_scripts(newscripts);
      if (res == DEVICE_SUCCESS)
        ROS_INFO_STREAM("Set dripper done");
      else {
        ROS_ERROR_STREAM("Set dripper failed");
        return res;
      }
    }

    auto insert_scripts = _robot_record_data["换手工作站"]._cmds["getgripper"];
    ROS_INFO_STREAM("Get gripper...");
    if (insert_scripts.isEmpty()) {
      ROS_ERROR_STREAM("cmd : \"getgripper\" not exist");
      return DEVICE_LACK_ATTRIBUTION;
    }
    StringList newscripts;
    for (int i = 0; i < insert_scripts.size(); i++)
      newscripts.push_back(insert_scripts[i].toLocal8Bit().toStdString());

    auto res = execute_scripts(newscripts);
    if (res == DEVICE_SUCCESS)
      ROS_INFO_STREAM("Set gripper done");
    else {
      ROS_ERROR_STREAM("Set gripper failed");
      return res;
    }
  }

  if (haveDP && !dripper_online && _pgiptr.get() != nullptr &&
      _dripper_ptr.get() != nullptr) {
    ROS_INFO_STREAM(
        "Scripts contains dripper operation,which is offline,get dripper...");
    if (_robot_record_data.count("换手工作站") == 0) {
      ROS_ERROR_STREAM("file:\"换手工作站.json\" not exist");
      return DEVICE_LACK_ATTRIBUTION;
    }

    if (pgi_online) {
      ROS_INFO_STREAM("Gripper is online, set gripper first");
      auto insert_scripts =
          _robot_record_data["换手工作站"]._cmds["setgripper"];
      if (insert_scripts.isEmpty()) {
        ROS_ERROR_STREAM("cmd : \"setgripper\" not exist");
        return DEVICE_LACK_ATTRIBUTION;
      }
      StringList newscripts;
      for (int i = 0; i < insert_scripts.size(); i++)
        newscripts.push_back(insert_scripts[i].toLocal8Bit().toStdString());

      auto res = execute_scripts(newscripts);
      if (res == DEVICE_SUCCESS)
        ROS_INFO_STREAM("Set gripper done");
      else {
        ROS_ERROR_STREAM("Set gripper failed");
        return res;
      }
    }

    auto insert_scripts = _robot_record_data["换手工作站"]._cmds["getdripper"];
    ROS_INFO_STREAM("Get dripper...");
    if (insert_scripts.isEmpty()) {
      ROS_ERROR_STREAM("cmd : \"getdripper\" not exist");
      return DEVICE_LACK_ATTRIBUTION;
    }
    StringList newscripts;
    for (int i = 0; i < insert_scripts.size(); i++)
      newscripts.push_back(insert_scripts[i].toLocal8Bit().toStdString());

    auto res = execute_scripts(newscripts);
    if (res == DEVICE_SUCCESS)
      ROS_INFO_STREAM("Get dripper done");
    else {
      ROS_ERROR_STREAM("Get dripper failed");
      return res;
    }
  }

  // check relocation before move pose
  if (current_statison_name != last_station_name)
    relocationed = false;
  
  if (!relocationed && (haveP || haveC)) {
    ROS_INFO_STREAM("robot not relocated,do relocation first...");
    auto insert_scripts =
        _robot_record_data[current_statison_name]._cmds["relocation"];
    if (insert_scripts.empty()) {
      ROS_ERROR_STREAM("file: "
                       << current_statison_name.toLocal8Bit().toStdString()
                       << ".jons do not have cmd : \"relocation\"");
      return DEVICE_LACK_ATTRIBUTION;
    }
    StringList newscripts;
    for (int i = 0; i < insert_scripts.size(); i++)
      newscripts.push_back(insert_scripts[i].toLocal8Bit().toStdString());

    auto res = execute_scripts(newscripts);
    if (res == DEVICE_SUCCESS){
      ROS_INFO_STREAM("Relocate done");
      relocationed = true;
    }
    else {
      ROS_ERROR_STREAM("Relocate failed");
      return res;
    }
  }

  // execute origional scripts
  int seq_id = seq_ranges.empty() ? -1 : 0;
  int i = 0;
  for (; i < scripts.size(); i++) {
    auto list = splitstr(scripts[i], ' ');
    // J/P sequence path
    if (seq_id >= 0 && seq_id < seq_ranges.size() &&
        i == seq_ranges[seq_id].start) {
      StringList seq_path;
      int j = i;
      for (; j < seq_ranges[seq_id].end; j++)
        seq_path.push_back(scripts[j]);

      // robot execute seq path
      res = _robot_ptr->execute_scripts(seq_path);
      if (res != DEVICE_SUCCESS)
        break;

      // update path id
      seq_id++;
      i = --j;
    } else // P(single),J(single),C,DO,G,SL,W
    {
      if (list[0] == "P" || list[0] == "J" || list[0] == "C" || list[0] == "DO" || list[0] == "RE")
        res = _robot_ptr->execute_script(scripts[i]);
      else if (list[0] == "G") {
        if (_pgiptr.get() != nullptr){
          auto robot_point_type = _robot_record_data[current_statison_name]._cmd_to_pointname[current_operation_name].first;
          auto station_point_type = _robot_record_data[current_statison_name]._cmd_to_pointname[current_operation_name].second;

          Json::Value json_msg;
          std_msgs::String msg;
          if(robot_point_type.contains("rack"))
            json_msg["gripped_target"] = "rack";
          else if(robot_point_type.contains("hole"))
            json_msg["gripped_target"] = "bottle";

          msg.data = json_msg.toStyledString();
          if(_pub5_.getNumSubscribers() > 0)
            _pub5_.publish(msg);

          int count = 0;
          res = _pgiptr->execute_script(scripts[i]);
          while(res != DEVICE_SUCCESS && count++ < 2){
            ROS_INFO_STREAM("PGI execute failed,retry...");
            ros::Duration(1).sleep();
            res = _pgiptr->execute_script(scripts[i]);
          }
        }
        else
          res = DEVICE_SUCCESS;
      } else if (list[0] == "DP") {
        if (_dripper_ptr.get() != nullptr){
          int count = 0;
          res = _dripper_ptr->execute_script(scripts[i]);
          while(res != DEVICE_SUCCESS && count++ < 2){
            ROS_INFO_STREAM("Dripper execute failed,retry...");
            ros::Duration(1).sleep();
            res = _dripper_ptr->execute_script(scripts[i]);
          }
        }
        else
          res = DEVICE_SUCCESS;
      } else if (list[0] == "SL") {
        if (_screwslide_ptr.get() != nullptr)
          res = _screwslide_ptr->execute_script(scripts[i]);
        else
          res = DEVICE_SUCCESS;
      } else if (list[0] == "W") {
        assert(list.size() == 2);
        auto delay = std::stod(list[1]);
        ros::Duration(delay).sleep();
      }
    }

    // check execute result
    if (res != DEVICE_SUCCESS)
      break;
  }

  if (res != DEVICE_SUCCESS)
    ROS_ERROR_STREAM("Script Sequence Excute failed at : [" +
                     std::to_string(i) + "]: " + scripts[i]);

  return res;
}

void ChemicalPlatform::JsonPubDone() {
  _msgptr->state = "done";
  JsonPub();
  _msgptr->state = "idle";
}

void ChemicalPlatform::JsonPubErr(std::string detail, bool need_reboot,
                                  bool not_pub) {
  _msgptr->state = "error";
  _msgptr->detail = detail;

  if (!not_pub)
    JsonPub();

  // if(!need_reboot)
  //   _msgptr->state = "idle";
}

void ChemicalPlatform::JsonPub() {
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
  while(_pub1_.getNumSubscribers() == 0)
  {
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("No subscriber for JsonPub(),waiting ...");
  }
  _pub1_.publish(pubmsgs);
}

void ChemicalPlatform::JointsPub(const ros::TimerEvent& event){
  auto _mgptr = *(moveit::planning_interface::MoveGroupInterfacePtr *)
                      _robot_ptr->getAttribute("_mgptr");
  auto robot_joints = _mgptr->getCurrentJointValues();
  if(robot_joints.empty())
    return;

  char buffer[128] = {0};
  sprintf(buffer, "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",
          180 * robot_joints[0] / M_PI, 180 * robot_joints[1] / M_PI,
          180 * robot_joints[2] / M_PI, 180 * robot_joints[3] / M_PI,
          180 * robot_joints[4] / M_PI, 180 * robot_joints[5] / M_PI);

  Json::Value root;
  root["time_stamp"] = std::to_string(
    std::chrono::duration_cast<std::chrono::milliseconds>
    (std::chrono::system_clock::now().time_since_epoch()).count());
  root["joints"] = buffer;
  std_msgs::String pubmsgs;
  pubmsgs.data = root.toStyledString();
  _pub4_.publish(pubmsgs);
}

