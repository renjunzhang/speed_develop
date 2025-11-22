#include <chemicalrobot_arm/dashboard.h>

dashboardsrv_client::dashboardsrv_client(ros::NodeHandle nh)
{
  nh_ = nh;
  bool no_serial;
  std::string soft_gripper;
  srv_client_ = nh_.serviceClient<ur_dashboard_msgs::RawRequest>("/ur_hardware_interface/dashboard/raw_request");
  load_client_ = nh_.serviceClient<ur_dashboard_msgs::Load>("/ur_hardware_interface/dashboard/load_program");
  DO_client_ = nh_.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
  program_client_ = nh_.serviceClient<ur_dashboard_msgs::GetProgramState>("/ur_hardware_interface/dashboard/program_state");
  safety_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/unlock_protective_stop");
  nh_.param<bool>("/chemicalrobot_arm/no_serial", no_serial, true);
  nh_.param<std::string>("/chemicalrobot_arm/soft_gripper", soft_gripper, "/dev/GRIPPER");
  if (no_serial)
    ;
  else
  {
    try
    {
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      ros_ser.setPort(soft_gripper);
      ros_ser.setBaudrate(9600);
      ros_ser.setTimeout(timeout);
      if(!ros_ser.isOpen())   ros_ser.open();
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR_STREAM("Unable to open port GRIPPER: " << e.what());
    }
  }
}

void dashboardsrv_client::DO_init()
{
  setDO(0, false);
  setDO(1, false);
  setDO(2, false);
  setDO(3, false);
}

void dashboardsrv_client::play()
{
  ur_dashboard_msgs::RawRequest play;
  play.request.query = "play\n";
  size_t i = 0;
  for (; i < 5; ++i)
  {
    if (!ros::topic::waitForMessage<std_msgs::Bool>("/ur_hardware_interface/robot_program_running")->data)
    {
      ros::Duration(0.5).sleep();
      srv_client_.call(play);
    }
    else
      return;
  }
  ROS_WARN_STREAM("尝试启动remote_control程序失败，请手动操作示教板");
}

void dashboardsrv_client::stop()
{
  ur_dashboard_msgs::RawRequest stop;
  stop.request.query = "stop\n";
  for (size_t i = 0; i < 5; ++i)
  {
    if (ros::topic::waitForMessage<std_msgs::Bool>("/ur_hardware_interface/robot_program_running")->data)
    {
      ros::Duration(0.5).sleep();
      srv_client_.call(stop);
    }
    else
      return;
  }
  ROS_WARN_STREAM("尝试停止remote_control程序失败，请手动操作示教板");
}

void dashboardsrv_client::unlockPS()
{
  std_srvs::Trigger unlock;
  ros::Duration(5).sleep();
  size_t num = 10;
  for (size_t i = 0; i < num; ++i)
  {
    if (ros::topic::waitForMessage<ur_dashboard_msgs::SafetyMode>("/ur_hardware_interface/safety_mode")->mode == ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP)
    {
      ros::Duration(1).sleep();
      safety_client_.call(unlock);
    }
    else
      return;
  }
  ROS_WARN_STREAM("尝试解除保护性停止失败，请手动操作示教板");
}

void dashboardsrv_client::load_program(const string filename)
{
  ROS_INFO_STREAM("等待" + filename.substr(0, filename.size() - 1) + "加载完成");
  auto programState = ros::topic::waitForMessage<std_msgs::Bool>("/ur_hardware_interface/robot_program_running");
  if (!programState->data)
  {
    // this->stop();
    ur_dashboard_msgs::Load load;
    load.request.filename = string(filename);
    load_client_.call(load);
    play();
    while (!ros::topic::waitForMessage<std_msgs::Bool>("/ur_hardware_interface/robot_program_running")->data)
    {
      ros::Duration(1).sleep();
    }
  }
}

void dashboardsrv_client::setDO(int8_t pin, bool state)
{
  ur_msgs::SetIO srv;
  srv.request.fun = srv.request.FUN_SET_DIGITAL_OUT;
  srv.request.pin = pin;
  srv.request.state = state ? srv.request.STATE_ON : srv.request.STATE_OFF;
  DO_client_.call(srv);
}

void dashboardsrv_client::robot_init()
{
  ur_dashboard_msgs::RawRequest init;
  ur_dashboard_msgs::RobotModeConstPtr robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>("/ur_hardware_interface/robot_mode");
  if (robotmode->mode == robotmode->RUNNING)
  {
    ROS_INFO_STREAM("Robot is RUNNING\n");
    return;
  }
  ROS_INFO_STREAM("Robot is Initializing\n");
  init.request.query = "power on\n";
  srv_client_.call(init);
  init.request.query = "brake release\n";
  srv_client_.call(init);
  while (robotmode->mode != robotmode->RUNNING)
  {
    robotmode = ros::topic::waitForMessage<ur_dashboard_msgs::RobotMode>("/ur_hardware_interface/robot_mode");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO_STREAM("Robot is RUNNING\n");
}

bool dashboardsrv_client::RG2_is_busy()
{
  auto DIstate = ros::topic::waitForMessage<ur_msgs::IOStates>("/ur_hardware_interface/io_states");
  if (DIstate->digital_in_states[0].state)
    return true;
  return false;
}

bool dashboardsrv_client::RG2_is_grip()
{
  auto DIstate = ros::topic::waitForMessage<ur_msgs::IOStates>("/ur_hardware_interface/io_states");
  if (DIstate->digital_in_states[1].state)
    return true;
  return false;
}

void dashboardsrv_client::RG2grip()
{
  setDO(0, true);
  setDO(1, false);
  setDO(2, false);
  while (RG2_is_busy())
    ;
}

void dashboardsrv_client::RG2release()
{
  setDO(0, false);
  setDO(1, false);
  setDO(2, false);
  while (RG2_is_busy())
    ;
}

// TODO：气动夹爪迁移
int dashboardsrv_client::close()
{
  int flag = 0;
  static uint8_t checksum;
  static uint8_t s1_buffer[100];
  static uint8_t l, m, n;
  s1_buffer[0] = 0xff;
  s1_buffer[1] = 0xff;
  s1_buffer[2] = 0x01;
  s1_buffer[3] = 0X09;
  s1_buffer[4] = 0X03;
  s1_buffer[5] = 0x2A;
  s1_buffer[6] = 0x8b;
  s1_buffer[7] = 0x0c;
  s1_buffer[8] = 0x00;
  s1_buffer[9] = 0x00;
  s1_buffer[10] = 0xE8;
  s1_buffer[11] = 0x01;
  checksum = s1_buffer[2] + s1_buffer[3] + s1_buffer[4];
  n = s1_buffer[3] - 2;
  for (l = 0; l < n; l++)
  {
    m = 5 + l;
    checksum += s1_buffer[m];
  }
  checksum = ~checksum;
  s1_buffer[m + 1] = checksum;
  // for(int p=0; p<n+6; p++)
  // {
  //     std::cout << std::hex << (s1_buffer[p] & 0xff) << " ";
  // }
  while (!flag)
  {
    ros_ser.write(s1_buffer, 13);
    sleep(1);
    flag = ros_ser.available();
  }
  ros_ser.flushInput();
  // ROS_INFO("flag=%d\nserial sends data success\n",flag);
  return (flag);
}

int dashboardsrv_client::open()
{
  int flag = 0;
  static uint8_t checksum;
  static uint8_t s_buffer[100];
  static uint8_t i, j, k;
  s_buffer[0] = 0xff;
  s_buffer[1] = 0xff;
  s_buffer[2] = 0x01;
  s_buffer[3] = 0X09;
  s_buffer[4] = 0X03;
  s_buffer[5] = 0x2A;
  s_buffer[6] = 0xff;
  s_buffer[7] = 0x11;
  s_buffer[8] = 0x00;
  s_buffer[9] = 0x00;
  s_buffer[10] = 0xE8;
  s_buffer[11] = 0x01;
  checksum = s_buffer[2] + s_buffer[3] + s_buffer[4];
  k = s_buffer[3] - 2;
  for (i = 0; i < k; i++)
  {
    j = 5 + i;
    checksum += s_buffer[j];
  }
  checksum = ~checksum;
  s_buffer[j + 1] = checksum;
  // for(int h=0; h<k+6; h++)
  // {
  //     std::cout << std::hex << (s_buffer[h] & 0xff) << " ";
  // }
  while (!flag)
  {
    ros_ser.write(s_buffer, 13);
    sleep(1);
    flag = ros_ser.available();
  }
  ros_ser.flushInput();
  // ROS_INFO("flag=%d\nserial sends data success\n",flag);
  return (flag);
}

int dashboardsrv_client::grasp()
{
  int flag = 0;
  static uint8_t s1_buffer[100], s2_buffer[100];
  s1_buffer[0] = 0x01;
  s1_buffer[1] = 0x05;
  s1_buffer[2] = 0x00;
  s1_buffer[3] = 0X05;
  s1_buffer[4] = 0Xff;
  s1_buffer[5] = 0x00;
  s1_buffer[6] = 0x9c;
  s1_buffer[7] = 0x3b;
  // for(int p=0; p<8; p++)
  // {
  //     std::cout << std::hex << (s1_buffer[p] & 0xff) << " ";
  // }
  while (!flag)
  {
    ros_ser.write(s1_buffer, 8);
    sleep(1);
    flag = ros_ser.available();
  }
  ros_ser.read(s2_buffer, flag);
  // for(int p=0; p<8; p++)
  // {
  //     std::cout << std::hex << (s2_buffer[p] & 0xff) << " ";
  // }
  ros_ser.flushInput();
  // ROS_INFO("flag=%d\nserial sends data success\n",flag);
  return (flag);
}

int dashboardsrv_client::loose()
{
  int p;
  int flag = 0;
  static uint8_t s_buffer[100];
  s_buffer[0] = 0x01;
  s_buffer[1] = 0x05;
  s_buffer[2] = 0x00;
  s_buffer[3] = 0X04;
  s_buffer[4] = 0Xff;
  s_buffer[5] = 0x00;
  s_buffer[6] = 0xcd;
  s_buffer[7] = 0xfb;

  while (!flag)
  {
    ros_ser.flushOutput();
    ros_ser.write(s_buffer, 8);
    sleep(1);
    flag = ros_ser.available();
    // ROS_INFO("flag=%d\n",flag);
  }
  ros_ser.flushInput();
  // ROS_INFO("flag=%d\nserial sends data success\n",flag);
  return (flag);
}

int dashboardsrv_client::reset()
{
  int p;
  int flag = 0;
  static uint8_t s1_buffer[100], s2_buffer[100];
  s1_buffer[0] = 0x01;
  s1_buffer[1] = 0x05;
  s1_buffer[2] = 0x00;
  s1_buffer[3] = 0X05;
  s1_buffer[4] = 0X00;
  s1_buffer[5] = 0x00;
  s1_buffer[6] = 0xdd;
  s1_buffer[7] = 0xcb;

  s2_buffer[0] = 0x01;
  s2_buffer[1] = 0x05;
  s2_buffer[2] = 0x00;
  s2_buffer[3] = 0X04;
  s2_buffer[4] = 0X00;
  s2_buffer[5] = 0x00;
  s2_buffer[6] = 0x8c;
  s2_buffer[7] = 0x0b;

  while (!flag)
  {
    ros_ser.flushOutput();
    ros_ser.write(s1_buffer, 8);
    sleep(1);
    flag = ros_ser.available();
    // ROS_INFO("flag=%d\n",flag);
  }
  ros_ser.flushInput();
  // ROS_INFO("flag=%d\nserial sends data success\n",flag);

  flag = 0;
  while (!flag)
  {
    ros_ser.flushOutput();
    ros_ser.write(s2_buffer, 8);
    sleep(1);
    flag = ros_ser.available();
    // ROS_INFO("flag=%d\n",flag);
  }
  ros_ser.flushInput();
  // ROS_INFO("flag=%d\nserial sends data success\n",flag);
  return (flag);
}