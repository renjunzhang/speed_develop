#include "devices/Gripper/pgi_control.h"
#include <std_msgs/String.h>
#include <jsoncpp/json/json.h>
bool pgi_initialized = false;
static std::atomic<int> pgi_dis;
int active_device_id = 0;
std::mutex serial_mutex;

pgi::pgi(ros::NodeHandle& nh)
{
  _attribute.clear();
  setAttribute("device_name",(void*)"/dev/PGI");
  setAttribute("baudrate",(void*)"115200");
  _pub1_ = nh.advertise<std_msgs::String>("/gripped_status", 1);
}

pgi::~pgi()
{
  theadstop.store(true);
}

DeviceErrorCode pgi::init(void*){
  //init all member variables
  _status = DS_UNCONNECTED;
  _script_history.clear();
  _current_scripts.clear();
  _curr_point = _check_point = _current_scripts.begin();
  theadstop.store(false);
  pgi_dis.store(-1);

  //get serial params
  auto devicename = *(std::string*)getAttribute("device_name");
  auto baudrate_str = *(std::string*)getAttribute("baudrate");
  if(devicename.empty() || baudrate_str.empty()){
    ROS_ERROR_STREAM(std::string("Pgi device lack attributions, current settings are:\"device_name\":\"") +
    devicename + "\",\"baudrate\":\"" + baudrate_str + "\"");
    return DEVICE_LACK_ATTRIBUTION;
  }
  dev = devicename;
  baudrate = 0;
  try{
    baudrate = std::stoi(baudrate_str);
  }
  catch(std::invalid_argument e)
  {
    ROS_ERROR_STREAM(std::string("Invalid baudrate attribution setting:[") + baudrate_str + "], ignored");
    return DEVICE_INVALID_ATTRIBUTION;
  }
  ROS_INFO_STREAM(std::string("Current device name: ") + dev + ",baudrate: " + baudrate_str);
  _rtu = std::make_shared<modbusrtu>(dev,baudrate);
  // check if serial alive in a thread loop, if not, reconnect it every 5 seconds
  if (_thread_check_serial_ptr == nullptr)
  {
    ROS_INFO_STREAM("Start PGI alive check thread...");
    _thread_check_serial_ptr = new std::thread([&](){
      while(!theadstop.load()){
        try {
          if (active_device_id == 0) 
          {
            if (!pgi_initialized)
            {
              connect();
              initSingle();
              //setSpeed(SPEED);
              //setForce(FORCE);
              pgi_initialized = true;
              _status = DS_IDEL;
            }
            else
            {
              pgi_dis.store(getPos());
              std::this_thread::sleep_for(std::chrono::milliseconds(200));
              continue;
            }
          }
        }
        catch (std::string e)
        {
          ROS_INFO_STREAM("PGI disconnected,switch to dripper connection");          
          pgi_initialized = false;
          pgi_dis.store(-1);
          _status = DS_UNCONNECTED;
          active_device_id = 1;
        }
      std::this_thread::sleep_for(std::chrono::seconds(2));
      } });
    _thread_check_serial_ptr->detach();
  }
  return DEVICE_SUCCESS;
}

DeviceErrorCode pgi::connect(){
  
  if(!_rtu->connect())
    return DEVICE_CONNECT_FAILED;

  try {
    setSpeed(SPEED);
    setForce(FORCE);
    _status = DS_IDEL;
    return DEVICE_SUCCESS;
  } catch (std::string e) {
    ROS_ERROR_STREAM("pgi communication failed");
    _status = DS_FAULT;
    return DEVICE_COMMUNICATION_ERROR;
  }
}

DeviceErrorCode pgi::execute_script(std::string script){
  ROS_INFO_STREAM("scripts size: 1");
  ROS_INFO_STREAM("script    : " + script);

  auto res = splitstr(script,' ');
  assert(res[0] == "G" && res.size() == 6);

  if(_status == DS_RUNNING)
    return DEVICE_ISRUNNIN;
  else if(_status == DS_UNCONNECTED)
    return DEVICE_CONNECT_FAILED;
  else if(_status == DS_FAULT)
    return DEVICE_IS_FAULT;
  
  try{
    auto position = std::stoi(res[1]);
    auto check = (res[2] == "true" ? true : false);
    auto block = (res[3] == "true" ? true : false);
    auto force = std::stoi(res[4]);
    auto speed = std::stoi(res[5]);
    _status = DS_RUNNING;
    setall(position,block,force,speed);
    auto gripped = isGrip();

    Json::Value json_msg;
    json_msg["gripped_status"] = gripped ? "gripped":"disgripped";
    std_msgs::String msg;
    msg.data = json_msg.toStyledString();
    if(_pub1_.getNumSubscribers() > 0)
      _pub1_.publish(msg);
    if (check) {
      if (!gripped) {
        ROS_ERROR_STREAM("没有夹到物体");
        return DEVICE_PGI_UNGRIPED;
      }
    }
    _status = DS_IDEL;
  }
  catch(std::invalid_argument e){
    ROS_ERROR_STREAM((std::string("PGI execute script[") + script + "] failed,error message:") + e.what());
    _status = DS_IDEL;
    return DEVICE_INVALID_SCRIPT;
  }
  catch (std::string e) {
    ROS_ERROR_STREAM(std::string("PGI execute script[") + script + "] failed,error message:" + e);
    _status = DS_FAULT;
    return DEVICE_IS_FAULT;
  }
  return DEVICE_SUCCESS;
}

DeviceErrorCode pgi::execute_scripts(std::vector<std::string> scripts)
{
  DeviceErrorCode code = DEVICE_SUCCESS;
  _current_scripts.swap(scripts);
	_check_point = _current_scripts.end();
  for (int i = 0; i < _current_scripts.size(); i++){
    _curr_point = _current_scripts.begin() + i;
    code = execute_script(*_curr_point);
    if(code!=DEVICE_SUCCESS)
      break;
    _script_history.push_back(_current_scripts[i]);
    _check_point = _curr_point;
  }
  return code;
}

DeviceErrorCode pgi::back_to_checkpoint(){
  ROS_INFO_STREAM("Go back to PGI check point ...");
  auto backward_script = std::vector<std::string>(_current_scripts.begin(), _check_point);
  std::reverse(backward_script.begin(),backward_script.end());
  auto res = execute_scripts(backward_script);
  if(res != DEVICE_SUCCESS)
    ROS_ERROR_STREAM("Go back to PGI check point failed");
  else
    ROS_INFO_STREAM("Go back to PGI check point ok");
  return res;
}

DeviceErrorCode pgi::getdata(std::string dataname,std::string& data){
  data.clear();
  try{
    if(dataname == "getPos"){
      data = std::to_string(pgi_dis.load());
    }
    else if(dataname == "getCurrent"){
      auto current = getCurrent();
      data = std::to_string(current);
    }
    else
      return DEVICE_INVALID_ATTRIBUTION;
  }
  catch (std::string e){
    ROS_ERROR_STREAM("PGI "+ dataname + " failed,error message:" + e);
    _status = DS_FAULT;
    return DEVICE_IS_FAULT;
  }
  return DEVICE_SUCCESS;
}

void pgi::initSingle() {
  uint8_t out_buf[8] = {0x01, 0x06, 0x01, 0x00, 0x00, 0x01};
  uint8_t in_buf[8];
  setcommand(out_buf, in_buf, "initSingle");
  std::cout << "gripper is initializing" << std::endl;
  while (!isInit())
    sleep(0.5);
  std::cout << "gripper init succeed" << std::endl;
}

void pgi::initDouble() {
  uint8_t out_buf[8] = {0x01, 0x06, 0x01, 0x00, 0x00, 0xA5};
  uint8_t in_buf[8];
  setcommand(out_buf, in_buf, "initDouble");
  std::cout << "gripper is initializing" << std::endl;
  while (!isInit())
    sleep(0.5);
  std::cout << "gripper init succeed" << std::endl;
}

void pgi::setall(uint16_t position, bool block, uint16_t force,
                 uint16_t speed) {
  if (speed != SPEED) {
    setSpeed(speed);
  }
  if (force != FORCE) {
    setForce(force);
  }
  setPos(position);
  if (block) {
    while (isBusy())
      sleep(0.5);
  }
}

void pgi::setPos(uint16_t command) {
  uint8_t out_buf[8] = {
      0x01, 0x06, 0x01, 0x03, uint8_t(command >> 8), uint8_t(command & 0xFF)};
  uint8_t in_buf[8];
  setcommand(out_buf, in_buf, "setPos");
}

void pgi::setForce(uint16_t command) {
  uint8_t out_buf[8] = {
      0x01, 0x06, 0x01, 0x01, uint8_t(command >> 8), uint8_t(command & 0xFF)};
  uint8_t in_buf[8];
  setcommand(out_buf, in_buf, "setForce");
}

void pgi::setSpeed(uint16_t command) {
  uint8_t out_buf[8] = {
      0x01, 0x06, 0x04, 0x01, uint8_t(command >> 8), uint8_t(command & 0xFF)};
  uint8_t in_buf[8];
  setcommand(out_buf, in_buf, "setSpeed");
}

bool pgi::isInit() {
  uint8_t out_buf[8] = {0x01, 0x03, 0x02, 0x00, 0x00, 0x01};
  uint8_t in_buf[8];
  readcommand(out_buf, in_buf, "isInit");
  return (in_buf[4] == 0x01);
}

bool pgi::isBusy() {
  uint8_t out_buf[8] = {0x01, 0x03, 0x02, 0x01, 0x00, 0x01};
  uint8_t in_buf[8];
  readcommand(out_buf, in_buf, "isBusy");
  return (in_buf[4] == 0x00);
}

bool pgi::isGrip() {
  uint8_t out_buf[8] = {0x01, 0x03, 0x02, 0x01, 0x00, 0x01};
  uint8_t in_buf[8];
  readcommand(out_buf, in_buf, "isGrip");
  std::cout << "isGrip: " << std::hex << (in_buf[4] & 0xff) << std::endl;
  return (in_buf[4] == 0x02);
}

bool pgi::isGripCur() {
  uint16_t current = getCurrent();
  // std::cout << "isGripCur: " << current <<std::endl;
  if (current > CURRENTLOW && current < CURRENTHIGH) {
    return true;
  }
  return false;
}

bool pgi::isDrop() {
  uint8_t out_buf[8] = {0x01, 0x03, 0x02, 0x01, 0x00, 0x01};
  uint8_t in_buf[8];
  size_t count = 0;
  for (size_t i = 0; i < 3; ++i) {
    readcommand(out_buf, in_buf, "isDrop");
    // std::cout << "isDrop: " << std::hex << (in_buf[4] & 0xff) <<std::endl;
    if (in_buf[4] != 0x03) {
      return false;
    }
  }
  return true;
}

bool pgi::isDropCur() {
  for (size_t i = 0; i < 3; ++i) {
    if (!isGripCur())
      return true;
  }
  return false;
}

uint16_t pgi::getPos() {
  uint8_t out_buf[8] = {0x01, 0x03, 0x02, 0x02, 0x00, 0x01};
  uint8_t in_buf[8];
  readcommand(out_buf, in_buf, "getPos");
  return ((in_buf[3] << 8) + (in_buf[4] & 0xFF));
}

uint16_t pgi::getCurrent() {
  uint8_t out_buf[8] = {0x01, 0x03, 0x02, 0x04, 0x00, 0x01};
  uint8_t in_buf[8];
  readcommand(out_buf, in_buf, "getCurrent");
  return ((in_buf[3] << 8) + (in_buf[4] & 0xFF));
}

void pgi::setcommand(uint8_t *out_buf, uint8_t *in_buf, std::string command) {
  std::unique_lock<std::mutex> lk(serial_mutex);
  int errorcount = 0;
ss:
  if (!_rtu->write(out_buf, 6)) {
    //ROS_ERROR_STREAM(command << " write failed!");
    errorcount++;
  }

  if (!_rtu->read(in_buf, 8)) {
    //ROS_ERROR_STREAM(command << " read failed!");
    errorcount++;
  }
  else{
    if(errorcount > 0)
      ROS_ERROR_STREAM(command << " read failed recorved!");
    errorcount = 0;
  }
  
  if(errorcount > 0){
    if(errorcount < 6)
      goto ss;
    else
      throw std::string(command + " failed!");
  }
}

void pgi::readcommand(uint8_t *out_buf, uint8_t *in_buf, std::string command) {
  std::unique_lock<std::mutex> lk(serial_mutex);
  int errorcount = 0;
ss:
  if (!_rtu->write(out_buf, 6)) {
    ROS_ERROR_STREAM(command << " write failed!");
    errorcount++;
  }

  if (!_rtu->read(in_buf, 7)) {
    ROS_ERROR_STREAM(command << " read failed!");
    errorcount++;
  }
  else{
    errorcount = 0;
  }
  
  if(errorcount > 0){
    if(errorcount < 6)
      goto ss;
    else
      throw std::string(command + " failed!");
  }
}
