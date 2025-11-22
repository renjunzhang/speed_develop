#include "devices/Dripper/dripper_control.h"
bool dripper_initialized = false;
std::atomic_bool serial_using;
extern std::mutex serial_mutex;
extern int active_device_id;

DripperControl::DripperControl() {}

DripperControl::~DripperControl() { theadstop.store(true); }

DeviceErrorCode DripperControl::init(void *) {
  // init all member variables
  _status = DS_UNCONNECTED;
  _script_history.clear();
  _current_scripts.clear();
  _curr_point = _check_point = _current_scripts.begin();
  theadstop.store(false);
  serial_using.store(false);

  // get serial params
  auto devicename = *(std::string *)getAttribute("device_name");
  if (devicename.empty()) {
    ROS_ERROR_STREAM(
        "Dripper device lack attributions, current settings "
        "are:\"device_name\":\""
        << devicename << "\"");
    return DEVICE_LACK_ATTRIBUTION;
  }
  _dev = devicename;
  ROS_INFO_STREAM("Current device name: " << _dev);

  // serial setting
  serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
  _ser.setPort(_dev);
  _ser.setBaudrate(115200);
  _ser.setParity(serial::parity_none);
  _ser.setStopbits(serial::stopbits_one);
  _ser.setTimeout(timeout);

  // check if serial alive in a thread loop, if not, reconnect it every 5
  // seconds
  if (_thread_check_serial_ptr == nullptr) {
    int error_count = 0;
    ROS_INFO_STREAM("Start Dripper alive check thread...");
    _thread_check_serial_ptr = new std::thread([&]() {
      while (!theadstop.load()) {
        if (active_device_id == 1) {
          if (!dripper_initialized) {
            if (connect() == DEVICE_SUCCESS && reset()) {
              dripper_initialized = true;
              _status = DS_IDEL;
              ROS_INFO_STREAM("Dripper connected succeed");
            } else {
              error_count++;
              if (error_count > 1) {
                ROS_INFO_STREAM("Dripper disconnected,switch to PGI connection");
                error_count = 0;
                dripper_initialized = false;
                _status = DS_UNCONNECTED;
                active_device_id = 0;
              }
            }
          } else {
            while (serial_using.load())
              std::this_thread::sleep_for(std::chrono::milliseconds(1));

            serial_using.store(true);
            // heartbeat
            if (check("") > DRIPPER_DONE)
              error_count++;
            else {
              error_count = 0;
              dripper_initialized = true;
            }
            serial_using.store(false);

            if (error_count > 1) {
              ROS_INFO_STREAM("Dripper disconnected,switch to PGI connection");
              error_count = 0;
              dripper_initialized = false;
              _status = DS_UNCONNECTED;
              active_device_id = 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
          }
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
      reset();
      _ser.close();
    });
    _thread_check_serial_ptr->detach();
  }
  return DEVICE_SUCCESS;
}

DeviceErrorCode DripperControl::connect() {
  _ser.close();
  try {
    _ser.open();
  } catch (serial::IOException e) {
    ROS_ERROR_STREAM(e.what());
    return DEVICE_CONNECT_FAILED;
  }
  return DEVICE_SUCCESS;
}

DeviceErrorCode DripperControl::execute_script(std::string script) {
  ROS_INFO_STREAM("scripts size: 1");
  ROS_INFO_STREAM("script    : " + script);

  auto res = splitstr(script, ' ');
  assert(res[0] == "DP" && res.size() == 2);

  if (_status == DS_RUNNING)
    return DEVICE_ISRUNNIN;
  else if (_status == DS_UNCONNECTED)
    return DEVICE_CONNECT_FAILED;
  else if (_status == DS_FAULT)
    return DEVICE_IS_FAULT;

  try {
    _status = DS_RUNNING;
    auto fuc = fuc_map[res[1]];
    bool succeed = false;
    if (fuc != nullptr) succeed = (this->*fuc)();
    _status = DS_IDEL;
    if (!succeed) return DEVICE_EXECUTE_FAILED;
  } catch (std::string e) {
    ROS_ERROR_STREAM(std::string("Dripper execute script[") + script +
                     "] failed,error message:" + e);
    _status = DS_FAULT;
    return DEVICE_IS_FAULT;
  }
  return DEVICE_SUCCESS;
}

DeviceErrorCode DripperControl::execute_scripts(
    std::vector<std::string> scripts) {
  DeviceErrorCode code = DEVICE_SUCCESS;
  _current_scripts.swap(scripts);
  _check_point = _current_scripts.end();
  for (int i = 0; i < _current_scripts.size(); i++) {
    _curr_point = _current_scripts.begin() + i;
    code = execute_script(*_curr_point);
    if (code != DEVICE_SUCCESS) break;
    _script_history.push_back(_current_scripts[i]);
    _check_point = _curr_point;
  }
  return code;
}

DeviceErrorCode DripperControl::back_to_checkpoint() {
  ROS_INFO_STREAM("Go back to Dripper check point ...");
  auto backward_script =
      std::vector<std::string>(_current_scripts.begin(), _check_point);
  std::reverse(backward_script.begin(), backward_script.end());
  auto res = execute_scripts(backward_script);
  if (res != DEVICE_SUCCESS)
    ROS_ERROR_STREAM("Go back to Dripper check point failed");
  else
    ROS_INFO_STREAM("Go back to Dripper check point ok");
  return res;
}

DeviceErrorCode DripperControl::getdata(std::string dataname,
                                        std::string &data) {
  data.clear();
  return DEVICE_SUCCESS;
}

bool DripperControl::ready_to_suck() { return send_commond_retry("zb_xiye"); }

bool DripperControl::suck() { return send_commond_retry("xiye"); }

bool DripperControl::drip() { return send_commond_retry("paiye"); }

bool DripperControl::change_tool() { return send_commond_retry("huantou"); }

bool DripperControl::reset() { return send_commond_retry("opt_over"); }

bool DripperControl::send_commond_retry(std::string cmd){
  int count = 0;
  while(!send_commond(cmd)){
    count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if(count > 2)
      return false;
  }
  return true;
}

bool DripperControl::send_commond(std::string cmd) {
  while (serial_using.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

  serial_using.store(true);
  std::string resp = "";
  {
    std::unique_lock<std::mutex> lk(serial_mutex);
    auto res = serial_write(cmd + "\r\n");
    if (!res) {
      serial_using.store(false);
      return false;
    }
    // ROS_INFO_STREAM("Dripper send:" << cmd << "\r\n");

    res = serial_read(resp);
    if (!res || resp.empty()) {
      serial_using.store(false);
      return false;
    }
    // ROS_INFO_STREAM("Dripper received:" << resp);
  }

  // check ack
  if (resp.find("beg") == std::string::npos) {
    ROS_ERROR_STREAM(
        "Dripper send commond: " << cmd << " failed, ack response: " << resp);
    serial_using.store(false);
    return false;
  }
  // sync
  auto res = wait_for_processing(cmd);
  serial_using.store(false);
  return res;
}

bool DripperControl::serial_write(std::string data) {
  try {
    _ser.flush();
    if (_ser.write(data) < data.size()) {
      // std::cout << _dev << " write buffer length error" << std::endl;
      ROS_ERROR_STREAM(
          "Dripper control write buffer length error, writed data: " + data);
      _ser.flushOutput();
      return false;
    }
    return true;
  } catch (serial::PortNotOpenedException e) {
    ROS_ERROR_STREAM("Dripper control write buffer exception: " << e.what());
  } catch (serial::SerialException e) {
    ROS_ERROR_STREAM("Dripper control write buffer exception: " << e.what());
  } catch (serial::IOException e) {
    ROS_ERROR_STREAM("Dripper control write buffer exception: " << e.what());
  }
  return false;
}

bool DripperControl::serial_read(std::string &data) {
  data.clear();
  std::vector<std::string> res;
  try {
    if (!_ser.waitReadable()) return false;
    auto num = _ser.available();
    std::string buffer;
    _ser.read(buffer, num);

    while (*(buffer.end() - 2) != '\r' && buffer.back() != '\n') {
      if (!_ser.waitReadable()) return false;

      num = _ser.available();
      std::string temp;
      _ser.read(temp, num);
      buffer += temp;
    }
    data = buffer;
  } catch (serial::PortNotOpenedException e) {
    ROS_ERROR_STREAM("Dripper control read buffer exception: " << e.what());
    return false;
  } catch (serial::SerialException e) {
    ROS_ERROR_STREAM("Dripper control read buffer exception: " << e.what());
    return false;
  }
  return true;
}

int DripperControl::check(std::string cmd) {
  std::string resp = "";
  {
    std::unique_lock<std::mutex> lk(serial_mutex);
    auto res = serial_write("check\r\n");
    if (!res){
      ROS_ERROR_STREAM("Dripper check commond serial write failed");
      return DRIPPER_COMMUNICATE_ERROR;
    }
      
    res = serial_read(resp);
    if (!res || resp.empty()) {
      ROS_ERROR_STREAM("Dripper check commond serial read failed");
      return DRIPPER_COMMUNICATE_ERROR;
    }
    // ROS_INFO_STREAM("Dripper received:" << resp);
  }
  if (!cmd.empty() && resp.find(cmd) == std::string::npos) {
    std::string resp2;
    auto res = serial_read(resp2);
    if (!res || resp2.empty()){
      ROS_ERROR_STREAM("Dripper check commond serial read failed");
      return DRIPPER_COMMUNICATE_ERROR; 
    }

    resp += resp2;
    if (resp.find(cmd) == std::string::npos) {
      ROS_ERROR_STREAM("Dripper check commond: " << cmd << " failed, ack response: ["
                                         << resp << "]");
      return DRIPPER_WRONG_CMD;
    }
  }

  if (resp.find("beg") != std::string::npos)
    return DRIPPER_PREPROCESS;
  else if (resp.find("busy") != std::string::npos)
    return DRIPPER_PROCESSING;
  else if (resp.find("over") != std::string::npos)
    return DRIPPER_DONE;
  else if (resp.find("error") != std::string::npos)
    return DRIPPER_ERROR;
  else {
    ROS_ERROR_STREAM("Dripper check response error,row data: " << resp);
    return DRIPPER_UNKNOWN_ERROR;
  }
}

bool DripperControl::wait_for_processing(std::string cmd, int timeout) {
  auto start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
  int res = -1;
  int error_count = 0;
  while (true) {
    res = check(cmd);
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
    if (now - start_time > timeout) 
      res = DRIPPER_TIMEOUT;

    if (res > DRIPPER_DONE)
      error_count++;
    else if (res == DRIPPER_DONE)
      break;
    
    if(error_count > 2){
      ROS_ERROR_STREAM("Execute Commond: " << cmd
                        << " failed, error status: " << res);
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  if (res > DRIPPER_DONE)
    return false;
  else
    return true;
}
