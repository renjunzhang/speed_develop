#include "devices/LinearModule/screwslide_control.h"

ScrewSlideControl::ScrewSlideControl()
{
    _attribute.clear();
    setAttribute("device_name", (void*)"/dev/track");
    setAttribute("baudrate", (void*)"115200");
}

ScrewSlideControl::~ScrewSlideControl()
{
    disable();
    _theadstop.store(true);
}

DeviceErrorCode ScrewSlideControl::init(void*)
{
    // init all member variables
    _status = DS_UNCONNECTED;
    _script_history.clear();
    _current_scripts.clear();
    _curr_point = _check_point = _current_scripts.begin();
    _theadstop.store(false);

    // get serial params
    auto devicename = *(std::string*)getAttribute("device_name");
    auto baudrate_str = *(std::string*)getAttribute("baudrate");
    if (devicename.empty() || baudrate_str.empty())
    {
        ROS_ERROR_STREAM(std::string("ScrewSlide device lack attributions, current settings are:\"device_name\":\"") +
                         devicename + "\",\"_baudrate\":\"" + baudrate_str + "\"");
        return DEVICE_LACK_ATTRIBUTION;
    }
    _dev = devicename;
    _baudrate = 0;
    try
    {
        _baudrate = std::stoi(baudrate_str);
    }
    catch (std::invalid_argument e)
    {
        ROS_ERROR_STREAM(std::string("Invalid baudrate attribution setting:[") + baudrate_str + "], ignored");
        return DEVICE_INVALID_ATTRIBUTION;
    }
    ROS_INFO_STREAM(std::string("Current device name: ") + _dev + ",baudrate: " + baudrate_str);

    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    _ser.setPort(_dev);
    _ser.setBaudrate(_baudrate);
    _ser.setTimeout(timeout);

    // check if serial alive in a thread loop, if not, reconnect it every 5 seconds
    if (_thread_check_serial_ptr == nullptr)
    {
        ROS_INFO_STREAM("Start Screw Slide alive check thread...");
        _thread_check_serial_ptr = new std::thread([&]()
                                                   {
      while(!_theadstop.load()){
        if(connect() != DEVICE_SUCCESS){
          _status = DS_UNCONNECTED;
          ROS_INFO_STREAM("Screw Slide will re-try connection in 5 seconds");
        }
				else if(movestatus() == -1){
					_status = DS_FAULT;
					ROS_INFO_STREAM("Screw Slide in fault status, recovering ...");
					clearfaults();
					enable();
					if(movestatus() == -1)
						ROS_INFO_STREAM("Screw Slide recovered failed, try again in 5 seconds");
					else{
						ROS_INFO_STREAM("Screw Slide recovered ok");
						_status = DS_IDEL;
					}
				}
        std::this_thread::sleep_for(std::chrono::seconds(5));
      } });
        _thread_check_serial_ptr->detach();
    }
  return DEVICE_SUCCESS;
}

DeviceErrorCode ScrewSlideControl::connect()
{
  try {
    if(!_ser.isOpen())
      _ser.open();
    enable();
  } 
  catch (serial::IOException e) {
    _status = DS_UNCONNECTED;
    ROS_ERROR_STREAM(
        std::string("Screw Slide Control connect failed,error messsage:") +
        e.what());
    return DEVICE_CONNECT_FAILED;
  } 
  catch (std::string e) {
    _status = DS_FAULT;
    ROS_ERROR_STREAM("Screw Slide Control communication failed");
    return DEVICE_IS_FAULT;
  }

  _status = DS_IDEL;
  return DEVICE_SUCCESS;
}

DeviceErrorCode ScrewSlideControl::execute_script(std::string script)
{
    ROS_INFO_STREAM("scripts size: 1");
    ROS_INFO_STREAM("script    : " + script);
    
    auto res = splitstr(script, ' ');
    assert(res[0] == "SL" && res.size() == 2);

    if (_status == DS_RUNNING)
      return DEVICE_ISRUNNIN;
    else if (_status == DS_UNCONNECTED)
      return DEVICE_CONNECT_FAILED;
		else if(_status == DS_FAULT)
			return DEVICE_IS_FAULT;

    try
    {
      auto position = std::stod(res[1]);
      auto speed = 90;//std::stod(res[2]);
      auto block = true;//(res[3] == "true" ? true : false);
      _status = DS_RUNNING;
      if(!moveabs(position,speed,block))
      {
				_status = DS_FAULT;
        return DEVICE_IS_FAULT;
      }
    }
    catch (std::invalid_argument e)
    {
      ROS_ERROR_STREAM((std::string("Screw Slide Control execute script[") + script + "] failed,error message:") + e.what());
      _status = DS_IDEL;
      return DEVICE_INVALID_SCRIPT;
    }
    catch (std::string e)
    {
      ROS_ERROR_STREAM(std::string("Screw Slide Control execute script[") + script + "] failed,error message:" + e);
      _status = DS_FAULT;
      return DEVICE_IS_FAULT;
    }

    _status = DS_IDEL;
    return DEVICE_SUCCESS;
}

DeviceErrorCode ScrewSlideControl::execute_scripts(std::vector<std::string> scripts)
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

DeviceErrorCode ScrewSlideControl::back_to_checkpoint()
{
	ROS_INFO_STREAM("Go back to Screw Slide check point ...");
  auto backward_script = std::vector<std::string>(_current_scripts.begin(), _check_point);
  std::reverse(backward_script.begin(),backward_script.end());
	auto code = execute_scripts(backward_script);
  if(code == DEVICE_SUCCESS)
    ROS_INFO_STREAM("Go back to Screw Slide check point ok");
  else
    ROS_INFO_STREAM("Go back to Screw Slide check point failed");
	return code;
}

DeviceErrorCode ScrewSlideControl::getdata(std::string dataname,std::string& data) {
 data.clear();
  try{
    if(dataname == "get_position"){
      auto pos = get_position();
      data = std::to_string(pos);
    }
    else
      return DEVICE_INVALID_ATTRIBUTION;
  }
  catch (std::string e){
    ROS_ERROR_STREAM("Screw Slide  "+ dataname + " failed,error message:" + e);
    _status = DS_FAULT;
    return DEVICE_IS_FAULT;
  }
  return DEVICE_SUCCESS;
}

void ScrewSlideControl::enable()
{
    std::string data = "EN\r";
    serial_write(data);
    serial_read(data);
}

void ScrewSlideControl::disable()
{
    std::string data = "K\r";
    serial_write(data);
    serial_read(data);
}

double ScrewSlideControl::get_position()
{
    int retry = 0; 
ss:
    std::string data = "PFB\r";
    serial_write(data);
    // 读取
    serial_read(data);
    // std::cout << "current position(" << data.size() << "): " << data << std::endl;
    double dis = 0;
    try{
      dis = std::stod(data);
    }
    catch(std::invalid_argument e){
      if(retry < 3)
      {
        retry++;
        goto ss;
      }
      else{
        ROS_ERROR_STREAM("Screw Slide Control get_position failed, " << e.what());
        throw (std::string("Screw Slide Control get_position failed, ") + e.what());
      }
    }
    return dis;
}

bool ScrewSlideControl::moveabs(double position, double speed, bool block)
{
    int retry = 0;
    std::stringstream ss1, ss2;
    ss1 << std::setiosflags(std::ios::fixed) << std::setprecision(3) << position;
    ss2 << std::setiosflags(std::ios::fixed) << std::setprecision(3) << speed;
ss:
    std::string data = "MOVEABS " + ss1.str() + " " +
                       ss2.str() + "\r";
    serial_write(data);
    serial_read(data);
    if (data != "-->")
    {
      if(retry < 3){
        retry++;
        goto ss;
      }
      else{
        ROS_ERROR_STREAM("Screw Slide Control moveabs failed,serial read response:" << data);
        return false;
      }
    }

    if (block)
    {
        auto status = movestatus();
        while (status == 1)
        {
            usleep(1000 * 200);
            status = movestatus();
        }
        if (status == -1)
        {
            ROS_ERROR_STREAM("Screw slide move fault with emergency stop");
            return false;
        }
    }
    return true;
}

int ScrewSlideControl::movestatus() {
    int fault_count = 0;
    int error_message_cout = 0;
ss:
    std::string data = "STOPPED\r";
    serial_write(data);
    serial_read(data);
    auto num = data[0];
    // std::cout << "stopped(" << data.size() << "): " << data << std::endl;
    // return data[0] == '2';
    if (num == '0')  // moving
        return 1;
    else if (num == '2')  // stopped
        return 0;
    else if (num == '-')  // fault
    {
        // check for 5 times
        if (fault_count < 5) {
            fault_count++;
            ros::Rate(1).sleep();
            goto ss;
        }
        return -1;
    } else {
        ROS_ERROR_STREAM(
            "Screw Slide Control \"movestatus\" received error response: "
            << data);
        error_message_cout++;
        if(error_message_cout > 3){
          error_message_cout = 0;
          std::cout << "Force continue? Y/N" << std::endl;
          std::string res;
          std::cin >> res;
          if(res == "Y" || res == "y")
            goto ss;
          else
            return -1;
        }
        else{
          ros::Rate(1).sleep();
          goto ss;
        }
    }
}

bool ScrewSlideControl::isopened()
{
    return _status != DS_UNCONNECTED;
}

void ScrewSlideControl::clearfaults()
{
    std::string data = "CLEARFAULTS\r";
    serial_write(data);
    serial_read(data);
}

void ScrewSlideControl::serial_write(std::string &data)
{
    std::unique_lock<std::mutex> lk(_mutex);
    if (_ser.write(data) < data.size())
    {
        ROS_ERROR_STREAM("Screw slide control write serial buffer length error, writed data: " + data);
				throw std::string("Screw slide control write serial buffer length error");
    }
}

void ScrewSlideControl::serial_read(std::string &data)
{
    std::unique_lock<std::mutex> lk(_mutex);
    try
    {
      data = _ser.readline();
      data = _ser.readline();
    }
    catch(serial::SerialException e)
    {
      ROS_ERROR_STREAM(e.what());
    }
    
		if(data.empty())
		{
			  ROS_ERROR_STREAM("Screw slide control read serial buffer empty");
				throw std::string("Screw slide control read serial buffer empty");
		}
    _ser.flush();
}
