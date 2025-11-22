#ifndef DRIPPER_CONTROL_H
#define DRIPPER_CONTROL_H
#include <vector>
#include <iostream>
#include <serial/serial.h>
#include <string>
#include <ros/ros.h>
#include <atomic>
#include <thread>
#include <mutex>

#include "chemical_common.h"
#include "devices/device.h"

class DripperControl : public chemical::device
{
	public:

	DripperControl();
	~DripperControl();

	DeviceErrorCode init(void*) override;
  DeviceErrorCode connect() override;
  DeviceErrorCode execute_script(std::string script) override;
  DeviceErrorCode execute_scripts(std::vector<std::string> scripts) override;
  DeviceErrorCode back_to_checkpoint() override;
  DeviceErrorCode getdata(std::string dataname,std::string& data) override;

private:
  bool ready_to_suck();
  bool suck();
  bool drip();
  bool change_tool();
  bool reset();
  int  check(std::string cmd);
  bool serial_write(std::string data);
  bool serial_read(std::string& data);
  bool wait_for_processing(std::string cmd, int timeout=10000);
  bool send_commond(std::string cmd);
  bool send_commond_retry(std::string cmd);

  bool connected = false;
	serial::Serial _ser;
  std::string _dev;
  std::thread* _thread_check_serial_ptr = nullptr;
  std::atomic_bool theadstop;

	typedef bool(DripperControl::*pfuc)(void);
	std::map<std::string,pfuc> fuc_map = {
		{"ready_to_suck",&DripperControl::ready_to_suck},
		{"suck",&DripperControl::suck},
		{"drip",&DripperControl::drip},
		{"change_tool",&DripperControl::change_tool},
		{"reset",&DripperControl::reset}
	};

	enum DripperError{
    DRIPPER_PREPROCESS,
    DRIPPER_PROCESSING,
    DRIPPER_DONE,
    DRIPPER_UNKNOWN_ERROR,
    DRIPPER_ERROR,
    DRIPPER_TIMEOUT,
    DRIPPER_WRONG_CMD,
    DRIPPER_COMMUNICATE_ERROR
	};
};
#endif