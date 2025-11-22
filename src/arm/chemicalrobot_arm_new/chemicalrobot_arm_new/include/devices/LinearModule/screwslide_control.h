#ifndef SCREWSLIDE_CONTROL_H
#define SCREWSLIDE_CONTROL_H

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


class ScrewSlideControl : public chemical::device
{
private:
  serial::Serial _ser;
  std::string _dev;
  int _baudrate;
  std::thread* _thread_check_serial_ptr = nullptr;
  std::atomic_bool _theadstop;
  std::mutex _mutex;
public:

  ScrewSlideControl();
  ~ScrewSlideControl();

  DeviceErrorCode init(void*) override;
  DeviceErrorCode connect() override;
  DeviceErrorCode execute_script(std::string script) override;
  DeviceErrorCode execute_scripts(std::vector<std::string> scripts) override;
  DeviceErrorCode back_to_checkpoint() override;
  DeviceErrorCode getdata(std::string dataname,std::string& data) override;
private:

    /**
   * @brief enable the driver
   */
  void enable();
  /**
   * @brief disable the driver
   */
  void disable();
  /**
   * @brief Get the position
   *
   * @return float current position(mm)
   */
  double get_position();
  /**
   * @brief move to absolute position
   *
   * @param position target position(mm)
   * @param speed target speed(mm/s)
   * @param block whether block until stop
   * 
   * @return true if success
   * @return false if fault
   */
  bool moveabs(double position, double speed, bool block);
  /**
   * @brief check move status
   *
   * @return 0 if stopped,1 if moving,-1 if fault
   */
  int movestatus();
  /**
   * @brief check if serial is connected
   *
   * @return true if connected,
   * @return otherwise false
   */
  bool isopened();
  /**
   * @brief clear faults in motor driven
   */
  void clearfaults();

  void serial_write(std::string& data);
  void serial_read(std::string& data);
};
#endif