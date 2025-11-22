#ifndef MODBUS_RTU_BASE_H
#define MODBUS_RTU_BASE_H
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

#define SPEED 50
#define FORCE 20
#define CURRENTLOW 100
#define CURRENTHIGH 1000

class modbusrtu;

class pgi : public chemical::device
{
private:
  std::shared_ptr<modbusrtu> _rtu;
  std::string dev;
  int baudrate;
  std::thread* _thread_check_serial_ptr = nullptr;
  std::atomic_bool theadstop;
  ros::Publisher _pub1_;
public:

  pgi(ros::NodeHandle& nh);
  ~pgi();

  DeviceErrorCode init(void*) override;
  DeviceErrorCode connect() override;
  DeviceErrorCode execute_script(std::string script) override;
  DeviceErrorCode execute_scripts(std::vector<std::string> scripts) override;
  DeviceErrorCode back_to_checkpoint() override;
  DeviceErrorCode getdata(std::string dataname,std::string& data) override;

private:
  /**
   * @brief 夹爪单向初始化
   *
   */
  void initSingle();
  /**
   * @brief 夹爪双向初始化
   *
   */
  void initDouble();
  /**
   * @brief 设定夹爪位置，速度，闭合力和是否阻塞
   *
   * @param position 目标位置
   * @param block 若为真则阻塞直至停止运动
   * @param force 设定闭合力
   * @param speed 设定运动速度
   */
  void setall(uint16_t position, bool block, uint16_t force = FORCE, uint16_t speed = SPEED);
  /**
   * @brief 设定夹爪目标位置
   *
   * @param command 目标位置
   */
  void setPos(uint16_t command);
  /**
   * @brief 设定夹爪闭合力
   *
   * @param command 闭合力
   */
  void setForce(uint16_t command);
  /**
   * @brief 设定夹爪运动速度
   *
   * @param command 运动速度
   */
  void setSpeed(uint16_t command);
  /**
   * @brief 判断是否初始化
   *
   * @return true 已初始化
   * @return false 未初始化
   */
  bool isInit();
  /**
   * @brief 判断夹爪是否运动
   *
   * @return true 正在运动
   * @return false 空闲
   */
  bool isBusy();
  /**
   * @brief 是否夹持物体
   *
   * @return true 有夹持
   * @return false 没有夹持
   */
  bool isGrip();
  /**
   * @brief 根据电流判断是否夹持物体
   *
   * @return true
   * @return false
   */
  bool isGripCur();
  /**
   * @brief 夹持物体是否掉落
   *
   * @return true 掉落
   * @return false 未掉落
   */
  bool isDrop();
  /**
   * @brief 根据电流判断是否掉落
   *
   * @return true
   * @return false
   */
  bool isDropCur();
  /**
   * @brief 内部函数，用于写入寄存器
   *
   * @param out_buf
   * @param in_buf
   * @param command
   */
  void setcommand(uint8_t *out_buf, uint8_t *in_buf, std::string command);
  /**
   * @brief 内部函数，用于读取寄存器
   *
   * @param out_buf
   * @param in_buf
   * @param command
   */
  void readcommand(uint8_t *out_buf, uint8_t *in_buf, std::string command);
  /**
   * @brief 获取夹爪当前位置
   *
   * @return uint16_t 夹爪当前位置
   */
  uint16_t getPos();

  /**
   * @brief 获取夹爪当前电流
   *
   * @return uint16_t 夹爪当前电流
   */
  uint16_t getCurrent();
};
#endif