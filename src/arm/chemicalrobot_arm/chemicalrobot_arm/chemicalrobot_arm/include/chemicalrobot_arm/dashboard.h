#include <ros/ros.h>
#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/GetRobotMode.h>
#include <ur_dashboard_msgs/RawRequest.h>
#include <ur_dashboard_msgs/GetProgramState.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <serial/serial.h>
#include <string>
using namespace std;

// 用于ur5e和气动夹爪
class dashboardsrv_client
{
public:
    dashboardsrv_client(ros::NodeHandle nh);

    /**
     * @brief 载入并启动程序
     *
     * @param filename 程序名称
     */
    void load_program(const string filename);

    /**
     * @brief 运行加载的程序
     *
     */
    void play();

    /**
     * @brief 停止程序
     *
     */
    void stop();

    /**
     * @brief 打开UR5e电源并释放制动
     *
     */
    void robot_init();

    /**
     * @brief 将数字输出引脚置0
     *
     */
    void DO_init();

    /**
     * @brief 设置数字输出引脚的电平
     *
     * @param pin 引脚的位置
     * @param state 设置的电平
     */
    void setDO(int8_t pin, bool state);

    /**
     * @brief 解除保护性停止
     *
     */
    void unlockPS();

    /**
     * @brief 已弃用
     *
     */
    void RG2grip();
    /**
     * @brief 已弃用
     *
     */
    void RG2release();
    /**
     * @brief 已弃用
     *
     */
    bool RG2_is_busy();
    /**
     * @brief 已弃用
     *
     */
    bool RG2_is_grip();

    /**
     * @brief 关闭电催化舵机
     *
     * @return int
     */
    int close(); //  close the engine
    /**
     * @brief 打开电催化舵机
     *
     * @return int
     */
    int open(); //  open the engine
    /**
     * @brief 气动夹爪闭合
     *
     * @return int
     */
    int grasp(); //  close the claw
    /**
     * @brief 气动夹爪打开
     *
     * @return int
     */
    int loose(); //  open the claw
    /**
     * @brief 气动夹爪复位
     *
     * @return int
     */
    int reset(); //  reset the claw

private:
    ros::NodeHandle nh_;
    ros::ServiceClient srv_client_;
    ros::ServiceClient load_client_;
    ros::ServiceClient DO_client_;
    ros::ServiceClient program_client_;
    ros::ServiceClient safety_client_;
    serial::Serial ros_ser;
};
