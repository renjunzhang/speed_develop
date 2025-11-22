#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_manager_msgs/SwitchController.h>

using namespace std;

/**
 * @brief 罗技F710手柄(用于将/joy话题信息映射到对应按键)
 * 
 */
class F710
{
public:
  F710();
  /**
   * @brief 按键
   * 
   */
  int x, a, b, y, LB, RB, LT, RT, BACK, unknown1, unknown2, unknown3;
  /**
   * @brief 摇杆
   * 
   */
  float button_l, button_up, rstick_up, rstick_l, lstick_up, lstick_l;
};

F710::F710()
{
  x = 0;
  a = 0;
  b = 0;
  y = 0;
  LB = 0;
  RB = 0;
  LT = 0;
  RT = 0;
  BACK = 0;
  unknown1 = 0;
  unknown2 = 0;
  unknown3 = 0;
  button_l = 0;
  button_up = 0;
  rstick_l = 0;
  rstick_up = 0;
  lstick_l = 0;
  lstick_up = 0;
}

/**
 * @brief 手柄控制UR5e机械臂
 * @details 平移:x,b为x方向平移;y,a为y方向平移;LB,RB为z方向平移;旋转:左右方向键为绕x轴旋转;上下方向键为绕y轴旋转;LT,RT为绕z轴旋转;夹爪控制:左遥控左右控制闭合位置
 */
class ur5e_js
{
public:
  /**
 * @brief 切换ros controller为twist_controller
 * 
 * @param nh 
 */
  ur5e_js(ros::NodeHandle nh);
  /**
   * @brief 析构,并切换ros controller为scaled_joint_trajectory_controller
   * 
   */
  ~ur5e_js();
  /**
   * @brief 遥控主程序
   * 
   */
  void run();

private:
  /**
 * @brief 发布twist消息
 * 
 */
  unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> _realtime_pub;
  /**
   * @brief 订阅/joy消息
   * 
   */
  ros::Subscriber _sub;
  /**
   * @brief 切换ros controller的服务
   * 
   */
  ros::ServiceClient _srv;
  /**
   * @brief 罗技F710
   * 
   */
  std::unique_ptr<F710> _joystick_ptr;
  /**
   * @brief 订阅/joy消息的回调处理函数
   * 
   * @param joy_cmd 
   */
  void Callback1(const sensor_msgs::Joy::ConstPtr joy_cmd);
  /**
   * @brief 平移和旋转速度缩放系数
   * 
   */
  double _tra_scale, _rot_scale;
};

ur5e_js::ur5e_js(ros::NodeHandle nh)
{
  _joystick_ptr = make_unique<F710>();
  _realtime_pub = make_unique<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(nh, "/twist_controller/command", 4);
  _sub = nh.subscribe("/joy", 10, &ur5e_js::Callback1, this);
  _srv = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  _srv.waitForExistence();
  while (!_sub.getNumPublishers())
  {
    ROS_WARN_THROTTLE(3, "waiting joystick connection");
  }
  nh.param<double>("ur5e_js/tra_scale", _tra_scale, 0.05);
  nh.param<double>("ur5e_js/rot_scale", _rot_scale, 0.05);
  controller_manager_msgs::SwitchController sw;
  sw.request.start_controllers = vector<string>{"twist_controller"};
  sw.request.stop_controllers = vector<string>{"scaled_pos_joint_traj_controller"};
  sw.request.strictness = sw.request.BEST_EFFORT;
  _srv.call(sw);
  if(!sw.response.ok)
  {
    ROS_ERROR("Cannot swtich controller");
    ros::shutdown();
  }
  std::cout<<"switched to twist_controller"<<std::endl;
}

ur5e_js::~ur5e_js()
{
  controller_manager_msgs::SwitchController sw;
  sw.request.start_controllers = vector<string>{"scaled_pos_joint_traj_controller"};
  sw.request.stop_controllers = vector<string>{"twist_controller"};
  sw.request.strictness = sw.request.BEST_EFFORT;
  _srv.call(sw);
  if(!sw.response.ok)
  {
    ROS_ERROR("Cannot swtich controller");
    ros::shutdown();
  }
  std::cout<<"switched to scaled_pos_joint_traj_controller"<<std::endl;
}

void ur5e_js::run()
{
  if (_realtime_pub->trylock())
  {
    // LINEAR
    if (_joystick_ptr->x)
      _realtime_pub->msg_.linear.x = _tra_scale;
    else if (_joystick_ptr->b)
      _realtime_pub->msg_.linear.x = -_tra_scale;
    else 
      _realtime_pub->msg_.linear.x = 0;
    
    if (_joystick_ptr->y)
      _realtime_pub->msg_.linear.y = _tra_scale;
    else if (_joystick_ptr->a)
      _realtime_pub->msg_.linear.y = -_tra_scale;
    else 
      _realtime_pub->msg_.linear.y = 0;

    if (_joystick_ptr->LB)
      _realtime_pub->msg_.linear.z = _tra_scale;
    else if (_joystick_ptr->RB)
      _realtime_pub->msg_.linear.z = -_tra_scale;
    else
      _realtime_pub->msg_.linear.z = 0;

    // ANGULAR
    _realtime_pub->msg_.angular.x = _rot_scale * _joystick_ptr->lstick_up;
    _realtime_pub->msg_.angular.y = _rot_scale * _joystick_ptr->lstick_l;
    if (_joystick_ptr->LT)
      _realtime_pub->msg_.angular.z = _rot_scale;
    else if (_joystick_ptr->RT)
      _realtime_pub->msg_.angular.z = -_rot_scale;
    else 
      _realtime_pub->msg_.angular.z = 0;
    // cout << _realtime_pub->msg_ << endl;
    _realtime_pub->unlockAndPublish();
  }
}

void ur5e_js::Callback1(const sensor_msgs::Joy::ConstPtr joy)
{
  _joystick_ptr->x = joy->buttons[0];
  _joystick_ptr->a = joy->buttons[1];
  _joystick_ptr->b = joy->buttons[2];
  _joystick_ptr->y = joy->buttons[3];
  _joystick_ptr->LB = joy->buttons[4];
  _joystick_ptr->RB = joy->buttons[5];
  _joystick_ptr->LT = joy->buttons[6];
  _joystick_ptr->RT = joy->buttons[7];
  _joystick_ptr->BACK = joy->buttons[8];
  _joystick_ptr->unknown1 = joy->buttons[9];
  _joystick_ptr->unknown2 = joy->buttons[10];
  _joystick_ptr->unknown3 = joy->buttons[11];
  _joystick_ptr->button_l = joy->axes[0];
  _joystick_ptr->button_up = joy->axes[1];
  _joystick_ptr->rstick_up = joy->axes[2];
  _joystick_ptr->rstick_l = joy->axes[3];
  _joystick_ptr->lstick_up = joy->axes[4];
  _joystick_ptr->lstick_l = joy->axes[5];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5e_js");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ur5e_js js_cmd(nh);
  auto rate = ros::Rate(50);
  while (ros::ok())
  {
    js_cmd.run();
    rate.sleep();
  }
  return 0;
}
