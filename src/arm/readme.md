# 机器人化学家_机械臂部分

## 启动流程

1. 启动机械臂ROS驱动(需要指定robot_ip,robot_description_file(可选)):

    ```bash
    roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.101.101 robot_description_file:=$(rospack find ur_description)/launch/load_ur5e_tfsensor_pgi140.launch
    ```

2. 启动moveit!:
g
    ```bash
    roslaunch ur5e_tfsensor_pgi140_moveit_config moveit_planning_execution.launch
    ```

3. 启动Rviz GUI(可选):

    ```bash
    roslaunch ur5e_tfsensor_pgi140_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz  
    ```

4. 启动视觉重定位：

    ```bash
    rosrun locator cvs5
    ```

5. 启动力矩传感器

    ```bash
    roslaunch robotiq_ft_sensor ft_sensor.launch
    ```

6. 启动机械臂操控程序:

    ```bash
    roslaunch chemicalrobot_arm chemicalrobot_arm.launch 
    ```
  
  或启动机械臂记录位姿程序:

  ```bash
  roslaunch chemicalrobot_arm record.launch
  ```

## 文件结构

> chemicalrobot_arm 机械臂操作的包
>> src  
>>> motion_planning.cpp 机械臂响应指令和反馈，轨迹规划，操作，异常处理等函数  
>>> motion_planning_util.cpp 机械臂载入位姿信息，内部处理等函数  
>>> dashboard.cpp UR5e远程启动和程序加载，气动夹爪和舵机控制函数  
>>> record.cpp 路径点记录函数

>> scripts  
>>> fake_command.py 模拟task_manager，用于向机械臂发布操作指令  
>>> fake_camerapose.py 模拟相机节点，用于向机械臂发布标签位姿  

>> data 用于存放各站点的位姿信息

> pgi140 夹爪模型和控制的包
>> src
>>> pgi_control.cpp 夹爪控制函数  
>>> pgi_test.cpp 夹爪控制测试函数  

> robotiq_ft_sensor 力传感器ros包  
> universal_robot UR5e的urdf文件，moveit配置等
> Universal_Robots_ROS_controllers_cartesian UR5e笛卡尔控制器
> Universal_Robots_ROS_Driver UR5e的ROS驱动

## 环境需求

1. 需要绑定usb串口名，夹爪为/dev/PGI，气动夹爪和舵机为/dev/GRIPPER，力传感器为FT_SENSOR
2. 为了能够正常生成日志，需要在/etc/profile中添加export DATE=$(date +%Y_%m_%d)  
3. ros的依赖库详见各个ros包的package.xml文件  
4. 注释可参考各个ros包下的doxygen/html/index.html
