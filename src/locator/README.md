RealSense相机驱动版本：
SDK：librealsense-2.48.0
ROS package：realsense-ros-2.3.1
**************************************************************************
代码所需的环境、包和库文件：C++，realsense2，OpenCV，OpenGL，Eigen3，（具体可见 CMakeLists.txt 文件）
**************************************************************************
代码文件：./src/chemist_visual_system_v5.cpp
**************************************************************************
启动视觉检测的命令，终端输入：rosrun locator cvs5
**************************************************************************





以下为调试过程可能需要用的终端命令：
**************************************************************************
打开realsense相机GUI，在终端输入： realsense-viewer
**************************************************************************
利用ROS launch打开相机节点，在终端输入：

roslaunch realsense2_camera rs_camera.launch
roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=colorizer
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
roslaunch realsense2_camera rs_rgbd.launch filters:=colorizer,pointcloud align_depth:=true pointcloud_texture_stream:=RS2_STREAM_COLOR
roslaunch realsense2_camera rs_camera.launch filters:=colorizer align_depth:=true
roslaunch realsense2_camera rs_camera.launch filters:=colorizer filters:=pointcloud align_depth:=true
roslaunch realsense2_camera rs_camera.launch filters:=hole_filling
roslaunch realsense2_camera rs_rgbd.launch
**************************************************************************
在终端输入操作命令，进行测试，修改data为对应operation_code即可。例如：rostopic pub /camera_operation std_msgs/Int32 "data: 21"
**************************************************************************
