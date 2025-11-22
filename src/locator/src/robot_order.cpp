#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "../examples/example.hpp"
using namespace std;
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_order");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Int32>("camera_operation", 1);

  std_msgs::Int32 op_code;
  op_code.data = 30;
  while(ros::ok()) {
    ros::spinOnce();
    pub.publish(op_code);
  }
}