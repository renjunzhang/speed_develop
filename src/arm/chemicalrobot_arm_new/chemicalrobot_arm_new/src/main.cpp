#include <QtWidgets/QApplication>
#include "chemical_platform_system.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "chemicalrobot_arm_new");
  setlocale(LC_ALL, "");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(10);
  spinner.start();

  QApplication a(argc, argv);
  
  ChemicalPlatform platform(nh);
  platform.Init();
  ROS_INFO_STREAM("-------------ready to receive command---------\n");
  // while (ros::ok())
  //   ;
  a.setQuitOnLastWindowClosed(false);
  a.exec();
  ROS_INFO_STREAM("-------------motion planning exit---------\n");
}
