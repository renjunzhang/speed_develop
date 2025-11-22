#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <typeinfo>

#include <Eigen/Core> // 包含Matrix和Array类，基础的线性代数运算和数组操作
#include <Eigen/Dense> //包含了下面的Core/Geometry/LU/Cholesky/SVD/QR/Eigenvalues模块
#include <Eigen/Geometry>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

/*
检测特定ID的标签，返回位姿，根据边长计算位姿的旋转矩阵，根据深度计算位姿的平移矩阵
参数说明：
frameset：相机获取的图像，相机内rs2类
color_image：彩色图像， OpenCV Mat类型
depth：深度图像数据，相机内rs2类
id：待检测的标签id
side_length：真实世界的标签边长，单位是米
cameraMatrix：相机内参矩阵
distCoeffs：相机畸变系数
depth_scale：缩放值
depth_intrin：深度相机内参
color_intrin：彩色相机内参
depth_extrin_to_color：深度相机到彩色相机的外参
color_extrin_to_depth：彩色相机到深度相机的外参
cflag：标志值，1表示取标签中心像素坐标，0表示取标签左上角点像素坐标
*/
geometry_msgs::PoseStamped estimate_aruco_pose(
    rs2::frameset frameset, Mat color_image, rs2::depth_frame depth, int id,
    float side_length, Mat cameraMatrix, Mat distCoeffs, float depth_scale,
    struct rs2_intrinsics depth_intrin, struct rs2_intrinsics color_intrin,
    struct rs2_extrinsics depth_extrin_to_color,
    struct rs2_extrinsics color_extrin_to_depth, int cflag) {
  geometry_msgs::PoseStamped aruco_pose;
  // Load the dictionary that was used to generate the markers.
  Ptr<aruco::Dictionary> dictionary =
      getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
  // Initialize the detector parameters using default values
  Ptr<aruco::DetectorParameters> parameters =
      aruco::DetectorParameters::create();
  // Declare the vectors that would contain the detected marker corners and
  // the rejected marker candidates
  vector<vector<Point2f>> markerCorners, rejectedCandidates;
  // The ids of the detected markers are stored in a vector
  vector<int> markerIds;
  // Detect the markers in the image
  detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters,
                rejectedCandidates);
  // aruco::drawDetectedMarkers(color_image, markerCorners, markerIds);

  float color_pixel[2]; // color pixel
  float depth_pixel[2];
  float depth_point[3]; // point in depth space
  float color_point[3];

  vector<Vec3d> rvecs, tvecs;
  Mat rotation_dst;
  Eigen::Matrix3f rotation_matrix(3, 3);
  int flag = 0;

  // 检测目标id的标签
  // if ((markerIds.size() == 1) && (markerIds[0] == 38)) {
  if (markerIds.size() > 0) {
    for (int k = 0; k < markerIds.size(); k++) {
      if (markerIds[k] == id) {
        if (cflag == 1) {
          // 取标签中心点像素坐标
          color_pixel[0] = (markerCorners[k][0].x + markerCorners[k][2].x +
                            markerCorners[k][1].x + markerCorners[k][3].x) /
                           4.0;
          color_pixel[1] = (markerCorners[k][0].y + markerCorners[k][2].y +
                            markerCorners[k][1].y + markerCorners[k][3].y) /
                           4.0;
        } else {
          // 取标签左上角像素坐标
          color_pixel[0] = markerCorners[k][0].x;
          color_pixel[1] = markerCorners[k][0].y;
        }

        circle(color_image, Point(round(color_pixel[0]), round(color_pixel[1])),
               2, Scalar(0, 0, 255), -1); // 画半径为2的圆点
        // 彩色相机下的像素坐标映射到深度相机下的像素坐标
        rs2_project_color_pixel_to_depth_pixel(
            depth_pixel,
            reinterpret_cast<const uint16_t *>(frameset.get_data()),
            depth_scale, 0.1, 3.0, &depth_intrin, &color_intrin,
            &color_extrin_to_depth, &depth_extrin_to_color, color_pixel);
        // 计算深度值
        auto dist_to_object =
            depth.get_distance(depth_pixel[0], depth_pixel[1]);
        // 深度像素映射到深度相机坐标系三维空间坐标
        rs2_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel,
                                     dist_to_object);
        // 深度相机坐标系三维空间坐标映射到彩色相机坐标系下三维空间坐标
        rs2_transform_point_to_point(color_point, &depth_extrin_to_color,
                                     depth_point);
        // 估计标签位姿
        aruco::estimatePoseSingleMarkers(
            markerCorners, side_length, cameraMatrix, distCoeffs, rvecs, tvecs);
        // aruco::drawAxis(color_image, cameraMatrix, distCoeffs, rvecs[k],
        //                 tvecs[k], 0.015);
        // 位姿旋转向量转换为旋转矩阵
        Rodrigues(rvecs[k], rotation_dst);
        // cout<<rotation_dst<<endl;
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            rotation_matrix(i, j) = rotation_dst.at<double>(i, j);
          }
        }
        // 旋转矩阵转换为四元数，并保存为ros内pose数据格式，注意此处的位置坐标是通过像素坐标计算得到的，而不是用标签位姿估计得到的位移向量
        Eigen::Quaternionf quat(rotation_matrix);

        aruco_pose.header.frame_id = "camera_color_frame";
        aruco_pose.pose.position.x = color_point[0];
        aruco_pose.pose.position.y = color_point[1];
        aruco_pose.pose.position.z = color_point[2];
        aruco_pose.pose.orientation.x = quat.x();
        aruco_pose.pose.orientation.y = quat.y();
        aruco_pose.pose.orientation.z = quat.z();
        aruco_pose.pose.orientation.w = quat.w();
        flag = 1;
        cout << "***** 已检测到标签 ID = " << id << " *****" << endl;
        cout << endl;
      }
    }
  }
  // 未检测到标签，将所有值置为0
  if (flag == 0) {
    aruco_pose.header.frame_id = "camera_color_frame";
    aruco_pose.pose.position.x = 0.0;
    aruco_pose.pose.position.y = 0.0;
    aruco_pose.pose.position.z = 0.0;
    aruco_pose.pose.orientation.x = 0.0;
    aruco_pose.pose.orientation.y = 0.0;
    aruco_pose.pose.orientation.z = 0.0;
    aruco_pose.pose.orientation.w = 0.0;
    cout << "***** 未检测到标签 ID = " << id << " *****" << endl;
    cout << endl;
  }
  // imshow("color_image", color_image);
  // waitKey(1);
  return aruco_pose;
}

// 检测目标id的标签在彩色图像上的中心像素坐标
vector<float> aruco_center_pixel(Mat color_image, int id) {
  // Load the dictionary that was used to generate the markers.
  Ptr<aruco::Dictionary> dictionary =
      getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
  // Initialize the detector parameters using default values
  Ptr<aruco::DetectorParameters> parameters =
      aruco::DetectorParameters::create();
  // Declare the vectors that would contain the detected marker corners and
  // the rejected marker candidates
  vector<vector<Point2f>> markerCorners, rejectedCandidates;
  // The ids of the detected markers are stored in a vector
  vector<int> markerIds;
  // Detect the markers in the image
  detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters,
                rejectedCandidates);
  // aruco::drawDetectedMarkers(color_image, markerCorners, markerIds);
  vector<float> c;
  float color_pixel[2]; // color pixel

  vector<Vec3d> rvecs, tvecs;
  int flag = 0;

  // if ((markerIds.size() == 1) && (markerIds[0] == 38)) {
  if (markerIds.size() > 0) {
    for (int k = 0; k < markerIds.size(); k++) {
      if (markerIds[k] == id) {
        color_pixel[0] = (markerCorners[k][0].x + markerCorners[k][2].x +
                          markerCorners[k][1].x + markerCorners[k][3].x) /
                         4.0;
        color_pixel[1] = (markerCorners[k][0].y + markerCorners[k][2].y +
                          markerCorners[k][1].y + markerCorners[k][3].y) /
                         4.0;
        // circle(color_image, Point(round(color_pixel[0]),
        // round(color_pixel[1])),
        //        2, Scalar(0, 0, 255), -1);  //画半径为2的圆点
        c.push_back(color_pixel[0]);
        c.push_back(color_pixel[1]);
        flag = 1;
      }
    }
  }
  if (flag == 0) {
    c.push_back(0.0);
    c.push_back(0.0);
  }
  // imshow("color_image", color_image);
  // waitKey(1);
  return c;
}

// 图像传输子线程
void video_sending(int argc, char **argv, rs2::pipeline p) {
  ros::init(argc, argv, "video_send_by_frame");
  ros::NodeHandle nh_vs;
  ros::Publisher pub_vs =
      nh_vs.advertise<sensor_msgs::Image>("color_frame_sending", 1);
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rs2::frameset video_frameset = p.wait_for_frames();
    auto color_frame = video_frameset.get_color_frame();
    const int color_fw = color_frame.as<rs2::video_frame>().get_width();
    const int color_fh = color_frame.as<rs2::video_frame>().get_height();
    // cout<< color_fw <<" "<<color_fh<<endl;
    Mat color_f(Size(color_fw, color_fh), CV_8UC3,
                (void *)color_frame.get_data(), Mat::AUTO_STEP);
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_f).toImageMsg();
    pub_vs.publish(*msg);
    rate.sleep();
  }
}

// 主函数
int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "aruco_to_point_and_talk");
  ros::NodeHandle nh;
  // 检测到的工作站标签位姿，会被发送到point_coordinate话题中
  ros::Publisher pub =
      nh.advertise<geometry_msgs::PoseStamped>("point_coordinate", 1);
  // 固体进样器加样头是否复位检测，检测到的标签位姿，会被发送到position_tp话题中
  ros::Publisher pub_position_tp =
      nh.advertise<std_msgs::Int32>("position_tp", 1);

  // 设置相机，并获取相机参数
  // Helper to colorize depth images
  rs2::colorizer c;
  // Create a pipeline to easily configure and start the camera
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16,
                    30); // 848, 480
  cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);

  rs2::pipeline_profile profile = pipe.start(cfg);
  auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();

  static float depth_scale = depth_sensor.get_depth_scale();
  static struct rs2_intrinsics depth_intrin;
  static struct rs2_intrinsics color_intrin;
  static struct rs2_extrinsics depth_extrin_to_color;
  static struct rs2_extrinsics color_extrin_to_depth;

  const rs2::stream_profile color_profile =
      profile.get_stream(RS2_STREAM_COLOR);
  const rs2::stream_profile depth_profile =
      profile.get_stream(RS2_STREAM_DEPTH);
  depth_intrin = depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
  color_intrin = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
  depth_extrin_to_color =
      depth_profile.as<rs2::video_stream_profile>().get_extrinsics_to(
          color_profile);
  color_extrin_to_depth =
      color_profile.as<rs2::video_stream_profile>().get_extrinsics_to(
          depth_profile);

  Mat cameraMatrix(3, 3, CV_32FC1);
  cameraMatrix.at<float>(0, 0) = color_intrin.fx;
  cameraMatrix.at<float>(0, 1) = 0.0;
  cameraMatrix.at<float>(0, 2) = color_intrin.ppx;
  cameraMatrix.at<float>(1, 0) = 0.0;
  cameraMatrix.at<float>(1, 1) = color_intrin.fy;
  cameraMatrix.at<float>(1, 2) = color_intrin.ppy;
  cameraMatrix.at<float>(2, 0) = 0.0;
  cameraMatrix.at<float>(2, 1) = 0.0;
  cameraMatrix.at<float>(2, 2) = 1.0;
  Mat distCoeffs(1, 5, CV_32FC1, color_intrin.coeffs);

  // publish color image for video sending
  thread t1(video_sending, argc, argv, pipe);
  t1.detach();

  // 声明待接收的来自机械臂的命令
  int operation_code; // 两位数操作码，十位代表任务编号，个位代表具体检测目标
  int oc1;            // operation_code 十位
  int oc2;            // operation_code 个位

  cout << endl;
  cout << "***** Hello, AI Chemist Visual System Started, Good Luck !!! *****"
       << endl;
  cout << endl;
  ros::Rate rate(10); // Hz, number of msgs per second

  // 开始标签视觉检测
  while (ros::ok()) // Application still alive?
  {
    cout << "***** 等待操作指令 *****" << endl;
    cout << endl;
    // destroyAllWindows();
    // 等待机械臂向话题camera_operation中发送消息，即operation_code
    auto operation =
        ros::topic::waitForMessage<std_msgs::Int32>("camera_operation");
    ros::Duration(0.5).sleep();
    operation_code = operation->data;
    oc2 = operation_code % 10;
    oc1 = (operation_code - oc2) / 10;
    int label_id = operation_code;
    cout << operation_code;
    ROS_INFO_STREAM("收到操作指令：" << std::to_string(operation_code));

    // ***** 1. aruco locator strat *****，检测工作站标签位姿，循环 10
    // 次取平均值
    if ((operation_code >= 0) &&
        (operation_code < 50)) { // operation_code与工作站标签一一对应
      ros::spinOnce();

      geometry_msgs::PoseStamped pose;
      int epoch_flag = 0;
      pose.header.frame_id = "camera_color_frame";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 0.0;
      // for 循环，位姿累加10次取平均值，如果有一次 z
      // 值为0，就跳出循环，返回重定位失败
      for (int epoch = 0; epoch < 10; epoch++) {
        // Using the align object, we block the application until a frameset
        // is available
        rs2::frameset frameset = pipe.wait_for_frames();
        // With the aligned frameset we proceed as usual
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        auto colorized_depth = c.colorize(depth);

        const int depth_w = depth.as<rs2::video_frame>().get_width();
        const int depth_h = depth.as<rs2::video_frame>().get_height();
        const int color_w = color.as<rs2::video_frame>().get_width();
        const int color_h = color.as<rs2::video_frame>().get_height();
        const int colorized_depth_w =
            colorized_depth.as<rs2::video_frame>().get_width();
        const int colorized_depth_h =
            colorized_depth.as<rs2::video_frame>().get_height();

        Mat depth_image(Size(depth_w, depth_h), CV_8UC3,
                        (void *)depth.get_data(), Mat::AUTO_STEP);
        Mat color_image(Size(color_w, color_h), CV_8UC3,
                        (void *)color.get_data(), Mat::AUTO_STEP);
        Mat colorized_depth_image(Size(colorized_depth_w, colorized_depth_h),
                                  CV_8UC3, (void *)colorized_depth.get_data(),
                                  Mat::AUTO_STEP);

        cout << "***** " << epoch << ", 开始标签重定位 *****" << endl;
        cout << endl;

        // ID， 边长
        geometry_msgs::PoseStamped pose_temp;
        pose_temp = estimate_aruco_pose(
            frameset, color_image, depth, label_id, 0.075, cameraMatrix,
            distCoeffs, depth_scale, depth_intrin, color_intrin,
            depth_extrin_to_color, color_extrin_to_depth, 0);
        pose.pose.position.x += pose_temp.pose.position.x;
        pose.pose.position.y += pose_temp.pose.position.y;
        pose.pose.position.z += pose_temp.pose.position.z;
        pose.pose.orientation.x += pose_temp.pose.orientation.x;
        pose.pose.orientation.y += pose_temp.pose.orientation.y;
        pose.pose.orientation.z += pose_temp.pose.orientation.z;
        pose.pose.orientation.w += pose_temp.pose.orientation.w;
        if (pose_temp.pose.position.z == 0.0) {
          break;
        } else {
          epoch_flag += 1;
        }
      }
      if (epoch_flag == 10) {
        pose.pose.position.x = pose.pose.position.x / 10.0;
        pose.pose.position.y = pose.pose.position.y / 10.0;
        pose.pose.position.z = pose.pose.position.z / 10.0;
        pose.pose.orientation.x = pose.pose.orientation.x / 10.0;
        pose.pose.orientation.y = pose.pose.orientation.y / 10.0;
        pose.pose.orientation.z = pose.pose.orientation.z / 10.0;
        pose.pose.orientation.w = pose.pose.orientation.w / 10.0;
        cout << "***** 重定位成功， 位姿: *****" << endl;
        cout << "Translation: " << endl;
        cout << "   " << pose.pose.position.x << endl;
        cout << "   " << pose.pose.position.y << endl;
        cout << "   " << pose.pose.position.z << endl;
        cout << "Rotation: " << endl;
        cout << "   " << pose.pose.orientation.x << endl;
        cout << "   " << pose.pose.orientation.y << endl;
        cout << "   " << pose.pose.orientation.z << endl;
        cout << "   " << pose.pose.orientation.w << endl;
        cout << endl;

        ROS_INFO_STREAM(
            "***** 重定位成功 ******: " << std::to_string(operation_code));
      } else {
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;
        cout << "***** 重定位失败 *****" << endl;
        cout << endl;
        ROS_ERROR_STREAM(
            "***** 重定位失败 ******: " << std::to_string(operation_code));
      }
      pub.publish(pose);
      cout << "***** 标签重定位结束 *****" << endl;
      cout << endl;
    }
    rate.sleep();
  }
}
