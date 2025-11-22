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
#include <jsoncpp/json/json.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

using namespace std;
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// using namespace cv;

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

class Rs2Camera {
public:
  rs2::pipeline pipe_;
  rs2::config cfg_;
  struct rs2_intrinsics depth_intrin_;
  struct rs2_intrinsics color_intrin_;
  struct rs2_extrinsics depth_extrin_to_color_;
  struct rs2_extrinsics color_extrin_to_depth_;
  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;
  rs2::colorizer color_image_;
  float depth_scale_;

public:
  Rs2Camera() {

    cfg_.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16,
                       30); // 848, 480
    cfg_.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);

    // start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe_.start(cfg_);
    //        profile = pipe_.start(cfg_);
    rs2::depth_sensor depth_sensor_ =
        profile.get_device().first<rs2::depth_sensor>();
    depth_scale_ = depth_sensor_.get_depth_scale();

    const rs2::stream_profile color_profile =
        profile.get_stream(RS2_STREAM_COLOR);
    const rs2::stream_profile depth_profile =
        profile.get_stream(RS2_STREAM_DEPTH);
    depth_intrin_ =
        depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
    color_intrin_ =
        color_profile.as<rs2::video_stream_profile>().get_intrinsics();
    depth_extrin_to_color_ =
        depth_profile.as<rs2::video_stream_profile>().get_extrinsics_to(
            color_profile);
    color_extrin_to_depth_ =
        color_profile.as<rs2::video_stream_profile>().get_extrinsics_to(
            depth_profile);

    cameraMatrix_ = cv::Mat(3, 3, CV_32FC1);
    cameraMatrix_.at<float>(0, 0) = color_intrin_.fx;
    cameraMatrix_.at<float>(0, 1) = 0.0;
    cameraMatrix_.at<float>(0, 2) = color_intrin_.ppx;
    cameraMatrix_.at<float>(1, 0) = 0.0;
    cameraMatrix_.at<float>(1, 1) = color_intrin_.fy;
    cameraMatrix_.at<float>(1, 2) = color_intrin_.ppy;
    cameraMatrix_.at<float>(2, 0) = 0.0;
    cameraMatrix_.at<float>(2, 1) = 0.0;
    cameraMatrix_.at<float>(2, 2) = 1.0;
    cv::Mat tmp(1, 5, CV_32FC1, color_intrin_.coeffs);
    distCoeffs_ = tmp;
  }
};

void IcpPoseEstimation_3d3d(const vector<cv::Point3f> &pts1,
                            const vector<cv::Point3f> &pts2,
                            cv::Vec3d &icp_rvec, cv::Vec3d &icp_tvec) {
  cv::Point3f p1, p2; // center of mass
  int N = min(pts1.size(), pts2.size());
  for (int i = 0; i < N; i++) {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 = cv::Point3f(cv::Vec3f(p1) / N);
  p2 = cv::Point3f(cv::Vec3f(p2) / N);
  vector<cv::Point3f> q1(N), q2(N); // remove the center
  for (int i = 0; i < N; i++) {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // compute q1*q2^T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (int i = 0; i < N; i++) {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) *
         Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  //    cout << "W=" << W << endl;

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  Eigen::Matrix3d R_ = U * (V.transpose());
  if (R_.determinant() < 0) {
    Eigen::Matrix3d tmp = Eigen::MatrixXd::Identity(3, 3);
    tmp(2, 2) = R_.determinant();

    R_ = U * tmp * (V.transpose());
    cout << "-R" << endl;
  }
  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) -
                       R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

  // convert to cv::Mat
  cv::Mat R = (cv::Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2), R_(1, 0),
               R_(1, 1), R_(1, 2), R_(2, 0), R_(2, 1), R_(2, 2));
  cv::Mat t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));

  icp_tvec = cv::Vec3d(t_(0, 0), t_(1, 0), t_(2, 0));
  cv::Rodrigues(R, icp_rvec);
}

void EstimateArucoPose(std::shared_ptr<Rs2Camera> rs2_camera,
                       const int &aruco_id, const std::string &solve_type,
                       Json::Value &pose_data) {
  int loop_nums = 0;
  vector<cv::Vec3d> pnp_rvecs, pnp_tvecs;
  std::vector<cv::Point3f> color_point_vector;
  float marker_length = 0.027;
  while (loop_nums < 10) {

    rs2::frameset frameset = rs2_camera->pipe_.wait_for_frames();
    // With the aligned frameset we proceed as usual
    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();
    auto colorized_depth = rs2_camera->color_image_.colorize(depth);

    const int depth_w = depth.as<rs2::video_frame>().get_width();
    const int depth_h = depth.as<rs2::video_frame>().get_height();
    const int color_w = color.as<rs2::video_frame>().get_width();
    const int color_h = color.as<rs2::video_frame>().get_height();
    const int colorized_depth_w =
        colorized_depth.as<rs2::video_frame>().get_width();
    const int colorized_depth_h =
        colorized_depth.as<rs2::video_frame>().get_height();

    cv::Mat depth_image(cv::Size(depth_w, depth_h), CV_8UC3,
                        (void *)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat color_image(cv::Size(color_w, color_h), CV_8UC3,
                        (void *)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat colorized_depth_image(
        cv::Size(colorized_depth_w, colorized_depth_h), CV_8UC3,
        (void *)colorized_depth.get_data(), cv::Mat::AUTO_STEP);

    geometry_msgs::PoseStamped aruco_pose;
    // Load the dictionary that was used to generate the markers.
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    // Initialize the detector parameters using default values
    cv::Ptr<cv::aruco::DetectorParameters> parameters =
        cv::aruco::DetectorParameters::create();
    // Declare the vectors that would contain the detected marker corners and
    // the rejected marker candidates
    vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
    // The ids of the detected markers are stored in a vector
    vector<int> markerIds;
    // Detect the markers in the image
    detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters,
                  rejectedCandidates);
    // cv::aruco::drawDetectedMarkers(color_image ,
    // markerCorners,aruco_id,cv::Scalar(0, 255, 0));

    int mark_index;
    auto mark_index_iter =
        std::find(markerIds.begin(), markerIds.end(), aruco_id);
    if (mark_index_iter != markerIds.end()) {
      cv::aruco::estimatePoseSingleMarkers(
          markerCorners, marker_length, rs2_camera->cameraMatrix_,
          rs2_camera->distCoeffs_, pnp_rvecs, pnp_tvecs);

      mark_index = mark_index_iter - markerIds.begin();
      //            cv::Point_<float> tmp_mark_corner(0.0, 0.0);
      //            for (auto markerCorner: markerCorners[mark_index]) {
      //                tmp_mark_corner.x += markerCorner.x;
      //                tmp_mark_corner.y += markerCorner.y;
      //            }
      //            markerCorners[mark_index].emplace_back(cv::Point_<float>(tmp_mark_corner.x/4
      //            , tmp_mark_corner.y/4 ));

      for (auto markerCorner : markerCorners[mark_index]) {
        circle(color_image, cv::Point2f(markerCorner), 2, cv::Scalar(0, 0, 255),
               -1);

        float color_pixel[2], depth_pixel[2], depth_point[3], color_point[3];
        color_pixel[0] = markerCorner.x;
        color_pixel[1] = markerCorner.y;
        rs2_project_color_pixel_to_depth_pixel(
            depth_pixel,
            reinterpret_cast<const uint16_t *>(frameset.get_data()),
            rs2_camera->depth_scale_, 0.05, 1.0, &rs2_camera->depth_intrin_,
            &rs2_camera->color_intrin_, &rs2_camera->color_extrin_to_depth_,
            &rs2_camera->depth_extrin_to_color_, color_pixel);

        if (depth_pixel[0] > 900) {
          std::cout << "采集不到深度信息深度信息" << std::endl;
          ROS_INFO_STREAM("采集不到深度信息深度信息");
          return;
        }
        auto dist_to_object =
            depth.get_distance(depth_pixel[0], depth_pixel[1]);
        if (dist_to_object < 1e-6) {
          ROS_INFO_STREAM("深度信息无效");
          std::cout << "深度信息无效" << std::endl;
          //   return;
        }

        // 深度像素映射到深度相机坐标系三维空间坐标
        rs2_deproject_pixel_to_point(depth_point, &rs2_camera->depth_intrin_,
                                     depth_pixel, dist_to_object);
        // 深度相机坐标系三维空间坐标映射到彩色相机坐标系下三维空间坐标
        rs2_transform_point_to_point(
            color_point, &rs2_camera->depth_extrin_to_color_, depth_point);
        color_point_vector.emplace_back(
            cv::Point3f(color_point[0], color_point[1], color_point[2]));
      }
    }
    loop_nums++;
  }

  if (40 != color_point_vector.size()) {
    return;
  }

  // icp
  std::vector<cv::Point3f> aruco_world_points;
  for (int i = 0; i < 10; i++) {
    aruco_world_points.emplace_back(
        cv::Point3f(-marker_length / 2, marker_length / 2, 0));
    aruco_world_points.emplace_back(
        cv::Point3f(marker_length / 2, marker_length / 2, 0));
    aruco_world_points.emplace_back(
        cv::Point3f(marker_length / 2, -marker_length / 2, 0));
    aruco_world_points.emplace_back(
        cv::Point3f(-marker_length / 2, -marker_length / 2, 0));
  }

  cv::Vec3d icp_rvec, icp_tvec;
  IcpPoseEstimation_3d3d(color_point_vector, aruco_world_points, icp_rvec,
                         icp_tvec);

  // 总结
  cv::Vec3d pnp_rvec(pnp_rvecs[0]);
  cv::Vec3d pnp_tvec(pnp_tvecs[0]);

  cv::Mat pnp_R, icp_R;
  cv::Rodrigues(pnp_rvec, pnp_R);
  cv::Rodrigues(icp_rvec, icp_R);

  Eigen::Matrix3d rotation_matrix(3, 3);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      rotation_matrix(i, j) = pnp_R.at<double>(i, j);
    }
  }
  Eigen::Vector3d pnp_eulerAngle = rotation_matrix.eulerAngles(2, 1, 0);
  cout << "pnp_eulerAngle" << pnp_eulerAngle << endl;

  Eigen::Quaterniond pnp_quat(rotation_matrix);
  //        cout<<pnp_quat.coeffs()<<endl;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      rotation_matrix(i, j) = icp_R.at<double>(i, j);
    }
  }

  Eigen::Quaterniond icp_quat(rotation_matrix);
  //        cout<<icp_quat.coeffs()<<endl;

  Eigen::Vector3d icp_eulerAngle = rotation_matrix.eulerAngles(2, 1, 0);
  cout << "icp_eulerAngle" << icp_eulerAngle << endl;

  std::cout << "pnp_tvec" << pnp_tvec << endl;
  std::cout << "icp_tvec" << icp_tvec << endl;

  Json::Value icp_pos_data, pnp_pose_data;

  icp_pos_data["x"] = icp_tvec[0];
  icp_pos_data["y"] = icp_tvec[1];
  icp_pos_data["z"] = icp_tvec[2];
  icp_pos_data["ox"] = icp_quat.x();
  icp_pos_data["oy"] = icp_quat.y();
  icp_pos_data["oz"] = icp_quat.z();
  icp_pos_data["ow"] = icp_quat.w();

  pnp_pose_data["x"] = pnp_tvec[0];
  pnp_pose_data["y"] = pnp_tvec[1];
  pnp_pose_data["z"] = pnp_tvec[2];
  pnp_pose_data["ox"] = pnp_quat.x();
  pnp_pose_data["oy"] = pnp_quat.y();
  pnp_pose_data["oz"] = pnp_quat.z();
  pnp_pose_data["ow"] = pnp_quat.w();

  pose_data["icp"] = icp_pos_data;
  pose_data["pnp"] = pnp_pose_data;
}

void CameraPoseCallback(std_msgs::String::ConstPtr msg,
                        ros::Publisher &pub_pose,
                        std::shared_ptr<Rs2Camera> rs2_cmera) {

  ROS_INFO_STREAM("进入回调函数");
  // msg:{"code":36,"solve_type":"pnp"} pnp,icp,mix
  Json::Reader jsonreader;
  Json::Value data;
  jsonreader.parse(msg->data, data);

  Json::Value pose_data;
  if (!data["code"].isNull()) {
    int aruco_id = data["code"].asInt();
    EstimateArucoPose(rs2_cmera, aruco_id, "pnp", pose_data);
  }

  std_msgs::String pubmsgs;
  pubmsgs.data = pose_data.toStyledString();
  ROS_INFO_STREAM("--------------auto reply info---------------\n"
                  << pubmsgs.data);
  while (pub_pose.getNumSubscribers() == 0)
    ;
  pub_pose.publish(pubmsgs);

  if ("pnp" == data["solve_type"].asString()) {

  }

  else if ("icp" == data["solve_type"].asString()) {

  } else if ("icp" == data["solve_type"].asString()) {

  }

  else {
  }
}

// 主函数
int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  try {
    ros::init(argc, argv, "camera_pose_talk_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("相机位姿采集节点");
    // 检测到的工作站标签位姿，会被发送到point_coordinate话题中
    ros::Publisher pub_pose = nh.advertise<std_msgs::String>("aruco_pose", 1);

    std::shared_ptr<Rs2Camera> rs2_camera = std::make_shared<Rs2Camera>();

    ros::Publisher pub_vs =
        nh.advertise<sensor_msgs::Image>("color_frame_sending", 1);

    ros::Subscriber sub_robot_pose = nh.subscribe<std_msgs::String>(
        "inquire_camera_pose", 10,
        std::bind(&CameraPoseCallback, std::placeholders::_1, pub_pose,
                  rs2_camera));
    ros::AsyncSpinner spinner(10);
    spinner.start();

    ros::Rate rate(10);
    while (ros::ok()) {
      //            ros::spinOnce();
      rs2::frameset video_frameset = rs2_camera->pipe_.wait_for_frames();
      auto color_frame = video_frameset.get_color_frame();
      const int color_fw = color_frame.as<rs2::video_frame>().get_width();
      const int color_fh = color_frame.as<rs2::video_frame>().get_height();
      // cout<< color_fw <<" "<<color_fh<<endl;
      cv::Mat color_f(cv::Size(color_fw, color_fh), CV_8UC3,
                      (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_f).toImageMsg();
      pub_vs.publish(*msg);
      rate.sleep();
    }

    return 0;
  }

  catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
