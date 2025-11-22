#include <string>
#include <Eigen/Geometry>

#include <QtCore/QDebug>
#include <QtCore/QDir>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QCoreApplication>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <jsoncpp/json/json.h>


using namespace std;
using namespace Eigen;

// 单个试管架西林瓶数量
#define HOLE_MAX 10


/**
 * @brief 将平移向量和四元数转为变换矩阵
 *
 * @param X 平移向量
 * @param Q 四元数
 * @return Isometry3d 对应的变换矩阵
 */
Isometry3d toMatrix(Vector3d &X, Quaterniond &Q);

/**
 * @brief 将平移向量和YPR欧拉角转为变换矩阵
 *
 * @param X 平移向量
 * @param YPR YPR欧拉角
 * @return Isometry3d 对应的变换矩阵
 */
Isometry3d toMatrix(Vector3d &X, Vector3d &YPR);

/**
 * @brief 将geometry_msgs::Pose转为变换矩阵
 *
 * @param pose 待转换的pose
 * @return Isometry3d 对应的变换矩阵
 */
Isometry3d toMatrix(geometry_msgs::Pose &pose);

/**
 * @brief 将double pose[7]转为变换矩阵
 *
 * @param pose 待转换的pose
 * @return Isometry3d 对应的变换矩阵
 */
Isometry3d toMatrix(double pose[7]);

/**
 * @brief 将变换矩阵转为geometry_msgs::Pose
 *
 * @param T
 * @return geometry_msgs::Pose 对应的geometry_msgs::Pose
 */
geometry_msgs::Pose toPose(Isometry3d &T);

/**
 * @brief 将平移向量和YPR欧拉角转为geometry_msgs::Pose
 *
 * @param X 平移向量
 * @param YPR YPR欧拉角
 * @return geometry_msgs::Pose 对应的geometry_msgs::Pose
 */
geometry_msgs::Pose toPose(Vector3d &X, Vector3d &YPR);

/**
 * @brief 将平移向量和四元数转为geometry_msgs::Pose
 *
 * @param X 平移向量
 * @param Q 四元数
 * @return geometry_msgs::Pose 对应的geometry_msgs::Pose
 */
geometry_msgs::Pose toPose(Vector3d &X, Quaterniond &Q);

Isometry3d remove_xy_rotaion(Isometry3d bTm,
                             geometry_msgs::Quaternion quaternion);
/**
 * @brief 异常
 *
 */
class myException {
public:
  myException(vector<string> msgs = {"YES", ""},
              vector<double> relocation_err = {},
              vector<int> exception_bottle = {}, int rack_absnum = 0);
  std_msgs::String what();
  string getException();
  Json::Value root1, root2;
  std_msgs::String _pubmsgs;
};
