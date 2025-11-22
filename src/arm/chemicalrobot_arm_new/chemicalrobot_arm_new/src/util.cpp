#include "chemical_util.h"
#include <tf2/utils.h>

Isometry3d toMatrix(Vector3d &X, Quaterniond &Q)
{
  Isometry3d T = Isometry3d::Identity();
  T.pretranslate(X);
  T.rotate(Q);
  return T;
}

Isometry3d toMatrix(Vector3d &X, Vector3d &YPR)
{
  Quaterniond Q = AngleAxisd(YPR[0], Vector3d::UnitZ()) *
                  AngleAxisd(YPR[1], Vector3d::UnitY()) *
                  AngleAxisd(YPR[2], Vector3d::UnitX());
  return toMatrix(X, Q);
}

Isometry3d toMatrix(geometry_msgs::Pose &pose)
{
  Vector3d X;
  Quaterniond Q;
  X[0] = pose.position.x;
  X[1] = pose.position.y;
  X[2] = pose.position.z;
  Q.coeffs()[0] = pose.orientation.x;
  Q.coeffs()[1] = pose.orientation.y;
  Q.coeffs()[2] = pose.orientation.z;
  Q.coeffs()[3] = pose.orientation.w;
  return toMatrix(X, Q);
}

Isometry3d toMatrix(double pose[7])
{
  Vector3d X;
  Quaterniond Q;
  X[0] = pose[0];
  X[1] = pose[1];
  X[2] = pose[2];
  Q.coeffs()[0] = pose[3];
  Q.coeffs()[1] = pose[4];
  Q.coeffs()[2] = pose[5];
  Q.coeffs()[3] = pose[6];
  return toMatrix(X, Q);
}

geometry_msgs::Pose toPose(Vector3d &X, Quaterniond &Q)
{
  geometry_msgs::Pose pose;
  pose.position.x = X[0];
  pose.position.y = X[1];
  pose.position.z = X[2];
  pose.orientation.x = Q.coeffs()[0];
  pose.orientation.y = Q.coeffs()[1];
  pose.orientation.z = Q.coeffs()[2];
  pose.orientation.w = Q.coeffs()[3];
  return pose;
}

geometry_msgs::Pose toPose(Vector3d &X, Vector3d &YPR)
{
  Quaterniond Q = AngleAxisd(YPR[0], Vector3d::UnitZ()) *
                  AngleAxisd(YPR[1], Vector3d::UnitY()) *
                  AngleAxisd(YPR[2], Vector3d::UnitX());
  return toPose(X, Q);
}

geometry_msgs::Pose toPose(Isometry3d &T)
{
  Vector3d X(T.translation());
  Quaterniond Q(T.rotation());
  return toPose(X, Q);
}

myException::myException(vector<string> msgs, vector<double> relocation_err,
                         vector<int> exception_bottle, int rack_absnum)
{
  root1["IsDone"] = msgs[0];
  root1["Exception_"] = msgs[1];
  root1["Exception_bottle_all"] = Json::Value(Json::ValueType::arrayValue);
  root1["rack_num"] = rack_absnum;
  for (int i = 0; i < relocation_err.size(); ++i)
    root2[i] = relocation_err[i];
  root1["location_error"] = root2;
  // TODO:对接exception_bottle的返回形式,需要重写
  if (exception_bottle.size() == HOLE_MAX)
  {
    for (int i = 0; i < HOLE_MAX; ++i)
      root2[i] = exception_bottle[i];
    root1["Excption_bottle_all"] = root2;
  }
  _pubmsgs.data = root1.toStyledString();
}

std_msgs::String myException::what() { return _pubmsgs; }

string myException::getException() { return root1["Exception_"].asString(); }

Isometry3d remove_xy_rotaion(Isometry3d bTm,
                             geometry_msgs::Quaternion quaternion)
{
  Vector3d bXm = bTm.translation();
  Vector3d bEAm;
  bEAm = bTm.rotation().eulerAngles(2, 1, 0);
  bEAm[0] = tf2::getYaw(quaternion);
  bEAm[1] = 0;
  bEAm[2] = 0;
  bTm = toMatrix(bXm, bEAm);
  return bTm;
}

