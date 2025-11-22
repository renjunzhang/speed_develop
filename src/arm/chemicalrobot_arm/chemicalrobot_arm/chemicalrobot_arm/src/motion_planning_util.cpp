#include <chemicalrobot_arm/motion_planning.h>

void ur5e_action::LoadStationInfo(string station_name) {
  string file_path = this->_file_root + station_name + ".json";
  Json::Reader jsonreader;
  Json::Value root;
  ifstream file(file_path, ios::binary);
  if (jsonreader.parse(file, root)) {
    cout << "pose value found" << endl;
    // 获取仪器关节角
    Json::Value JOINTS = root["joints"];
    string joints_name;
    for (int i = 0; i < JOINTS.size(); ++i) {
      joints_name = JOINTS[i]["name"].asString();
      // cout << joints_name << endl;
      this->_Inst_joints[joints_name].resize(6);
      for (int j = 0; j < 6; ++j)
        this->_Inst_joints[joints_name].at(j) =
            JOINTS[i]["value"][j].asDouble();
    }
    // 获取仪器相对标签位姿
    Json::Value POSES = root["poses"];
    geometry_msgs::Pose named_pose;
    string pose_name;
    for (int i = 0; i < POSES.size(); ++i) {
      pose_name = POSES[i]["name"].asString();
      named_pose.position.x = POSES[i]["value"][0].asDouble();
      named_pose.position.y = POSES[i]["value"][1].asDouble();
      named_pose.position.z = POSES[i]["value"][2].asDouble();
      named_pose.orientation.x = POSES[i]["value"][3].asDouble();
      named_pose.orientation.y = POSES[i]["value"][4].asDouble();
      named_pose.orientation.z = POSES[i]["value"][5].asDouble();
      named_pose.orientation.w = POSES[i]["value"][6].asDouble();
      this->_Inst_relPoses[pose_name] = named_pose;
    }
    // 获取marker_id
    Json::Value MARKER = root["marker"];
    _marker_id = MARKER["id"].asInt();
    // marker_size[0] = MARKER["size"][0].asDouble();
    // marker_size[1] = MARKER["size"][1].asDouble();
  } else {
    cout << "none pose values found" << endl;
  }
  file.close();
}

void ur5e_action::LoadRobotJointInfo() {
  string file_path = this->_file_root + "机械臂关节.json";
  Json::Reader jsonreader;
  Json::Value root;
  ifstream file(file_path, ios::binary);
  if (jsonreader.parse(file, root)) {
    // 获取关节角
    Json::Value JOINTS = root["joints"];
    string joints_name;
    for (int i = 0; i < JOINTS.size(); ++i) {
      joints_name = JOINTS[i]["name"].asString();
      this->_Robot_joints[joints_name].resize(6);
      for (int j = 0; j < 6; ++j)
        this->_Robot_joints[joints_name].at(j) =
            JOINTS[i]["value"][j].asDouble();
    }
  }
  file.close();
}

vector<double> ur5e_action::GetJoints(string name) {
  if (_Inst_joints.find(name) != _Inst_joints.end()) {
    return (_Inst_joints.at(name));
  } else if (_Robot_joints.find(name) != _Robot_joints.end()) {
    return (_Robot_joints.at(name));
  } else {
    ROS_ERROR_STREAM("no_joints_value:"+name);
    throw(myException(vector<string>{"NO", "internal_error"}));
  }
}

void ur5e_action::safety_Callback(
    const ur_dashboard_msgs::SafetyModeConstPtr &safetymode) {
  //cout << "safety_Callback = " << safetymode->mode << endl;
  this->_safety_flag = safetymode->mode;
}

void ur5e_action::exec_Callback(
    const moveit_msgs::ExecuteTrajectoryActionResultConstPtr &execresult) {
  this->_exec_flag = execresult->result.error_code.val;
}

myException::myException(vector<string> msgs, vector<double> relocation_err,
                         vector<int> exception_bottle, int rack_absnum) {
  root1["IsDone"] = msgs[0];
  root1["Exception_"] = msgs[1];
  root1["Exception_bottle_all"] = Json::Value(Json::ValueType::arrayValue);
  root1["rack_num"] = rack_absnum;
  for (int i = 0; i < relocation_err.size(); ++i)
    root2[i] = relocation_err[i];
  root1["location_error"] = root2;
  // TODO:对接exception_bottle的返回形式,需要重写
  if (exception_bottle.size() == HOLE_MAX) {
    for (int i = 0; i < HOLE_MAX; ++i)
      root2[i] = exception_bottle[i];
    root1["Excption_bottle_all"] = root2;
  }
  _pubmsgs.data = root1.toStyledString();
}

std_msgs::String myException::what() { return _pubmsgs; }

string myException::getException() { return root1["Exception_"].asString(); }

Isometry3d toMatrix(Vector3d &X, Quaterniond &Q) {
  Isometry3d T = Isometry3d::Identity();
  T.pretranslate(X);
  T.rotate(Q);
  return T;
}

Isometry3d toMatrix(Vector3d &X, Vector3d &YPR) {
  Quaterniond Q = AngleAxisd(YPR[0], Vector3d::UnitZ()) *
                  AngleAxisd(YPR[1], Vector3d::UnitY()) *
                  AngleAxisd(YPR[2], Vector3d::UnitX());
  return toMatrix(X, Q);
}

Isometry3d toMatrix(geometry_msgs::Pose &pose) {
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

geometry_msgs::Pose toPose(Vector3d &X, Quaterniond &Q) {
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

geometry_msgs::Pose toPose(Vector3d &X, Vector3d &YPR) {
  Quaterniond Q = AngleAxisd(YPR[0], Vector3d::UnitZ()) *
                  AngleAxisd(YPR[1], Vector3d::UnitY()) *
                  AngleAxisd(YPR[2], Vector3d::UnitX());
  return toPose(X, Q);
}

geometry_msgs::Pose toPose(Isometry3d &T) {
  Vector3d X(T.translation());
  Quaterniond Q(T.rotation());
  return toPose(X, Q);
}

// void
// ur5e_action::setAvgCartesianSpeed(moveit::planning_interface::MoveGroupInterface::Plan
// &plan, const string end_effector, const double speed)
// {
//     robot_model_loader::RobotModelLoader
//     robot_model_loader("robot_description"); robot_model::RobotModelPtr
//     kinematic_model = robot_model_loader.getModel();
//     robot_state::RobotStatePtr _kinematic_state(new
//     robot_state::RobotState(kinematic_model));
//     _kinematic_state->setToDefaultValues();

//     int num_waypoints = plan.trajectory_.joint_trajectory.points.size();
//     //gets the number of waypoints in the trajectory const vector<string>
//     joint_names = plan.trajectory_.joint_trajectory.joint_names;    //gets
//     the names of the joints being updated in the trajectory

//     //set joint positions of zeroth waypoint
//     _kinematic_state->setVariablePositions(joint_names,
//     plan.trajectory_.joint_trajectory.points.at(0).positions);

//     Eigen::Affine3d current_end_effector_state =
//     _kinematic_state->getGlobalLinkTransform(end_effector); Eigen::Affine3d
//     next_end_effector_state; double euclidean_distance, new_timestamp,
//     old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
//     trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint,
//     *next_waypoint;

//     for(int i = 0; i < num_waypoints - 1; i++)      //loop through all
//     waypoints
//     {
//         curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
//         next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);

//         //set joints for next waypoint
//         _kinematic_state->setVariablePositions(joint_names,
//         next_waypoint->positions);

//         //do forward kinematics to get cartesian positions of end effector
//         for next waypoint next_end_effector_state =
//         _kinematic_state->getGlobalLinkTransform(end_effector);

//         //get euclidean distance between the two waypoints
//         euclidean_distance = pow(pow(next_end_effector_state.translation()[0]
//         - current_end_effector_state.translation()[0], 2) +
//                             pow(next_end_effector_state.translation()[1] -
//                             current_end_effector_state.translation()[1], 2) +
//                             pow(next_end_effector_state.translation()[2] -
//                             current_end_effector_state.translation()[2], 2),
//                             0.5);

//         new_timestamp = curr_waypoint->time_from_start.toSec() +
//         (euclidean_distance / speed);      //start by printing out all 3 of
//         these! old_timestamp = next_waypoint->time_from_start.toSec();

//         //update next waypoint timestamp & joint velocities/accelerations if
//         joint velocity/acceleration constraints allow if(new_timestamp >
//         old_timestamp)
//             next_waypoint->time_from_start.fromSec(new_timestamp);
//         else
//         {
//             // time_t rawtime = time(NULL);
//             // char temp[64];
//             // strftime(nowtime,sizeof(nowtime),"[%Y-%m-%d
//             %H:%M:%S]",localtime( &rawtime));
//             //ROS_WARN_NAMED(temp<<"setAvgCartesianSpeed", "Average speed is
//             too fast. Moving as fast as joint constraints allow.");
//         }

//         //update current_end_effector_state for next iteration
//         current_end_effector_state = next_end_effector_state;
//     }

//     //now that timestamps are updated, update joint velocities/accelerations
//     (used updateTrajectory from iterative_time_parameterization as a
//     reference) for(int i = 0; i < num_waypoints; i++)
//     {
//         curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
//         //set current, previous & next waypoints if(i > 0)
//             prev_waypoint =
//             &plan.trajectory_.joint_trajectory.points.at(i-1);
//         if(i < num_waypoints-1)
//             next_waypoint =
//             &plan.trajectory_.joint_trajectory.points.at(i+1);

//         if(i == 0)          //update dt's based on waypoint (do this outside
//         of loop to save time)
//             dt1 = dt2 = next_waypoint->time_from_start.toSec() -
//             curr_waypoint->time_from_start.toSec();
//         else if(i < num_waypoints-1)
//         {
//             dt1 = curr_waypoint->time_from_start.toSec() -
//             prev_waypoint->time_from_start.toSec(); dt2 =
//             next_waypoint->time_from_start.toSec() -
//             curr_waypoint->time_from_start.toSec();
//         }
//         else
//             dt1 = dt2 = curr_waypoint->time_from_start.toSec() -
//             prev_waypoint->time_from_start.toSec();

//         for(int j = 0; j < joint_names.size(); j++)     //loop through all
//         joints in waypoint
//         {
//             if(i == 0)                      //first point
//             {
//                 q1 = next_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = q1;
//             }
//             else if(i < num_waypoints-1)    //middle points
//             {
//                 q1 = prev_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = next_waypoint->positions.at(j);
//             }
//             else                            //last point
//             {
//                 q1 = prev_waypoint->positions.at(j);
//                 q2 = curr_waypoint->positions.at(j);
//                 q3 = q1;
//             }

//             if(dt1 == 0.0 || dt2 == 0.0)
//                 v1 = v2 = a = 0.0;
//             else
//             {
//                 v1 = (q2 - q1)/dt1;
//                 v2 = (q3 - q2)/dt2;
//                 a = 2.0*(v2 - v1)/(dt1+dt2);
//             }

//             //actually set the velocity and acceleration
//             curr_waypoint->velocities.at(j) = (v1+v2)/2;
//             curr_waypoint->accelerations.at(j) = a;
//         }
//     }
// }
