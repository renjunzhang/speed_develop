#include <chemicalrobot_arm/motion_planning.h>

/**
 * @brief 离心机孔位信息
 *
 */
class hole_cls {
public:
  hole_cls(int num) {
    Vector3d mXh2(0.00, 0.00, 0.00);
    Vector3d mEAh2;
    if (num == 0) {
      mEAh2 << -2 * M_PI / 3, 0.0, 0.0;
      _fix_seq1.emplace_back(3);
      _fix_seq1.emplace_back(4);
      _fix_seq1.emplace_back(5);
      _fix_seq2.emplace_back(6);
      _fix_angle = 2 * M_PI / 3;
    } else if (num == 1) {
      mEAh2 << 0.0, 0.0, 0.0;
      _fix_seq1.emplace_back(7);
      _fix_seq1.emplace_back(8);
      _fix_seq1.emplace_back(9);
      _fix_seq2.emplace_back(10);
      _fix_angle = 0;
    } else if (num == 2) {
      mEAh2 << 2 * M_PI / 3, 0.0, 0.0;
      _fix_seq1.emplace_back(11);
      _fix_seq1.emplace_back(0);
      _fix_seq1.emplace_back(1);
      _fix_seq2.emplace_back(2);
      _fix_angle = -2 * M_PI / 3;
    }
    Isometry3d mTh2 = toMatrix(mXh2, mEAh2);
    Vector3d mXh1(-0.05, 0.00, 0.2076);
    Vector3d mEAh1(0.0, -2.74159265, 0.0);
    Isometry3d mTh1 = toMatrix(mXh1, mEAh1);
    _mTh = mTh2 * mTh1;
  }
  Isometry3d _mTh;
  vector<int> _fix_seq1;
  vector<int> _fix_seq2;
  double _fix_angle;
};

void SaveStationJoint(string file_path, string name, vector<double> joints) {
  Json::Reader jsonreader;
  Json::Value root;
  Json::StyledWriter writer;
  ifstream ifile(file_path, ios::binary);
  if (jsonreader.parse(ifile, root)) {
    int i;
    for (i = 0; i < root["joints"].size(); ++i) {
      if (root["joints"][i]["name"].asString() == name)
        break;
    }
    Json::Value joint;
    joint["name"] = Json::Value(name);
    for (int j = 0; j < 6; ++j) {
      joint["value"][j] = Json::Value(joints[j]);
    }
    if (i < root["joints"].size()) {
      root["joints"][i] = joint;
      cout << file_path << "\n修改关节角:\n" << joint.toStyledString() << endl;
    } else {
      root["joints"].append(joint);
      cout << file_path << "\n添加关节角:\n" << joint.toStyledString() << endl;
    }
    ifile.close();
    ofstream ofile(file_path, ios::binary);
    ofile << writer.write(root);
    ofile.close();
  }
}

void SaveStationPose(string file_path, string name, Vector3d X, Quaterniond Q) {
  Json::Reader jsonreader;
  Json::Value root;
  Json::StyledWriter writer;
  ifstream ifile(file_path, ios::binary);
  if (jsonreader.parse(ifile, root)) {
    int i;
    for (i = 0; i < root["poses"].size(); ++i) {
      if (root["poses"][i]["name"].asString() == name)
        break;
    }
    Json::Value pose;
    pose["name"] = Json::Value(name);
    pose["value"][0] = Json::Value(X[0]);
    pose["value"][1] = Json::Value(X[1]);
    pose["value"][2] = Json::Value(X[2]);

    pose["value"][3] = Json::Value(Q.coeffs()[0]);
    pose["value"][4] = Json::Value(Q.coeffs()[1]);
    pose["value"][5] = Json::Value(Q.coeffs()[2]);
    pose["value"][6] = Json::Value(Q.coeffs()[3]);
    if (i < root["poses"].size()) {
      root["poses"][i] = pose;
      cout << file_path << "\n修改位姿:\n" << pose.toStyledString() << endl;
    } else {
      root["poses"].append(pose);
      cout << file_path << "\n添加位姿:\n" << pose.toStyledString() << endl;
    }
    ifile.close();
    ofstream ofile(file_path, ios::binary);
    ofile << writer.write(root);
    ofile.close();
  }
}

void Go(ur5e_action &ur5e) {
  string name;
  while (ros::ok()) {
    cout << "输入前往的位置(q退出)" << endl;
    cin >> name;
    if (name == string("q"))
      break;
    char flag = name.at(0);
    string pos_name = name.substr(1);
    vector<string> V_name{name};
    vector<double> V_radius{0.0};
    vector<double> V_speed{NORMAL_SPEED};
    vector<double> V_acc{NORMAL_ACC};
    vector<string> V_planner{"PTP"};
    try {
      ur5e.SequenceValidate_pilz(vector<vector<string> *>{&V_name},
                                 vector<vector<double> *>{&V_radius},
                                 vector<vector<double> *>{&V_speed},
                                 vector<vector<double> *>{&V_acc},
                                 vector<vector<string> *>{&V_planner});
      ur5e.SequenceMove_pilz(V_name, V_radius, V_speed, V_acc, V_planner, false,
                             true);
    } catch (myException &e) {
      if (e.getException() != string("")) {
        cout << e.what() << endl;
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "record_pose");
  setlocale(LC_ALL, "");
  ros::NodeHandle nh;
  shared_ptr<dashboardsrv_client> dbptr = make_shared<dashboardsrv_client>(nh);
  moveit::planning_interface::MoveGroupInterfacePtr mgptr =
      make_shared<moveit::planning_interface::MoveGroupInterface>(
          "manipulator");
  ur5e_action ur5e(nh, dbptr, mgptr);
  ros::AsyncSpinner spinner(10);
  spinner.start();
  geometry_msgs::Pose target_pose;
  string curr_station, name, flag;
  ur5e._curr_station = "Jdefault_0";
  cout << "输入P记录位姿,输入J记录关节角" << endl;
  cin >> flag;
  cout << "输入需要记录的站点" << endl;
  cin >> curr_station;
  try {
    if (curr_station == "滴液制样") {
      ur5e.LoadStationInfo("电催化工作站");
    }
    if (flag == string("P")) {
      char aa;
      Isometry3d bTm0, bTm;
      cout << "正在采集重定位点的位姿" << endl << endl;
      if (curr_station == "离心孔") {
        int HOLE = 2;
        hole_cls holeclass(HOLE);
        // Tool0到figer_link(夹爪末端)的变换
        // Vector3d tXf(0.0, 0.143, 0.074);
        Vector3d tXf(0.0, 0.0, 0.0);
        Vector3d tEAf(0.0, 0.0, -M_PI / 2);
        Isometry3d tTf = toMatrix(tXf, tEAf);
        Isometry3d mTt = holeclass._mTh * tTf.inverse();
        ur5e.LoadStationInfo("离心工作站");
        vector<string> V_name_{"J离心工作站", "Jrelocation_joints_离心工作站"};
        vector<double> V_radius_{0.0, 0.0};
        vector<double> V_speed_{NORMAL_SPEED, NORMAL_SPEED};
        vector<double> V_acc_{NORMAL_ACC, NORMAL_ACC};
        vector<string> V_planner_{"PTP", "PTP"};
        ur5e.SequenceMove_pilz(V_name_, V_radius_, V_speed_, V_acc_, V_planner_,
                               false, true);
        // ur5e.GotoDefaultPose(string("relocation_joints_离心工作站"));
        ur5e.CalculatePoses("离心工作站");
        ur5e.LoadStationInfo("离心孔");
        ur5e.GotoDefaultPose(string("离心工作站"));
        cout << "手动打开离心机盖,之后输入字符继续" << endl;
        cin >> aa;
        aa = 'a';

        while (ros::ok()) {
          cout << "移动到下一位置之后,再输入需要记录的位置名称(Q退出,g前往,"
                  "r观察转筒,其他将记录)"
               << endl;
          cin >> name;
          if (name == string("r")) {
            Isometry3d bTt;
            moveit::planning_interface::MoveGroupInterface::Plan plan_;
            ros::Publisher pub_;
            pub_ = nh.advertise<geometry_msgs::PoseStamped>("/hole_pose", 1);
            vector<string> V_name{"P离心工作站_2"};
            vector<double> V_radius{0.0};
            vector<double> V_speed{NORMAL_SPEED};
            vector<double> V_acc{NORMAL_ACC};
            vector<string> V_planner{"PTP"};
            try {
              // ur5e.SequenceValidate_pilz(vector<vector<string> *>{&V_name},
              // 												vector<vector<double> *>{&V_radius},
              // 												vector<vector<double> *>{&V_speed},
              // 												vector<vector<double> *>{&V_acc},
              // 												vector<vector<string> *>{&V_planner});
              ur5e.SequenceMove_pilz(V_name, V_radius, V_speed, V_acc,
                                     V_planner, false, true);
            } catch (myException &e) {
              if (e.getException() != string("")) {
                cout << e.what() << endl;
              }
            }
            while (aa != 'q') {
              bTm = ur5e.CalculatePoses("离心孔");
              bTt = bTm * mTt;
              // 根据yaw角差值动态补偿
              Isometry3d mTm1 = bTm.inverse() * toMatrix(ur5e._CTF_pose);
              geometry_msgs::Pose mpose = toPose(mTm1);
              double dyaw = tf2::getYaw(mpose.orientation) + M_PI;
              int seq = floor(dyaw / (M_PI / 6));
              for (int each : holeclass._fix_seq1) {
                if (each == seq) {
                  dyaw -= M_PI / 3;
                  break;
                }
              }
              for (int each : holeclass._fix_seq2) {
                if (each == seq) {
                  dyaw -= M_PI / 3;
                  break;
                }
              }
              Vector3d X(0.0, 0.0, 0.0);
              Vector3d YPR(0.0, -(dyaw + holeclass._fix_angle), 0.0);
              Isometry3d T = toMatrix(X, YPR);
              bTt = bTt * T;
              Vector3d X1(-0.1, 0.1, 0.0);
              Vector3d YPR1(0.0, 0.0, 0.0);
              Isometry3d T1 = toMatrix(X1, YPR1);
              bTt = T1 * bTt;
              geometry_msgs::PoseStamped pose_;
              pose_.pose = toPose(bTt);
              pose_.header.frame_id = "base_link";
              pub_.publish(pose_);
              ur5e._mgptr->setPoseTarget(toPose(bTt));
              ur5e._mgptr->plan(plan_);
              cout << "旋转转筒,q退出。" << endl;
              cin >> aa;
            }
            aa = 'a';
            // 只有在没有数据的时候再进行move
            cout << "执行轨迹,输入y" << endl;
            cin >> aa;
            if (aa == 'y') {
              ur5e._mgptr->execute(plan_);
            }
          } else if (name == string("Q")) {
            return 0;
          } else if (name == string("g")) {
            Go(ur5e);
          } else {
            target_pose = ur5e._mgptr->getCurrentPose().pose;
            vector<double> joints = ur5e._mgptr->getCurrentJointValues();
            Isometry3d bTt = toMatrix(target_pose);
            Isometry3d mTt = bTm.inverse() * bTt;
            Quaterniond mQt(mTt.rotation());
            Vector3d mXt(mTt.translation());
            SaveStationJoint(ur5e._file_root + curr_station + ".json",
                             name + "_inverse", joints);
            SaveStationPose(ur5e._file_root + curr_station + ".json", name, mXt,
                            mQt);
          }
        }
      } else {
        ur5e._curr_station = curr_station;
        ur5e.LoadStationInfo(ur5e._curr_station);
        vector<string> V_name1{"Jrelocation_joints_" + curr_station};
        vector<double> V_radius1{0.0};
        vector<double> V_speed1{NORMAL_SPEED};
        vector<double> V_acc1{NORMAL_ACC};
        vector<string> V_planner1{"PTP"};
        ur5e.SequenceValidate_pilz(vector<vector<string> *>{&V_name1},
                                   vector<vector<double> *>{&V_radius1},
                                   vector<vector<double> *>{&V_speed1},
                                   vector<vector<double> *>{&V_acc1},
                                   vector<vector<string> *>{&V_planner1});
        ur5e.SequenceMove_pilz(V_name1, V_radius1, V_speed1, V_acc1, V_planner1,
                               false, true);
        bTm0 = ur5e.CalculatePoses(curr_station);
        cout << "bTm:\n" << bTm0.matrix() << endl << endl;
        while (ros::ok()) {
          cout << "移动到下一位置之后,再输入需要记录的位置名称(Q退出,g前往)"
               << endl;
          cin >> name;
          if (name == string("Q"))
            return 0;
          else if (name == string("g")) {
            Go(ur5e);
          } else {
            target_pose = ur5e._mgptr->getCurrentPose().pose;
            Isometry3d bTt = toMatrix(target_pose);
            Isometry3d mTt = bTm0.inverse() * bTt;
            Quaterniond mQt(mTt.rotation());
            Vector3d mXt(mTt.translation());
            vector<double> joints = ur5e._mgptr->getCurrentJointValues();
            SaveStationJoint(ur5e._file_root + curr_station + ".json",
                             name + "_inverse", joints);
            SaveStationPose(ur5e._file_root + curr_station + ".json", name, mXt,
                            mQt);
          }
        }
      }
    }

    else if (flag == string("J")) {
      ur5e.LoadStationInfo(curr_station);
      while (ros::ok()) {
        cout << "移动到下一位置之后,再输入需要记录的位置名称(Q退出,g前往)"
             << endl;
        cin >> name;
        if (name == string("Q"))
          return 0;
        else if (name == string("g")) {
          Go(ur5e);
        } else {
          vector<double> joints = ur5e._mgptr->getCurrentJointValues();
          // vector<regex> RobotRegex{regex("^rack.*"), regex("^hole.*"),
          // regex("^default.*")}; bool matchflag = false; for (auto each :
          // RobotRegex)
          // {
          // 	if (regex_match(name, each))
          // 	{
          // 		matchflag = true;
          // 		break;
          // 	}
          // }
          curr_station == "机械臂关节"
              ? SaveStationJoint(ur5e._file_root + "机械臂关节.json", name,
                                 joints)
              : SaveStationJoint(ur5e._file_root + curr_station + ".json", name,
                                 joints);
        }
      }
    }

    else {
      cout << "输入错误" << endl;
    }
  } catch (myException &e) {
    std::cerr << e.getException() << '\n';
  }
}
