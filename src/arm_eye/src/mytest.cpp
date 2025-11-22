#include <chemicalrobot_arm/motion_planning.h>
void test01()
{
    sleep(1);
    time_t rawtime = time(NULL);
    char nowtime[64];
    strftime(nowtime,sizeof(nowtime),"[%Y-%m-%d %H:%M:%S]",localtime( &rawtime));
    ROS_INFO_STREAM(nowtime<<"INFO MESSAGE");
    ROS_WARN_STREAM(nowtime<<"WARN MESSAGE");
    ROS_ERROR_STREAM(nowtime<<"ERROR MESSAGE");
}

void test02(string s)
{
    vector<regex> defaultPose{regex("^Jrack[0-3]{1}_0$"),regex("^Jbottle[0-3]{2}_0$"),regex("^Jdefault_0$"),regex("^J[\u4e00-\u9fa5]*[站,架]_?[0-1]?"),regex("^Jrelocation_joints_[\u4e00-\u9fa5]*[站,架]$")};
    for_each(defaultPose.begin(),defaultPose.end(),[&s](regex a){cout<<regex_match(s, a)<<endl;});
}

geometry_msgs::PoseStamped transformpose()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // 等待从/point_coordinate的topic接收pose
    geometry_msgs::PoseStampedConstPtr camera_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/point_coordinate");
    // camera_pose->header.frame_id 表示camera_pose的参考坐标系
    
    // camera_pose – The object to transform.
    // "base_link" – The string identifer for the frame to transform into.
    // ros::Duration(1) – How long to wait for the target frame. Default value is zero (no blocking).
    geometry_msgs::PoseStamped marker_pose = tfBuffer.transform(*camera_pose,"base_link",ros::Duration(1));
    std::cout<<marker_pose<<endl;
    return marker_pose;
}

void test03()
{
    vector<int> Vname{1,2,3,4,5};
    Vname.erase(Vname.end(),Vname.end());
    for_each(Vname.rbegin(),Vname.rend(),[](int a){cout<<a<<endl;});
}

void test04(vector<vector<double>*> V_radius)
{
    for(size_t i=0;i<V_radius.size();++i)
    {
        vector<double>((*V_radius[i]).size(),0).swap(*V_radius[i]);
    }
}

void test04()
{
    Eigen::VectorXd qdot;
    Eigen::VectorXd twist(6);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr rmodel = robot_model_loader.getModel();
    robot_model::RobotStatePtr rstate = make_shared<robot_state::RobotState>(rmodel);
    // moveit::core::JointModelGroup* jmg = rstate->getJointModelGroup("manipulator");
    rstate->computeVariableVelocity(rmodel->getJointModelGroup("manipulator"),qdot,twist,rmodel->getLinkModel("tool0"));
}

void test05(ur5e_action& ur5e)
{
    // ur5e._dbptr->loose();
    // ros::Duration(1).sleep();
    // ur5e._dbptr->reset();
    // ros::Duration(1).sleep();
    // ur5e._dbptr->grasp();
    // ros::Duration(1).sleep();
    char a;
    cin>>a;
    if(a=='1')  ur5e._dbptr->close();
    else ur5e._dbptr->open();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mytest");
    ros::NodeHandle nh;
    shared_ptr<dashboardsrv_client> dbptr = make_shared<dashboardsrv_client>(nh);
    moveit::planning_interface::MoveGroupInterfacePtr mgptr = make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");
    ur5e_action ur5e(nh,dbptr,mgptr);
    setlocale(LC_ALL,"");
    while(ros::ok())
    {
        test05(ur5e);
    }
}