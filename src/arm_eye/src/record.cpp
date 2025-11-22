#include <chemicalrobot_arm/motion_planning.h>




class TestUr{
public:
    ros::NodeHandle nh_;
    TestUr(ros::NodeHandle nh)
    {
        nh_=nh;
        ROS_INFO("TestUr被构造");

    }
    ~TestUr()
    {
        ROS_INFO("TestUr被析构");

    }

    geometry_msgs::PoseStamped GetPose()
    {

        geometry_msgs::PoseStamped tmp_pose;

        return tmp_pose;

    }


};

void InquireRobotPose(const std_msgs::String::ConstPtr& msg_p,ros::Publisher &pub_robot_pose,shared_ptr<ur5e_action> ur5e)
{


    ROS_INFO("进入回调函数");
//    pub_robot_pose.publish(ur5e._mgptr->getCurrentPose());
//    pub_robot_pose.publish(ur5e.GetPose());
    geometry_msgs::PoseStamped tmp_pose;
    while (pub_robot_pose.getNumSubscribers() == 0)
        ;
    pub_robot_pose.publish(ur5e->_mgptr->getCurrentPose());

    ROS_INFO("退出回调函数");



//    std_msgs::Int32 op_code;
//    op_code.data = 30;
//    // 等待第一次重定位的位姿
//    ros::Publisher _pub2_ = nh.advertise<std_msgs::Int32>("/camera_operation", 1);
//    while (_pub2_.getNumSubscribers() == 0)
//        ;
//    _pub2_.publish(op_code);
//    geometry_msgs::PoseStampedConstPtr camera_pose =
//            ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
//                    "/point_coordinate");


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_pose");
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;
    ros::Publisher pub_robot_pose = nh.advertise<geometry_msgs::PoseStamped>("answer_robot_pose",10);


   shared_ptr<dashboardsrv_client> dbptr = make_shared<dashboardsrv_client>(nh);
   moveit::planning_interface::MoveGroupInterfacePtr mgptr =
     make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");


    shared_ptr<ur5e_action> ur5e = make_shared<ur5e_action>(nh, dbptr, mgptr);

    //ur5e->_mgptr->startStateMonitor();
    auto call_fun=std::bind(&InquireRobotPose,std::placeholders::_1, pub_robot_pose,ur5e);



    ros::Subscriber sub_robot_pose = nh.subscribe<std_msgs::String>("inquire_robot_pose",10,
                                                                    call_fun);



  ros::AsyncSpinner spinner(10);
  spinner.start();
    while (ros::ok()){


    };

}
