#include <chemicalrobot_arm/motion_planning.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planning");
    setlocale(LC_ALL,"");
    ros::NodeHandle nh;
    shared_ptr<dashboardsrv_client> dbptr = make_shared<dashboardsrv_client>(nh);
    moveit::planning_interface::MoveGroupInterfacePtr mgptr = make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");
    ur5e_action ur5e(nh,dbptr,mgptr);

    // ur5e_action ur5e;
    ros::AsyncSpinner spinner(10);
    // ur5e.AddCollisions();
    spinner.start();

    ur5e.Init("remote_control.urp\n");
    ROS_INFO_STREAM("-------------ready to receive command---------\n");
    // try{
    //     ur5e.Gripper_op(900,true,true);
    // }
    // catch(const string e){
    //     ROS_ERROR_STREAM(e);
    // }
    
    // ur5e.dashboard_ptr->reset();
    // ur5e.dashboard_ptr->grasp();
    // 设定目标位姿
    // geometry_msgs::PoseStamped next_pose;
    // next_pose.header.frame_id = "base_link";
    // next_pose.pose.position = ur5e.move_group_ptr->getCurrentPose().pose.position;
    // tf2::Quaternion q;
    // q.setRPY(0.0,1.5707963268-0.399264836,0.0);
    // // 0.399264836+
    // next_pose.pose.orientation = tf2::toMsg(q);
    // // next_pose.pose.orientation.x = 0;
    // // next_pose.pose.orientation.y = sqrt(2)/2;
    // // next_pose.pose.orientation.z = 0;
    // // next_pose.pose.orientation.w = 1;
    // // tf2_ros::Buffer tfBuffer;
    // // tf2_ros::TransformListener tfListener(tfBuffer);
    // // geometry_msgs::PoseStamped relocate_pose = tfBuffer.transform(next_pose,"base_link",ros::Duration(1));
    // // relocate_pose.pose.position.z += 0.4;
    // // relocate_pose.pose.orientation = ur5e.move_group_ptr->getCurrentPose().pose.orientation;
    
    // ur5e.move_group_ptr->setPoseTarget(next_pose);
    // ur5e.move_group_ptr->move();

    // 设定关节角
    // 重定位的关节角
    // ur5e.move_group_ptr->setJointValueTarget(vector<double>{
    //         3.1529130935668945,
    //         -1.9925133190550746,
    //         -2.4556121826171875,
    //         -1.8338715038695277,
    //         -1.5506122748004358,
    //         3.1457235813140869});
    // ur5e.move_group_ptr->move();
    // auto RPY = ur5e.move_group_ptr->getCurrentRPY();
    // for(auto each:RPY){
    //     cout<<each<<endl;
    // }

    // thread isdrop(&dashboardsrv_client::PGI_isDrop,dbptr);
    // future<bool> _drop_flag;
    while (ros::ok()){
        // char a;
        // cin>>a;
        // if(a=='1')
        // dbptr->close();
        // else
        // dbptr->open();
        // if(ur5e._grasp_flag)
        // {
        //     // isdrop.join();
        //     if(dbptr->PGI_isDrop())
        //     {
        //         ur5e._mgptr->stop();
        //         ur5e.JsonPub(vector<string>{"NO","grasp_drop"});
        //     }
        // }
    };
}
