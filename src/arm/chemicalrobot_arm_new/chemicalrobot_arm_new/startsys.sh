#!/bin/bash
terminator --new-tab -x "cd ~/ursim-5.13.1.1131001;./start-ursim.sh;exec bash;"
sleep 10s

terminator --new-tab -x "source ~/.bashrc;roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=172.26.202.12;exec bash;"
sleep 5s

terminator --new-tab -x "roslaunch ur5e_tfsensor_pgi140_moveit_config moveit_planning_execution.launch;exec bash;"
sleep 1s

terminator --new-tab -x "roslaunch locator cv_locator.launch;exec bash;"
sleep 1s

terminator --new-tab -x "roslaunch chemicalrobot_arm_new chemicalrobot_arm_new.launch have_pgi:=true;exec bash;"
sleep 1s

terminator --new-tab -x "source ~/.bashrc;
                         export LIBGL_ALWAYS_INDIRECT=0;
                         export LIBGL_ALWAYS_SOFTWARE=1;
                         roslaunch ur_robot_driver example_rviz.launch;
                         exec bash;"
#terminator --new-tab -x "rosrun high_throughput_platform_message_bridge dms_to_platform_robot.py;exec bash;"

