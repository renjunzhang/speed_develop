#!/bin/zsh

# Check if the robot_name parameter is provided
# if [ -z "$1" ]; then
#     echo "Error: Missing robot_name"
#     echo "Usage: $0 <robot_name>"
#     exit 1
# fi
#--------------------------- platform END ------------------------------------


terminator  --new-tab -x "source ~/.zshrc;roslaunch locator cv_locator.launch;exec zsh;"
sleep 1s
terminator  --new-tab -x "source ~/.zshrc;roslaunch web_video_server web_video_server.launch;exec zsh"
sleep 1s
terminator  --new-tab -x "source ~/.zshrc;roslaunch chemical_arm_driver chemical_arm_driver.launch;exec zsh"
sleep 1s
terminator  --new-tab -x "source ~/.zshrc;roslaunch record_rtde record_rtde.launch;exec zsh"
sleep 1s


#------------------------ robot message bridge ---------------------------------
terminator  --new-tab -x "source ~/.zshrc;roslaunch robot_message_bridge robot_message_bridge.launch;exec zsh;"
sleep 1s
