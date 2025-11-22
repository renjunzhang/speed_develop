cmake_minimum_required(VERSION 3.0.2)
project(arm_eye)



## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_EXTENSIONS OFF)

# set(lib_DIR /usr/lib/x86_64-linux-gnu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        tf2
        tf2_ros
        tf2_geometry_msgs
        serial
        ur_dashboard_msgs
        ur_msgs
        std_msgs
        std_srvs
        actionlib_msgs
        tf2_kdl
        kdl_conversions
        rosbag
        pgi140
        geometry_msgs
        controller_manager_msgs
        robotiq_ft_sensor
        std_msgs
        message_generation
        cv_bridge
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ur_client_library REQUIRED)
# find_package(robotiq_ft_sensor)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
# find_package(${lib_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
        FILES
        DmsService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ur5e_motion_planning
        CATKIN_DEPENDS
        roscpp
        rospy
        moveit_core
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
        DEPENDS
        EIGEN3
        Boost
        ur_client_library
        cv_bridge

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/motion_planning.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ur5e_cp
        src/ur5e_main.cpp
        src/motion_planning.cpp
        src/dashboard.cpp
        src/motion_planning_util.cpp
        )
add_executable(record_cp
        src/record.cpp
        src/motion_planning.cpp
        src/dashboard.cpp
        src/motion_planning_util.cpp
        )
add_executable(mytest_cp
        src/mytest.cpp
        src/motion_planning.cpp
        src/dashboard.cpp
        src/motion_planning_util.cpp
        )

add_executable(ur5e_js_cp src/joystick_control.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ur5e_cp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(record_cp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mytest_cp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ur5e_js_cp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ur5e_cp
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${JSONCPP_LIBRARIES}
        )
target_link_libraries(record_cp
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${JSONCPP_LIBRARIES}
        )
target_link_libraries(mytest_cp
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${JSONCPP_LIBRARIES}
        )
target_link_libraries(ur5e_js_cp
        ${catkin_LIBRARIES}
        )



find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})	#添加OpenCV头文件


add_executable(eye_in_hand_cal
        src/eye_in_hand_cal.cpp

        )

target_link_libraries(eye_in_hand_cal ${catkin_LIBRARIES} ${DEPENDENCIES} ${OpenCV_LIBS})


find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(realsense2)
add_executable(eye_1 src/eye/eye.cpp)
add_dependencies(eye_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(eye_1 ${catkin_LIBRARIES} ${DEPENDENCIES} ${realsense2_LIBRARY}
        ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${JSONCPP_LIBRARIES} ${PCL_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chemicalrobot_arm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
