cmake_minimum_required(VERSION 3.0.2)
project(locator)


set(DEPENDENCIES ${OPENGL_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  realsense2_camera
  std_msgs
  cv_bridge
  image_transport
  gl_dependency
  message_generation
  tf
)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)

find_package(realsense2)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BlackPoint3D.msg
  RackHole.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES locator
   CATKIN_DEPENDS roscpp realsense2_camera std_msgs cv_bridge image_transport gl_dependency message_runtime tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})


add_executable(cvs5 src/chemist_visual_system_v5.cpp)
add_dependencies(cvs5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cvs5 ${catkin_LIBRARIES} ${DEPENDENCIES} ${realsense2_LIBRARY} ${OpenCV_LIBS} ${OPENGL_LIBRARIES})


