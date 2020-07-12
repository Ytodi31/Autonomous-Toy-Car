cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(toycar_controller)

## Compile as C++11, supported in ROS Kinetic and newer


find_package(catkin REQUIRED urdf move_base tf roscpp rospy roslaunch std_msgs)

catkin_package()


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)




add_executable(toycar_control src/controller.cpp src/main.cpp include/controller.h)

include_directories(include ${catkin_INCLUDE_DIRS})

## Specify libraries to link a library or executable target against
target_link_libraries(toycar_control ${catkin_LIBRARIES})
