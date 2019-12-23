#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "ToyCar.hpp"



int main(int argc, char** argv){
  ros::init(argc, argv, "toycar_publisher");
  ToyCar mycar;
  mycar.velocityController();
  return 0;
}
