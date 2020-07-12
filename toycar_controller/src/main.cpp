#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "controller.h"



int main(int argc, char** argv){
  ros::init(argc, argv, "toycar_control");
  controller c;
  c.CarController();
  return 0;
}
