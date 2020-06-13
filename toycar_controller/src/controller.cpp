#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include "ros/ros.h"
#include "controller.h"

Controller::Controller(){}
Controller::~Controller(){}

  void velCallback(const  geometry_msgs::Twist::ConstPtr& msg)
  {
    vx_ = msg->linear.x;
    vtheta_ = msg->angular.z;
    if(vx_ > 10)
      velocity_ = 10;
    else if (vx < -10)
      velocity = -10;
    else
      velocity_ = vx;
    if(vtheta_ > 0.5)
      steer_ = 0.5;
    else if (vtheta_ < -0.5)
      steer_ = -0.5;
    else
      steer_ = vtheta_;
  }


  int main(int argc, char** argv){

    ros::init(argc, argv, "toycar_control");

    while(ros::ok()){
      left_wheel_pos.publish(steer_);
      right_wheel_pos.publish(steer_);
      left_wheel_vel.publish(velocity_);
      right_wheel_vel.publish(velocity_);
      ros::spinOnce();
      loop_.sleep();
    }
    return 0;
  }
