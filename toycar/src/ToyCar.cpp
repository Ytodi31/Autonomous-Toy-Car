/**
*BSD 3-Clause License
*
*Copyright (c) 2019, Yashaarth Todi
*All rights reserved.
*
*Redistribution and use in source and binary forms, with or without
*modification, are permitted provided that the following conditions are met:
*1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*3. Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file ToyCar.cpp
 * @brief This file contains the definition of member functions of class ToyCar
 *
 * This project contains the execution to navigate a ToyCar using lidar data
 *
 * @copyright Copyright (c) Yashaarth Todi
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Yashaarth Todi
 *
 * @date 12-04-2019
 */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <std_msgs/Float64.h>
#include <time.h>
#include "ToyCar.hpp"
#include <nav_msgs/Odometry.h>

ToyCar::ToyCar() {}
ToyCar::~ToyCar() {}
void ToyCar::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i =0; i < msg -> ranges.size(); ++i) {
    // Checking if there is an obstacle in range of 1m of robot
    if (msg -> ranges[i] < 4) {
      // Setting robot state as unsafe to move
      ToyCar::robotState = 0;
      ROS_INFO_STREAM("Obstacle very close, turning around");
      return;
    }
    else {
      // Setting robot state as safe to move
      ToyCar::robotState = 1;
    }
  }
}

// void ToyCar::GetOdom(tf::StampedTransform mapToRobot) {
//   try {
//     mapToBaseTfListen.lookupTransform("/map", "scanmatcher_frame",
//                                       ros::Time(0), mapToRobot);
//   } catch (tf::TransformException ex) {
//   ROS_ERROR_STREAM("Exception : " << ex.what());
//   ros::Duration(1.0).sleep();
//   }
//   odom.header.stamp = ros::Time::now();
//   odom.header.frame_id = "map";
//   odom.pose.pose.position.x = mapToRobot.getOrigin().x();
//   odom.pose.pose.position.y = mapToRobot.getOrigin().y();
//   odom.pose.pose.position.z = mapToRobot.getOrigin().z();
//   odom.pose.pose.orientation.x = mapToRobot.getRotation().x();
//   odom.pose.pose.orientation.y = mapToRobot.getRotation().y();
//   odom.pose.pose.orientation.z = mapToRobot.getRotation().z();
//   odom.pose.pose.orientation.w = mapToRobot.getRotation().w();
//   odom.child_frame_id = "chassis";
//   odom.twist.twist.linear.x = velocity.data*cos(steer_angle.data*57.30);
//   odom.twist.twist.linear.y = velocity.data*sin(steer_angle.data*57.30);
//   odom.twist.twist.angular.z = velocity.data*9.84;
//  }
void ToyCar::odomCallback(const  geometry_msgs::PoseWithCovarianceStamped
::ConstPtr& msg)
{
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = msg -> pose.pose.position.x;
    odom.pose.pose.position.y = msg -> pose.pose.position.y;
    odom.pose.pose.position.z = msg -> pose.pose.position.z;
    odom.pose.pose.orientation.x = msg -> pose.pose.orientation.x;
    odom.pose.pose.orientation.y = msg -> pose.pose.orientation.y;
    odom.pose.pose.orientation.z = msg -> pose.pose.orientation.z;
    odom.pose.pose.orientation.w = msg -> pose.pose.orientation.w;
    // odom.pose.pose = msg -> pose
    odom.pose.covariance = msg -> pose.covariance;
    odom.child_frame_id = "chassis";
    odom.twist.twist.linear.x = velocity.data*cos(steer_angle.data*57.30);
    odom.twist.twist.linear.y = velocity.data*sin(steer_angle.data*57.30);
    odom.twist.twist.angular.z = velocity.data*9.84;
    odom.twist.covariance = msg -> pose.covariance;

}

// void ToyCar::GetOdom(){
//   ros::Subscriber odomsub = n.subscribe("/poseupdate", 1000, &ToyCar::odomCallback, this);
// }

void ToyCar::velocityController() {
  ros::Publisher left_wheel_pos = n.advertise<std_msgs::Float64>
  ("/chassis_build1/leftjoint_position_controller/command", 1000);
  ros::Publisher right_wheel_pos = n.advertise<std_msgs::Float64>
  ("/chassis_build1/rightjoint_position_controller/command", 1000);
  ros::Publisher left_wheel_vel = n.advertise<std_msgs::Float64>
  ("/chassis_build1/leftwheel_velocity_controller/command", 1000);
  ros::Publisher right_wheel_vel = n.advertise<std_msgs::Float64>
  ("/chassis_build1/rightwheel_velocity_controller/command", 1000);
  ros::Publisher lidar_control = n.advertise<std_msgs::Float64>
  ("/chassis_build1/lidar_controller/command", 1000);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/wheelodom", 1000);
  ros::Subscriber odomsub = n.subscribe("/poseupdate", 1000,
  &ToyCar::odomCallback, this);
  // Creating an object of geometry_msgs/twost type to access velocity

  std_msgs::Float64 lidar;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  tf::StampedTransform mapToRobot;


  // Defining a subscriber to read the laser scans from topic /scan
  auto sub = n.subscribe<sensor_msgs::LaserScan>
             ("/scan", 1000, &ToyCar::laserCallback, this);

  ros::Rate loop_rate(100);
  int multiplier = -1;
  // Checking for successful connection to ROS master
  while (ros::ok()) {

    current_time = ros::Time::now();
    lidar.data = 0;
    lidar_control.publish(lidar);
    ros::Time beginTime = ros::Time::now();
  // Checking if it is safe to move, assigns linear velocity to robot
  if (ToyCar::robotState == 1) {
    steer_angle.data = 0;
    velocity.data = 5/2;
    left_wheel_pos.publish(steer_angle);
    right_wheel_pos.publish(steer_angle);
    left_wheel_vel.publish(velocity);
    right_wheel_vel.publish(velocity);
    ROS_INFO_STREAM("No obstacle in 1m, moving ahead");
odom_pub.publish(odom);

  }

   if(ToyCar::robotState == 0)  {
    // If unsafe to move,assigns angular velocity for new heading direction
    velocity.data = 0;
    steer_angle.data = 0;
    ros::Time endTime = ros::Time::now() + ros::Duration(1.5);
    while(ros::Time::now() < endTime){
      left_wheel_pos.publish(steer_angle);
      right_wheel_pos.publish(steer_angle);
      ros::Duration(0.05).sleep();
      left_wheel_vel.publish(velocity);
      right_wheel_vel.publish(velocity);
      odom_pub.publish(odom);
      // GetOdom();
      // odom_pub.publish(odom);
    }
    velocity.data = 4.5/2;
    steer_angle.data = -0.6*multiplier;
    endTime = ros::Time::now() + ros::Duration(1.5);
    while(ros::Time::now() < endTime){
      left_wheel_pos.publish(steer_angle);
      right_wheel_pos.publish(steer_angle);
      ros::Duration(0.05).sleep();
      left_wheel_vel.publish(velocity);
      right_wheel_vel.publish(velocity);
      odom_pub.publish(odom);
      // GetOdom();
      // odom_pub.publish(odom);
    }
    velocity.data = 1.5/2;
    steer_angle.data = 0.01*multiplier;
    endTime = ros::Time::now() + ros::Duration(1.5);
    while(ros::Time::now() < endTime){
      left_wheel_pos.publish(steer_angle);
      right_wheel_pos.publish(steer_angle);
      ros::Duration(0.05).sleep();
      left_wheel_vel.publish(velocity);
      right_wheel_vel.publish(velocity);
      odom_pub.publish(odom);
    }
    multiplier = multiplier*multiplier;

  }
  last_time = current_time;
  ros::spinOnce();
  loop_rate.sleep();
  }
}
