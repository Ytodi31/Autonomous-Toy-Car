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
 * @file ToyCar.hpp
 * @brief This file contains the class declaration for class ToyCar
 *
 * This project contains the execution to navigate a ToyCar using lidar data
 *
 * @copyright Copyright (c) Yashaarth Todi
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Yashaarth Todi
 *
 * @date 12-06-2020
 */
#ifndef INCLUDE_CONTROLLER_H
#define INCLUDE_CONTROLLER_H

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

class Controller {
public:
  Controller() : vx_(0), vy_(0), vtheta_(0){}
  ~Controller(){}

  ros::Publisher left_wheel_pos = n.advertise<std_msgs::Float64>
  ("/toycar/leftjoint_position_controller/command", 1000);

  ros::Publisher right_wheel_pos = n.advertise<std_msgs::Float64>
  ("/toycar/rightjoint_position_controller/command", 1000);

  ros::Publisher left_wheel_vel = n.advertise<std_msgs::Float64>
  ("/toycar/leftwheel_velocity_controller/command", 1000);

  ros::Publisher right_wheel_vel = n.advertise<std_msgs::Float64>
  ("/toycar/rightwheel_velocity_controller/command", 1000);

  ros::NodeHandle n;

  ros::Subscriber velsub = n.subscribe("/cmd_vel", 1000,
  &ToyCar::velCallback, this);

  void velCallback(const  geometry_msgs::Twist::ConstPtr& msg);

private:
  float vx_;
  float vtheta_;
  ros::Rate loop_(100);
  float velocity_ =0;
  float steer_ = 0;

};

#endif
