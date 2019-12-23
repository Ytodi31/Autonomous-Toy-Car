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
 * @date 12-04-2019
 */
#ifndef INCLUDE_TOYCAR_H_
#define INCLUDE_TOYCAR_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
/**
 * @brief ToyCar checks for obstacle and commanfs robot to navigate
 */
class ToyCar {
 public:
  ToyCar();
  ~ToyCar();
  /**
   * @brief The function checks for obstacles in the robot vicinity
   * @param pointer to Laser scan messages being published on rostopic /scan
   * @return none
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);

  void odomCallback(const  geometry_msgs::PoseWithCovarianceStamped
  ::ConstPtr& msg);

  /**
   * @brief The function provides control to the robot by giving velocity
   * @param none
   * @return none
   */
  void velocityController();

  void GetOdom();

  std_msgs::Float64 velocity;

  std_msgs::Float64 steer_angle;

 private:
  /**
   * @brief ros parameter of node handle type to create a local node
   */
    ros::NodeHandle n;

    /**
     * @brief paramater containing different states of the robot
     */
    int robotState;

    tf::TransformListener mapToBaseTfListen;

    nav_msgs::Odometry odom;
};
#endif  // INCLUDE_TOYCAR_H_
