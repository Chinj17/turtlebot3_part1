/*
 * BSD 3-Clause License
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @file Obstacle.cpp
 * @brief This is the implementation file of the Obstacle class
 *
 * @copyright Copyright (c) Chinmay Joshi
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi
 * @date 17-02-2020
 */
#include<ros/ros.h>
#include<Obstacle.h>
#include<geometry_msgs/Twist.h>

/*
 * @brief Main function to run Obstacle detection
 */

int main(int argc, char** argv) {
  // Initializing the code as a ROS node
  ros::init(argc, argv, "mobile_robot");
  // creating a ROS node handle needed for every ROS node
  ros::NodeHandle n;
  // Creating an object for the class Obstacle
  Obstacle vad;
  // Subscriber for the LaserScan data
  auto laserSensor = n.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                                             &Obstacle::readDistance, &vad);
  // Publisher to advertise linear and angular velocities
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  // Creating an object for Twist msg class
  geometry_msgs::Twist msg;
  // Starting from rest
  msg.linear.x = 0.0;
  msg.angular.z = 0.0;

  ros::Rate loop_rate(2);

  while(ros::ok()) {
    // if no obstacle
    if(!vad.isObstacle()) {
      // move forward
      msg.linear.x = 0.5;
      // robot should not rotate
      msg.angular.z = 0.0;
      // print going forward
      ROS_INFO_STREAM("Going Forward");

    } else {
      // If obstacle is present
      // Rotate on axis to avoid obstacle
      msg.angular.z = 0.5;
      // Robot should not move forward while rotating
      msg.linear.x = 0.0;
      // give information about the movement in the terminal
      ROS_INFO_STREAM("Obstacle detected! Turning to avoid it");
     }
    // publish the velocities
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
