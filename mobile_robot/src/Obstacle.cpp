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
#include <Obstacle.h>
#include <iostream>

// Declaring the state to be false in the begining
bool Obstacle::obs = false;

/*
 * @brief First method of the class. It is used to read the distance
 *
 * @param a pointer of pointing to the distance values from LaserScan topic
 *
 * @return This method returns nothing
 */
void Obstacle::readDistance(const sensor_msgs::LaserScan::ConstPtr& val) {
  // getting the reading when the sensor angle is 0 and 90
  if (val -> ranges[0] < 1.5 || val -> ranges[180] < 0.6) {
    // if value below threshold then obstacle present
    obs = true;
    // exit the function
    return;
  }
  // if no obstacle then return false
  obs = false;

}

/*
 * @brief Second method of the class. It
 *
 * @param This method has no parameters
 *
 * @return This method returns the obstacle state
 */
bool Obstacle::isObstacle() {
  // return the value of obs variable
  return obs;
}
