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
 * @file image_converter.cpp
 * @brief This is the c++ file to convert ROS images to OpenCV
 *        readable images
 *
 * @copyright Copyright (c) Chinmay Joshi
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi
 * @date 17-02-2020
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Used for creating a window while viewing images
static const std::string OPENCV_WINDOW = "Image window";

/*
 * @brief This is the Image call back function.
 *
 * @param Input is the message published from camera feed.
 * 
 * @return Does not return anything.
 */

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    // creating a pointer to point to the image
    cv_bridge::CvImagePtr cv_ptr;
    // creating a publisher for publishing the cv_bridge image
    image_transport::Publisher imagePub;

    try
    {
      // creating a copy of the image to display
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // resizing the image for viewing purposes
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(800, 640));
    // converting image to hsv space
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, CV_BGR2HSV);
    // Creating masks to segment out red objects
    cv::Mat mask1, mask2, mask;
    // taking minimum and maximum value of red in hsv range
    cv::inRange(hsv_image, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv_image, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);

    // Generating the final mask
    mask = mask1 + mask2;
    // creating an image for the masking operation
    cv::Mat result;
    cv::bitwise_and(cv_ptr->image, cv_ptr->image, result, mask = mask);
    // Creating a variable to store contours
    std::vector<std::vector<cv::Point>> contours;
    // finding contours to draw bounding boxes
    cv::findContours(mask, contours,
          CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    // Drawing bounding boxes around the objects
    cv::drawContours(cv_ptr->image, contours, -1, cv::Scalar(0, 0, 0), 2);
    // displaying the image
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
    // updating the window to show as a video stream
    imagePub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
  // Initializing the code as a ROS node
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;
  // cv_bridge variable used to convert ROS images
  image_transport::ImageTransport imgT(nh);
  // Subscriber to subscribe to turtlebot camera feed
  image_transport::Subscriber imageSub;
  // Subscribing to the camera feed from the turtlebot
  imageSub = imgT.subscribe("/camera/rgb/image_raw", 1,
      &imageCb);
  // generating a window name
  cv::namedWindow(OPENCV_WINDOW);
  ros::spin();
  return 0;
}
