/*
 * @file ROSChargeDockDetectionTest.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief Class detects presence of charging dock in image captured by robot
 */

/*
 * MIT License
 *
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "../include/ROSChargeDockDetection.h"
#include "../include/ChargeDockDetection.h"
#include "../include/ChargeDock.h"
#include "../include/ROSChargeDock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Twist.h"

class ROSChargeDockDetectionTest {
 public:
  bool isImagePublished;
  bool isDepthPublished;
  bool isImageConverted;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber imageSub;
  ros::Subscriber depthSub;
  image_transport::Subscriber convertedImage;
  /**
   * @brief callback function for image
   * @param msg pointer to image data
   * @return none
   */
  void callbackImageSub(const sensor_msgs::ImageConstPtr& msg) {
    isImagePublished = true;
  }
  /**
   * @brief callback function for point cloud data form RGBD sensor
   * @param cloudMsg pointer to RGBD message
   * @return none
   */
  void callbackDepthSub(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    isDepthPublished = true;
  }
  /**
   * @brief callback function for image
   * @param msg pointer to image data
   * @return none
   */
  void callbackConvertedImage(const sensor_msgs::ImageConstPtr& msg) {
    isImageConverted = true;
  }
  /**
   * @brief Constructor for the class
   * @param _nh Node handle
   * @return None
   */
  explicit ROSChargeDockDetectionTest(const ros::NodeHandle &_nh)
      : nh(_nh),
        it(_nh) {
    isImagePublished = false;
    isDepthPublished = false;
    isImageConverted = false;
    imageSub = it.subscribe("/camera/rgb/image_raw", 1,
                            &ROSChargeDockDetectionTest::callbackImageSub,
                            this);
    depthSub = nh.subscribe("/camera/depth/points", 1,
                            &ROSChargeDockDetectionTest::callbackDepthSub,
                            this);
    convertedImage = it.subscribe(
        "/image_converter/output_video", 1,
        &ROSChargeDockDetectionTest::callbackConvertedImage, this);
  }
  /**
   * @brief destructor for the class
   * @param None
   * @return None
   */
  ~ROSChargeDockDetectionTest() {
  }
};

/**
 * @brief Testing for image publidhed
 */
TEST(inputTest, checkImageInput) {
  ros::NodeHandle n;
  ROSChargeDockDetectionTest rosChargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(rosChargeDockDetectionTest.isImagePublished);
}

/**
 * @brief Testing if depth image is recieved
 */
TEST(inputTest, checkDepthInput) {
  ros::NodeHandle n;
  ROSChargeDockDetectionTest rosChargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(rosChargeDockDetectionTest.isDepthPublished);
}

/**
 * @brief Testing if image of type accepted by OpenCV ireceived
 */
TEST(outputTest, checkConvertedImage) {
  ros::NodeHandle n;
  ROSChargeDockDetectionTest rosChargeDockDetectionTest(n);
  ROSChargeDockDetection rosChargeDockDetection(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(rosChargeDockDetectionTest.isImageConverted);
}

