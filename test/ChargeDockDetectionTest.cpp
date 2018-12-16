/*
 * @file ChargeDockDetectionTest.cpp
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

#include <vector>
#include "../include/ROSChargeDockDetection.h"
#include "../include/ChargeDockDetection.h"
#include "../include/ChargeDock.h"
#include "../include/ROSChargeDock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud2.h"

/**
 * @brief Test fixture class for level 2 ros test
 */
class ChargeDockDetectionTest {
 public:
  bool isImagePublished;
  bool isDepthPublished;
  bool isImageConverted;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber imageSub;
  ros::Subscriber depthSub;
  sensor_msgs::PointCloud2ConstPtr pointCloud;
  image_transport::Subscriber convertedImage;
  sensor_msgs::ImageConstPtr imageRaw;
  /**
   * @brief callback function for image
   * @param msg pointer to image data
   * @return none
   */
  void callbackImageSub(const sensor_msgs::ImageConstPtr& msg) {
    isImagePublished = true;
    imageRaw = msg;
  }
  /**
   * @brief Callback for point cloud data
   * @param cloudMsg pointer to point cloud data
   * @return none
   */
  void callbackDepthSub(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    isDepthPublished = true;
    pointCloud = cloudMsg;
  }
  /**
   * @brief Call back function to image data
   * @param msg pointer to image message received
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
  explicit ChargeDockDetectionTest(const ros::NodeHandle &_nh)
      : nh(_nh),
        it(_nh) {
    isImagePublished = false;
    isDepthPublished = false;
    isImageConverted = false;
    imageSub = it.subscribe("/camera/rgb/image_raw", 1,
                            &ChargeDockDetectionTest::callbackImageSub, this);
    depthSub = nh.subscribe("/camera/depth/points", 1,
                            &ChargeDockDetectionTest::callbackDepthSub, this);
    convertedImage = it.subscribe(
        "/image_converter/output_video", 1,
        &ChargeDockDetectionTest::callbackConvertedImage, this);
  }
  /**
   * @brief destructor for the class
   * @param None
   * @return None
   */
  ~ChargeDockDetectionTest() {
  }
};

/**
 * @brief Testing the centroid value of charge dock
 */
TEST(centroidTest, checkCentroidValue) {
  ChargeDockDetection chargeDockDetection;
  cv::Point2f point;
  std::vector<cv::Point2f> points;
  point.x = 0;
  point.y = 0;
  points.push_back(point);
  point.x = 10;
  point.y = 0;
  points.push_back(point);
  cv::Point2f output;
  output = chargeDockDetection.centroid(points);
  EXPECT_EQ(output.x, 5);
}

/**
 * @brief testing local broadcast of transform frame with respect to camera link
 */
TEST(broadcastTflocalTest, checkLocalBroadcast) {
  ChargeDockDetection chargeDockDetection;
  tf::Transform output;
  output = chargeDockDetection.broadcastTflocal(5, 0, 0);
  EXPECT_EQ(output.getOrigin().x(), 5);
}

/**
 * @brief testing final broadcast of transform with respect to odom
 */
TEST(broadcastTfodomTest, checkOdomBroadcast) {
  ChargeDockDetection chargeDockDetection;
  tf::Transform local;
  local = chargeDockDetection.broadcastTflocal(5, 0, 0);
  tf::StampedTransform input(local, ros::Time(), "/volta", "/odom");
  tf::Transform output;
  float x = 15;
  float y = 0;
  float z = 0;
  output = chargeDockDetection.broadcastTfodom(input, x, y, z);
  EXPECT_EQ(output.getOrigin().x(), 20);
}

/**
 * @brief Checking if callback is properly called for point cloud data
 */
TEST(depthcallbackTest, checkTrue) {
  ros::NodeHandle n;
  ChargeDockDetection chargeDockDetection;
  ChargeDockDetectionTest chargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(
      chargeDockDetection.depthcallback(chargeDockDetectionTest.pointCloud));
}

/**
 * @brief testing forward of robot move function
 */
TEST(getXYZTest, checkTrueValue) {
  ros::NodeHandle n;
  ChargeDockDetection chargeDockDetection;
  ChargeDockDetectionTest chargeDockDetectionTest(n);
  float x;
  float y;
  float z;
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  chargeDockDetection.depthcallback(chargeDockDetectionTest.pointCloud);
  EXPECT_TRUE(chargeDockDetection.getXYZ(5, 0, x, y, z));
}

/**
 * @brief testing for image reception in cv pointer
 */
TEST(getCheckForChargeDockTest, checkCvPtr) {
  ros::NodeHandle n;
  ChargeDockDetection chargeDockDetection;
  ChargeDockDetectionTest chargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  cv_bridge::CvImagePtr cvPtr = chargeDockDetection.checkForChargeDock(
      chargeDockDetectionTest.imageRaw);
  cv::Mat img = cvPtr->image;
  EXPECT_NE(img.size().width, 0);
}

/**
 * @brief testing for image size received in topic
 */
TEST(checkerBoardDetectTest, checkImage) {
  ros::NodeHandle n;
  ChargeDockDetection chargeDockDetection;
  ChargeDockDetectionTest chargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  std::vector<cv::Point2f> corners;
  cv_bridge::CvImagePtr cvPtr = chargeDockDetection.checkForChargeDock(
      chargeDockDetectionTest.imageRaw);
  cv::Mat img = chargeDockDetection.checkerBoardDetect(cvPtr, corners);
  EXPECT_NE(img.size().width, 0);
}

