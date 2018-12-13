/*
 * @file ROSChargeDockDetection.h
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

#ifndef VOLTA_SRC_ROSCHARGEDOCKDETECTION_H_
#define VOLTA_SRC_ROSCHARGEDOCKDETECTION_H_

#include <vector>
#include <string>
#include "Explore.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "ChargeDock.h"
#include "ROSChargeDock.h"
#include "ChargeDockDetection.h"

/**
 * @brief Class detects charging docks and publishes related messages
 */
class ROSChargeDockDetection {
 public:
  // <!Node nandle reference
  ros::NodeHandle nh;
  // <!Image in OpenCV format
  image_transport::ImageTransport it;
  // <!Subscriber to camera sensor image
  image_transport::Subscriber imageSub;
  // <!Publish OpenCV formated image
  image_transport::Publisher imagePub;
  // <!Display window of camera sensor
  // <!Twist message publisher
  ros::Publisher twistpublisher;
  // <!Point cloud data of turtle bot RGBD sensor
  // <!Flag to check for new RGBD image
  bool hasNewPcl = false;
  // <!Flag for checking if image has charging dock
  bool check = false;
  // <!Subscribe to depth image sensor (RGBD)
  ros::Subscriber dep;
  // <!Coordinates of charging dock
  cv::Point2f center;
  ROSChargeDock ChargeDockROS;
  ChargeDockDetection ChDockDetect;
  tf::TransformBroadcaster br;

  /**
   * @brief Constructor for the class
   * @param nh Node handle
   * @return None
   */
  ROSChargeDockDetection(ros::NodeHandle _nh);
  /**
   * @brief Call back function for receiving RGBD image
   * @param Pointer to depth image
   * @return None
   */
  void depthcallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  /**
   * @brief Image processing algorithm to find presence of charging dock
   * @param msg Pointer for image in OpenCV format
   * @return none
   */
  void checkForChargeDock(const sensor_msgs::ImageConstPtr& msg);
  /**
   * @brief Places markers for charging docks in rviz environment
   * @param Transform Transform form odom to camera link
   * @return none
   */
  void broadcastTf(float x, float y, float z);
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  virtual ~ROSChargeDockDetection();
};

#endif /* VOLTA_SRC_ROSCHARGEDOCKDETECTION_H_ */
