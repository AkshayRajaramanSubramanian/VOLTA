/*
 * @file ChargeDockDetection.h
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief Header file for bug ChargeDockDetection class.
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

#ifndef VOLTA_SRC_CHARGEDOCKDETECTION_H_
#define VOLTA_SRC_CHARGEDOCKDETECTION_H_

#include <vector>
#include <string>
#include "Explore.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp"
#include "opencv-3.3.1-dev/opencv2/highgui/highgui.hpp"
#include "opencv-3.3.1-dev/opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv-3.3.1-dev/opencv2/core/core.hpp"
#include "opencv-3.3.1-dev/opencv2/calib3d/calib3d.hpp"
#include "ChargeDock.h"

/**
 * @brief Class runs image processing on captured images to detect charging dock
 */
class ChargeDockDetection {
 private:
  // <!Node nandle reference
  ros::NodeHandle nh;
  // <!Image in OpenCV format
  image_transport::ImageTransport it;
  // <!Subscriber to camera sensor image
  image_transport::Subscriber imageSub;
  // <!Publish OpenCV formated image
  image_transport::Publisher imagePub;
  // <!Display window of camera sensor
  const std::string OPENCV_WINDOW = "chargedock";
  const std::string CHK_WINDOW = "checkerimg";
  // <!Twist message publisher
  ros::Publisher twistpublisher;
  // <!Point cloud data of turtle bot RGBD sensor
  sensor_msgs::PointCloud2 my_pcl;
  // <!Flag to check for new RGBD image
  bool hasNewPcl = false;
  // <!Flag for checking if image has charging dock
  bool check = false;
  // <!Subscribe to depth image sensor (RGBD)
  ros::Subscriber dep;
  // <!Coordinates of charging dock
  point chargeMarker;
  cv::Point2f center;
  ChargeDock dock;
 public:

  /**
   * @brief Publish charge dock coordinates to chargeDock topic
   * @param None
   * @return None
   */
  void publishChargerDocPos();
  /**
   * @brief Image processing algorithm to find presence of charging dock
   * @param msg Pointer for image in OpenCV format
   * @return None
   */
  void checkForChargeDock(const sensor_msgs::ImageConstPtr& msg);
  /**
   * @brief Calculate charging dock coordinates with respect to map frame
   * @param None
   * @return None
   */
  void findChargePosition();
  /**
   * @brief Training module for charge dock detection algorithm
   * @param None
   * @return None
   */
  void svmTrainer();
  /**
   * @brief Image detection algorithm for checker board
   * @param Pointer to OpenCV image
   * @return None
   */
  void checkerBoardDetect(cv_bridge::CvImagePtr cvPtr);
  /**
   * @brief Call back function for receiving RGBD image
   * @param Pointer to depth image
   * @return None
   */
  void depthcallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  /**
   * @brief Get X, Y and Z coordinates of charge dock
   * @param X, Y the xy points on image were charge dock is detected
   * @return None
   */
  void getXYZ(int x, int y);
  /**
   * @brief Places markers for charging docks in rviz environment
   * @param points Points for which centroid has to be calculated
   * @return None
   */
  cv::Point2f centroid(std::vector<cv::Point2f> points);
  /**
   * @brief Places markers for charging docks in rviz environment
   * @param z,y,z Point for which transformation form odom has to be found
   * @return None
   */
  void broadcastTf(float x, float y, float z);
  /**
   * @brief Constructor for the class
   * @param nh Node handle
   * @return None
   */
  ChargeDockDetection(ros::NodeHandle _nh);
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  virtual ~ChargeDockDetection();
};

#endif /* VOLTA_SRC_CHARGEDOCKDETECTION_H_ */

