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
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "ChargeDock.h"
#include "ROSChargeDock.h"

/**
 * @brief Class runs image processing on captured images to detect charging dock
 */
class ChargeDockDetection {
 private:
  sensor_msgs::PointCloud2 my_pcl;
  cv::Point2f center;
 public:
  /**
   * @brief Image processing algorithm to find presence of charging dock
   * @param msg Pointer for image in OpenCV format
   * @return CvPointer Pointer to image of type bridge image message
   */
  cv_bridge::CvImagePtr checkForChargeDock(
      const sensor_msgs::ImageConstPtr& msg);
  /**
   * @brief Image detection algorithm for checker board
   * @param Pointer Pointer to OpenCV image and
   * @param Corners Corners points of the checker board squares
   * @return Image Image that has checker board marked
   */
  cv::Mat checkerBoardDetect(cv_bridge::CvImagePtr cvPtr,
                             std::vector<cv::Point2f> &corners);
  /**
   * @brief Call back function for receiving RGBD image
   * @param Pointer to depth image
   * @return Boolean Returns true every time RGBD image is receieved
   */
  bool depthcallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  /**
   * @brief Get X, Y and Z coordinates of charge dock
   * @param X,Y the xy points on image were charge dock is detected
   * @param depthX,depthY,depthX X, Y, Z points of the dock with respect to camera frame
   * @return Boolean True if valid coordinates are found
   */
  bool getXYZ(int x, int y, float &depthX, float &depthY, float &depthZ);
  /**
   * @brief Places markers for charging docks in rviz environment
   * @param points Points for which centroid has to be calculated
   * @return None
   */
  cv::Point2f centroid(std::vector<cv::Point2f> points);
  /**
   * @brief Calculates transforms for charging docks in rviz environment
   * @param x,y,z Point for which transformation form camera link has to be found
   * @return tr Transform of the point with respect to camera frame
   */
  tf::Transform broadcastTflocal(float x, float y, float z);
  /**
   * @brief Calculates transforms for charging docks in rviz environment
   * @param Transform Transform form odom to camera link
   * @param x,y,z Point for which transformation form odom has to be found
   * @return tr Transform of the point with respect to camera frame
   */
  tf::Transform broadcastTfodom(tf::StampedTransform transform, float &x,
                                float &y, float &z);
  /**
   * @brief Constructor for the class
   * @param nh Node handle
   * @return None
   */
  ChargeDockDetection();
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  virtual ~ChargeDockDetection();
};

#endif /* VOLTA_SRC_CHARGEDOCKDETECTION_H_ */

