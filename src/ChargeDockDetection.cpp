/*
 * @file ChargeDockDetection.cpp
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
#include <string>
#include <cmath>
#include "../include/ChargeDockDetection.h"
#include "../include/ChargeDock.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp"
#include "opencv-3.3.1-dev/opencv2/highgui/highgui.hpp"
#include "opencv-3.3.1-dev/opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv-3.3.1-dev/opencv2/core/core.hpp"
#include "opencv-3.3.1-dev/opencv2/calib3d/calib3d.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_datatypes.h"

cv::Point2f ChargeDockDetection::centroid(std::vector<cv::Point2f> points) {
  int xSum = 0;
  int ySum = 0;
  for (auto &i : points) {
    xSum += i.x;
    ySum += i.y;
  }
  center.x = xSum / points.size();
  center.y = ySum / points.size();
  return center;
}

tf::Transform ChargeDockDetection::broadcastTflocal(float x, float y, float z) {
  // Create transform variable
  tf::Transform tr;
  // Set origin of transformation
  tr.setOrigin(tf::Vector3(x, y, z));
  // Initialize quaternion for rotation
  tf::Quaternion q;
  // Set zero rotation
  q.setRPY(0, 0, 0);
  tr.setRotation(q);
  return tr;
}
tf::Transform ChargeDockDetection::broadcastTfodom(
    tf::StampedTransform transform, float &x, float &y, float &z) {
  // Calculate the charge dock in real world coordinates
  // W = R.P + T
  tf::Matrix3x3 m(transform.getRotation());

  float r11 = m.getColumn(0).getX();
  float r21 = m.getColumn(0).getY();
  float r31 = m.getColumn(0).getZ();

  float r12 = m.getColumn(1).getX();
  float r22 = m.getColumn(1).getY();
  float r32 = m.getColumn(1).getZ();

  float r13 = m.getColumn(2).getX();
  float r23 = m.getColumn(2).getY();
  float r33 = m.getColumn(2).getZ();

  float Tx = transform.getOrigin().x();
  float Ty = transform.getOrigin().y();
  float Tz = transform.getOrigin().z();

  float wX = r11 * x + r12 * y + r13 * z + Tx;
  float wY = r21 * x + r22 * y + r23 * z + Ty;
  float wZ = r31 * z + r32 * y + r33 * z + Tz;

  // Broadcast calculated point with respect to odom
  tf::Transform tr;
  tf::Quaternion q;
  tr.setOrigin(tf::Vector3(wX, wY, wZ));
  q.setRPY(0, 0, 0);
  tr.setRotation(q);
  // Store the points in variables used by caller
  x = wX;
  y = wY;
  z = wZ;

  return tr;
}

bool ChargeDockDetection::depthcallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // Save latest RGBD image
  my_pcl = *cloud_msg;
  return true;
}

bool ChargeDockDetection::getXYZ(int x, int y, float &depthX, float &depthY,
                                 float &depthZ) {
  // Decrypt X, Y,Z position form RGBD data
  int arrayPosition = y * my_pcl.row_step + x * my_pcl.point_step;
  int arrayPosX = arrayPosition + my_pcl.fields[0].offset;
  int arrayPosY = arrayPosition + my_pcl.fields[1].offset;
  int arrayPosZ = arrayPosition + my_pcl.fields[2].offset;
  // X, Y, Z points calculated from RGBD
  memcpy(&depthX, &my_pcl.data[arrayPosX], sizeof(float));
  memcpy(&depthY, &my_pcl.data[arrayPosY], sizeof(float));
  memcpy(&depthZ, &my_pcl.data[arrayPosZ], sizeof(float));
  return !(std::isnan(depthX) || std::isnan(depthY) || std::isnan(depthZ));
}

ChargeDockDetection::ChargeDockDetection() {
}

cv_bridge::CvImagePtr ChargeDockDetection::checkForChargeDock(
    const sensor_msgs::ImageConstPtr& msg) {
  // Convert Raw image to OpenCV image format
  cv_bridge::CvImagePtr cvPtr;
  cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  return cvPtr;
}

cv::Mat ChargeDockDetection::checkerBoardDetect(
    cv_bridge::CvImagePtr cvPtr, std::vector<cv::Point2f> &corners) {
  cv::Mat img, gray;
  cv::Size patternsize(6, 7);  // interior number of corners
  img = cvPtr->image;
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  bool patternfound = findChessboardCorners(
      gray,
      patternsize,
      corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
          + cv::CALIB_CB_FAST_CHECK);
  drawChessboardCorners(img, patternsize, cv::Mat(corners), patternfound);
  return img;
}

ChargeDockDetection::~ChargeDockDetection() {
}

