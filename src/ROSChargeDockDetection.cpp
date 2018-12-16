/*
 * @file ROSChargeDockDetection.cpp
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
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_broadcaster.h"
#include "image_transport/image_transport.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "../include/ChargeDockDetection.h"
#include "../include/ChargeDock.h"
#include "../include/ROSChargeDock.h"
#include "../include/ROSChargeDockDetection.h"

ROSChargeDockDetection::ROSChargeDockDetection(ros::NodeHandle _nh)
    : nh(_nh),
      it(_nh),
      ChargeDockROS(_nh) {
// Subbscribe to Raw image
  imageSub = it.subscribe("/camera/rgb/image_raw", 1,
                          &ROSChargeDockDetection::checkForChargeDock, this);
  if (imageSub == NULL) {
    ROS_ERROR_STREAM("Images not getting read properly");
  }
  imagePub = it.advertise("/image_converter/output_video", 100);
  dep = nh.subscribe("/camera/depth/points", 1,
                     &ROSChargeDockDetection::depthcallback, this);
}

void ROSChargeDockDetection::depthcallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // Save latest RGBD image
  hasNewPcl = ChDockDetect.depthcallback(cloud_msg);
}

void ROSChargeDockDetection::checkForChargeDock(
    const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cvPtr;
  cv::Mat img;
  std::vector<cv::Point2f> corners;
  // Get image in which checker board has to be detected
  cvPtr = ChDockDetect.checkForChargeDock(msg);
  // Finds the corner points and marks them on image
  img = ChDockDetect.checkerBoardDetect(cvPtr, corners);
  // Publish the marked image for RVIZ visuvalization
  imagePub.publish(cv_bridge::CvImage(cvPtr->header, "bgr8", img).toImageMsg());
  if (corners.size() > 30) {
    // Check if new RGBD image is available
    if (hasNewPcl) {
      // Calculate center of the checker board
      center = ChDockDetect.centroid(corners);
      // Variables to hold X,Y,Z coordinates in map
      float depthX;
      float depthY;
      float depthZ;
      // Check if charge dock point can be found in RGBD image
      if (ChDockDetect.getXYZ(center.x, center.y, depthX, depthY, depthZ)) {
        broadcastTf(depthX, depthY, depthZ);
      }
      // Flag to specify the latest RGBD image is processed
      hasNewPcl = false;
    }
  }
}

void ROSChargeDockDetection::broadcastTf(float x, float y, float z) {
  // Get transform in terms of camera link frame
  tf::Transform tr = ChDockDetect.broadcastTflocal(x, y, z);
  // Broadcast transform frame to talker with respect to bamera base frame
  br.sendTransform(
      tf::StampedTransform(tr, ros::Time::now(), "camera_depth_optical_frame",
                           "volta"));
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.waitForTransform("/odom", "/camera_depth_optical_frame",
                              ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("/odom", "/camera_depth_optical_frame",
                             ros::Time(0), transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // Get transform in terms of odom frame
  tr = ChDockDetect.broadcastTfodom(transform, x, y, z);
  br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "odom", "volta"));
  // Place charge dock marker
  ChargeDockROS.placeChargeDock(x, y, z);
}

ROSChargeDockDetection::~ROSChargeDockDetection() {
}

