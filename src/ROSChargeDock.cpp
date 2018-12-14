/*
 * @file ROSChargeDock.cpp
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
#include "../include/ROSChargeDock.h"
#include "ros/ros.h"
#include "../include/ChargeDock.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "opencv-3.3.1-dev/opencv2/core/core.hpp"
#include "opencv-3.3.1-dev/opencv2/core/types.hpp"

ROSChargeDock::ROSChargeDock(ros::NodeHandle _nh)
    : n(_nh) {
  markerPub = n.advertise<visualization_msgs::Marker>("ChargePoint", 10);
}

void ROSChargeDock::PublishChargeDock(visualization_msgs::Marker marker) {
  markerPub.publish(marker);
}

void ROSChargeDock::placeChargeDock(float x, float y, float z) {
  // <! Build cernter point structure
  cv::Point3f center;
  center.x = x;
  center.y = y;
  center.z = z;
  // <!check if charge dock is already published and place marker accordingly
  if (!Chargedock.ballParkCheck(center)) {
    PublishChargeDock(Chargedock.placeChargeDock(x, y, z));
  }
}

ROSChargeDock::~ROSChargeDock() {
}

