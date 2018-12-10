/*
 * @file ChargeDock.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief Class maintaining all charging dock information
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

#include "../include/ChargeDock.h"

#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "cv_bridge/cv_bridge.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "opencv-3.3.1-dev/opencv2/core/core.hpp"
#include "opencv-3.3.1-dev/opencv2/core/types.hpp"

ChargeDock::ChargeDock(ros::NodeHandle &_nh)
    : n(_nh) {
  markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker",
                                                      10);
}

void ChargeDock::placeChargeDock(float x, float y, float z) {
  float f = 0.0;
  visualization_msgs::Marker points;
  points.header.frame_id = "/odom";
  points.header.stamp = ros::Time::now();
  points.ns = "chargeDock";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.id = id;

  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // Points are green
  points.color.g = 1.0;
  points.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;

  points.points.push_back(p);

  markerPub.publish(points);
  id += 1;
  ROS_INFO_STREAM("Marker placed");
}

ChargeDock::~ChargeDock() {

}

