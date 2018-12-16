/*
 * @file ROSChargeDocktest.cpp
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
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"

/**
 * @brief Fixture class for level 2 ros test
 */
class ROSChargeDockTest {
 public:
  bool isMarkerSet;
  ros::NodeHandle nh;
  ros::Subscriber markerSub;
  void callbackMarkerSub(const visualization_msgs::Marker::ConstPtr& msg) {
    isMarkerSet = true;
  }
  explicit ROSChargeDockTest(const ros::NodeHandle &_nh)
      : nh(_nh) {
    isMarkerSet = false;
    markerSub = nh.subscribe("/ChargePoint", 10,
                             &ROSChargeDockTest::callbackMarkerSub, this);
  }
  ~ROSChargeDockTest() {
  }
};

/**
 * @brief Testing marker placement
 */
TEST(rosChargeDockTest, checkMarkerPlacement) {
  ros::NodeHandle n;
  ROSChargeDock rosChargeDock(n);
  visualization_msgs::Marker points;
  points.header.frame_id = "/odom";
  points.header.stamp = ros::Time::now();
  points.ns = "chargeDock";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 1;
  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 1;
  points.scale.y = 1;
  points.scale.z = 0.1;

  // Points are green
  points.color.g = 1.0;
  points.color.a = 1.0;

  // Point to be matked
  geometry_msgs::Point p;
  p.x = 5;
  p.y = 0;
  p.z = 0;
  points.points.push_back(p);
  rosChargeDock.PublishChargeDock(points);
  ROSChargeDockTest rosChargeDockTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(!rosChargeDockTest.isMarkerSet);
}

