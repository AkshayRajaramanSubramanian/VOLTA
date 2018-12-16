/*
 * @file ExploreTest.cpp
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

#include "../include/ChargeDock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief Testing if ball park is outside the is outside limits
 */
TEST(ballParkTest, outBallPark) {
  ChargeDock chargeDock;
  cv::Point3f center;
  center.x = 5;
  center.y = 0;
  center.z = 0;
  chargeDock.centers.push_back(center);
  cv::Point3f point;
  point.x = 25;
  point.y = 0;
  point.z = 0;
  bool inBallPark = chargeDock.ballParkCheck(point);
  // Test
  EXPECT_FALSE(inBallPark);
}

/**
 * @brief Testing for points inside ball park bounds
 */
TEST(ballParkTest, inBallPark) {
  ChargeDock chargeDock;
  cv::Point3f center;
  center.x = 5;
  center.y = 0;
  center.z = 0;
  chargeDock.centers.push_back(center);
  cv::Point3f point;
  point.x = 7;
  point.y = 0;
  point.z = 0;
  bool inBallPark = chargeDock.ballParkCheck(point);
  // Test
  EXPECT_TRUE(inBallPark);
}

/**
 * @brief Testing for placing a point outside ball park bounds
 */
TEST(placeChargeDockTest, placeOutBallPark) {
  ros::NodeHandle nh;
  ChargeDock chargeDock;
  cv::Point3f center;
  center.x = 5;
  center.y = 0;
  center.z = 0;
  chargeDock.centers.push_back(center);
  visualization_msgs::Marker placedPoints = chargeDock.placeChargeDock(25, 0,
                                                                       0);
  // Test
  EXPECT_EQ(placedPoints.points[0].x, 25);
}

/**
 * @brief Test for placing point inside ball park region
 */
TEST(placeChargeDockTest, placeInBallPark) {
  ros::NodeHandle nh;
  ChargeDock chargeDock;
  cv::Point3f center;
  center.x = 5;
  center.y = 0;
  center.z = 0;
  chargeDock.centers.push_back(center);
  visualization_msgs::Marker placedPoints = chargeDock.placeChargeDock(7, 0, 0);
  // Test
  EXPECT_NE(placedPoints.points[0].x, 25);
}
