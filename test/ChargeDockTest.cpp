/*
 * ExploreTest.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: Akshay
 */
#include <ros/console.h>

#include "../include/ChargeDock.h"

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"

#include <math.h>
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

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

TEST(placeChargeDockTest, placeOutBallPark) {
	ros::NodeHandle nh;
  ChargeDock chargeDock;
  cv::Point3f center;
  center.x = 5;
  center.y = 0;
  center.z = 0;
  chargeDock.centers.push_back(center);
  visualization_msgs::Marker placedPoints = chargeDock.placeChargeDock(25,0,0);
  // Test
	EXPECT_EQ(placedPoints.points[0].x,25);
}

TEST(placeChargeDockTest, placeInBallPark) {
	ros::NodeHandle nh;
  ChargeDock chargeDock;
  cv::Point3f center;
  center.x = 5;
  center.y = 0;
  center.z = 0;
  chargeDock.centers.push_back(center);
  visualization_msgs::Marker placedPoints = chargeDock.placeChargeDock(7,0,0);
  // Test
	EXPECT_NE(placedPoints.points[0].x,25);
}
