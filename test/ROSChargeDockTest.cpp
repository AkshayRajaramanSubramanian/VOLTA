#include <ros/console.h>

#include "../include/ROSChargeDockDetection.h"
#include "../include/ChargeDockDetection.h"
#include "../include/ChargeDock.h"
#include "../include/ROSChargeDock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>
#include "visualization_msgs/Marker.h"

class ROSChargeDockTest {
 public:
	bool isMarkerSet;
	ros::NodeHandle nh;
	ros::Subscriber markerSub;
	void callbackMarkerSub (const visualization_msgs::Marker::ConstPtr& msg) {
		isMarkerSet = true;
	}
	ROSChargeDockTest(ros::NodeHandle &_nh) : nh(_nh){
		isMarkerSet = false;
		markerSub = nh.subscribe("/ChargePoint", 10, &ROSChargeDockTest::callbackMarkerSub, this);
	}
	~ROSChargeDockTest() {
	}
};

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

