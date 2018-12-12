/*
 * ROSChargeDock.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: bala
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

