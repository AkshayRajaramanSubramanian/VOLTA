/*
 * ROSChargeDock.h
 *
 *  Created on: Dec 12, 2018
 *      Author: bala
 */

#ifndef VOLTA_SRC_ROSCHARGEDOCK_H_
#define VOLTA_SRC_ROSCHARGEDOCK_H_

#include <vector>

#include "ChargeDock.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "opencv-3.3.1-dev/opencv2/core/types.hpp"

class ROSChargeDock {
  ros::NodeHandle n;
  ros::Publisher markerPub;   // <! Publisher to publish RVIX marker
  ChargeDock Chargedock;
 public:
  /**
   * @brief Constructor for the class
   * @param None
   * @return None
   */
  ROSChargeDock(ros::NodeHandle _nh);
  /**
   * @brief publishes chargedock marker
   * @param marker Data to be published
   * @return None
   */
  void PublishChargeDock(visualization_msgs::Marker marker);
  /**
   * @brief returns the charging dock information on request
   * @param x,y,z Points that has to be published
   * @return none
   */
  void placeChargeDock(float x, float y, float z);
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  virtual ~ROSChargeDock();
};

#endif /* VOLTA_SRC_ROSCHARGEDOCK_H_ */
