/*
 * @file ROSChargeDock.h
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief Header file for bug ChargeDockDetection class.
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

#ifndef VOLTA_SRC_ROSCHARGEDOCK_H_
#define VOLTA_SRC_ROSCHARGEDOCK_H_

#include <vector>

#include "ChargeDock.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "opencv-3.3.1-dev/opencv2/core/types.hpp"

/**
 * @brief Class publishes the marker of charging dock
 */
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
