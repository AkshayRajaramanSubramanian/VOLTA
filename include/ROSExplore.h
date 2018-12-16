/*
 * @file ROSExplore.h
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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INCLUDE_ROSEXPLORE_H_
#define INCLUDE_ROSEXPLORE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "Explore.h"

/**
 * @brief Wrapper node for the Explore class
 */
class ROSExplore {
 public:
  ros::NodeHandle n;
  Explore explore;
  ros::Publisher motorCommandPublisher;
  ros::Subscriber laserSubscriber;
  ros::Subscriber mapSubscriber;
  /**
   * @brief Constructor for the class
   * @param Node Handle
   * @return None
   */
  explicit ROSExplore(ros::NodeHandle &n);
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  virtual ~ROSExplore();
  /**
   * @brief Wrapper class for getLaserData if explore class
   * @param scanMssg Laser scan data
   * @return None
   */
  void getLaserDataWrapper(const sensor_msgs::LaserScan::ConstPtr &scanMssg);
};

#endif /* INCLUDE_ROSEXPLORE_H_ */
