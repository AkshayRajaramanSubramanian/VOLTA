/*
 * @file ROSExplore.cpp
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
#include "../include/ROSExplore.h"

/**
 * @brief Wrapper node for the Explore class
 * @param NodeHandle n
 */

ROSExplore::ROSExplore(ros::NodeHandle &n) : n(n) {
  // publish motor commands
  motorCommandPublisher =
      n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  // subscribe to laser data
  laserSubscriber =
      n.subscribe("/scan", 1000, &ROSExplore::getLaserDataWrapper, this);
  // subscribe to map data
  mapSubscriber = n.subscribe("/map", 1000, &Explore::getMapData, &explore);
}
void ROSExplore::getLaserDataWrapper(
    const sensor_msgs::LaserScan::ConstPtr &scanMssg) {
  sensor_msgs::LaserScan scanMsg = *scanMssg;
  geometry_msgs::Twist motor_command = explore.getLaserData(scanMsg);
  // publish motor command
  motorCommandPublisher.publish(explore.motorCommand);
  usleep(10);
}
ROSExplore::~ROSExplore() {}
