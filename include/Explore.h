/*
 * @file Explore.h
 * @Copyright MIT license
 * Copyright (c) 2018 Akshay Rajaraman
 * @author Akshay Rajaraman
 * @brief class desciption for the Explore implementation
 *
 */

/*
 * MIT License
 *
 * Copyright (c) 2018 Akshay Rajaraman
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


#ifndef _INCLUDE_GAZEBOT_EXPLORE_EXPLORE_H_
#define _INCLUDE_GAZEBOT_EXPLORE_EXPLORE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>
/**
 * @brief class description for Explore
 */

typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    GO_RIGHT,
    GO_LEFT

} ROBOT_MOVEMENT;


class Explore {
 public:
  ros::NodeHandle n;
  explicit Explore(ros::NodeHandle &n);
  ~Explore();
  bool robot_move (const ROBOT_MOVEMENT move_type);
  void getLaserData (const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  void getMapData (const nav_msgs::OccupancyGrid::ConstPtr &msg);
  ros::Publisher motor_command_publisher;
  ros::Subscriber laser_subscriber;
  ros::Subscriber map_subscriber;
  sensor_msgs::LaserScan laser_msg;
  nav_msgs::OccupancyGrid map_msg;
  geometry_msgs::Twist motor_command;
  bool following_wall = false;
  bool thats_a_door = false;
  bool crashed = false;
};

#endif //   _INCLUDE_GAZEBOT_WALKER_WALKER_H_
