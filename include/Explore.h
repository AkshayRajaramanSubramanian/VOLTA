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

#ifndef INCLUDE_EXPLORE_H_
#define INCLUDE_EXPLORE_H_

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stack>

/**
 * @brief enum describing possible robot movements
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

/**
 * @brief struct for type point
 */

struct point {
  int x;  // !<X co-ordinate of point
  int y;  // !<Y co-ordinate of point
};

class Explore {
 public:
  sensor_msgs::LaserScan laser_msg;
  nav_msgs::OccupancyGrid map_msg;
  geometry_msgs::Twist motor_command;
  bool following_wall = false;
  bool thats_a_door = false;
  bool crashed = false;
  Explore();
  ~Explore();
  /**
   * @brief function to set the movement of the robot
   * @param movement type [LEFT, RIGHT, FOWARD, REVERSE etc]
   */
  bool robot_move(const ROBOT_MOVEMENT move_type);
  /**
   * @brief callback function for receiving the laser scan data
   * @param pointer to laser scan data
   */
  geometry_msgs::Twist getLaserData(
      const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  /**
   * @brief callback function for receiving the map data
   * @param pointer to occupancy grid data
   */
  void getMapData(const nav_msgs::OccupancyGrid::ConstPtr &msg);
};

#endif  // INCLUDE_EXPLORE_H_
