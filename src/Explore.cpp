/*
 * @file Explore.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Akshay Rajaraman
 * @author Akshay Rajaraman
 * @brief class that describes the function of the explore node
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

#include "../include/Explore.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stack>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"

#define PI 3.141592

/**
 * @brief constructor for initializing Explore object
 * @param NodeHandle n
 */

Explore::Explore() {}

/**
 * @brief destructor function
 */

Explore::~Explore() {}

/**
 * @brief function to set the movement of the robot
 * @param movement type [LEFT, RIGHT, FOWARD, REVERSE etc]
 */

bool Explore::robot_move(const ROBOT_MOVEMENT move_type) {
  // set motor commands for each scenario
  if (move_type == STOP) {
    motor_command.angular.z = 0.0;
    motor_command.linear.x = 0.0;
  } else if (move_type == FORWARD) {
    motor_command.angular.z = 0.0;
    motor_command.linear.x = 0.5;
  } else if (move_type == BACKWARD) {
    motor_command.linear.x = -0.5;
    motor_command.angular.z = 0.0;
  } else if (move_type == TURN_LEFT) {
    motor_command.linear.x = 0.0;
    motor_command.angular.z = 0.5;
  } else if (move_type == TURN_RIGHT) {
    motor_command.linear.x = 0.0;
    motor_command.angular.z = -0.5;
  } else if (move_type == GO_RIGHT) {
    motor_command.linear.x = 0.25;
    motor_command.angular.z = -0.25;
  } else if (move_type == GO_LEFT) {
    motor_command.linear.x = 0.25;
    motor_command.angular.z = 0.25;
  } else {
    return false;
  }
  return true;
}

/**
 * @brief callback function for receiving the laser scan data
 * @param pointer to laser scan data
 */

geometry_msgs::Twist Explore::getLaserData(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  laser_msg = *scan_msg;
  std::vector<float> laser_ranges;
  // get laser scan range
  laser_ranges = laser_msg.ranges;
  size_t range_size = laser_ranges.size();
  float left_side = 0.0, right_side = 0.0;
  // get maximum and minimum ranges for laser scan
  float range_min = laser_msg.range_max;
  float range_max = laser_msg.range_min;
  int nan_count = 0;
  for (size_t i = 0; i < range_size; i++) {
    if (laser_ranges[i] < range_min) {
      range_min = laser_ranges[i];
    }

    if (std::isnan(laser_ranges[i])) {
      nan_count++;
    }
    if (i < range_size / 4) {
      if (laser_ranges[i] > range_max) {
        range_max = laser_ranges[i];
      }
    }

    if (i > range_size / 2) {
      left_side += laser_ranges[i];
    } else {
      right_side += laser_ranges[i];
    }
  }

  if (nan_count > (range_size * 0.9) || laser_ranges[range_size / 2] < 0.25) {
    crashed = true;
  } else {
    crashed = false;
  }
  if (!crashed) {
    if (range_min <= 1 && !thats_a_door) {
      following_wall = true;
      crashed = false;
      // stop the robot before taking a decision to move
      robot_move(STOP);

      if (left_side >= right_side) {
        // turn the robot to the right
        robot_move(TURN_RIGHT);
      } else {
        // turn the robot to the left
        robot_move(TURN_LEFT);
      }
    } else {
      robot_move(STOP);
      if (following_wall) {
        if (range_max >= 1.5) {
          thats_a_door = true;
          following_wall = false;
        }
      }
      if (thats_a_door) {
        if (laser_ranges[0] <= 1) {
          thats_a_door = false;
        } else {
          // move the robot to the right
          robot_move(GO_RIGHT);
        }

      } else {
        // move the robot forward
        robot_move(FORWARD);
      }
    }
  } else {
    // move the robot backwards
    robot_move(BACKWARD);
  }
  return motor_command;
}

/**
 * @brief callback function for receiving the map data
 * @param pointer to occupancy grid data
 */

void Explore::getMapData(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  const bool chatty_map = true;
  map_msg = *msg;
  double map_width = map_msg.info.width;
  double map_height = map_msg.info.width;
  double map_origin_x = map_msg.info.origin.position.x;
  double map_origin_y = map_msg.info.origin.position.y;
  double map_orientation = acos(map_msg.info.origin.orientation.z);
  std::vector<signed char> map = map_msg.data;
}
