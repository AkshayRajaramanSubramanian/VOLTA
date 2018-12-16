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

/**
 * @brief constructor for initializing Explore object
 * @param NodeHandle n
 */

Explore::Explore() {
}

/**
 * @brief destructor function
 */

Explore::~Explore() {
}

/**
 * @brief function to set the movement of the robot
 * @param movement type [LEFT, RIGHT, FOWARD, REVERSE etc]
 */

bool Explore::robotMove(const ROBOT_MOVEMENT move_type) {
  // set motor commands for each scenario
  if (move_type == STOP) {
    motorCommand.angular.z = 0.0;
    motorCommand.linear.x = 0.0;
  } else if (move_type == FORWARD) {
    motorCommand.angular.z = 0.0;
    motorCommand.linear.x = 0.5;
  } else if (move_type == BACKWARD) {
    motorCommand.linear.x = -0.5;
    motorCommand.angular.z = 0.0;
  } else if (move_type == TURN_LEFT) {
    motorCommand.linear.x = 0.0;
    motorCommand.angular.z = 0.5;
  } else if (move_type == TURN_RIGHT) {
    motorCommand.linear.x = 0.0;
    motorCommand.angular.z = -0.5;
  } else if (move_type == GO_RIGHT) {
    motorCommand.linear.x = 0.25;
    motorCommand.angular.z = -0.25;
  } else if (move_type == GO_LEFT) {
    motorCommand.linear.x = 0.25;
    motorCommand.angular.z = 0.25;
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
    const sensor_msgs::LaserScan laserMsg) {
  std::vector<float> laserRanges;
  // get laser scan range
  laserRanges = laserMsg.ranges;
  size_t rangeSize = laserRanges.size();
  float leftSide = 0.0;
  float rightSide = 0.0;
  // get maximum and minimum ranges for laser scan
  float rangeMin = laserMsg.range_max;
  float rangeMax = laserMsg.range_min;
  int nanCount = 0;
  for (size_t i = 0; i < rangeSize; i++) {
    if (laserRanges[i] < rangeMin) {
      rangeMin = laserRanges[i];
    }

    if (std::isnan(laserRanges[i])) {
      nanCount++;
    }
    if (i < rangeSize / 4) {
      if (laserRanges[i] > rangeMax) {
        rangeMax = laserRanges[i];
      }
    }

    if (i > rangeSize / 2) {
      leftSide += laserRanges[i];
    } else {
      rightSide += laserRanges[i];
    }
  }

  if (nanCount > (rangeSize * 0.9) || laserRanges[rangeSize / 2] < 0.25) {
    crashed = true;
  } else {
    crashed = false;
  }
  if (!explored) {
    if (!crashed) {
      if (rangeMin <= 1 && !thatsADoor) {
        followingWall = true;
        crashed = false;
        // stop the robot before taking a decision to move
        robotMove(STOP);
        if (leftSide >= rightSide) {
          // turn the robot to the right
          robotMove(TURN_RIGHT);
        } else {
          // turn the robot to the left
          robotMove(TURN_LEFT);
        }
      } else {
        robotMove(STOP);
        if (followingWall) {
          if (rangeMax >= 1.5) {
            thatsADoor = true;
            followingWall = false;
          }
        }
        if (thatsADoor) {
          if (laserRanges[0] <= 1) {
            thatsADoor = false;
          } else {
            // move the robot to the right
            robotMove(GO_RIGHT);
          }

        } else {
          // move the robot forward
          robotMove(FORWARD);
        }
      }
    } else {
      // move the robot backwards
      robotMove(BACKWARD);
    }
  } else {
    robotMove(STOP);
  }
  return motorCommand;
}

/**
 * @brief callback function for receiving the map data
 * @param pointer to occupancy grid data
 */

void Explore::getMapData(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  // const bool chattyMap = true;
  mapMsg = *msg;
  // double mapWidth = mapMsg.info.width;
  // double mapHeight = mapMsg.info.width;
  // double mapOriginX = mapMsg.info.origin.position.x;
  // double mapOriginY = mapMsg.info.origin.position.y;
  // double mapOrientation = acos(mapMsg.info.origin.orientation.z);
  std::vector<signed char> map = mapMsg.data;

  int sumOfElems = 0;
  for (auto &n : map)
    sumOfElems += n;
  if (sumOfElems > -15940000)
    explored = true;
}
