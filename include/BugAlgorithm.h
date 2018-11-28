/*
 * @file BugAlgorithm.h
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief header file for bug algorithm class.
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

#ifndef VOLTA_SRC_BUGALGORITHM_H_
#define VOLTA_SRC_BUGALGORITHM_H_

#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief Structure to hold 2D point
 */
struct point {
  int x;  // !<X co-ordinate of point
  int y;  // !<Y co-ordinate of point
};

/**
 * @brief Structure to hold attributes of path which are twist and duration
 */

struct pathAttribute {
  geometry_msgs::Twist twist;  // !<Velocity and Orientation of robot
  double duration;  // !<Duration for which robot has to travel in specified velocity
};

/**
 * @brief BugAlgorithm class generates the path that
 * robot has to traverse for generating SLAM
 */
class BugAlgorithm {
 public:
  // !<Velocity and duration for which robot will move closed path
  std::vector<pathAttribute> closedPath;
  // !<Boundary for slam
  std::vector<point> boundary;
  /**
   * @brief Generate closed path for robot
   * @param None
   * @return None
   */
  void generatePathPoints();
  /**
   * @brief Publish the path to control topic
   * @param None
   * @return None
   */
  void publishPath();
  /**
   * @brief Constructor for the class
   * @param None
   * @return None
   */
  BugAlgorithm();
  /**
   * @brief Destructor for class
   * @param None
   * @return None
   */
  virtual ~BugAlgorithm();
};

#endif /* VOLTA_SRC_BUGALGORITHM_H_ */
