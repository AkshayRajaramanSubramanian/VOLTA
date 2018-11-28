/*
 * @file BugControl.h
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief header file for bug algorithm.
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

#ifndef VOLTA_SRC_BUGCONTROL_H_
#define VOLTA_SRC_BUGCONTROL_H_

#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "BugAlgorithm.h"

/**
 * @brief Class controls robot movement
 */
class BugControl {
 public:
  // !<Velocity and Orientation of robot
  geometry_msgs::Twist twist;
  // !<Velocity and duration for which robot will move closed path
  std::vector<pathAttribute> closedPath;

  /**
   * @brief Move robot to follow path generated for SLAM
   * @param None
   * @return None
   */
  void pathTraverse();
  /**
   * @brief Publish wheel velocity and orientation to bot topic
   * @param None
   * @return None
   */
  void pubishControlVariables();
  /**
   * @brief Constructor for the class
   * @param None
   * @return None
   */
  BugControl();
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  virtual ~BugControl();
};

#endif /* VOLTA_SRC_BUGCONTROL_H_ */
