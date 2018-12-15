/*
 * @file ROSExploreTest.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief Testing functions of ROSExplorer class
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

#include <cmath>
#include <algorithm>
#include <ros/console.h>
#include "../include/Explore.h"
#include "gtest/gtest.h"
#include "ros/service_client.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "../include/ROSExplore.h"

/**
 * @brief Fixture class for level 2 ros test
 */
class ExploreTest {
 public:
  ros::Subscriber msub;
  bool mcallback;
  ros::NodeHandle nh;
  geometry_msgs::Twist mCmd;
  /**
   * @brief Command call back for testing the message published by explore class
   */
  void callbackMotorCommand(const geometry_msgs::Twist &motorCommand) {
    mcallback = true;
    mCmd = motorCommand;
  }
  /**
   * @brief Constructor for the class
   * @param None
   * @return None
   */
  ExploreTest() {
    mcallback = false;
    msub = nh.subscribe("/mobile_base/commands/velocity", 10,
                        &ExploreTest::callbackMotorCommand, this);
  }
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  ~ExploreTest() {
  }
};

/**
 * @brief testing number of publishers for velocity
 */
TEST(ExploreTest, publishers) {
  ExploreTest t;
  ROSExplore ex(t.nh);
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  ASSERT_GE(2, t.msub.getNumPublishers());
}

/**
 * @brief testing if message is published to velocity topic
 */
TEST(ExploreTest, messageExist) {
  ExploreTest t;
  ROSExplore ex(t.nh);
  //ex.robot_move(GO_LEFT);
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  ASSERT_TRUE(t.mcallback);
}
