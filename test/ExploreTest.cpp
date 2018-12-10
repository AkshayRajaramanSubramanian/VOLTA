/*
 * ExploreTest.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: bala
 */
#include <ros/console.h>

#include "../include/Explore.h"

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"

#include <math.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

TEST(robot_move, stop) {
  // Assert
  ros::NodeHandle nh;
  Explore ex(nh);
  // Act
  ex.robot_move(STOP);
  // Test
  EXPECT_DOUBLE_EQ(0.0, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(0.0, ex.motor_command.linear.x);
}

TEST(robot_move, forward) {
  // Assert
  ros::NodeHandle nh;
  Explore ex(nh);
  // Act
  ex.robot_move(FORWARD);
  // Test
  EXPECT_DOUBLE_EQ(0.0, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(0.5, ex.motor_command.linear.x);
}

TEST(robot_move, backward) {
  // Assert
  ros::NodeHandle nh;
  Explore ex(nh);
  // Act
  ex.robot_move(BACKWARD);
  // Test
  EXPECT_DOUBLE_EQ(0.0, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(-0.75, ex.motor_command.linear.x);
}

TEST(robot_move, trun_left) {
  // Assert
  ros::NodeHandle nh;
  Explore ex(nh);
  // Act
  ex.robot_move(TURN_LEFT);
  // Test
  EXPECT_DOUBLE_EQ(1.0, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(0.0, ex.motor_command.linear.x);
}

TEST(robot_move, turn_right) {
  // Assert
  ros::NodeHandle nh;
  Explore ex(nh);
  // Act
  ex.robot_move(TURN_RIGHT);
  // Test
  EXPECT_DOUBLE_EQ(-1.0, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(0.0, ex.motor_command.linear.x);
}

TEST(robot_move, go_right) {
  // Assert
  ros::NodeHandle nh;
  Explore ex(nh);
  // Act
  ex.robot_move(GO_RIGHT);
  // Test
  EXPECT_DOUBLE_EQ(-0.25, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(0.25, ex.motor_command.linear.x);
}

TEST(robot_move, go_left) {
  // Assert
  ros::NodeHandle nh;
  Explore ex(nh);
  // Act
  ex.robot_move(GO_LEFT);
  // Test
  EXPECT_DOUBLE_EQ(0.25, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(0.25, ex.motor_command.linear.x);
}
/*
void callbackMotorCommand(const geometry_msgs::Twist &motor_command) {

}
*/
/*
 TEST(test_1,test) {
 bool callback;
 ros::Subscriber sub;
 ros::NodeHandle nh;
 callback = false;
 sub = nh.subscribe("/mobile_base/commands/velocity", 10,
 &callbackMotorCommand);
 }
 */

class ExploreTest {
 public:
  ros::Subscriber msub, lsub;
  ros::Publisher lpub;
  bool mcallback, lcallback;
  ros::NodeHandle nh;
  geometry_msgs::Twist mCmd;
  sensor_msgs::LaserScan laserMsg;

  void callbackMotorCommand(const geometry_msgs::Twist &motorCommand) {
    mcallback = true;
    mCmd = motorCommand;
  }

  void getLaserData( const sensor_msgs::LaserScan::ConstPtr &scanMsg ){
    lcallback = true;
    laserMsg = *scanMsg;
  }

  void createLaserPublisher(){
    lpub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
  }

  ExploreTest() {
    mcallback = false;
    lcallback = false;
    msub = nh.subscribe("/mobile_base/commands/velocity", 10,
                       &ExploreTest::callbackMotorCommand, this);
    //lsub = nh.subscribe("/scan", 10,
    //                       &ExploreTest::getLaserData, this);
  }

  ~ExploreTest() {
  }

};

TEST(ExploreTest, publishers) {
  ExploreTest t;
  Explore ex(t.nh);
  ex.robot_move(GO_LEFT);
  ros::Duration(2.0).sleep();
  ASSERT_STREQ("/mobile_base/commands/velocity", t.msub.getTopic().c_str());
  ASSERT_GE(2,t.msub.getNumPublishers());
  ros::spinOnce();
}

TEST(ExploreTest, messageExist) {
  ExploreTest t;
  Explore ex(t.nh);
  ex.robot_move(GO_LEFT);
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  ASSERT_TRUE(t.mcallback);
}

TEST(ExploreTest, messageValue) {
  ExploreTest t;
  Explore ex(t.nh);
  ex.robot_move(GO_RIGHT);
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  EXPECT_DOUBLE_EQ(-0.25, t.mCmd.angular.z);
  EXPECT_DOUBLE_EQ(0.25, t.mCmd.linear.x);
}

TEST(ExploreTest, laserCallbackTest) {
  sensor_msgs::LaserScan laserMsg;
  std::vector<float> range;
  range.push_back(NAN);
  range.push_back(NAN);
  range.push_back(NAN);
  range.push_back(NAN);
  laserMsg.ranges = range;
  laserMsg.range_max = 10.0;
  laserMsg.range_min = 0.449999988079;

  ExploreTest t;
  Explore ex(t.nh);
  t.createLaserPublisher();
  t.lpub.publish(laserMsg);
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  EXPECT_DOUBLE_EQ(0.0, ex.motor_command.angular.z);
  EXPECT_DOUBLE_EQ(-0.75, ex.motor_command.linear.x);
}
