/*
 * ExploreTest.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: bala
 */

#include "../include/Explore.h"

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/custom_message.h"

TEST(robot_move, stop){
  // Assert
 ros::NodeHandle nh;
 Explore ex(nh);
 // Act
 ex.robot_move(STOP);
 // Test
 EXPECT_DOUBLE_EQ(0.0,ex.motor_command.angular.z);
 EXPECT_DOUBLE_EQ(0.0,ex.motor_command.linear.x);
}

TEST(robot_move, forward){
  // Assert
 ros::NodeHandle nh;
 Explore ex(nh);
 // Act
 ex.robot_move(FORWARD);
 // Test
 EXPECT_DOUBLE_EQ(0.0,ex.motor_command.angular.z);
 EXPECT_DOUBLE_EQ(0.5,ex.motor_command.linear.x);
}

TEST(robot_move, backward){
  // Assert
 ros::NodeHandle nh;
 Explore ex(nh);
 // Act
 ex.robot_move(BACKWARD);
 // Test
 EXPECT_DOUBLE_EQ(0.0,ex.motor_command.angular.z);
 EXPECT_DOUBLE_EQ(-0.75,ex.motor_command.linear.x);
}

TEST(robot_move, trun_left){
  // Assert
 ros::NodeHandle nh;
 Explore ex(nh);
 // Act
 ex.robot_move(TURN_LEFT);
 // Test
 EXPECT_DOUBLE_EQ(0.1,ex.motor_command.angular.z);
 EXPECT_DOUBLE_EQ(0.0,ex.motor_command.linear.x);
}

TEST(robot_move, turn_right){
  // Assert
 ros::NodeHandle nh;
 Explore ex(nh);
 // Act
 ex.robot_move(TURN_RIGHT);
 // Test
 EXPECT_DOUBLE_EQ(-1.0,ex.motor_command.angular.z);
 EXPECT_DOUBLE_EQ(0.0,ex.motor_command.linear.x);
}

TEST(robot_move, go_right){
  // Assert
 ros::NodeHandle nh;
 Explore ex(nh);
 // Act
 ex.robot_move(GO_RIGHT);
 // Test
 EXPECT_DOUBLE_EQ(-0.25,ex.motor_command.angular.z);
 EXPECT_DOUBLE_EQ(0.25,ex.motor_command.linear.x);
}

TEST(robot_move, go_left){
  // Assert
 ros::NodeHandle nh;
 Explore ex(nh);
 // Act
 ex.robot_move(GO_LEFT);
 // Test
 EXPECT_DOUBLE_EQ(0.25,ex.motor_command.angular.z);
 EXPECT_DOUBLE_EQ(0.25,ex.motor_command.linear.x);
}