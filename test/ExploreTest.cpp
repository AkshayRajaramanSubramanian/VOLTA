/*
 * ExploreTest.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: bala
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


TEST(robot_move, stop) {
  // Assert
  Explore ex;
  // Act
  ex.robotMove(STOP);
  // Test
  EXPECT_DOUBLE_EQ(0.0, ex.motorCommand.angular.z);
  EXPECT_DOUBLE_EQ(0.0, ex.motorCommand.linear.x);
}

TEST(robot_move, forward) {
  // Assert
  Explore ex;
  // Act
  ex.robotMove(FORWARD);
  // Test
  EXPECT_DOUBLE_EQ(0.0, ex.motorCommand.angular.z);
  EXPECT_DOUBLE_EQ(0.5, ex.motorCommand.linear.x);
}

TEST(robot_move, backward) {
  // Assert
  Explore ex;
  // Act
  ex.robotMove(BACKWARD);
  // Test
  EXPECT_DOUBLE_EQ(0.0, ex.motorCommand.angular.z);
  EXPECT_DOUBLE_EQ(-0.5, ex.motorCommand.linear.x);
}

TEST(robot_move, trun_left) {
  // Assert
  Explore ex;
  // Act
  ex.robotMove(TURN_LEFT);
  // Test
  EXPECT_DOUBLE_EQ(0.5, ex.motorCommand.angular.z);
  EXPECT_DOUBLE_EQ(0.0, ex.motorCommand.linear.x);
}

TEST(robot_move, turn_right) {
  // Assert
  //ros::NodeHandle nh;
  Explore ex;
  // Act
  ex.robotMove(TURN_RIGHT);
  // Test
  EXPECT_DOUBLE_EQ(-0.5, ex.motorCommand.angular.z);
  EXPECT_DOUBLE_EQ(0.0, ex.motorCommand.linear.x);
}

TEST(robot_move, go_right) {
  // Assert
  Explore ex;
  // Act
  ex.robotMove(GO_RIGHT);
  // Test
  EXPECT_DOUBLE_EQ(-0.25, ex.motorCommand.angular.z);
  EXPECT_DOUBLE_EQ(0.25, ex.motorCommand.linear.x);
}

TEST(robot_move, go_left) {
  // Assert
  Explore ex;
  // Act
  ex.robotMove(GO_LEFT);
  // Test
  EXPECT_DOUBLE_EQ(0.25, ex.motorCommand.angular.z);
  EXPECT_DOUBLE_EQ(0.25, ex.motorCommand.linear.x);
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
/*
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
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  ASSERT_TRUE(t.mcallback);
}

TEST(ExploreTest, messageValue) {
  ExploreTest t;
  Explore ex(t.nh);
  ex.robot_move(GO_RIGHT);
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  EXPECT_DOUBLE_EQ(-0.25, t.mCmd.angular.z);
  EXPECT_DOUBLE_EQ(0.25, t.mCmd.linear.x);
}
*/

TEST(ExploreTest, backward) {
  sensor_msgs::LaserScan laserMsg;
  std::vector<float> range;
  range.push_back(NAN);
  range.push_back(NAN);
  range.push_back(NAN);
  range.push_back(NAN);
  laserMsg.ranges = range;
  laserMsg.range_max = 10.0;
  laserMsg.range_min = 0.449999988079;
  Explore ex;
  geometry_msgs::Twist motorCmd;
  motorCmd = ex.getLaserData(laserMsg);
  EXPECT_DOUBLE_EQ(0.0, motorCmd.angular.z);
  EXPECT_DOUBLE_EQ(-0.5, motorCmd.linear.x);
}

TEST(ExploreTest, forward) {
  sensor_msgs::LaserScan laserMsg;
  std::vector<float> range;
  range.push_back(4.0);
  range.push_back(4.0);
  range.push_back(4.0);
  range.push_back(4.0);
  laserMsg.ranges = range;
  laserMsg.range_max = 10.0;
  laserMsg.range_min = 0.449999988079;
  Explore ex;
  geometry_msgs::Twist motorCmd;
  motorCmd = ex.getLaserData(laserMsg);
  EXPECT_DOUBLE_EQ(0.0, motorCmd.angular.z);
  EXPECT_DOUBLE_EQ(0.5, motorCmd.linear.x);
}

TEST(ExploreTest, turnLeft) {
  sensor_msgs::LaserScan laserMsg;
  std::vector<float> range;
  range.push_back(6.0);
  range.push_back(6.0);
  range.push_back(0.4);
  range.push_back(0.4);
  laserMsg.ranges = range;
  laserMsg.range_max = 10.0;
  laserMsg.range_min = 0.449999988079;
  Explore ex;
  geometry_msgs::Twist motorCmd;
  motorCmd = ex.getLaserData(laserMsg);
  EXPECT_DOUBLE_EQ(0.5, motorCmd.angular.z);
  EXPECT_DOUBLE_EQ(0.0, motorCmd.linear.x);
}

TEST(ExploreTest, turnRight) {
  sensor_msgs::LaserScan laserMsg;
  std::vector<float> range;
  range.push_back(0.4);
  range.push_back(0.4);
  range.push_back(0.4);
  range.push_back(6.0);
  laserMsg.ranges = range;
  laserMsg.range_max = 10.0;
  laserMsg.range_min = 0.449999988079;
  Explore ex;
  geometry_msgs::Twist motorCmd;
  motorCmd = ex.getLaserData(laserMsg);
  EXPECT_DOUBLE_EQ(-0.5, motorCmd.angular.z);
  EXPECT_DOUBLE_EQ(0.0, motorCmd.linear.x);
}

TEST(ExploreTest, goRight) {
  sensor_msgs::LaserScan laserMsg;
  std::vector<float> range;
  range.push_back(1.4);
  range.push_back(1.6);
  range.push_back(5.4);
  range.push_back(5.6);
  laserMsg.ranges = range;
  laserMsg.range_max = 10.0;
  laserMsg.range_min = 0.449999988079;
  Explore ex;
  ex.thatsADoor=true;
  geometry_msgs::Twist motorCmd;
  motorCmd = ex.getLaserData(laserMsg);
  EXPECT_DOUBLE_EQ(-0.25, motorCmd.angular.z);
  EXPECT_DOUBLE_EQ(0.25, motorCmd.linear.x);
}

