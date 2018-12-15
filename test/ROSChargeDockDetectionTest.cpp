#include <ros/console.h>

#include "../include/ROSChargeDockDetection.h"
#include "../include/ChargeDockDetection.h"
#include "../include/ChargeDock.h"
#include "../include/ROSChargeDock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>

class ROSChargeDockDetectionTest {
 public:
	bool isImagePublished;
	bool isDepthPublished;
	bool isImageConverted;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber imageSub;
	ros::Subscriber depthSub;
	image_transport::Subscriber convertedImage;
	void callbackImageSub (const sensor_msgs::ImageConstPtr& msg) {
		isImagePublished = true;
	}
	void callbackDepthSub (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
		isDepthPublished = true;
	}
	void callbackConvertedImage (const sensor_msgs::ImageConstPtr& msg) {
		isImageConverted = true;
	}
	ROSChargeDockDetectionTest(ros::NodeHandle &_nh) : nh(_nh), it(_nh){
		isImagePublished = false;
		isDepthPublished = false;
		isImageConverted = false;
		imageSub = it.subscribe("/camera/rgb/image_raw", 1, &ROSChargeDockDetectionTest::callbackImageSub, this);
		depthSub = nh.subscribe("/camera/depth/points", 1, &ROSChargeDockDetectionTest::callbackDepthSub, this);
		convertedImage = it.subscribe("/image_converter/output_video", 1, &ROSChargeDockDetectionTest::callbackConvertedImage, this);
	}
	~ROSChargeDockDetectionTest() {
	}
};

TEST(inputTest, checkImageInput) {
  ros::NodeHandle n;
  ROSChargeDockDetectionTest rosChargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(rosChargeDockDetectionTest.isImagePublished);
}

TEST(inputTest, checkDepthInput) {
  ros::NodeHandle n;
  ROSChargeDockDetectionTest rosChargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(rosChargeDockDetectionTest.isDepthPublished);
}

TEST(outputTest, checkConvertedImage) {
  ros::NodeHandle n;
  ROSChargeDockDetectionTest rosChargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_TRUE(rosChargeDockDetectionTest.isImageConverted);
}



