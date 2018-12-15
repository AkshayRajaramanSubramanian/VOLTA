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
#include <vector>
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud2.h"

class ChargeDockDetectionTest {
 public:
	bool isImagePublished;
	bool isDepthPublished;
	bool isImageConverted;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber imageSub;
	ros::Subscriber depthSub;
	sensor_msgs::PointCloud2ConstPtr pointCloud;
	image_transport::Subscriber convertedImage;
	sensor_msgs::ImageConstPtr imageRaw;
	void callbackImageSub (const sensor_msgs::ImageConstPtr& msg) {
		isImagePublished = true;
		imageRaw = msg;
	}
	void callbackDepthSub (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
		isDepthPublished = true;
		pointCloud = cloud_msg;
	}
	void callbackConvertedImage (const sensor_msgs::ImageConstPtr& msg) {
		isImageConverted = true;
	}
	ChargeDockDetectionTest(ros::NodeHandle &_nh) : nh(_nh), it(_nh){
		isImagePublished = false;
		isDepthPublished = false;
		isImageConverted = false;
		imageSub = it.subscribe("/camera/rgb/image_raw", 1, &ChargeDockDetectionTest::callbackImageSub, this);
		depthSub = nh.subscribe("/camera/depth/points", 1, &ChargeDockDetectionTest::callbackDepthSub, this);
		convertedImage = it.subscribe("/image_converter/output_video", 1, &ChargeDockDetectionTest::callbackConvertedImage, this);
	}
	~ChargeDockDetectionTest() {
	}
};

TEST(centroidTest, checkCentroidValue) {
	ChargeDockDetection chargeDockDetection;
	cv::Point2f point;
	std::vector<cv::Point2f> points;
	point.x = 0;
	point.y = 0;
	points.push_back(point);
	point.x = 10;
	point.y = 0;
	points.push_back(point);
	cv::Point2f output;
	output = chargeDockDetection.centroid(points);
	EXPECT_EQ(output.x,5);
}

TEST(broadcastTflocalTest, checkLocalBroadcast) {
	ChargeDockDetection chargeDockDetection;
	tf::Transform output;
	output = chargeDockDetection.broadcastTflocal(5,0,0);
	EXPECT_EQ(output.getOrigin().x(),5);
}

TEST(broadcastTfodomTest, checkOdomBroadcast) {
	ChargeDockDetection chargeDockDetection;
	tf::Transform local;
	local = chargeDockDetection.broadcastTflocal(5,0,0);
	tf::StampedTransform input(local,ros::Time(),"/volta","/odom");
	tf::Transform output;
	float x = 15;
	float y = 0;
	float z = 0;
	output = chargeDockDetection.broadcastTfodom(input,x,y,z);
	EXPECT_EQ(output.getOrigin().x(),20);
}

TEST(depthcallbackTest, checkTrue) {
	ros::NodeHandle n;
	ChargeDockDetection chargeDockDetection;
	ChargeDockDetectionTest chargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
	ros::spinOnce();
	EXPECT_TRUE(chargeDockDetection.depthcallback(chargeDockDetectionTest.pointCloud));
}

TEST(getXYZTest, checkTrueValue) {
	ros::NodeHandle n;
	ChargeDockDetection chargeDockDetection;
	ChargeDockDetectionTest chargeDockDetectionTest(n);
  float x;
	float y;
	float z;
  ros::Duration(1.0).sleep();
	ros::spinOnce();
	chargeDockDetection.depthcallback(chargeDockDetectionTest.pointCloud);
	EXPECT_TRUE(chargeDockDetection.getXYZ(5,0,x,y,z));
}

TEST(getCheckForChargeDockTest, checkCvPtr) {
	ros::NodeHandle n;
	ChargeDockDetection chargeDockDetection;
	ChargeDockDetectionTest chargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
	ros::spinOnce();
	cv_bridge::CvImagePtr cvPtr = chargeDockDetection.checkForChargeDock(chargeDockDetectionTest.imageRaw);
  cv::Mat img = cvPtr->image;
	EXPECT_NE(img.size().width,0);
}

TEST(checkerBoardDetectTest, checkImage) {
	ros::NodeHandle n;
	ChargeDockDetection chargeDockDetection;
	ChargeDockDetectionTest chargeDockDetectionTest(n);
  ros::Duration(1.0).sleep();
	ros::spinOnce();
  std::vector<cv::Point2f> corners;
	cv_bridge::CvImagePtr cvPtr = chargeDockDetection.checkForChargeDock(chargeDockDetectionTest.imageRaw);
  cv::Mat img = chargeDockDetection.checkerBoardDetect(cvPtr,corners);
	EXPECT_NE(img.size().width,0);
}

