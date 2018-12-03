/*
 * @file ChargeDockDetection.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief Class detects presence of charging dock in image captured by robot
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

#include "../include/ChargeDockDetection.h"

//using namespace cv;

ChargeDockDetection::ChargeDockDetection(ros::NodeHandle _nh)
    : nh(_nh),
      it(_nh) {
  imageSub = it.subscribe("/camera/rgb/image_raw", 100,
                          &ChargeDockDetection::checkForChargeDock, this);
  if (imageSub == NULL) {
    ROS_ERROR_STREAM("Images not getting read properly");
  }
  imagePub = it.advertise("/image_converter/output_video", 100);
  cv::namedWindow(OPENCV_WINDOW);
  cv::namedWindow(CHK_WINDOW);
}

void ChargeDockDetection::publishChargerDocPos() {

}

void ChargeDockDetection::checkForChargeDock(
    const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: "<< e.what());
    return;
  }
  // Draw an example circle on the video stream
  //if (cvPtr->image.rows > 60 && cvPtr->image.cols > 60)
  //  cv::circle(cvPtr->image, cv::Point(300, 300), 100, CV_RGB(255, 0, 0));
  //cv::rotate(cvPtr->image, cvPtr->image, cv::ROTATE_90_CLOCKWISE);
  // Update GUI Window
  //cv::imshow(OPENCV_WINDOW, cvPtr->image);
  checkerBoardDetect(cvPtr);
  cv::waitKey(1);
}

void ChargeDockDetection::checkerBoardDetect(cv_bridge::CvImagePtr cvPtr) {
  cv::Mat grey, img;
  cv::Size patternsize(7, 7);  //interior number of corners
  img = cvPtr->image;  // imread( "/home/bala/rosbag/test.jpg", IMREAD_COLOR  );//imread( argv[1], IMREAD_COLOR );
  cv::cvtColor(img, grey, CV_BGR2GRAY);
  std::vector<cv::Point2f> corners;  //this will be filled by the detected corners

  //CALIB_CB_FAST_CHECK saves a lot of time on images
  //that do not contain any chessboard corners
  bool patternfound = findChessboardCorners(
      grey,
      patternsize,
      corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
          + cv::CALIB_CB_FAST_CHECK);
  /*
   if(patternfound)
   {
   ROS_INFO_STREAM("Charging Dock Found; no corners : " << corners.size());
   cornerSubPix(grey, corners, cv::Size(11, 11), cv::Size(-1, -1),
   cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
   }
   else{
   ROS_INFO_STREAM("Charging Dock Not Found; no corners : " << corners.size());
   }
   */
  if (corners.size() > 30) {
    ROS_INFO_STREAM("Charging Dock Found; no corners : " << corners.size());
  } else {
    ROS_INFO_STREAM("Charging Dock Not Found; no corners : " << corners.size());
  }
  drawChessboardCorners(img, patternsize, cv::Mat(corners), patternfound);
  cv::imshow(CHK_WINDOW, img);
}

void ChargeDockDetection::findChargePosition() {

}

void ChargeDockDetection::svmTrainer() {

}

ChargeDockDetection::~ChargeDockDetection() {
  cv::destroyWindow(OPENCV_WINDOW);
  cv::destroyWindow(CHK_WINDOW);
}

