/*
 * SLAM.h
 *
 *  Created on: Nov 27, 2018
 *      Author: bala
 */

#ifndef VOLTA_SRC_SLAM_H_
#define VOLTA_SRC_SLAM_H_

class SLAM {
 public:

  void getTurtlebotImages();
  void publishImage();
  void convertImageFormat();
  SLAM();
  virtual ~SLAM();
};

#endif /* VOLTA_SRC_SLAM_H_ */
