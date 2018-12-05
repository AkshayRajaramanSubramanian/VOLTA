/*
 * @file ChargeDockDetection.h
 * @Copyright MIT license
 * Copyright (c) 2018 Bala Murali Manoghar Sai Sudhakar, Akshay Rajaraman
 * @author Bala Murali Manoghar Sai Sudhakar
 * @author Akshay Rajaraman
 * @brief Header file for bug ChargeDockDetection class.
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

#ifndef VOLTA_SRC_CHARGEDOCKDETECTION_H_
#define VOLTA_SRC_CHARGEDOCKDETECTION_H_

#include "Explore.h"

/**
 * @brief Class runs image processing on captured images to detect charging dock
 */
class ChargeDockDetection {
 public:
  // <!Coordinates of charging dock
  point chargeMarker;
  /**
   * @brief Publish charge dock coordinates to chargeDock topic
   * @param None
   * @return None
   */
  void publishChargerDocPos();
  /**
   * @brief Image processing algorithm to find presence of charging dock
   * @param None
   * @return None
   */
  void checkForChargeDock();
  /**
   * @brief Calculate charging dock coordinates with respect to map frame
   * @param None
   * @return None
   */
  void findChargePosition();
  /**
   * @brief Training module for charge dock detection algorithm
   * @param None
   * @return None
   */
  void svmTrainer();
  /**
   * @brief Constructor for the class
   * @param None
   * @return None
   */
  ChargeDockDetection();
  /**
   * @brief Destructor for the class
   * @param None
   * @return None
   */
  virtual ~ChargeDockDetection();
};

#endif /* VOLTA_SRC_CHARGEDOCKDETECTION_H_ */
