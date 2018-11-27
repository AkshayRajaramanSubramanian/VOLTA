# VOLTA
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/bsaisudh/VOLTA/blob/master/LICENSE)
[![Build Status](https://travis-ci.org/bsaisudh/VOLTA.svg?branch=master)](https://travis-ci.org/bsaisudh/VOLTA)
[![Coverage Status](https://coveralls.io/repos/github/bsaisudh/VOLTA/badge.svg)](https://coveralls.io/github/bsaisudh/VOLTA)

##Overview
With increase in use of swarm of robots and each robot being limited by its power
capability, it is necessary for these robots to plan is charging cycle and recharge by
itself. With the above motivation, this project tries to mitigate the problem. The project
involves SLAM of a indoor space and marking the robot charging dock. This information
can be shared among a swarm of robots which can utilize the information at the state of
necessity.
The turtlebot-type robot will use monocular camera for localization and mapping the
indoor space. Each charging dock will have a sign board which will be used by the robot
for identification using image processing techniques and will be marked.
The packages will be developed with Test Driven Development, Pair programming and
agile development strategies to ensure accurate intended operation of the robot.
The software development process can be subdivided into four tasks: Charging station

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/bsaisudh/VOLTA/blob/master/LICENSE) file for details

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* catkin
* Ubuntu 16.04
* Gazebo

## Installation of Dependencies

* ROS Kinetic - Visit [ROS Kinetic installation](http://wiki.ros.org/kinetic/Installation) page and follow the steps.
* Catkin - Visit [catkin installation](http://wiki.ros.org/catkin) page and follow the steps.
* OpenCV - Visit [openCV][](https://opencv.org/about.html) page for information on useage and libraries avaialbe. 
* Rostest - Visit [rostest](http://wiki.ros.org/rostest) page for more information on useage and testing information.
* Gtest - Visit [gtest](http://wiki.ros.org/gtest) for instruction on how to use gtest gor ros packages.
* Travis CI (check bage above).
* Coveralls (check bage above).

## Agile Development Process
Pair programming is an agile software development technique in which two programmers work together at one workstation. One, the driver, writes code while the other, the observer or navigator, reviews each line of code as it is typed in. The two programmers switch roles frequently.. You can view the product backlog and development process in this [SPREADSHEET](https://docs.google.com/spreadsheets/d/1tfC8Jz-bgWB9GVRdzZb_OC4-lZSaKUidK2g5616C-TQ/edit#gid=904828225). Check [Sprint Review](https://docs.google.com/document/d/1OSMLGCIpMDP75UOx2Cv_yI2V-nHLj1y-wEeSGF1F2Mw/edit) document for more information on each sprint.

