# VOLTA
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/bsaisudh/VOLTA/blob/master/LICENSE)
[![Build Status](https://travis-ci.org/bsaisudh/VOLTA.svg?branch=master)](https://travis-ci.org/bsaisudh/VOLTA)
[![Coverage Status](https://coveralls.io/repos/github/bsaisudh/VOLTA/badge.svg)](https://coveralls.io/github/bsaisudh/VOLTA)

## Overview
With increase in use of swarm of robots and each robot being limited by its power
capability, it is necessary for these robots to plan their charging cycle and recharge by
themselves.
</br>
</br>
With the above motivation, this project tries to mitigate the problem. The project
involves SLAM of an indoor space and marking the robot charging docks. This information
can be shared among a swarm of robots which can utilize the information at the state of
necessity.
</br>
</br>
The repository is a ROS package implementing a simple charge dock detection and mapping algorithm with the Turtlebot2 robot. The turtlebot will use the kinect camera for localization and mapping the
indoor space. Each charging dock will have a checkerboard which will be used by the robot
for identification using image processing techniques and will be marked.
</br>
</br>
The packages will be developed with Test Driven Development, Pair programming and
agile development strategies to ensure accurate intended operation of the robot.

## About the Authors

[Bala Murali Manoghar Sai Sudhakar](https://www.linkedin.com/in/bala-murali-manoghar/) : </br>
_"I am a student currently pursuing Masters in Robotics at the University of Maryland with a background in mechatronics. I am interested in the Industrial Applications of Robotics"_ </br>
[Akshay Rajaraman](https://www.linkedin.com/in/akshay-rajaraman/) : </br>
_"I am a student currently pursuing Masters in Robotics at the University of Maryland. My interests include Motion Planning, Machine Learning and Biomedical Robotics."_

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/bsaisudh/VOLTA/blob/master/LICENSE) file for details

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* catkin
* Ubuntu 16.04
* Gazebo
* gmapping
* OpenCV
* Gtest
* rviz_launchers

## Installation of Dependencies

* ROS Kinetic - Visit [ROS Kinetic installation](http://wiki.ros.org/kinetic/Installation) page and follow the steps.
* Catkin - Visit [catkin installation](http://wiki.ros.org/catkin) page and follow the steps.
* OpenCV - Visit [openCV][](https://opencv.org/about.html) page for information on useage and libraries avaialbe. 
* Rostest - Visit [rostest](http://wiki.ros.org/rostest) page for more information on useage and testing information.
* Gtest - Visit [gtest](http://wiki.ros.org/gtest) for instruction on how to use gtest gor ros packages.
* Travis CI (check bage above).
* Coveralls (check bage above).


## Package Dependencies
geometry_msgs </br>
nav_msgs </br>
visualization_msgs </br>
cv_bridge </br>
image_transport </br>
sensor_msgs </br>
pcl_conversions </br>
pcl_ros </br>
message_generation </br>

## Agile Development Process
Pair programming is an agile software development technique in which two programmers work together at one workstation. One, the driver, writes code while the other, the observer or navigator, reviews each line of code as it is typed in. The two programmers switch roles frequently.. You can view the product backlog and development process in this [SPREADSHEET](https://docs.google.com/spreadsheets/d/1tfC8Jz-bgWB9GVRdzZb_OC4-lZSaKUidK2g5616C-TQ/edit#gid=904828225). Check [Sprint Review](https://docs.google.com/document/d/1OSMLGCIpMDP75UOx2Cv_yI2V-nHLj1y-wEeSGF1F2Mw/edit) document for more information on each sprint.

## Build
Run the following scripts in your terminal to build the package
```
cd <path_to_catkin_workspace>/src
git clone --recursive https://github.com/bsaisudh/VOLTA.git
cd ..
catkin_make
```

## Run
The nodes needed to run this package can be launched by running the following scripts in your terminal
```
source <path_to_catkin_workspace>/devel/setup.bash
roslaunch volta volta.launch
```

## Tests
The google unit tests and rostests can be executed by running the following scripts in the terminal
```
cd <path_to_catkin_workspace>
source devel/setup.bash
catkin_make run_tests volta
```

## About volta's environment
The robot is placed in a square room of size 10m X 10m. The room is assumed to have no obstacles in the path of the robot. </br>
The walls of the room are made of bricks to ensure efficient feature detection for mapping. </br>
The charging  points in the room are represented by checkerboards attached to the wall as shown below :
<p align="center"> 
<img src="https://raw.githubusercontent.com/bsaisudh/VOLTA/master/readme_images/gazeboWorld.png">
</p>

## Results 
The map generated with Markers in the location of the charging ports is as shown below : 
<p align="center"> 
<img src="https://raw.githubusercontent.com/bsaisudh/VOLTA/master/readme_images/rvizMapResults.png">
</p>

## Demonstration Video
_Link to demo video will be updated soon_

## Known Issues
* There is a lot of odometry correction, which causes the turtlebot to re-orient itself constantly. This causes some distortion in the map generated by the gmapping package.
* The maximum range of the kinect sensor on the turtlebot is 5m. Hence, if the robot was placed in an environment where no features can be detected by it, it would cause significant errors in the map generated by the gmapping package.
