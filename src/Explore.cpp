/*
 * @file Explore.cpp
 * @Copyright MIT license
 * Copyright (c) 2018 Akshay Rajaraman
 * @author Akshay Rajaraman
 * @brief class that describes the function of the explore node
 *
 */



/*
 * MIT License
 *
 * Copyright (c) 2018 Akshay Rajaraman
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

#include "../include/Explore.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>
#define PI 3.141592
/**
 * @brief constructor for initializing Explore object
 * @param NodeHandle n
 */

Explore::Explore(ros::NodeHandle &n) : n(n) {
	motor_command_publisher = n.advertise<geometry_msgs::Twist> ( "/mobile_base/commands/velocity", 100 );
    laser_subscriber = n.subscribe ( "/scan", 1000, &Explore::getLaserData, this);
    map_subscriber = n.subscribe ( "/map", 1000, &Explore::getMapData, this);
}

Explore::~Explore() {}

/**
 * @brief callback function that runs when data is being subscribed from the robot
 */
bool Explore::robot_move ( const ROBOT_MOVEMENT move_type ) {
    if ( move_type == STOP ) {
        
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
    }

    else if ( move_type == FORWARD ) {
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.5;
    }

    else if ( move_type == BACKWARD ) {
        motor_command.linear.x = -0.75;
        motor_command.angular.z = 0.0;
    }

    else if ( move_type == TURN_LEFT ) {
        motor_command.linear.x = 0.0;
        motor_command.angular.z = 1.0;
    }

    else if ( move_type == TURN_RIGHT ) {
        motor_command.linear.x = 0.0;
        motor_command.angular.z = -1.0;
    } 
    else if ( move_type == GO_RIGHT ) {
        motor_command.linear.x = 0.25;
        motor_command.angular.z = -0.25;
    }
    else if ( move_type == GO_LEFT ) {
        motor_command.linear.x = 0.25;
        motor_command.angular.z = 0.25;
    }
    else {
        return false;
    }

    motor_command_publisher.publish ( motor_command );
    usleep(10);
    return true;
}

void Explore::getLaserData ( const sensor_msgs::LaserScan::ConstPtr &scan_msg ) {
    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();    
    float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_max;
	float range_max = laser_msg.range_min;
    int nan_count = 0;
    for(size_t i = 0; i < range_size; i++){
        if (laser_ranges[i] < range_min){
            range_min = laser_ranges[i];
        }
        
        if (std::isnan(laser_ranges[i])){
            nan_count++;
        }
        if (i < range_size / 4 ) {
            if (laser_ranges[i] > range_max){
                range_max = laser_ranges[i];
            }
        }
        
        if (i > range_size / 2){
            left_side += laser_ranges[i];
        }
        else {
            right_side += laser_ranges[i];
        }
    }
    
    if (nan_count > (range_size * 0.9) || laser_ranges[range_size / 2] < 0.25) {
        crashed = true;
    }
    else {
        crashed = false;
    }
    if (!crashed) {
        
        if (range_min <= 0.5 && !thats_a_door){
            following_wall = true;
            crashed = false;
            robot_move(STOP);
            
            if (left_side >= right_side) {
                robot_move(TURN_RIGHT);
            }
            else {
                robot_move(TURN_LEFT);  
            }
        }
        else {
            robot_move(STOP);
            if ( following_wall ) {
                if (range_max >= 1.5){
                    thats_a_door = true;
                    following_wall = false;
                }
            } 
            if (thats_a_door) {
                if (laser_ranges[0] <= 0.5){
                    thats_a_door = false;
                }
                else {
                    robot_move(GO_RIGHT);  
                }
                
            }
            else {
                robot_move(FORWARD);  
            }
        }
    }
    else {
        robot_move(BACKWARD);
    }
}

void Explore::getMapData ( const nav_msgs::OccupancyGrid::ConstPtr &msg ) {
    const bool chatty_map = true;
    map_msg = *msg;
    double map_width = map_msg.info.width;
    double map_height = map_msg.info.width;
    double map_origin_x = map_msg.info.origin.position.x;
    double map_origin_y = map_msg.info.origin.position.y;
    double map_orientation = acos ( map_msg.info.origin.orientation.z );
    std::vector<signed char > map = map_msg.data;
}

