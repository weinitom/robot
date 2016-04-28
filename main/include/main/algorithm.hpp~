/**
 * algorithm.hpp
 *
 * This is the main algorithm, it gets information from the people_detection where users are located and further 
 * choose a user to mitigate malfunctions.
 * 
 * @author Thomas Weingartshofer thomas.wein@hotmail.com
 * @date April 2016
 */

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <ios>
#include "std_msgs/String.h"
#include "ros/ros.h"

#include <main/algorithm.hpp>
#include <main/people_detection.hpp>

class Algorithm
{
public:
  Algorithm();

private:
  ros::NodeHandle nh;
  ros::Subscriber error_sub;


  void errorCallback(const std_msgs::String::ConstPtr& msg);
};

#endif
