/**
 * algorithm.cpp
 *
 * This is the main algorithm, it gets information from the people_detection where users are located and further 
 * choose a user to mitigate malfunctions.
 * 
 * @author Thomas Weingartshofer thomas.wein@hotmail.com
 * @date April 2016
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <ios>
#include "std_msgs/String.h"
#include "ros/ros.h"

#include <main/algorithm.hpp>
#include <main/people_detection.hpp>

void Algorithm::errorCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_ERROR("Error: [%s]", msg->data.c_str());

// To send error message over topic:
// rostopic pub error std_msgs/String "test"

}

Algorithm::Algorithm()
{
  error_sub = nh.subscribe("error", 100, &Algorithm::errorCallback,this);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "algorithm");

  PeopleDetector people;
  Algorithm alg;

  ros::spin();

  return 0;
}
