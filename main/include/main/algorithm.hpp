/**
 * algorithm.hpp
 *
 * Reads data from the leg_detector and attention_tracker and decide at which position the robot has to go to get commands.
 * Further a algorithm is implemented to choose a user to mitigate malfunctions.
 *
 * 
 * @author Thomas Weingartshofer thomas.wein@hotmail.com
 * @date April 2016
 */

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ios>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <main/algorithm.hpp>

class PeopleDetector
{
public:
  PeopleDetector();
  void run();

private:
  ros::NodeHandle nh;
  ros::Subscriber leg_sub;
  ros::Subscriber face_sub;

  std::vector<double> face_pos;
  std::vector<double> leg_pos;

  void legCallback(const std_msgs::String::ConstPtr& msg);
  void faceCallback(const std_msgs::String::ConstPtr& msg);
};

#endif
