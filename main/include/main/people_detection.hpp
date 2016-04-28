/**
 * people_detection.hpp
 *
 * Reads data from the leg_detector and attention_tracker and detect the position of the users.
 * 
 * @author Thomas Weingartshofer thomas.wein@hotmail.com
 * @date April 2016
 */

#ifndef PEOPLEDETECTION_H
#define PEOPLEDETECTION_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <ios>
#include "std_msgs/String.h"
#include "ros/ros.h"

#include <main/algorithm.hpp>
#include <main/people_detection.hpp>

struct coordinates{
  double x,y,z;
};

class PeopleDetector
{
public:
  PeopleDetector();

private:
  ros::NodeHandle nh;
  ros::Subscriber leg_sub;
  ros::Subscriber face_sub;
  ros::Subscriber attention_sub;

  coordinates face_pos[10];
  coordinates leg_pos[10];
  coordinates attention_pos[10];

  void legCallback(const std_msgs::String::ConstPtr& msg);
  void faceCallback(const std_msgs::String::ConstPtr& msg);
  void attentionCallback(const std_msgs::String::ConstPtr& msg);
};

#endif
