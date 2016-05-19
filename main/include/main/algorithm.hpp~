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
#include "std_msgs/Int32.h"
#include "ros/ros.h"

#include <main/algorithm.hpp>
#include <main/people_detection.hpp>

class Algorithm
{
public:
  Algorithm();
  void Normal(coordinates home,coordinates pointA, coordinates pointB);
  void Error(PeopleDetector people, double timeerror, double timethreshold, coordinates pointA, coordinates pointB);
  char returnStatus(void);

private:
  ros::NodeHandle nh;
  ros::Subscriber command_sub;
  ros::Publisher goal_pub;
  ros::Subscriber reachedgoal_sub;

  char status;
  char statusbefore;

  double timelastcmd;

  coordinates users_sorted[USERS];
  int index_sorted[USERS];
  int choosen_user;

  void commandCallback(const std_msgs::String::ConstPtr& msg);
  void reachedGoalCallback(const std_msgs::Int32::ConstPtr& msg);
};

#endif
