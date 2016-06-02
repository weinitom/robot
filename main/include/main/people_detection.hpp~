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
#include <tf/transform_listener.h>

#include <main/algorithm.hpp>
#include <main/people_detection.hpp>

#define USERS 10

struct coordinates{
  double x,y,z;
};

class PeopleDetector
{
public:
  PeopleDetector();
  void detectUsers(void);
  coordinates getUserCoordinates(int index);
  void getClosestUsers(int *index_sorted);
  void setRange(double range);
  int getAttention(int index);
  int getUserSize(void);

private:
  ros::NodeHandle nh;
  ros::Subscriber leg_sub;
  ros::Subscriber face_sub;
  ros::Subscriber attention_sub;

  ros::Publisher face_pose;
  ros::Publisher attention_pose;
  ros::Publisher leg_pose;

  tf::TransformListener *leglistener;
  tf::TransformListener *facelistener;

  coordinates face_pos[USERS];
  coordinates leg_pos[USERS];
  int attention_pos[USERS];

  coordinates users[USERS];

  double range;

  void legCallback(const std_msgs::String::ConstPtr& msg);
  void faceCallback(const std_msgs::String::ConstPtr& msg);
  void attentionCallback(const std_msgs::String::ConstPtr& msg);
};

#endif
