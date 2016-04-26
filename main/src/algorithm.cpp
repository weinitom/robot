/**
 * algorithm.cpp
 *
 * Reads data from the leg_detector and attention_tracker and decide at which position the robot has to go to get commands.
 * Further a algorithm is implemented to choose a user to mitigate malfunctions.
 *
 * 
 * @author Thomas Weingartshofer thomas.wein@hotmail.com
 * @date April 2016
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ios>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <main/algorithm.hpp>

void PeopleDetector::faceCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Faces: [%s]", msg->data.c_str());
}

void PeopleDetector::legCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Legs: [%s]", msg->data.c_str());
}

PeopleDetector::PeopleDetector()
{
  leg_sub = nh.subscribe("laser_person", 100, &PeopleDetector::legCallback,this);
  face_sub = nh.subscribe("face_pos", 100, &PeopleDetector::faceCallback,this);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "algorithm");

  PeopleDetector people;

  ros::spin();

  return 0;
}
