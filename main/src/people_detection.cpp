/**
 * people_detection.cpp
 *
 * Reads data from the leg_detector and attention_tracker and detect the position of the users.
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

void PeopleDetector::faceCallback(const std_msgs::String::ConstPtr& msg)
{
  const char* str = msg->data.c_str();

  int i=0;
  int count=1;
  int index=0;

  for (i = 0; i < strlen(str); i++)
  {
      if (str[i] == '\n' || count==1)
      {
         if(i==0)
         {
            face_pos[index].x = atof(&str[i]);
         }
         else if(index<10)
         {
            face_pos[index].x = atof(&str[i+1]);
         }
         count=2;
         continue;
      }
      if (str[i] == ' ' && count==2)
      {
         face_pos[index].y = atof(&str[i+1]);
         count=3;
         continue;
      }
      if (str[i] == ' ' && count==3)
      {
         face_pos[index].z = atof(&str[i+1]);
         count=1;
         index++;
         continue;
      } 
  }
  //ROS_INFO("Faces: [%s]",str);
  for (i = 0; i < index; i++)
  {
     ROS_INFO("Face %d: [x=%f y=%f z=%f]",i+1,face_pos[i].x,face_pos[i].y,face_pos[i].z);
  }
}

void PeopleDetector::legCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Legs: [%s]", msg->data.c_str());
}

void PeopleDetector::attentionCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Attention: [%s]", msg->data.c_str());
}

PeopleDetector::PeopleDetector()
{
  leg_sub = nh.subscribe("laser_person", 100, &PeopleDetector::legCallback,this);
  face_sub = nh.subscribe("face_pos", 100, &PeopleDetector::faceCallback,this);
  attention_sub = nh.subscribe("attentive_faces", 100, &PeopleDetector::attentionCallback,this);
}
