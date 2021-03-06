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
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <main/algorithm.hpp>
#include <main/people_detection.hpp>

#define TRANSFORM 1
#define OUTPUT 1

void PeopleDetector::faceCallback(const std_msgs::String::ConstPtr& msg)
{
  const char* str = msg->data.c_str();	// define shortcut

  // define variables
  int i=0;
  int count=1;
  int index=0;

  // read out face coordinates (x y z) from string (0.000000 0.000000 0.000000\n0.000000 0.000000 0.000000\n...)
  for (i = 0; i < strlen(str); i++)
  {
      if (str[i] == '\n' || count==1)
      {
         if(i==0)
         {
            face_pos[index].x = atof(&str[i]);
         }
         else if(index<USERS)
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

  ros::Time time=ros::Time::now();		// save current time

  // generate and transform PoseStamped Message
  for (i = 0; i < index; i++)
  {
     // generate PoseStamped Message
     geometry_msgs::PoseStamped msg;

     msg.pose.position.x=face_pos[i].x;
     msg.pose.position.y=face_pos[i].y;
     msg.pose.position.z=face_pos[i].z;
     msg.pose.orientation.w=1;

     msg.header.frame_id="kinect_link";
     msg.header.stamp.sec=time.sec;
     msg.header.stamp.nsec=time.nsec;

     // generate new transformed PoseStamped Message
     geometry_msgs::PoseStamped msg_trans;
     msg_trans.header=msg.header;
 
     #if TRANSFORM
     // Transform PoseStamped Message in map frame
     tf::TransformListener listener;
     while (ros::ok() && !facelistener->waitForTransform("kinect_link", "base_link", ros::Time::now(), ros::Duration(5.0))) {
        ROS_ERROR("Face-Transformation not ready yet");
        ros::Duration(1).sleep();
     }
     try{
      facelistener->transformPose("base_link", msg, msg_trans);
     }
     catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
     }
     #else
     msg_trans=msg;
     #endif

     // publish transformed PoseStamped Message
     face_pose.publish(msg_trans);
     face_pos[i].x=msg_trans.pose.position.x;
     face_pos[i].y=msg_trans.pose.position.y;
     face_pos[i].z=msg_trans.pose.position.z;

     // show face positions
     #if OUTPUT
     ROS_INFO("Face %d: [x=%f y=%f z=%f]",i+1,face_pos[i].x,face_pos[i].y,face_pos[i].z);
     #endif
  }

}

void PeopleDetector::legCallback(const std_msgs::String::ConstPtr& msg)
{
  const char* str = msg->data.c_str();	// define shortcut

  // define variables
  int i=0;
  int count=1;
  int index=0;

  // read out leg coordinates (x y) from string (0.000000 0.000000\n0.000000 0.000000\n...)
  for (i = 0; i < strlen(str); i++)
  {
      if (str[i] == '\n' || count==1)
      {
         if(i==0)
         {
            leg_pos[index].x = atof(&str[i]);
         }
         else if(index<USERS)
         {
            leg_pos[index].x = atof(&str[i+1]);
         }
         count=2;
         continue;
      }
      if (str[i] == ' ' && count==2)
      {
         leg_pos[index].y = atof(&str[i+1]);
         count=1;
         index++;
         continue;
      }
  }  

  ros::Time time=ros::Time::now();		// save current time

  // generate and transform PoseStamped Message
  for (i = 0; i < index; i++)
  {
     // generate PoseStamped Message
     geometry_msgs::PoseStamped msg;

     msg.pose.position.x=leg_pos[i].x;
     msg.pose.position.y=leg_pos[i].y;
     msg.pose.position.z=0;
     msg.pose.orientation.w=1;

     msg.header.frame_id="kinect_link";
     msg.header.stamp.sec=time.sec;
     msg.header.stamp.nsec=time.nsec;

     // generate new transformed PoseStamped Message
     geometry_msgs::PoseStamped msg_trans;
     msg_trans.header=msg.header;
 
     #if TRANSFORM
     // Transform PoseStamped Message in map frame
     while (ros::ok() && !leglistener->waitForTransform("kinect_link", "base_link", ros::Time::now(), ros::Duration(5.0))) {
        ROS_ERROR("Leg-Transformation not ready yet");
        ros::Duration(1).sleep();
     }
     try{
      leglistener->transformPose("base_link", msg, msg_trans);
     }
     catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
     }
     #else
     msg_trans=msg;
     #endif

     // publish transformed PoseStamped Message
     leg_pose.publish(msg_trans);
     leg_pos[i].x=msg_trans.pose.position.x;
     leg_pos[i].y=msg_trans.pose.position.y;
     leg_pos[i].z=0;

     // show leg positions
     #if OUTPUT
     ROS_INFO("Legs %d: [x=%f y=%f]",i+1,leg_pos[i].x,leg_pos[i].y);
     #endif
  }

}

void PeopleDetector::attentionCallback(const std_msgs::String::ConstPtr& msg)
{
  const char* str = msg->data.c_str();	// define shortcut

  // define variables
  int i=0;
  int index=0;

  // initialize attention variable
  for (i=0;i<USERS;i++)
  {
  	attention_pos[i]=0;
  }

  // read out which faces give attention (1 3 5 \n)
  for (i = 0; i < strlen(str); i++)
  {
      if(i==0)
      {
         index = atol(&str[i]);
      }
      else if (str[i] == ' ')
      {
         index = atol(&str[i+1]);
      }
      if(index>=0 && index<USERS)
       	 attention_pos[atol(&str[i])] = 1;
  }  

  // listen to attention tracker (1...attention is given, 0...no attention)
  #if OUTPUT
  ROS_INFO("Attention: [%d %d %d %d %d]", attention_pos[0],attention_pos[1],attention_pos[2],attention_pos[3],attention_pos[4]);
  #endif
}

int PeopleDetector::getAttention(int index)
{
	return attention_pos[index];	// return private variable
}

void PeopleDetector::detectUsers(void)
{
   // initialize variable
   int i,j;
   
   // iterate through all leg- and face positions to find users, if the data is on top of each other
   for(i=0;i<USERS;i++)
   {
      for(j=0;j<USERS;j++)
      {
   	if(face_pos[i].x+range>leg_pos[j].x && face_pos[i].x-range<leg_pos[j].x && face_pos[i].y+range>leg_pos[j].y && face_pos[i].y+range>leg_pos[j].y && face_pos[i].x!=0 && face_pos[i].y!=0 && leg_pos[j].x!=0 && leg_pos[j].y!=0)
	{
   		users[i].x=leg_pos[j].x;
		users[i].y=leg_pos[j].y;
	}
      }
   }
}

int PeopleDetector::getUserSize()
{
   // initialize variable
    int size=0;

    // iterate through all users, if users are found
    for(int i=0;i<USERS;i++)
    {
	if(users[i].x!=0 && users[i].y!=0)
		size++;
    }
    return size;
}

void PeopleDetector::getClosestUsers(int *index_sorted)
{
    // initialize variables
    int i,j;
    int newn;
    int size=0;
   
    // length from users
    double length[USERS];

    // help variables for bubble sort
    double helplength;
    int helpindex;

    // size of users
    size=getUserSize();

    // calculate length and matching index
    for(i=0;i<size;i++)
    {
        length[i]=sqrt(users[i].x*users[i].x+users[i].y*users[i].y);
        *(index_sorted+i)=i;
    }

    // Bubble sort
    int n=size;
    do{
      newn = 1;
      for(i=0;i<n-1;i++)
      {
          if(length[i] > length[i+1])
	  {
		// swap data with help variable
          	helplength=length[i];
          	helpindex=*(index_sorted+i);

          	length[i]=length[i+1];
          	*(index_sorted+i)=*(index_sorted+i+1);

          	length[i+1]=helplength;
          	*(index_sorted+i+1)=helpindex;

          	newn = i+1;
          }
      }
      n = newn;
    }while (n > 1);
}

void PeopleDetector::setRange(double setrange)
{
   range=setrange;	// set private range
}

coordinates PeopleDetector::getUserCoordinates(int index)
{
   return users[index];   // return private coordinates from user
}


PeopleDetector::PeopleDetector()
{
  // initialize subscribers
  leg_sub = nh.subscribe("laser_person", 100, &PeopleDetector::legCallback,this);
  face_sub = nh.subscribe("face_pos", 100, &PeopleDetector::faceCallback,this);
  attention_sub = nh.subscribe("attentive_faces", 100, &PeopleDetector::attentionCallback,this);

  // initialize pose msg publisher
  leg_pose = nh.advertise<geometry_msgs::PoseStamped>("leg_pose", 10);
  attention_pose = nh.advertise<geometry_msgs::PoseStamped>("attention_pose", 10);
  face_pose = nh.advertise<geometry_msgs::PoseStamped>("face_pose", 10);

  // initialize listener
  leglistener = new tf::TransformListener();
  facelistener = new tf::TransformListener();

  // initialize variables
  coordinates init;
  init.x=0;
  init.y=0;
  init.z=0;
  for (int i=0;i<USERS;i++)
  {
  	face_pos[i]=init;
  	leg_pos[i]=init;
  	attention_pos[i]=0;

  	users[i]=init;
  }
}
