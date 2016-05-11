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
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <sstream>

#include <main/algorithm.hpp>
#include <main/people_detection.hpp>

void Algorithm::commandCallback(const std_msgs::String::ConstPtr& msg)
{
  // to send a command over topic "command":

  // starting the whole process
  // rostopic pub command std_msgs/String "start"

  // mitigate malfunction
  // rostopic pub command std_msgs/String "error"

  const char* str = msg->data.c_str();  // define shortcut

  // read command
  // s for normal behaviour (h->A->h->B->h)
  // e for error algorithm (get to last waypoint or closest user)
  switch(str[0])
  {
    case 's':
    case 'S':
       ROS_INFO("Starting normal movement");
       status='a';
       break;
    case 'e':
    case 'E':
       ROS_ERROR("Starting failure procedure");
       status='e';
       break;
    default:
       ROS_ERROR("no valid command");
  }

}

void Algorithm::reachedGoalCallback(const std_msgs::Int32::ConstPtr& msg)
{
       // to confirm that the goal is reached use topic "goal_reached"
       // rostopic pub goal_reached std_msgs/Int32 1

       timelastcmd = ros::Time::now().toSec();	// save time at last command

       if(status=='a' || status=='b')  // if pointA or pointB is reached -> drive to home
       {
          statusbefore=status;	// save last waypoint
          status='h';
       } 
       else if(status=='h' && statusbefore=='a')  // if home is reached and robot was at pointA before -> pointB
       {
          status='b';
       }
       else if(status=='h' && statusbefore=='b')  //if home is reached and robot was at pointB before -> pointA
       {
          status='a';
       }
       else if(status=='e')  // if robot reaches user to mitigate malfunction
       {

       }
}

Algorithm::Algorithm()
{
    // initialize publisher and subscribers
    command_sub = nh.subscribe("command", 100, &Algorithm::commandCallback,this);

    goal_pub = nh.advertise<std_msgs::String>("goal", 1);
    reachedgoal_sub = nh.subscribe("goal_reached", 1, &Algorithm::reachedGoalCallback,this);
}

void Algorithm::Normal(coordinates home, coordinates pointA, coordinates pointB)
{
    // define variables
    std_msgs::String msg;
    std::stringstream ss;
    coordinates current;

    // write coordinates to drive to
    switch(status)
    {
       case 'a':
          current=pointA;
          break;
       case 'b':
          current=pointB;
          break;
       case 'h':
          current=home;
          break;
       default:
          return;  
    }

    // publish coordinates
    ss << current.x << " " << current.y;
    msg.data = ss.str();

    goal_pub.publish(msg);
}

char Algorithm::returnStatus(void)
{
   return status;	   // return private variable status
}

void Algorithm::Error(PeopleDetector people, double timethreshold, coordinates pointA, coordinates pointB)
{
   // initialize variables
   std_msgs::String msg;
   std::stringstream ss;
   coordinates current;

   people.detectUsers();	// detect users

   double timenow = ros::Time::now().toSec();	// save time at last command

   // check if users are available 
   if((people.getUserCoordinates(0).x!=0 || people.getUserCoordinates(0).y!=0 || people.getUserCoordinates(0).z!=0) && (statusbefore=='a' || statusbefore=='b'))
   {
      if(timenow-timelastcmd<timethreshold)	// check if time since last command is lower than threshold
      {
         // write coordinates from last waypoint to drive to
         switch(statusbefore)
         {
            case 'a':
              current=pointA;
              break;
       	    case 'b':
              current=pointB;
              break;
            default:
              ROS_ERROR("last commanding User not found");
              return;  
         }
      }
      else
      {
         current=people.getClosestUser();	// write coordinates from closest user
      }
   }

   // publish coordinates
   ss << current.x << " " << current.y;
   msg.data = ss.str();

   goal_pub.publish(msg);
}

int main(int argc, char **argv)
{
  // initialize ROS
  ros::init(argc, argv, "algorithm");

  // generate objects
  Algorithm alg;
  PeopleDetector people;

  // define waypoints
  coordinates home,pointA,pointB;

  home.x=0;
  home.y=0;

  pointA.x=10;
  pointA.y=10;

  pointB.x=-10;
  pointB.y=-10;

  // define time threshold in seconds
  double timethreshold = 10;

  //ros::Rate loop_rate(1);
  //loop_rate.sleep();

  while(ros::ok())
  {
     // decide between normal mode and error mode
     if(alg.returnStatus()=='e')
     {
         alg.Error(people,timethreshold,pointA,pointB);
     }
     else
     {  
         alg.Normal(home,pointA,pointB);
     }

     // listen to callbacks
     ros::spinOnce();
  }

  return 0;
}
