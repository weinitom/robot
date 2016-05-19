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
	    // sorted users and indizes are stored, decide if attention is given and then go to nomal behaviour or to next closest user
       }
}

Algorithm::Algorithm()
{
    // initialize publisher and subscribers
    command_sub = nh.subscribe("command", 100, &Algorithm::commandCallback,this);

    goal_pub = nh.advertise<std_msgs::String>("goal", 1);
    reachedgoal_sub = nh.subscribe("goal_reached", 1, &Algorithm::reachedGoalCallback,this);

    for(int i=0;i<USERS;i++)
	index_sorted[i]=-1;
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

void Algorithm::Error(PeopleDetector people, double timeerror, double timethreshold, coordinates pointA, coordinates pointB)
{
   // initialize variables
   std_msgs::String msg;
   std::stringstream ss;
   coordinates current;
   int size=0;

   people.detectUsers();	// detect users
   size=people.getUserSize();   // get size of users

   // check if users are available 
   if(size!=0 && (statusbefore=='a' || statusbefore=='b'))
   {
      if(timeerror-timelastcmd<timethreshold)	// check if time since last command is lower than threshold
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
        people.getClosestUsers(&index_sorted[0]);	// write sorted indizes from users
       	// get User Coordinates
  	for(int i=0;i<size;i++)
  	{
		users_sorted[i]=people.getUserCoordinates(index_sorted[i]);
   	}

   	// choose closest user
   	choosen_user=0;
   	current=people.getUserCoordinates(index_sorted[choosen_user]);
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

  // define time threshold and time when error occur
  double timethreshold = 10;
  double timeerror = 0;

  // define range for leg - and face data to compare to find users
  people.setRange(0.2);

  while(ros::ok())
  {
     // decide between normal mode and error mode
     if(alg.returnStatus()=='e')
     {
         if(timeerror==0)
	 {
         	timeerror = ros::Time::now().toSec();			// save time at error
         	alg.Error(people,timeerror,timethreshold,pointA,pointB);// decide which user is the best to help
	 }
     }
     else
     {   timeerror = 0;
         alg.Normal(home,pointA,pointB);
     }

     // listen to callbacks
     ros::spinOnce();
  }

  return 0;
}
