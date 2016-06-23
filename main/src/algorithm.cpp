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

#define SEARCH_USER 30

void Algorithm::commandCallback(const std_msgs::String::ConstPtr& msg)
{
  // to send a command over topic "command":

  // starting the whole process
  // rostopic pub command std_msgs/String "start"

  // mitigate malfunction
  // rostopic pub command std_msgs/String "error"

  // solve malfunction
  // rostopic pub command std_msgs/String "yes"

  const char* str = msg->data.c_str();  // define shortcut

  // read command
  // s for normal behaviour (h->A->h->B->h)
  // e for error algorithm (go to last waypoint or closest user)
  // y if help was given (go to home station and back in normal mode)
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
    case 'y':
    case 'Y':
       ROS_INFO("Failure solved");
       status='h';
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
       int attention = 1;
       std_msgs::String text;
       std::stringstream ss;
       coordinates current;

       if(status=='a' || status=='b')  // if pointA or pointB is reached -> drive to home
       {
          statusbefore=status;	// save last waypoint
          status='h';		// set new destination
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
	        // define timeout to wait for attention
		ros::Time start_time = ros::Time::now();
   		ros::Duration timeout(SEARCH_USER);

		// wait for attention until timeout is over or a user gives attention
        	while(attention==0 && ros::Time::now() - start_time < timeout && ros::ok())
		{
			// search attention from all users
			for(int i=0;i<USERS;i++)
	  	  	{
				attention=people->getAttention(i);
            		}
		}
 
		// if attention is give, ask for help
            	if(attention==1)
	    	{
			ROS_INFO("Please, can you help me!");
            	}
	    	else	//if not, choose next closest user to drive to
	    	{
			choosen_user++;
			if(choosen_user < people->getUserSize())  // if there are other users
			{
				current=people->getUserCoordinates(index_sorted[choosen_user]);
			}
			else	// if no more users are found, go to home station and quit the programm
			{
				current=home;
				status='q';
			}

 	        	// publish coordinates
               	 	ss << current.x << " " << current.y;
   	        	text.data = ss.str();

                	goal_pub.publish(text);
            	}
       	}
}

Algorithm::Algorithm()
{
    // initialize publisher and subscribers
    command_sub = nh.subscribe("command", 100, &Algorithm::commandCallback,this);

    goal_pub = nh.advertise<std_msgs::String>("goal", 1);
    reachedgoal_sub = nh.subscribe("goal_reached", 1, &Algorithm::reachedGoalCallback,this);

    // initialize class PeopleDetector in private variable
    people = new PeopleDetector();

    // initialize array index_sorted
    for(int i=0;i<USERS;i++)
	index_sorted[i]=-1;
}

void Algorithm::Normal()
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

void Algorithm::Error(double timeerror, double timethreshold)
{
   // initialize variables
   std_msgs::String msg;
   std::stringstream ss;
   coordinates current;
   int size=0;

   // define timeout to search for users
   ros::Time start_time = ros::Time::now();
   ros::Duration timeout(SEARCH_USER);

   // search for users until timeout is over or user is found
   while(size==0 && ros::Time::now() - start_time < timeout && ros::ok())
   {
   	people->detectUsers();	        // detect users
   	size=people->getUserSize();     // get size of users
        ros::spinOnce();		// listen to callbacks
   }

   if(timeerror-timelastcmd<timethreshold && (statusbefore=='a' || statusbefore=='b'))   // check if time since last command is lower than threshold
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
	 ROS_INFO("Drive to last waypoint");
   }
   else if(size>0)
   {
        people->getClosestUsers(&index_sorted[0]);	// write sorted indizes from users
       	// get User Coordinates
  	for(int i=0;i<size;i++)
  	{
		users_sorted[i]=people->getUserCoordinates(index_sorted[i]);
   	}

   	// choose closest user
   	choosen_user=0;
   	current=people->getUserCoordinates(index_sorted[choosen_user]);

	ROS_INFO("Drive to closest user");
   }
   else
   {
        // drive to home station and quit programm, if no user is found
	current=home;
	status='q';
	ROS_ERROR("No User found, drive to home station");
   }

   // publish coordinates
   ss << current.x << " " << current.y;
   msg.data = ss.str();

   goal_pub.publish(msg);
}

void Algorithm::setRange(double range)
{
   people->setRange(range);	// set private varible range
}

void Algorithm::setPoints(coordinates sethome, coordinates setPointA, coordinates setPointB)
{
   // set waypoints to private variables
   home=sethome;
   pointA=setPointA;
   pointB=setPointB;
}

int main(int argc, char **argv)
{
  // initialize ROS
  ros::init(argc, argv, "algorithm");

  // generate objects
  Algorithm alg;

  // define waypoints and set in private variables
  coordinates home,pointA,pointB;

  home.x=0;
  home.y=0;

  pointA.x=10;
  pointA.y=10;

  pointB.x=-10;
  pointB.y=-10;

  alg.setPoints(home,pointA,pointB);

  // define time threshold and time when error occur
  double timethreshold = 20;
  double timeerror = 0;

  // define range for leg - and face data to compare to find users
  alg.setRange(0.3);

  while(ros::ok())
  {
     // decide between normal mode and error mode, if no user is helps, quit (q) the programm
     if(alg.returnStatus()=='e')
     {
         if(timeerror==0)						// execute just one time
	 {
         	timeerror = ros::Time::now().toSec();			// save time at error
         	alg.Error(timeerror,timethreshold);			// decide which user is the best to help in error mode
	 }
     }
     else if(alg.returnStatus()=='q')
     {
	return 0;
     }
     else
     {   timeerror = 0;		// reset time when error appeared
         alg.Normal();		// normal mode
     }

     // listen to callbacks
     ros::spinOnce();
  }

  return 0;
}
