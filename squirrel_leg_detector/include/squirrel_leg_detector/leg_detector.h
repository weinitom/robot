/**
 * leg_detector.h
 *
 * Detects persons as pairs of legs in 2D laser range data.
 *
 * This wraps the People2D code by Luciano Spinello
 * http://www2.informatik.uni-freiburg.de/~spinello/people2D.html
 * implementing the method described in
 *
 * Arras K.O., Mozos O.M., Burgard W., Using Boosted Features for the Detection
 * of People in 2D Range Data, IEEE International Conference on Robotics and
 * Automation (ICRA), 2007
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef LEG_DETECTOR_H
#define LEG_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <squirrel_leg_detector/people2D_engine.hpp>

class LegDetector
{
public:
  LegDetector();
  virtual ~LegDetector();
  void initialise(int argc, char **argv);
  void run();

private:
  people2D_engine *ppl2D_;
  ros::NodeHandle nh_;
  ros::Subscriber laserSub_;
  ros::Publisher personPub_;
  ros::Publisher markerPub_;

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserMsg);
  void visualisePerson(LSL_Point3D_container &person);
  void visualiseScan(const sensor_msgs::LaserScan::ConstPtr& laserMsg);
};

#endif
