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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ios>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <squirrel_leg_detector/leg_detector.h> 

LegDetector::LegDetector()
{
  ppl2D_ = 0;
}

LegDetector::~LegDetector()
{
  delete ppl2D_;
}

void LegDetector::initialise(int argc, char **argv)
{
  sw_param_str sw_param;

  // load trained person model
  nh_.param<std::string>("model_file", sw_param.modelfile,"MODEL_FILE_NOT_SET");
  // segmentation distance, default 0.2m
  nh_.param("segmentation_distance", sw_param.dseg, 0.2);
  // feature set mix, default 0:all
  nh_.param("feature_mix", sw_param.featuremix, 0);

  ppl2D_ = new people2D_engine(sw_param);
  ppl2D_->set_featureset();
  if(!ppl2D_->load_adaboost_model(sw_param.modelfile))
  {
    ROS_ERROR("failed to open model file '%s'\n", sw_param.modelfile.c_str());
    exit(1);
  }
  laserSub_ = nh_.subscribe("/scan_filtered", 1, &LegDetector::laserCallback, this);
  markerPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  personPub_ = nh_.advertise<std_msgs::String>("laser_person", 0);
}

void LegDetector::run()
{
  ros::spin();
}

void LegDetector::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
  laserscan_data scan;
  std::vector<LSL_Point3D_container> people;
  scan.data.pts.resize(laserMsg->ranges.size());
  scan.timestamp = 0.;
  //FILE *dumpfile = fopen("laserdump.txt", "w");//
  //fprintf(dumpfile, "%.6f ", scan.timestamp);//
  for(size_t i = 0; i < laserMsg->ranges.size(); i++)
  {
    float r = std::max(std::min(laserMsg->ranges[i], laserMsg->range_max), laserMsg->range_min);
    if(!std::isfinite(r))
      r = laserMsg->range_max - 0.10*(float)random()/(float)RAND_MAX;
    scan.data.pts[i].x = r*cos(laserMsg->angle_min + (float)i*laserMsg->angle_increment);
    scan.data.pts[i].y = r*sin(laserMsg->angle_min + (float)i*laserMsg->angle_increment);
    scan.data.pts[i].label = 1;
    //fprintf(dumpfile, "%.6f %.6f %d ", scan.data.pts[i].x, scan.data.pts[i].y, scan.data.pts[i].label);//
  }
  //fclose(dumpfile);//
  //visualiseScan(laserMsg);  // HACK
  ppl2D_->detect_people(scan, people);

  /*if(people.size() > 0)
  {
    std_msgs::String pos;
    std::stringstream poss;
    LSL_Point3D_str center;
    people[0].compute_cog(&center);
    poss << center.x << " " << center.y;
    pos.data = poss.str();
    personPub_.publish(pos);
  }*/
  std::stringstream poss;//Tom
  for(size_t i = 0; i < people.size(); i++)
  {
  std_msgs::String pos;//Tom


    LSL_Point3D_str center;
    people[i].compute_cog(&center);
    //ROS_INFO("person at %.2f %.2f", center.x, center.y);

    poss << center.x << " " << center.y << "\n";//Tom
    if(i==people.size()-1)//Tom
    {
        pos.data = poss.str();//Tom

        personPub_.publish(pos);//Tom
    }
  }
  if(people.size() > 0)
    //ROS_INFO("--");

  for(size_t i = 0; i < people.size(); i++)
    visualisePerson(people[i]);
}

// HACK
void LegDetector::visualiseScan(const sensor_msgs::LaserScan::ConstPtr& laserMsg)
{
  for(size_t i = 0; i < laserMsg->ranges.size(); i++)
  {
    if(i%10 != 0)
      continue;

    float r = std::max(std::min(laserMsg->ranges[i], laserMsg->range_max), laserMsg->range_min);
    float x = r*cos(laserMsg->angle_min + (float)i*laserMsg->angle_increment);
    float y = r*sin(laserMsg->angle_min + (float)i*laserMsg->angle_increment);

	  visualization_msgs::Marker marker;
	  marker.header.frame_id = "hokuyo_link";
	  marker.header.stamp = ros::Time();
	  marker.ns = "laserdot";
	  marker.id = i;
	  marker.lifetime = ros::Duration(1.);
	  marker.type = visualization_msgs::Marker::CYLINDER;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.pose.position.x = x;
	  marker.pose.position.y = y;
	  marker.pose.position.z = 0.;
	  marker.pose.orientation.x = 0.0;
	  marker.pose.orientation.y = 0.0;
	  marker.pose.orientation.z = 0.0;
	  marker.pose.orientation.w = 1.0;
	  marker.scale.x = 0.04;
	  marker.scale.y = 0.04;
	  marker.scale.z = 0.15;
	  marker.color.r = 0.0;
	  marker.color.g = 1.0;
	  marker.color.b = 0.0;
	  marker.color.a = 0.5; // Don't forget to set the alpha!
	  markerPub_.publish(marker);
  }
}

void LegDetector::visualisePerson(LSL_Point3D_container &person)
{
  static int cnt = 0;

  LSL_Point3D_str center;
  person.compute_cog(&center); 

  visualization_msgs::Marker marker;
  marker.header.frame_id = "hokuyo_link";
  marker.header.stamp = ros::Time();
  marker.ns = "person";
  marker.id = cnt;
  marker.lifetime = ros::Duration(1.);
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center.x;
  marker.pose.position.y = center.y;
  marker.pose.position.z = 0.;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 1.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  markerPub_.publish(marker);
  
  cnt++;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "squirrel_leg_detector_node");
  LegDetector ld;
  ld.initialise(argc, argv);
  ld.run();
  exit(0);
}
