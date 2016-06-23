#include <string> // chris
#include <std_msgs/String.h> // chris

#include "ros_head_pose_estimator.hpp"

using namespace std;
using namespace cv;

// how many second in the *future* the face transformation should be published?
// this allow to compensate for the 'slowness' of face detection, but introduce
// some lag in TF.
#define TRANSFORM_FUTURE_DATING 0

HeadPoseEstimator::HeadPoseEstimator(ros::NodeHandle& rosNode,
                                     const string& modelFilename, const string& mode) :
            rosNode(rosNode),
            it(rosNode),
            warnUncalibratedImage(true),
            estimator(modelFilename)

{
    if(mode == "camera") // chris
        sub = it.subscribeCamera("image", 1, &HeadPoseEstimator::detectFaces, this);
    else if(mode == "rosbag")
        sub = it.subscribeCamera("/kinect/rgb/image_raw", 1, &HeadPoseEstimator::detectFaces, this);
    else
        ROS_WARN("NO MODE SET -> NO IMAGE DATA");    

    nb_detected_faces_pub = rosNode.advertise<std_msgs::Char>("nb_detected_faces", 1);
    face_coordinates_pub = rosNode.advertise<std_msgs::String>("face_pos", 1);

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    pub = it.advertise("attention_tracker/faces/image",1);
#endif
}

void HeadPoseEstimator::detectFaces(const sensor_msgs::ImageConstPtr& msg, 
                                    const sensor_msgs::CameraInfoConstPtr& camerainfo)
{
    // updating the camera model is cheap if not modified
    cameramodel.fromCameraInfo(camerainfo);
    // publishing uncalibrated images? -> return (according to CameraInfo message documentation,
    // K[0] == 0.0 <=> uncalibrated).
    if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
        if(warnUncalibratedImage) {
            warnUncalibratedImage = false;
            ROS_ERROR("Camera publishes uncalibrated images. Can not estimate face position.");
            ROS_WARN("Detection will start over again when camera info is available.");
        }
        return;
    }
    warnUncalibratedImage = true;
    
    estimator.focalLength = cameramodel.fx(); 
    estimator.opticalCenterX = cameramodel.cx();
    estimator.opticalCenterY = cameramodel.cy();

    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    inputImage = cv_bridge::toCvShare(msg)->image; 

    /********************************************************************
    *                      Faces detection                              *
    ********************************************************************/

    estimator.update(inputImage);

    auto poses = estimator.poses();
    //ROS_INFO_STREAM(poses.size() << " faces detected.");
    nb_detected_faces_pub.publish(poses.size());

    string s; // chris

    for(size_t face_idx = 0; face_idx < poses.size(); ++face_idx) {

	auto coords = estimator.pose(face_idx); // chris
	//s += to_string(double(coords(0,3))) + " " + to_string(double(coords(1,3))) + " " + to_string(double(coords(2,3))) + "\n"; // chris
	s += to_string(-double(coords(2,3))) + " " + to_string(double(coords(1,3))) + " " + to_string(double(coords(0,3))) + "\n"; // chris

        auto trans = poses[face_idx];

        tf::Transform face_pose;
        
        // Frame orientation of the camera follows the classical camera
        // convention (Z forward)

        auto z = -trans(2,3);

        if (z < 0) continue; // the head can not be *behind* the camera!

        face_pose.setOrigin( tf::Vector3( -trans(0,3),
                                          -trans(1,3),
                                          z) );

        tf::Quaternion qrot;
        tf::Matrix3x3 mrot(
                trans(0,0), -trans(0,1), -trans(0,2),
                trans(1,0), -trans(1,1), -trans(1,2),
                trans(2,0), -trans(2,1), -trans(2,2)); // chris // - turns the coordinates by 180° about the x-axis
        mrot.getRotation(qrot);
        face_pose.setRotation(qrot);

        br.sendTransform(
                tf::StampedTransform(face_pose, 
                                     ros::Time::now() + ros::Duration(TRANSFORM_FUTURE_DATING), 
                                     cameramodel.tfFrame(),
                                     "face_" + to_string(face_idx)));

    }

    face_coordinates_pub.publish(s); // chris

#ifdef HEAD_POSE_ESTIMATION_DEBUG
    if(pub.getNumSubscribers() > 0) {
        ROS_INFO_ONCE("Starting to publish face tracking output for debug");
        auto debugmsg = cv_bridge::CvImage(msg->header, "bgr8", estimator._debug).toImageMsg();
        pub.publish(debugmsg);
    }
#endif
}

