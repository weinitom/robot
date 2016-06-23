src folder for ros project

ROS Project to mitigate malfunctions with the help of the user


TO RUN THE APPLICATION ON ROBOTINO:

in attention_tracking.launch - change the parameter "mode" from camera to rosbag
			     - comment out the node webcam_source
in pkg attention tracker CMakeLists.txt - change the path of dlib where dlib is located on the robot
