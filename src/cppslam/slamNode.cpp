#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "ekfslam/ekf.h"

int main (int argc, char **argv)
{
	ros::init(argc, argv, "slamNode");
	ros::NodeHandle n; 
	ROS_INFO_STREAM("Slam node started");
	
}

