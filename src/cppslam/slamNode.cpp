#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "ekfslam/ekf.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"slamNode");

	ros::NodeHandle n; 

	ROS_INFO_STREAM("MUR SLAM: LAUNCHED");


	ekf slam(n); 

	Matrix X0(4,1);
	X0.Fill(0.0);
	slam.runnable(10, X0);	
}
