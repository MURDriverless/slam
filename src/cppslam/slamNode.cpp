#include "ros/ros.h"
#include <sstream>
#include "ekfslam/ekfslam.h"
#include <Eigen/Dense>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slamNode");

	ros::NodeHandle n;

	ekfslam slam(n);

	ROS_INFO_STREAM("EKF SLAM: LAUNCHED");

	slam.runnable();

	return 0;
}
