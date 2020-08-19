#include "ros/ros.h"
#include <sstream>
#include "ekfslam/ekfslam.h"
#include <Eigen/Dense>

#define STATE_SIZE 5
#define HZ 30

int main(int argc, char **argv)
{
/* High level manager for the slam node. 
Will call a class of slam that has been developed in subfolders to this directory. 
Currently only has ekf slam.	
 */
	ros::init(argc, argv, "slamNode");

	ros::NodeHandle n;

	ekfslam slam(n,STATE_SIZE, HZ);

	ROS_INFO_STREAM("EKF SLAM: LAUNCHED");

	slam.runnable();
	return 0;
}
