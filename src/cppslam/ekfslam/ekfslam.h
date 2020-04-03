#ifndef EKF_H
#define EFK_H
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <Eigen/Dense>

class ekfslam
{

public:
	ekfslam(ros::NodeHandle n);
	void runnable();

private:
	void ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data);
	void ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data);
	int stateSizeCalc(Eigen::MatrixXd z, Eigen::MatrixXd x);
	ros::Subscriber camCld;
	ros::Subscriber lidarCld;
	int initialiseSubs();
	Eigen::Matrix2d motionModel(Eigen::Matrix2d x, Eigen::Matrix2d u);
	int stateSize;
	int controlSize;
	ros::NodeHandle nh;
	double dt;

	//static message topic names
	std::string CAM_TOPIC = "/camera/cones";
	std::string LIDAR_TOPIC = "/lidar/cones";
	std::string FILTERED_TOPIC = "/slam/map";
	std::string SLAM_POSE_TOPIC = "/slam/odom";
	static const int que_size = 1;
};
#endif
