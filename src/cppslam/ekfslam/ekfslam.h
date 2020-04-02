#ifndef EKF_H
#define EFK_H
#include "../matrix/matrix.h"
#include "../matrix/matrixUtils.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>

class ekfslam
{

public:
	ekfslam(ros::NodeHandle n);
	void runnable();

private:
	void ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data);
	void ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data);

	ros::Subscriber camCld;
	ros::Subscriber lidarCld;
	int initialiseSubs();
	Matrix motionModel(Matrix &x, Matrix &u);
	int stateSize;
	int controlSize;
	ros::NodeHandle nh;
	double dt;

	//static message topic names
	std::string CAM_TOPIC = "/camera/cones";
	std::string LIDAR_TOPIC = "/lidar/cones";
	std::string FILTERED_TOPIC = "/slam/map";
	std::string SLAM_POSE_TOPIC = "/slam/odom";
	static const int que = 1;
};
#endif
