#ifndef EKFSENSOR_CPP
#define EKFSENSOR_CPP
#include "Sensors.h"

Sensors::Sensors(ros::NodeHandle n)
{
}

void Sensors::ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data)
{
	ROS_INFO_STREAM("Camera callback recieved");
	// ROS_INFO_STREAM("Camera callback recieved");
	// camcld = *data;
	// cout << it.end() << '\n';

	// double x, y, z;
	// for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); it++)
	// {
	// 	x = it[0];
	// 	y = it[1];
	// 	z = it[2];
	// }
}
void Sensors::ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data)
{
	ROS_INFO_STREAM("Lidar callback recieved");
}
#endif
