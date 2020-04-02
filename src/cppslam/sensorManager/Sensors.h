#ifndef EKFSENSOR_H
#define EKFSENSOR_H

#include "../matrix/matrix.h"
#include <string>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"

struct pointcloud
{
	std::string msg_source;
	int size;
	double **data;
} pointcloud;
class Sensors
{
public:
	Sensors(ros::NodeHandle n); // constructor type

	void publishMap(Matrix map);
	void publishOdom(Matrix state);

	void ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data);
	void ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data);

private:
	//node handles & publishers & subscribers
	ros::NodeHandle nh;
	ros::Publisher ptcldPub;
	ros::Publisher odomPub;
	ros::Subscriber lidarcld;
	ros::Subscriber camcld;

	// callbacks

	//Ptcloud messages
};
#endif
