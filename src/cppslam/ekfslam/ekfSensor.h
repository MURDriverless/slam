#ifndef EKFSENSOR_H
#define EKFSENSOR_H

#include "../matrix/matrix.h"
#include <string>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
class ekfSensor
{
public:
	ekfSensor()
	{
		return;
	}
	ekfSensor(ros::NodeHandle n); // constructor type

	void publishMap(Matrix map);
	void publishOdom(Matrix state);

private:
	//node handles & publishers & subscribers
	ros::NodeHandle nh;
	ros::Publisher ptcldPub;
	ros::Publisher odomPub;
	ros::Subscriber lidarcld;
	ros::Subscriber camcld;

	// callbacks
	void ptcloudclb(const sensor_msgs::PointCloud2ConstPtr &data);

	//static message topic names
	std::string CAM_TOPIC = "/camera/cloud";
	std::string LIDAR_TOPIC = "/lidar/cloud";
	std::string FILTERED_TOPIC = "/slam/map";
	std::string SLAM_POSE_TOPIC = "/slam/odom";
};
#endif
