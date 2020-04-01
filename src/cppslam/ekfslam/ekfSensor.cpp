#ifndef EKFSENSOR_CPP
#define EKFSENSOR_CPP

#include "./ekfSensor.h "

ekfSensor::ekfSensor(ros::NodeHandle n)
	: nh = n; //assign nodehandle to class

{
	ptcldPub = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_TOPICI, 1);
	odomPub = nh.advertise<nav_msgs::Odometry>(SLAM_POSE_TOPIC, 1) return;
}
void ekfSensor::ptcloudclb(const sensor_msgs::PointCloud2CloudConstPtr &data)
{
	ROS_INFO_STREAM("callback recieved");
}

#endif
