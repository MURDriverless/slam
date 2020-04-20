#ifndef EKF_H
#define EFK_H
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include <tuple>
  
class ekfslam
{
public:
	ekfslam(ros::NodeHandle n);
	void runnable();

private:
	void ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data);
	void ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data);
	void controlclb(std_msgs::String msg);

	ros::Subscriber camCld;
	ros::Subscriber lidarCld;
	ros::Subscriber control; 

	int stateSizeCalc(Eigen::MatrixXd z, Eigen::MatrixXd x);
	int initialiseSubs();
	void motionModel();
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
